/**
 *************************************************
 * @file p7-crypto.c
 * @brief P7 Analogic to Digital Converter driver
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * @author Karl Leplat <karl.leplat@parrot.com>
 * @date 2013-09-19
 ************************************************
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include "p7-crypto_regs.h"

#include <linux/sched.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/crypto.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <crypto/scatterwalk.h>
#include <crypto/aes.h>
#include <crypto/des.h>

#define IDLE_TIMEOUT		200 /* msec */

#define ACRYPTO_OP_DECODE	0
#define ACRYPTO_OP_ENCODE	1

#define ACRYPTO_MODE_ECB	0
#define ACRYPTO_MODE_CBC	1
#define ACRYPTO_MODE_OFB	2
#define ACRYPTO_MODE_CTR	3

#define ACRYPTO_TYPE_AES_128	0
#define ACRYPTO_TYPE_AES_192	1
#define ACRYPTO_TYPE_AES_256	2
#define ACRYPTO_TYPE_3DES	3
#define ACRYPTO_TYPE_DES	4

#define P7CA_NR_KEYSLOTS	8
struct p7ca_key_slot {
	struct list_head node;
	int slot_num;
};

#define P7CA_NR_DESCSLOTS	16
struct p7ca_desc_slot {
	int				slot_num;
	size_t				total;
	struct scatterlist		*in_sg;
	size_t				in_offset;
	struct scatterlist		*out_sg;
	size_t				out_offset;

	size_t				buflen;
	void				*buf_in;
	size_t				dma_size;
	dma_addr_t			dma_addr_in;
	void				*buf_out;
	dma_addr_t			dma_addr_out;
	struct ablkcipher_request	*req;
	unsigned long			flags;
};

struct p7ca_ctx {
	struct p7ca_dev		*dd;
	struct p7ca_key_slot	*slot;
	int			keylen;
	u8			key[AES_KEYSIZE_256];
	unsigned long		flags;
};

struct p7ca_reqctx {
	struct p7ca_desc_slot		*slot;
	u8				*iv;
	unsigned int			ivsize;
	u8				op, type, mode, unused;
	struct ablkcipher_walk		walk;
	int				count;
};

#define FLAGS_CMD_FAST		BIT(0)
#define FLAGS_SUSPENDED		BIT(1)
#define FLAGS_NEW_KEY		BIT(2)
#define FLAGS_DESC_UNUSED       BIT(3)
#define FLAGS_DESC_USED		BIT(4)

#define P7CA_QUEUE_LENGTH	50
#define P7CA_CACHE_SIZE	        0

struct p7ca_dev {
	unsigned long			phys_base;
	void __iomem			*io_base;
	struct clk			*iclk;
	struct device			*dev;
	unsigned long			flags;
	int				err;
	spinlock_t			lock;
	struct crypto_queue		queue;
	struct work_struct		done_task;
	struct work_struct		queue_task;
	struct p7ca_key_slot		*key_slots;
	struct p7ca_desc_slot		*desc_slots;
	int				cur_desc;
	int				irq;
	unsigned long			irq_flags;
};

static struct p7ca_dev *p7ca_dev;

/* keep registered devices data here */
static struct list_head dev_list;
static DEFINE_SPINLOCK(list_lock);

static inline u32 p7ca_read(struct p7ca_dev *dd, u32 offset)
{
	return __raw_readl(dd->io_base + offset);
}

static inline void p7ca_write(struct p7ca_dev *dd, u32 offset,
		u32 value)
{
	__raw_writel(value, dd->io_base + offset);
}

static inline void p7ca_write_mask(struct p7ca_dev *dd, u32 offset,
		u32 value, u32 mask)
{
	u32 val;

	val = p7ca_read(dd, offset);
	val &= ~mask;
	val |= value;
	p7ca_write(dd, offset, val);
}

static void p7ca_write_n(struct p7ca_dev *dd, u32 offset,
		u32 *value, int count)
{
	for (; count--; value++, offset += 4)
		p7ca_write(dd, offset, *value);
}

static int p7ca_write_ctrl(struct p7ca_dev *dd, struct p7ca_desc_slot *slot)
{
	unsigned long end, flags;
	struct p7ca_ctx *ctx = crypto_tfm_ctx(slot->req->base.tfm);
	struct p7ca_reqctx *rctx = ablkcipher_request_ctx(slot->req);
	u32 val = 0;

	if (!ctx->slot)
		return -EINVAL;

	val |= P7CA_DESC_CTRL_INT_ENABLE;

	if (rctx->op == ACRYPTO_OP_DECODE)
		val |= P7CA_DESC_CTRL_DECODE;

	switch (rctx->mode) {
	case ACRYPTO_MODE_ECB:
		val |= P7CA_DESC_CTRL_CIPH_MODE_ECB;
		break;
	case ACRYPTO_MODE_CBC:
		val |= P7CA_DESC_CTRL_CIPH_MODE_CBC;
		break;
	case ACRYPTO_MODE_OFB:
		val |= P7CA_DESC_CTRL_CIPH_MODE_OFB;
		break;
	case ACRYPTO_MODE_CTR:
		val |= P7CA_DESC_CTRL_CIPH_MODE_CTR;
		break;
	default:
		goto err_out;
	}

	switch (rctx->type) {
	case ACRYPTO_TYPE_AES_128:
		if (ctx->keylen != AES_KEYSIZE_128)
			goto err_out;
		val |= P7CA_DESC_CTRL_CIPH_ALG_AES_128;
		break;
	case ACRYPTO_TYPE_AES_192:
		if (ctx->keylen != AES_KEYSIZE_192)
			goto err_out;
		val |= P7CA_DESC_CTRL_CIPH_ALG_AES_192;
		break;
	case ACRYPTO_TYPE_AES_256:
		if (ctx->keylen != AES_KEYSIZE_256)
			goto err_out;
		val |= P7CA_DESC_CTRL_CIPH_ALG_AES_256;
		break;
	case ACRYPTO_TYPE_3DES:
		if (ctx->keylen != DES3_EDE_KEY_SIZE)
			goto err_out;
		val |= P7CA_DESC_CTRL_CIPH_ALG_TDEA;
		break;
	case ACRYPTO_TYPE_DES:
		if (ctx->keylen != DES_KEY_SIZE)
			goto err_out;
		val |= P7CA_DESC_CTRL_CIPH_ALG_DES;
		break;
	default:
		goto err_out;
	}

	if (ctx->flags & FLAGS_NEW_KEY) {
		/* Workaround to fix a IP Crypto bug:
		 * If you try to configure new descriptors
		 * when one or more descriptors are active,
		 * dma result may be corrupted */
		end = msecs_to_jiffies(IDLE_TIMEOUT) + jiffies;
		while (((p7ca_read(dd, P7CA_REG_CTRL_STATUS)
			& CRYPTO_CTRL_AND_STATUS_NB_OF_ACTIVE_DESC_MASK) != 0)) {
			if (time_after_eq(jiffies, end)) {
				dev_err(dd->dev, "Idle ip crypto request timed out\n");
				goto err_out;
			}
			cond_resched();
		}

		/* copy the key to the key slot */
		p7ca_write_n(dd, P7CA_REG_KEY(ctx->slot->slot_num),
				(u32 *)ctx->key,
				ctx->keylen / sizeof(u32));

		val |= P7CA_DESC_CTRL_NEW_KEY_SET |
			P7CA_DESC_CTRL_KEY_INDEX(ctx->slot->slot_num);
		ctx->flags &= ~FLAGS_NEW_KEY;
	}

	if (((rctx->mode == ACRYPTO_MODE_CBC)
				|| (rctx->mode == ACRYPTO_MODE_OFB)
				|| (rctx->mode == ACRYPTO_MODE_CTR))
			&& rctx->iv) {
		/* Workaround to fix a IP Crypto bug:
		 * If you try to configure new descriptors
		 * when one or more descriptors are active,
		 * dma result may be corrupted */
		end = msecs_to_jiffies(IDLE_TIMEOUT) + jiffies;
		while (((p7ca_read(dd, P7CA_REG_CTRL_STATUS)
			& CRYPTO_CTRL_AND_STATUS_NB_OF_ACTIVE_DESC_MASK) != 0)) {
			if (time_after_eq(jiffies, end)) {
				dev_err(dd->dev, "Idle ip crypto request timed out\n");
				goto err_out;
			}
			cond_resched();
		}

		/* copy the IV to the iv slot */
		p7ca_write_n(dd, P7CA_REG_IV(ctx->slot->slot_num), (u32 *)rctx->iv,
				rctx->ivsize / sizeof(u32));

		/* Needed to reload initial vector */
		val |= P7CA_DESC_CTRL_NEW_KEY_SET |
			P7CA_DESC_CTRL_KEY_INDEX(ctx->slot->slot_num);
	}

	spin_lock_irqsave(&dd->lock, flags);
	if (++dd->cur_desc >= P7CA_NR_DESCSLOTS)
		dd->cur_desc = 0;
	slot->slot_num = dd->cur_desc;
	spin_unlock_irqrestore(&dd->lock, flags);

	p7ca_write(dd, P7CA_REG_DESC_CTRL(slot->slot_num), val);

	return 0;

err_out:
	return -EINVAL;
}

static void p7ca_release_key_slot(struct p7ca_key_slot *slot)
{
	spin_lock(&list_lock);
	list_add_tail(&slot->node, &dev_list);
	spin_unlock(&list_lock);
}

static struct p7ca_key_slot *p7ca_find_key_slot(void)
{
	struct p7ca_key_slot *slot = NULL;
	int empty;

	spin_lock(&list_lock);
	empty = list_empty(&dev_list);
	if (!empty) {
		slot = list_first_entry(&dev_list, struct p7ca_key_slot, node);
		list_del(&slot->node);
	}
	spin_unlock(&list_lock);

	return slot;
}

static void p7ca_dma_cleanup(struct p7ca_dev *dd, struct p7ca_desc_slot *slot)
{
	if (slot->buf_in != NULL)
		dma_free_coherent(dd->dev, PAGE_SIZE << P7CA_CACHE_SIZE,
				slot->buf_in, slot->dma_addr_in);
	if (slot->buf_out != NULL)
		dma_free_coherent(dd->dev,  PAGE_SIZE << P7CA_CACHE_SIZE,
				slot->buf_out, slot->dma_addr_out);
}

static irqreturn_t p7ca_interrupt(int irq, void *dev_id)
{
	unsigned long reg, flags;
	struct p7ca_dev *dd = dev_id;

	spin_lock_irqsave(&dd->lock, flags);

	p7ca_write(dd,
		P7CA_REG_IRQ_MEM,
		P7CA_IRQ_MEM_CLEAR);

	/* Read status and ACK all interrupts */
	reg = p7ca_read(dd, P7CA_REG_IRQ_STATUS);
	p7ca_write(dd, P7CA_REG_IRQ_STATUS, reg);
	dd->irq_flags |= reg;

	schedule_work(&dd->done_task);

	spin_unlock_irqrestore(&dd->lock, flags);

	return IRQ_HANDLED;
}

static int p7ca_dma_alloc_desc(struct p7ca_dev *dd, struct p7ca_desc_slot *slot)
{
	int err = -ENOMEM;

	slot->buf_in = dma_alloc_coherent(dd->dev, PAGE_SIZE << P7CA_CACHE_SIZE,
					&slot->dma_addr_in, GFP_KERNEL);
	slot->buf_out = dma_alloc_coherent(dd->dev, PAGE_SIZE << P7CA_CACHE_SIZE,
					&slot->dma_addr_out, GFP_KERNEL);

	if (!slot->buf_in || !slot->buf_out) {
		dev_err(dd->dev, "unable to alloc pages.\n");
		goto err_alloc;
	}
	slot->buflen = PAGE_SIZE;
	slot->buflen &= ~(AES_BLOCK_SIZE - 1);

	return 0;

err_alloc:
	p7ca_dma_cleanup(dd, slot);

	dev_err(dd->dev, "error: %d\n", err);

	return err;
}

static void p7ca_free_desc(struct p7ca_dev *dd, struct p7ca_desc_slot *slot, int isLast)
{
	if (!slot)
		return;

	slot->flags = FLAGS_DESC_UNUSED;
	slot->slot_num = -1;
}

static struct p7ca_desc_slot *p7ca_get_desc_by_num(struct p7ca_dev *dd, int num)
{
	int i;
	for (i = 0; i < P7CA_NR_DESCSLOTS; i++) {
		if (dd->desc_slots[i].slot_num == num)
			return &dd->desc_slots[i];
	}

	return NULL;
}

static struct p7ca_desc_slot *p7ca_get_desc(struct p7ca_dev *dd)
{
	int i;

	for (i = 0; i < P7CA_NR_DESCSLOTS; i++) {
		if (dd->desc_slots[i].flags == FLAGS_DESC_UNUSED) {
			dd->desc_slots[i].flags = FLAGS_DESC_USED;
			return &dd->desc_slots[i];
		}
	}

	return NULL;
}

static void sg_copy_buf(struct p7ca_dev *dd, void *buf, struct scatterlist *sg,
		unsigned int start, unsigned int nbytes, int out)
{
	struct scatter_walk walk;

	if (!nbytes)
		return;

	scatterwalk_start(&walk, sg);
	scatterwalk_advance(&walk, start);
	scatterwalk_copychunks(buf, &walk, nbytes, out);
	scatterwalk_done(&walk, out, 0);
}

static int sg_copy(struct p7ca_dev *dd, struct scatterlist **sg,
		size_t *offset, void *buf,
		size_t buflen, size_t total, int out)
{
	unsigned int count, off = 0;

	while (buflen && total) {

		count = min((*sg)->length - *offset, total);
		count = min(count, buflen);

		if (!count)
			return off;

		/*
		 * buflen and total are AES_BLOCK_SIZE size aligned,
		 * so count should be also aligned
		 */
		sg_copy_buf(dd, buf + off, *sg, *offset, count, out);

		off += count;
		buflen -= count;
		*offset += count;
		total -= count;

		if (*offset == (*sg)->length) {
			*sg = sg_next(*sg);
			if (*sg)
				*offset = 0;
			else
				total = 0;
		}
	}

	return off;
}

static int p7ca_crypt_dma(struct p7ca_dev *dd,
		dma_addr_t dma_addr_in,
		dma_addr_t dma_addr_out, int length,
		struct p7ca_desc_slot *slot)
{
	int len32;

	slot->dma_size = length;
	len32 = DIV_ROUND_UP(length, sizeof(u32));
	p7ca_write(dd, P7CA_REG_DESC_SIZE(slot->slot_num), len32);
	p7ca_write(dd, P7CA_REG_DESC_SOURCE(slot->slot_num), dma_addr_in);
	p7ca_write(dd, P7CA_REG_DESC_DESTINATION(slot->slot_num), dma_addr_out);

	/* Start DMA */
	p7ca_write_mask(dd, P7CA_REG_DESC_CTRL(slot->slot_num), P7CA_DESC_CTRL_DESC_VALID,
			P7CA_DESC_CTRL_DESC_VALID);

	return 0;
}

static int p7ca_crypt_dma_start(struct p7ca_dev *dd, struct p7ca_desc_slot *slot)
{
	int err, fast = 0, in, out;
	size_t count;
	dma_addr_t addr_in, addr_out;

	if (sg_is_last(slot->in_sg) && sg_is_last(slot->out_sg)) {
		/* check for alignment */
		in = IS_ALIGNED((u32)slot->in_sg->offset, sizeof(u32));
		out = IS_ALIGNED((u32)slot->out_sg->offset, sizeof(u32));

		fast = in && out;
	}

	if (fast)  {
		count = min(slot->total, sg_dma_len(slot->in_sg));
		count = min(count, sg_dma_len(slot->out_sg));

		if (count != slot->total) {
			dev_err(dd->dev, "request length != buffer length\n");
			return -EINVAL;
		}

		err = dma_map_sg(dd->dev, slot->in_sg, 1, DMA_TO_DEVICE);
		if (!err) {
			dev_err(dd->dev, "dma_map_sg() error\n");
			return -EINVAL;
		}

		err = dma_map_sg(dd->dev, slot->out_sg, 1, DMA_FROM_DEVICE);
		if (!err) {
			dev_err(dd->dev, "dma_map_sg() error\n");
			dma_unmap_sg(dd->dev, slot->in_sg, 1, DMA_TO_DEVICE);
			return -EINVAL;
		}

		addr_in = sg_dma_address(slot->in_sg);
		addr_out = sg_dma_address(slot->out_sg);

		dd->flags |= FLAGS_CMD_FAST;

	} else {
		/* use cache buffers */
		count = sg_copy(dd, &slot->in_sg, &slot->in_offset, slot->buf_in,
				slot->buflen, slot->total, 0);

		addr_in = slot->dma_addr_in;
		addr_out = slot->dma_addr_out;

		dd->flags &= ~FLAGS_CMD_FAST;

	}

	slot->total -= count;

	err = p7ca_crypt_dma(dd, addr_in, addr_out, count, slot);
	if (err) {
		dma_unmap_sg(dd->dev, slot->in_sg, 1, DMA_TO_DEVICE);
		dma_unmap_sg(dd->dev, slot->out_sg, 1, DMA_TO_DEVICE);
	}

	return err;
}

static void p7ca_finish_req(struct p7ca_dev *dd,
		struct p7ca_desc_slot *slot,
		int err)
{
	if (slot->req->base.complete)
		slot->req->base.complete(&slot->req->base, err);
}

static int p7ca_crypt_dma_stop(struct p7ca_dev *dd, struct p7ca_desc_slot *slot)
{
	int err = 0;
	size_t count;
	struct p7ca_reqctx *rctx = ablkcipher_request_ctx(slot->req);

	if (dd->flags & FLAGS_CMD_FAST) {
		dma_unmap_sg(dd->dev, slot->out_sg, 1, DMA_FROM_DEVICE);
		dma_unmap_sg(dd->dev, slot->in_sg, 1, DMA_TO_DEVICE);

	} else {
		/* copy data */
		count = sg_copy(dd, &slot->out_sg, &slot->out_offset, slot->buf_out,
				slot->buflen, slot->dma_size, 1);

		if (count != slot->dma_size) {
			err = -EINVAL;
			dev_err(dd->dev, "not all data converted: %u\n", count);
		} else {
			if (rctx != NULL && slot->buf_out != NULL) {
				if (((rctx->mode == ACRYPTO_MODE_CBC)
							|| (rctx->mode == ACRYPTO_MODE_OFB)
							|| (rctx->mode == ACRYPTO_MODE_CTR))
						&& slot->total
						&& rctx->ivsize) {
					memcpy(rctx->iv, slot->buf_out + slot->dma_size - rctx->ivsize,
							rctx->ivsize);
				}
			}
		}
	}

	return err;
}

static int p7ca_handle_queue(struct p7ca_dev *dd,
		struct ablkcipher_request *req)
{
	struct p7ca_desc_slot *slot;
	struct crypto_async_request *async_req, *backlog;
	struct p7ca_key_slot *key_slot;
	unsigned long flags;
	int err, ret = 0;
	struct p7ca_ctx *ctx;

	spin_lock_irqsave(&dd->lock, flags);

	if (dd->flags & FLAGS_SUSPENDED) {
		spin_unlock_irqrestore(&dd->lock, flags);
		return -EAGAIN;
	}

	if (req)
		ret = ablkcipher_enqueue_request(&dd->queue, req);

	backlog = crypto_get_backlog(&dd->queue);
	async_req = crypto_dequeue_request(&dd->queue);

	spin_unlock_irqrestore(&dd->lock, flags);

	if (!async_req)
		return ret;

	req = ablkcipher_request_cast(async_req);
	ctx = crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
	ctx->dd = dd;

	if (!ctx->slot) {
		key_slot = p7ca_find_key_slot();
		if (key_slot)
			ctx->slot = key_slot;
		else {
			dev_dbg(dd->dev, "%s:no empty key slot\n", __func__);
			spin_lock_irqsave(&dd->lock, flags);
			ablkcipher_enqueue_request(&dd->queue, req);
			spin_unlock_irqrestore(&dd->lock, flags);
			return ret;
		}
	}

	spin_lock_irqsave(&dd->lock, flags);
	slot = p7ca_get_desc(dd);
	spin_unlock_irqrestore(&dd->lock, flags);

	if (!slot) {
		dev_dbg(dd->dev, "%s:no empty desc slot\n", __func__);
		spin_lock_irqsave(&dd->lock, flags);
		ablkcipher_enqueue_request(&dd->queue, req);
		spin_unlock_irqrestore(&dd->lock, flags);
		return ret;
	}

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	/* assign new request to device */
	slot->req = req;
	slot->total = req->nbytes;
	slot->in_offset = 0;
	slot->in_sg = req->src;
	slot->out_offset = 0;
	slot->out_sg = req->dst;

	err = p7ca_write_ctrl(dd, slot);
	if (!err)
		err = p7ca_crypt_dma_start(dd, slot);

	if (err) {

		/* aes_task will not finish it, so do it here */
		p7ca_finish_req(dd, slot, err);
		p7ca_free_desc(dd, slot, 1);
		schedule_work(&dd->queue_task);
		return ret;
	}

	return ret; /* return ret, which is enqueue return value */
}

static void p7ca_done_task(struct work_struct *work)
{
	struct p7ca_dev *dd = container_of(work, struct p7ca_dev, done_task);
	unsigned long flags;
	struct p7ca_desc_slot *slot;
	int i, err;

	for (i = 0; i < P7CA_NR_DESCSLOTS; i++) {

		if (!(dd->irq_flags & P7CA_IRQ_STATUS_EN(i)))
			continue;

		spin_lock_irqsave(&dd->lock, flags);
		slot = p7ca_get_desc_by_num(dd, i);
		dd->irq_flags &= ~P7CA_IRQ_STATUS_EN(i);
		spin_unlock_irqrestore(&dd->lock, flags);

		err = p7ca_crypt_dma_stop(dd, slot);

		if (slot->total && !err) {

			err = p7ca_write_ctrl(dd, slot);
			if (!err) {
				err = p7ca_crypt_dma_start(dd, slot);
				if (!err)
					continue; /* DMA started. Not fininishing. */
			}
		}

		p7ca_finish_req(dd, slot, err);
		p7ca_free_desc(dd, slot, 0);
	}

	p7ca_handle_queue(dd, NULL);
}

static void p7ca_queue_task(struct work_struct *work)
{
	struct p7ca_dev *dd = container_of(work, struct p7ca_dev, queue_task);
	p7ca_handle_queue(dd, NULL);
}

static int p7ca_crypt(struct ablkcipher_request *req, u8 op,
		u8 type, u8 mode)
{
	struct p7ca_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct p7ca_reqctx *rctx = ablkcipher_request_ctx(req);
	struct p7ca_dev *dd = p7ca_dev;
	u32 blocksize = crypto_tfm_alg_blocksize(req->base.tfm);
	unsigned ivsize;

	ivsize = crypto_ablkcipher_ivsize(crypto_ablkcipher_reqtfm(req));

	if (!IS_ALIGNED(req->nbytes, blocksize)) {
		dev_err(dd->dev, "request size(%d) is not exact amount of blocks nbytes(%d)\n",
				req->nbytes,
				blocksize);
		return -EINVAL;
	}

	if (ctx->keylen != AES_KEYSIZE_128 && type == ACRYPTO_TYPE_AES_128) {
		if (ctx->keylen == AES_KEYSIZE_192)
			type = ACRYPTO_TYPE_AES_192;
		else if (ctx->keylen == AES_KEYSIZE_256)
			type = ACRYPTO_TYPE_AES_256;
	}

	rctx->op = op;
	rctx->mode = mode;
	rctx->type = type;
	rctx->iv = req->info;
	rctx->ivsize = ivsize;

	return p7ca_handle_queue(dd, req);
}

/* ********************** ALG API ************************************ */

static int p7ca_setkey(struct crypto_ablkcipher *tfm, const u8 *key,
		unsigned int keylen)
{
	struct p7ca_ctx *ctx = crypto_ablkcipher_ctx(tfm);

	if (keylen != DES_KEY_SIZE && keylen != DES3_EDE_KEY_SIZE
			&& keylen != AES_KEYSIZE_128
			&& keylen != AES_KEYSIZE_192
			&& keylen != AES_KEYSIZE_256) {
		return -EINVAL;
	}

	if (key) {
		memcpy(ctx->key, key, keylen);
		ctx->keylen = keylen;
	}

	ctx->flags |= FLAGS_NEW_KEY;

	return 0;
}

static int p7ca_aes_ecb_encrypt(struct ablkcipher_request *req)
{
	return p7ca_crypt(req,
			ACRYPTO_OP_ENCODE,
			ACRYPTO_TYPE_AES_128,
			ACRYPTO_MODE_ECB);
}

static int p7ca_aes_ecb_decrypt(struct ablkcipher_request *req)
{
	return p7ca_crypt(req,
			ACRYPTO_OP_DECODE,
			ACRYPTO_TYPE_AES_128,
			ACRYPTO_MODE_ECB);
}

static int p7ca_aes_cbc_encrypt(struct ablkcipher_request *req)
{
	return p7ca_crypt(req,
			ACRYPTO_OP_ENCODE,
			ACRYPTO_TYPE_AES_128,
			ACRYPTO_MODE_CBC);
}

static int p7ca_aes_cbc_decrypt(struct ablkcipher_request *req)
{
	return p7ca_crypt(req,
			ACRYPTO_OP_DECODE,
			ACRYPTO_TYPE_AES_128,
			ACRYPTO_MODE_CBC);
}

static int p7ca_aes_ofb_encrypt(struct ablkcipher_request *req)
{
	return p7ca_crypt(req,
			ACRYPTO_OP_ENCODE,
			ACRYPTO_TYPE_AES_128,
			ACRYPTO_MODE_OFB);
}

static int p7ca_aes_ofb_decrypt(struct ablkcipher_request *req)
{
	return p7ca_crypt(req,
			ACRYPTO_OP_DECODE,
			ACRYPTO_TYPE_AES_128,
			ACRYPTO_MODE_OFB);
}

static int p7ca_aes_ctr_encrypt(struct ablkcipher_request *req)
{
	return p7ca_crypt(req,
			ACRYPTO_OP_ENCODE,
			ACRYPTO_TYPE_AES_128,
			ACRYPTO_MODE_CTR);
}

static int p7ca_aes_ctr_decrypt(struct ablkcipher_request *req)
{
	return p7ca_crypt(req,
			ACRYPTO_OP_DECODE,
			ACRYPTO_TYPE_AES_128,
			ACRYPTO_MODE_CTR);
}

static int p7ca_des_ecb_encrypt(struct ablkcipher_request *req)
{
	return p7ca_crypt(req,
			ACRYPTO_OP_ENCODE,
			ACRYPTO_TYPE_DES,
			ACRYPTO_MODE_ECB);
}

static int p7ca_des_ecb_decrypt(struct ablkcipher_request *req)
{
	return p7ca_crypt(req,
			ACRYPTO_OP_DECODE,
			ACRYPTO_TYPE_DES,
			ACRYPTO_MODE_ECB);
}

static int p7ca_des_cbc_encrypt(struct ablkcipher_request *req)
{
	return p7ca_crypt(req,
			ACRYPTO_OP_ENCODE,
			ACRYPTO_TYPE_DES,
			ACRYPTO_MODE_CBC);
}

static int p7ca_des_cbc_decrypt(struct ablkcipher_request *req)
{
	return p7ca_crypt(req,
			ACRYPTO_OP_DECODE,
			ACRYPTO_TYPE_DES,
			ACRYPTO_MODE_CBC);
}

static int p7ca_des_ofb_encrypt(struct ablkcipher_request *req)
{
	return p7ca_crypt(req,
			ACRYPTO_OP_ENCODE,
			ACRYPTO_TYPE_DES,
			ACRYPTO_MODE_OFB);
}

static int p7ca_des_ofb_decrypt(struct ablkcipher_request *req)
{
	return p7ca_crypt(req,
			ACRYPTO_OP_DECODE,
			ACRYPTO_TYPE_DES,
			ACRYPTO_MODE_OFB);
}

static int p7ca_3des_ecb_encrypt(struct ablkcipher_request *req)
{
	return p7ca_crypt(req,
			ACRYPTO_OP_ENCODE,
			ACRYPTO_TYPE_3DES,
			ACRYPTO_MODE_ECB);
}

static int p7ca_3des_ecb_decrypt(struct ablkcipher_request *req)
{
	return p7ca_crypt(req,
			ACRYPTO_OP_DECODE,
			ACRYPTO_TYPE_3DES,
			ACRYPTO_MODE_ECB);
}

static int p7ca_3des_cbc_encrypt(struct ablkcipher_request *req)
{
	return p7ca_crypt(req,
			ACRYPTO_OP_ENCODE,
			ACRYPTO_TYPE_3DES,
			ACRYPTO_MODE_CBC);
}

static int p7ca_3des_cbc_decrypt(struct ablkcipher_request *req)
{
	return p7ca_crypt(req,
			ACRYPTO_OP_DECODE,
			ACRYPTO_TYPE_3DES,
			ACRYPTO_MODE_CBC);
}

static int p7ca_3des_ofb_encrypt(struct ablkcipher_request *req)
{
	return p7ca_crypt(req,
			ACRYPTO_OP_ENCODE,
			ACRYPTO_TYPE_3DES,
			ACRYPTO_MODE_OFB);
}

static int p7ca_3des_ofb_decrypt(struct ablkcipher_request *req)
{
	return p7ca_crypt(req,
			ACRYPTO_OP_DECODE,
			ACRYPTO_TYPE_3DES,
			ACRYPTO_MODE_OFB);
}

static int p7ca_cra_init(struct crypto_tfm *tfm)
{
	tfm->crt_ablkcipher.reqsize = sizeof(struct p7ca_reqctx);

	return 0;
}

static void p7ca_cra_exit(struct crypto_tfm *tfm)
{
	struct p7ca_ctx *ctx =
		crypto_ablkcipher_ctx((struct crypto_ablkcipher *)tfm);

	if (ctx && ctx->slot)
		p7ca_release_key_slot(ctx->slot);
}

/* ********************** ALGS ************************************ */

static struct crypto_alg algs[] = {
	/* 3DES: ECB CBC and OFB are supported */
	{
		.cra_name		= "ecb(des3_ede)",
		.cra_driver_name	= "ecb-3des-p7",
		.cra_priority		= 100,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_KERN_DRIVER_ONLY |
			CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES3_EDE_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct p7ca_ctx),
		.cra_alignmask		= 0,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module		= THIS_MODULE,
		.cra_init		= p7ca_cra_init,
		.cra_exit		= p7ca_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize	= DES3_EDE_KEY_SIZE,
			.max_keysize	= DES3_EDE_KEY_SIZE,
			.setkey		= p7ca_setkey,
			.encrypt	= p7ca_3des_ecb_encrypt,
			.decrypt	= p7ca_3des_ecb_decrypt,
		}
	},
	{
		.cra_name		= "cbc(des3_ede)",
		.cra_driver_name	= "cbc-3des-p7",
		.cra_priority		= 100,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_KERN_DRIVER_ONLY |
			CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES3_EDE_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct p7ca_ctx),
		.cra_alignmask		= 0,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module		= THIS_MODULE,
		.cra_init		= p7ca_cra_init,
		.cra_exit		= p7ca_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize	= DES3_EDE_KEY_SIZE,
			.max_keysize	= DES3_EDE_KEY_SIZE,
			.ivsize		= DES3_EDE_BLOCK_SIZE,
			.setkey		= p7ca_setkey,
			.encrypt	= p7ca_3des_cbc_encrypt,
			.decrypt	= p7ca_3des_cbc_decrypt,
		}
	},
	{
		.cra_name		= "ofb(des3_ede)",
		.cra_driver_name	= "ofb-3des-p7",
		.cra_priority		= 100,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_KERN_DRIVER_ONLY |
			CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES3_EDE_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct p7ca_ctx),
		.cra_alignmask		= 0,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module		= THIS_MODULE,
		.cra_init		= p7ca_cra_init,
		.cra_exit		= p7ca_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize	= DES3_EDE_KEY_SIZE,
			.max_keysize	= DES3_EDE_KEY_SIZE,
			.ivsize		= DES3_EDE_BLOCK_SIZE,
			.setkey		= p7ca_setkey,
			.encrypt	= p7ca_3des_ofb_encrypt,
			.decrypt	= p7ca_3des_ofb_decrypt,
		}
	},
	/* DES: ECB CBC and OFB are supported */
	{
		.cra_name		= "ecb(des)",
		.cra_driver_name	= "ecb-des-p7",
		.cra_priority		= 100,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_KERN_DRIVER_ONLY |
			CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct p7ca_ctx),
		.cra_alignmask		= 0,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module		= THIS_MODULE,
		.cra_init		= p7ca_cra_init,
		.cra_exit		= p7ca_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize	= DES_KEY_SIZE,
			.max_keysize	= DES_KEY_SIZE,
			.setkey		= p7ca_setkey,
			.encrypt	= p7ca_des_ecb_encrypt,
			.decrypt	= p7ca_des_ecb_decrypt,
		}
	},
	{
		.cra_name		= "cbc(des)",
		.cra_driver_name	= "cbc-des-p7",
		.cra_priority		= 100,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_KERN_DRIVER_ONLY |
			CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct p7ca_ctx),
		.cra_alignmask		= 0,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module		= THIS_MODULE,
		.cra_init		= p7ca_cra_init,
		.cra_exit		= p7ca_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize	= DES_KEY_SIZE,
			.max_keysize	= DES_KEY_SIZE,
			.ivsize		= DES_BLOCK_SIZE,
			.setkey		= p7ca_setkey,
			.encrypt	= p7ca_des_cbc_encrypt,
			.decrypt	= p7ca_des_cbc_decrypt,
		}
	},
	{
		.cra_name		= "ofb(des)",
		.cra_driver_name	= "ofb-des-p7",
		.cra_priority		= 100,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_KERN_DRIVER_ONLY |
			CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct p7ca_ctx),
		.cra_alignmask		= 0,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module		= THIS_MODULE,
		.cra_init		= p7ca_cra_init,
		.cra_exit		= p7ca_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize	= DES_KEY_SIZE,
			.max_keysize	= DES_KEY_SIZE,
			.ivsize		= DES_BLOCK_SIZE,
			.setkey		= p7ca_setkey,
			.encrypt	= p7ca_des_ofb_encrypt,
			.decrypt	= p7ca_des_ofb_decrypt,
		}
	},
	/* AES: ECB CBC OFB and CTR are supported */
	{
		.cra_name		= "ecb(aes)",
		.cra_driver_name	= "ecb-aes-p7",
		.cra_priority		= 100,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_KERN_DRIVER_ONLY |
			CRYPTO_ALG_ASYNC,
		.cra_blocksize		= AES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct p7ca_ctx),
		.cra_alignmask		= 0,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module		= THIS_MODULE,
		.cra_init		= p7ca_cra_init,
		.cra_exit		= p7ca_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE,
			.setkey		= p7ca_setkey,
			.encrypt	= p7ca_aes_ecb_encrypt,
			.decrypt	= p7ca_aes_ecb_decrypt,
		}
	},
	{
		.cra_name		= "cbc(aes)",
		.cra_driver_name	= "cbc-aes-p7",
		.cra_priority		= 100,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_KERN_DRIVER_ONLY |
			CRYPTO_ALG_ASYNC,
		.cra_blocksize		= AES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct p7ca_ctx),
		.cra_alignmask		= 0,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module		= THIS_MODULE,
		.cra_init		= p7ca_cra_init,
		.cra_exit		= p7ca_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE,
			.ivsize		= AES_BLOCK_SIZE,
			.setkey		= p7ca_setkey,
			.encrypt	= p7ca_aes_cbc_encrypt,
			.decrypt	= p7ca_aes_cbc_decrypt,
		}
	},
	{
		.cra_name		= "ofb(aes)",
		.cra_driver_name	= "ofb-aes-p7",
		.cra_priority		= 100,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_KERN_DRIVER_ONLY |
			CRYPTO_ALG_ASYNC,
		.cra_blocksize		= AES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct p7ca_ctx),
		.cra_alignmask		= 0,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module		= THIS_MODULE,
		.cra_init		= p7ca_cra_init,
		.cra_exit		= p7ca_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE,
			.ivsize		= AES_BLOCK_SIZE,
			.setkey		= p7ca_setkey,
			.encrypt	= p7ca_aes_ofb_encrypt,
			.decrypt	= p7ca_aes_ofb_decrypt,
		}
	},
	{
		.cra_name		= "ctr(aes)",
		.cra_driver_name	= "ctr-aes-p7",
		.cra_priority		= 100,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_KERN_DRIVER_ONLY |
			CRYPTO_ALG_ASYNC,
		.cra_blocksize		= AES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct p7ca_ctx),
		.cra_alignmask		= 0,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module		= THIS_MODULE,
		.cra_init		= p7ca_cra_init,
		.cra_exit		= p7ca_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE,
			.ivsize		= AES_BLOCK_SIZE,
			.setkey		= p7ca_setkey,
			.encrypt	= p7ca_aes_ctr_encrypt,
			.decrypt	= p7ca_aes_ctr_decrypt,
		}
	},
};

static int p7ca_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct p7ca_dev *dd;
	struct resource *res;
	int err = -ENOMEM, i, j;

	dd = kzalloc(sizeof(struct p7ca_dev), GFP_KERNEL);
	if (dd == NULL) {
		dev_err(dd->dev, "unable to alloc data struct.\n");
		return err;
	}

	dd->dev = dev;
	platform_set_drvdata(pdev, dd);

	dd->key_slots = kzalloc(sizeof(struct p7ca_key_slot) *
				 P7CA_NR_KEYSLOTS, GFP_KERNEL);
	if (dd->key_slots == NULL) {
		dev_err(dd->dev, "unable to alloc slot struct.\n");
		goto err_alloc_key_slots;
	}

	dd->desc_slots = kzalloc(sizeof(struct p7ca_desc_slot) *
				 P7CA_NR_DESCSLOTS, GFP_KERNEL);
	if (dd->desc_slots == NULL) {
		dev_err(dd->dev, "unable to alloc slot struct.\n");
		goto err_alloc_desc_slots;
	}
	dd->cur_desc = -1;

	spin_lock_init(&dd->lock);
	crypto_init_queue(&dd->queue, P7CA_QUEUE_LENGTH);

	/* Get the base address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dd->dev, "invalid resource type\n");
		err = -ENODEV;
		goto err_res;
	}
	dd->phys_base = res->start;

	dd->iclk = clk_get(dev, NULL);
	if (IS_ERR(dd->iclk)) {
		dev_err(dd->dev, "clock intialization failed.\n");
		err = PTR_ERR(dd->iclk);
		goto err_res;
	}

	err = clk_prepare_enable(dd->iclk);
	if (err) {
		dev_err(dd->dev, "Cant start clock\n") ;
		goto err_putclk;
	}

	dd->irq = platform_get_irq(pdev, 0);

	err = request_irq(dd->irq,
			p7ca_interrupt,
			0,
			dev_name(&pdev->dev),
			dd);
	if (err) {
		dev_err(dd->dev, "request irq failed (%d)\n", err);
		goto err_putclk;
	}

	dd->io_base = ioremap(dd->phys_base, SZ_4K);
	if (!dd->io_base) {
		dev_err(dd->dev, "can't ioremap\n");
		err = -ENOMEM;
		goto err_io;
	}

	INIT_WORK(&dd->done_task,
			p7ca_done_task);
	INIT_WORK(&dd->queue_task,
			p7ca_queue_task);

	for (i = 0; i < P7CA_NR_DESCSLOTS; i++) {
		dd->desc_slots[i].slot_num = -1;
		dd->desc_slots[i].flags = FLAGS_DESC_UNUSED;
		err = p7ca_dma_alloc_desc(dd,
				&dd->desc_slots[i]);
		if (err) {
			dev_err(dd->dev, "error dma alloc(%d)\n", err);
			goto err_dma;
		}
	}

	INIT_LIST_HEAD(&dev_list);

	spin_lock_init(&list_lock);
	spin_lock(&list_lock);
	for (i = 0; i < P7CA_NR_KEYSLOTS; i++) {
		dd->key_slots[i].slot_num = i;
		INIT_LIST_HEAD(&dd->key_slots[i].node);
		list_add_tail(&dd->key_slots[i].node, &dev_list);
	}
	spin_unlock(&list_lock);

	p7ca_dev = dd;
	for (i = 0; i < ARRAY_SIZE(algs); i++) {
		INIT_LIST_HEAD(&algs[i].cra_list);
		err = crypto_register_alg(&algs[i]);
		if (err)
			goto err_algs;
	}

	p7ca_write(dd,
		P7CA_REG_IRQ_EN,
		P7CA_IRQ_EN_ENABLE);

	dev_info(dev, "registered\n");

	return 0;

err_algs:
	for (j = 0; j < i; j++)
		crypto_unregister_alg(&algs[j]);

err_dma:
	for (i = 0; i < P7CA_NR_DESCSLOTS; i++)
		p7ca_dma_cleanup(dd,
			&dd->desc_slots[i]);

	cancel_work_sync(&dd->done_task);
	cancel_work_sync(&dd->queue_task);
	iounmap(dd->io_base);
err_io:
	free_irq(dd->irq, dd);
	clk_disable_unprepare(dd->iclk);
err_putclk:
	clk_put(dd->iclk);
err_alloc_desc_slots:
	kfree(dd->desc_slots);
err_alloc_key_slots:
	kfree(dd->key_slots);
err_res:
	kfree(dd);
	p7ca_dev = NULL;

	dev_err(dd->dev, "initialization failed.\n");
	return err;
}

static int p7ca_remove(struct platform_device *pdev)
{
	struct p7ca_dev *dd = platform_get_drvdata(pdev);
	int i;

	if (!dd)
		return -ENODEV;

	spin_lock(&list_lock);
	list_del(&dev_list);
	spin_unlock(&list_lock);

	for (i = 0; i < ARRAY_SIZE(algs); i++)
		crypto_unregister_alg(&algs[i]);

	cancel_work_sync(&dd->done_task);
	cancel_work_sync(&dd->queue_task);

	p7ca_write(dd,
		P7CA_REG_IRQ_EN, 0);

	for (i = 0; i < P7CA_NR_DESCSLOTS; i++)
		p7ca_dma_cleanup(dd,
			&dd->desc_slots[i]);

	free_irq(dd->irq, dd);

	iounmap(dd->io_base);
	clk_disable_unprepare(dd->iclk);
	clk_put(dd->iclk);
	kfree(dd->key_slots);
	kfree(dd->desc_slots);
	kfree(dd);
	p7ca_dev = NULL;

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int p7ca_suspend(struct device *dev)
{
	struct p7ca_dev *dd = p7ca_dev;

	dd->flags |= FLAGS_SUSPENDED;
	p7ca_write(dd,
		P7CA_REG_IRQ_EN, 0);
	clk_disable_unprepare(dd->iclk);

	return 0;
}

static int p7ca_resume(struct device *dev)
{
	struct p7ca_dev *dd = p7ca_dev;

	clk_prepare_enable(dd->iclk);
	p7ca_write(dd,
		P7CA_REG_IRQ_EN,
		P7CA_IRQ_EN_ENABLE);
	dd->flags &= ~FLAGS_SUSPENDED;
	p7ca_handle_queue(dd, NULL);

	return 0;
}
#else
#define p7ca_suspend   NULL
#define p7ca_resume    NULL
#endif

static const struct dev_pm_ops p7ca_dev_pm_ops = {
	.suspend  = p7ca_suspend,
	.resume   = p7ca_resume,
};

static struct platform_driver p7ca_driver = {
	.probe	= p7ca_probe,
	.remove	= p7ca_remove,
	.driver	= {
		.name	= "p7-crypto",
		.owner	= THIS_MODULE,
		.pm     = &p7ca_dev_pm_ops,
	},
};

static int __init p7ca_mod_init(void)
{
	int err;

	pr_info("loading %s driver\n", "p7-crypto");

	err = platform_driver_register(&p7ca_driver);
	if (err)
		return err;

	return 0;
}

static void __exit p7ca_mod_exit(void)
{
	platform_driver_unregister(&p7ca_driver);
}

module_init(p7ca_mod_init);
module_exit(p7ca_mod_exit);

MODULE_DESCRIPTION("P7 AES hw acceleration support.");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Karl Leplat");

