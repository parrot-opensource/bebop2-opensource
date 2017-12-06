/*
 * linux/drivers/parrot/media/p7-mpegts.c - Parrot7 MPEG-TS driver
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * @author: Damien Riegel <damien.riegel.ext@parrot.com>
 * @author: Jeremie Samuel <jeremie.samuel.ext@parrot.com>
 *
 * This file is released under the GPL
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/lcm.h>

#include <spi/p7-swb.h>

#include <spi/p7-swb.h>

#include "p7-mpegts.h"
#include "p7-mpegts_priv.h"

#define MAX_BLK_NB 9
#define MIN_BLK_NB 3

#ifdef CONFIG_PM_SLEEP

static u32 p7mpg_save_regs[] = {
	P7MPG_CTRL,
	P7MPG_SYNC_VAL,
	P7MPG_SYNC_NUM,
	P7MPG_PKT_SIZE,
	P7MPG_DATA_PIPE,
	P7MPG_ITEN,
	P7MPG_ITACK,
	P7MPG_RXFIFO_TH,
	P7MPG_RXCNT_TH,
};

#endif

static DECLARE_WAIT_QUEUE_HEAD(poll_wait_queue);

static bool p7mpg_filter(struct dma_chan* chan, void* param)
{
	if (chan->chan_id == (unsigned int) param)
		return true;

	return false;
}

int p7mpg_request_pad(struct p7mpg_core const * core, unsigned int pad,
                      enum p7swb_direction dir, enum p7swb_function func)
{
	return p7swb_request_pad(core->id, P7SPI_CORE_MPEG,
	                           (void const * const) core, pad, dir, func);
}

void p7mpg_release_pad(struct p7mpg_core const * core, unsigned int pad,
                       enum p7swb_direction dir)
{
	p7swb_release_pad((void const * const) core, pad, dir);
}

static struct p7mpg_core* p7mpg_to_core(void *data)
{
	return container_of(data, struct p7mpg_core, miscdev);
}

static unsigned int p7mpg_get_pkt_sz(struct p7mpg_core *core)
{
	return __raw_readl(core->vaddr + P7MPG_PKT_SIZE);
}

static inline unsigned int p7mpg_get_min_pkt_by_blk(unsigned int pkt_size,
						    size_t dma_min)
{
	return lcm(pkt_size, dma_min) / pkt_size;
}

static inline unsigned int p7mpg_set_pkt_by_blk(struct p7mpg_core *core,
						unsigned int blk_size,
						unsigned int pkt_size)
{
	/* Blocks and DMA must be aligned! */
	unsigned int min_pkt_by_blk = p7mpg_get_min_pkt_by_blk(pkt_size,
							core->dma_min);
	return roundup(blk_size / pkt_size, min_pkt_by_blk);
}

static inline unsigned int p7mpg_get_max_blk_nb(struct p7mpg_core *core,
						unsigned int blk_size)
{
	int max_blk_nb = rounddown(core->dma_region_sz, blk_size) / blk_size;
	if (max_blk_nb > MAX_BLK_NB)
		return MAX_BLK_NB;
	else if (max_blk_nb < MIN_BLK_NB)
		return 0;
	else 
		return max_blk_nb;
}

void p7mpg_setup_metadata(struct p7mpg_core* core)
{
	core->metadata->pkt_size  = p7mpg_get_pkt_sz(core);
	core->metadata->blk_size  = core->metadata->pkt_size * core->pkt_by_blk;
	core->metadata->blk_by_rb = core->blk_by_rb;
	core->metadata->blks_states = 0;
}

static int p7mpg_setup_core(struct p7mpg_core *core,
			    struct p7mpg_plat_data const * pdata)
{
	ssize_t fifo_sz;

	fifo_sz = p7spi_alloc_fifo(p7mpg_core_id(core), P7SPI_CORE_MPEG,
	                           pdata->fifo_wcnt);
	if (fifo_sz == -EAGAIN)
		return -EPROBE_DEFER;
	else if (fifo_sz < 0)
		return fifo_sz;
	core->fifo_sz = (size_t)fifo_sz * sizeof(u32);

	/* Compute FIFO wakeup threshold in number of bytes. */
	core->thres_sz = pdata->thres_wcnt * sizeof(u32);
	if (core->thres_sz > (core->fifo_sz - sizeof(u32))) {
		size_t const thres = core->fifo_sz / 2;

		dev_warn(core->device,
		         "correcting invalid specified threshold (%u -> %u)",
		         core->thres_sz, thres);
		core->thres_sz = thres;
	}

	/* Compute DMA burst length and threshold. */
	core->dma_cfg.src_maxburst = min_t(u32,
	                                   core->thres_sz /
	                                   core->dma_cfg.src_addr_width,
	                                   16);
	core->dma_cfg.dst_maxburst = core->dma_cfg.src_maxburst;
	core->dma_thres = core->dma_cfg.src_maxburst *
	                  core->dma_cfg.src_addr_width;

	if (core->thres_sz != core->dma_thres)
		dev_info(core->device,
		         "aligning DMA burst threshold (%u -> %u)",
		         core->thres_sz, core->dma_thres);

	core->dma_min = lcm(core->dma_thres, dma_get_cache_alignment());

	return 0;
}

void p7mpg_complete_dma(void* ctxt)
{
	struct p7mpg_core *core = (struct p7mpg_core*) ctxt;
	unsigned int       next_period;
	unsigned int       shift = 2 * core->dma_block;

	/*
	 * when entering this function, dma_block is the block that trigerred
	 * interrupt. Update metadata in the ring buffer and mark next periods
	 * as being reserved for DMA xfer.
	 */
	atomic_set_mask(3 << shift, &core->metadata->blks_states);

	core->dma_block = (core->dma_block + 1) % core->blk_by_rb;

	/*
	 * okay now dma_block is the block that the DMA is currently writting
	 * in, so we reserve the next period.
	 */
	next_period = (core->dma_block + 1) % core->blk_by_rb;
	shift = 2 * next_period;
	atomic_clear_mask(3 << shift, &core->metadata->blks_states);

	wake_up(&poll_wait_queue);

	dev_dbg(core->device, "DMA completion called\n");
}

static int p7mpg_config_dma(struct p7mpg_core* core)
{
	int err;
	size_t pkt_size = p7mpg_get_pkt_sz(core);
	size_t blk_size = pkt_size * core->pkt_by_blk;
	size_t map_len  = blk_size * core->blk_by_rb;
	size_t max_mappable = core->dma_region_sz;

	/*
	 * There is no much choices here. DMA is only used to received
	 * data so set direction to DEV_TO_MEM.
	 */
	core->dma_cfg.direction = DMA_DEV_TO_MEM;
	core->dma_cfg.src_addr = core->fifo_paddr;

	if (blk_size % core->dma_min) {
		dev_err(core->device,
		        "block size must be a multiple of %d (%d is not)\n",
		        core->dma_min, blk_size);
		return -EINVAL;
	}

	if (map_len > max_mappable) {
		dev_err(core->device,
		        "can't map %dB bytes (max %dB)\n",
		        map_len, max_mappable);
		return -EINVAL;
	}

	err = dmaengine_slave_config(core->dma_chan, &core->dma_cfg);
	return err;
}

static int p7mpg_run_dma(struct p7mpg_core* core,
			 void (*complete)(void*))
{
	size_t pkt_size = p7mpg_get_pkt_sz(core);
	int err;
	size_t period_len = core->pkt_by_blk * pkt_size;
	size_t map_len = period_len * core->blk_by_rb;
	struct dma_chan *chan = core->dma_chan;
	dma_addr_t buff_addr = (dma_addr_t) core->dma_bus;
	struct dma_async_tx_descriptor* const   txd =
	    chan->device->device_prep_dma_cyclic(chan,
	                                         buff_addr,
	                                         map_len,
	                                         period_len,
	                                         core->dma_cfg.direction,
	                                         NULL);

	if (! txd)
		return -ENOMEM;

	txd->callback = complete;
	txd->callback_param = core;

	core->dma_cook = dmaengine_submit(txd);
	err = dma_submit_error(core->dma_cook);
	if (err)
		return err;

	dma_async_issue_pending(core->dma_chan);
	dev_dbg(core->device, "Starting cylic DMA\n");

	return 0;
}

static int p7mpg_prepare_dma(struct p7mpg_core* core)
{
	int err = 0;

	/*
	 * Setup DMA operation properties for cyclic buffer.
	 */
	err = p7mpg_config_dma(core);
	if (err)
		return err;

	/* Submit DMA operation. */
	err = p7mpg_run_dma(core, p7mpg_complete_dma);
	return err;
}

static int p7mpg_init_dmamem(struct p7mpg_core* core,
                             struct platform_device* pdev)
{
	struct p7mpg_plat_data *pdata = pdev->dev.platform_data;
	core->dma_region_sz = pdata->dma_sz;

	core->dma_virt = dma_alloc_coherent(core->device,
	                                    core->dma_region_sz,
	                                    &core->dma_bus,
	                                    GFP_KERNEL);
	if (core->dma_virt)
		return 0;

	dev_err(core->device, "failed to reserve DMA'ble memory\n");
	return -ENOMEM;
}

static void p7mpg_release_dmamem(struct p7mpg_core *core)
{
	dma_free_coherent(core->device,
	                  core->dma_region_sz,
	                  core->dma_virt,
	                  core->dma_bus);
}

static void p7mpg_close_vmdma(struct vm_area_struct* vma)
{
	struct p7mpg_core* const  core = p7mpg_to_core(vma->vm_file->private_data);

	dev_dbg(core->device, "VMA of device p7mpg %d closed.\n",
	        p7mpg_core_id(core));
}

static int p7mpg_fault_vmdma(struct vm_area_struct* vma, struct vm_fault *vmf)
{
	return VM_FAULT_SIGBUS;
}

static struct vm_operations_struct const p7mpg_vm_ops = {
	.close = p7mpg_close_vmdma,
	.fault = p7mpg_fault_vmdma,
};

static int p7mpg_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int err;
	unsigned long start_addr;
	struct p7mpg_core* core = p7mpg_to_core(filp->private_data);
	unsigned long const   pageno = vma->vm_pgoff;
	size_t const          sz = vma->vm_end - vma->vm_start;

	if (! (vma->vm_flags & VM_MAYSHARE)) {
		dev_err(core->device,
			"invalid vma flags (0x%lx)\n",
			vma->vm_flags);
		return -EINVAL;
	}

	if (vma->vm_end <= vma->vm_start) {
		dev_err(core->device,
		        "invalid vma region: 0x%08lx-0x%08lx\n",
		        vma->vm_start, vma->vm_end);
		return -EINVAL;
	}

	switch (pageno) {
		case 0: 
		{
			/* Metadata mapping */
			start_addr = PFN_DOWN(virt_to_phys((void *)core->metadata));
			break;
		}
		case 1:
			/* DMA memory mapping */
			if (sz > core->dma_region_sz) {
				dev_err(core->device,
				        "invalid vma size %u (limited to %u)\n",
				        (unsigned int) sz, (unsigned int) core->dma_region_sz);
				return -EINVAL;
			}
			start_addr = PFN_DOWN(core->dma_bus);
			vma->vm_page_prot = pgprot_dmacoherent(vm_get_page_prot(vma->vm_flags));
			break;
		default:
			dev_dbg(core->device,
			        "invalid vma offset page %lu.\n",
			        pageno);
			return -EINVAL;
	}

	vma->vm_flags &= ~(VM_MAYEXEC | VM_EXEC);
	vma->vm_flags |= VM_SPECIAL | VM_DONTCOPY;

	err = remap_pfn_range(vma,
			      vma->vm_start,
			      start_addr,
     			      sz,
			      vma->vm_page_prot);
	if (err)
		return err;

	vma->vm_ops = &p7mpg_vm_ops;

	dev_dbg(core->device, "device p7mpegts %d mmapped.\n", p7mpg_core_id(core));
	return 0;
}

static int p7mpg_set_config(struct p7mpg_core *core, struct p7mpg_settings *config)
{
	int ctrl = 0;

	/* The IP must be disabled when configured. */
	if (__raw_readl(core->vaddr + P7MPG_CTRL) & P7MPG_CTRL_ENABLE)
		return -EBUSY;

	if (config->pkt_by_blk || config->blk_by_rb) {
		unsigned int pkt_size, pkt_by_blk, blk_size, max_blk_by_rb;

		if (config->pkt_size)
			pkt_size = config->pkt_size;
		else
			pkt_size = __raw_readl(core->vaddr + P7MPG_PKT_SIZE);

		if (config->pkt_by_blk)
			pkt_by_blk = p7mpg_set_pkt_by_blk (core,
						pkt_size * config->pkt_by_blk,
						pkt_size);
		else
			pkt_by_blk = core->pkt_by_blk;

		blk_size = pkt_size * pkt_by_blk;
		max_blk_by_rb = p7mpg_get_max_blk_nb(core, blk_size);

		if (!max_blk_by_rb)
			return -EINVAL;
		else if (config->blk_by_rb) {
			if (config->blk_by_rb > max_blk_by_rb)
				core->blk_by_rb = max_blk_by_rb;
			else
				core->blk_by_rb = config->blk_by_rb;
		} else if (core->blk_by_rb > max_blk_by_rb) {
			core->blk_by_rb = max_blk_by_rb;
		}

		core->pkt_by_blk = pkt_by_blk;
	}

	/* Set IP parameters */
	__raw_writel(config->sync_val, core->vaddr + P7MPG_SYNC_VAL);
	if (config->sync_num > 1)
		__raw_writel(config->sync_num, core->vaddr + P7MPG_SYNC_NUM);
	if (config->pkt_size)
		__raw_writel(config->pkt_size, core->vaddr + P7MPG_PKT_SIZE);
	__raw_writel(config->timing_dl, core->vaddr + P7MPG_DATA_PIPE);

	if (config->fall_sampling)
		ctrl |= P7MPG_CTRL_FALL_SAMPLING;
	if (config->lsb)
		ctrl |= P7MPG_CTRL_LSB_FIRST;
	if (config->valid_en && core->valid_pin)
		ctrl |= P7MPG_CTRL_VALID;
	if (config->sync_en && core->sync_pin)
		ctrl |= P7MPG_CTRL_SYNC;

	__raw_writel(ctrl, core->vaddr + P7MPG_CTRL);

	return 0;
}

static int p7mpg_get_config(struct p7mpg_core *core, struct p7mpg_settings *config)
{
	int ctrl = __raw_readl(core->vaddr + P7MPG_CTRL);

	config->fall_sampling = (ctrl & P7MPG_CTRL_FALL_SAMPLING) ? 1 : 0;
	config->lsb           = (ctrl & P7MPG_CTRL_LSB_FIRST)     ? 1 : 0;
	config->valid_en      = (ctrl & P7MPG_CTRL_VALID)         ? 1 : 0;
	config->sync_en       = (ctrl & P7MPG_CTRL_SYNC)          ? 1 : 0;

	config->sync_val      = __raw_readl(core->vaddr + P7MPG_SYNC_VAL);
	config->sync_num      = __raw_readl(core->vaddr + P7MPG_SYNC_NUM);
	config->pkt_size      = __raw_readl(core->vaddr + P7MPG_PKT_SIZE);
	config->timing_dl     = __raw_readl(core->vaddr + P7MPG_DATA_PIPE);

	config->pkt_by_blk    = core->pkt_by_blk;
	config->blk_by_rb     = core->blk_by_rb;

	return 0;
}

static int p7mpg_start(struct p7mpg_core *core)
{
	int ctrl;
	int err = -ENOMEM;

	core->dma_block = 0;
	p7mpg_setup_metadata(core);

	/* flush FIFO */
	__raw_writel(1, core->vaddr + P7MPG_FLUSH);
	__raw_writel(core->dma_thres / sizeof(u32), core->vaddr + P7MPG_RXFIFO_TH);

	err = p7mpg_prepare_dma(core);
	if (err)
		return err;

	ctrl = __raw_readl(core->vaddr + P7MPG_CTRL);
	ctrl |= P7MPG_CTRL_DMAC_ENABLE;
	ctrl |= P7MPG_CTRL_ENABLE;

	__raw_writel(ctrl, core->vaddr + P7MPG_CTRL);

	return 0;
}

static int p7mpg_stop(struct p7mpg_core *core)
{
	int ctrl = __raw_readl(core->vaddr + P7MPG_CTRL);

	__raw_writel(ctrl & ~(P7MPG_CTRL_ENABLE), core->vaddr + P7MPG_CTRL);

	dmaengine_terminate_all(core->dma_chan);

	return 0;
}

static int p7mpg_release(struct inode *inode, struct file *filp)
{
	struct p7mpg_core* core = p7mpg_to_core(filp->private_data);

	/* If the IP is enabled, disable it. */
	if (__raw_readl(core->vaddr + P7MPG_CTRL) & P7MPG_CTRL_ENABLE)
		p7mpg_stop(core);

	dev_dbg(core->device, "device p7mpegts %d closed.\n", core->id);
	return 0;
}

static int p7mpg_open(struct inode *inode, struct file* filp)
{
	struct p7mpg_core* core = p7mpg_to_core(filp->private_data);

	dev_dbg(core->device, "device p7mpegts %d opened.\n", core->id);
	return 0;
}

static long p7mpg_ioctl(struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct p7mpg_core* core = p7mpg_to_core(filp->private_data);
	
	if (_IOC_TYPE(cmd) != P7MPG_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > P7MPG_IOC_MAXNR) return -ENOTTY;

	switch (cmd) {
	case P7MPG_IOC_SET_CONFIG:
		ret = p7mpg_set_config(core, (struct p7mpg_settings *)arg);
		dev_dbg(core->device, "device p7mpegts %d : P7MPG_IOC_SET_CONFIG ioctl.\n", core->id);
		break;
	case P7MPG_IOC_GET_CONFIG:
		ret = p7mpg_get_config(core, (struct p7mpg_settings *)arg);
		dev_dbg(core->device, "device p7mpegts %d : P7MPG_IOC_GET_CONFIG ioctl.\n", core->id);
		break;
	case P7MPG_IOC_START:
		ret = p7mpg_start(core);
		dev_dbg(core->device, "device p7mpegts %d : P7MPG_IOC_START ioctl.\n", core->id);
		break;
	case P7MPG_IOC_STOP:
		ret = p7mpg_stop(core);
		dev_dbg(core->device, "device p7mpegts %d : P7MPG_IOC_STOP ioctl.\n", core->id);
		break;
	default:
		ret = -ENOTTY;
		break;
	}
	
	return ret;
}

static unsigned int p7mpg_poll(struct file *filep, struct poll_table_struct *wait)
{
	struct p7mpg_core* core = p7mpg_to_core(filep->private_data);
	int i;

	poll_wait(filep, &poll_wait_queue, wait);

	/* If there is one at least one block valid, return POLLIN. */
	for(i = 0; i < core->blk_by_rb; i++) {
		if (((core->metadata->blks_states >> 2 * i) & 3) == P7MPG_BLOCK_VALID)
			return POLLIN | POLLRDNORM;
	}

	return 0;
}

static struct file_operations const p7mpg_fops = {
	.owner          = THIS_MODULE,
	.open           = p7mpg_open,
	.release        = p7mpg_release,
	.mmap           = p7mpg_mmap,
	.unlocked_ioctl = p7mpg_ioctl,
	.poll           = p7mpg_poll,
};

static int p7mpg_init_core(struct p7mpg_core *core, 
                              struct platform_device* pdev)
{
	int                                 err;
	struct resource const*              res;
	dma_cap_mask_t                      mask;

#ifdef DEBUG
	BUG_ON(! core);
	BUG_ON(! pdev);
#endif

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (! res) {
		dev_err(core->device, "failed to get core memory region\n");
		return -ENXIO;
	}

	core->regs = request_mem_region(res->start,
	                                resource_size(res),
	                                dev_name(core->device));
	if (! core->regs) {
		dev_err(core->device,
		        "failed to request core memory region [%08x:%08x]\n",
				res->start,
				res->end);
		return -EBUSY;
	}

	core->vaddr = (unsigned long) ioremap(res->start, resource_size(res));
	if (! core->vaddr) {
		dev_err(core->device,
		        "failed to remap core memory region [%08x:%08x]\n",
				res->start,
				res->end);
		err = -ENXIO;
		goto release;
	}
	core->fifo_paddr = res->start + P7MPG_DATA;

	core->dma_chan = NULL;
	res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (! res) {
		dev_err(core->device, "failed to get DMA resource\n");
		err = -ENXIO;
		goto unmap;
	}

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_cap_set(DMA_CYCLIC, mask);
	core->dma_chan = dma_request_channel(mask, p7mpg_filter, (void*) res->start);
	if (! core->dma_chan) {
		dev_err(core->device,
		        "failed to request DMA channel %d\n", res->start);
		err = -EINVAL;
		goto unmap;
	}

	/*
	 * Setup DMA configuration fields remaining constant over controller
	 * life-cycle.
	 */
	core->dma_cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_8_BYTES;
	core->dma_cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_8_BYTES;

	core->pctl = pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(core->pctl)) {
		err = PTR_ERR(core->pctl);
		dev_err(&pdev->dev, "failed to select I/O pins (%d)\n", err);
		goto release_channel;
	}

	core->metadata_mem = kmalloc(2 * PAGE_SIZE, GFP_KERNEL);
	if (!core->metadata_mem) {
		err = -ENOMEM;
		goto put_pins;
	}
	core->metadata = (struct p7mpg_metadata *)PFN_ALIGN(core->metadata_mem);

#ifdef CONFIG_PM_SLEEP
	core->saved_regs = kcalloc(ARRAY_SIZE(p7mpg_save_regs),
	                           sizeof(u32), GFP_KERNEL);
	if (!core->saved_regs) {
		err = -ENOMEM;
		goto free_metadata;
	}
#endif

	return 0;

#ifdef CONFIG_PM_SLEEP
free_metadata:
	kfree(core->metadata);
#endif
put_pins:
	pinctrl_put(core->pctl);
release_channel:
	dma_release_channel(core->dma_chan);
unmap:
	iounmap((void*) core->vaddr);
release:
	release_mem_region(core->regs->start, resource_size(core->regs));
	return err;
}

static int p7mpg_request_pads(struct p7mpg_core* core,
                            struct platform_device *pdev)
{
	int err = 0;
	struct p7mpg_plat_data *pdata = pdev->dev.platform_data;
	struct p7spi_swb const* swb = pdata->swb;

	while (swb->pad != -1) {
		err = p7mpg_request_pad(core, swb->pad, swb->direction,
					swb->function);
		if (err) {
			struct p7spi_swb const* swb_rel = pdata->swb;
			while (swb_rel != swb) {
				p7mpg_release_pad(core, swb_rel->pad,
						  swb_rel->direction);
				swb_rel++;
			}

			goto finish;
		}
		swb++;
	}

finish:
	return err;
}

static void p7mpg_init_config(struct p7mpg_core* core) 
{
	struct p7mpg_settings config;

	config.fall_sampling = 0;
	config.lsb           = 0;
	config.valid_en      = 0;
	config.sync_en       = 0;
	config.sync_val      = 0x47;
	config.sync_num      = 2;
	config.pkt_size      = 188;
	config.timing_dl     = 0;
	config.pkt_by_blk    = p7mpg_set_pkt_by_blk(core, SZ_32K,
	                                            config.pkt_size);
	config.blk_by_rb     = p7mpg_get_max_blk_nb(core,
	                                            config.pkt_size *
	                                            config.pkt_by_blk);

	while (config.blk_by_rb < 4) {
		config.pkt_by_blk -= p7mpg_get_min_pkt_by_blk(config.pkt_size,
								core->dma_min);
		config.blk_by_rb     = p7mpg_get_max_blk_nb(core,
							     config.pkt_size *
							     config.pkt_by_blk);
	}

	p7mpg_set_config(core, &config);
}

static int p7mpg_add_device(struct p7mpg_core* core,
                               struct platform_device *pdev)
{
	int err;
	char *miscdev_name;
	int len = strlen(pdev->name) + 3; /* dot + id (one digit)+ terminating char */

	miscdev_name = kmalloc(len, GFP_KERNEL);
	if (! miscdev_name)
		return -ENOMEM;

	err = snprintf(miscdev_name, len,
	               "%s.%d", pdev->name, pdev->id);
	if (err < 0) {
		dev_err(core->device, "failed to set device name\n");
		goto free;
	}

	core->miscdev.minor = MISC_DYNAMIC_MINOR;
	core->miscdev.name = miscdev_name;
	core->miscdev.fops = &p7mpg_fops;

	err = misc_register(&core->miscdev);
	if (!err)
		return 0;

free:
	kfree(miscdev_name);

	return err;
}

static void p7mpg_exit_core(struct p7mpg_core *core)
{
#ifdef DEBUG
	BUG_ON(! core);
#endif

	__raw_writel(0, core->vaddr + P7MPG_CTRL);

	if (core->fifo_sz)
		p7spi_free_fifo(p7mpg_core_id(core), P7SPI_CORE_MPEG);
#ifdef CONFIG_PM_SLEEP
	kfree(core->saved_regs);
#endif
	kfree(core->metadata_mem);
	pinctrl_put(core->pctl);
	dma_release_channel(core->dma_chan);
	iounmap((void*) core->vaddr);
	release_mem_region(core->regs->start, resource_size(core->regs));
}

static void p7mpg_release_pads(struct p7mpg_core* core,
                               struct platform_device *pdev)
{
	struct p7mpg_plat_data *pdata = pdev->dev.platform_data;
	struct p7spi_swb const* swb = pdata->swb;

	while (swb->pad != -1) {
		p7mpg_release_pad(core, swb->pad, swb->direction);
		swb++;
	}
}

static void p7mpg_remove_device(struct p7mpg_core* core)
{
	misc_deregister(&core->miscdev);
	kfree(core->miscdev.name);
}

#ifdef CONFIG_PM
static int p7mpg_suspend(struct platform_device* pdev, pm_message_t state)
{
	int i;
	struct p7mpg_core* core = platform_get_drvdata(pdev);
	
	core->started = __raw_readl(core->vaddr + P7MPG_CTRL) & 
		P7MPG_CTRL_ENABLE;

	if (core->started)
		p7mpg_stop(core);

	for(i = 0; i < ARRAY_SIZE(p7mpg_save_regs); i++) {
		core->saved_regs[i] = __raw_readl(core->vaddr +
		                                  p7mpg_save_regs[i]);
	}

	return 0;
}

static int p7mpg_resume(struct platform_device* pdev)
{
	int i;
	struct p7mpg_core* core = platform_get_drvdata(pdev);

	/* Restore registers */
	for (i = 0; i < ARRAY_SIZE(p7mpg_save_regs); i++) {
		__raw_writel(core->saved_regs[i],
		             core->vaddr + p7mpg_save_regs[i]);
	}

	if (core->started)
		p7mpg_start(core);

	return 0;
}
#else

#define p7mpg_suspend    NULL
#define p7mpg_resume     NULL

#endif  /* CONFIG_PM */

static int __devinit p7mpg_probe(struct platform_device* pdev)
{
	int err;
	int dev_id = pdev->id;
	struct p7mpg_core* core = kzalloc(sizeof(*core), GFP_KERNEL);

	if (!core)
		return -ENOMEM;

	/*
	 * early set of core->id because it's going to be used
	 * in almost all init functions
	 */
	core->id = dev_id;
	core->device = &pdev->dev;

	err = p7mpg_init_core(core, pdev);
	if (err)
		return err;

	err = p7mpg_setup_core(core, pdev->dev.platform_data);
	if (err)
		goto exit_core;

	err = p7mpg_init_dmamem(core, pdev);
	if (err)
		goto exit_core;

	err = p7mpg_request_pads(core, pdev);
	if (err)
		goto release_dmamem;

	p7mpg_init_config(core);

	err = p7mpg_add_device(core, pdev);
	if (! err) {
		platform_set_drvdata(pdev, core);
		return 0;
	}

	p7mpg_release_pads(core, pdev);
release_dmamem:
	p7mpg_release_dmamem(core);
exit_core:
	p7mpg_exit_core(core);

	kfree(core);
	return err;
}

static int p7mpg_remove(struct platform_device *pdev)
{
	struct p7mpg_core* core = platform_get_drvdata(pdev);
	if (!core)
		return -EINVAL;

	p7mpg_remove_device(core);
	p7mpg_release_pads(core, pdev);
	p7mpg_release_dmamem(core);
	p7mpg_exit_core(core);
	kfree(core);
	return 0;
}

static struct platform_driver p7mpg_driver = {
	.driver = {
		.name   = P7MPG_DRV_NAME,
		.owner  = THIS_MODULE,
	},
	.probe		= p7mpg_probe,
	.remove		= __devexit_p(p7mpg_remove),
	.suspend        = p7mpg_suspend,
	.resume         = p7mpg_resume,
};

static int __init p7mpg_init(void)
{
	return platform_driver_register(&p7mpg_driver);
}

static void __exit p7mpg_exit(void)
{
	platform_driver_unregister(&p7mpg_driver);
}

module_init(p7mpg_init);
module_exit(p7mpg_exit);

MODULE_LICENSE("GPL");
