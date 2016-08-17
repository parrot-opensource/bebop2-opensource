/**
 * linux/driver/parrot/i2c/plds_i2cs.c - Parrot7 I2C slave driver
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Victor Lambret <victor.lambret.ext@parrot.com>
 * date:    18-Sep-2013
 *
 * I2C Slave driver dedicated to PLDS protocol. 
 *
 * Limitation :
 * - This driver can only run on P7R3
 *
 * This file is released under the GPL
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <asm/io.h>
#include "plds_i2cs.h"
#include "p7-i2cs_regs.h"

/* This driver implement the block protocol layer of the PLDS DVD player. It
 * allows the transmission of commands.
 *
 * The application read or write a complete command at a time using read or
 * write syscalls.
 *
 * PLDS commands are encapsulated in block loayer trames like this :
 *
 *      LEN command CRC
 *
 * LEN : the number of bytes of command + 1 for CRC
 * CRC : sum of command bytes on 8 bits
 *
 * Example : application command :     0x10 0x11 0x12
 *           block layer trame :   0x4 0x10 0x11 0x12 0x33
 *
 * TODO :   - Avoid to put CRQ down during a transfer, wait for the transfer
 *            to STOP before putting it down.
 *          - Add an mecanism to report problems to applciation with read 
 *            syscall : RX buffer overflow or premature STOP while transmitting
 *            a message.
 * */

/***************************
 * PARAMETERS
 ***************************/

#define I2CS_RX_THRESHOLD       0x7
#define I2CS_ADDR               0x70
#define I2CS_MAX_RETRIES	5

int plds_msg_size   = 128;
int plds_msg_nb     = 32;
int plds_i2cs_dbg   = 0;

module_param(plds_msg_size, int, 0);
module_param(plds_msg_nb, int, 0);
module_param(plds_i2cs_dbg, int, 0);

static DECLARE_WAIT_QUEUE_HEAD(plds_read_wait);
static DECLARE_WAIT_QUEUE_HEAD(plds_write_wait);

/***************************
 * GENERIC HELPERS
 ***************************/

#define LOG(fun, fmt, arg...) fun(&i2cs.pdev->dev, fmt "\n", ##arg)
#define LOG_INFO(fmt, arg...) LOG(dev_info, fmt, ##arg)
#define LOG_WARN(fmt, arg...) LOG(dev_warn, fmt, ##arg)
#define LOG_ERROR(fmt, arg...) LOG(dev_err, fmt, ##arg)
#define LOG_DEBUG(fmt, arg...)            \
do {                                            \
	if (plds_i2cs_dbg)                      \
		LOG(dev_info, fmt, ##arg);\
} while(0)

/***************************
 * Messages management
 ***************************/

struct plds_msg {
	__u16                    size;  /* buffer size */
	__u16                    len;   /* Number of significant bytes */
	__u8                    *buf;
	struct list_head         list;
};

struct msg_pool {
	spinlock_t              lock;
	struct plds_msg         *msgs;  /* messages descriptors */
	void                    *mem;   /* memory allocated for all messages */
	struct list_head         free;
	struct list_head         filled;
};

// TODO: merge struct plds and struct plds_i2cs
struct plds {
	struct cdev              cdev;
	unsigned int             major;
	struct class            *class;
	unsigned int             open_cnt;

	struct msg_pool          rx_pool;
	struct msg_pool          tx_pool;
} plds;

struct plds_i2cs {
	spinlock_t                       lock;
	struct platform_device          *pdev;
	const struct plds_i2cs_pdata    *pdata;
	struct resource                 *iomem;
	void __iomem                    *base;
	struct clk                      *clk;
	struct pinctrl                  *pctl;
	int                              irq;

	/* Current RX and TX messages */
	/* RX */
	int				 retry;
	int                              flush;
	struct plds_msg                 *rx_msg;

	/* TX */
	int                              send;
	struct plds_msg                 *tx_msg;
} i2cs;

/* Allocate memory for both message buffers and descriptors and initialise 
 * message descriptors*/
static int plds_init_pool(struct msg_pool *pool)
{
	int i, rt = -ENOMEM;

	pool->mem = kzalloc(plds_msg_size * plds_msg_nb, GFP_KERNEL);
	if (!pool->mem )
		return rt;

	pool->msgs = kzalloc(plds_msg_nb * sizeof(struct plds_msg), GFP_KERNEL);
	if (!pool->msgs)
		goto free_mem;

	INIT_LIST_HEAD(&pool->free);
	INIT_LIST_HEAD(&pool->filled);
	spin_lock_init(&pool->lock);
	for (i = 0; i < plds_msg_nb; i++) {
		pool->msgs[i].size = plds_msg_size;
		pool->msgs[i].buf = pool->mem + i * plds_msg_size;
		list_add_tail(&pool->msgs[i].list, &pool->free);
	};

	return 0;
free_mem:
	kfree(pool->mem);
	return rt;
}

static void plds_release_pool(struct msg_pool *pool)
{
	unsigned long flags;

	spin_lock_irqsave(&pool->lock, flags);

	INIT_LIST_HEAD(&pool->free);
	INIT_LIST_HEAD(&pool->filled);


	kfree(pool->msgs);
	kfree(pool->mem);

	spin_unlock_irqrestore(&pool->lock, flags);
}

/***************************
 * I2C Slave
 ***************************/

static int plds_i2cs_enable(void)
{
	writel(I2CS_RX_THRESHOLD, i2cs.base + I2CS_CONTROL);
	writel(I2CS_FIFO_FLUSH_TX | I2CS_FIFO_FLUSH_RX,
	       i2cs.base + I2CS_FIFO_FLUSH);
	writel(I2CS_ITEN_TX | I2CS_ITEN_RX | I2CS_ITEN_TRANSFER_DONE,
	       i2cs.base + I2CS_ITEN);
	writel(I2CS_ADDR ,i2cs.base + I2CS_ADDRESS);

	LOG_DEBUG("Enabled");

	return 0;
}

static int plds_i2cs_disable(void)
{
	LOG_DEBUG("Disabled");

	writel(0,i2cs.base + I2CS_ADDRESS);
	writel(0, i2cs.base + I2CS_ITEN);

	return 0;
}

/**************************************
 * PLDS API
 **************************************/

static u8 plds_crc(const char *buf, int n)
{
	u8 crc = 0;
	int i;

	for (i = 0; i < n; i++)
		crc += buf[i];

	return crc;
};

static inline  void plds_put_request_down(void)
{
	LOG_DEBUG("Put CRQ down");
	gpio_set_value(i2cs.pdata->gpio_request, 0);
}

static inline  void plds_put_request_up(void)
{
	LOG_DEBUG("Put CRQ up");
	gpio_set_value(i2cs.pdata->gpio_request, 1);
}

static inline void plds_reset(void)
{
	unsigned long	i2cs_flags, rx_pool_flags,
				tx_pool_flags;
	struct		plds_msg  *p_msg;

	LOG_INFO("Reset plds");

	if (gpio_is_valid(i2cs.pdata->gpio_reset)) {
		spin_lock_irqsave(&i2cs.lock, i2cs_flags);

		/* reset plds */
		gpio_set_value(i2cs.pdata->gpio_reset, 0);
		plds_put_request_up();

		/* dissable plds */
		plds_i2cs_disable();

		/* flush data */
		spin_lock_irqsave(&plds.tx_pool.lock, tx_pool_flags);
		if (i2cs.tx_msg) {
			i2cs.send   = 0;
			i2cs.retry  = 0;
			i2cs.flush  = 0;
			i2cs.tx_msg->len = 0;
			list_add_tail(&i2cs.tx_msg->list, &plds.tx_pool.free);
			i2cs.tx_msg = NULL;
		}

		list_for_each_entry(p_msg, &plds.tx_pool.filled, list) {
			p_msg->len = 0;
			list_add_tail(&p_msg->list, &plds.tx_pool.free);
		}
		spin_unlock_irqrestore(&plds.tx_pool.lock, tx_pool_flags);

		spin_lock_irqsave(&plds.rx_pool.lock, rx_pool_flags);
		if (i2cs.rx_msg) {
			i2cs.rx_msg->len = 0;
			list_add_tail(&i2cs.rx_msg->list, &plds.rx_pool.free);
			i2cs.rx_msg = NULL;
		}

		list_for_each_entry(p_msg, &plds.rx_pool.filled, list) {
			p_msg->len = 0;
			list_add_tail(&p_msg->list, &plds.rx_pool.free);
		}
		spin_unlock_irqrestore(&plds.rx_pool.lock, rx_pool_flags);

		plds_i2cs_enable();
		gpio_set_value(i2cs.pdata->gpio_reset, 1);
		mdelay(50);

		spin_unlock_irqrestore(&i2cs.lock, i2cs_flags);

		//Not mandatory
		wake_up_interruptible(&plds_read_wait);
		wake_up_interruptible(&plds_write_wait);
	} else
		LOG_ERROR("Unable to perform reset.");

}

/*  */
static int plds_wait_rx_filled_msg(void)
{
	unsigned long   flags;
	int             rt = 0;
	spin_lock_irqsave(&plds.rx_pool.lock,flags);
	if (list_empty(&plds.rx_pool.filled))
		rt = 1;
	spin_unlock_irqrestore(&plds.rx_pool.lock,flags);

	return rt;
}

static int plds_wait_tx_free_msg(void)
{
	unsigned long   flags;
	int             rt = 0;
	spin_lock_irqsave(&plds.tx_pool.lock,flags);
	if (list_empty(&plds.tx_pool.free))
		rt = 1;
	spin_unlock_irqrestore(&plds.tx_pool.lock,flags);

	return rt;
}

static int plds_count_list_entries(struct list_head *list,
		spinlock_t *plock)
{
	struct list_head *msg;
	unsigned long	flags;
	int count = 0;

	spin_lock_irqsave(plock, flags);
	list_for_each(msg, list)
		count++;
	spin_unlock_irqrestore(plock, flags);

	return count;
}

static void  plds_debug(void)
{
	u32             addr, iten, status, count;
	struct		plds_msg  *p_msg;
	unsigned long	flags;

	addr    = readl(i2cs.base + I2CS_ADDRESS);
	iten    = readl(i2cs.base + I2CS_ITEN);
	status  = readl(i2cs.base + I2CS_STATUS);

	spin_lock_irqsave(&i2cs.lock, flags);
	LOG_INFO("== GPIOS ===\n"
	         "    request=%d",
	         gpio_get_value(i2cs.pdata->gpio_request));

	if (i2cs.rx_msg) {
		LOG_INFO("[RX] received [%d/%d] size=%d flush=%d",
		         i2cs.rx_msg->len,
		         i2cs.rx_msg->buf[0],
		         i2cs.rx_msg->size,
		         i2cs.flush);

		print_hex_dump(KERN_INFO, "[RX]>",
				DUMP_PREFIX_NONE,
				16, 1, i2cs.tx_msg->buf,
				i2cs.tx_msg->len, 1);


		if (!list_empty(&plds.tx_pool.free)) {
			/* Last message sent */
			p_msg = list_entry((plds.tx_pool.free).prev,
						struct plds_msg, list);

			print_hex_dump(KERN_INFO, "[TX] (last sent)>",
					DUMP_PREFIX_NONE,
					16, 1, p_msg->buf,
					p_msg->len, 1);
		}

	} else {
		LOG_INFO("[RX] none");
	}

	if (i2cs.tx_msg) {
		LOG_INFO("[TX] send [%d/%d] size=%d",
		         i2cs.send,
		         i2cs.tx_msg->len,
		         i2cs.tx_msg->size);
	} else {
		LOG_INFO("[TX] none");
	}

	LOG_INFO("== PLDS Messages lists ===\n"
	         "     TX free=%d, filled=%d\n"
	         "     RX free=%d, filled=%d",
		plds_count_list_entries(&plds.tx_pool.free,
				&plds.tx_pool.lock),
		plds_count_list_entries(&plds.tx_pool.filled,
				&plds.tx_pool.lock),
		plds_count_list_entries(&plds.rx_pool.free,
				&plds.rx_pool.lock),
		plds_count_list_entries(&plds.rx_pool.filled,
				&plds.rx_pool.lock));

	count = 0;

	list_for_each_entry(p_msg, &plds.rx_pool.filled, list) {
		count++;

		LOG_INFO("[RX][%d] (%d bytes)>\n",
					count, p_msg->len);
		print_hex_dump(KERN_INFO, "",
				DUMP_PREFIX_NONE,
				16, 1, p_msg->buf,
				p_msg->len, 1);
	}

	count = 0;

	list_for_each_entry(p_msg, &plds.tx_pool.filled, list) {

		LOG_INFO("[TX][%d] (%d bytes)>\n",
					count, p_msg->len);
		print_hex_dump(KERN_INFO, "",
				DUMP_PREFIX_NONE,
				16, 1, p_msg->buf,
				p_msg->len, 1);
	}

	LOG_INFO("== I2CS %d DUMP REG ===\n"
	         "     address = 0x%08x\n"
	         "     iten = 0x%08x\n"
	         "     status = 0x%08x",
	         i2cs.pdata->i2c_bus, addr, iten, status);

	spin_unlock_irqrestore(&i2cs.lock, flags);
}

/**************************************
 * IRQ I2CS HELPERS
 **************************************/

static void plds_i2cs_next_txbuffer(void)
{
	BUG_ON(i2cs.tx_msg);
	BUG_ON(i2cs.send != 0);

	if (list_empty(&plds.tx_pool.filled)) {
		LOG_DEBUG("No TX buffer to transmit");
	} else {
		plds_put_request_up();
		i2cs.tx_msg = list_first_entry(&plds.tx_pool.filled,
		                               struct plds_msg,
		                               list);
		list_del(&i2cs.tx_msg->list);
		LOG_DEBUG("New TX buffer: %p",i2cs.tx_msg);
	}
}

static void plds_receive_bytes(void)
{
	u32 rx, spurious = 0;
	unsigned long i2cs_flags, rx_pool_flags;

	rx = readl(i2cs.base + I2CS_RECEIVE);
	while (!(rx & I2CS_RECEIVE_INVAL)) {
		LOG_DEBUG("rx=%x",rx);

		/* RX STOP */
		if (rx & I2CS_RECEIVE_STOP) {
			spin_lock_irqsave(&i2cs.lock, i2cs_flags);

			i2cs.flush = 0;
			if (!i2cs.rx_msg) {

				if (!list_empty(&plds.rx_pool.free))
					spurious = 1;

				spin_unlock_irqrestore(&i2cs.lock, i2cs_flags);

				if (spurious) {
					LOG_INFO("Spurious STOP %x",rx);
				} else {
					LOG_ERROR("RX message missed %x",rx);
				}
			} else {
				LOG_DEBUG("STOP RX, %d bytes received",
				          i2cs.rx_msg->len);

				spin_lock_irqsave(&plds.rx_pool.lock,
								rx_pool_flags);
				list_add_tail(&i2cs.rx_msg->list,
				              &plds.rx_pool.filled);
				spin_unlock_irqrestore(&plds.rx_pool.lock,
								rx_pool_flags);

				i2cs.rx_msg = NULL;

				spin_unlock_irqrestore(&i2cs.lock, i2cs_flags);
				wake_up_interruptible(&plds_read_wait);
			}
			goto next_byte;
		}

		/* Try to get next free buffer if needed */
		spin_lock_irqsave(&i2cs.lock, i2cs_flags);

		spin_lock_irqsave(&plds.rx_pool.lock, rx_pool_flags);
		if (!i2cs.flush
		    && (!i2cs.rx_msg)
		    && !list_empty(&plds.rx_pool.free)) {
			i2cs.rx_msg = list_first_entry(&plds.rx_pool.free,
			                               struct plds_msg,
			                               list);
			list_del(&i2cs.rx_msg->list);
			i2cs.rx_msg->len = 0;
		}
		spin_unlock_irqrestore(&plds.rx_pool.lock, rx_pool_flags);

		/* XXX : No RX buffer we flush at the moment. A better way to
		 * handle it would be to wait for a free RX buffer */
		if ((!i2cs.rx_msg) || (i2cs.rx_msg->len >= i2cs.rx_msg->size)) {
			i2cs.flush = 1;
			/* set length to 0 */
			/* i2cs.rx_msg->len = 0; */
			/* inform users */
		}

		if (!i2cs.flush) {
			LOG_DEBUG("buf[%d] = %x",i2cs.rx_msg->len, rx);
			i2cs.rx_msg->buf[i2cs.rx_msg->len] = (__u8) (rx & 0xFF);
			i2cs.rx_msg->len++;
		}

		spin_unlock_irqrestore(&i2cs.lock, i2cs_flags);
next_byte:
		rx = readl(i2cs.base + I2CS_RECEIVE);
	}
}

static void plds_transmit_bytes(void)
{
	u32     status;
	unsigned long	flags;

	spin_lock_irqsave(&i2cs.lock, flags);
	if (!i2cs.tx_msg) {
		plds_i2cs_next_txbuffer();
		if (!i2cs.tx_msg) {
			LOG_ERROR("No message to transmit");
			goto terminate_tx;
		}
		LOG_DEBUG("New buffer %p",i2cs.tx_msg);
	} else	/* retransmission of a message */
		plds_put_request_up();

	if (i2cs.send >= i2cs.tx_msg->len) {
		LOG_ERROR("TX underflow (send=%d, len=%d)",
		          i2cs.send,i2cs.tx_msg->len);
		goto terminate_tx;
	}

	/* Transmit bytes until TX FIFO is full or end of message */
	status = readl(i2cs.base + I2CS_STATUS);
	while ((~status & I2CS_STATUS_TX_FULL)
	       && (i2cs.send < i2cs.tx_msg->len)) {
		LOG_DEBUG("TX [%d/%d] = %x",
		          i2cs.send,i2cs.tx_msg->len,
		          i2cs.tx_msg->buf[i2cs.send]);
		writel(i2cs.tx_msg->buf[i2cs.send],
		       i2cs.base + I2CS_TRANSMIT);
		status = readl(i2cs.base + I2CS_STATUS);
		i2cs.send++;
		// XXX : Temporary ACK : transmit one byte at a time
		//return;
	}
	spin_unlock_irqrestore(&i2cs.lock, flags);

	return;

terminate_tx:
	spin_unlock_irqrestore(&i2cs.lock, flags);
	// XXX : To be implemented here
	LOG_ERROR("Terminate TX");
}

static irqreturn_t plds_i2cs_irq(int irq, void *dev_id)
{
	unsigned long	flags, i2cs_flags;
	u32 status = readl(i2cs.base + I2CS_STATUS);
	LOG_DEBUG("IRQ status=%x",status);

#if 1
	/* IT RX, we empty RX FIFO */
	if (status & I2CS_STATUS_IT_RX) {
		plds_receive_bytes();
		writel(I2CS_ITACK_RX, i2cs.base + I2CS_ITACK);
	}

	if( (status & I2CS_STATUS_IT_TRANSFER_DONE)
		&& (~status & I2CS_STATUS_RX_EMPTY)
		&& (~status & I2CS_STATUS_IT_RX)
		&& (~status & I2CS_STATUS_IT_TX) ){
		/* Manage I2C RX STOP when the received message
		   is multiple of the RX FIFO in size.

		   I2CS_STATUS_IT_TRANSFER_DONE is set, RX FIFO is not empty
		   and IT_RX or IT_TX are not set.

		   IT_RX has been set in previous interrupt to empty the RX FIFO
		   but there was no place left for the I2C RX STOP.
		   Then the acknoledge is made in 2nd interrupt */

		plds_receive_bytes();
	}

	/* IT TX */
	if (status & I2CS_STATUS_IT_TX) {
		plds_transmit_bytes();
		writel(I2CS_ITACK_TX, i2cs.base + I2CS_ITACK);
	}

	/* RX STOP */
	spin_lock_irqsave(&i2cs.lock, i2cs_flags);
	if (((!i2cs.tx_msg)
		|| (i2cs.tx_msg && !i2cs.send)) /* avoid retransmission case */
		&& (status & I2CS_STATUS_IT_TRANSFER_DONE)) {
		writel(I2CS_ITACK_TRANSFER_DONE,
		       i2cs.base + I2CS_ITACK);
	}

	/* TX STOP */
	if (i2cs.tx_msg && (status & I2CS_STATUS_IT_TRANSFER_DONE)) {
		/* When transfer is not complete it's a premature stop */
		if (((~status & I2CS_STATUS_TX_EMPTY)
			|| (i2cs.send < i2cs.tx_msg->len))
			&& (i2cs.retry < I2CS_MAX_RETRIES)) {

			LOG_ERROR("Premature TX STOP: transfer [%d/%d] FIFO TX:%lu (%04x)",
			          i2cs.send,
			          i2cs.tx_msg->len,
			          (~status & I2CS_STATUS_TX_EMPTY),
				  status);

			/* Previous transfer was a failure, try again */
			/* TODO : Add a try counter here ? */
			writel(I2CS_FIFO_FLUSH_TX,
			       i2cs.base + I2CS_FIFO_FLUSH);
			i2cs.send = 0;
			i2cs.retry++;

			print_hex_dump(KERN_ERR, "TX>", DUMP_PREFIX_NONE,
					16, 1, i2cs.tx_msg->buf,
					i2cs.tx_msg->len, 0);
		} else {
			/* Message correctly transmitted
			   or message dropped */
			if (i2cs.retry >= I2CS_MAX_RETRIES)
				LOG_ERROR("Message dropped\n");

			i2cs.send = 0;
			i2cs.retry = 0;
			spin_lock_irqsave(&plds.tx_pool.lock, flags);
			list_add_tail(&i2cs.tx_msg->list, &plds.tx_pool.free);
			spin_unlock_irqrestore(&plds.tx_pool.lock, flags);

			i2cs.tx_msg = NULL;
			wake_up_interruptible(&plds_write_wait);
		}
		writel(I2CS_ITACK_TRANSFER_DONE,
		       i2cs.base + I2CS_ITACK);

		/* Request again if there is still messages to transmit */
		spin_lock_irqsave(&plds.tx_pool.lock, flags);
		if (!list_empty(&plds.tx_pool.filled) ||
			/* retransmission of a message */
			(list_empty(&plds.tx_pool.filled)
				&& i2cs.tx_msg
				&& !i2cs.send) )
			plds_put_request_down();
		spin_unlock_irqrestore(&plds.tx_pool.lock, flags);
	}

	spin_unlock_irqrestore(&i2cs.lock, i2cs_flags);

#endif
	return IRQ_HANDLED;
}

/*****************************************
 * DEV INTERFACE
 ****************************************/

/* read : Receive an complete application command.
 *
 * ERRORS
 *      no error    : command size
 *      EAGAIN      : no command available
 *      ENOBUFS     : command is larger than buffer size
 *      EMSGSIZE    : bad frame (length error)
 *      EBADMSG     : bad frame (CRC error)
 *      ERESTARTSYS : call interrupted by signal
 */
static ssize_t plds_read(struct file *file, char __user *buf, size_t count,
                              loff_t *offset)
{
	unsigned long            flags;
	struct plds_msg         *msg;
	int                      rt;
	u8                       len, crc, crc_msg;

	LOG_DEBUG("%s",__func__);

	/* Wait for an incoming frame to be enqueued */
	spin_lock_irqsave(&plds.rx_pool.lock,flags);

	if (list_empty(&plds.rx_pool.filled)) {
		if (file->f_flags & O_NONBLOCK) {
			rt = -EAGAIN;
			goto unlock;
		}
		else {
			spin_unlock_irqrestore(&plds.rx_pool.lock,flags);
			wait_event_interruptible(plds_read_wait,
			                         plds_wait_rx_filled_msg());
			spin_lock_irqsave(&plds.rx_pool.lock,flags);
			if (signal_pending(current)) {
				rt = -ERESTARTSYS;
				goto unlock;
			}
		}
	}

	msg = list_first_entry(&plds.rx_pool.filled,
	                       struct plds_msg,
	                       list);

	if (msg->len - 1 > count) {
		rt = -ENOBUFS;
		goto unlock;
	}

	list_del(&msg->list);

	len = msg->buf[0];
	if (len != msg->len -1 || len == 1) {
		LOG_DEBUG("read: message len error (len=%d, buf_len=%d)",
		          len, msg->len);
		rt = EMSGSIZE;
		goto requeue_buf;
	}

	crc_msg = msg->buf[len];
	crc = plds_crc(msg->buf + 1, len-1);
	if (crc != crc_msg) {
		LOG_ERROR("read: message len error (crc=%x, crc_msg=%x)",
		          crc, crc_msg);
		rt = EMSGSIZE;
		goto requeue_buf;
	}

	//rt = copy_to_user(msg->buf+1, buf, len-1);
	rt = copy_to_user(buf, msg->buf+1, len-1);
	if (rt) {
		// TODO : Message is lost, requeue it in filled queue 
		LOG_DEBUG("read: copy to user error: %d",rt);
		rt = -EFAULT;
		goto requeue_buf;
	}
	rt = len - 1;

requeue_buf:
	list_add_tail(&msg->list, &plds.rx_pool.free);
unlock:
	spin_unlock_irqrestore(&plds.rx_pool.lock,flags);
	return rt;

}

/* write : Send a complete command to dvd player
 *
 * ERRORS
 *      no error    : command size
 *      EAGAIN      : PLDS block busy with a previous command
 *      EMSGSIZE    : command too large for internal buffer
 *      ERESTARTSYS : call interrupted by signal
 */
static ssize_t plds_write(struct file *file, const char __user *buf,
                               size_t count, loff_t *offset)
{
	int                      rt;
	unsigned long            flags, i2cs_flags;
	struct plds_msg         *msg;

	LOG_DEBUG("write %d",count);

	if (count + 2 > plds_msg_size)
		return -EMSGSIZE;

	spin_lock_irqsave(&plds.tx_pool.lock,flags);
	if (list_empty(&plds.tx_pool.free)) {
		if (file->f_flags & O_NONBLOCK) {
			rt = -EAGAIN;
			goto unlock;
		}
		else {
			spin_unlock_irqrestore(&plds.tx_pool.lock,flags);
			wait_event_interruptible(plds_write_wait,
			                         plds_wait_tx_free_msg());
			spin_lock_irqsave(&plds.tx_pool.lock,flags);
			if (signal_pending(current)) {
				rt = -ERESTARTSYS;
				goto unlock;
			}
		}
	}

	msg = list_first_entry(&plds.tx_pool.free,
	                       struct plds_msg,
	                       list);
	list_del(&msg->list);

	/* Construct the PLDS command LEN + DATAS + CRC in message */
	rt = copy_from_user(msg->buf+1, buf, count);
	if (rt) {
		list_add_tail(&msg->list, &plds.tx_pool.free);
		goto unlock;
	}
	msg->buf[0] = count + 1;
	msg->buf[count + 1] = plds_crc(buf, count);

	msg->len = count + 2;

	/* Put CRQ down when not transfer in progress and no previous data */
	spin_lock_irqsave(&i2cs.lock, i2cs_flags);
	if ((list_empty(&plds.tx_pool.filled)) && (!i2cs.tx_msg))
		plds_put_request_down();
	spin_unlock_irqrestore(&i2cs.lock, i2cs_flags);

	list_add_tail(&msg->list, &plds.tx_pool.filled);
	rt = count;
unlock:
	spin_unlock_irqrestore(&plds.tx_pool.lock,flags);

	return rt;
}

/* ioctl
 *
 * PLDS_SET_WBUF: Set the write buffer size. returns 0 if success or
 *                     ENOMEM for errors.
 * PLDS_SET_RBUF: Set the read buffer size. returns 0 if success or
 *                     ENOMEM for errors.
 */
static long plds_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	LOG_DEBUG("%s:cmd=%d",__func__,cmd);

	switch (cmd) {
	case PLDS_DUMP_DEBUG:
		plds_debug();
		break;
	case PLDS_RESET:
		plds_reset();
		break;
	case PLDS_FLUSH_ALL:
		LOG_DEBUG("flush FIFOS... not implemented !");
		return -ENOTTY;
	default:
		return -ENOTTY;
	}

	return 0;
}

static int plds_open(struct inode *inode, struct file *file)
{
	int ret;

	if (plds.open_cnt > 0) {
		plds.open_cnt++;
		LOG_DEBUG("Already Opened");
		return 0;
	}

	ret= plds_init_pool(&plds.rx_pool);
	if (ret)
		return ret;

	ret = plds_init_pool(&plds.tx_pool);
	if (ret)
		goto release_rx_pool;

	ret = plds_i2cs_enable();
	if (ret)
		goto release_tx_pool;

	LOG_DEBUG("Opened");

	plds.open_cnt++;

	return 0;

release_tx_pool:
	plds_release_pool(&plds.tx_pool);
release_rx_pool:
	plds_release_pool(&plds.rx_pool);

	LOG_DEBUG("Cant open, error %d", ret);
	return ret;
}

static int plds_release(struct inode *inode, struct file *file)
{
	unsigned long flags;

	plds.open_cnt--;
	if (plds.open_cnt > 0) {
		LOG_DEBUG("Keep opened");
		return 0;
	}

	plds_i2cs_disable();
	plds_put_request_up();

	spin_lock_irqsave(&i2cs.lock, flags);
	i2cs.send   = 0;
	i2cs.retry  = 0;
	i2cs.flush  = 0;

	/* avoid memory leak */
	if (i2cs.tx_msg) {
		list_add_tail(&i2cs.tx_msg->list, &plds.tx_pool.free);
		i2cs.tx_msg = NULL;
	}

	if (i2cs.rx_msg) {
		list_add_tail(&i2cs.rx_msg->list, &plds.rx_pool.free);
		i2cs.rx_msg = NULL;
	}
	spin_unlock_irqrestore(&i2cs.lock, flags);

	plds_release_pool(&plds.tx_pool);
	plds_release_pool(&plds.rx_pool);

	LOG_DEBUG("Release");
	return 0;
}

static unsigned int plds_poll(struct file *file, poll_table * wait)
{
	unsigned long flags;
	unsigned int mask;

	poll_wait(file, &plds_read_wait, wait);
	poll_wait(file, &plds_write_wait, wait);
	mask = 0;

	spin_lock_irqsave(&plds.rx_pool.lock,flags);
	if (!list_empty(&plds.rx_pool.filled))
		mask |= POLLIN | POLLRDNORM;
	spin_unlock_irqrestore(&plds.rx_pool.lock,flags);

	spin_lock_irqsave(&plds.tx_pool.lock,flags);
	if (!list_empty(&plds.tx_pool.free))
		mask |= POLLOUT | POLLWRNORM;
	spin_unlock_irqrestore(&plds.tx_pool.lock,flags);

	return mask;
}

static const struct file_operations plds_fops = {
	.owner          = THIS_MODULE,
	.llseek         = no_llseek,
	.read           = plds_read,
	.write          = plds_write,
	.unlocked_ioctl = plds_ioctl,
	.open           = plds_open,
	.release        = plds_release,
	.poll           = plds_poll,
};

static int plds_create_dev(void)
{
	int                      rt;
	dev_t                    devt;
	struct device           *device;

	rt = alloc_chrdev_region(&devt, 0, 1, PLDS_I2CS_NAME);
	if (rt) {
		LOG_ERROR("cant request region");
		return rt;
	}

	plds.major = MAJOR(devt);

	plds.class = class_create(THIS_MODULE, PLDS_I2CS_NAME);
	if (IS_ERR(plds.class)) {
		LOG_ERROR("cant create class");
		rt = PTR_ERR(plds.class);
		goto unregister;
	}

	cdev_init(&plds.cdev, &plds_fops);
	plds.cdev.owner = THIS_MODULE;

	rt = cdev_add(&plds.cdev, devt,1);
	if (rt) {
		LOG_ERROR("cant add char device");
		goto destroy_class;
	}

	device = device_create(plds.class, NULL, /* no parent device */
	                       devt, NULL, /* no additional data */
	                       PLDS_I2CS_NAME);

	if (IS_ERR(device)) {
		LOG_ERROR("cant create device");
		rt = PTR_ERR(device);
		goto del_cdev;
	}

	plds.open_cnt = 0;

	LOG_INFO("PLDS driver registred");

	return 0;

del_cdev:
	cdev_del(&plds.cdev);
destroy_class:
	class_destroy(plds.class);
unregister:
	unregister_chrdev_region(devt,1);

	return rt;
}

static int plds_destroy_dev(void)
{
	dev_t            devt   = MKDEV(plds.major, 0);

	device_destroy(plds.class, devt);
	cdev_del(&plds.cdev);
	class_destroy(plds.class);
	unregister_chrdev_region(devt,1);

	pr_info("plds freed\n");

	return 0;
}

/*****************************************
 * MODULE INTERFACE
 ****************************************/

static int __devinit plds_i2cs_probe(struct platform_device *pdev)
{
	struct resource *res;
	int              ret;
	char             clk_name[16];

	BUG_ON(!pdev->dev.platform_data);

	i2cs.retry = 0;
	i2cs.pdev  = pdev;
	i2cs.pdata = pdev->dev.platform_data;
	spin_lock_init(&i2cs.lock);

	/* P7R1 I2C Slave is too limited to be used */
	if (i2cs.pdata->revision != I2CS_REVISION_3) {
		LOG_ERROR("PLDS I2C Slave cant work on P7R1 or P7R2");
		return -EINVAL;
	}

	sprintf(clk_name, "p7-i2cs.%d", pdev->id );
	i2cs.clk = clk_get_sys(clk_name, NULL);
	if (IS_ERR(i2cs.clk)) {
		LOG_ERROR("cant get clock %s", clk_name);
		return PTR_ERR(i2cs.clk);
	}

	ret = clk_prepare_enable(i2cs.clk);
	if (ret) {
		LOG_ERROR("cant prepare clock");
		goto put_clk;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		LOG_ERROR("cant get iomem resource");
		ret = -EINVAL;
		goto clk_disable;
	}

	i2cs.iomem = request_mem_region(res->start,
	                                 resource_size(res),
	                                 pdev->name);
	if (!i2cs.iomem) {
		LOG_ERROR("cant request mem region");
		ret = -EBUSY;
		goto clk_disable;
	}

	i2cs.base = ioremap(res->start, resource_size(res));
	if (!i2cs.base) {
		LOG_ERROR("cant ioremap");
		ret = -ENOMEM;
		goto release;
	}

	i2cs.irq = platform_get_irq(pdev, 0);

	if (i2cs.irq < 0) {
		LOG_ERROR("cant get irq");
		ret = -EINVAL;
		goto unmap;
	}

	i2cs.pctl = pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(i2cs.pctl)) {
		LOG_ERROR("cant get pins");
		ret = PTR_ERR(i2cs.pctl);
		goto unmap;
	}

	ret = request_irq(i2cs.irq, plds_i2cs_irq, 0, pdev->name, &i2cs);
	if (ret) {
		LOG_ERROR("cant request irq");
		goto put_pin;
	}

	ret = gpio_request_one(i2cs.pdata->gpio_request,
	                      GPIOF_OUT_INIT_HIGH,
	                      PLDS_I2CS_NAME);
	if (ret) {
		LOG_ERROR("cant request gpio %d for CRQREQ",
		          i2cs.pdata->gpio_request);
		goto free_irq;
	}

	platform_set_drvdata(pdev, &i2cs);

	/* XXX : Adjust here plds_msg_size for message excessing 256 bytes */
	ret = plds_create_dev();
	if (ret) {
		goto free_request;
	}

	LOG_INFO("I2CS driver initialized [id=%d]", i2cs.pdata->i2c_bus);

	return 0;

free_request:
	gpio_free(i2cs.pdata->gpio_request);
free_irq:
	free_irq(i2cs.irq, &i2cs);
put_pin:
	pinctrl_put(i2cs.pctl);
unmap:
	iounmap(i2cs.base);
release:
	release_mem_region(i2cs.iomem->start, resource_size(i2cs.iomem));
clk_disable:
	clk_disable_unprepare(i2cs.clk);
put_clk:
	clk_put(i2cs.clk);
	return ret;
}

static int __devexit plds_i2cs_remove(struct platform_device *pdev)
{
	/* Turn PLDS DVD player off before */

	plds_destroy_dev();
	gpio_free(i2cs.pdata->gpio_request);
	free_irq(i2cs.irq, &i2cs);
	pinctrl_put(i2cs.pctl);
	iounmap(i2cs.base);
	release_mem_region(i2cs.iomem->start, resource_size(i2cs.iomem));
	clk_disable_unprepare(i2cs.clk);
	clk_put(i2cs.clk);

	return 0;
}

static int plds_i2cs_suspend(struct platform_device *pdev, pm_message_t state)
{
	clk_disable_unprepare(i2cs.clk);

	return 0;
}

static int plds_i2cs_resume(struct platform_device *pdev)
{
	/* need reopen from userspace ATM */
	clk_prepare_enable(i2cs.clk);

	return 0;
}

/**
 * platform driver structure
 */
static struct platform_driver plds_i2cs_driver = {
	.probe          = plds_i2cs_probe,
	.remove         = plds_i2cs_remove,
	.suspend        = plds_i2cs_suspend,
	.resume	        = plds_i2cs_resume,
	.driver         = {
		.owner  = THIS_MODULE,
		.name   = PLDS_I2CS_NAME,
	},
};
module_platform_driver(plds_i2cs_driver);

MODULE_AUTHOR("Victor Lambret <victor.lambret.ext@parrot.com>");
MODULE_DESCRIPTION("Parrot7 PLDS block protococal I2C Slave based driver");
MODULE_LICENSE("GPL");
