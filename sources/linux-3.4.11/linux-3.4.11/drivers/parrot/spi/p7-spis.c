/**
 * linux/drivers/parrot/spi/p7-spis.c - Parrot7 SPI slave controller driver
 *                                      implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * @author: alvaro.moran@parrot.com
 * @author: Gregor Boirie <gregor.boirie@parrot.com>
 * @author: Damien Riegel <damien.riegel.ext@parrot.com>
 * @date:   2012-10-13
 *
 * This file is released under the GPL
 *
 * Notes:
 * ------
 * To see implementation details, check also 7-spim.c.
 * 
 * DMA cyclic:
 *   One of the requirements of this driver is to handle high speed transfer
 * rate, up to 60Mbits/s. In slave mode, we don't have any control on the
 * clock and the FIFO is quite short (as few as 16 words). Using DMA in a
 * typical way means that you only have the depth of the FIFO to prepare the
 * next DMA request. We just can't do that with such a high transfer rate. The
 * solution chosen here is to use DMA in cyclic mode. In this mode, the DMAC is
 * given a circular buffer split into periods. At the end of each period, a
 * completion handler is called but the DMA keeps transfering. Because the
 * circular buffer length is configurable, time constraints are less severe.
 * 
 * Instrumentation:
 *   Enabling SPI slave instrumentation allows to measure the time, for a
 * period, between the DMA completion handler call and the end of the copy.
 * It is the time for scheduling (from tasklet to kthread) + the time to 
 * make a copy (from circular buffer to SPI transfer buffer).
 * So to get a significant measure, you must transfer one SPI message that is
 * n-period long to get n values. You'll get min, max and mean.
 * 
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/dmaengine.h>
#include <asm/io.h>
#include "p7-spi.h"
#include "p7-spi_priv.h"
#include "p7-spis.h"


static unsigned int period_len = 0;
module_param(period_len, uint, 0644);
MODULE_PARM_DESC(period_len, "period length in bytes");

static unsigned int nb_periods = 0;
module_param(nb_periods, uint, 0644);
MODULE_PARM_DESC(nb_periods, "number of periods");


enum p7spis_period_state {
	P7SPIS_PERIOD_FILLED,     /* buffer is ready for copy */
	P7SPIS_PERIOD_CONSUMED,   /* buffer has been read */
	P7SPIS_PERIOD_PREP,       /* DMA xfer in progress */
};

struct p7spis_period_desc {
	char*                       buf;
	size_t                      pos;

	enum p7spis_period_state    state;

	spinlock_t                  lock;
};


/*
 * Slave core instance internal state
 */
struct p7spis_core {
	/* Common SPI core instance */
	struct p7spi_core       core;
	spinlock_t              lock;
	wait_queue_head_t       wq;

	char*                   circ_buf;
	size_t                  circ_buf_len;
	size_t                  circ_buf_period_len;

	struct p7spis_period_desc *periods;
	unsigned int            nb_periods;
	struct p7spis_period_desc *circ_head;
	struct p7spis_period_desc *circ_tail;

	struct list_head        pending_messages;
	bool                    in_tasklet;
	bool                    paused;
	u32                     ctrl_reg;
};



/*
 * Given a p7spis_core and a period descriptor, returns the following
 * period descriptor, abstracting the way they are stored.
 */
static struct p7spis_period_desc* p7spis_get_next_period_desc(
	                                      struct p7spis_core* ctrl,
	                                      struct p7spis_period_desc* desc)
{
	int index = desc - ctrl->periods;
	return &ctrl->periods[(index + 1) % ctrl->nb_periods];
}


static size_t p7spis_copy_period(struct spi_transfer *xfer,
                                 size_t *rem_bytes,
                                 struct p7spis_period_desc *period,
                                 size_t period_len)
{
	size_t copy;
	void *xfer_addr;

	/*
	 * rem_bytes is the number of bytes to be
	 * transfered to complete spi_transfer copy
	 */
	xfer_addr = xfer->rx_buf + xfer->len - *rem_bytes;
	copy = min(*rem_bytes, period_len - period->pos);
	memcpy(xfer_addr, &period->buf[period->pos], copy);
	*rem_bytes -= copy;

	return copy;
}


/*
 * This function pushes the head if it has been consumed.
 */
static void p7spis_head_consumed(struct p7spis_core *ctrl,
                                 struct p7spis_period_desc *head,
                                 size_t bytes_copied)
{
	head->pos += bytes_copied;
	if (head->pos == ctrl->circ_buf_period_len) {
		head->state = P7SPIS_PERIOD_CONSUMED;
		ctrl->circ_head = p7spis_get_next_period_desc(ctrl, head);
	}
}

/*
 * Flush SPI slave circular buffer content.
 */
void p7spis_flush(struct spi_master *master)
{
	struct p7spis_core * const ctrl =
					spi_master_get_devdata(master);

	spin_lock_bh(&ctrl->lock);
	while (ctrl->circ_head != ctrl->circ_tail) {
		ctrl->circ_head->state = P7SPIS_PERIOD_CONSUMED;
		ctrl->circ_head = p7spis_get_next_period_desc(ctrl, ctrl->circ_head);
	}
	spin_unlock_bh(&ctrl->lock);
}
EXPORT_SYMBOL(p7spis_flush);


/*
 * This completion handler is called in a tasklet context everytime a period
 * completes.
 *
 * The period is first synced, making data valid and consistent when read by
 * CPU. The tail is then moved to the following period. Note that at this time,
 * DMA can already have started transfering to the new tail.
 * To handle that case, we set the period following the new tail as PREP (owned
 * by DMA).
 * If the head was pointing to this period, move it one period forward to
 * prevent from pointing to a period where we can't read data.
 */
static void p7spis_complete_dma_cyclic(void* arg)
{
	struct p7spis_core* const   ctrl = arg;
	struct p7spi_core* const    core = &ctrl->core;
	struct p7spis_period_desc *desc;


	spin_lock(&ctrl->lock);
	ctrl->in_tasklet = true;
	dma_sync_single_for_cpu(core->dma_chan->device->dev,
			core->dma_addr + (ctrl->circ_tail->buf - ctrl->circ_buf),
			ctrl->circ_buf_period_len,
			core->dma_cfg.direction);
	ctrl->circ_tail->state = P7SPIS_PERIOD_FILLED;

	/* Move tail to the next period */
	ctrl->circ_tail = p7spis_get_next_period_desc(ctrl, ctrl->circ_tail);

	/*
	 * When this completion handler is called, the buffer just after the one
	 * that triggered the completion may already be receiving DMA transfers.
	 * That's why we get the next period and set it as busy (write in progress
	 * by DMA).
	 */
	desc = p7spis_get_next_period_desc(ctrl, ctrl->circ_tail);

	/*
	 * push head forward if necessary to prevent
	 * reader from reading out of date data
	 */
	if (ctrl->circ_head == desc) {
		if (desc->state == P7SPIS_PERIOD_FILLED)
			dev_warn(p7spi_core_dev(core),
					"overwritting a non-consumed buffer\n");
		ctrl->circ_head = p7spis_get_next_period_desc(ctrl, desc);
	}

	if (desc->state == P7SPIS_PERIOD_FILLED ||
	    desc->state == P7SPIS_PERIOD_CONSUMED) {
		dma_sync_single_for_device(core->dma_chan->device->dev,
				core->dma_addr + (desc->buf - ctrl->circ_buf),
				ctrl->circ_buf_period_len,
				core->dma_cfg.direction);
		desc->pos = 0;

	}
	desc->state = P7SPIS_PERIOD_PREP;

	/*
	 * Alright ! We do the copy in tasklet if there are pending spi
	 * messages.
	 */
	if (! list_empty(&ctrl->pending_messages)) {
		struct spi_message  *mesg, *next;
		struct spi_transfer *xfer;

		list_for_each_entry_safe(mesg, next, &ctrl->pending_messages, queue) {
			size_t bytes_copied = 0;

			list_for_each_entry(xfer, &mesg->transfers, transfer_list) {
				size_t bytes;

				/* The copy may have been started in p7spis_transfer. Get to
				 * the first spi_transfer that is not complete */
				if (mesg->actual_length >= (xfer->len + bytes_copied)) {
					bytes_copied += xfer->len;
					continue;
				}

				bytes = xfer->len - (mesg->actual_length - bytes_copied);
				while (bytes) {
					struct p7spis_period_desc *head = ctrl->circ_head;

					if (head->state == P7SPIS_PERIOD_FILLED) {
						size_t copy = p7spis_copy_period(xfer, &bytes,
						                    head, ctrl->circ_buf_period_len);

						p7spis_head_consumed(ctrl, head, copy);
						mesg->actual_length += copy;
						bytes_copied += copy;
					}
					else {
						spin_unlock(&ctrl->lock);
						ctrl->in_tasklet = false;
						return;
					}
				}
			}

			list_del_init(&mesg->queue);
			spin_unlock(&ctrl->lock);
			mesg->state = NULL;
			mesg->status = 0;
			/* XXX complete can call spi_async and
			   call p7spis_transfer, that will change the msg list
			   we are reading...
			 */
			mesg->complete(mesg->context);
			ctrl->in_tasklet = false;
			return;
//			spin_lock(&ctrl->lock);
		}
	}
	
	ctrl->in_tasklet = false;
	spin_unlock(&ctrl->lock);
}


/*
 * Config DMA. Basically, we force direction to DEV_TO_MEM and the source
 * address as the fifo address, whatever settings are in p7spis_core.
 */
static int p7spis_config_dma_cyclic(struct p7spis_core *ctrl)
{
	int err;
	struct p7spi_core *core = &ctrl->core;

	core->dma_cfg.direction = DMA_DEV_TO_MEM;
	core->dma_cfg.src_addr = core->fifo_paddr;

	err = dmaengine_slave_config(core->dma_chan, &core->dma_cfg);
	return err;
}


/*
 * Map circular buffer.
 */
static ssize_t p7spis_map_dma_cyclic(struct p7spis_core* ctrl)
{
	struct p7spi_core *core = &ctrl->core;

#ifdef DEBUG
	BUG_ON(! p7spi_kern.dev);
	BUG_ON(core->dma_cfg.direction != DMA_DEV_TO_MEM);
	BUG_ON(core->dma_cfg.src_addr != core->fifo_paddr);
	BUG_ON(! ctrl->circ_buf_len);
	BUG_ON(! IS_ALIGNED((unsigned long) ctrl->circ_buf, dma_get_cache_alignment()));
	BUG_ON(! IS_ALIGNED(ctrl->circ_buf_len, dma_get_cache_alignment()));
	BUG_ON(ctrl->circ_buf_len % core->dma_thres);
#endif

	BUG_ON(! virt_addr_valid(ctrl->circ_buf));
	BUG_ON(! virt_addr_valid(ctrl->circ_buf + ctrl->circ_buf_len - 1));

	core->dma_addr = dma_map_single(core->dma_chan->device->dev,
									ctrl->circ_buf,
									ctrl->circ_buf_len,
									core->dma_cfg.direction);
	if (unlikely(dma_mapping_error(core->dma_chan->device->dev,
								   core->dma_addr)))
		return -ENOBUFS;

	return ctrl->circ_buf_len;
}


/*
 * Unmap circular buffer.
 */
static inline void p7spis_unmap_dma_cyclic(struct p7spis_core* ctrl)
{
	struct p7spi_core* const    core = &ctrl->core;

	dma_unmap_single(core->dma_chan->device->dev,
					 core->dma_addr,
					 ctrl->circ_buf_len,
					 core->dma_cfg.direction);
}


/*
 * Starts the DMA cyclic operation.
 */
static int p7spis_run_dma_cyclic(struct p7spis_core* ctrl,
								void (*complete)(void*))
{
	int                                     err;
	struct p7spi_core *core = &ctrl->core;
	struct dma_chan *chan = core->dma_chan;
	struct dma_async_tx_descriptor* const   txd =
	    chan->device->device_prep_dma_cyclic(chan,
	                                         core->dma_addr,
	                                         ctrl->circ_buf_len,
	                                         ctrl->circ_buf_period_len,
	                                         core->dma_cfg.direction,
	                                         NULL);

	if (! txd)
		return -ENOMEM;

	txd->callback = complete;
	txd->callback_param = ctrl;

	core->dma_cook = dmaengine_submit(txd);
	err = dma_submit_error(core->dma_cook);
	if (err)
		return err;

	dma_async_issue_pending(core->dma_chan);
	dev_dbg(p7spi_core_dev(core), "Starting cylic DMA\n");

	return 0;
}


static int p7spis_prepare_dma_cylic(struct p7spis_core* ctrl)
{
	int err = 0;
	ssize_t mapped;

	/*
	 * Setup DMA operation properties for cyclic buffer.
	 */
	err = p7spis_config_dma_cyclic(ctrl);
	if (err)
		return err;

	p7spi_cook_dmaif(&ctrl->core);

	/*
	 * Map circular buffer.
	 */
	mapped = p7spis_map_dma_cyclic(ctrl);
	if (mapped < 0)
		return mapped;

	/* Submit DMA operation. */
	err = p7spis_run_dma_cyclic(ctrl, p7spis_complete_dma_cyclic);

	if (err) {
		/* unmap only if an error occured */
		p7spis_unmap_dma_cyclic(ctrl);
		return err;
	}

	return 0;
}

int p7spis_transfer(struct spi_device *spi, struct spi_message *mesg)
{
	struct spi_master           *slave = spi->master;
	struct p7spis_core* const   ctrl = spi_master_get_devdata(slave);
	struct p7spi_core* const    core = &ctrl->core;
	struct spi_transfer*        xfer;
	int err;


	err = 0;
	mesg->actual_length = 0;

	list_for_each_entry(xfer, &mesg->transfers, transfer_list) {
		size_t bytes;

		bytes = xfer->len;
		if (bytes < ctrl->circ_buf_period_len) {
			/* we need each transfert to be at least circ_buf_period_len,
			   otherwise, we can have recursive spi_async that deadlock :
			   - dma_cyclic call complete
			   - complete call spi_async (p7spis_transfer)
			     there is still data, we get data and call complete
			   - complete call spi_async that deadlock on spinlock
			  */
			dev_err(p7spi_core_dev(core), "spi message is too short %d."
					"It should be bigger than %d\n",
					bytes, ctrl->circ_buf_period_len);
			err = -EINVAL;
			goto ret;
		}
		if (bytes % ctrl->circ_buf_period_len) {
			dev_err(p7spi_core_dev(core), "spi message size (%d) need to be"
					"multiple of %d\n",
					bytes, ctrl->circ_buf_period_len);
			err = -EINVAL;
			goto ret;
		}
		while (bytes) {
			struct p7spis_period_desc *head;

			spin_lock(&ctrl->lock);
			if (ctrl->paused) {
				spin_unlock(&ctrl->lock);
				err = -EBUSY;
				goto ret;
			}
			head = ctrl->circ_head;

			if (head->state == P7SPIS_PERIOD_PREP || ctrl->in_tasklet ||
			    !list_empty(&ctrl->pending_messages)) {
				/* This function may be called from tasklet
				 * with the following flow:
				 * - We were in tasklet
				 *      -> called spi message completion
				 *      -> in msg completion, message is resubmitted (spy_async)
				 *      -> here we are.
				 * - If there are pending messages, we don't to steal their data,
				 *   go in queue
				 * - If the period is not ready yet, go in queue as well
				 */
				list_add_tail(&mesg->queue, &ctrl->pending_messages);
				spin_unlock(&ctrl->lock);
				err = 0;
				goto ret;
			}
			else if (head->state == P7SPIS_PERIOD_FILLED) {
				size_t copy;

				/* make the copy without holding the lock */
				spin_unlock(&ctrl->lock);

				copy = p7spis_copy_period(xfer, &bytes, head,
				                                 ctrl->circ_buf_period_len);

				/* check if head has changed */
				spin_lock(&ctrl->lock);
				if (head != ctrl->circ_head) {
					/* damn ! DMA completion handler pushed the head forward,
					 * It means that it wants this buffer for DMA operation and
					 * that this buffer is invalid
					 */
					spin_unlock(&ctrl->lock);
					err = -EINVAL;
					dev_err(p7spi_core_dev(core),
							"head has changed, message is invalid\n");
					goto finalize_message;
				}

				p7spis_head_consumed(ctrl, head, copy);
				spin_unlock(&ctrl->lock);

				mesg->actual_length += copy;
				dev_dbg(p7spi_core_dev(core),
					"xfered %dB from buffer [%p] to spi_transfer [%p]\n",
					copy, &head->buf[head->pos - copy],
					xfer->rx_buf + xfer->len - bytes - copy);
			}
			else {
				/*
				 * We shouldn't be here.
				 */
				enum p7spis_period_state state = head->state;
				spin_unlock(&ctrl->lock);
				dev_err(p7spi_core_dev(core),
						"Unexpected state value: 0x%X\n", state);
				BUG();
			}
		}

	}
finalize_message:
	mesg->state = NULL;
	mesg->status = err;
	mesg->complete(mesg->context);
ret:
	return err;
}


static void p7spis_clean_circ_buffer(struct p7spis_core *ctrl)
{
	if (ctrl->periods)
		kfree(ctrl->periods);
	if (ctrl->circ_buf)
		kfree(ctrl->circ_buf);

	ctrl->circ_buf = NULL;
}

static int p7spis_setup_circ_buffer(struct p7spis_core *ctrl,
									struct p7spis_ctrl_data const * const cdata)
{
	struct p7spi_core* const core = &ctrl->core;

	int i;
	int err = 0;
	struct p7spis_period_desc *p_desc;
	unsigned int p_len;
	unsigned periods;

	if (! core->dma_chan) {
		dev_err(p7spi_core_dev(core),
				"can't proceed without DMA\n");
		return -EINVAL;
	}

	p_len = period_len ? period_len : cdata->circ_buf_period;

	periods = nb_periods ? nb_periods : cdata->periods;
	ctrl->nb_periods = max_t(unsigned int, periods, 3);

	if (periods != ctrl->nb_periods) {
		dev_warn(p7spi_core_dev(core),
				"correcting number of periods (%d -> %d)\n",
				periods, ctrl->nb_periods);
	}



	/* adjusts buffer size */
	ctrl->circ_buf_period_len = roundup(max_t(size_t, 1, p_len),
	                                core->dma_min);
	ctrl->circ_buf_len = ctrl->nb_periods * ctrl->circ_buf_period_len;
	if (ctrl->circ_buf_period_len != p_len) {
		dev_info(p7spi_core_dev(core),
	             "correcting circular buffer period length (%dB -> %dB)"
	              " (multiple of %d)\n",
	              p_len, ctrl->circ_buf_period_len,
	              core->dma_min);
	}

	/* allocate memory for circular buffer */
	ctrl->circ_buf = kzalloc(ctrl->circ_buf_len, GFP_KERNEL | GFP_DMA);
	if (ctrl->circ_buf == NULL) {
		dev_err(p7spi_core_dev(core),
				"Failed to allocate bytes for circular buffer (len: %d)\n",
				ctrl->circ_buf_len);
		return -ENOMEM;
	}

	/* allocate memory for period descriptor array */
	ctrl->periods = kzalloc(ctrl->nb_periods * sizeof(struct p7spis_period_desc),
	                        GFP_KERNEL);
	if (ctrl->periods == NULL) {
		dev_err(p7spi_core_dev(core),
				"Failed to allocate memory for period descriptors\n");
		err = -ENOMEM;
		goto free_period_desc;
	}

	/* Initialize periods */
	for(i = 0; i < ctrl->nb_periods; i++) {

		p_desc = &ctrl->periods[i];

		p_desc->buf = ctrl->circ_buf + i * ctrl->circ_buf_period_len;
		p_desc->state = P7SPIS_PERIOD_PREP;
		spin_lock_init(&p_desc->lock);
	}

	ctrl->circ_head = ctrl->circ_tail = ctrl->periods;

	return 0;

free_period_desc:
	p7spis_clean_circ_buffer(ctrl);
	return err;
}



static void p7spis_disable_core(struct p7spis_core *ctrl)
{
	struct p7spi_core const* const core = &ctrl->core;

	ctrl->ctrl_reg = __raw_readl(core->vaddr + P7SPI_CTRL);
	writel(0, core->vaddr + P7SPI_CTRL);

}

static void p7spis_enable_core(struct p7spis_core *ctrl)
{
	struct p7spi_core* const core = &ctrl->core;

	p7spi_enable_core(core, NULL, ctrl->ctrl_reg);
}


static int p7spis_setup(struct spi_device* master)
{
	struct p7spis_core* const               ctrl = spi_master_get_devdata(master->master);
	struct p7spi_core* const                core = &ctrl->core;
	struct p7spis_ctrl_data const* const    spis_cdata = master->controller_data;
	struct p7spi_ctrl_data const* const     cdata = &spis_cdata->common;
	u32                                     ctrl_reg = 0;
	int                                     err;

	if (cdata->half_duplex)
		master->master->flags |= SPI_MASTER_HALF_DUPLEX;
	if (!cdata->read)
		master->master->flags |= SPI_MASTER_NO_RX;
	if (!cdata->write)
		master->master->flags |= SPI_MASTER_NO_TX;
	if (! cdata->half_duplex)
		dev_dbg(p7spi_core_dev(core),
			"Full duplex transfers are not reliable in slave mode");
	if (! (cdata->half_duplex || (cdata->read ^ cdata->write))) {
		dev_err(p7spi_core_dev(core),
		        "invalid transfer mode for device %s (%s duplex, %sread, %swrite)\n",
		        dev_name(&master->dev),
				cdata->half_duplex ? "half" : "full",
				cdata->read ? "" : "no ",
				cdata->write ? "" : "no ");
		return -EINVAL;
	}

	err = p7spi_setup_core(core, master, &ctrl_reg);
	if (err == -EAGAIN)
		return -EPROBE_DEFER;
	else if (err)
		return err;

	if (p7spi_kern.rev == SPI_REVISION_3) {
		/*
		 * This is a bug due to unaligned transfer handling. It's not
		 * supposed to be enabled when in free run mode but it is,
		 * triggering a bug every 64KB (every time a 16-bit counter
		 * wraps). Moreover, The alignment info is in the instruction
		 * register which is not reset...
		 * To fix that, we must make the controller belive that it
		 * makes aligned transfers. To do so, we send an instruction
		 * to receive 4B (the size of a word), then we reset the
		 * controller (to stop the instruction, we don't want these
		 * 4B anyway). After that, we can continue as usual, transfers
		 * are now aligned.
		 */
		u32 temp_reg = ctrl_reg;

		temp_reg |= P7SPI_CTRL_ENABLE;
		p7spi_enable_core(core, master, temp_reg);

		writel(0x80000004, core->vaddr + P7SPI_INSTR);
		clk_disable_unprepare(core->clk);
		clk_prepare_enable(core->clk);
	}


	/* Setup transfer mode. */
	ctrl_reg |= (cdata->xfer_mode << P7SPI_CTRL_SMODE_SHIFT);

	/* Start core in slave mode. Force Read-Only and DMA */
	ctrl_reg |= P7SPI_CTRL_ENABLE;
	ctrl_reg |= P7SPI_CTRL_RWS_RO | P7SPI_CTRL_DMAES_ENABLED;
	if (p7spi_kern.rev) {
		/* for rev > R1, set RX free run flag */
		ctrl_reg |= P7SPI_CTRL_RX_FREE_RUNS;
	}
	p7spi_enable_core(core, master, ctrl_reg);

	if (ctrl->circ_buf == NULL) {
		err = p7spis_setup_circ_buffer(ctrl, spis_cdata);
		if (err)
			goto disable_core;

		err = p7spis_prepare_dma_cylic(ctrl);
		if (err)
			goto free_circ_buf;

		ctrl->paused = false;
	}

	dev_info(p7spi_core_dev(core),
			 "enabled device %s on slave core:\n"
			 "\t%s duplex/%sread/%swrite\n"
			 "\tmode: %s\n"
			 "\tclock: polarity=%d phase=%d\n"
			 "\tfifo: %uB/%uB\n"
			 "\tbuffer: [%p] %uB (%d periods)\n",
			 dev_name(&master->dev),
			 cdata->half_duplex ? "half" : "full",
			 cdata->read ? "" : "no ",
			 cdata->write ? "" : "no ",
			 p7spi_mode_name(cdata->xfer_mode),
	         !! (master->mode & SPI_CPOL),
	         !! (master->mode & SPI_CPHA),
	         (unsigned int) core->thres_sz,
	         (unsigned int) core->fifo_sz,
	         ctrl->circ_buf, ctrl->circ_buf_len, ctrl->nb_periods);
	return 0;

free_circ_buf:
	p7spis_clean_circ_buffer(ctrl);
disable_core:
	p7spis_disable_core(ctrl);

	dev_err(p7spi_core_dev(core),
			 "failed to enable device %s on slave core:\n",
			 dev_name(&master->dev));
	return err;
}

static int __p7spis_pause(struct spi_master* master)
{
	int i;
	struct p7spis_core * const ctrl =
	                spi_master_get_devdata(master);

	/*
	 * TODO:
	 *  - don't pause until all pending messages are consumed
	 *  - return error code when calling spi_async and core is paused
	 */
	if (ctrl->paused) {
		return -ESHUTDOWN;
	}

	p7spis_disable_core(ctrl);
	dmaengine_terminate_all(ctrl->core.dma_chan);
	ctrl->circ_head = ctrl->circ_tail = ctrl->periods;
	ctrl->paused = true;
	for(i = 0; i < ctrl->nb_periods; i++) {
		struct p7spis_period_desc *desc = &ctrl->periods[i];
		desc->state = P7SPIS_PERIOD_PREP;
		desc->pos = 0;
	}
	if (! list_empty(&ctrl->pending_messages)) {
		struct spi_message  *mesg, *next;

		list_for_each_entry_safe(mesg, next, &ctrl->pending_messages, queue) {
			list_del_init(&mesg->queue);
			mesg->state = NULL;
			mesg->status = -ENODEV;
			mesg->complete(mesg->context);
		}
	}

	return 0;
}

int p7spis_pause(struct spi_master* master)
{
	int ret;
	struct p7spis_core * const ctrl =
	                spi_master_get_devdata(master);

	spin_lock_bh(&ctrl->lock);
	ret = __p7spis_pause(master);
	spin_unlock_bh(&ctrl->lock);

	return ret;
}
EXPORT_SYMBOL(p7spis_pause);

static int __p7spis_resume(struct spi_master* master)
{
	struct p7spis_core * const ctrl =
	                spi_master_get_devdata(master);
	int ret = -EBUSY;

	if (! ctrl->paused) {
		return ret;
	}

	p7spis_enable_core(ctrl);
	ret = p7spis_prepare_dma_cylic(ctrl);
	ctrl->paused = false;
	return ret;
}


int p7spis_resume(struct spi_master *master)
{
	struct p7spis_core * const ctrl =
	                spi_master_get_devdata(master);
	int ret;

	spin_lock_bh(&ctrl->lock);
	ret = __p7spis_resume(master);
	spin_unlock_bh(&ctrl->lock);

	return ret;
}
EXPORT_SYMBOL(p7spis_resume);

static int p7spis_dummy_xfer(struct spi_master* master, struct spi_message* msg)
{
	return 0;
}


static struct p7spi_ops p7spis_ops __devinitdata = {
	.setup      = p7spis_setup,
	.xfer       = p7spis_dummy_xfer,
	.rt         = false,
	.dma_cyclic = true,
};

static int __devinit p7spis_probe(struct platform_device* pdev)
{
	int ret;

	ret = p7spi_create_ctrl(pdev, &p7spis_ops, struct p7spis_core);

	if (! ret) {
		struct p7spis_core* const ctrl =
						(struct p7spis_core*) platform_get_drvdata(pdev);

		spin_lock_init(&ctrl->lock);
		init_waitqueue_head(&ctrl->wq);
		INIT_LIST_HEAD(&ctrl->pending_messages);

		/* shunt default transfer function. So spi_async will call
		 * p7spis_transfer */
		ctrl->core.ctrl->transfer = p7spis_transfer;

		/*
		 * p7spi_register calls spi_register_master internally. As soon as
		 * this function is called, p7spis_setup can be called, so we need to
		 * initialize ctrl->lock before that
		 */
		ret = p7spi_register(pdev, &ctrl->core);
		if (ret) {
			p7spi_abort_create(pdev);
			return ret;
		}
	}

	return ret;
}

static int __devexit p7spis_remove(struct platform_device *pdev)
{
	struct p7spis_core* const ctrl =
					(struct p7spis_core*) platform_get_drvdata(pdev);

	if (ctrl->circ_buf) {
		dmaengine_terminate_all(ctrl->core.dma_chan);
		p7spis_unmap_dma_cyclic(ctrl);
		p7spis_clean_circ_buffer(ctrl);
	}

	return p7spi_destroy_ctrl(pdev);
}

#ifdef CONFIG_PM_SLEEP
static int p7spis_pm_suspend(struct device* dev)
{
	struct platform_device* pdev = to_platform_device(dev);
	struct p7spi_core* core = platform_get_drvdata(pdev);
	int ret;

	ret = p7spis_pause(core->ctrl);
	clk_disable_unprepare(core->clk);
	return ret;
}

static int p7spis_pm_resume(struct device* dev)
{
	struct platform_device* pdev = to_platform_device(dev);
	struct p7spi_core* core = platform_get_drvdata(pdev);

	clk_prepare_enable(core->clk);
	return p7spis_resume(core->ctrl);
}
#else
#define p7spis_pm_suspend       NULL
#define p7spis_pm_resume        NULL
#endif

static struct dev_pm_ops p7spis_dev_pm_ops = {
	.suspend    = p7spis_pm_suspend,
	.resume     = p7spis_pm_resume,
};

static struct platform_driver p7spis_driver = {
	.driver = {
		.name   = P7SPIS_DRV_NAME,
		.owner  = THIS_MODULE,
		.pm     = &p7spis_dev_pm_ops,
	},
	.probe		= p7spis_probe,
	.remove		= __devexit_p(p7spis_remove),
};

static int __init p7spis_init(void)
{
	p7spis_ops.dma = true;

	return platform_driver_register(&p7spis_driver);
}
module_init(p7spis_init);

static void __exit p7spis_exit(void)
{
	platform_driver_unregister(&p7spis_driver);
}
module_exit(p7spis_exit);

MODULE_AUTHOR("Alvaro Moran - Parrot S.A. <alvaro.moran@parrot.com>");
MODULE_DESCRIPTION("Parrot7 SPI slave controller driver");
MODULE_LICENSE("GPL");
