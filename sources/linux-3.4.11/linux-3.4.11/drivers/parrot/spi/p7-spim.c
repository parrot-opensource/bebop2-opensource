/**
 * linux/drivers/parrot/spi/p7-spim.c - Parrot7 SPI master controller driver
 *                                      implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * @author: alvaro.moran@parrot.com
 * @author: Gregor Boirie <gregor.boirie@parrot.com>
 * @date:   2012-01-09
 *
 * This file is released under the GPL
 *
 * TODO:
 *   - remove unneeded timeout while polling controller status,
 *   - clean up debug / log / BUG_ON mess,
 *   - refactor with slave driver common parts,
 *   - document anatomy of a transfer (message / xfer / segments /rounds) in
 *     DMA, interrupt and polling modes,
 *   - implement full support for full duplex in PIO & DMA modes,
 *   - implement MPW2 instructions queueing ahead of time when possible,
 *   - review.
 *
 * Notes:
 * ------
 * In MPW1, FIFO TX Threshold interrupt is not generated if there is enough
 * data in FIFO to finish the current instruction. That's why we push
 * instructions only when we want to (en/de)queue more bytes than what was
 * previously instructed.
 * This forces us to maintain a count of bytes remaining to order for the
 * current transfer (see ctrl->order). This SHOULD be fixed in MPW2.
 *
 * Currently, full duplex transfers are partially implemented: full duplex will
 * work only for tx & rx buffers start @ aligned on the same boundary. Moreover,
 * full duplex support in DMA mode cannot be implemented untill pl330 driver
 * supports interleaved mode.
 *
 * In MPW1, if a given instruction/frame has an unaligned number of bytes
 * (not multiple of 4) to be received, the last 32-bit word in the RX FIFO
 * will be merged with the byte(s) of the next instruction/frame.
 * This prevents us from queueing instructions ahead of time and introduces
 * sub-optimal wait states between aligned and unaligned transfers (where we
 * have to wait for all instructions completion before firing up a new round).
 * This MUST be fixed in MPW2.
 *
 * MPW1 does not provide software with a way to flush instruction FIFOs. This
 * forces driver to poll for all instructions completion when flushing (see
 * p7spim_flush). This MUST be fixed in MPW2.
 *
 * MPW1 does not provide dsoftware with a way to de-select a slave peripheral
 * without toggling clock, MISO or MOSI pins (see p7spim_flush).
 * This SHOULD be fixed for MPW2.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/dmaengine.h>
#include <asm/io.h>
#include "p7-spi.h"
#include "p7-spim.h"
#include "p7-spi_priv.h"

static bool noirq = false;
module_param(noirq, bool, 0644);
MODULE_PARM_DESC(noirq, "do not use interrupt handling");

static bool nodma = false;
module_param(nodma, bool, 0644);
MODULE_PARM_DESC(nodma, "do not use dma operations");

/* In P7 R1 and R2 the way to release the SS to high is different, 
 * we setup at init.*/
static u32 p7spim_SSEND = 0;

/*
 * Master core instance internal state
 */
struct p7spim_core {
	/* Common SPI core instance */
	struct p7spi_core   core;
	/* Number of bytes left to instruct for current transfer */
	size_t              order;
	/* State of slave select when transfers ends */
	bool                drop_ss;
	/* Hardware mode control register word for current device. */
	uint32_t            mode;
	/* Core input clock rate in Hz */
	unsigned int        base_hz;
	/* Controller / core / master current SPI clock rate in Hz */
	unsigned int        curr_hz;
#ifdef CONFIG_PM_SLEEP
	/* Saved control register accross suspend */
	u32                 saved_ctrl;
#endif
};

/*
 * Return hardware divisor to reach SPI bus frequency passed in argument.
 *
 * Bus clock frequency is computed using the following formula:
 * bus freq = parent clock freq / ((divisor + 1) × 2)
 */
static u32 p7spim_freq_div(struct p7spim_core const* ctrl, unsigned int hz)
{
	u32 const div = DIV_ROUND_UP(ctrl->base_hz, 2 * hz);
	u32 speed_max_div;

	if (p7spi_kern.rev == SPI_REVISION_1)
		speed_max_div = P7SPI_SPEED_MAXDIV_R1;
	else
		speed_max_div = P7SPI_SPEED_MAXDIV_R23;

	if ((div - 1) & ~speed_max_div)
		dev_dbg(p7spi_core_dev(&ctrl->core),
		        "unreacheable clock frequency requested (%uHz)\n",
		        hz);

	return (div - 1) & speed_max_div;
}

/*
 * Express delay passed in argument as number of bus clock ticks using the
 * specified bus clock period (in nano seconds).
 * Number of tick will be rounded to the closest upper one.
 */
static u32 p7spim_delay_ticks(struct p7spim_core const* ctrl,
                              unsigned int delay,
                              unsigned int ns)
{
	u32 const ticks = DIV_ROUND_UP(delay, ns);

#define P7SPI_TIMING_MAXTICKS   (0xfU)

	if (ticks & ~P7SPI_TIMING_MAXTICKS)
		dev_dbg(p7spi_core_dev(&ctrl->core),
		        "unreacheable delay requested (%uns)\n",
		        delay);

	return ticks & P7SPI_TIMING_MAXTICKS;
}

/* Adjust timings given the bus clock frequency passed in argument. */
static void p7spim_adjust_clk(struct p7spim_core* ctrl,
                              struct p7spi_plat_data const* pdata,
                              struct p7spi_ctrl_data const* cdata,
                              unsigned int hz)
{
	struct p7spi_core* const    core = &ctrl->core;
	u32                         div_reg;
	u32                         tim_reg;
	u32                         ns;

	/* Save uncorrected requested frequency. */
	ctrl->curr_hz = hz;

	/* Round requested frequency to possible limits. */
	hz = max_t(unsigned int, hz, pdata->min_hz);
	hz = min_t(unsigned int, hz, pdata->max_master_hz);

	/* Compute divisor to reach maximum SPI master clock frequency. */
	div_reg = p7spim_freq_div(ctrl, hz);

	/* Compute timings according to above frequency. */
	ns = ctrl->base_hz / ((div_reg + 1) * 2);
	dev_dbg(p7spi_core_dev(core),
		"bus clock frequency set to %uHz",
	        ns);
	ns = NSEC_PER_SEC / ns;
	tim_reg = (p7spim_delay_ticks(ctrl,
	                              cdata->tsetup_ss_ns,
	                              ns) << P7SPI_TSETUP_SHIFT) |
	          (p7spim_delay_ticks(ctrl,
	                              cdata->thold_ss_ns,
	                              ns) << P7SPI_THOLD_SHIFT) |
		      (p7spim_delay_ticks(ctrl,
		                          cdata->toffclk_ns,
		                          ns) << P7SPI_TOFFCLK_SHIFT) |
		      (p7spim_delay_ticks(ctrl,
		                          cdata->toffspi_ns,
		                          ns) << P7SPI_TOFFSPI_SHIFT) |
		      (p7spim_delay_ticks(ctrl,
		                          cdata->tcapture_delay_ns +
								  (P7SPI_TCAPTURE_DELAY_MIN *
								   NSEC_PER_SEC / ctrl->base_hz),
								  NSEC_PER_SEC / ctrl->base_hz) <<
		       P7SPI_TCAPTURE_DELAY_SHIFT);

	/* Setup master clock divisor. */
	__raw_writel(div_reg, core->vaddr + P7SPI_SPEED);
	/* Setup master timings. */
	__raw_writel(tim_reg, core->vaddr + P7SPI_TIMING);
}

/*
 * Check if pushed instructions (i.e. sequence of instructed transfers)
 * are all completed.
 */
static int p7spim_instr_done(struct p7spi_core const* core, u32 stat)
{
	if (! (stat & P7SPI_STATUS_END_TRANSM))
		return -EAGAIN;

	return 0;
}

/*
 * Order a new transfer, i.e. push a new instruction to fire up a transfer
 * round.
 */
static void p7spim_push_instr(struct p7spim_core const* ctrl,
                              size_t bytes,
                              bool dma,
                              bool drop_ss)
{
	struct p7spi_core const* const  core = &ctrl->core;
	u32                             cmd = ctrl->mode | bytes;

	/*
	 * Build instruction word.
	 */
	if (core->tx_buff)
		/* Write requested. */
		cmd |= P7SPI_INSTR_RW_WRITE;

	if (core->rx_buff)
		/* Read requested. */
		cmd |= P7SPI_INSTR_RW_READ;

	if (drop_ss)
		/* Slave de-select requested. */
		cmd |= p7spim_SSEND;

	if (dma)
		/* DMA requested. */
        cmd |= P7SPI_INSTR_DMAE_ENABLE;

#ifdef DEBUG
	dev_dbg(p7spi_core_dev(core), "\t\t\t%uB instruction: 0x%08x\n", bytes, cmd);
	BUG_ON(! ctrl);
	BUG_ON(! bytes);
	BUG_ON(bytes > P7SPI_INSTR_LEN_MASK);
	BUG_ON(__raw_readl(core->vaddr + P7SPI_STATUS) &
		   P7SPI_STATUS_INSTR_FULL);
#endif
	/*
	 * Push instruction word.
	 */
	__raw_writel(cmd, core->vaddr + P7SPI_INSTR);
}


/*
 * Limitation on r1 and r2 using DMA in TX mode.
 * The SPI notify DMA that it is ready to receive bytes when
 *       nb_bytes_in_instruction - TXCNT_VAL > DMA burst,
 * and TXCNT_VAL is reset when the first byte of a new instruction is
 * written in the FIFO. So when a first transfer completes, TXCNT_VAL
 * is equal to the amount of bytes that have been transfered. Then, if
 * another round tries to transfer the same amount by DMA, TXCNT_VAL
 * is equal to nb_bytes and the DMA is not notified, so the DMA
 * transfer can't start and TXCNT_VAL is not reset. Deadlock.
 * The workaround is that before any DMA transfer are started in
 * TX mode, wait for the previous instruction to complete
 * and reset TXCNT_VAL
 */
static int p7spim_reset_txcnt(struct p7spim_core const* ctrl)
{
	struct p7spi_core const* const  core = &ctrl->core;

	if (core->tx_buff) {
		int err;
		err = p7spi_poll_stat(core, p7spim_instr_done, 5 * HZ);
		if (err) {
			p7spi_dump_regs(core, "dma timeout");
			return err;
		}
		__raw_writel(P7SPI_ITACK_TXCNT_ACK, core->vaddr + P7SPI_ITACK);
		if (p7spi_kern.rev == SPI_REVISION_1) {
			/* MPW1 needs 0 to be rewritten in the register */
			writel(0, core->vaddr + P7SPI_ITACK);
		}

	}

	return 0;
}


static bool p7spim_must_drop_ss(struct p7spim_core* ctrl, size_t rem_bytes)
{
	struct p7spi_core* const    core = &ctrl->core;

	return (core->bytes == rem_bytes) && ctrl->drop_ss;
}


/*
 * Launch a new transfer performing an alignment round.
 * This will transfer enought bytes in polling mode to align overall transfer
 * on a 32 bits boundary so that subsequent round may use either interrupts or
 * DMA mode (FIFOs are 32bits wide).
 */
static int p7spim_start_xfer(struct p7spim_core* ctrl)
{
	struct p7spi_core* const    core = &ctrl->core;
	char const* const           buff = core->rx_buff ? : core->tx_buff;
	size_t                      bytes = core->bytes;
	bool                        ss_dropped;

	if (! bytes)
		/*
		 * No more bytes to transfer: this may happen when slave de-select is
		 * requested.
		 */
		return 0;

	bytes = min(bytes, (size_t) (PTR_ALIGN(buff, sizeof(u32)) - buff));
	if (bytes) {
		/* Not aligned on word boundary: perform an alignment round. */
		int err;

		/*
		 * Instruct transfer ahead of time so that bytes are transmitted as soon
		 * as bytes are queued into FIFO. Controller will suspend transmission
		 * (i.e., stop toggling bus clock) if FIFOs are not ready (tx empty or
		 * rx full).
		 *
		 * On R1, if he have to deselect slave-select, it will be done in another
		 * function (_end_xfer). On R2 and R3, we do it in place.
		 */
		if ((p7spi_kern.rev != SPI_REVISION_1) &&
		    p7spim_must_drop_ss(ctrl, bytes)) {
			p7spim_push_instr(ctrl, bytes, false, true);
			ss_dropped = true;
		}
		else {
			p7spim_push_instr(ctrl, bytes, false, false);
			ss_dropped = false;
		}

		if (core->tx_buff)
			/* Feed tx FIFO. */
			p7spi_writeb_fifo(core, bytes);

		dev_dbg(p7spi_core_dev(core),
				"\t\t\t%uB align %swr[%p] rd[%p]\n",
				bytes,
				ss_dropped ? "cs " : "",
				core->tx_buff ? (core->tx_buff - bytes) : NULL,
				core->rx_buff);

		/* Wait for completion of instruction. */
		err = p7spi_poll_stat(core, p7spim_instr_done, 5 * HZ);
		if (err) {
			p7spi_dump_regs(core, "align round failed");
			return err;
		}

		if (core->rx_buff)
			/* Fetch data from rx FIFO. */
			p7spi_readb_fifo(core, bytes);

		/* Update count of completed bytes. */
		core->bytes -= bytes;
	}

	/*
	 * Return bytes count left to transfer to inform caller wether or not it
	 * must run additional transfer rounds.
	 */
	return core->bytes;
}

/*
 * Finalize a transfer. When processing terminal bytes of a transfer, take
 * care of slave select line as described below.
 *
 * As of MPW1, controller hardware only allow software to de-select slave after
 * each transfered bytes (i.e., software cannot request to de-select slave when
 * transmitting last byte of a transfer). This is the reason why we have to
 * build a separate transfer round to take care of slave select line.
 *
 * MPW2 will modify this behavior.
 */
static int p7spim_end_xfer(struct p7spim_core* ctrl, bool drop_ss)
{
	int                         err;
	struct p7spi_core* const    core = &ctrl->core;

	if (p7spi_kern.rev != SPI_REVISION_1)
		/*
		 * On newer versions, CS is raised at the end of the
		 * instruction and there is no need to send a lonely byte
		 * to do so
		 */
		return 0;

	if (! drop_ss)
		/* Silently ignore if slave deselect is not required. */
		return 0;

	p7spim_push_instr(ctrl, 1U, false, true);

	if (core->tx_buff)
		p7spi_writeb_fifo(core, 1U);

	dev_dbg(p7spi_core_dev(core),
			"\t\t\t%uB cs wr[%p] rd[%p]\n",
			1U,
			core->tx_buff ? (core->tx_buff - 1U) : NULL,
			core->rx_buff);

	err = p7spi_poll_stat(core, p7spim_instr_done, 5 * HZ);
	if (err) {
		p7spi_dump_regs(core, "cs round failed");
		return err;
	}

	if (core->rx_buff)
		p7spi_readb_fifo(core, 1U);

	return 0;
}

/*
 * Launch the very first round of the first segment of a transfer,
 * warming up the chaining of successive transfer rounds machinery
 * (by preloading tx FIFO if required).
 * This is used in polling and / or interrupt mode (also when DMA is enabled
 * if transfer size is not long enought to run the DMA controller).
 *
 * At this time, we are guaranteed the transfer start @ is aligned on a 32 bits
 * boundary (see p7spim_start_xfer).
 */
static ssize_t p7spim_start_sgm(struct p7spim_core* ctrl)
{
	struct p7spi_core* const    core = &ctrl->core;
	size_t const                bytes = min(core->bytes, core->fifo_sz);
	bool                        ss_dropped;

#ifdef DEBUG
	BUG_ON(! bytes);
#endif

	if (core->bytes > core->fifo_sz) {
		/*
		 * Compute maximum number of bytes to order
		 * (up to maximum segment size).
		 */
		size_t const order = min(core->bytes, core->sgm_sz);

		/*
		 * Instruct to transfer a number of bytes rounded down to
		 * maximum transfer segment size so that a single instruction is needed
		 * per segment.
		 */
		if ((p7spi_kern.rev != SPI_REVISION_1) &&
		    p7spim_must_drop_ss(ctrl, order)) {
			p7spim_push_instr(ctrl, order, false, true);
			ss_dropped = true;
		}
		else {
			p7spim_push_instr(ctrl, order, false, false);
			ss_dropped = false;
		}

		/* Update count of bytes left to order to complete current transfer. */
		ctrl->order = core->bytes - order;
	}
	else {
		/* A single round is enought to complete this segment. */
		if ((p7spi_kern.rev != SPI_REVISION_1) && ctrl->drop_ss) {
			p7spim_push_instr(ctrl, bytes, false, true);
			ss_dropped = true;
		}
		else {
			p7spim_push_instr(ctrl, bytes, false, false);
			ss_dropped = false;
		}
	}

	if (core->tx_buff) {
		/*
		 * Enqueue up to one FIFO depth bytes. In cases where we need multiple
		 * rounds, this allows us to preload tx FIFO with as many bytes as we
		 * can.
		 */
		p7spi_write_fifo(core, bytes);
		dev_dbg(p7spi_core_dev(core),
				"\t\t\t%uB %sproc wr[%p]\n",
				bytes,
				ss_dropped ? "cs " : "",
				core->tx_buff - bytes);
	}

	/*
	 * Return bytes count left to process to inform caller wether or not it
	 * must run additional transfer rounds.
	 * This will return 0 as long as the number of bytes to transfer is <= FIFO
	 * depth.
	 */
	return core->bytes - bytes;
}

/*
 * Complete a transfer round and initiate a new one if needed.
 * Should be called after a hardware completion event happened
 * (either FIFO threshold reached, last byte transferred or last
 * instruction completed).
 * Transfer rounds carried out by p7spim_process_round are always threshold
 * bytes long.
 * This is used in both polling and interrupt mode of operations.
 */
static int p7spim_process_round(struct p7spim_core* ctrl)
{
	struct p7spi_core* const    core = &ctrl->core;
	size_t const                thres = core->thres_sz;
	size_t const                sz = core->fifo_sz + thres;
	size_t const                bytes = core->bytes;
	bool                        ss_dropped = false;

	/*
	 * At this time:
	 *  bytes left to order / instruct:
	 *      ctrl->order
	 *  bytes left to queue:
	 *      core->bytes - core->fifo_sz
	 *  bytes left to transfer (i.e., minus bytes still present within fifo) :
	 *      core->bytes - (core->fifo_sz - core->thres_sz)
	 *  bytes left to dequeue:
	 *      core->bytes
	 */

	if (ctrl->order && (bytes < (ctrl->order + sz))) {
		/*
		 * There are still some more bytes to order and the number of bytes left
		 * to queue at the end of this function
		 * (core->bytes - (core->fifo_sz + core->thres_sz)) is < ctrl->order,
		 * the current number of bytes ordered, i.e., at the
		 * end of this function we will have queued more bytes than the current
		 * number of ordered bytes.
		 * Order as many bytes as we can, i.e., up to maximum segment size...
		 */
		size_t const order = min(ctrl->order, core->sgm_sz);

		if ((p7spi_kern.rev != SPI_REVISION_1) &&
		    (ctrl->order == order) &&
		    ctrl->drop_ss) {
			p7spim_push_instr(ctrl, order, false, true);
			ss_dropped = true;
		}
		else {
			p7spim_push_instr(ctrl, order, false, false);
			ss_dropped = false;
		}
		ctrl->order -= order;
	}

	if (bytes <= sz)
		/*
		 * Remaining bytes to queue (core->bytes - core->fifo_sz) is <=
		 * threshold.
		 * we still have to perform an extra threshold sized fifo read to
		 * catch up with the one we should have done just below (see
		 * p7spim_end_sgm).
		 * Return 0 to inform caller there is no need to perform additional
		 * rounds.
		 */
		return 0;

	if (core->rx_buff)
		/* Fetch input bytes from receive FIFO. */
		p7spi_readl_fifo(core, thres);

	/* Update count of remaining bytes to dequeue. */
	core->bytes -= thres;

	/*
	 * Now start another round... At this time, remaining bytes to
	 * queue (core->bytes + core->thres_sz - core->fifo_sz) MUST
	 * be >= core->thres_sz since we perform threshold sized transfers
	 * only.
	 */
	if (core->tx_buff)
		/* Feed transmit FIFO. */
		p7spi_writel_fifo(core, thres);

	dev_dbg(p7spi_core_dev(core),
			"\t\t\t%uB %sproc wr[%p] rd[%p]\n",
			thres,
			ss_dropped ? "cs " : "",
			core->tx_buff ? (core->tx_buff - thres) : NULL,
			core->rx_buff ? (core->rx_buff - thres) : NULL);

	/*
	 * Return count of bytes left to process to inform caller we need to perform
	 * additional rounds.
	 */
	return core->bytes;
}

/*
 * Perform the last round of the last segment of a transfer taking care of last
 * "residue" bytes.
 *
 * At this time, we are guaranteed the transfer start @ is aligned on a 32 bits
 * boundary (see p7spim_process_round).
 */
static int p7spim_end_sgm(struct p7spim_core* ctrl)
{
	struct p7spi_core* const    core = &ctrl->core;
	size_t const                fifo = core->fifo_sz;
	int                         err;

	if (core->bytes > fifo) {
		/*
		 * We had to perform more than a full FIFO bytes long transfer, meaning
		 * we are comming from p7spim_process_round and we still have to run an
		 * extra threshold sized fifo read.
		 */
		size_t const    bytes = core->bytes - fifo;
		size_t const    thres = core->thres_sz;

#ifdef DEBUG
		BUG_ON(core->bytes > (fifo + thres));
#endif

		if (core->rx_buff) {
			/* Fetch input bytes from receive FIFO. */
			p7spi_readl_fifo(core, thres);
			dev_dbg(p7spi_core_dev(core),
					"\t\t\t%uB proc rd[%p]\n",
					thres,
					core->rx_buff - thres);
		}

		/* Update completed bytes counter. */
		core->bytes -= thres;

		if (bytes && core->tx_buff) {
			/* Enqueue last bytes to send if any. */
			p7spi_write_fifo(core, bytes);
			dev_dbg(p7spi_core_dev(core),
					"\t\t\t%uB end wr[%p]\n",
					bytes,
					core->tx_buff - bytes);
		}
	}

	/* Ensure all instructions were completed. */
	err = p7spi_poll_stat(core, p7spim_instr_done, 5 * HZ);
	if (err) {
		p7spi_dump_regs(core, "end round failed");
		return err;
	}

	if (! (core->bytes && core->rx_buff))
		/* No more bytes to receive. */
		return 0;

	/* Fetch bytes from input FIFO. */
	p7spi_read_fifo(core, core->bytes);
	dev_dbg(p7spi_core_dev(core),
			"\t\t\t%uB end rd[%p]\n",
			core->bytes,
			core->rx_buff - core->bytes);
	return 0;
}

/*
 * Perform transfer in polling mode.
 */
static int p7spim_xfer_bypoll(struct p7spim_core* ctrl)
{
	if (p7spim_start_sgm(ctrl) > 0) {
		int                         err;
		struct p7spi_core* const    core = &ctrl->core;

		do {
			err = p7spi_poll_stat(core, p7spi_data_ready, 15 * HZ);
		} while (! err && p7spim_process_round(ctrl));

		if (err) {
			p7spi_dump_regs(core, "proc round failed");
			return err;
		}
	}

	return p7spim_end_sgm(ctrl);
}

/*
 * Master controller interrupt handler
 * In interrupt mode, process threshold bytes long transfer rounds.
 */
static irqreturn_t p7spim_handle_irq(int irq, void* dev_id)
{
	struct p7spim_core* const   ctrl = (struct p7spim_core*) dev_id;
	struct p7spi_core* const    core = &ctrl->core;

	core->stat = p7spi_poll_stat(core, p7spi_data_ready, 0);
	switch (core->stat) {
	case -EAGAIN:
		/* Status reports nothing particular: spurious interrupt ?? */
		return IRQ_NONE;

	case 0:
		if (p7spim_process_round(ctrl))
			/*
			 * Round processed but there are still some more threshold bytes
			 * long rounds to perform (i.e., remaining bytes to complete >
			 * threshold).
			 */
			return IRQ_HANDLED;
	}

	/*
	 * Disable interrutps since remaining bytes will be transfered in polling
	 * mode.
	 */
	__raw_writel(0, core->vaddr + P7SPI_ITEN);

	/* Notify worker we are done. */
	complete(&core->done);
	return IRQ_HANDLED;
}

/*
 * Perform transfer in interrupt mode.
 */
static int p7spim_xfer_byirq(struct p7spim_core* ctrl)
{
	struct p7spi_core* const    core = &ctrl->core;

	if (p7spi_has_irq(core)) {
		/* Require being notified of FIFOs (over/under)flow (MPW2 only). */
		u32 iten = P7SPI_ITEN_RXACCESS_ERROR |
		           P7SPI_ITEN_TXACCESS_ERROR;

		if (p7spim_start_sgm(ctrl) > 0) {
			/*
			 * Initialize interrupt mode completion token: we will be notified
			 * once "residue" bytes only are left to be processed.
			 */
			INIT_COMPLETION(core->done);

			/*
			 * When transfering in rx & tx mode, we might catch 2 interrupts:
			 * activate rx-side interrupt only when rx & tx required.
			 */
			if (core->rx_buff)
				/* rx and / or tx requested. */
				iten |= P7SPI_ITEN_RX_TH_REACHED;
			else if (core->tx_buff)
				/* tx only. */
				iten |= P7SPI_ITEN_TX_TH_REACHED;
			__raw_writel(iten, core->vaddr + P7SPI_ITEN);

			/* Wait for end of threshold bytes long rounds processing. */
			if (! wait_for_completion_timeout(&core->done, 15 * HZ)) {
				p7spi_dump_regs(core, "proc round failed");

				/* Disable interrupts since interrupt handler could not do it. */
				__raw_writel(0, core->vaddr + P7SPI_ITEN);

				/*
				 * Return timeout error only if rounds processing did not
				 * encounter any.
				 */
				core->stat = core->stat ? : -ETIMEDOUT;
			}

			if (core->stat)
				return core->stat;
		}

		return p7spim_end_sgm(ctrl);
	}

	return p7spim_xfer_bypoll(ctrl);
}

/*
 * DMA task completion handler (run in tasklet context).
 * Just notify xfer_msg function that it's done.
 */
static void p7spim_complete_dma(void* controller)
{
	struct p7spim_core* const   ctrl = controller;
	struct p7spi_core* const    core = &ctrl->core;

	/* Notify worker we are done. */
	complete(&core->done);
}

/*
 * Transfer extra bytes after DMA rounds.
 */
static int p7spim_xfer_dma_extra(struct p7spim_core* ctrl, size_t bytes)
{
	struct p7spi_core* const core = &ctrl->core;
	int err;

#ifdef DEBUG
	BUG_ON(bytes >= core->fifo_sz);
#endif

	if (!bytes)
		return 0;

	if (core->tx_buff) {
		err = p7spi_poll_stat(core, p7spi_data_ready, 5 * HZ);
		if (err) {
			p7spi_dump_regs(core, "xtra round failed");
			return err;
		}
		p7spi_write_fifo(core, bytes);
		dev_dbg(p7spi_core_dev(core),
		        "\t\t\t%uB xtra wr[%p]\n",
		        bytes, core->tx_buff - bytes);
	}

	err = p7spi_poll_stat(core, p7spim_instr_done, 5 * HZ);
	if (err) {
		p7spi_dump_regs(core, "xtra round failed");
		return err;
	}

	if (core->rx_buff) {
		p7spi_read_fifo(core, bytes);
		dev_dbg(p7spi_core_dev(core),
		        "\t\t\t%uB extra rd[%p]\n",
		        bytes, core->rx_buff - bytes);
	}

	core->bytes -= bytes;
	return 0;
}

/*
 * Perform transfer in DMA mode.
 * DMA transfers must satisfy following constraints:
 *  - start @ must be aligned on a cache line boundary,
 *  - size must be a multiple of cache line size,
 *  - DMA controller <-> SPI controller exchange burst size must be equal to
 *    SPI controller FIFO threshold to properly handle DMA peripheral interface
 *    handshakes (this means transfer size must be a multiple of burst size as
 *    well).
 * When these condition cannot be satisfied, fall back to interrupt / polling
 * mode.
 */
static int p7spim_xfer_bydma(struct p7spim_core* ctrl)
{
	int                         err = 0;
	bool                        ss_dropped;
	struct p7spi_core* const    core = &ctrl->core;
	char const* const           buff = core->rx_buff ? : core->tx_buff;
	ssize_t                     bytes;

	bytes = min_t(ssize_t,
	              core->bytes,
	              PTR_ALIGN(buff,dma_get_cache_alignment()) - buff);

	if (! p7spi_has_dma(core) ||
	    ((core->bytes - (size_t) bytes) < core->dma_min))
		/*
		 * Cannot perform transfer using DMA because either DMA is disabled or
		 * transfer size and / or start @ don't comply with the above
		 * contraints. Fall back to interrupt / polling mode.
		 */
		return p7spim_xfer_byirq(ctrl);

	if (bytes) {
		/*
		 * Transfer enought byte to align start @ to cache line / burst size
		 * boundary if needed. core->bytes content is altered to fool
		 * interrupt / polling implementation so that it transfers required
		 * count of bytes only.
		 */
		size_t const sz = core->bytes - (size_t) bytes;
		bool save_drop_ss = ctrl->drop_ss;

		/* Request p7spim_xfer_byirq to transfer sz bytes only. */
		core->bytes = (size_t) bytes;
		ctrl->drop_ss = false;

		err = p7spim_xfer_byirq(ctrl);
		if (err) {
			ctrl->drop_ss = save_drop_ss;
			return err;
		}

		/*
		 * Restore and update proper bytes count now that DMA operation is
		 * possible.
		 */
		core->bytes = sz;
		ctrl->drop_ss = save_drop_ss;
	}

	/*
	 * Setup DMA operation properties common to all segments for this
	 * transfer.
	 */
	err = p7spi_config_dma(core);
	if (err)
		return err;

	/* Setup SPI controller DMA peripheral interface. */
	p7spi_cook_dmaif(core);

	do {
		size_t extra_bytes = 0;
		/*
		 * map segment by segment.
		 */
		bytes = p7spi_map_dma(core);
		if (bytes < 0)
			goto unmap;
		else if (core->bytes - bytes > 0 &&
			 !core->tx_buff &&
		         core->bytes - bytes < core->fifo_sz &&
			 core->bytes < core->sgm_sz) {
			/*
			 * This is a little hack to use the same instruction
			 * to transfer remaining bytes in RX mode.
			 */
			extra_bytes = core->bytes - bytes;
			bytes = core->bytes;
			dev_dbg(p7spi_core_dev(core),
				"\t\t\tusing dma instr to send %uB extra\n",
				extra_bytes);
		}

		/* Submit DMA operation for current segment. */
		err = p7spi_run_dma(core, p7spim_complete_dma);
		if (err)
			goto unmap;

		if (p7spi_kern.rev == SPI_REVISION_1) {
			/* we have to reet txcnt between two transfers only on R1 */
			err = p7spim_reset_txcnt(ctrl);
			if (err) {
				p7spi_cancel_dma(core);
				goto unmap;
			}
		}

		/* Instruct SPI controller to start transfer. */
		if ((p7spi_kern.rev != SPI_REVISION_1) &&
		    p7spim_must_drop_ss(ctrl, extra_bytes)) {
			/*
			 * core->bytes is already decreased in p7spi_run_dma, so at this
			 * point, we check if this is the last round of bytes or not.
			 */
			p7spim_push_instr(ctrl, (size_t) bytes, true, true);
			ss_dropped = true;
		}
		else {
			p7spim_push_instr(ctrl, (size_t) bytes, true, false);
			ss_dropped = false;
		}

		dev_dbg(p7spi_core_dev(core),
				"\t\t\t%uB dma %swr[%p] rd[%p]\n",
				(size_t) bytes,
				ss_dropped ? "cs " : "",
				core->tx_buff ? core->tx_buff - bytes + extra_bytes : NULL,
				core->rx_buff ? core->rx_buff - bytes + extra_bytes : NULL);

		/* Wait for segment completion. */
		err = p7spi_wait_dma(core);
		if (err) {
			/* Cancel current DMA operations if still undergoing... */
			p7spi_cancel_dma(core);
			goto unmap;
		}

		core->stat = p7spi_check_dma(core);
		if (core->stat) {
			err = core->stat;
			goto unmap;
		}

		p7spi_unmap_dma(core);

		err = p7spim_xfer_dma_extra(ctrl, extra_bytes);
		if (err) {
			p7spi_uncook_dmaif(core);
			return err;
		}


	} while (core->bytes >= core->dma_min);

unmap:
	p7spi_uncook_dmaif(core);
	if (err) {
		p7spi_unmap_dma(core);
		return err;
	}

	/*
	 * When DMA is completed in TX mode, we don't know how many bytes are laying
	 * in the FIFO.
	 * As xfer_byirq or xber_bypoll start by filling completely the FIFO, we
	 * need to ensure that there is room for these bytes, so we just wait for
	 * the "dma" bytes to be sent before continuing.
	 */
	if (core->tx_buff) {
		err = p7spi_poll_stat(core, p7spim_instr_done, 5 * HZ);
		if (err) {
			p7spi_dump_regs(core, "dma wait timeout");
			return err;
		}

		/*
		 * After a DMA TX xfer, an extra DMA request is going to be
		 * generated when the next instruction is queued and the "FIFO
		 * TX threshold reached" condition is true. It triggers a bug
		 * if the next instruction is RX DMA because in this case the
		 * DMA controller will start too soon and access an empty FIFO.
		 * The workaround is to send a "null" instruction and to flush
		 * the DMA request generated by this instruction to be in a
		 * clean state.
		 */
		__raw_writel(0, core->vaddr + P7SPI_INSTR);
		writel(P7SPI_DMA_FLUSH, core->vaddr + P7SPI_FIFO_FLUSH);
	}

	if (! core->bytes)
		return 0;

	/*
	 * Finalize transfer by handling "residue" bytes in interrupt / polling mode.
	 */
	return p7spim_xfer_byirq(ctrl);
}

static inline int p7spim_do_xfer(struct p7spim_core* ctrl)
{
	return p7spim_xfer_bydma(ctrl);
}

/*
 * Called when tring to flush FIFOs on MPW1.
 * Since MPW1 misses a way to flush instruction FIFOs, we need to ensure data
 * FIFOs are flushed as long as all instructions are not completed.
 */
static int p7spim_empty_fifo(struct p7spi_core const* core, u32 stat)
{
	/* Always flush receive FIFO if not empty. */
	__raw_writel(P7SPI_FIFORX_FLUSH, core->vaddr + P7SPI_FIFO_FLUSH);

    if ((stat & (P7SPI_STATUS_RXEMPTY | P7SPI_STATUS_END_TRANSM)) ==
        (P7SPI_STATUS_RXEMPTY | P7SPI_STATUS_END_TRANSM))
	    /*
	     * No more instructions to process and receive FIFO is empty: flush
		 * succeeded.
		 */
        return 0;

    if (core->tx_buff &&
        ! (stat & (P7SPI_STATUS_TXFULL | P7SPI_STATUS_END_TRANSM)))
	    /* Provide tx FIFO with fake data to consume instructions. */
        __raw_writel(0, core->vaddr + P7SPI_DATA);

    /* Tell caller we still need some more work to complete flush operation. */
    return -EAGAIN;
}

static void p7spim_flush(struct p7spim_core const* ctrl)
{
	struct p7spi_core const* const  core = &ctrl->core;

	if (p7spi_kern.rev == SPI_REVISION_1) {
		/* This is MPW1: flush + slave de-select. */
		int err = p7spi_poll_stat(core, p7spim_empty_fifo, 5 * HZ);

		if (err)
			dev_err(p7spi_core_dev(core),
					"failed to flush FIFOs (%d)\n",
					err);

		/*
		 * At last, de-select slave...
		 * As of MPW1, there is no way to alter slave select signal without toggling
		 * clock / MISO / MOSI.
		 * Request controller to read one byte and de-select slave just after.
		 * This will change in MPW2 !
		 */
		__raw_writel(P7SPI_INSTR_RW_READ |
					 p7spim_SSEND |
					 ctrl->mode |
					 1U,
					 core->vaddr + P7SPI_INSTR);

		err = p7spi_poll_stat(core, p7spim_instr_done, 5 *HZ);
		if (err)
			dev_err(p7spi_core_dev(core),
					"failed to de-select slave (%d)\n",
					err);
	}
	else {
		/*
		 * MPW2 implementation: instructions FIFO is flushed at reset time, which
		 * is performed by clk implementation.
		 */
		clk_disable(core->clk);
		clk_enable(core->clk);
	}

    __raw_writel(P7SPI_FIFOTX_FLUSH | P7SPI_FIFORX_FLUSH,
                 core->vaddr + P7SPI_FIFO_FLUSH);
}

/*
 * Process generic SPI layer messages (run in workqueue / kthread context).
 */
static int p7spim_xfer_msg(struct spi_master* master, struct spi_message* msg)
{
	struct p7spim_core* const           ctrl = spi_master_get_devdata(master);
	struct p7spi_core* const            core = &ctrl->core;
	struct spi_device const* const      slave = msg->spi;
	struct p7spi_plat_data const* const pdata = dev_get_platdata(p7spi_kern.dev);
	struct p7spi_ctrl_data const* const cdata = slave->controller_data;
	unsigned int const                  slave_hz = slave->max_speed_hz ? :
	                                                   pdata->max_master_hz;
	struct spi_transfer*                xfer;
	int                                 err = -ENOMSG;

	P7SPI_ASSERT_MSG(msg, ctrl, master, slave);

	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		/*
		 * When processing last transfer of current message, we are
		 * required to keep slave device selected between each message
		 * if cs_change is true.
		 * When processing other transfers of current message, we are
		 * required to deselect slave device between each transfer if
		 * cs_change is true.
		 */
		ctrl->drop_ss = list_is_last(&xfer->transfer_list,
		                             &msg->transfers) ^ xfer->cs_change;

		err = p7spi_init_xfer(core, xfer, msg, master);
		if (err)
			break;

		if ((p7spi_kern.rev == SPI_REVISION_1) && ctrl->drop_ss) {
			/*
			 * If we need to de-select slave at end of transfer, reduce size by
			 * one so that p7spim_end_sgm may handle it properly.
			 */
			core->bytes--;
		}

		/* Setup SPI bus clock rate if requested on a per-transfer basis. */
		if (!xfer->speed_hz || xfer->speed_hz > slave_hz)
			xfer->speed_hz = slave_hz;
		if (xfer->speed_hz != ctrl->curr_hz)
			p7spim_adjust_clk(ctrl, pdata, cdata, xfer->speed_hz);

		/* Start transfer. */
		err = p7spim_start_xfer(ctrl);
		if (err < 0)
			break;

		if (err > 0) {
			/*
			 * Run additional transfer rounds if some more bytes are left to
			 * transfer.
			 */
			err = p7spim_do_xfer(ctrl);
			if (err)
				break;
		}

		/* At last, take care of slave select output. */
		err = p7spim_end_xfer(ctrl, ctrl->drop_ss);
		if (err)
			break;

		msg->actual_length += xfer->len;
		if (xfer->delay_usecs)
			udelay(xfer->delay_usecs);
	}

	if (err) {
        p7spim_flush(ctrl);
		dev_err(p7spi_core_dev(core),
		        "failed to transfer message (%d)\n",
				err);
    }
	msg->status = err;
	spi_finalize_current_message(master);
	return err;
}

static int p7spim_setup(struct spi_device* slave)
{
	struct p7spi_plat_data const* const     pdata = dev_get_platdata(p7spi_kern.dev);
	struct p7spim_core* const               ctrl = spi_master_get_devdata(slave->master);
	struct p7spi_core* const                core = &ctrl->core;
	struct p7spi_ctrl_data const* const     cdata = slave->controller_data;
	u32                                     ctrl_reg = 0;
	int                                     err;

	if (cdata->half_duplex)
		slave->master->flags |= SPI_MASTER_HALF_DUPLEX;
	if (! cdata->read)
		slave->master->flags |= SPI_MASTER_NO_RX;
	if (! cdata->write)
		slave->master->flags |= SPI_MASTER_NO_TX;
	if (! (cdata->half_duplex || (cdata->read ^ cdata->write))) {
		dev_err(p7spi_core_dev(core),
		        "invalid transfer mode for device %s (%s duplex, %sread, %swrite)\n",
		        dev_name(&slave->dev),
				cdata->half_duplex ? "half" : "full",
				cdata->read ? "" : "no ",
				cdata->write ? "" : "no ");
		return -EINVAL;
	}

	err = p7spi_setup_core(core, slave, &ctrl_reg);
	if (err == -EAGAIN)
		return -EPROBE_DEFER;
	else if (err)
		return err;

	/* Setup clock and timings. */
	ctrl->base_hz = clk_get_rate(p7spi_kern.clk);
	p7spim_adjust_clk(ctrl,
	                  pdata,
	                  cdata,
	                  slave->max_speed_hz ? : pdata->max_master_hz);

	/* Setup transfer mode. */
	ctrl->mode = cdata->xfer_mode << P7SPI_INSTR_SPI_TMODE_SHIFT;

	/* Start core in master mode. */
	ctrl_reg |= P7SPI_CTRL_MSTR | P7SPI_CTRL_ENABLE;
	p7spi_enable_core(core, slave, ctrl_reg);


	dev_dbg(p7spi_core_dev(core),
			 "enabled device %s on master core:\n"
			 "\t%s duplex/%sread/%swrite\n"
			 "\tmode: %s\n"
			 "\tclock: polarity=%d phase=%d\n"
			 "\tfifo: %uB/%uB",
			 dev_name(&slave->dev),
			 cdata->half_duplex ? "half" : "full",
			 cdata->read ? "" : "no ",
			 cdata->write ? "" : "no ",
			 p7spi_mode_name(cdata->xfer_mode),
	         !! (slave->mode & SPI_CPOL),
	         !! (slave->mode & SPI_CPHA),
	         (unsigned int) core->thres_sz,
	         (unsigned int) core->fifo_sz);
	return 0;
}

static struct p7spi_ops p7spim_ops __devinitdata = {
	.setup      = p7spim_setup,
	.xfer       = p7spim_xfer_msg,
	.rt         = false,
	.dma_cyclic = false,
};

static int __devinit p7spim_probe(struct platform_device* pdev)
{
	int ret;

	ret = p7spi_create_ctrl(pdev, &p7spim_ops, struct p7spim_core);
	if (! ret) {
		struct p7spim_core* const ctrl =
						(struct p7spim_core*) platform_get_drvdata(pdev);
		ret = p7spi_register(pdev, &ctrl->core);
		if (ret)
			p7spi_abort_create(pdev);
	}

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int p7spim_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct p7spi_core* core = platform_get_drvdata(pdev);
	struct p7spim_core* ctrl = (struct p7spim_core*) core;
	int err;

	err = spi_master_suspend(core->ctrl);
	if (err)
		return err;

	ctrl->saved_ctrl = __raw_readl(core->vaddr + P7SPI_CTRL);
	__raw_writel(0, core->vaddr + P7SPI_CTRL);
	clk_disable_unprepare(core->clk);

	return 0;
}

static int p7spim_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct p7spi_core* core = platform_get_drvdata(pdev);
	struct p7spim_core* ctrl = (struct p7spim_core*) core;
	int err;

	clk_prepare_enable(core->clk);
	err = spi_master_resume(core->ctrl);
	if (err)
		return err;

	ctrl->curr_hz = 0;

	p7spi_enable_core(core, NULL, ctrl->saved_ctrl);
	return 0;
}
#else
#define p7spim_suspend   NULL
#define p7spim_resume    NULL
#endif

static struct dev_pm_ops p7spim_dev_pm_ops = {
	.suspend  = p7spim_suspend,
	.resume   = p7spim_resume,
};

static struct platform_driver p7spim_driver = {
	.driver = {
		.name   = P7SPIM_DRV_NAME,
		.owner  = THIS_MODULE,
		.pm     = &p7spim_dev_pm_ops,
	},
	.probe		= p7spim_probe,
	.remove		= __devexit_p(p7spi_destroy_ctrl),
};

static int __init p7spim_init(void)
{
	if (! noirq)
		p7spim_ops.handle_irq = p7spim_handle_irq;

	if (! nodma)
		p7spim_ops.dma = true;

	if (p7spi_kern.rev == SPI_REVISION_1)
		p7spim_SSEND = P7SPI_INSTR_SS_END_BYTE_HIGH;
	else
		p7spim_SSEND = P7SPI_INSTR_SS_END_INSTR_HIGH;

	return platform_driver_register(&p7spim_driver);
}
module_init(p7spim_init);

static void __exit p7spim_exit(void)
{
	platform_driver_unregister(&p7spim_driver);
}
module_exit(p7spim_exit);

MODULE_AUTHOR("Alvaro Moran - Parrot S.A. <alvaro.moran@parrot.com>");
MODULE_DESCRIPTION("Parrot7 SPI master controller driver");
MODULE_LICENSE("GPL");
