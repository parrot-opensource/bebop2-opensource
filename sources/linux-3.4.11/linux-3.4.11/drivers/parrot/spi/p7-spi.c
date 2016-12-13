/**
 * linux/drivers/parrot/spi/p7-spi.c - Parrot7 SPI shared resources
 *                                     driver implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    04-Jul-2012
 *
 * This file is released under the GPL
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/pinctrl/consumer.h>
#include <linux/spi/spi.h>
#include <linux/lcm.h>
#include "p7-spi.h"
#include "p7-spi_priv.h"

struct p7spi_kernel p7spi_kern = {
	.lck = __MUTEX_INITIALIZER(p7spi_kern.lck)
};
EXPORT_SYMBOL(p7spi_kern);

#if defined(DEBUG) || defined(CONFIG_DEBUG_FS)
#include <linux/debugfs.h>
#include <linux/stringify.h>

#define P7SPI_INIT_DBGREG(_name)    \
	{ .name = __stringify(_name),   .offset = P7SPI_ ## _name }

static struct debugfs_reg32 p7spi_regs[] = {
	P7SPI_INIT_DBGREG(CTRL),
	P7SPI_INIT_DBGREG(SPEED),
	P7SPI_INIT_DBGREG(TIMING),
	P7SPI_INIT_DBGREG(STATUS),
	P7SPI_INIT_DBGREG(ITEN),
	P7SPI_INIT_DBGREG(ITACK),
	P7SPI_INIT_DBGREG(TH_RX),
	P7SPI_INIT_DBGREG(TH_TX),
	P7SPI_INIT_DBGREG(TH_RXCNT),
	P7SPI_INIT_DBGREG(TH_TXCNT),
	P7SPI_INIT_DBGREG(RXCNT),
	P7SPI_INIT_DBGREG(TXCNT),
	P7SPI_INIT_DBGREG(RX_VALID_BYTES),
	P7SPI_INIT_DBGREG(FIFO_FLUSH),
	P7SPI_INIT_DBGREG(FIFO_RXLVL),
	P7SPI_INIT_DBGREG(FIFO_TXLVL)
};

#endif  /* #if defined(DEBUG) || defined(CONFIG_DEBUG_FS) */

#ifdef DEBUG

void p7spi_dump_regs(struct p7spi_core const* core, char const* msg)
{
	int r;

	dev_dbg(p7spi_core_dev(core), "%s:\n", msg);

	for (r = 0; r < ARRAY_SIZE(p7spi_regs); r++)
		dev_dbg(p7spi_core_dev(core),
		        "\t%s = 0x%08x\n",
		        p7spi_regs[r].name,
		        readl(core->vaddr + p7spi_regs[r].offset));
}
EXPORT_SYMBOL(p7spi_dump_regs);

#endif

#if defined(CONFIG_DEBUG_FS)

static void __devinit p7spi_init_kern_dbgfs(void)
{
	static struct debugfs_reg32 set[] = {
		P7SPI_INIT_DBGREG(MEM),
		P7SPI_INIT_DBGREG(SWB_OPAD(0)),
		P7SPI_INIT_DBGREG(SWB_OPAD(1)),
		P7SPI_INIT_DBGREG(SWB_OPAD(2)),
		P7SPI_INIT_DBGREG(SWB_OPAD(3)),
		P7SPI_INIT_DBGREG(SWB_OPAD(4)),
		P7SPI_INIT_DBGREG(SWB_OPAD(5)),
		P7SPI_INIT_DBGREG(SWB_OPAD(6)),
		P7SPI_INIT_DBGREG(SWB_OPAD(7)),
		P7SPI_INIT_DBGREG(SWB_OPAD(8)),
		P7SPI_INIT_DBGREG(SWB_OPAD(9)),
		P7SPI_INIT_DBGREG(SWB_OPAD(10)),
		P7SPI_INIT_DBGREG(SWB_OPAD(11)),
		P7SPI_INIT_DBGREG(SWB_OPAD(12)),
		P7SPI_INIT_DBGREG(SWB_OPAD(13)),
		P7SPI_INIT_DBGREG(SWB_OPAD(14)),
		P7SPI_INIT_DBGREG(SWB_OPAD(15)),
		P7SPI_INIT_DBGREG(SWB_IPAD_CLK(0)),
		P7SPI_INIT_DBGREG(SWB_IPAD_SS(0)),
		P7SPI_INIT_DBGREG(SWB_IPAD_DATA0(0)),
		P7SPI_INIT_DBGREG(SWB_IPAD_DATA1(0)),
		P7SPI_INIT_DBGREG(SWB_IPAD_DATA2(0)),
		P7SPI_INIT_DBGREG(SWB_IPAD_DATA3(0)),
		P7SPI_INIT_DBGREG(SWB_IPAD_CLK(1)),
		P7SPI_INIT_DBGREG(SWB_IPAD_SS(1)),
		P7SPI_INIT_DBGREG(SWB_IPAD_DATA1(1)),
		P7SPI_INIT_DBGREG(SWB_IPAD_DATA1(1)),
		P7SPI_INIT_DBGREG(SWB_IPAD_DATA2(1)),
		P7SPI_INIT_DBGREG(SWB_IPAD_DATA3(1)),
		P7SPI_INIT_DBGREG(SWB_IPAD_CLK(2)),
		P7SPI_INIT_DBGREG(SWB_IPAD_SS(2)),
		P7SPI_INIT_DBGREG(SWB_IPAD_DATA2(2)),
		P7SPI_INIT_DBGREG(SWB_IPAD_DATA1(2)),
		P7SPI_INIT_DBGREG(SWB_IPAD_DATA2(2)),
		P7SPI_INIT_DBGREG(SWB_IPAD_DATA3(2)),
		P7SPI_INIT_DBGREG(SWB_IPAD_CLK(3)),
		P7SPI_INIT_DBGREG(SWB_IPAD_SS(3)),
		P7SPI_INIT_DBGREG(SWB_IPAD_DATA3(3)),
		P7SPI_INIT_DBGREG(SWB_IPAD_DATA1(3)),
		P7SPI_INIT_DBGREG(SWB_IPAD_DATA2(3)),
		P7SPI_INIT_DBGREG(SWB_IPAD_DATA3(3)),
	};

#ifdef DEBUG
	BUG_ON(! p7spi_kern.dev);
#endif

	p7spi_kern.dbg_dir = debugfs_create_dir(P7SPI_DRV_NAME, NULL);
	if (! p7spi_kern.dbg_dir) {
		dev_warn(p7spi_kern.dev, "failed to create debugfs directory\n");
		return;
	}

	p7spi_kern.dbg_regs.regs = set;
	p7spi_kern.dbg_regs.nregs = ARRAY_SIZE(set);
	p7spi_kern.dbg_regs.base = (void __iomem*) p7spi_kern.vaddr;
	p7spi_kern.dbg_file = debugfs_create_regset32("common",
	                                              S_IRUGO,
	                                              p7spi_kern.dbg_dir,
	                                              &p7spi_kern.dbg_regs);
	if (! p7spi_kern.dbg_file) {
		debugfs_remove(p7spi_kern.dbg_dir);
		p7spi_kern.dbg_dir = NULL;
		dev_warn(p7spi_kern.dev, "failed to create debugfs file\n");
	}
}

static void __devexit p7spi_exit_kern_dbgfs(void)
{
#ifdef DEBUG
	BUG_ON(! p7spi_kern.dev);
#endif

	if (p7spi_kern.dbg_dir) {
		debugfs_remove(p7spi_kern.dbg_file);
		debugfs_remove(p7spi_kern.dbg_dir);
	}
}

static void p7spi_init_core_dbgfs(struct p7spi_core* core)
{
#ifdef DEBUG
	BUG_ON(! core);
	BUG_ON(! p7spi_kern.dev);
#endif

	if (! p7spi_kern.dbg_dir) {
		core->dbg_file = NULL;
		return;
	}

	core->dbg_regs.regs = p7spi_regs;
	core->dbg_regs.nregs = ARRAY_SIZE(p7spi_regs);
	core->dbg_regs.base = (void __iomem*) core->vaddr;
	core->dbg_file = debugfs_create_regset32(dev_name(p7spi_core_dev(core)->parent),
	                                         S_IRUGO,
	                                         p7spi_kern.dbg_dir,
	                                         &core->dbg_regs);
	if (! core->dbg_file)
		dev_warn(core->ctrl->dev.parent, "failed to create debugfs file\n");
}

static void p7spi_exit_core_dbgfs(struct p7spi_core* core)
{
#ifdef DEBUG
	BUG_ON(! core);
	BUG_ON(! p7spi_kern.dev);
#endif

	if (core->dbg_file)
		debugfs_remove(core->dbg_file);
}

#else   /* ! defined(CONFIG_DEBUG_FS) */

static inline void p7spi_init_kern_dbgfs(void) {}
static inline void p7spi_exit_kern_dbgfs(void) {}
static inline void p7spi_init_core_dbgfs(struct p7spi_core* core) {}
static inline void p7spi_exit_core_dbgfs(struct p7spi_core* core) {}

#endif  /* defined(CONFIG_DEBUG_FS) */

static int p7spi_request_pad(struct p7spi_core const * const core,
                             unsigned int pad,
                             enum p7swb_direction dir, enum p7swb_function func)
{
	int core_id = p7spi_core_id(core);

	return p7swb_request_pad(core_id, P7SPI_CORE_SPI, core,
	                           pad, dir, func);
}

static void p7spi_release_pad(struct p7spi_core const * const core,
                              unsigned int pad,
                              enum p7swb_direction dir)
{
	p7swb_release_pad(core, pad, dir);
}

int p7spi_core_index(int core_id, enum p7spi_core_type type)
{
	int i, same_core_type;

	/*
	 * core_id is the index of cores of the same type. We need an
	 * index that is valid in p7spi_kern.cores.
	 */
	for (i = 0, same_core_type = 0; i < p7spi_kern.num_cores; i++) {
		if (p7spi_kern.cores[i] == type) {
			if (same_core_type == core_id)
				return i;
			same_core_type++;
		}
	}

	return -1;
}

static size_t p7spi_fifo_wcnt(u32 mem_reg, int core_id)
{
	size_t wcnt;

	wcnt = (mem_reg >> (core_id * P7SPI_MEM_BITLEN) & P7SPI_MEM_MASK);
	wcnt = ((!! wcnt) * 8U) * (1U << wcnt);
	return wcnt;
}

ssize_t p7spi_alloc_fifo(int core_id, enum p7spi_core_type type, size_t hint)
{
	int i, core_index;
	size_t core_sz, avl_mem = p7spi_kern.fifo_wcnt;
	u32 mem_reg;
	ssize_t ret;

	if (!p7spi_kern.vaddr)
		return -EAGAIN;

	mutex_lock(&p7spi_kern.lck);
	mem_reg = __raw_readl(p7spi_kern.vaddr + P7SPI_MEM);

	core_index = p7spi_core_index(core_id, type);
	if (core_index == -1) {
		ret = -EINVAL;
		goto err;
	}

	/* shortpath if memory already allocated for this core */
	core_sz = p7spi_fifo_wcnt(mem_reg, core_index);
	if (core_sz != 0) {
		ret = core_sz;
		goto err;
	}

	for (i = 0; i < p7spi_kern.num_cores; i++) {
		/* compute fifo size for this core */
		core_sz = p7spi_fifo_wcnt(mem_reg, i);

		/* SPI have TX and RX fifos, mpeg only RX */
		if (p7spi_kern.cores[i] == P7SPI_CORE_SPI)
			core_sz *= 2;

		if (core_sz > avl_mem) {
			ret = -EINVAL;
			goto err;
		} else {
			avl_mem -= core_sz;
		}
	}

	if (!avl_mem) {
		ret = -ENOMEM;
		goto err;
	}

	/*
	 * try to make the fifo fit in available
	 * memory if caller asks for too much.
	 */
	hint = max_t(size_t, hint, 16);
	if (type == P7SPI_CORE_SPI)
		hint *= 2;
	hint = min_t(size_t,  avl_mem, hint);
	if (type == P7SPI_CORE_SPI)
		hint /= 2;

	/* round on valid values */
	if (hint < 16) {
		ret = -EINVAL;
		goto err;
	} else if (hint < 32) {
	      ret = 16;
	} else if (hint < 64) {
	      ret = 32;
	} else {
	      ret = 64;
	}

	dev_dbg(p7spi_kern.dev,
	        "fifo: set fifo size of %d words for core %d\n",
	        ret, core_index);

	mem_reg |= __fls(ret / 8) << (core_index * P7SPI_MEM_BITLEN);
	writel(mem_reg, p7spi_kern.vaddr + P7SPI_MEM);
err:
	mutex_unlock(&p7spi_kern.lck);
	if (ret < 0)
		dev_err(p7spi_kern.dev,
		        "fifo: failed to allocate (%d)\n", ret);
	return ret;
}
EXPORT_SYMBOL(p7spi_alloc_fifo);

void p7spi_free_fifo(int core_id, enum p7spi_core_type type)
{
	int core_index;
	u32 mem_reg;

	BUG_ON(!p7spi_kern.vaddr);

	mutex_lock(&p7spi_kern.lck);
	mem_reg = __raw_readl(p7spi_kern.vaddr + P7SPI_MEM);

	core_index = p7spi_core_index(core_id, type);
	if (core_index == -1)
		goto err;

	mem_reg &= ~(P7SPI_MEM_MASK << (core_index * P7SPI_MEM_BITLEN));
	writel(mem_reg, p7spi_kern.vaddr + P7SPI_MEM);

err:
	mutex_unlock(&p7spi_kern.lck);
}
EXPORT_SYMBOL(p7spi_free_fifo);


/*
 * Current transfer's last unaligned bytes copy (FIFOs only support 32 bits
 * word aligned accesses).
 */
static void p7spi_memcpy_unalign(char* dst, char const* last, size_t bytes)
{
#ifdef DEBUG
	BUG_ON(! dst);
	BUG_ON(! last);
#endif
	switch (bytes) {
	case 3:
		*(dst + 2) = *(last + 2);
	case 2:
		*(dst + 1) = *(last + 1);
	case 1:
		*dst = *last;
		break;
#ifdef DEBUG
	default:
		BUG();
#endif
	}
}

/*
 * 32 bits word aligned receive FIFO data extraction.
 */
void p7spi_readl_fifo(struct p7spi_core* core, size_t bytes)
{
#ifdef DEBUG
	BUG_ON(! bytes);
	BUG_ON(bytes > core->fifo_sz);
	BUG_ON(! IS_ALIGNED((unsigned long) core->rx_buff, sizeof(u32)));
	BUG_ON(! IS_ALIGNED(bytes, sizeof(u32)));
#endif

	for (; bytes; core->rx_buff += sizeof(u32), bytes -= sizeof(u32))
		*((u32*) core->rx_buff) = __raw_readl(core->vaddr + P7SPI_DATA);
}
EXPORT_SYMBOL(p7spi_readl_fifo);

/*
 * Fetch input bytes from receive FIFO.
 */
void p7spi_readb_fifo(struct p7spi_core* core,
                          size_t bytes)
{
	u32 const word = __raw_readl(core->vaddr + P7SPI_DATA);

	p7spi_memcpy_unalign(core->rx_buff, (char const*) &word, bytes);

	core->rx_buff += bytes;
}
EXPORT_SYMBOL(p7spi_readb_fifo);

void p7spi_read_fifo(struct p7spi_core* core, size_t bytes)
{
	size_t const    align = round_down(bytes, sizeof(u32));
	size_t const    unalign = bytes & __round_mask(bytes, sizeof(u32));

	if (align)
		p7spi_readl_fifo(core, align);

	if (unalign)
		p7spi_readb_fifo(core, unalign);
}
EXPORT_SYMBOL(p7spi_read_fifo);

/*
 * Feed transmit FIFO with 32 bits word aligned data.
 */
void p7spi_writel_fifo(struct p7spi_core* core, size_t bytes)
{
#ifdef DEBUG
	BUG_ON(! bytes);
	BUG_ON(bytes > core->fifo_sz);
	BUG_ON(! IS_ALIGNED((unsigned long) core->tx_buff, sizeof(u32)));
	BUG_ON(! IS_ALIGNED(bytes, sizeof(u32)));
#endif

	for (; bytes; core->tx_buff += sizeof(u32), bytes -= sizeof(u32))
		__raw_writel(*((u32 const*) core->tx_buff), core->vaddr + P7SPI_DATA);
}
EXPORT_SYMBOL(p7spi_writel_fifo);

void p7spi_writeb_fifo(struct p7spi_core* core, size_t bytes)
{
	u32 word = 0;

	p7spi_memcpy_unalign((char*) &word, core->tx_buff, bytes);
	__raw_writel(word, core->vaddr + P7SPI_DATA);

	core->tx_buff += bytes;
}
EXPORT_SYMBOL(p7spi_writeb_fifo);

void p7spi_write_fifo(struct p7spi_core* core, size_t bytes)
{
	size_t const    align = round_down(bytes, sizeof(u32));
	size_t const    unalign = bytes & __round_mask(bytes, sizeof(u32));

	if (align)
		p7spi_writel_fifo(core, align);

	if (unalign)
		p7spi_writeb_fifo(core, unalign);
}
EXPORT_SYMBOL(p7spi_write_fifo);

int p7spi_data_ready(struct p7spi_core const* core, u32 stat)
{
	if (! stat)
		/* Hugh ?! No interrupt raised / nothing special happened... */
		return -EAGAIN;

	if (core->rx_buff) {
		if (! (stat & P7SPI_STATUS_RX_TH_REACHED))
			/* Not the one we expected for rx and / or tx. */
			return -ENODATA;
	}
	else if (core->tx_buff) {
		/*
		 * Monitor tx status when tx only to prevent from catching
		 * both rx & tx interrupts.
		 */
		if (! (stat & P7SPI_STATUS_TX_TH_REACHED))
			/* Not the one we expected for tx. */
			return -EIO;
	}
#ifdef DEBUG
	else
		BUG();
#endif

	return 0;
}
EXPORT_SYMBOL(p7spi_data_ready);

int p7spi_poll_stat(struct p7spi_core const* core,
                    int (*check_stat)(struct p7spi_core const*, u32),
                    unsigned long tmout)
{
	u32         stat;
	int         err;

#ifdef DEBUG
	BUG_ON(! p7spi_kern.dev);
	BUG_ON(! core);
	BUG_ON(! check_stat);
#endif

	stat = __raw_readl(core->vaddr + P7SPI_STATUS);
	if (stat & (P7SPI_STATUS_RXACCESS_ERROR |
				P7SPI_STATUS_TXACCESS_ERROR))
		return -EFAULT;

	if (! tmout)
		/* Fast path when caller does not want to block. */
		return (*check_stat)(core, stat);

	tmout += jiffies;
	while (1) {
		err = (*check_stat)(core, stat);
		if (! err)
			break;

		if (time_is_before_jiffies(tmout))
			/*
			 * Timeout elapsed: return to prevent from waiting for a never ending
			 * transfer to prevent from excessive context switches.
			 */
			break;

		/*
		 * Give Linux a chance to preempt us if necessary. cpu_relax is an
		 * alternate solution.
		 */
		cond_resched();

		/* Fetch status again. */
		stat = __raw_readl(core->vaddr + P7SPI_STATUS);
		if (stat & (P7SPI_STATUS_RXACCESS_ERROR |
		            P7SPI_STATUS_TXACCESS_ERROR)) {
			err = -EFAULT;
			break;
		}

	}

	return err;
}
EXPORT_SYMBOL(p7spi_poll_stat);

int p7spi_init_xfer(struct p7spi_core* core,
                    struct spi_transfer* xfer,
                    struct spi_message const* msg __attribute__((unused)),
                    struct spi_master const* dev __attribute__((unused)))
{
#ifdef DEBUG
	BUG_ON(! p7spi_kern.dev);
	BUG_ON(! core);
	BUG_ON(! xfer);
	BUG_ON(! msg);
	BUG_ON(! dev);

	dev_dbg(p7spi_core_dev(core),
	        "\t%uB xfer wr[%p] rd[%p] @%uHz with cs=%d delay=%huusec\n",
	        xfer->len,
	        xfer->tx_buf,
	        xfer->rx_buf,
	        xfer->speed_hz,
	        xfer->cs_change,
			xfer->delay_usecs);

	/* All of this should be properly setup by the generic SPI layer. */
	BUG_ON(! (xfer->tx_buf || xfer->rx_buf));
	BUG_ON(xfer->tx_buf && (dev->flags & SPI_MASTER_NO_TX));
	BUG_ON(xfer->rx_buf && (dev->flags & SPI_MASTER_NO_RX));
	BUG_ON(! xfer->len);
	if (msg->is_dma_mapped) {
		BUG_ON((!! xfer->tx_buf) ^ (!! xfer->tx_dma));
		BUG_ON((!! xfer->rx_buf) ^ (!! xfer->rx_dma));
	}
#endif

	/* Setup SPI bus word width if requested on a per-transfer basis. */
	xfer->bits_per_word = xfer->bits_per_word ? : 8;
	if (xfer->bits_per_word != 8)
		return -EINVAL;

	/* Set internal core fields to handle current transfer rounds. */
	core->tx_buff = xfer->tx_buf;
	core->rx_buff = xfer->rx_buf;
	core->bytes = (size_t) xfer->len;
	core->stat = 0;

	if (msg->is_dma_mapped && core->dma_chan) {
		core->map_dma = false;
		core->dma_addr = xfer->tx_dma ? : xfer->rx_dma;
	}
	else
		core->map_dma = true;

	return 0;
}
EXPORT_SYMBOL(p7spi_init_xfer);

int p7spi_config_dma(struct p7spi_core* core)
{
	int err;

#ifdef DEBUG
	BUG_ON(! p7spi_kern.dev);
	/*
	 * Implement full duplex using dmaengine device_prep_interleaved_dma method
	 * (pl330 driver does not support this interface yet).
	 */
	BUG_ON(! ((!! core->rx_buff) ^ (!! core->tx_buff)));
	BUG_ON(! core->dma_chan);
#endif

	if (core->tx_buff) {
		core->dma_cfg.direction = DMA_MEM_TO_DEV;
		core->dma_cfg.dst_addr = core->fifo_paddr;
	}
	else if (core->rx_buff) {
		core->dma_cfg.direction = DMA_DEV_TO_MEM;
		core->dma_cfg.src_addr = core->fifo_paddr;
	}

	err = dmaengine_slave_config(core->dma_chan, &core->dma_cfg);
	if (err)
		return err;

	INIT_COMPLETION(core->done);
	return 0;
}
EXPORT_SYMBOL(p7spi_config_dma);

ssize_t p7spi_map_dma(struct p7spi_core* core)
{
	char* const buff = core->tx_buff ? (char*) core->tx_buff : core->rx_buff;

	core->dma_sz = min(rounddown(core->bytes, core->dma_min), core->sgm_sz);

#ifdef DEBUG
		BUG_ON(! p7spi_kern.dev);
		/*
		 * No support for full duplex DMA yet.
		 * Implement full duplex using dmaengine device_prep_interleaved_dma
		 * method (pl330 driver does not support this interface yet).
		 */
		BUG_ON(! ((!! core->rx_buff) ^ (!! core->tx_buff)));
		BUG_ON(! core->dma_chan);
		if (core->tx_buff) {
			BUG_ON(core->dma_cfg.direction != DMA_MEM_TO_DEV);
			BUG_ON(core->dma_cfg.dst_addr != core->fifo_paddr);
		}
		else {
			BUG_ON(core->dma_cfg.direction != DMA_DEV_TO_MEM);
			BUG_ON(core->dma_cfg.src_addr != core->fifo_paddr);
		}
		BUG_ON(! core->dma_sz);
		BUG_ON(! IS_ALIGNED((unsigned long) buff, dma_get_cache_alignment()));
		BUG_ON(! IS_ALIGNED(core->dma_sz, dma_get_cache_alignment()));
		BUG_ON(core->dma_sz % core->dma_thres);
#endif

	if (core->map_dma) {
		/*
		 * Buffers submitted to SPI layer MUST be DMA-safe, i.e., appropriate for
		 * dma_map_single usage ! (see Documentation/spi/spi-summary).
		 */
		BUG_ON(! virt_addr_valid(buff));
		BUG_ON(! virt_addr_valid(buff + core->dma_sz - 1));

		/* Map buffer for DMA usage. */
		core->dma_addr = dma_map_single(core->dma_chan->device->dev,
										buff,
										core->dma_sz,
										core->dma_cfg.direction);
		if (unlikely(dma_mapping_error(core->dma_chan->device->dev,
									   core->dma_addr)))
			return -ENOBUFS;
	}

	return core->dma_sz;
}
EXPORT_SYMBOL(p7spi_map_dma);

/* As of v3.4 dmaengin_prep_slave_single is useless... */
inline struct dma_async_tx_descriptor*
p7spi_dmaengine_prep_slave_single(struct dma_chan* chan,
                                  dma_addr_t buf,
                                  size_t len,
                                  enum dma_transfer_direction dir,
                                  unsigned long flags)
{
	struct scatterlist sg;

	sg_init_table(&sg, 1);
	sg_dma_address(&sg) = buf;
	sg_dma_len(&sg) = len;

	return chan->device->device_prep_slave_sg(chan, &sg, 1, dir, flags, NULL);
}
EXPORT_SYMBOL(p7spi_dmaengine_prep_slave_single);

int p7spi_run_dma(struct p7spi_core* core, void (*complete)(void*))
{
	int                                     err;
	struct dma_async_tx_descriptor* const   txd =
		p7spi_dmaengine_prep_slave_single(core->dma_chan,
		                                  core->dma_addr,
		                                  core->dma_sz,
		                                  core->dma_cfg.direction,
		                                  DMA_PREP_INTERRUPT |
		                                  DMA_CTRL_ACK |
		                                  DMA_COMPL_SKIP_SRC_UNMAP |
		                                  DMA_COMPL_SKIP_DEST_UNMAP);
	if (! txd)
		return -ENOMEM;

	txd->callback = complete;
	txd->callback_param = core;

	core->dma_cook = dmaengine_submit(txd);
	err = dma_submit_error(core->dma_cook);
	if (err)
		return err;

	/*
	 * Update bytes counters ahead of time so that we don't need to manage them
	 * at DMA completion time.
	 */
	core->bytes -= core->dma_sz;
	if (core->tx_buff)
		core->tx_buff += core->dma_sz;
	if (core->rx_buff)
		core->rx_buff += core->dma_sz;
	if (! core->map_dma)
		/*
		 * If buffer was already DMA mapped, update DMA bus address only
		 * once used by p7spi_dmaengine_prep_slave_single.
		 */
		core->dma_addr += core->dma_sz;

	dma_async_issue_pending(core->dma_chan);
	return 0;
}
EXPORT_SYMBOL(p7spi_run_dma);

int p7spi_wait_dma(struct p7spi_core* core)
{
	int err = 0;

	if (! wait_for_completion_timeout(&core->done, 15 * HZ))
		err = -ETIMEDOUT;

	if (! err)
		err = p7spi_check_dma(core);

	if (! err)
		err = core->stat;

	if (err)
		p7spi_dump_regs(core, "dma segment failed");

	return err;
}
EXPORT_SYMBOL(p7spi_wait_dma);

int p7spi_setup_core(struct p7spi_core* core,
                     struct spi_device* device,
                     u32* ctrl_reg)
{
	struct p7spi_plat_data const*   pdata;
	struct p7spi_ctrl_data const*   cdata;
	ssize_t                         fifo_sz;

#ifdef DEBUG
	BUG_ON(! p7spi_kern.dev);
	BUG_ON(! core);
	BUG_ON(! device);
	BUG_ON(! device->master);
	BUG_ON(core->dma_cfg.src_addr_width != core->dma_cfg.dst_addr_width);
#endif

	pdata = (struct p7spi_plat_data const*) dev_get_platdata(p7spi_kern.dev);
	cdata = (struct p7spi_ctrl_data const*) device->controller_data;
	if (! (cdata && pdata)) {
		dev_err(p7spi_core_dev(core),
		        "missing platform data for device %s\n",
		        dev_name(&device->dev));
		return -EINVAL;
	}

	if (device->chip_select) {
		dev_err(p7spi_core_dev(core),
		        "invalid slave select %d for device %s "
		        "(single device buses support only)\n",
		        device->chip_select,
		        dev_name(&device->dev));
		return -EINVAL;
	}

	if (! device->bits_per_word)
		/* Zero (the default) here means 8 bits */
		device->bits_per_word = 8;
	if (device->bits_per_word != 8) {
		dev_err(p7spi_core_dev(core),
		        "unsupported word bit width %d for device %s",
		        device->bits_per_word,
		        dev_name(&device->dev));
		return -EINVAL;
	}

	/* allocate some fifo */
	fifo_sz = p7spi_alloc_fifo(p7spi_core_id(core), P7SPI_CORE_SPI,
	                           cdata->fifo_wcnt);
	if (fifo_sz < 0)
		return fifo_sz;
	core->fifo_sz = (size_t)fifo_sz * sizeof(u32);

	/* Compute FIFO wakeup threshold in number of bytes. */
	core->thres_sz = cdata->thres_wcnt * sizeof(u32);
	if (core->thres_sz > (core->fifo_sz - sizeof(u32))) {
		size_t const thres = core->fifo_sz / 2;

		dev_warn(p7spi_core_dev(core),
				 "correcting invalid specified threshold (%u -> %u)",
				 core->thres_sz,
				 thres);
		core->thres_sz = thres;
	}

	if (core->dma_chan) {
		/* Compute DMA burst length and threshold. */
		core->dma_cfg.src_maxburst = min_t(u32,
		                                   core->thres_sz /
										   core->dma_cfg.src_addr_width,
		                                   16);
		core->dma_cfg.dst_maxburst = core->dma_cfg.src_maxburst;
		core->dma_thres = core->dma_cfg.src_maxburst *
		                  core->dma_cfg.src_addr_width;

		if (core->thres_sz != core->dma_thres)
			dev_info(p7spi_core_dev(core),
					 "aligning DMA burst threshold (%u -> %u)",
					 core->thres_sz,
					 core->dma_thres);

		core->dma_min = lcm(core->dma_thres, dma_get_cache_alignment());
		/* Compute maximum segment size. */
		core->sgm_sz = rounddown(P7SPI_INSTR_LEN_MASK, core->dma_min);
	}
	else
		core->sgm_sz = rounddown(P7SPI_INSTR_LEN_MASK, core->thres_sz);

	if (device->mode & SPI_CPHA)
		*ctrl_reg |= P7SPI_CTRL_CPHA;
	if (device->mode & SPI_CPOL)
		*ctrl_reg |= P7SPI_CTRL_CPOL;
	if (device->mode & SPI_LSB_FIRST)
		*ctrl_reg |= P7SPI_CTRL_LSB;

	return 0;
}
EXPORT_SYMBOL(p7spi_setup_core);

static int p7spi_enable_swb(struct p7spi_core const* core,
                            struct p7spi_swb* switchbox)
{
	struct p7spi_swb const* swb;
	int err = 0;

	swb = switchbox;
	while (swb->pad != -1) {
		err = p7spi_request_pad(core, swb->pad, swb->direction, swb->function);
		if (err) {
			struct p7spi_swb const* swb_rel = switchbox;
			while (swb_rel != swb) {
				p7spi_release_pad(core, swb_rel->pad, swb_rel->direction);
				swb_rel++;
			}

			goto finish;
		}
		swb++;
	}

finish:
	return err;
}

static void p7spi_release_swb(struct p7spi_core const* core,
                              struct p7spi_swb* switchbox)
{
	struct p7spi_swb const* swb;

	swb = switchbox;
	while (swb->pad != -1) {
		p7spi_release_pad(core, swb->pad, swb->direction);
		swb++;
	}
}

void p7spi_enable_core(struct p7spi_core const* core,
                       struct spi_device const* device,
                       u32 ctrl_reg)
{
	size_t const                    wcnt = core->thres_sz / sizeof(u32);

#ifdef DEBUG
	BUG_ON(! p7spi_kern.dev);
#endif

	__raw_writel(0, core->vaddr + P7SPI_ITEN);
	__raw_writel(P7SPI_FIFORX_FLUSH | P7SPI_FIFOTX_FLUSH,
	             core->vaddr + P7SPI_FIFO_FLUSH);

	__raw_writel((core->fifo_sz / sizeof(u32)) - wcnt,
	             core->vaddr + P7SPI_TH_TX);
	__raw_writel(wcnt, core->vaddr + P7SPI_TH_RX);

	writel(ctrl_reg, core->vaddr + P7SPI_CTRL);
}
EXPORT_SYMBOL(p7spi_enable_core);

static void p7spi_cleanup_core(struct spi_device* device)
{
	struct p7spi_core const* const core = spi_master_get_devdata(device->master);

#ifdef DEBUG
	BUG_ON(! p7spi_kern.dev);
#endif

	p7spi_free_fifo(p7spi_core_id(core), P7SPI_CORE_SPI);
	__raw_writel(0, core->vaddr + P7SPI_CTRL);
}

static bool p7spi_filter(struct dma_chan* chan, void* param)
{
	if (chan->chan_id == (unsigned int) param)
		return true;

	return false;
}

static int p7spi_init_core(struct p7spi_core* core,
                           struct platform_device* pdev,
                           struct spi_master* ctrl,
                           struct p7spi_ops const* ops)
{
	int                                 err;
	struct resource const*              res;
	unsigned long                       irqfl = 0;
	char                                stringify_core[2];

#ifdef DEBUG
	BUG_ON(! core);
	BUG_ON(! pdev);
#endif

	if (! p7spi_kern.dev)
		return -ENODEV;

	core->ctrl = ctrl;

	core->irq = -1;
	if (ops->handle_irq) {
		res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

		if (res) {
			core->irq = res->start;
			if (res->flags & IORESOURCE_IRQ_SHAREABLE)
				irqfl |= IRQF_SHARED;
		}
		else
			dev_warn(&pdev->dev,
			         "failed to get core interrupt, "
			         "fall back to polling mode\n");
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (! res) {
		dev_err(&pdev->dev, "failed to get core memory region\n");
		return -ENXIO;
	}

	core->regs = request_mem_region(res->start,
	                                resource_size(res),
	                                dev_name(&pdev->dev));
	if (! core->regs) {
		dev_err(&pdev->dev,
		        "failed to request core memory region [%08x:%08x]\n",
				res->start,
				res->end);
		return -EBUSY;
	}

	core->vaddr = (unsigned long) ioremap(res->start, resource_size(res));
	if (! core->vaddr) {
		dev_err(&pdev->dev,
		        "failed to remap core memory region [%08x:%08x]\n",
				res->start,
				res->end);
		err = -ENXIO;
		goto release;
	}
	core->fifo_paddr = res->start + P7SPI_DATA;

	err = snprintf(stringify_core, 2, "%d", p7spi_core_id(core));
	if (err < 0) {
		dev_err(&pdev->dev,
		        "failed to stringify core id\n");
		goto release;
	}

	core->clk = clk_get_sys(stringify_core, P7SPI_DRV_NAME);
	if (IS_ERR(core->clk)) {
		err = PTR_ERR(core->clk);
		dev_err(&pdev->dev,
		        "failed to get core clock\n");
		goto release;
	}

	err = clk_prepare_enable(core->clk);
	if (err) {
		dev_err(&pdev->dev,
		        "failed to enable core clock\n");
		goto put;
	}

	if (core->irq >= 0) {
		err = request_irq(core->irq,
						  ops->handle_irq,
						  irqfl,
						  dev_name(&pdev->dev),
						  core);
		if (err) {
			dev_warn(&pdev->dev,
			         "failed to request irq %d, fall back to polling mode (%d)\n",
			         core->irq,
			         err);
			core->irq = -1;
		}
	}

	core->dma_chan = NULL;
	if (ops->dma) {
		res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
		if (res) {
            dma_cap_mask_t  mask;

            dma_cap_zero(mask);
            dma_cap_set(DMA_SLAVE, mask);
            if (ops->dma_cyclic)
	            dma_cap_set(DMA_CYCLIC, mask);
            core->dma_chan = dma_request_channel(mask,
                                                 p7spi_filter,
                                                 (void*) res->start);
            if (! core->dma_chan)
                dev_warn(&pdev->dev,
                         "failed to request DMA channel %d, "
                         "fall back to PIO mode\n",
                         res->start);

			/*
			 * Setup DMA configuration fields remaining constant over controller
			 * life-cycle.
			 */
			core->dma_cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_8_BYTES;
			core->dma_cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_8_BYTES;
		}
		else
			dev_warn(&pdev->dev,
			         "unspecified DMA channel, fall back to PIO mode\n");
	}

	init_completion(&core->done);

	if (core->dma_chan)
		dev_dbg(&pdev->dev,
				"loaded core %d @0x%08x with irq=%d and dma channel %d\n",
				pdev->id,
				core->regs->start,
				core->irq,
				core->dma_chan->chan_id);
	else
		dev_dbg(&pdev->dev,
				"loaded core %d @0x%08x with irq=%d in PIO mode\n",
				pdev->id,
				core->regs->start,
				core->irq);
	return 0;

put:
	clk_put(core->clk);
release:
	release_mem_region(core->regs->start, resource_size(core->regs));
	return err;
}

static void p7spi_exit_core(struct p7spi_core* core)
{
#ifdef DEBUG
	BUG_ON(! core);
	BUG_ON(! p7spi_kern.dev);
#endif

	if (core->dma_chan)
		dma_release_channel(core->dma_chan);

	if (core->irq >= 0)
		free_irq(core->irq, (void*) core);

	__raw_writel(0, core->vaddr + P7SPI_CTRL);

	clk_disable_unprepare(core->clk);
	clk_put(core->clk);
	iounmap((void*) core->vaddr);
	release_mem_region(core->regs->start, resource_size(core->regs));
}

static int p7spi_prepare_xfer(struct spi_master* ctrl)
{
	pm_runtime_get_sync(p7spi_core_dev((struct p7spi_core*)
	                                   spi_master_get_devdata(ctrl))->parent);
	return 0;
}

static int p7spi_unprepare_xfer(struct spi_master* ctrl)
{
	pm_runtime_put(p7spi_core_dev((struct p7spi_core*)
	                              spi_master_get_devdata(ctrl))->parent);
	return 0;
}

int _p7spi_create_ctrl(struct platform_device* pdev,
                       struct p7spi_ops const* ops,
					   size_t ctrl_sz)
{
	struct spi_master*  ctrl;
	struct p7spi_core*  core;
	int                 err;

#ifdef DEBUG
	BUG_ON(! pdev);
	BUG_ON(! ops);
	BUG_ON(! ops->setup);
	BUG_ON(! ops->xfer);
	BUG_ON(! ctrl_sz);
#endif

	ctrl = spi_alloc_master(&pdev->dev, ctrl_sz);
	if (! ctrl) {
		dev_err(&pdev->dev, "failed to allocate controller memory\n");
		return -ENOMEM;
	}

	ctrl->bus_num = pdev->id;
	ctrl->num_chipselect = 1;
	ctrl->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST | SPI_3WIRE;
	ctrl->setup = ops->setup;
	ctrl->cleanup = p7spi_cleanup_core;
	ctrl->prepare_transfer_hardware = p7spi_prepare_xfer;
	ctrl->transfer_one_message = ops->xfer;
	ctrl->unprepare_transfer_hardware = p7spi_unprepare_xfer;
	ctrl->rt = ops->rt;

	core = (struct p7spi_core*) spi_master_get_devdata(ctrl);
	err = p7spi_init_core(core, pdev, ctrl, ops);
	if (err)
		goto put;

	core->pctl = pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(core->pctl)) {
		dev_err(&pdev->dev, "failed to enable pins\n");
		err = PTR_ERR(core->pctl);
		goto exit;
	}

	err = p7spi_enable_swb(core,
	                       (struct p7spi_swb*) pdev->dev.platform_data);
	if (err)
		goto put_pin;

	platform_set_drvdata(pdev, core);
	p7spi_init_core_dbgfs(core);

	return 0;

put_pin:
	pinctrl_put(core->pctl);
exit:
	p7spi_exit_core(core);
put:
	spi_master_put(ctrl);
	return err;
}
EXPORT_SYMBOL(_p7spi_create_ctrl);

int p7spi_register(struct platform_device* pdev, struct p7spi_core *core)
{
	struct spi_master *ctrl = core->ctrl;

	int err = spi_register_master(ctrl);
	if (err) {
		dev_err(&pdev->dev, "failed to register controller.\n");
		return err;
	}

	dev_info(&pdev->dev, "core %d ready\n", pdev->id);
	return 0;
}
EXPORT_SYMBOL(p7spi_register);

int p7spi_abort_create(struct platform_device* pdev)
{
	struct p7spi_core* const    core = platform_get_drvdata(pdev);

	p7spi_exit_core_dbgfs(core);
	p7spi_exit_core(core);
	pinctrl_put(core->pctl);

	platform_set_drvdata(pdev, NULL);
	spi_master_put(core->ctrl);

	dev_info(&pdev->dev, "core %d removed\n", pdev->id);
	return 0;
}
EXPORT_SYMBOL(p7spi_abort_create);

int p7spi_destroy_ctrl(struct platform_device* pdev)
{
	struct p7spi_core* const    core = platform_get_drvdata(pdev);

#ifdef DEBUG
	BUG_ON(! core);
	BUG_ON(! core->ctrl);
	BUG_ON(! spi_master_get_devdata(core->ctrl));
#endif

	p7spi_exit_core_dbgfs(core);

	spi_unregister_master(core->ctrl);
	p7spi_exit_core(core);
	p7spi_release_swb(core, (struct p7spi_swb*) pdev->dev.platform_data);
	pinctrl_put(core->pctl);

	platform_set_drvdata(pdev, NULL);
	spi_master_put(core->ctrl);

	dev_info(&pdev->dev, "core %d removed\n", pdev->id);
	return 0;
}
EXPORT_SYMBOL(p7spi_destroy_ctrl);

static int __devinit p7spi_probe_kern(struct platform_device* pdev)
{
	int                                 err, i __maybe_unused;
	struct resource const*              res;
	char const*                         msg;
	struct p7spi_plat_data const* const pdata = (struct p7spi_plat_data const*)
	                                            dev_get_platdata(&pdev->dev);

#ifdef DEBUG
	/*
	 * Check fifo size settings consistency for all cores.
	 */
	BUG_ON(! pdata);
	BUG_ON(! pdata->wcnt);
	BUG_ON(! pdata->max_master_hz);
	BUG_ON(! pdata->max_slave_hz);
	BUG_ON(! pdata->min_hz);
	BUG_ON(p7spi_kern.dev);
#endif

	p7spi_kern.num_cores = pdata->num_cores;
	p7spi_kern.cores = kmemdup(pdata->cores,
	                       pdata->num_cores * sizeof(enum p7spi_core_type),
	                       GFP_KERNEL);
	if (!p7spi_kern.cores) {
		err = -ENOMEM;
		msg = "no memory to store cores";
		goto err;
	}

	p7spi_kern.fifo_wcnt = pdata->wcnt;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (! res) {
		err = -ENXIO;
		msg = "failed to get common memory region";
		goto free_cores;
	}

	p7spi_kern.res = request_mem_region(res->start,
	                                    resource_size(res),
	                                    dev_name(&pdev->dev));
	if (! p7spi_kern.res) {
		err = -EBUSY;
		msg = "failed to request common memory region";
		goto free_cores;
	}

	p7spi_kern.vaddr = (unsigned long) ioremap(res->start,
	                                           resource_size(res));
	if (! p7spi_kern.vaddr) {
		err = -ENXIO;
		msg = "failed to remap common memory region";
		goto release;
	}

	p7spi_kern.clk = clk_get_sys("common", P7SPI_DRV_NAME);
	if (IS_ERR(p7spi_kern.clk)) {
		err = PTR_ERR(p7spi_kern.clk);
		msg = "failed to get clock";
		goto unmap;
	}

	err = clk_prepare_enable(p7spi_kern.clk);
	if (err) {
		msg = "failed to enable clock";
		goto put;
	}

	p7spi_kern.swb = kzalloc(
	            pdata->num_pads * sizeof(struct p7swb_desc),
	            GFP_KERNEL);
	if (!p7spi_kern.swb) {
		err = -ENOMEM;
		msg = "failed to allocate memory for switchbox";
		goto unprepare;
	}
	p7spi_kern.swb_sz = pdata->num_pads;

#ifdef CONFIG_PM_SLEEP
	p7spi_kern.nb_swb_in = 0;
	for (i = 0; i < p7spi_kern.num_cores; i++) {
		p7spi_kern.nb_swb_in += p7swb_core_offset(p7spi_kern.cores[i]);
	}

	p7spi_kern.saved_swb_in = kmalloc(p7spi_kern.nb_swb_in, GFP_KERNEL);
	p7spi_kern.saved_swb_out = kmalloc(p7spi_kern.swb_sz, GFP_KERNEL);

	if (!p7spi_kern.saved_swb_in || !p7spi_kern.saved_swb_out) {
		kfree(p7spi_kern.saved_swb_in);
		kfree(p7spi_kern.saved_swb_out);
		kfree(p7spi_kern.swb);

		err = -ENOMEM;
		msg = "failed to allocate memory for switchbox suspend";
		goto unprepare;
	}
#endif

	/* Mark all fifo as available */
	__raw_writel(0, p7spi_kern.vaddr + P7SPI_MEM);
	p7spi_kern.rev = pdata->rev;
	p7spi_kern.dev = &pdev->dev;

	dev_info(&pdev->dev, "Loaded SPI revision %d\n", p7spi_kern.rev);

	p7spi_init_kern_dbgfs();
	return 0;

unprepare:
	clk_disable_unprepare(p7spi_kern.clk);
put:
	clk_put(p7spi_kern.clk);
unmap:
	iounmap((void*) p7spi_kern.vaddr);
release:
	release_mem_region(res->start, resource_size(res));
free_cores:
	kfree(p7spi_kern.cores);
err:
	dev_err(&pdev->dev, "%s\n", msg);
	return err;
}

static int __devexit p7spi_remove_kern(struct platform_device* pdev)
{
	p7spi_exit_kern_dbgfs();

#ifdef CONFIG_PM_SLEEP
	kfree(p7spi_kern.saved_swb_in);
	kfree(p7spi_kern.saved_swb_out);
#endif

	kfree(p7spi_kern.swb);

	clk_disable_unprepare(p7spi_kern.clk);
	clk_put(p7spi_kern.clk);

	iounmap((void*) p7spi_kern.vaddr);
	release_mem_region(p7spi_kern.res->start, resource_size(p7spi_kern.res));

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int p7spi_suspend(struct device *dev)
{
	int i, word_offset;

	p7spi_kern.saved_fifo_sz = __raw_readl(p7spi_kern.vaddr + P7SPI_MEM);

	for (i = 0; i < p7spi_kern.swb_sz; i++) {
		word_offset = i * 4;
		p7spi_kern.saved_swb_out[i] =
			(u8) __raw_readl(p7spi_kern.vaddr + 0x100 + word_offset);
	}
	for (i = 0; i < p7spi_kern.nb_swb_in; i++) {
		word_offset = i * 4;
		p7spi_kern.saved_swb_in[i] =
			(u8) __raw_readl(p7spi_kern.vaddr + 0x200 + word_offset);
	}

	clk_disable_unprepare(p7spi_kern.clk);

	return 0;
}

static int p7spi_resume(struct device *dev)
{
	int i, word_offset;

	clk_prepare_enable(p7spi_kern.clk);

	__raw_writel(p7spi_kern.saved_fifo_sz,
	             p7spi_kern.vaddr + P7SPI_MEM);

	for (i = 0; i < p7spi_kern.swb_sz; i++) {
		word_offset = i * 4;
		__raw_writel(p7spi_kern.saved_swb_out[i],
		             p7spi_kern.vaddr + 0x100 + word_offset);
	}
	for (i = 0; i < p7spi_kern.nb_swb_in; i++) {
		word_offset = i * 4;
		__raw_writel(p7spi_kern.saved_swb_in[i],
		             p7spi_kern.vaddr + 0x200 + word_offset);
	}

	return 0;
}
#else
#define p7spi_suspend   NULL
#define p7spi_resume    NULL
#endif

static struct dev_pm_ops p7spi_dev_pm_ops = {
	.suspend  = p7spi_suspend,
	.resume   = p7spi_resume,
};

static struct platform_driver p7spi_kern_driver = {
	.driver = {
		.name   = P7SPI_DRV_NAME,
		.bus    = &spi_bus_type,
		.owner  = THIS_MODULE,
		.pm     = &p7spi_dev_pm_ops,
	},
	.probe		= p7spi_probe_kern,
	.remove		= __devexit_p(p7spi_remove_kern)
};
module_platform_driver(p7spi_kern_driver);

MODULE_AUTHOR("Gregor Boirie <gregor.boirie@parrot.com>");
MODULE_DESCRIPTION("Parrot7 SPI shared resources driver");
MODULE_LICENSE("GPL");
