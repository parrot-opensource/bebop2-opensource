/**
 *
 *       @file  p7-spi_int.h
 *
 *      @brief  
 *
 *     @author  Gregor Boirie <gregor.boirie@parrot.com>
 *       @date  26-Jul-2012
 *
 *        $Id:$
 *
 *       Copyright (C) 2008 Parrot S.A.
 */

#ifndef _P7_SPI_PRIV_H
#define _P7_SPI_PRIV_H

#if ! defined(CONFIG_SPI_PARROT7) && ! defined(CONFIG_SPI_PARROT7_MODULE)
#error Cannot build disabled Parrot7 SPI driver !
#endif

#include <linux/debugfs.h>
#include <linux/pinctrl/consumer.h>
#include <linux/dma-mapping.h>
#include "p7-spi_regs.h"

/**********************
 * SPI kernel handling
 **********************/

struct clk;
struct resource;

struct p7spi_kernel {
	struct mutex                lck;
	unsigned long               vaddr;
	struct device*              dev;
	struct resource*            res;
	struct clk*                 clk;
	unsigned int                rev;
	unsigned int                fifo_wcnt;
#if defined(CONFIG_DEBUG_FS)
	struct dentry*              dbg_dir;
	struct dentry*              dbg_file;
	struct debugfs_regset32     dbg_regs;
#endif
	enum p7spi_core_type*       cores;
	unsigned int                num_cores;
	struct p7swb_desc*          swb;
	unsigned int                swb_sz;
#ifdef CONFIG_PM_SLEEP
	u32                         saved_fifo_sz;
	unsigned int                nb_swb_in;
	u8                          *saved_swb_in;
	u8                          *saved_swb_out;
#endif
};

extern struct p7spi_kernel p7spi_kern;

/*****************
 * Cores handling
 *****************/

struct dma_chan;
struct device;
struct platform_device;
struct p7spi_swb;

/*
 * Reserve enough scatter-gather entries to cope with the maximum
 * transfer size a single instruction may handle. Each entry will
 * be mapped onto one physical memory page on demand.
 */
#define P7SPI_SGNR_MAX  (DIV_ROUND_UP(P7SPI_INSTR_LEN_MASK, PAGE_SIZE) + 1)

struct p7spi_core {
	/* Number of completed bytes for current transfer */
	size_t                      bytes;
	/* Transmit buffer for current transfer */
	char const*                 tx_buff;
	/* Receive buffer for current transfer */
	char*                       rx_buff;
	unsigned long               vaddr;
	/* Message completion token. */
	struct completion           done;
	unsigned long               fifo_paddr;
	struct dma_chan*            dma_chan;
	bool                        map_dma;
	struct dma_slave_config     dma_cfg;
	dma_cookie_t                dma_cook;
	dma_addr_t                  dma_addr;
	size_t                      dma_sz;
	size_t                      dma_thres;
	size_t                      dma_min;
	size_t                      fifo_sz;
	/* Transfer bytes threshold */
	size_t                      thres_sz;
	/* Maximum number of bytes that can be process by a single segment. */
	size_t                      sgm_sz;
	/* Current transfer hardware status */
	int                         stat;
	struct spi_master*          ctrl;
	struct clk*                 clk;
	struct pinctrl*             pctl;
	struct resource const*      regs;
	int                         irq;
#if defined(CONFIG_DEBUG_FS)
	struct dentry*              dbg_file;
	struct debugfs_regset32     dbg_regs;
#endif
};

static inline int p7spi_core_id(struct p7spi_core const* core)
{
#ifdef DEBUG
	BUG_ON(! core->ctrl);
	BUG_ON(! p7spi_kern.dev);
#endif

	return (int) core->ctrl->bus_num;
}

static inline struct device* p7spi_core_dev(struct p7spi_core const* core)
{
#ifdef DEBUG
	BUG_ON(! core->ctrl);
	BUG_ON(! p7spi_kern.dev);
#endif

	return &core->ctrl->dev;
}

extern void p7spi_readb_fifo(struct p7spi_core*, size_t);
extern void p7spi_writeb_fifo(struct p7spi_core*, size_t);
extern void p7spi_readl_fifo(struct p7spi_core*, size_t);
extern void p7spi_writel_fifo(struct p7spi_core*, size_t);
extern void p7spi_read_fifo(struct p7spi_core*, size_t);
extern void p7spi_write_fifo(struct p7spi_core*, size_t);

#if 0
extern ssize_t p7spi_start_dma(struct p7spi_core*,
                               struct dma_slave_config*,
                               dma_cookie_t*,
                               void (*)(void*));

extern int p7spi_wait_dma(struct p7spi_core*,
                          struct dma_slave_config const*,
                          dma_cookie_t,
                          int (*check_stat)(struct p7spi_core const*, u32),
                          size_t);
#endif

extern void p7spi_enable_core(struct p7spi_core const*,
                              struct spi_device const*,
                              u32 ctrl_reg);

extern int p7spi_setup_core(struct p7spi_core*,
                            struct spi_device*,
                            u32*);

struct p7spi_ops {
	int           (*setup)(struct spi_device*);
	int           (*xfer)(struct spi_master*, struct spi_message*);
	irq_handler_t   handle_irq;
	bool            rt;
	bool            dma;
	bool            dma_cyclic;
};

extern int _p7spi_create_ctrl(struct platform_device*,
                              struct p7spi_ops const*,
                              size_t);

#define p7spi_create_ctrl(_pdev, _ops, _type)           \
	     _p7spi_create_ctrl(_pdev,                      \
	                        _ops,                       \
						    sizeof(_type))

extern int p7spi_register(struct platform_device*,
						  struct p7spi_core *);

extern int p7spi_abort_create(struct platform_device* pdev);

extern int p7spi_destroy_ctrl(struct platform_device*);

#ifdef DEBUG

/* All of this should be properly setup by the generic SPI layer. */
#define P7SPI_ASSERT_MSG(_msg, _ctrl, _spi, _dev)   \
	BUG_ON(! p7spi_kern.dev);                       \
	BUG_ON(! _msg);                                 \
	BUG_ON(! _ctrl);                                \
	BUG_ON(! _spi);                                 \
	BUG_ON(! _dev);                                 \
	BUG_ON(! (_dev)->master);                       \
	BUG_ON((_dev)->master != _spi);                 \
	BUG_ON((_dev)->chip_select);                    \
	BUG_ON((_dev)->mode & ~(_spi)->mode_bits);      \
	BUG_ON((_dev)->bits_per_word != 8);             \
	BUG_ON(((_spi)->flags & SPI_MASTER_NO_RX) &&    \
	       ((_spi)->flags & SPI_MASTER_NO_TX));     \
	BUG_ON((_msg)->actual_length);                  \
	BUG_ON(list_empty(&(_msg)->transfers));         \
	dev_dbg(p7spi_core_dev(&(_ctrl)->core),         \
	        "%smsg xfer requested for device %s\n", \
	        (_msg)->is_dma_mapped ?                 \
	            "DMA mapped " : "",                 \
			dev_name(&(_dev)->dev))


extern void p7spi_dump_regs(struct p7spi_core const*, char const*);

#else

#define P7SPI_ASSERT_MSG(_msg, _ctrl, _spi, _dev)
#define p7spi_dump_regs(_core, _msg)

#endif

struct spi_transfer;
struct spi_message;
struct spi_master;

extern int p7spi_init_xfer(struct p7spi_core*,
                           struct spi_transfer*,
                           struct spi_message const* __attribute__((unused)),
                           struct spi_master const*  __attribute__((unused)));

extern int p7spi_data_ready(struct p7spi_core const*, u32);

extern int p7spi_poll_stat(struct p7spi_core const*,
                           int (*check_stat)(struct p7spi_core const*, u32),
                           unsigned long);

static inline bool p7spi_has_dma(struct p7spi_core const* core)
{
	return core->dma_chan;
}

static inline bool p7spi_has_irq(struct p7spi_core const* core)
{
	return core->irq >= 0;
}

int p7spi_core_index(int core_id, enum p7spi_core_type type);
extern ssize_t p7spi_alloc_fifo(int, enum p7spi_core_type, size_t);
extern void p7spi_free_fifo(int core_id, enum p7spi_core_type type);

static inline char const* p7spi_mode_name(enum p7spi_xfer_mode mode)
{
	switch (mode) {
	case P7SPI_SINGLE_XFER:
		return "single";
	case P7SPI_DUAL_XFER:
		return "dual";
	case P7SPI_QUAD_XFER:
		return "quad";
	}

#ifdef DEBUG
	BUG();
#endif
	return NULL;
}

/*************************
 * DMA related operations
 *************************/

extern int p7spi_config_dma(struct p7spi_core*);
extern int p7spi_map_dma(struct p7spi_core*);
extern int p7spi_run_dma(struct p7spi_core*, void (*complete)(void*));
extern int p7spi_wait_dma(struct p7spi_core*);
extern struct dma_async_tx_descriptor*
p7spi_dmaengine_prep_slave_single(struct dma_chan* chan,
                                  dma_addr_t buf,
                                  size_t len,
                                  enum dma_transfer_direction dir,
                                  unsigned long flags);

static inline void p7spi_unmap_dma(struct p7spi_core* core)
{
	if (core->map_dma)
		dma_unmap_single(core->dma_chan->device->dev,
						 core->dma_addr,
						 core->dma_sz,
						 core->dma_cfg.direction);
}

static inline int p7spi_check_dma(struct p7spi_core* core)
{
	if (likely(dma_async_is_tx_complete(core->dma_chan,
	                                    core->dma_cook,
	                                    NULL,
	                                    NULL) == DMA_SUCCESS))
		return 0;
	return -EBADFD;
}

static inline void p7spi_cancel_dma(struct p7spi_core* core)
{
	dmaengine_terminate_all(core->dma_chan);
}

static inline void p7spi_cook_dmaif(struct p7spi_core const* core)
{
	/*
	 * Setup wakeup threshold to synchronize SPI and DMA controller transfers.
	 */
	__raw_writel(core->dma_thres / sizeof(u32),
	             core->vaddr + P7SPI_TH_RX);
	__raw_writel((core->fifo_sz - core->dma_thres) / sizeof(u32),
	             core->vaddr + P7SPI_TH_TX);
}

static inline void p7spi_uncook_dmaif(struct p7spi_core const* core)
{
	/*
	 * Setup wakeup threshold for interrupt / polling mode.
	 */
	__raw_writel(core->thres_sz / sizeof(u32),
	             core->vaddr + P7SPI_TH_RX);
	__raw_writel((core->fifo_sz - core->thres_sz) / sizeof(u32),
				 core->vaddr + P7SPI_TH_TX);
}

#endif
