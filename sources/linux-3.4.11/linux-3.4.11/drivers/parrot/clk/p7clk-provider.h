/*
 *  drivers/parrot/clk/p7clk-provider.h
 *
 *  Copyright (c) 2010-2013
 *  
 *  @author  Gregor Boirie <gregor.boirie@parrot.com>
 *  @author  Damien Riegel <damien.riegel.ext@parrot.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _P7_CLK_PROVIDER_H
#define _P7_CLK_PROVIDER_H

#include <linux/clk-provider.h>
#include <linux/io.h>

#ifdef CONFIG_COMMON_CLK

/**
 * struct p7_clk_gbc - GBC clock descriptor.
 * @clk_reg: virtual address of register used to enable clock
 * @clk_msk: mask used to enable/disable clock
 * @reset_reg: virtual address of register used to reset child block
 *
 * @p7_clk_gbc defines GBC informations required to perform clock and reset
 * related operations.
 * @reset_reg is optional and unused if null
 */
struct p7_clk_gbc {
	unsigned long   clk_reg;
	u32             clk_msk;
	unsigned long   reset_reg;
	u32             reset_offset;
	unsigned long   power_reg;
	u32		power_msk;
};

/**
 * struct clk_p7_gbc - Gate and reset individual IPs
 * @gbc: gbc info for this clock
 * @lock: spinlock pointer
 */
struct clk_p7_gbc {
	struct clk_hw               hw;
	struct p7_clk_gbc           gbc;
	spinlock_t*                 lock;
};

extern struct clk_ops const p7_clk_gbc_ops;
extern struct clk *clk_register_gbc(struct device*, const char *,
				const char *, unsigned long,
				unsigned long, u32, unsigned long, u32,
				unsigned long, spinlock_t*);


/**
 * struct clk_p7_pll - One level (master) PLL descriptor.
 * @gateable: boolean to indicate if this PLL is gateable
 * @lock_reg: address of the lock register
 * @lock_bit_idx: lock-bit index in @lock_reg
 * @pll_conf_reg: PLL configuration register address
 * @cfg_nr: @pll_cfg number of entries
 * @cfg_idx: PLLs frequency table
 * @pll_cfg: table of possible PLL's frequency configurations
 * @lock: spinlock pointer
 */
struct clk_p7_pll {
	struct clk_hw               hw;
	bool                        gateable;
	unsigned long               lock_reg;
	u8                          lock_bit_idx;
	unsigned long               pll_conf_reg;
	size_t                      cfg_nr;
	unsigned long const*        cfg_idx;
	unsigned long const*        pll_cfg;
	spinlock_t*                 lock;
};

extern struct clk_ops const p7_clk_pll_ops;
extern struct clk_ops const p7_clk_fixed_pll_ops;
extern struct clk *clk_register_pll(struct device *, const char *,
                                    const char *, const bool, const unsigned long,
                                    const unsigned long, const u8,
                                    const unsigned long, unsigned long const * const,
                                    unsigned long const * const, const size_t, spinlock_t *);
/**
 * struct p7_avipll - Two level AVI PLL descriptor.
 * @lock_reg: address of the lock register
 * @lock_bit_idx: lock-bit index in @lock_reg
 * @pll_conf_reg: PLL configuration register address
 * @cfg_nr: PLLs configuration table number of entries
 * @cfg_idx: PLLs frequency table
 * @master_cfg: table of first level possible PLL's frequency configurations
 * @slave_cfg: table of second level possible PLL's frequency configurations
 * @lock: spinlock pointer
 */
struct clk_p7_avipll {
	struct clk_hw               hw;
	unsigned long               lock_reg;
	u8                          lock_bit_idx;
	unsigned long               pll_conf_reg;
	size_t                      cfg_nr;
	unsigned long const*        cfg_idx;
	unsigned long const*        master_cfg;
	unsigned long const*        slave_cfg;
	spinlock_t*                 lock;
};

extern struct clk_ops const p7_clk_avipll_ops;
extern struct clk *clk_register_avipll(struct device *, const char *,
                                       const char *, const unsigned long,
                                       const unsigned long, const u8,
				       const unsigned long, unsigned long const * const,
				       unsigned long const * const, unsigned long const * const,
				       const size_t, spinlock_t *);

/**
 * struct clk_p7_xin - SDIO Xin clock
 * @clk_reg: register address
 */
struct clk_p7_xin {
	struct clk_hw       hw;
	unsigned long       clk_reg;
};

extern struct clk_ops const p7_clk_xin_ops;
extern struct clk *clk_register_xin(struct device *, const char *,
                                    const char *, const unsigned long);

#define SDIOx_XIN_CLK_EN        (1UL)
#define SDIOx_XIN_CLK_DIV_SFT   (1UL)
#define SDIOx_XIN_CLK_DIV_MSK   (0xfUL)
#define SDIOx_XIN_CLK_RATE_MAX  (260000000UL)

extern struct clk *clk_register_sd(struct device *, const char *, const char *);

#define P7_POWER_OFF (1 << 2)
#define P7_POWER_ON  (1 << 1)
#define P7_POWER_REQ (1 << 0)
#define P7_REQUEST_POWER_ON 1
#define P7_REQUEST_POWER_OFF 0

#endif /* CONFIG_COMMON_CLK */
#endif /* _P7_CLK_PROVIDER_H */
