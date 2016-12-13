/**
 * linux/driver/parrot/mmc/acs3-sdhci.h - Arasan Chip Systems eMMC / SD / SDIO
 *                                        host driver interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    10-Sep-2012
 *
 * This file is released under the GPL
 *
 * See linux/drivers/parrot/mmc/acs3-sdhci.c file header
 */

#ifndef _ACS3_SDHCI_H
#define _ACS3_SDHCI_H

#include "../pinctrl/p7-pinctrl.h"

#define ACS3_DRV_NAME   "acs3-sdhci"

/**
 * struct acs3_regs - acs3 host registers addresses
 * tdl1:   TDL1 config register
 * tdl2:   TDL2 config register
 * ctrl:   Control register
 * retune: Retune request register (for retuning mode 2)
 */
struct acs3_regs {
	unsigned long tdl1;
	unsigned long tdl2;
	unsigned long ctrl;
	unsigned long retune;
};

#define ACS3_CTRL_CMD_CONFLICT_DIS_MASK 0x1
#define ACS3_CTRL_SUCCESS_CNT_MASK      0x1e
#define ACS3_CTRL_SUCCESS_CNT_SHIFT     1
#define ACS3_CTRL_RETUNING_MODE_MASK    0x60
#define ACS3_CTRL_RETUNING_MODE_SHIFT   5

#define ACS3_RETUNING_MODE_1 0
#define ACS3_RETUNING_MODE_2 1
#define ACS3_RETUNING_MODE_3 2

/**
 * struct acs3_plat_data - platform / board specific data
 * @led_gpio:            GPIO line used to drive activity led
 * @wp_gpio:             card lock / read only status GPIO line
 * @cd_gpio:             card detect GPIO line
 * @rst_gpio:            eMMC4 hardware reset GPIO line
 * @brd_ocr:             bit field of card Vdd voltages board supports
 * @max_clk:             max clk supported by board
 * @mmc_caps:            host specific capabilities
 * @mmc_caps2:           more host specific capabilities
 * @regs:                addresses of the host's GBC registers
 * @default_mode_config: array of default modes configurations (limited to 4 
 *                       different configurations)
 * @mode_config:         array of modes configurations
 * @nb_config:           size of @mode_config
 */
struct acs3_plat_data {
	int led_gpio;
	int wp_gpio;
	int cd_gpio;
	int rst_gpio;
	u32 brd_ocr;
	u32 mmc_caps;
	u32 mmc_caps2;
	const struct acs3_regs* regs;
	const struct acs3_mode_config* default_mode_config;
	const struct acs3_mode_config* mode_config;
	unsigned int nb_config;
	bool disable_ddr50;
};

/**
 * The preset value registers support 8 modes (Initialization, default speed,
 * high speed, SDR12, SDR25, SDR50, SDR104 and DDR50).
 */
#define ACS3_INIT           (1U)
#define ACS3_DEFAULT_SPEED  (1U << 1)
#define ACS3_HIGH_SPEED     (1U << 2)
#define ACS3_UHS_SDR12      (1U << 3)
#define ACS3_UHS_SDR25      (1U << 4)
#define ACS3_UHS_SDR50      (1U << 5)
#define ACS3_UHS_SDR104     (1U << 6)
#define ACS3_UHS_DDR50      (1U << 7)
#define ACS3_UHS_SDR        (ACS3_UHS_SDR12 | ACS3_UHS_SDR25 | \
                             ACS3_UHS_SDR50 | ACS3_UHS_SDR104)

/**
 * struct acs3_mode_config - mode specific configuration
 * @mode:                 Modes supported by this configuration. This parameter
 *                        is a bitfield.
 * @max_Hz:               Max clock supported in this mode
 * @tdl1:                 TDL1 for this mode (supported in HS & UHS)
 * @tdl2:                 TDL2 for this mode (supported only in DDR50 mode)
 * @tuning_success_count: Number of success tuning pattern (supported only in 
 *                        SDR50 & SDR104 mode)
 * @retune_timer:         Timer count for re-tuning (supported only in SDR50 &
 *                        SDR104 mode)
 */
struct acs3_mode_config {
	u32 mode; 
	u32 max_Hz;
	u32 tdl1;
	u32 tdl2;
	u32 tuning_success_count;
	u32 retune_timer;
};

#endif
