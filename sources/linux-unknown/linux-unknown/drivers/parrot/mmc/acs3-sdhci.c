/**
 * linux/driver/parrot/mmc/acs3-sdhci.h - Arasan Chip Systems eMMC / SD / SDIO
 *                                        host driver implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Samir Ammenouche <samir.ammenouche@parrot.com>
 *          Gregor Boirie <gregor.boirie@parrot.com>
 * date:    20-Jan-2012
 *
 * This file is released under the GPL.
 *
 * Head to https://smet/projects/p7/wiki/SDIO !!
 *
 * TODO:
 *  - add support for SD bus clock / data edges alignment
 *  - add support for SD bus I/Os power cycling
 *  - add support for driver type selection
 *  - add support for voltage regulator if needed (see vmms field of
 *    sdhci_host structure);
 *  - review me !!
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mmc/host.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/mmc/mmc.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include "acs3-sdhci.h"
#include "../../mmc/host/sdhci.h"
#include "../clk/p7clk-provider.h"

#define ACS3_NAME_MAX   (32U)

#define ACS3_CUR_RETUNING_MODE ACS3_RETUNING_MODE_1

#define ACS3_MAX_CURRENT 500

/*
 * Per host private data holder
 */
struct acs3_priv {
	/* Platform specific data */
	struct acs3_plat_data const*    pdata;
	/* Current mode */
	int                             cur_mode;
	/* Current mode config */
	const struct acs3_mode_config*  cur_config; 
	/* SD bus clock descriptor */
	struct clk*                     sd;
	/* XIN bus clock descriptor */
	struct clk*                     xin;
	/* Optional activity led GPIO line name */
	char                            led_name[ACS3_NAME_MAX];
	/* Optional write protect GPIO line name */
	char                            wp_name[ACS3_NAME_MAX];
	/* Optional card detect GPIO line name */
	char                            cd_name[ACS3_NAME_MAX];
	/* Optional eMMC hardware reset GPIO line name */
	char                            rst_name[ACS3_NAME_MAX];
	/* Card detect interrupt if any */
	int                             cd_irq;
	/* Host controller clock */
	struct clk*                     clk;
	/* Host controller pins */
	struct pinctrl*                 pctl;
	/* Host controller register space */
	struct resource*                res;
	/* Array to save GBC registers during suspend/resume procedure */
	u32                             gbc_regs[4];
};

static void acs3_set_current_config(struct sdhci_host* host, u32 mode);

/*************************
 * SDHCI layer operations
 *************************/

/*
 * Capabilities & present state registers interposer.
 *
 * Some capabilities MUST be set by host driver. Moreover we need to implement
 * GPIO based card detection.
 *
 * May be called from interrupt context.
 */
static u32 acs3_readl(struct sdhci_host* host, int reg)
{
	struct acs3_priv * const   priv = sdhci_priv(host);
	u32                        val = __raw_readl(host->ioaddr + reg);

	if (unlikely(reg == SDHCI_PRESENT_STATE)) {
		if (! (host->quirks & SDHCI_QUIRK_BROKEN_CARD_DETECTION)) {
			/* Use card detect gpio if not polling card status. */
			if (gpio_get_value_cansleep(priv->pdata->cd_gpio))
				/* gpio line tells us there is no card. */
				val &= ~SDHCI_CARD_PRESENT;
		}
		/* When no card is present, the configuration is reseted.
		 * This is necessary when a ACS3_DEFAULT_SPEED card is inserted
		 * after an ACS3_HIGH_SPEED or UHS card.
		 */
		if (!(val & SDHCI_CARD_PRESENT))
			acs3_set_current_config(host, ACS3_DEFAULT_SPEED);
	}
	else if (unlikely(reg == SDHCI_CAPABILITIES_1)) {
		/* Timer count cannot be 0. */
		val = (val & ~SDHCI_RETUNING_TIMER_COUNT_MASK) + (1 << SDHCI_RETUNING_TIMER_COUNT_SHIFT);
		/* Disable DDR50 support on P7R2 */
		if (priv->pdata->disable_ddr50)
			val &= ~(SDHCI_SUPPORT_DDR50);
	}
	else if (unlikely(reg == SDHCI_MAX_CURRENT)) {
		u32 const   max_curr = ACS3_MAX_CURRENT / SDHCI_MAX_CURRENT_MULTIPLIER;

		/*
		 * Properly advertise board specific maximum current the
		 * host may deliver.
		 */
		val |= (max_curr << SDHCI_MAX_CURRENT_330_SHIFT) &
		       SDHCI_MAX_CURRENT_330_MASK;
		val |= (max_curr << SDHCI_MAX_CURRENT_300_SHIFT) &
		       SDHCI_MAX_CURRENT_300_MASK;

		/* Set maximum current for 1.8V capable combinations. */
		val |= (max_curr << SDHCI_MAX_CURRENT_180_SHIFT) &
		       SDHCI_MAX_CURRENT_180_MASK;
	}

	return val;
}

/*
 * Interrupt related registers interposer.
 *
 * When using card detect GPIO interrupt (i.e., not polling card status), ensure
 * we don't enable card presence notifications at host controller level (might
 * also generate unexpected interrupts otherwise ??).
 */
static void acs3_writel(struct sdhci_host* host, u32 val, int reg)
{
	if (unlikely(reg == SDHCI_INT_ENABLE || reg == SDHCI_SIGNAL_ENABLE)) {
		if (! (host->quirks & SDHCI_QUIRK_BROKEN_CARD_DETECTION)) {
			/*
			 * These interrupts won't work with a custom
			 * card detect gpio. Do not use them.
			 */
			val &= ~(SDHCI_INT_CARD_REMOVE | SDHCI_INT_CARD_INSERT);
		}
	}

	__raw_writel(val, host->ioaddr + reg);
}

/*
 * Host control 2 register interposer.
 *
 * Prevents the enabling of preset values.
 *
 * As 1.8V switch signals (sig_en_18v & ush1_18v_vreg_fail) are not connected,
 * we need to wrap write accesses to properly drive external voltage switching
 * device (managed using GPIO line).
 *
 * Set specific configuration for UHS modes.
 */
static void acs3_writew(struct sdhci_host* host, u16 val, int reg)
{
	if (unlikely(reg == SDHCI_HOST_CONTROL2) && (val & SDHCI_CTRL_PRESET_VAL_ENABLE)) {
		val &= ~(SDHCI_CTRL_PRESET_VAL_ENABLE);
	}

	__raw_writew(val, host->ioaddr + reg);
	
	if (unlikely(reg == SDHCI_HOST_CONTROL2) && host->vqmmc && 
			regulator_is_supported_voltage(host->vqmmc, 1800000, 1800000) &&
			(val & SDHCI_CTRL_VDD_180)) {
		if ((val & SDHCI_CTRL_UHS_MASK) == SDHCI_CTRL_UHS_SDR12)
			acs3_set_current_config(host, ACS3_UHS_SDR12);
		else if ((val & SDHCI_CTRL_UHS_MASK) == SDHCI_CTRL_UHS_SDR25)
			acs3_set_current_config(host, ACS3_UHS_SDR25);
		else if ((val & SDHCI_CTRL_UHS_MASK) == SDHCI_CTRL_UHS_SDR50)
			acs3_set_current_config(host, ACS3_UHS_SDR50);
		else if ((val & SDHCI_CTRL_UHS_MASK) == SDHCI_CTRL_UHS_SDR104)
			acs3_set_current_config(host, ACS3_UHS_SDR104);
		else if ((val & SDHCI_CTRL_UHS_MASK) == SDHCI_CTRL_UHS_DDR50)
			acs3_set_current_config(host, ACS3_UHS_DDR50);
	}
}

static u16 acs3_readw(struct sdhci_host* host, int reg)
{
	u32                             val = __raw_readl(host->ioaddr + reg);

	if (unlikely(reg == SDHCI_HOST_CONTROL2)) {
		val &= ~SDHCI_CTRL_VDD_180;

		if (host->vqmmc && (regulator_get_voltage(host->vqmmc) == 1800000))
			/* Request voltage switching to 1.8V. */
			val |= SDHCI_CTRL_VDD_180;
	}
	/* Empty the buffer register during the tuning process. */
	else if (unlikely(reg == SDHCI_COMMAND)) {
		
		if (SDHCI_GET_CMD(val) == MMC_SEND_TUNING_BLOCK ||
		    SDHCI_GET_CMD(val) == MMC_SEND_TUNING_BLOCK_HS200) {
			int i;
			for (i = 0; i < 16; i++) {
				__raw_readl(host->ioaddr + SDHCI_BUFFER);
			}
		}
	}

	return val;
}

/*
 * Host control register interposer.
 *
 * As activity led signal is not connected, we need to wrap write accesses to
 * drive activity led GPIO line accordingly.
 *
 * Activate ACS3_HIGH_SPEED configuration in High-Speed mode.
 *
 * May be called from atomic context.
 */
static void acs3_writeb(struct sdhci_host* host, u8 val, int reg)
{
	if (unlikely(reg == SDHCI_HOST_CONTROL)) {
		struct acs3_priv const* const   priv = sdhci_priv(host);

		if (priv->led_name[0])
			/* Blink led. */
			gpio_set_value(priv->pdata->led_gpio, val & SDHCI_CTRL_LED);
	}

	__raw_writeb(val, host->ioaddr + reg);

	/* Sometimes in UHS mode the bit SDHCI_CTRL_HISPD is high, so this case has
	 * to be checked before activating ACS3_HIGH_SPEED configuration.
	 */
	if (unlikely(reg == SDHCI_HOST_CONTROL)){
		u32 val2 = __raw_readl(host->ioaddr + SDHCI_HOST_CONTROL2);
		if ((val & SDHCI_CTRL_HISPD) && !(val2 & SDHCI_CTRL_VDD_180)) {
			acs3_set_current_config(host, ACS3_HIGH_SPEED);
		}
	}
}

/*
 * Maximum SD clock reachable frequency is 130 mHz.
 */
static unsigned int acs3_get_maxclk(struct sdhci_host* host)
{
	struct acs3_priv const* const   priv = sdhci_priv(host);

	if ((priv->cur_config) && (priv->cur_config->max_Hz))
		return priv->cur_config->max_Hz;
	else
		return 130000000;
}

/*
 * Compute minimum reachable SD clock frequency.
 */
static unsigned int acs3_get_minclk(struct sdhci_host* host)
{
	return (unsigned int)
	       clk_round_rate(((struct acs3_priv*) sdhci_priv(host))->xin, 0) /
	       SDHCI_MAX_DIV_SPEC_300;
}

/*
 * Set XIN clock in order to optimize SD clock rate.
 */
static void acs3_set_clk(struct sdhci_host* host, unsigned int hz)
{
	if (hz) {
		struct acs3_priv const* const priv = sdhci_priv(host);
		int err;

		err = clk_set_rate(priv->sd, hz);
		if (err) {
			dev_warn(mmc_dev(host->mmc),
			         "failed to set sd clock rate to %u (%d)\n",
			         hz,
			         err);
			return;
		}

		host->max_clk = clk_get_rate(priv->xin);
	}
}

/*
 * Card lock / read only handling.
 *
 * As card write protect pin is not connected, we get read only status through
 * an optional GPIO line usage.
 * May be called from atomic context.
 */
static unsigned int acs3_get_ro(struct sdhci_host* host)
{
	struct acs3_priv const* const   priv = sdhci_priv(host);

	if (! priv->wp_name[0])
		/*
		 * Warning: this won't work if SDHCI_QUIRK_INVERTED_WRITE_PROTECT quirk
		 * is used (see sdhci_check_ro).
		 * If needed, implement mmc_host_ops.get_ro directly instead of using
		 * the one installed be sdhci layer (see sdhci_get_ro). Or use a
		 * platform specific flag to indicate RO GPIO line meaning is inverted.
		 */
		return -ENOSYS;

	return gpio_get_value_cansleep(priv->pdata->wp_gpio);
}

/*
 * eMMC4 hardware reset handling (extracted from sdhci-pci.c)
 *
 * As eMMC reset pin is unconnected, hardware reset is carried out using a GPIO
 * line. Some board may choose not to provide one.
 */
static void acs3_reset(struct sdhci_host* host)
{
	struct acs3_priv const* const   priv = sdhci_priv(host);

	if (priv->rst_name[0]) {
		/* Perform hardware reset only when GPIO line is available. */
		int const   gpio = priv->pdata->rst_gpio;

		gpio_set_value_cansleep(gpio, 0);
		/* For eMMC, minimum is 1us but give it 10us for good measure */
		udelay(10);

		gpio_set_value_cansleep(gpio, 1);
		/* For eMMC, minimum is 200us but give it 300us for good measure */
		usleep_range(300, 1000);
	}
}

static struct sdhci_ops acs3_sdhci_ops = {
	.read_l         = acs3_readl,
	.read_w         = acs3_readw,
	.write_l        = acs3_writel,
	.write_w        = acs3_writew,
	.write_b        = acs3_writeb,
	.set_clock      = acs3_set_clk,
	.get_max_clock  = acs3_get_maxclk,
	.get_min_clock  = acs3_get_minclk,
	.get_ro         = acs3_get_ro,
	.hw_reset       = acs3_reset
};

/*
 * TDL (Tap Delay Logic) setting
 *
 * TDL1 : In some cases the board layout may not be optimal and the Data may 
 * have hold time violations on the card. TDL1 can be used to delay the data so
 * that the Hold time violations are fixed.
 *
 * TDL2 : During read operation the host controller acts as a receiver and the
 * data may not be exactly aligned with respect to the clock. The TDL2 is used
 * to phase shift the clock so that it is aligned with the received data.
 */
static inline void acs3_set_tdl(struct acs3_priv * priv, u32 mode)
{
	if ((!priv->pdata->regs) || (!priv->cur_config))
		return;
	
	/* TDL1 is used in all HS and UHS modes */
	if (priv->pdata->regs->tdl1) {
		if (priv->cur_config->tdl1)
			__raw_writel(priv->cur_config->tdl1, priv->pdata->regs->tdl1);
		else
			__raw_writel(0x100, priv->pdata->regs->tdl1); /* 0x100 is the reset value of TDL1 register */

	}

	/* In DDR50 mode, the TDL2 is manually set. */
	if ((mode & ACS3_UHS_DDR50) && (priv->pdata->regs->tdl2)) {
		if (priv->cur_config->tdl2)
			__raw_writel(priv->cur_config->tdl2, priv->pdata->regs->tdl2);
		else
			__raw_writel(0x100, priv->pdata->regs->tdl2); /* 0x100 is the reset value of TDL2 register */
	}
	/* In SDR50 and SDR104 modes, the TDL2 is calculated by the tuning. */
	else if (mode & (ACS3_UHS_SDR50 | ACS3_UHS_SDR104)) {
		/* When tuning is active, TDL2 must be enabled. */
		if (priv->pdata->regs->tdl2) {
			u32 tdl2_reg = __raw_readl(priv->pdata->regs->tdl2);
			tdl2_reg |= 1;				/* enable TDL2 */
			__raw_writel(tdl2_reg, priv->pdata->regs->tdl2);	
		}
		/* Tuning success count cannot be 0. */
		if (priv->pdata->regs->ctrl) {
			/* Get the CTRL register's current value and clear
			 * the tuning succes count. */
			u32 val = __raw_readl(priv->pdata->regs->ctrl) & ~ACS3_CTRL_SUCCESS_CNT_MASK;

			/* Add the tuning success count. */
			if (!priv->cur_config->tuning_success_count)
				val += 1 << ACS3_CTRL_SUCCESS_CNT_SHIFT;
			else if (priv->cur_config->tuning_success_count > 0xf)
				val += 0xf << ACS3_CTRL_SUCCESS_CNT_SHIFT;
			else 
				val += priv->cur_config->tuning_success_count << ACS3_CTRL_SUCCESS_CNT_SHIFT;

			__raw_writel(val, priv->pdata->regs->ctrl);
		}
	}
}

/*
 * Set max clock according to current mode configuration
 */
static inline void acs3_set_mode_clock(struct sdhci_host* host) {
	struct mmc_host *mmc = host->mmc;

	/* Those parameters are used to set the host clock */	
	if (mmc) {
		mmc->f_max = acs3_get_maxclk(host);
		mmc->f_min = acs3_get_minclk(host);
	}
}

/*
 * Run through the mode_configs array to find a matching configuration
 *
 * Return 0 : if no matching configuration.
 * Return 1 : if the matching configuration is different than the previous one.
 * Return 2 : if the matching configuration is the same than the previous one.
 */
static inline int acs3_get_new_config(struct acs3_priv* priv, u32 mode,
				const struct acs3_mode_config* mode_configs,
				int mode_configs_size) {
	int i;

	for (i = 0; i < mode_configs_size; i++) {
		if (mode_configs[i].mode & mode) {
			if (priv->cur_config != &(mode_configs[i])) {
				/* Update the config */
				priv->cur_config = &(mode_configs[i]);
				return 1;
			}
			else 
				/* In this case, the config in the array is 
 				 * found and is the same than the previous one.
 				 */
				return 2;
		}
	}
	/* The mode is not supported in the mode_configs array */
	return 0;
}

/*
 * When the host changes mode, the configuration must be updated. 
 * 
 * The following parameters must be updated:
 * - max clock
 * - TDLs
 * - retuning parameters
 */
static void acs3_set_current_config(struct sdhci_host* host, u32 mode) {
	struct acs3_priv* priv = sdhci_priv(host);
	int mode_changed = (priv->cur_mode == mode) ? 0 : 1;

	if (mode_changed) {
		int config_found = 0;
		/* Get a new configuration in privat_data if needed. */
		if (priv->pdata->mode_config)
			config_found = acs3_get_new_config(priv, mode,
							priv->pdata->mode_config,
							priv->pdata->nb_config);
	
		/* If no configuration for the current mode is defined in the
		 * private data, we get the default configuration for this 
		 * mode. */
		if (!config_found)
			acs3_get_new_config(priv, mode,
					priv->pdata->default_mode_config, 4);

		/* If the mode has changed, apply the configuration. */
		priv->cur_mode = mode;
		acs3_set_tdl(priv, mode);
		acs3_set_mode_clock(host);
		if (priv->cur_config->retune_timer)
			host->tuning_count = priv->cur_config->retune_timer;
	}
}

/********************
 * GPIO line helpers
 ********************/

static int _acs3_init_gpio(struct sdhci_host* host,
                           int gpio,
                           unsigned long flags,
                           char* name,
                           char const* suffix,
                           size_t size)
{
	struct device const* const      dev = mmc_dev(host->mmc);
	char const* const               dname = dev_name(dev);

	name[0] = '\0';
	if (! gpio_is_valid(gpio))
		return -EINVAL;

	snprintf(name,
	         min(strlen(dname) + 1 + size, ACS3_NAME_MAX),
	         "%s %s",
	         dname,
	         suffix);
	if (! gpio_request_one(gpio, flags, name))
		return 0;

	name[0] = '\0';
	dev_warn(dev,
	         "failed to request %s gpio %d (" __stringify(-EBUSY) ")\n",
	         suffix,
	         gpio);

	return -EBUSY;
}

#define acs3_init_gpio(_host, _gpio, _flags, _name, _suffix) \
	_acs3_init_gpio(_host, _gpio, _flags, _name, _suffix, sizeof(_suffix))

static void acs3_exit_gpio(int gpio, char const* name)
{
	if (name[0])
		gpio_free(gpio);
}

static irqreturn_t acs3_handle_cd(int irq, void* dev_id)
{
	struct work_struct *w = dev_id;

	/* Since the request_irq has to be called before the add_host where the  
	*  workqueue is initialized, it is mandatory to test the initialization of 
	*  the workqueue before schedule it. */
	if (w->func)
		schedule_work(w);
	return IRQ_HANDLED;
};

/***********************************
 * Host controller setup & teardown
 ***********************************/

static void acs3_exit_host(struct sdhci_host* host,
                           struct acs3_plat_data const* pdata,
                           struct acs3_priv const* priv)
{
	acs3_exit_gpio(pdata->rst_gpio, priv->rst_name);
	if (! (host->quirks & SDHCI_QUIRK_BROKEN_CARD_DETECTION)) {
		free_irq(priv->cd_irq, &host->card_detect_work);
		gpio_free(pdata->cd_gpio);
	}
	acs3_exit_gpio(pdata->wp_gpio, priv->wp_name);
	acs3_exit_gpio(pdata->led_gpio, priv->led_name);
}

static int __devinit acs3_init_host(struct sdhci_host* host,
                                    struct acs3_plat_data const* pdata,
                                    struct acs3_priv* priv)
{
	int err;

	priv->pdata = pdata;
	host->hw_name = ACS3_DRV_NAME;
	host->ops = &acs3_sdhci_ops;
	acs3_set_current_config(host, ACS3_DEFAULT_SPEED);

	/*
	 * Controller does not support unaligned buffer start address and size for
	 * ADMA mode (see section 2.9 of Arasan SD3.0 / SDIO3.0 / eMMC4.41 AHB Host
	 * Controller user guide).
	 */
	host->quirks = SDHCI_QUIRK_32BIT_ADMA_SIZE |
	               SDHCI_QUIRK_32BIT_DMA_ADDR;

	/* Timeout clock (TMCLK) is the same as sd clock.
	 * Note: sometimes a device may specify timeouts that are larger than the
	 * highest possible timeout in the host controller. In this case Linux
	 * generates a warning you may inhibit using the
	 * SDHCI_QUIRK_BROKEN_TIMEOUT_VAL (see sdhci_calc_timeout).
	 */
	host->quirks |= SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK;

	/* Setup activity led blinking */
	acs3_init_gpio(host,
	               pdata->led_gpio,
	               GPIOF_OUT_INIT_LOW,
	               priv->led_name,
	               "led");

	/* Setup card lock / read only status */
	acs3_init_gpio(host,
	               pdata->wp_gpio,
	               GPIOF_IN,
	               priv->wp_name,
	               "wp");

	/* Setup card detect */
	err = acs3_init_gpio(host,
	                     pdata->cd_gpio,
	                     GPIOF_IN,
	                     priv->cd_name,
	                     "cd");
	if (! err) {
		priv->cd_irq = gpio_to_irq(pdata->cd_gpio);
#ifdef DEBUG
		BUG_ON(priv->cd_irq < 0);
#endif

		/* If the GPIO access can sleep we have to use a threaded IRQ
		 * handler. */
		if (gpio_cansleep(pdata->cd_gpio))
			err = request_threaded_irq(priv->cd_irq,
						   NULL,
						   acs3_handle_cd,
						   IRQF_TRIGGER_FALLING |
						   IRQF_TRIGGER_RISING,
						   dev_name(mmc_dev(host->mmc)),
						   &host->card_detect_work);
		else
			err = request_irq(priv->cd_irq,
					  acs3_handle_cd,
					  IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
					  dev_name(mmc_dev(host->mmc)),
					  &host->card_detect_work);

		if (err) {
			gpio_free(pdata->cd_gpio);
			dev_warn(mmc_dev(host->mmc),
			         "failed to request cd irq %d (%d)\n",
			         priv->cd_irq,
			         err);
		}
	}
	if (err)
		/* Enable card detection in polling mode. */
		host->quirks |= SDHCI_QUIRK_BROKEN_CARD_DETECTION;

	/* Setup eMMC hardware reset */
	if (! acs3_init_gpio(host,
	                     pdata->rst_gpio,
	                     GPIOF_OUT_INIT_LOW,
	                     priv->rst_name,
	                     "rst"))
		host->mmc->caps |= MMC_CAP_HW_RESET;

	/* Setup card available voltages. */
#ifdef DEBUG
	BUG_ON(! pdata->brd_ocr);
#endif
	host->ocr_avail_mmc = pdata->brd_ocr;
	host->ocr_avail_sd = host->ocr_avail_mmc;
	host->ocr_avail_sdio = host->ocr_avail_mmc;

	/* Enable DDR mode for eMMC chips */
	host->mmc->caps |= MMC_CAP_1_8V_DDR;

	/* Some specific capabilities can be set according to plat_data.
	 * It has to be done before the sdhci_add_host, so the 
	 * necessary caps are already set when the host is initialized. */
	if (pdata->mmc_caps)	
		host->mmc->caps |= pdata->mmc_caps;
	if (pdata->mmc_caps2)
		host->mmc->caps2 |= pdata->mmc_caps2;

	/* Register host to SDHCI layer */
	err = sdhci_add_host(host);
	if (! err)
		return 0;

	acs3_exit_host(host, pdata, priv);
	dev_err(mmc_dev(host->mmc), "failed to register host (%d)\n", err);
	return err;
}

/*************************
 * Power management hooks
 *************************/

#ifdef CONFIG_PM

static int acs3_suspend(struct platform_device* pdev, pm_message_t state)
{
	struct sdhci_host* const   host = platform_get_drvdata(pdev);
	struct acs3_priv * const   priv = sdhci_priv(host);
	int const                  err = sdhci_suspend_host(host);

	if (err)
		return err;

	/* Save SDIO GBC registers */
	priv->gbc_regs[0] = __raw_readl(priv->pdata->regs->tdl1);
	priv->gbc_regs[1] = __raw_readl(priv->pdata->regs->tdl2);
	priv->gbc_regs[2] = __raw_readl(priv->pdata->regs->ctrl);
	priv->gbc_regs[3] = __raw_readl(priv->pdata->regs->retune);

	clk_disable_unprepare(priv->xin);
	clk_disable_unprepare(priv->clk);
	return 0;
}

static int acs3_resume(struct platform_device* pdev)
{
	struct sdhci_host* const        host = platform_get_drvdata(pdev);
	struct acs3_priv const* const   priv = sdhci_priv(host);
	int                             err;

	clk_prepare_enable(priv->clk);
	clk_prepare_enable(priv->xin);

	/* Restore SDIO GBC registers */
	__raw_writel(priv->gbc_regs[0], priv->pdata->regs->tdl1);
	__raw_writel(priv->gbc_regs[1], priv->pdata->regs->tdl2);
	__raw_writel(priv->gbc_regs[2], priv->pdata->regs->ctrl);
	__raw_writel(priv->gbc_regs[3], priv->pdata->regs->retune);

	err = sdhci_resume_host(host);
	if (! err)
		return 0;

	clk_disable_unprepare(priv->xin);
	clk_disable_unprepare(priv->clk);
	return err;
}

#else   /* CONFIG_PM */

#define acs3_suspend    NULL
#define acs3_resume     NULL

#endif  /* CONFIG_PM */

/************************
 *  Platform driver glue
 ************************/

static int __devinit acs3_probe(struct platform_device* pdev)
{
	int                             err;
	struct resource*                res;
	struct acs3_priv*               priv;
	struct acs3_plat_data* const    pdata = dev_get_platdata(&pdev->dev);
	struct sdhci_host* const        host = sdhci_alloc_host(&pdev->dev,
	                                                        sizeof(*priv));

	if (IS_ERR(host)) {
		dev_err(&pdev->dev, "failed to allocate SDHCI host\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (! res) {
		err = -ENXIO;
		dev_err(&pdev->dev, "failed to retrieve register space\n");
		goto free;
	}

	err = platform_get_irq(pdev, 0);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to get interrupt\n");
		goto free;
	}
	host->irq = err;

	priv = sdhci_priv(host);
	priv->res = request_mem_region(res->start,
	                               resource_size(res),
	                               dev_name(&pdev->dev));
	if (! priv->res) {
		err = -EBUSY;
		dev_err(&pdev->dev, "failed to request register space\n");
		goto free;
	}

	/* Set the retuning mode. This has to be done before the controller is unresetted */
	if (pdata->regs && pdata->regs->ctrl) {
		u32 val = __raw_readl(pdata->regs->ctrl) & ~ACS3_CTRL_RETUNING_MODE_MASK;
		val += ACS3_CUR_RETUNING_MODE << ACS3_CTRL_RETUNING_MODE_SHIFT;
		__raw_writel(val, pdata->regs->ctrl);
	}

	host->ioaddr = ioremap(res->start, resource_size(res));
	if (! host->ioaddr) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "failed to remap register space\n");
		goto release;
	}

	priv->clk = clk_get(&pdev->dev, "host");
	if (IS_ERR(priv->clk)) {
		err = PTR_ERR(priv->clk);
		dev_err(&pdev->dev, "failed to get host clock (%d)\n", err);
		goto unmap;
	}

	priv->xin = clk_get(&pdev->dev, "xin");
	if (IS_ERR(priv->xin)) {
		err = PTR_ERR(priv->xin);
		dev_err(&pdev->dev, "failed to get xin clock (%d)\n", err);
		goto put_clk;
	}

	/* Not sure if this is really necessary... */
	err = clk_set_rate(priv->xin, 25000000);
	if (err) {
		dev_err(&pdev->dev, "failed to set initial xin clock rate (%d)\n", err);
		goto put_xin;
	}

	priv->sd = clk_get(&pdev->dev, "sd");
	if (IS_ERR(priv->sd)) {
		err = PTR_ERR(priv->sd);
		dev_err(&pdev->dev, "failed to get sd clock (%d)\n", err);
		goto put_xin;
	}

	priv->pctl = pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(priv->pctl)) {
		err = PTR_ERR(priv->pctl);
		dev_err(&pdev->dev, "failed to select I/O pins (%d)\n", err);
		goto put_sd;
	}

	err = clk_prepare_enable(priv->clk);
	if (err) {
		dev_err(&pdev->dev, "failed to enable host clock (%d)\n", err);
		goto put_pins;
	}

	err = clk_prepare_enable(priv->xin);
	if (err) {
		dev_err(&pdev->dev, "failed to enable xin clock (%d)\n", err);
		goto disable_clk;
	}

	err = acs3_init_host(host, pdata, priv);
	if (! err) {
		platform_set_drvdata(pdev, host);
		return 0;
	}

	clk_disable_unprepare(priv->xin);

disable_clk:
	clk_disable_unprepare(priv->clk);
put_pins:
	pinctrl_put(priv->pctl);
put_sd:
	clk_put(priv->sd);
put_xin:
	clk_put(priv->xin);
put_clk:
	clk_put(priv->clk);
unmap:
	iounmap(host->ioaddr);
release:
	release_mem_region(res->start, resource_size(res));
free:
	sdhci_free_host(host);
	return err;
}

static int __devexit acs3_remove(struct platform_device* pdev)
{
	struct sdhci_host* const        host = platform_get_drvdata(pdev);
	struct acs3_priv* const         priv = sdhci_priv(host);
	int			err = 0;

	sdhci_remove_host(host,
	                  __raw_readl(host->ioaddr +
	                              SDHCI_INT_STATUS) == ~(0U));

	acs3_exit_host(host, priv->pdata, priv);

	clk_disable_unprepare(priv->xin);
	clk_disable_unprepare(priv->clk);
	clk_put(priv->sd);
	clk_put(priv->xin);
	clk_put(priv->clk);

	pinctrl_put(priv->pctl);

	iounmap(host->ioaddr);
	release_mem_region(priv->res->start, resource_size(priv->res));

	sdhci_free_host(host);
	return err;
}

static struct platform_driver acs3_driver = {
	.probe      = acs3_probe,
	.remove     = __devexit_p(acs3_remove),
	.suspend    = acs3_suspend,
	.resume     = acs3_resume,
	.driver     = {
		.name       = ACS3_DRV_NAME,
		.owner      = THIS_MODULE,
	},
};
module_platform_driver(acs3_driver);

MODULE_AUTHOR("Samir Ammenouche <samir.ammenouche@parrot.com>");
MODULE_DESCRIPTION("Arasan Chip Systems SDHCI driver");
MODULE_LICENSE("GPL");
