#include "common.h"
#include "system.h"
#include "board-common.h"

#include <linux/gpio.h>

#include <linux/mmc/host.h>
#include <host/sdhci.h>
#include <mmc/acs3-sdhci.h>

#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/wl12xx.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>

#include "wl18xx.h"

static struct acs3_plat_data wl18xx_sdhci_wlan_pdata = {
	.led_gpio   = -1,                               /* No activity led GPIO */
	.wp_gpio    = -1,                               /* No write protect GPIO */
	.cd_gpio    = -1,                               /* No card detect GPIO */
	.rst_gpio   = -1,                               /* No eMMC hardware reset */
	.brd_ocr    = MMC_VDD_165_195,
	.mmc_caps   = MMC_CAP_NONREMOVABLE,             /* wl18xx chip is non removable */
	.mmc_caps2  = MMC_CAP2_BROKEN_VOLTAGE,          /* sdio bus voltage is fixed in hardware */
};

static char *acs3_sdhci_wlan_vmmc2_supply_dev_name = "acs3-sdhci.0";

static struct regulator_consumer_supply acs3_sdhci_wlan_vmmc2_supply =
	REGULATOR_SUPPLY("vmmc", NULL);

/* VMMC2 for driving the wl18xx module */
static struct regulator_init_data acs3_sdhci_wlan_vmmc2 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies = &acs3_sdhci_wlan_vmmc2_supply,
};

static struct fixed_voltage_config acs3_sdhci_wlan_vwlan = {
	.supply_name            = "vwl18xx",
	.microvolts             = 1800000, /* 1.80V */
	.gpio                   = -1,      /* filled in init_wl18xx */
	.startup_delay          = 70000,   /* 70ms */
	.enable_high            = 1,
	.enabled_at_boot        = 0,
	.init_data              = &acs3_sdhci_wlan_vmmc2,
};

static struct platform_device acs3_sdhci_wlan_regulator = {
	.name           = "reg-fixed-voltage",
	.id             = 0,
	.dev = {
		.platform_data  = &acs3_sdhci_wlan_vwlan,
	},
};
#ifndef CONFIG_REGULATOR_FIXED_VOLTAGE
#warning  CONFIG_REGULATOR_FIXED_VOLTAGE is needed for wifi
#endif


static struct wl12xx_platform_data wl18xx_pdata = {
	.board_ref_clock = WL12XX_REFCLOCK_26,
	.board_tcxo_clock = WL12XX_TCXOCLOCK_26,
};

static struct ti_st_plat_data wilink_st_pdata = {
 .nshutdown_gpio = -1, /* filled in init_wl18xx */
 .dev_name = "/dev/ttyPA1",
 .flow_cntrl = 1,
 .baud_rate = 3000000,
};

static struct platform_device wilink_st_device = {
 .name = "kim",
 .id = -1,
 .dev.platform_data = &wilink_st_pdata,
};


void __init init_wl18xx(struct wl18xx_resources* res,
			struct pinctrl_map *pins,
			size_t pins_cnt)
{
	int err;
	int val;

	acs3_sdhci_wlan_vwlan.gpio = res->wlen_gpio;

	err = gpio_request(res->wlirq_gpio, "wl18xx IT");
	if (err) {
		pr_err("wl18xx: couldn't request GPIO %d "
		       "for wl18xx IT [%d]\n",
		       res->wlirq_gpio, err);
	}

	p7_gpio_interrupt_register(res->wlirq_gpio);
	wl18xx_pdata.irq = gpio_to_irq(res->wlirq_gpio);
	if (wl18xx_pdata.irq < 0) {
		pr_err("wl18xx: couldn't find an IRQ for gpio %d [%d]\n",
		       res->wlirq_gpio, wl18xx_pdata.irq);
		gpio_free(res->wlirq_gpio);
	}

	err = wl12xx_set_platform_data(&wl18xx_pdata);
	if (err)
		pr_err("wl18xx: error setting wl12xx data: %d\n", err);

	BUG_ON(res->wl_sdhci_slot < 0 || res->wl_sdhci_slot > 2);
	acs3_sdhci_wlan_vmmc2_supply.dev_name = acs3_sdhci_wlan_vmmc2_supply_dev_name;
	acs3_sdhci_wlan_vmmc2_supply_dev_name[11] = '0' + res->wl_sdhci_slot;
	p7brd_init_sdhci(res->wl_sdhci_slot, &wl18xx_sdhci_wlan_pdata,
			 &acs3_sdhci_wlan_regulator,
			 NULL, NULL, pins, pins_cnt);

	if (res->bt_rst_gpio) {
		/* uart for BT, gps */
		p7brd_init_uart(res->bt_uart_slot,1);

		BUG_ON(res->bt_uart_slot < 0 || res->bt_uart_slot > 8);
		wilink_st_pdata.dev_name[10] = '0' + res->bt_uart_slot;

		p7brd_export_gpio(res->bt_rst_gpio, GPIOF_OUT_INIT_LOW, "bt-rst");

		wilink_st_pdata.nshutdown_gpio = res->bt_rst_gpio;
		err = platform_device_register(&wilink_st_device);
		if (err)
			pr_err("wl18xx: error setting wilink_st data: %d\n", err);
	}

	if (res->wl_bt_ant_gpio) {
		err = parrot_gpio_user_in_init(P7_GPIO_NR(res->wl_bt_ant_gpio),
					       P7CTL_PUD_CFG(HIGHZ), "wifi-bt-antenna");
		if (err == 0) {
			val = gpio_get_value(P7_GPIO_NR(res->wl_bt_ant_gpio));

			if (val != 0x00) {
				printk(KERN_NOTICE "Failed to detect wifi/BT antenna\n");
			}
		}
	}

	if (res->wl5g_ant_gpio) {
		err = parrot_gpio_user_in_init(P7_GPIO_NR(res->wl5g_ant_gpio),
					       P7CTL_PUD_CFG(HIGHZ), "wifi_5g-antenna");
		if (err == 0) {
			val = gpio_get_value(P7_GPIO_NR(res->wl5g_ant_gpio));

			if (val != 0x00) {
				printk(KERN_NOTICE "Failed to detect wifi 5GHz antenna\n");
			}
		}
	}
}
