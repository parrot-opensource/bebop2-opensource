/**
 * linux/arch/arm/mach-parrot7/ether.c - Parrot7 Ethernet platform interface
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Jimmy Perchet <jimmy.perchet@parrot.com>
 * date:    27-08-2013
 *
 * This file is released under the GPL
 */

#include <linux/clk.h>
#include <linux/stmmac.h>
#include <linux/phy.h>
#include <linux/clkdev.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio.h>
#include <mach/ether.h>
#include <mach/irqs.h>
#include "common.h"
#include "clock.h"
#include "system.h"
#include "pinctrl.h"
#include <linux/delay.h>
#include <linux/clk-provider.h>


struct p7_eth_priv_data {
	enum phy_iface	iface;
	bool		gentxclk;
	struct clk	*txclk;
	struct clk      *ctrlclk;
	struct pinctrl	*pctl;
	int		phy_reset_gpio;
};

static const uint8_t iface_cfg[4] = {
	[PHY_IFACE_MII]     = (0 << 0)|(2 << 4),
	[PHY_IFACE_RGMII]   = (1 << 0)|(1 << 4),
	[PHY_IFACE_MIILITE] = (2 << 0)|(2 << 4),
	[PHY_IFACE_SSMII]   = (0 << 0)|(1 << 4),
};

static char * parrot_ethernet_source_clock = "pll_usb_clk";
static int parrot_ethernet_root_clock_is_rx = 0;

static int __init parrot_ethernet_source_clock_fn(char *str)
{
	if (strstr(str,"eth_rx")) {
		parrot_ethernet_source_clock = "pll_usb_clk";
		parrot_ethernet_root_clock_is_rx = 1;
	}
	else if (strstr(str,"pll_avi")) {
		if (p7_chiprev() == P7_CHIPREV_R3)
			parrot_ethernet_source_clock = "pll_avi0_clk";
		else if (p7_chiprev() == P7_CHIPREV_R2)
			parrot_ethernet_source_clock = "pll_avi1_clk";
	}
	return 1;
}
__setup("parrot_ethernet_source_clock", parrot_ethernet_source_clock_fn);


static int p7_eth_init(struct platform_device *p7_eth_device)
{
	int err = 0;
	struct plat_stmmacenet_data *ppdata;
	struct p7_eth_priv_data *priv;
	struct clk *pll_parent;

	ppdata = dev_get_platdata(&p7_eth_device->dev);
	priv = (struct p7_eth_priv_data *)ppdata->bsp_priv;

	priv->ctrlclk = clk_get(&p7_eth_device->dev, "stmmaceth");
	if (IS_ERR(priv->ctrlclk)) {
		err = PTR_ERR(priv->ctrlclk);
		goto error;
	}

	if (priv->phy_reset_gpio >= 0) {
		err = gpio_request_one(priv->phy_reset_gpio,
					GPIOF_OUT_INIT_LOW,
					"eth phy nreset");
		if (err) {
			printk(KERN_ERR "Gpio request failed :%d\n", err);
			goto put_ctrl_clk;
		}
	}

	priv->pctl = pinctrl_get_select_default(&p7_eth_device->dev);
	if (IS_ERR(priv->pctl)) {
		err = PTR_ERR(priv->pctl);
		goto gpio_free;
	}

	/* configure interface and clock source */
	writel(iface_cfg[priv->iface],
			__MMIO_P2V(P7_HSP_GBC) + HSP_GBC_ETHERNET_CONFIG);
	//XXX: add 2us delay ?

	if (priv->gentxclk) {

		pll_parent = clk_get_sys(parrot_ethernet_source_clock, NULL);
		if (IS_ERR(pll_parent)) {
			printk(KERN_ERR "eth: failed to get parent's eth_tx_clk : %s\n",
						parrot_ethernet_source_clock);
			err = PTR_ERR(pll_parent);
			goto rel_pinctrl;
		}

		priv->txclk = clk_get(&p7_eth_device->dev, "eth_tx_clk");
		if (IS_ERR(priv->txclk)) {
			err = PTR_ERR(priv->pctl);
			goto put_pll_parent;
		}

		/*
		 * Configure the clock tree:
		 *  Set PLL USB as parent of eth_tx
		 *  Set eth_tx_clk at 2.5MHz
		 *  Ungate
		 */
		err = clk_set_parent(priv->txclk, pll_parent);
		if (err) {
			printk(KERN_ERR "eth: failed to reparent eth_tx_clk "
			       "with %s (%d)\n", __clk_get_name(pll_parent), err);
			goto put_tx_clk;
		}

		if (strcmp(parrot_ethernet_source_clock, "pll_usb_clk") == 0) {
			/*
			 * When we call clk_set_rate on eth_tx, it propagates
			 * to pll_usb and then to its children (USB and CAN).
			 * But prior to writting to USB_DIV or CAN_DIV, we must
			 * enable the USB PLL or it triggers a freeze.
			 */
			err = clk_prepare_enable(pll_parent);
			if (err) {
				printk(KERN_ERR "eth: failed to enable "
				       "pll_usb clock (%d)\n", err);
				goto put_tx_clk;
			}
		}

		/* we want 2.5MHz */
		err = clk_set_rate(priv->txclk, 2500000);
		if (err) {
			printk(KERN_ERR "eth: failed to set eth_tx rate (%d)\n",
			       err);
			goto put_tx_clk;
		}

		err = clk_prepare_enable(priv->txclk);
		if (err) {
			printk(KERN_ERR "eth: failed to enable eth_tx clock (%d)\n",
			       err);
			goto put_tx_clk;
		}

		/*
		 * Now that reparenting is done, we can
		 * drop this clock
		 */
		clk_put(pll_parent);
	}

	clk_prepare_enable(priv->ctrlclk);

	return 0;

put_tx_clk:
	clk_put(priv->txclk);
put_pll_parent:
	clk_put(pll_parent);
rel_pinctrl:
	pinctrl_put(priv->pctl);
gpio_free:
	if (priv->phy_reset_gpio >= 0)
		gpio_free(priv->phy_reset_gpio);
put_ctrl_clk:
	clk_put(priv->ctrlclk);
error:
	return err;
}

static void p7_eth_exit(struct platform_device *p7_eth_device)
{
	struct plat_stmmacenet_data *ppdata;
	struct p7_eth_priv_data *priv;
	struct clk *pll_usb;

	ppdata = dev_get_platdata(&p7_eth_device->dev);
	priv = (struct p7_eth_priv_data *)ppdata->bsp_priv;

	pinctrl_put(priv->pctl);

	if (priv->gentxclk) {
		clk_disable_unprepare(priv->txclk);
		clk_put(priv->txclk);
		if (strcmp(parrot_ethernet_source_clock, "pll_usb_clk") == 0) {
			pll_usb = clk_get_sys(parrot_ethernet_source_clock,
			                      NULL);
			if (IS_ERR(pll_usb)) {
				printk(KERN_ERR "eth: failed to get pll_usb_clk\n");
				goto disable_ctrlclk;
			}
			clk_disable_unprepare(pll_usb);
			clk_put(pll_usb);
		}
	}

disable_ctrlclk:
	clk_disable_unprepare(priv->ctrlclk);
	clk_put(priv->ctrlclk);

	if (priv->phy_reset_gpio >= 0)
		gpio_free(priv->phy_reset_gpio);
}

static void p7_eth_suspend(struct platform_device *p7_eth_device)
{
	struct plat_stmmacenet_data *ppdata;
	struct p7_eth_priv_data *priv;

	ppdata = dev_get_platdata(&p7_eth_device->dev);
	priv = (struct p7_eth_priv_data *)ppdata->bsp_priv;

	if (priv->gentxclk)
		clk_disable_unprepare(priv->txclk);

	clk_disable_unprepare(priv->ctrlclk);
}

static void p7_eth_resume(struct platform_device *p7_eth_device)
{
	struct plat_stmmacenet_data *ppdata;
	struct p7_eth_priv_data *priv;

	ppdata = dev_get_platdata(&p7_eth_device->dev);
	priv = (struct p7_eth_priv_data *)ppdata->bsp_priv;

	/* configure interface and clock source */
	writel(iface_cfg[priv->iface],
			__MMIO_P2V(P7_HSP_GBC) + HSP_GBC_ETHERNET_CONFIG);

	if (priv->gentxclk)
		clk_prepare_enable(priv->txclk);

	clk_prepare_enable(priv->ctrlclk);
}

static void p7_update_tx_clock(void *_priv, unsigned int speed)
{
	unsigned long rate, new_rate;
	struct p7_eth_priv_data *priv = (struct p7_eth_priv_data *)_priv;

	if (!priv->gentxclk)
		/* nothing to do, we are not generating tx clock */
		return;

	clk_disable_unprepare(priv->txclk);

	switch (speed) {
	case 1000:
		/* 125 MHz */
		rate = 125125000;
		break;
	case 100:
		/* 25 MHz */
		rate = 25000000;
		break;
	case 10:
		/* 2.5 MHz */
		rate = 2500000;
		break;
	default:
		/* leave clock disabled */
		return;
	}
	clk_set_rate(priv->txclk, rate);

	new_rate = clk_get_rate(priv->txclk);
	if (new_rate != rate)
		printk(KERN_ERR "eth: failed to get correct clock %ld (expected %ld)\n", new_rate, rate);
	clk_prepare_enable(priv->txclk);
}

static int p7_phy_reset(void *_priv)
{
	struct p7_eth_priv_data *priv = (struct p7_eth_priv_data *)_priv;
	if (gpio_is_valid(priv->phy_reset_gpio)) {
		gpio_set_value_cansleep(priv->phy_reset_gpio, 0);
		mdelay(10);
		gpio_set_value_cansleep(priv->phy_reset_gpio, 1);
		mdelay(50);
	}
	return 0;
}

static struct resource p7_eth_resources[] = {
	{
		.start	= P7_ETHER,
		.end	= P7_ETHER + 2*SZ_64K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= P7_ETH0_IRQ,
		.end	= P7_ETH0_IRQ,
		.flags	= IORESOURCE_IRQ,
		.name   = "macirq",
	},
	{
		.start	= P7_ETH1_IRQ,
		.end	= P7_ETH1_IRQ,
		.flags	= IORESOURCE_IRQ,
		.name   = "eth_wake_irq",
	},
	{
		.start	= P7_ETH2_IRQ,
		.end	= P7_ETH2_IRQ,
		.flags	= IORESOURCE_IRQ,
		.name   = "eth_lpi",
	},
};

static struct stmmac_mdio_bus_data p7_eth_mdio_bus_data = {
	/*Try all addresses*/
	.phy_mask       = 0x0,
	.irqs           = NULL,
	.probed_phy_irq = PHY_POLL,
	.phy_reset	= p7_phy_reset,
};

static struct p7_eth_priv_data p7_eth_priv_data;

static struct stmmac_dma_cfg p7_eth_dma_cfg = {
	/* According to "DesignWar Cores Ethernet MAC Universal Databook,
	   Version 3.71a", Table 6-6, PBL = 32x8 = 256 is OK for 32-bit bus
	   and 2kB fifos.
	   But, in this case, the maximum frame size the Tx Checksum Offload
	   Engine(COE) can work on is 1012o (see Note in 4.7.1 Transmit
	   Checksum Offload Engine:
	   MaxFrameSize = TxFifo - (PBL+3)*(BusWidth/8)).
	   In order to enable the Tx COE to work on full ethernet frame,
	   reduce the PBL to 128(MaxFrameSize=1524). To handle jumbo frames
	   we have to enable "bugged_jumbo" flag.
	   A more optimal way would be to let RxPBL = 256, and set TxPBL to
	   128, but driver impose RxPBL==TxPBL.
	  */
	.pbl		= 16,

	/* On P7, IP is configured with AXI_BL=16; So default behavior is OK. */
	.fixed_burst	= 0,
	.mixed_burst	= 0,

	/* Driver write all the DMA_AXI_BUS_MODE register with burst_len value,
	   thus it can perform more than it was made for.
	   To be safe, use the DMA_AXI_BUS_MODE register reset value here. */
	.burst_len	= 0x110000,
};

static struct plat_stmmacenet_data p7_eth_plat_data = {
	.bus_id         = 0,
	.phy_addr       = -1,
	.interface      = PHY_INTERFACE_MODE_RGMII_ID,
	.mdio_bus_data  = &p7_eth_mdio_bus_data,
	.dma_cfg        = &p7_eth_dma_cfg,
	.clk_csr        = STMMAC_CSR_250_300M,
	.has_gmac       = 1,
	.enh_desc	= 1,
	.tx_coe         = 1,
	.rx_coe		= STMMAC_RX_COE_TYPE2,
	.bugged_jumbo	= 1,
	.pmt		= 1,
	.init           = p7_eth_init,
	.exit           = p7_eth_exit,
	.suspend        = p7_eth_suspend,
	.resume         = p7_eth_resume,
	.fix_mac_speed  = p7_update_tx_clock,
	.bsp_priv       = &p7_eth_priv_data,
};


static u64 p7_eth_dma_mask = DMA_BIT_MASK(32);

static struct platform_device p7_eth_device = {
	.name		   = "stmmaceth",
	.id		   = -1,
	.resource	   = p7_eth_resources,
	.num_resources	   = ARRAY_SIZE(p7_eth_resources),
	.dev               = {
	    .platform_data      = &p7_eth_plat_data,
	    .dma_mask           = &p7_eth_dma_mask,
	    .coherent_dma_mask  = DMA_BIT_MASK(32),
	},
};

static unsigned long p7_eth_pinconf[] = {
	P7CTL_PUD_CFG(HIGHZ), /* no pull up/down unable */
	P7CTL_DRV_CFG(5),     /* default to max drive. Needs to be in 2nd place
			       * for p7_init_ether! */
};

static struct pinctrl_map p7_eth_pads_mii[] __initdata = {
	P7_INIT_PINMAP(P7_ETH_MII_TXER),
	P7_INIT_PINCFG(P7_ETH_MII_TXER, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_MII_RXER),
	P7_INIT_PINCFG(P7_ETH_MII_RXER, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_MII_CRS),
	P7_INIT_PINCFG(P7_ETH_MII_CRS, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_MII_COL),
	P7_INIT_PINCFG(P7_ETH_MII_COL, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_TXC),
	P7_INIT_PINCFG(P7_ETH_RGMII_TXC, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_TXD_00),
	P7_INIT_PINCFG(P7_ETH_RGMII_TXD_00, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_TXD_01),
	P7_INIT_PINCFG(P7_ETH_RGMII_TXD_01, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_TXD_02),
	P7_INIT_PINCFG(P7_ETH_RGMII_TXD_02, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_TXD_03),
	P7_INIT_PINCFG(P7_ETH_RGMII_TXD_03, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_TX_CTL),
	P7_INIT_PINCFG(P7_ETH_RGMII_TX_CTL, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_RXC),
	P7_INIT_PINCFG(P7_ETH_RGMII_RXC, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_RXD_00),
	P7_INIT_PINCFG(P7_ETH_RGMII_RXD_00, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_RXD_01),
	P7_INIT_PINCFG(P7_ETH_RGMII_RXD_01, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_RXD_02),
	P7_INIT_PINCFG(P7_ETH_RGMII_RXD_02, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_RXD_03),
	P7_INIT_PINCFG(P7_ETH_RGMII_RXD_03, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_RX_CTL),
	P7_INIT_PINCFG(P7_ETH_RGMII_RX_CTL, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_MDC),
	P7_INIT_PINCFG(P7_ETH_MDC, p7_eth_pinconf), /* external pull assumed */
	P7_INIT_PINMAP(P7_ETH_MDIO),
	P7_INIT_PINCFG(P7_ETH_MDIO, p7_eth_pinconf), /* external pull assumed */
};

static struct pinctrl_map p7_eth_pads_rgmii[] __initdata = {
	P7_INIT_PINMAP(P7_ETH_RGMII_TXC),
	P7_INIT_PINCFG(P7_ETH_RGMII_TXC, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_TXD_00),
	P7_INIT_PINCFG(P7_ETH_RGMII_TXD_00, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_TXD_01),
	P7_INIT_PINCFG(P7_ETH_RGMII_TXD_01, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_TXD_02),
	P7_INIT_PINCFG(P7_ETH_RGMII_TXD_02, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_TXD_03),
	P7_INIT_PINCFG(P7_ETH_RGMII_TXD_03, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_TX_CTL),
	P7_INIT_PINCFG(P7_ETH_RGMII_TX_CTL, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_RXC),
	P7_INIT_PINCFG(P7_ETH_RGMII_RXC, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_RXD_00),
	P7_INIT_PINCFG(P7_ETH_RGMII_RXD_00, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_RXD_01),
	P7_INIT_PINCFG(P7_ETH_RGMII_RXD_01, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_RXD_02),
	P7_INIT_PINCFG(P7_ETH_RGMII_RXD_02, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_RXD_03),
	P7_INIT_PINCFG(P7_ETH_RGMII_RXD_03, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_RGMII_RX_CTL),
	P7_INIT_PINCFG(P7_ETH_RGMII_RX_CTL, p7_eth_pinconf),
	P7_INIT_PINMAP(P7_ETH_MDC),
	P7_INIT_PINCFG(P7_ETH_MDC, p7_eth_pinconf), /* external pull assumed */
	P7_INIT_PINMAP(P7_ETH_MDIO),
	P7_INIT_PINCFG(P7_ETH_MDIO, p7_eth_pinconf), /* external pull assumed */
};

void __init p7_init_ether(enum phy_iface iface,
			  int phy_reset_gpio,
			  unsigned long drive_strength)
{
	int err;
	if (p7_chiprev() == P7_CHIPREV_R1) {
		printk(KERN_ERR "Ethernet not available on P7R1\n");
		return;
	}

	/* Override drive strength */
	p7_eth_pinconf[1] = drive_strength;

	p7_eth_priv_data.iface = iface;
	p7_eth_priv_data.gentxclk = (iface == PHY_IFACE_RGMII)
					|| (iface == PHY_IFACE_SSMII);

	p7_eth_priv_data.phy_reset_gpio = phy_reset_gpio;

	switch (iface) {
	case PHY_IFACE_RGMII:
		p7_eth_plat_data.interface = PHY_INTERFACE_MODE_RGMII_ID,
		err = p7_init_dev(&p7_eth_device, NULL,
					p7_eth_pads_rgmii,
					ARRAY_SIZE(p7_eth_pads_rgmii));
		break;
	case PHY_IFACE_MIILITE:
		p7_eth_plat_data.interface = PHY_INTERFACE_MODE_MII;
		err = p7_init_dev(&p7_eth_device, NULL,
					p7_eth_pads_rgmii,
					ARRAY_SIZE(p7_eth_pads_rgmii));
		break;
	case PHY_IFACE_MII:
	case PHY_IFACE_SSMII:
		p7_eth_plat_data.interface = PHY_INTERFACE_MODE_MII;
		err = p7_init_dev(&p7_eth_device, NULL,
					p7_eth_pads_mii,
					ARRAY_SIZE(p7_eth_pads_mii));
		break;
	default:
		printk(KERN_ERR "Ethernet : bad phy interface\n");
	}
}

