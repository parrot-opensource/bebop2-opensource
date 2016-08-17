/*
 *	Copyright (c) 2013 Parrot SA (Jimmy Perchet)
 *
 *	Inspired by code from generic phy
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/phy.h>
static int fp_config_init(struct phy_device *phydev)
{
	u32 features;

	printk(KERN_INFO "fakephy config init\n");
	features = SUPPORTED_MII;

	/* When autoneg is enabled, rx clock is not generated
	   and therefore is not usable. So disable autoneg.
	*/
	phydev->autoneg = AUTONEG_DISABLE;

	features |= SUPPORTED_100baseT_Full;

	phydev->supported = features;
	phydev->advertising = features;

	/* set 100Mbits*/
	phydev->speed = SPEED_100;
	phydev->duplex = DUPLEX_FULL;

	return 0;
}


static int fp_setup_forced(struct phy_device *phydev)
{
	phydev->pause = phydev->asym_pause = 0;

	return 0;
}

static int fp_config_aneg(struct phy_device *phydev)
{
	fp_setup_forced(phydev);
	return 0;
}

static int fp_update_link(struct phy_device *phydev)
{
	phydev->link = 1;
	return 0;
}

static int fp_read_status(struct phy_device *phydev)
{
	phydev->link = 1;
	phydev->speed = SPEED_100;
	phydev->duplex = DUPLEX_FULL;
	phydev->pause = phydev->asym_pause = 0;
	return 0;
}

static int fp_suspend(struct phy_device *phydev)
{
	return 0;
}

static int fp_resume(struct phy_device *phydev)
{
	return 0;
}

static struct phy_driver fp_drivers = {
	.phy_id		= 0xFF003521,
	.phy_id_mask	= 0xfffffff0,
	.name		= "FAKE PHY",
	.config_init	= fp_config_init,
	.features	= 0,
	.config_aneg	= fp_config_aneg,
	.read_status	= fp_read_status,
	.update_link	= fp_update_link,
	.suspend	= fp_suspend,
	.resume		= fp_resume,
	.driver		= {.owner= THIS_MODULE, },
};

static int __init fakephy_init(void)
{
	return phy_driver_register(&fp_drivers);
}

static void __exit fakephy_exit(void)
{
	phy_driver_unregister(&fp_drivers);
}

module_init(fakephy_init);
module_exit(fakephy_exit);

static struct mdio_device_id __maybe_unused fakephy_tbl[] = {
	{ 0xFF003521, 0xfffffff0 },
	{ }
};

MODULE_DEVICE_TABLE(mdio, fakephy_tbl);
MODULE_LICENSE("GPL");


