/*
 *	Copyright (c) 2013 Parrot SA (Matthieu CASTET)
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
static int br_config_init(struct phy_device *phydev)
{
	u32 features;

	features = SUPPORTED_MII;

	/* When autoneg is enabled, rx clock is not generated
	   and therefore is not usable. So disable autoneg.
	*/
	phydev->autoneg = AUTONEG_DISABLE;

	features |= SUPPORTED_100baseT_Full;
	features |= SUPPORTED_10baseT_Full;

	phydev->supported = features;
	phydev->advertising = features;

	/* set slave 100Mbits 1 pair */
	phydev->speed = SPEED_100;
	phydev->duplex = DUPLEX_FULL;

	/*
	 * PHY "optimization"
	 * Brodcoam's soup (ported from 89810A2_script_v2_2)
	 *
	 */
	/* reset */
	phy_write(phydev, MII_BMCR, 0x8000);

	phy_write(phydev, 0x17, 0x0F93);
	phy_write(phydev, 0x15, 0x107F);
	phy_write(phydev, 0x17, 0x0F90);
	phy_write(phydev, 0x15, 0x0001);
	phy_write(phydev, MII_BMCR, 0x3000);
	phy_write(phydev, MII_BMCR, 0x0200);

	phy_write(phydev, 0x18, 0x0C00);

	phy_write(phydev, 0x17, 0x0F90);
	phy_write(phydev, 0x15, 0x0000);
	phy_write(phydev, MII_BMCR, 0x0100);

	phy_write(phydev, 0x17, 0x0001);
	phy_write(phydev, 0x15, 0x0027);

	phy_write(phydev, 0x17, 0x000E);
	phy_write(phydev, 0x15, 0x9B52);

	phy_write(phydev, 0x17, 0x000F);
	phy_write(phydev, 0x15, 0xA04D);

	phy_write(phydev, 0x17, 0x0F90);
	phy_write(phydev, 0x15, 0x0001);

	phy_write(phydev, 0x17, 0x0F92);
	phy_write(phydev, 0x15, 0x9225);

	phy_write(phydev, 0x17, 0x000A);
	phy_write(phydev, 0x15, 0x0323);

	phy_write(phydev, 0x17, 0x0FFD);
	phy_write(phydev, 0x15, 0x1C3F);

	phy_write(phydev, 0x17, 0x0FFE);
	phy_write(phydev, 0x15, 0x1C3F);

	phy_write(phydev, 0x17, 0x0F99);
	phy_write(phydev, 0x15, 0x7180);

	phy_write(phydev, 0x17, 0x0F9A);
	phy_write(phydev, 0x15, 0x34C0);

	phy_write(phydev, 0x17, 0x0F0E);
	phy_write(phydev, 0x15, 0x0000);
	phy_write(phydev, 0x17, 0x0F9F);
	phy_write(phydev, 0x15, 0x0000);
	phy_write(phydev, 0x18, 0xF1E7);


	phy_write(phydev, MII_BMCR, 1<<9);
	pr_info("forced slave 100Mbits 1 pair\n");

	return 0;
}

static int br_setup_forced(struct phy_device *phydev)
{
	int err;
	int ctl = phy_read(phydev, MII_BMCR);

	phydev->pause = phydev->asym_pause = 0;

	if (SPEED_100 == phydev->speed)
		ctl |= 0x8 << 6;
	else
		ctl &= ~(0x8 << 6);

	/*if (DUPLEX_FULL == phydev->duplex)
		ctl |= BMCR_FULLDPLX;*/
	/* XXX how to set pair and master/slave ? */

	err = phy_write(phydev, MII_BMCR, ctl);

	return err;
}

static inline u32 ethtool_adv_to_brmii_adv_t(u32 ethadv)
{
	u32 result = 0;

	if (ethadv & ADVERTISED_10baseT_Half)
		result |= 1<<1;
	if (ethadv & ADVERTISED_10baseT_Full)
		result |= 1<<1;
	if (ethadv & ADVERTISED_100baseT_Half)
		result |= 1<<5;
	if (ethadv & ADVERTISED_100baseT_Full)
		result |= 1<<5;
	if (ethadv & ADVERTISED_Pause)
		result |= 1<<14;
	if (ethadv & ADVERTISED_Asym_Pause)
		result |= 1<<15;

	return result;
}

static int br_config_advert(struct phy_device *phydev)
{
	u32 advertise;
	int oldadv, adv;
	int err, changed = 0;

	/* Only allow advertising what
	 * this PHY supports */
	phydev->advertising &= phydev->supported;
	advertise = phydev->advertising;

	/* Setup standard advertisement */
	oldadv = adv = phy_read(phydev, MII_ADVERTISE);

	if (adv < 0)
		return adv;

	adv &= ~(0xffff);
	adv |= ethtool_adv_to_brmii_adv_t(advertise);

	if (adv != oldadv) {
		err = phy_write(phydev, MII_ADVERTISE, adv);

		if (err < 0)
			return err;
		changed = 1;
	}

	return changed;
}

static int br_config_aneg(struct phy_device *phydev)
{
	int result;

	if (AUTONEG_ENABLE != phydev->autoneg)
		return br_setup_forced(phydev);

	result = br_config_advert(phydev);

	if (result < 0) /* error */
		return result;

	if (result == 0) {
		/* Advertisement hasn't changed, but maybe aneg was never on to
		 * begin with?  Or maybe phy was isolated? */
		int ctl = phy_read(phydev, MII_BMCR);

		if (ctl < 0)
			return ctl;

		if (!(ctl & BMCR_ANENABLE) || (ctl & BMCR_ISOLATE))
			result = 1; /* do restart aneg */
	}

	/* Only restart aneg if we are advertising something different
	 * than we were before.	 */
	if (result > 0)
		result = genphy_restart_aneg(phydev);

	return result;
}

static int br_read_status(struct phy_device *phydev)
{
	int adv;
	int err;
	int lpa;

	/* Update the link, but return if there
	 * was an error */
	err = genphy_update_link(phydev);
	if (err)
		return err;

	if (AUTONEG_ENABLE == phydev->autoneg) {

		lpa = phy_read(phydev, 0x7);

		if (lpa < 0)
			return lpa;

		adv = phy_read(phydev, MII_ADVERTISE);

		if (adv < 0)
			return adv;

		lpa &= adv;

		phydev->speed = SPEED_10;
		phydev->duplex = DUPLEX_FULL; /* ???*/
		phydev->pause = phydev->asym_pause = 0;

		if (lpa & (1<<5)) {
			phydev->speed = SPEED_100;

		}

		if (phydev->duplex == DUPLEX_FULL){
			phydev->pause = lpa & (1<<14) ? 1 : 0;
			phydev->asym_pause = lpa & (1<<15) ? 1 : 0;
		}
	} else {
		int bmcr = phy_read(phydev, MII_BMCR);
		if (bmcr < 0)
			return bmcr;

		phydev->duplex = DUPLEX_FULL;

		if ((bmcr & (0xf << 6)) == 0x8 << 6)
			phydev->speed = SPEED_100;
		else
			phydev->speed = SPEED_10;

		phydev->pause = phydev->asym_pause = 0;
	}

	return 0;
}

static struct phy_driver br_drivers = {
	.phy_id		= 0x03625cc2,
	/* XXX check mask */
	.phy_id_mask	= 0xfffffff0,
	.name		= "BR PHY",
	.config_init	= br_config_init,
	.features	= 0,
	.config_aneg	= br_config_aneg,
	.read_status	= br_read_status,
	.update_link	= genphy_update_link,
	.suspend	= genphy_suspend,
	.resume		= genphy_resume,
	.driver		= {.owner= THIS_MODULE, },
};

static int __init broadcom_init(void)
{
	return phy_driver_register(&br_drivers);
}

static void __exit broadcom_exit(void)
{
	phy_driver_unregister(&br_drivers);
}

module_init(broadcom_init);
module_exit(broadcom_exit);

static struct mdio_device_id __maybe_unused broadcom_tbl[] = {
	{ 0x03625cc2, 0xfffffff0 },
	{ }
};

MODULE_DEVICE_TABLE(mdio, broadcom_tbl);
MODULE_LICENSE("GPL");
