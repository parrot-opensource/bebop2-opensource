/**
 * drivers/parrot/nand/cast_nand.h - Evatronix NANDFLASH-CTRL IP definitions &
 *                                   NAND FLASH definitions
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:	guangye.tian@parrot.com
 * date:	2010-09-22
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */
#ifndef __CAST_NAND_H
#define __CAST_NAND_H

#if defined(CONFIG_MTD_NAND_CAST) || \
    defined(CONFIG_MTD_NAND_CAST_MODULE)

#define CAST_DRV_NAME   "cast-nand"

struct mtd_partition;

struct cast_plat_data {
	struct mtd_partition const* const   parts;
	size_t                              parts_nr;
	unsigned int			    chip_rev;
};

#define FLASH_COUNT	        1 /* Nb of Nand Flash Devices */
#define SUPPORT_BUSWIDTH_16	0

#endif  /* defined(CONFIG_MTD_NAND_CAST) || \
           defined(CONFIG_MTD_NAND_CAST_MODULE) */

#endif /* __CAST_NAND_H */
