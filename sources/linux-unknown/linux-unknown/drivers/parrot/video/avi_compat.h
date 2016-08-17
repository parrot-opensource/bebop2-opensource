#ifndef _AVI_COMPAT_H_
#define _AVI_COMPAT_H_

/*
 *  linux/drivers/parrot/video/avi_compat.h
 *
 *  Copyright (C) 2013 Parrot S.A.
 *
 * @author  Lionel Flandrin <lionel.flandrin@parrot.com>
 * @date  27-Aug-2013
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * This file contains the AVI compatibility code for the various revisions of
 * the IP.
 */

#include "avi.h"
#include "avi_logger.h"

#ifdef CONFIG_AVI_LOGGER
/* We need the compatibility layer to log register access */
#define AVI_BACKWARD_COMPAT
#endif

#ifdef AVI_BACKWARD_COMPAT

extern void avi_compat_r1_write(u32 val, u32 addr);
extern u32  avi_compat_r1_read(u32 addr);

extern void avi_compat_r2_write(u32 val, u32 addr);
extern u32  avi_compat_r2_read(u32 addr);

/*
 * Those functions are made so that they can be replaced by simple
 * "__raw_writel" and "__raw_readl" if we only need to support the final version
 * of the chip (currently P7R3).
 */

static inline void avi_writel(enum avi_revision rev,
			      u32 val, u32 addr)
{
	avi_log_access(addr, val, AVI_LOG_WRITE);

	switch (rev) {
	case AVI_REVISION_1:
		avi_compat_r1_write(val, addr);
		break;
	case AVI_REVISION_2:
		avi_compat_r2_write(val, addr);
		break;
	case AVI_REVISION_3:
		__raw_writel(val, addr);
		break;
	default:
		BUG();
	}
}

static inline u32 avi_readl(enum avi_revision rev, u32 addr)
{
	u32 val;

	switch (rev) {
	case AVI_REVISION_1:
		val = avi_compat_r1_read(addr);
		break;
	case AVI_REVISION_2:
		val = avi_compat_r2_read(addr);
		break;
	case AVI_REVISION_3:
		val = __raw_readl(addr);
		break;
	default:
		BUG();
	}

	avi_log_access(addr, val, AVI_LOG_READ);


	return val;

}

/* A dirty but handy macro to avoid mentioning avi_ctrl.revision for each
 * access */
#define AVI_WRITE(_v, _a)	avi_writel(avi_ctrl.revision, (_v), (u32)(_a))
#define AVI_READ(_a)		avi_readl(avi_ctrl.revision, (u32)(_a))

#else /* AVI_BACKWARD_COMPAT */

#define AVI_WRITE  __raw_writel
#define AVI_READ   __raw_readl

#endif /* AVI_BACKWARD_COMPAT */


#endif /* _AVI_COMPAT_H_ */
