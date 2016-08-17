/*
 *  linux/drivers/parrot/video/media/avi_v4l2_isp.h
 *
 *  Copyright (C) 2015 Parrot S.A.
 *
 * @author  Alexandre Dilly <alexandre.dilly@parrot.com>
 * @date  01-Jan-2015
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

#ifndef _AVI_V4L2_ISP_H_
#define _AVI_V4L2_ISP_H_

#include <media/v4l2-ctrls.h>

#include "../../video/avi_isp.h"

#define AVI_V4L2_ISP_CONTROL_HINT 182

struct avi_v4l2_isp;

/** avi_v4l2_isp_blanking() - Blanking ISP task.
  * @isp:	The ISP handler.
  *
  * Some ISP configuration tasks must be done during blanking period like I3D
  * LUT. It is recommanded to call this function just after getting a new frame.
  */
extern int avi_v4l2_isp_blanking(struct avi_v4l2_isp *isp);

/** avi_v4l2_isp_activate() - Activate ISP chain.
  * @isp:	The ISP handler.
  * @offsets:	The ISP chain offsets.
  *
  * This activate ISP chain and restore all context from controls (all ISP chain
  *  registers of selected).
  *
  * This function should be called in a streamon().
  */
extern int avi_v4l2_isp_activate(struct avi_v4l2_isp *isp,
				 struct avi_isp_offsets *offsets);

/** avi_v4l2_isp_deactivate() - Deactivate ISP chain.
  * @isp:	The ISP handler.
  *
  * This deactivate ISP chain. ISP chain can be used by another module. The
  * context is saved in controls and will be restored at next call of
  * avi_v4l2_isp_activate().
  *
  * This function should be called in a streamoff().
  */
extern void avi_v4l2_isp_deactivate(struct avi_v4l2_isp *isp);

/** avi_v4l2_isp_init() - Init ISP control handler.
  * @isp:		The ISP handler.
  * @ctrl_handler:	The control handler.
  *
  * This allocates and init ISP handler and add all controls to control handler.
  */
extern int avi_v4l2_isp_init(struct avi_v4l2_isp **isp,
			     struct v4l2_ctrl_handler *ctrl_handler);

/** avi_v4l2_isp_free() - Free ISP control handler.
  * @isp:		The ISP handler.
  */
extern void avi_v4l2_isp_free(struct avi_v4l2_isp *isp);

#endif /* _AVI_V4L2_ISP_H_ */
