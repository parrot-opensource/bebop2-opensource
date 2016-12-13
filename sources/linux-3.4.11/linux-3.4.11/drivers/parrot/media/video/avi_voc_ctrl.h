/*
 *      linux/drivers/parrot/video/avi_voc_ctrl.h
 *
 *      Copyright (C) 2014 Parrot S.A.
 *
 * @author  Victor Lambret <victor.lambret.ext@parrot.com>
 * @date    20-Oct-2014
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

#ifndef _AVI_VOC_CTRLS_H_
#define _AVI_VOC_CTRLS_H_

#include "avi_voc_core.h"

int avi_voc_ctrl_create(struct avi_voc *voc);

void avi_voc_ctrl_free(struct avi_voc *voc);

void avi_voc_ctrl_unhide(struct avi_voc *voc);

#endif
