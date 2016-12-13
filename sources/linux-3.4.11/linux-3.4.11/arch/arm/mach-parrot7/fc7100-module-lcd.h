/*
 * FC7100 mezz
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _ARCH_FC7100_MODULE_FC71XX_H
#define _ARCH_FC7100_MODULE_FC71XX_H

#include "avi.h"

/* lcd module */
void __init fc7100_mezz_avi_init_lcd(struct avifb_overlay *overlay, int overlay_size);
void __init fc7100_mezz_reserve_mem_for_lcd(struct avifb_overlay *overlays, int noverlays);

void fc7100_mezz_avi_init_lcd_screen(const char *screen_name, int *xres, int *yres);

#endif /* _ARCH_MEZZS2_FC71XX_H */
