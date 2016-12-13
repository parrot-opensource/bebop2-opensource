/*
 *  linux/drivers/parrot/video/media/avi_m2m.h
 *
 *  Copyright (C) 2015 Parrot S.A.
 *
 * @author  Alexandre Dilly <alexandre.dilly@parrot.com>
 * @date  03-Aug-2015
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
#ifndef _AVI_M2M_H_
#define _AVI_M2M_H_

/**
 * struct avi_m2m_platform_data - AVI M2M specific data
 */
struct avi_m2m_platform_data {
	unsigned long caps;
	int enable_stats;
	enum vb2_cache_flags vb2_cache_flags;
	enum vb2_cache_flags stat_vb2_cache_flags;
};

#endif /* _AVI_M2M_H_ */
