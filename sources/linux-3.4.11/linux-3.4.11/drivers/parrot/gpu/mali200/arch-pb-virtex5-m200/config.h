/*
 * Copyright (C) 2010-2012 ARM Limited. All rights reserved.
 * 
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 * 
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __ARCH_CONFIG_H__
#define __ARCH_CONFIG_H__

/* Note: IRQ auto detection (setting irq to -1) only works if the IRQ is not shared with any other hardware resource */

static _mali_osk_resource_t arch_configuration [] =
{
	{
		.type = MALIGP2,
		.description = "MALI GP2",
		.base = 0xc0002000,
		.irq = -1,
		.mmu_id = 1
	},
	{
		.type = MMU,
		.base = 0xc0003000,
		.irq = -1, /*105*/
		.description = "Mali MMU",
		.mmu_id = 1
	},
	{
		.type = MALI200,
		.base = 0xc0000000,
		.irq = -1/*106*/,
		.description = "Mali 200 (GX525)",
		.mmu_id = 1
	},
#if USING_OS_MEMORY
	{
		.type = OS_MEMORY,
		.description = "OS Memory",
		.alloc_order = 10, /* Lowest preference for this memory */
		.size = 0x04000000, /* 64MB, to avoid OOM-killer */
		.flags = _MALI_CPU_WRITEABLE | _MALI_CPU_READABLE | _MALI_PP_READABLE | _MALI_PP_WRITEABLE |_MALI_GP_READABLE | _MALI_GP_WRITEABLE
	},
#endif /** USING_OS_MEMORY */
	{
		.type = MEMORY,
		.description = "Mali SDRAM remapped to baseboard",
		.cpu_usage_adjust = -0x50000000,
		.alloc_order = 0, /* Highest preference for this memory */
		.base = 0xD0000000,
		.size = 0x10000000, /* 256 MB */
		.flags = _MALI_CPU_WRITEABLE | _MALI_CPU_READABLE | _MALI_PP_READABLE | _MALI_PP_WRITEABLE |_MALI_GP_READABLE | _MALI_GP_WRITEABLE
	},
	{
		.type = MEMORY,
		.description = "Mali ZBT",
		.alloc_order = 5, /* Medium preference for this memory */
		.base = 0xe1000000,
		.size = 0x01000000,
		.flags = _MALI_CPU_WRITEABLE | _MALI_CPU_READABLE | _MALI_PP_READABLE | _MALI_PP_WRITEABLE |_MALI_GP_READABLE | _MALI_GP_WRITEABLE
	},
	{
		.type = MEM_VALIDATION,
		.description = "Framebuffer",
		.base = 0xe0000000,
		.size = 0x01000000,
		.flags = _MALI_CPU_WRITEABLE | _MALI_CPU_READABLE | _MALI_PP_WRITEABLE | _MALI_PP_READABLE
	},

};

#endif /* __ARCH_CONFIG_H__ */
