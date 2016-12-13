#ifndef _AVI_VSYNC_GEN_H_
#define _AVI_VSYNC_GEN_H_

#include "avi.h"

#define AVIVSYNC_OFFSET_NR 5

struct avivsync_offsets {
	union avi_lcd_vsync_gen_on_0	vsync_on;
	union avi_lcd_vsync_gen_off_0	vsync_off;
};

struct avivsync_timings {
	unsigned		pixclock_freq;
	/* Length of a line in pixclock */
	unsigned		htotal;
	/* Height of the frame in lines */
	unsigned		vtotal;
	/* Vsync specification */
	unsigned		vsync_von;
	unsigned		vsync_hon;
	unsigned		vsync_voff;
	unsigned		vsync_hoff;
	/* VSync offsets */
	struct avivsync_offsets	offsets[AVIVSYNC_OFFSET_NR];
};

struct avivsync_platform_data {
	struct avivsync_timings  timings;
	int			 enabled;
	struct avi_group	*avi_group;
};

#ifdef _SAMPLE_BSP_DO_NOT_REMOVE_

#include <video/avi_vsync_gen.h>

static struct pinctrl_map rnb6_avi_vsync_pins[] __initdata = {
	P7_INIT_PINMAP(P7_LCD_0_CLK),
	P7_INIT_PINMAP(P7_LCD_0_VS),
	/* This is for P7R2. In P7R3 all pads are shifted by one: the "master"
	 * VSync is DATA16 and the offsets 0 to 4 are on pads DATA17 to 21. */
	P7_INIT_PINMAP(P7_LCD_0_DATA15),
	P7_INIT_PINMAP(P7_LCD_0_DATA16),
	P7_INIT_PINMAP(P7_LCD_0_DATA17),
	P7_INIT_PINMAP(P7_LCD_0_DATA18),
	P7_INIT_PINMAP(P7_LCD_0_DATA19),
	P7_INIT_PINMAP(P7_LCD_0_DATA20),
};

static unsigned int const rnb6_avi_vsync_nodes[] = {
	AVI_LCD0_NODE
};

static struct avi_chan rnb6_avi_vsync_chan = {
	.type = AVI_VSYNC_GEN_CHAN_TYPE,
	.nodes = rnb6_avi_vsync_nodes,
	.nodes_nr = ARRAY_SIZE(rnb6_avi_vsync_nodes),
};

static struct avi_group rnb6_avi_vsync_group = {
	.name	  = "avi_vsync.lcd0",
	.lck	  = __SPIN_LOCK_UNLOCKED(rnb6_avi_vsync_group.lck),
	.channels = &rnb6_avi_vsync_chan,
	.chan_nr  = 1,
};

static struct avivsync_platform_data rnb6_avi_vsync_pdata = {
	.avi_group = &rnb6_avi_vsync_group,
};

static struct platform_device rnb6_avi_vsync_dev = {
	.name           = "avi_vsync_gen",
	.id             = 0,
};

#endif /* _SAMPLE_BSP_DO_NOT_REMOVE_ */

#endif /* _AVI_VSYNC_GEN_H_ */
