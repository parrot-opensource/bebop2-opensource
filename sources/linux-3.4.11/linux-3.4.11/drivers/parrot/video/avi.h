/*
 *  linux/drivers/parrot/video/avi.h
 *
 *  Copyright (C) 2011 Parrot S.A.
 *
 * @author  Gregor Boirie <gregor.boirie@parrot.com>
 * @date  18-Feb-2011
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

#ifndef _AVI_H
#define _AVI_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <asm/io.h>

#include "reg_avi.h"

/* Only needed if we need to support older revisions of the AVI */
#define AVI_BACKWARD_COMPAT

/*
 * Revision of AVI module
 */
enum avi_revision {
        AVI_REVISION_1 = 1,
        AVI_REVISION_2,
        AVI_REVISION_3,
        AVI_REVISION_NR
};

/*
 * Kind of AVI module instances
 */
enum avi_node_type {
	AVI_CFG_NODE_TYPE,
	AVI_INTER_NODE_TYPE,
	AVI_FIFO_NODE_TYPE,
	AVI_CONV_NODE_TYPE,
	AVI_BLEND_NODE_TYPE,
	AVI_BLENDIN_NODE_TYPE,
	AVI_LCD_NODE_TYPE,
	AVI_CAM_NODE_TYPE,
	AVI_SCALROT_NODE_TYPE,
	AVI_GAM_NODE_TYPE,
	AVI_ISP_CHAIN_BAYER_NODE_TYPE,
	AVI_STATS_BAYER_NODE_TYPE,
	AVI_STATS_YUV_NODE_TYPE,
	AVI_ISP_CHAIN_YUV_NODE_TYPE,
	AVI_NODE_TYPE_NR,
};

/*
 * AVI module instances identifiers
 */
enum avi_node_id {
	AVI_FIFO00_NODE,
	AVI_FIFO01_NODE,
	AVI_FIFO02_NODE,
	AVI_FIFO03_NODE,
	AVI_FIFO04_NODE,
	AVI_FIFO05_NODE,
	AVI_FIFO06_NODE,
	AVI_FIFO07_NODE,
	AVI_FIFO08_NODE,
	AVI_FIFO09_NODE,
	AVI_FIFO10_NODE,
	AVI_FIFO11_NODE,
	AVI_CONV0_NODE,
	AVI_CONV1_NODE,
	AVI_CONV2_NODE,
	AVI_CONV3_NODE,
	AVI_BLEND0_NODE,
	AVI_BLEND1_NODE,
	AVI_LCD0_NODE,
	AVI_LCD1_NODE,
	AVI_CAM0_NODE,
	AVI_CAM1_NODE,
	AVI_CAM2_NODE,
	AVI_CAM3_NODE,
	AVI_CAM4_NODE,
	AVI_CAM5_NODE,
	AVI_SCAL00_NODE,
	AVI_SCAL01_NODE,
	AVI_ROT0_NODE,
	AVI_SCAL10_NODE,
	AVI_SCAL11_NODE,
	AVI_ROT1_NODE,
	AVI_GAM0_NODE,
	AVI_GAM1_NODE,
	AVI_ISP_CHAIN_BAYER0_NODE,
	AVI_ISP_CHAIN_BAYER1_NODE,
	AVI_STATS_BAYER0_NODE,
	AVI_STATS_BAYER1_NODE,
	AVI_STATS_YUV0_NODE,
	AVI_STATS_YUV1_NODE,
	AVI_ISP_CHAIN_YUV0_NODE,
	AVI_ISP_CHAIN_YUV1_NODE,
	AVI_INTER_NODE,
	AVI_CFG_NODE,
	AVI_NODE_NR,
};

struct clk;
struct avi_node;

/**
 * struct avi_ctrl - AVI controller descriptor.
 */
struct avi_controller {
    unsigned long               base;
    spinlock_t                  cfg_lck;
    struct mutex                blend_lck;
    struct task_struct*         blend_lck_owner;
    int                         blend_lck_depth;
    struct resource*            iomem;
    struct clk*                 clock;
    struct device*              dev;
    struct avi_node* const      nodes[AVI_NODE_NR];
    enum avi_revision           revision;
    unsigned                    irq_base;
    spinlock_t                  node_lck;
};

extern struct avi_controller avi_ctrl;

/**************************
 * AVI nodes declarations.
 **************************/

/* Value denoting an invalid src_id or sink_id in struct avi_node */
#define AVI_SRCSINK_NONE 0xff

/**
 * struct avi_node - AVI module instance descriptor.
 */
struct avi_node {
	u32 const		base_off;
	unsigned char const     type;
	unsigned char const     src_id;
	unsigned char           sink_id;
	unsigned char const     cfg_off;
	unsigned const          cfg_mask;
	unsigned char           busy;
	char const * const	name;
	unsigned		irq_offset;
	unsigned		assigned_caps;
	enum avi_node_id	node_id;
};

static inline enum avi_revision avi_get_revision(void)
{
	return avi_ctrl.revision;
}

extern int  avi_dereset(void);
extern void avi_reset(void);

static inline unsigned int avi_node_type(struct avi_node const* node)
{
	return (unsigned int) node->type;
}

static inline struct avi_node* avi_get_node(unsigned int id)
{
	BUG_ON(id >= AVI_NODE_NR);

	return avi_ctrl.nodes[id];
}

static inline unsigned long avi_node_base(struct avi_node const* node)
{
	return avi_ctrl.base + ((unsigned long) node->base_off);
}

static inline int avi_probed(void)
{
	/* If the probe fails avi_ctrl.base is set to 0 to indicate that the
	 * driver is KO.*/
	return avi_ctrl.base != 0;
}

static inline unsigned avi_node_irq(struct avi_node const* node)
{
	return avi_ctrl.irq_base + node->irq_offset;
}

extern void memcpy_from_registers(void *, unsigned long, size_t);
extern void avi_cfg_get_registers(struct avi_cfg_regs *);
extern void avi_apply_node(struct avi_node const*);
extern void avi_enable_node(struct avi_node const*);
extern void avi_disable_node(struct avi_node const*);
extern void avi_lock_node(struct avi_node const*);
extern void avi_unlock_node(struct avi_node const*);

/**
 * enum avi_field - Frame field.
 */
enum avi_field {
    AVI_NONE_FIELD,
    AVI_TOPBOT_FIELD,
    AVI_BOTTOP_FIELD,
    AVI_TOP_FIELD,
    AVI_BOT_FIELD,
    AVI_BAD_FIELD,
    AVI_ANY_FIELD,
    AVI_ALTERNATE_FIELD,
};

/*
 * avi_handle_irq_fn can return a bitxsmask of the following values.
 */
/* IRQ handled, do not forward to other clients */
#define AVI_IRQ_HANDLED (1UL << 0)
/* The interrupt has been acked at the node level (for instance using
 * avi_ack_node_irq) by the interrupt callback. There's no need to do it in the
 * generic code. */
#define AVI_IRQ_ACKED   (1UL << 2)

extern int avi_node_get_irq_flag(struct avi_node const *node);
extern void avi_ack_node_irq(struct avi_node const *);
extern void avi_enable_node_irq(struct avi_node const *);
extern void avi_disable_node_irq(struct avi_node const *);

/**
 * struct avi_platform_data - AVI plateform specific data.
 */
struct avi_platform_data {
	enum avi_revision	revision;
	unsigned		irq_base;
};

/***********
 * FIFO API
 ***********/

extern void avi_fifo_get_registers(struct avi_node const*,
                                   struct avi_fifo_registers *);
extern void avi_fifo_set_registers(struct avi_node const*,
                                   struct avi_fifo_registers const*);

/* set oneshot ("single") mode in FIFO.CFG */
extern void avi_fifo_set_oneshot(struct avi_node const *fifo, bool oneshot);

extern void avi_fifo_set_dmasa(struct avi_node const *fifo,
                               u32 dmasa0,
                               u32 dmasa1);
extern void avi_fifo_set_dmalstr(struct avi_node const *fifo,
                                 u32 dmalstr0,
                                 u32 dmalstr1);

extern union avi_fifo_status avi_fifo_get_status(struct avi_node const* node);

extern void avi_fifo_force_clear(struct avi_node const *node);

/**
 * struct avi_blendin_cfg - Blending configuration holder.
 */
struct avi_blendin_cfg {
    u32             offset;
    u32             alpha;
};

/* Per pixel transparency using alpha channel */
#define AVI_ALPHA_OSD  (-1)
/* parameter in percent. 100% is fully opaque. */
#define AVI_ALPHA(_p)  ((_p) * 0xff / 100)


extern void avi_blendin_2cfg(struct avi_blendin_cfg*,
                             struct avi_node const*,
                             u16,
                             u16,
                             int);

extern void avi_set_blendin_cfg(struct avi_node const*,
                                  struct avi_blendin_cfg const*);
extern void avi_get_blendin_cfg(struct avi_node const*,
				struct avi_blendin_cfg*);

extern void avi_setup_blender_input(struct avi_node const*,
                                    unsigned,
                                    unsigned,
                                    unsigned,
                                    int);

extern void avi_setup_blendin(struct avi_node const*, u16, u16, int);
extern void avi_setup_blendin_alpha(struct avi_node const*, int);

static inline void avi_hide_blendin(struct avi_node const *blendin)
{
	/* We hide the blendin by sending it completely out of frame */
	avi_setup_blendin(blendin, 0xffff, 0xffff, 0x0);
}

extern void avi_setup_blend(struct avi_node const*, u32, u16, u16);

extern u32 avi_blend_get_bg(struct avi_node const *);
extern void avi_blend_set_bg(struct avi_node const *, u32);

/**********************
 * Gamma corrector API
 **********************/

#define AVI_CMAP_SZ 0x400

struct avi_cmap
{
	u8 red  [AVI_CMAP_SZ];
	u8 green[AVI_CMAP_SZ];
	u8 blue [AVI_CMAP_SZ];
};

extern void avi_gam_setup(struct avi_node const*, int, int, int);
extern u32  avi_gam_get_config(struct avi_node const*);
extern void avi_gam_set_entry(struct avi_node const*, u16, u8, u8, u8);
extern void avi_gam_set_cmap(struct avi_node const*, struct avi_cmap const *);
extern void avi_gam_get_cmap(struct avi_node const*, struct avi_cmap*);

/*********************
 * LCD controller API
 *********************/

/* Inspired by fb_videomode but duplicated here to avoid a dependency on
 * fbdev */

#define AVI_VIDEOMODE_INTERLACED	(1 << 0)
#define AVI_VIDEOMODE_HSYNC_ACTIVE_HIGH (1 << 4)
#define AVI_VIDEOMODE_VSYNC_ACTIVE_HIGH (1 << 5)

#define AVI_VIDEOMODE_SYNC_ACTIVE_HIGH		\
	(AVI_VIDEOMODE_HSYNC_ACTIVE_HIGH |	\
	 AVI_VIDEOMODE_VSYNC_ACTIVE_HIGH)

struct avi_videomode {
	const char	*name;
	u32		 xres;
	u32		 yres;
	/* pixclock in kHz *unlike* fb_videomode where it's in ps. */
	u32		 pixclock;
	u32		 left_margin;
	u32		 right_margin;
	u32		 upper_margin;
	u32		 lower_margin;
	u32		 hsync_len;
	u32		 vsync_len;
	u32		 flags;
};

extern void avi_lcd_get_registers(struct avi_node const*,
                                  struct avi_lcd_regs *);
extern void avi_lcd_set_registers(struct avi_node const*,
                                  struct avi_lcd_regs const*);
extern enum avi_field avi_lcd_get_current_field(struct avi_node const*);

extern unsigned avi_lcd_get_colorbar(struct avi_node const*);
extern void     avi_lcd_set_colorbar(struct avi_node const*, unsigned);
extern unsigned avi_lcd_get_force_clear(struct avi_node const *);
extern void     avi_lcd_set_force_clear(struct avi_node const *, unsigned);
extern union avi_lcd_status avi_lcd_get_status(struct avi_node const*);

extern u32  avi_lcd_get_dpd(struct avi_node const *);
extern void avi_lcd_set_dpd(struct avi_node const *, u32);

extern int avi_lcd_pixclock_enable(struct avi_node const *);
extern int avi_lcd_pixclock_disable(struct avi_node const *);
extern unsigned long avi_lcd_pixclock_round_rate(struct avi_node const *,
						 unsigned long );
extern int avi_lcd_pixclock_set_rate(struct avi_node const *,
				     unsigned long);
extern unsigned long avi_lcd_pixclock_get_rate(struct avi_node const *);

extern void avi_vsync_gen_get_registers(struct avi_node const*,
					struct avi_lcd_avi_vsync_gen_regs *);
extern void avi_vsync_gen_set_registers(struct avi_node const*,
					struct avi_lcd_avi_vsync_gen_regs const*);

extern union avi_cam_status avi_cam_get_status(struct avi_node const*);
extern void avi_cam_get_measure(struct avi_node const*,
                                struct avi_cam_measure_regs *);
extern void avi_cam_set_itsource(struct avi_node const*,
                                 union avi_cam_itsource);
extern void avi_cam_set_timings(struct avi_node const*,
                                union avi_cam_h_timing,
                                union avi_cam_v_timing);
extern void avi_cam_set_run(struct avi_node const*, union avi_cam_run);
extern void avi_cam_run(struct avi_node const*);
extern void avi_cam_freerun(struct avi_node const*);

extern void avi_cam_set_registers(struct avi_node const*,
                                  struct avi_cam_registers const*);
extern void avi_cam_get_registers(struct avi_node const *,
                                  struct avi_cam_registers *);
extern void avi_inter_get_registers(u32[AVI_INTER_NSINK]);

/**
 * Rotator
 */
enum avi_rotation
{
	AVI_ROTATION_0,
	AVI_ROTATION_90,
	AVI_ROTATION_180,
	AVI_ROTATION_270,
	AVI_ROTATION_NR
};

extern void avi_rotator_setup(struct avi_node const*,
			      enum avi_rotation,
			      bool,
			      unsigned height_out,
			      unsigned nstrips);

/**
 * struct avi_rect - AVI frame area / rectangle descriptor.
 * @top: position / offset from top of containing frame in number of pixels
 * @left: position / offset from left of containing frame in number of pixels
 * @width: rectangle width in pixels
 * @height: rectangle height in pixels
 */
struct avi_rect {
    u16 top;
    u16 left;
    u16 width;
    u16 height;
};

/**********
 * ISP API
 **********/
/* Inter */
extern void avi_isp_chain_bayer_inter_set_registers(
		struct avi_node const *,
		struct avi_isp_chain_bayer_inter_regs const *);

extern void avi_isp_chain_bayer_inter_get_registers(
		struct avi_node const *,
		struct avi_isp_chain_bayer_inter_regs *);

extern void avi_isp_chain_yuv_inter_set_registers(
		struct avi_node const *,
		struct avi_isp_chain_yuv_inter_regs const *);

extern void avi_isp_chain_yuv_inter_get_registers(
		struct avi_node const *,
		struct avi_isp_chain_yuv_inter_regs *);

/* Bayer */
extern void avi_isp_bayer_set_registers(struct avi_node const *,
					struct avi_isp_bayer_regs const *);
extern void avi_isp_bayer_get_registers(struct avi_node const *,
					struct avi_isp_bayer_regs *);

/* Stats */
extern void avi_isp_statistics_bayer_set_registers(
	struct avi_node *,
	const struct avi_isp_statistics_bayer_regs *);

extern void avi_isp_statistics_bayer_get_registers(
	struct avi_node *,
	struct avi_isp_statistics_bayer_regs *);

/* Pedestal */
extern void avi_isp_pedestal_get_registers(struct avi_node *chain_bayer,
					   struct avi_isp_pedestal_regs *regs);

/* Green imbalance */
extern void avi_isp_grim_get_registers(struct avi_node *chain_bayer,
		struct avi_isp_green_imbalance_regs *regs,
		struct avi_isp_green_imbalance_green_red_coeff_mem_regs *gr_regs,
		struct avi_isp_green_imbalance_green_blue_coeff_mem_regs *gb_regs);

/* Dead pixel correction + RGrim */
extern void avi_isp_dpc_rgrim_get_registers(struct avi_node *chain_bayer,
		struct avi_isp_dead_pixel_correction_regs *regs,
		struct avi_isp_dead_pixel_correction_list_mem_regs *list);

/* Denoising */
extern void avi_isp_denoising_get_registers(struct avi_node *chain_bayer,
					    struct avi_isp_denoising_regs *regs);

/* LSC + WB Gain */
extern void avi_isp_lsc_get_registers(struct avi_node *chain_bayer,
		struct avi_isp_lens_shading_correction_regs *regs,
		struct avi_isp_lens_shading_correction_red_coeff_mem_regs *r_regs,
		struct avi_isp_lens_shading_correction_green_coeff_mem_regs *g_regs,
		struct avi_isp_lens_shading_correction_blue_coeff_mem_regs *b_regs);

/* Chromatic aberration correction */
extern void avi_isp_ca_correction_get_registers(struct avi_node *chain_bayer,
		struct avi_isp_chromatic_aberration_regs *regs);

/* Color correction */
extern void avi_isp_color_correction_get_registers(struct avi_node *chain_bayer,
		struct avi_isp_color_correction_regs *regs);
#endif
