/*
 *  linux/drivers/parrot/video/avi_debugfs.c
 *
 *  Copyright (C) 2011 Parrot S.A.
 *
 * @author  Lionel Flandrin <lionel.flandrin@parrot.com>
 *   @date  27-Jul-2011
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

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include "avi_compat.h"
#include "avi_segment.h"
#include "avi_debug.h"
#include "avi_logger.h"

/* The root of our debugfs filesystem. */
static struct dentry *avi_debugfs_root = NULL;
/* Subdirs for ISP */
static struct dentry *avi_isp0_debugfs_root = NULL;
static struct dentry *avi_isp1_debugfs_root = NULL;

static const char *avi_inter_src_name[] =
{
	"unconnected",
	"FIFO00          ",
	"FIFO01          ",
	"FIFO02          ",
	"FIFO03          ",
	"FIFO04          ",
	"FIFO05          ",
	"FIFO06          ",
	"FIFO07          ",
	"FIFO08          ",
	"FIFO09          ",
	"FIFO10          ",
	"FIFO11          ",
	"CONV0           ",
	"CONV1           ",
	"CONV2           ",
	"CONV3           ",
	"BLEND0          ",
	"BLEND1          ",
	"CAM0            ",
	"CAM1            ",
	"CAM2            ",
	"CAM3            ",
	"CAM4            ",
	"CAM5            ",
	"SCAL0           ",
	"ROT0            ",
	"SCAL1           ",
	"ROT1            ",
	"GAM0            ",
	"GAM1            ",
	"FORK0_0         ",
	"FORK0_1         ",
	"FORK1_0         ",
	"FORK1_1         ",
	"FORK2_0         ",
	"FORK2_1         ",
	"FORK3_0         ",
	"FORK3_1         ",
	"SAT_0           ",
	"SAT_1           ",
	"STATS_YUV0      ",
	"STATS_YUV1      ",
	"STATS_BAYER0    ",
	"STATS_BAYER1    ",
	"ISP_CHAIN_BAYER0",
	"ISP_CHAIN_BAYER1",
	"ISP_CHAIN_YUV0  ",
	"ISP_CHAIN_YUV1  ",
};

static void avi_inter_display_sel(struct seq_file *s, const char *sink, u8 sel)
{
	if (sel == 0)
		/* The node is not connected, don't display it */
		return;

	if (sel < ARRAY_SIZE(avi_inter_src_name))
		seq_printf(s, "%s -> %s\n", avi_inter_src_name[sel], sink);
	else
		seq_printf(s, "INVAL(0x%02x) -> %s\n", sel, sink);
}

static int avi_inter_show(struct seq_file *s, void *unused)
{
	u32 regs[AVI_INTER_NSINK];
	u8  *srcsel;

	avi_inter_get_registers(regs);

	srcsel = (u8*)regs;

	avi_inter_display_sel(s, "FIFO00",           *(srcsel++));
	avi_inter_display_sel(s, "FIFO01",           *(srcsel++));
	avi_inter_display_sel(s, "FIFO02",           *(srcsel++));
	avi_inter_display_sel(s, "FIFO03",           *(srcsel++));
	avi_inter_display_sel(s, "FIFO04",           *(srcsel++));
	avi_inter_display_sel(s, "FIFO05",           *(srcsel++));
	avi_inter_display_sel(s, "FIFO06",           *(srcsel++));
	avi_inter_display_sel(s, "FIFO07",           *(srcsel++));
	avi_inter_display_sel(s, "FIFO08",           *(srcsel++));
	avi_inter_display_sel(s, "FIFO09",           *(srcsel++));
	avi_inter_display_sel(s, "FIFO10",           *(srcsel++));
	avi_inter_display_sel(s, "FIFO11",           *(srcsel++));
	avi_inter_display_sel(s, "CONV0",            *(srcsel++));
	avi_inter_display_sel(s, "CONV1",            *(srcsel++));
	avi_inter_display_sel(s, "CONV2",            *(srcsel++));
	avi_inter_display_sel(s, "CONV3",            *(srcsel++));
	avi_inter_display_sel(s, "BLEND0_0",         *(srcsel++));
	avi_inter_display_sel(s, "BLEND0_1",         *(srcsel++));
	avi_inter_display_sel(s, "BLEND0_2",         *(srcsel++));
	avi_inter_display_sel(s, "BLEND0_3",         *(srcsel++));
	avi_inter_display_sel(s, "BLEND1_0",         *(srcsel++));
	avi_inter_display_sel(s, "BLEND1_1",         *(srcsel++));
	avi_inter_display_sel(s, "BLEND1_2",         *(srcsel++));
	avi_inter_display_sel(s, "BLEND1_3",         *(srcsel++));
	avi_inter_display_sel(s, "GAM0",             *(srcsel++));
	avi_inter_display_sel(s, "GAM1",             *(srcsel++));
	avi_inter_display_sel(s, "SCAL0_0",          *(srcsel++));
	avi_inter_display_sel(s, "SCAL0_1",          *(srcsel++));
	avi_inter_display_sel(s, "ROT0",             *(srcsel++));
	avi_inter_display_sel(s, "SCAL1_0",          *(srcsel++));
	avi_inter_display_sel(s, "SCAL1_1",          *(srcsel++));
	avi_inter_display_sel(s, "ROT1",             *(srcsel++));
	avi_inter_display_sel(s, "LCD0",             *(srcsel++));
	avi_inter_display_sel(s, "LCD1",             *(srcsel++));
	avi_inter_display_sel(s, "FORK0",            *(srcsel++));
	avi_inter_display_sel(s, "FORK1",            *(srcsel++));
	avi_inter_display_sel(s, "FORK2",            *(srcsel++));
	avi_inter_display_sel(s, "FORK3",            *(srcsel++));
	avi_inter_display_sel(s, "SAT0",             *(srcsel++));
	avi_inter_display_sel(s, "SAT1",             *(srcsel++));
	avi_inter_display_sel(s, "STATS_YUV0",       *(srcsel++));
	avi_inter_display_sel(s, "STATS_YUV1",       *(srcsel++));
	avi_inter_display_sel(s, "ISP_CHAIN_BAYER0", *(srcsel++));
	avi_inter_display_sel(s, "ISP_CHAIN_BAYER1", *(srcsel++));
	avi_inter_display_sel(s, "ISP_CHAIN_YUV0",   *(srcsel++));
	avi_inter_display_sel(s, "ISP_CHAIN_YUV1",   *(srcsel++));

	return 0;
}

static void avi_conv_show(struct seq_file *s, const struct avi_node *conv)
{
	struct avi_isp_chroma_regs regs;

	avi_conv_get_registers(conv, &regs);


	seq_printf(s, "[ %04x %04x %04x ]   / [RY]   [ %02x ] \\   [ %02x ];"
		   " %02x <= RY <= %02x\n",
		   regs.coeff_01_00.coeff_00,
		   regs.coeff_01_00.coeff_01,
		   regs.coeff_10_02.coeff_02,
		   regs.offset_ry.offset_in,
		   regs.offset_ry.offset_out,
		   regs.clip_ry.clip_min,
		   regs.clip_ry.clip_max);
	seq_printf(s, "[ %04x %04x %04x ] x | [GU] - [ %02x ] | + [ %02x ];"
		   " %02x <= GU <= %02x\n",
		   regs.coeff_10_02.coeff_10,
		   regs.coeff_12_11.coeff_11,
		   regs.coeff_12_11.coeff_12,
		   regs.offset_gu.offset_in,
		   regs.offset_gu.offset_out,
		   regs.clip_gu.clip_min,
		   regs.clip_gu.clip_max);
	seq_printf(s, "[ %04x %04x %04x ]   \\ [BV]   [ %02x ] /   [ %02x ];"
		   " %02x <= BV <= %02x\n",
		   regs.coeff_21_20.coeff_20,
		   regs.coeff_21_20.coeff_21,
		   regs.coeff_22.coeff_22,
		   regs.offset_bv.offset_in,
		   regs.offset_bv.offset_out,
		   regs.clip_bv.clip_min,
		   regs.clip_bv.clip_max);
}

#define MAKE_AVI_CONV_SHOW(_nr)						\
	static int avi_conv##_nr##_show(struct seq_file *s,		\
					void *unused)			\
	{								\
		avi_conv_show(s, avi_ctrl.nodes[AVI_CONV##_nr##_NODE]);	\
		return 0;						\
	}

MAKE_AVI_CONV_SHOW(0)
MAKE_AVI_CONV_SHOW(1)
MAKE_AVI_CONV_SHOW(2)
MAKE_AVI_CONV_SHOW(3)

static void avi_fifo_show(struct seq_file *s, const struct avi_node *fifo)
{
	struct avi_fifo_registers  regs;
	static const char *fifo_format_to_str[] = {
		[0] = "4:4:4",
		[1] = "4:2:2",
		[2] = "INVALID",
		[3] = "4:2:0",
	};
	static const char *fifobitenc[] = {
		[0] = "4444",
		[1] = "565",
		[2] = "1 LSB vote + 555",
		[3] = "1 alpha + 555",
		[4] = "888",
		[5] = "INVAL",
		[6] = "INVAL",
		[7] = "INVAL",
		[8] = "8888",
	};
#define GET_FIFOBITENC(b) (((b) >= ARRAY_SIZE(fifobitenc)) ? "INVAL" : fifobitenc[b])

	avi_fifo_get_registers(fifo, &regs);

	seq_printf(s, "STATUS:      RUNNING: %x IT_ALL: %x\n",
		   regs.status.running,
		   regs.status.it_all);
	seq_printf(s, "STATUS:      IT_MONITOR0: %x IT_MONITOR1: %x\n",
		   regs.status.it_monitor0,
		   regs.status.it_monitor1);
	seq_printf(s, "STATUS:      IT_NL0: %x IT_NL1: %x\n",
		   regs.status.it_nl0,
		   regs.status.it_nl1);
	seq_printf(s, "CFG:         SRCTYPE: %s DSTTYPE: %s\n",
		   regs.cfg.srctype ? "DMA" : "VL",
		   regs.cfg.dsttype ? "DMA" : "VL");
	seq_printf(s, "CFG:         SRCORG: %s DSTORG: %s\n",
		   regs.cfg.srcorg ? "semi-planar" : "interleaved",
		   regs.cfg.dstorg ? "semi-planar" : "interleaved");
	seq_printf(s, "CFG:         SRCFORMAT: %s DSTFORMAT: %s\n",
		   fifo_format_to_str[regs.cfg.srcformat],
		   fifo_format_to_str[regs.cfg.dstformat]);
	seq_printf(s, "CFG:         SRCBITENC: %s DSTBITENC: %s\n",
		   GET_FIFOBITENC(regs.cfg.srcbitenc),
		   GET_FIFOBITENC(regs.cfg.dstbitenc));
	seq_printf(s, "CFG:         REORG: %x DITHERING: %x\n",
		   regs.cfg.reorg, regs.cfg.dithering);
	seq_printf(s, "CFG:         CDSDIS: %x CUSDIS: %x PHASE: %x\n",
		   regs.cfg.cdsdis, regs.cfg.cusdis, regs.cfg.phase);
	seq_printf(s, "CFG:         SCALPLANE: %x KEYING: %x\n",
		   regs.cfg.scalplane, regs.cfg.keying);
	seq_printf(s, "CFG:         ITSOURCE: %x ITLINE: %x\n",
		   regs.cfg.itsource, regs.cfg.itline);
	seq_printf(s, "CFG:         SINGLE: %x\n", regs.cfg.single);

	seq_printf(s, "FRAMESIZE:   %u x %u\n",
			   regs.framesize.width, regs.framesize.height);
	seq_printf(s, "MONITOR:     0x%08x\n", regs.monitor._register);
	seq_printf(s,
		   "TIMEOUT:     BURST: %d x 64bits ISSUING_CAPABILITY: %d\n",
		   16 << regs.timeout.burst,
		   regs.timeout.issuing_capability + 1);
	seq_printf(s, "TIMEOUT:     PINCR: %d HBLANK: %d VBLANK: %d\n",
		   regs.timeout.pincr, regs.timeout.hblank,
		   regs.timeout.vblank);
	seq_printf(s, "SWAP0:       0x%08x\n", regs.swap0._register);
	seq_printf(s, "SWAP1:       0x%08x\n", regs.swap1._register);
	seq_printf(s, "DMASA0:      0x%08x\n", regs.dmasa0);
	seq_printf(s, "DMASA1:      0x%08x\n", regs.dmasa1);
	seq_printf(s, "DMALSTR0:    %d\n", regs.dmalstr0);
	seq_printf(s, "DMALSTR1:    %d\n", regs.dmalstr1);
	seq_printf(s, "DMAFXYCFG:   %u x %u [%u]\n",
			   regs.dmafxycfg.dmafxnb,
			   regs.dmafxycfg.dmafynb,
			   regs.dmafxycfg.dmafxymode);
	seq_printf(s, "DMAFXSTR0:   %d\n", regs.dmafxstr0);
	seq_printf(s, "DMAFXSTR1:   %d\n", regs.dmafxstr1);
	seq_printf(s, "DMAFYSTR0:   %d\n", regs.dmafystr0);
	seq_printf(s, "DMAFYSTR1:   %d\n", regs.dmafystr1);
	seq_printf(s, "KEYINGMIN:   #%02x%02x%02x\n",
			   regs.keyingmin.ry,
			   regs.keyingmin.gu,
			   regs.keyingmin.bv);
	seq_printf(s, "KEYINGMAX:   #%02x%02x%02x\n",
			   regs.keyingmax.ry,
			   regs.keyingmax.gu,
			   regs.keyingmax.bv);
	seq_printf(s, "KEYINGALPHA: 0x%02x\n", regs.keyingalpha);
	seq_printf(s, "VLIN:        FORCE_CLEAR: %s[%x]\n",
		   avi_debug_force_clear_to_str(regs.vlin.force_clear),
		   regs.vlin.force_clear);
}

#define MAKE_AVI_FIFO_SHOW(_nr)                                 \
	static int avi_fifo##_nr##_show(struct seq_file *s,         \
									void *unused)               \
	{                                                           \
		avi_fifo_show(s, avi_ctrl.nodes[AVI_FIFO##_nr##_NODE]); \
		return 0;                                               \
	}

MAKE_AVI_FIFO_SHOW(00)
MAKE_AVI_FIFO_SHOW(01)
MAKE_AVI_FIFO_SHOW(02)
MAKE_AVI_FIFO_SHOW(03)
MAKE_AVI_FIFO_SHOW(04)
MAKE_AVI_FIFO_SHOW(05)
MAKE_AVI_FIFO_SHOW(06)
MAKE_AVI_FIFO_SHOW(07)
MAKE_AVI_FIFO_SHOW(08)
MAKE_AVI_FIFO_SHOW(09)
MAKE_AVI_FIFO_SHOW(10)
MAKE_AVI_FIFO_SHOW(11)


static void avi_format_show(struct seq_file *s, unsigned format)
{
	static const char *names[] = {
		[AVI_FORMAT_CONTROL_RGB888_1X24] = "RGB888/YUV888_1X24",
		[0x1]                            = "RGB888/YUV888_1X24",
		[0x2]                            = "RGB888/YUV888_1X24",
		[0x3]                            = "RGB888/YUV888_1X24",
#define AVI_FORMAT_CONTROL(_name) [AVI_FORMAT_CONTROL_ ## _name] = #_name
		AVI_FORMAT_CONTROL(YUYV_1X16),
		AVI_FORMAT_CONTROL(YVYU_1X16),
		AVI_FORMAT_CONTROL(UYVY_2X8),
		AVI_FORMAT_CONTROL(VYUY_2X8),
		AVI_FORMAT_CONTROL(YUYV_2X8),
		AVI_FORMAT_CONTROL(YVYU_2X8),
		AVI_FORMAT_CONTROL(RGB888_3X8),
		AVI_FORMAT_CONTROL(BGR888_3X8),
		AVI_FORMAT_CONTROL(RGBX8888_4X8),
		AVI_FORMAT_CONTROL(BGRX8888_4X8),
		AVI_FORMAT_CONTROL(RGB565_2X8),
		AVI_FORMAT_CONTROL(BGR565_2X8),
		AVI_FORMAT_CONTROL(RGB555_2X8),
		AVI_FORMAT_CONTROL(BGR555_2X8),
		AVI_FORMAT_CONTROL(RGB444_2X8),
		AVI_FORMAT_CONTROL(BGR444_2X8),
	};
	const char *name;


	if (format >= ARRAY_SIZE(names) || !names[format])
		name = "invalid";
	else
		name = names[format];

	seq_printf(s, "%s[%u]", name, format);
}

static void avi_cam_show(struct seq_file *s, const struct avi_node *cam)
{
	struct avi_cam_registers    regs;
	struct avi_cam_measure_regs measures;

	avi_cam_get_registers(cam, &regs);
	avi_cam_get_measure(cam, &measures);

	seq_printf(s, "STATUS:      FIELD: %s, FOF: %d, DONE: %d\n",
			   regs.status.field ? "BOT" : "TOP",
			   regs.status.fof, regs.status.done);
	seq_printf(s, "ITSOURCE:    FOF: %d, DONE: %d\n",
			   regs.itsource.fof_en, regs.itsource.done_en);
	seq_printf(s, "INTERFACE:   IVS: %d, IHS: %d, IPC: %d, IOE: %d\n",
			   regs.interface.ivs,
			   regs.interface.ihs,
			   regs.interface.ipc,
			   regs.interface.ioe);
	seq_printf(s, "INTERFACE:   PSYNC_RF: %d, PSYNC_EN: %d\n",
			   regs.interface.psync_rf,
			   regs.interface.psync_en);
	seq_printf(s, "INTERFACE:   ITU656: %d, TIMING_AUTO: %d\n",
			   regs.interface.itu656,
			   regs.interface.timing_auto);
	seq_printf(s, "INTERFACE:   FORMAT_CONTROL: ");
	avi_format_show(s, regs.interface.format_control);
	seq_printf(s, "\n"
		      "INTERFACE:   PAD_SELECT: 0x%x\n",
		   regs.interface.pad_select);
	seq_printf(s, "INTERFACE:   UNPACKER: %d, RAW10: %d\n",
		   regs.interface.unpacker,
		   regs.interface.raw10);
	seq_printf(s, "INTERFACE:   ROL_LSB: %d, ROR_LSB: %d\n",
		   regs.interface.rol_lsb,
		   regs.interface.ror_lsb);
	seq_printf(s, "RUN:         %d\n", regs.run.run);
	seq_printf(s, "FREE_RUN:    %d\n", regs.run.free_run);
	seq_printf(s, "HACTIVE_ON:  %u\n", regs.h_timing.hactive_on);
	seq_printf(s, "HACTIVE_OFF: %u\n", regs.h_timing.hactive_off);
	seq_printf(s, "VACTIVE_ON:  %u\n", regs.v_timing.vactive_on);
	seq_printf(s, "VACTIVE_OFF: %u\n", regs.v_timing.vactive_off);
	seq_printf(s, "resolution: %u x %u\n",
		   regs.h_timing.hactive_off - regs.h_timing.hactive_on,
		   regs.v_timing.vactive_off - regs.v_timing.vactive_on);

	seq_printf(s, "\nMeasures:\n");
	seq_printf(s, "HSYNC_OFF:   %u\n", measures.hsync_off);
	seq_printf(s, "HACTIVE_ON:  %u\n", measures.hactive_on);
	seq_printf(s, "HACTIVE_OFF: %u\n", measures.hactive_off);
	seq_printf(s, "HTOTAL:      %u\n", measures.htotal);
	seq_printf(s, "VSYNC_HON:   %u\n", measures.vsync_hon);
	seq_printf(s, "VSYNC_VON:   %u\n", measures.vsync_von);
	seq_printf(s, "VSYNC_HOFF:  %u\n", measures.vsync_hoff);
	seq_printf(s, "VSYNC_VOFF:  %u\n", measures.vsync_voff);
	seq_printf(s, "VACTIVE_ON:  %u\n", measures.vactive_on);
	seq_printf(s, "VACTIVE_OFF: %u\n", measures.vactive_off);
	seq_printf(s, "VTOTAL:      %u\n", measures.vtotal);

	seq_printf(s, "\nleft_margin/xres/right_margin/hsync_len: "
		   "%u/%u/%u/%u\n",
		   measures.hactive_on  - measures.hsync_off,
		   measures.hactive_off - measures.hactive_on,
		   measures.htotal      - measures.hactive_off,
		   measures.hsync_off);
	seq_printf(s, "top_margin/yres/bot_margin/vsync_len: "
		   "%u/%u/%u/%u\n",
		   measures.vactive_on  - measures.vsync_voff,
		   measures.vactive_off - measures.vactive_on,
		   measures.vtotal      - measures.vactive_off,
		   measures.vsync_voff);
}

#define MAKE_AVI_CAM_SHOW(_nr)						\
	static int avi_cam##_nr##_show(struct seq_file *s,		\
				       void *unused)			\
	{								\
		avi_cam_show(s, avi_ctrl.nodes[AVI_CAM##_nr##_NODE]);   \
		return 0;                                               \
	}

MAKE_AVI_CAM_SHOW(0)
MAKE_AVI_CAM_SHOW(1)
MAKE_AVI_CAM_SHOW(2)
MAKE_AVI_CAM_SHOW(3)
MAKE_AVI_CAM_SHOW(4)
MAKE_AVI_CAM_SHOW(5)

static void avi_gam_show(struct seq_file *s, const struct avi_node *gam)
{
	union avi_isp_gamma_corrector_conf	 cfg;
	struct avi_cmap				*cmap;
	unsigned				 i;
	unsigned				 column_sz;

	/* struct avi_cmap is too big to be safely allocated on the stack (it
	 * contains all the lookup tables) */
	cmap = kzalloc(sizeof(*cmap), GFP_KERNEL);

	if (!cmap) {
		seq_printf(s, "Error: can't allocate cmap struct\n");
		return;
	}

	cfg._register = avi_gam_get_config(gam);

	seq_printf(s, "BYPASS:     %d\n",  cfg.bypass);
	seq_printf(s, "PALETTE:    %d\n",  cfg.palette);
	seq_printf(s, "COMP_WIDTH: %db\n", cfg.comp_width ? 10 : 8);

	avi_gam_get_cmap(gam, cmap);

	seq_printf(s, "\n"
		   "ID : RY GU BV | "
		   "ID : RY GU BV | "
		   "ID : RY GU BV | "
		   "ID : RY GU BV\n");

	/* Divide CLUT display in 4 columns. */
	BUILD_BUG_ON(AVI_CMAP_SZ % 4 != 0);
	column_sz = AVI_CMAP_SZ / 4;

	for (i = 0; i < column_sz; i++) {
		unsigned c1 = i;
		unsigned c2 = c1 + column_sz;
		unsigned c3 = c2 + column_sz;
		unsigned c4 = c3 + column_sz;

		seq_printf(s,
			   "%03x: %02x %02x %02x | "
			   "%03x: %02x %02x %02x | "
			   "%03x: %02x %02x %02x | "
			   "%03x: %02x %02x %02x\n",
			   c1, cmap->red[c1], cmap->green[c1], cmap->blue [c1],
			   c2, cmap->red[c2], cmap->green[c2], cmap->blue [c2],
			   c3, cmap->red[c3], cmap->green[c3], cmap->blue [c3],
			   c4, cmap->red[c4], cmap->green[c4], cmap->blue [c4]);
	}

	kfree(cmap);
}

#define MAKE_AVI_GAM_SHOW(_nr)						\
	static int avi_gam##_nr##_show(struct seq_file *s,		\
				       void *unused)			\
	{								\
		avi_gam_show(s, avi_ctrl.nodes[AVI_GAM##_nr##_NODE]);   \
		return 0;                                               \
	}

MAKE_AVI_GAM_SHOW(0)
MAKE_AVI_GAM_SHOW(1)

static void avi_lcd_show(struct seq_file *s, const struct avi_node *lcd)
{
	struct avi_lcd_regs			regs;
	struct avi_lcd_avi_vsync_gen_regs	vsync_gen_regs;

	avi_lcd_get_registers(lcd, &regs);

	seq_printf(s, "STATUS:      FIELD: %s, FUF: %d, DONE: %d\n",
		   regs.status.field ? "BOT" : "TOP",
		   regs.status.fuf, regs.status.done);
	seq_printf(s, "ITSOURCE:    FUF: %d, DONE: %d\n",
		   regs.itsource.fuf_en, regs.itsource.done_en);
	seq_printf(s, "INTERFACE:   IVS: %d, IHS: %d, IPC: %d, IOE: %d\n",
		   regs.interface.ivs,
		   regs.interface.ihs,
		   regs.interface.ipc,
		   regs.interface.ioe);
	seq_printf(s, "INTERFACE:   PSYNC_RF: %d, PSYNC_EN: %d\n",
		   regs.interface.psync_rf,
		   regs.interface.psync_en);
	seq_printf(s, "INTERFACE:   ITU656: %d, PROG: %d\n",
		   regs.interface.itu656,
		   regs.interface.prog);
	seq_printf(s, "INTERFACE:   CLIP_EN: %d, FREE_RUN: %d\n",
		   regs.interface.clip_en,
		   regs.interface.free_run);
	seq_printf(s, "INTERFACE:   PAD_SELECT: %d, VSYNC_GEN: %d\n",
		   regs.interface.pad_select,
		   regs.interface.vsync_gen);
	seq_printf(s, "DPD:         0x%06x\n", regs.dpd.dpd);
	switch (regs.dpd.colorbar) {
	case 0:
	case 1:
		seq_printf(s, "COLORBAR:    NONE\n");
		break;
	case 2:
		seq_printf(s, "COLORBAR:    YUV\n");
		break;
	case 3:
		seq_printf(s, "COLORBAR:    RGB\n");
		break;
	}

#ifdef AVI_BACKWARD_COMPAT
	if (avi_get_revision() >= AVI_REVISION_3) {
#endif /* AVI_BACKWARD_COMPAT */
	seq_printf(s, "FORCE_CLEAR: %s[%x]\n",
	           avi_debug_force_clear_to_str(regs.force_clear.force_clear),
		   regs.force_clear.force_clear);
#ifdef AVI_BACKWARD_COMPAT
	}
#endif /* AVI_BACKWARD_COMPAT */

#define PRINT_TIMINGS(w) do {						\
	seq_printf(s, "FORMAT_CONTROL: ");				\
	avi_format_show(s, regs.w##_format_ctrl.w##_format_control);	\
	seq_printf(s, "\n");						\
	seq_printf(s, "HSYNC_OFF:      %u\n",				\
		   regs.w##_h_timing0.w##_hsync_off);			\
	seq_printf(s, "HACTIVE_ON:     %u\n",				\
		   regs.w##_h_timing0.w##_hactive_on);			\
	seq_printf(s, "HACTIVE_OFF:    %u\n",				\
		   regs.w##_h_timing1.w##_hactive_off);			\
	seq_printf(s, "HTOTAL:         %u\n",				\
		   regs.w##_h_timing1.w##_htotal);			\
	seq_printf(s, "VSYNC_HON:      %u\n",				\
		   regs.w##_v_timing0.w##_vsync_hon);			\
	seq_printf(s, "VSYNC_VON:      %u\n",				\
		   regs.w##_v_timing0.w##_vsync_von);			\
	seq_printf(s, "VSYNC_HOFF:     %u\n",				\
		   regs.w##_v_timing1.w##_vsync_hoff);			\
	seq_printf(s, "VSYNC_VOFF:     %u\n",				\
		   regs.w##_v_timing1.w##_vsync_voff);			\
	seq_printf(s, "VACTIVE_ON:     %u\n",				\
		   regs.w##_v_timing2.w##_vactive_on);			\
	seq_printf(s, "VACTIVE_OFF:    %u\n",				\
		   regs.w##_v_timing2.w##_vactive_off);			\
	seq_printf(s, "VTOTAL:         %u\n",				\
		   regs.w##_v_timing3.w##_vtotal);			\
	seq_printf(s, "\nleft_margin/xres/right_margin/hsync_len: "	\
		   "%u/%u/%u/%u\n",					\
		   regs.w##_h_timing0.w##_hactive_on			\
		   - regs.w##_h_timing0.w##_hsync_off,			\
		   regs.w##_h_timing1.w##_hactive_off			\
		   - regs.w##_h_timing0.w##_hactive_on,			\
		   regs.w##_h_timing1.w##_htotal			\
		   - regs.w##_h_timing1.w##_hactive_off,		\
		   regs.w##_h_timing0.w##_hsync_off);			\
	seq_printf(s, "top_margin/yres/bot_margin/vsync_len: "		\
		   "%u/%u/%u/%u\n",					\
		   regs.w##_v_timing2.w##_vactive_on			\
		   - regs.w##_v_timing1.w##_vsync_voff,			\
		   regs.w##_v_timing2.w##_vactive_off			\
		   - regs.w##_v_timing2.w##_vactive_on,			\
		   regs.w##_v_timing3.w##_vtotal			\
		   - regs.w##_v_timing2.w##_vactive_off,		\
		   regs.w##_v_timing1.w##_vsync_voff);			\
	} while(0);

	seq_printf(s, "\nTOP timings:\n");
	PRINT_TIMINGS(top);
	seq_printf(s, "\nBOT timings:\n");
	PRINT_TIMINGS(bot);

	avi_vsync_gen_get_registers(lcd, &vsync_gen_regs);
	seq_printf(s, "\nVSync generator:\n");

#define PRINT_VSYNC(_n)							\
	seq_printf(s, #_n ": %u:%u -> %u:%u\n",				\
		   vsync_gen_regs.vsync_gen_on_##_n.vsync_gen_von,	\
		   vsync_gen_regs.vsync_gen_on_##_n.vsync_gen_hon,	\
		   vsync_gen_regs.vsync_gen_off_##_n.vsync_gen_voff,	\
		   vsync_gen_regs.vsync_gen_off_##_n.vsync_gen_hoff)

	PRINT_VSYNC(0);
	PRINT_VSYNC(1);
	PRINT_VSYNC(2);
	PRINT_VSYNC(3);
	PRINT_VSYNC(4);
}

#define MAKE_AVI_LCD_SHOW(_nr)						\
	static int avi_lcd##_nr##_show(struct seq_file *s,		\
				       void *unused)			\
	{								\
		avi_lcd_show(s, avi_ctrl.nodes[AVI_LCD##_nr##_NODE]);   \
		return 0;                                               \
	}

MAKE_AVI_LCD_SHOW(0)
MAKE_AVI_LCD_SHOW(1)

static void avi_blend_show(struct seq_file *s, const struct avi_node *blend)
{
	unsigned long	base = avi_node_base(blend);
	u32		val;
	unsigned	i;

	seq_printf(s, "BACKGROUND: #%06x\n",
		   AVI_READ(base + AVI_BLEND_BACKGROUND));
	val = AVI_READ(base + AVI_BLEND_FRAMEOUTSIZE);
	seq_printf(s, "FRAMEOUTSIZE: %u x %u\n",
		   val & 0xffff, val >> 16);

	/* Display the 4 plane configurations */
	for (i = 0; i < 4; i++) {
		val = AVI_READ(base + AVI_BLEND_OFFSET0 + (i * 8));
		seq_printf(s, "PLANE%u: x: %04x y: %04x alpha: %03x\n",
			   i, val & 0xffff, val >> 16,
			   AVI_READ(base + AVI_BLEND_ALPHA0 + (i * 8)));
	}
}

#define MAKE_AVI_BLEND_SHOW(_nr)					  \
	static int avi_blend##_nr##_show(struct seq_file *s,		  \
					 void *unused)			  \
	{								  \
		avi_blend_show(s, avi_ctrl.nodes[AVI_BLEND##_nr##_NODE]); \
		return 0;						  \
	}

MAKE_AVI_BLEND_SHOW(0)
MAKE_AVI_BLEND_SHOW(1)

static void avi_scal_coeff_histo(struct seq_file *s, unsigned tap,
				 unsigned n, u8 val)
{
	char	histo[66];
	/* values are stored in 8 bit in 2's complement */
	s8	sval = (s8)val;
	int	index = sval * 2;

	memset(histo, ' ', sizeof(histo));
        histo[65] = '\0';
	histo[32] = '|';

	/* Check if the value of the coeff is out of range */
	if (index > 128)
		histo[64] = '>';
	else if (index < -128)
		histo[0]  = '<';
	else
		histo[(index + 128) / 4] = 'X';

	seq_printf(s, " %u-%02d: % 4d [%s]\n", tap, n, sval, histo);
}

static void avi_scal_dump_coeff(struct seq_file *s, unsigned base, unsigned ntap)
{
	u32			val;
	unsigned		i;
	unsigned		j;
	unsigned		nphases;
	union avi_scal_coeff	coeff;

	val = AVI_READ(base + AVI_SCAL_OFFSET) & 0xffffff;
	seq_printf(s, "OFFSET: %u\n", val);

	val = AVI_READ(base + AVI_SCAL_INCR) & 0xffffff;

	seq_printf(s, "INCR:   %u/256 (~%u)\n", val, val >> 8);

	memcpy_from_registers(&coeff, base + AVI_SCAL_COEFF, sizeof(coeff));

	nphases = (32 / ntap);

	for (i = 0; i < ntap; i++)
		for (j = nphases; j--;)
			avi_scal_coeff_histo(s,
					     i,
					     i * nphases + j,
					     coeff.bytes[i * nphases + j]);

	seq_printf(s, "Phase sums:");
	for (j = nphases; j--; ) {
		int sum = 0;
		for (i = 0; i < ntap; i++)
			sum += (s8)coeff.bytes[i * nphases + j];
		seq_printf(s, " %d", sum);
	}
	seq_printf(s, "\n");
}

static void avi_scal_dump_plane(struct seq_file *s, unsigned base)
{
	u32		val;
	unsigned	ntapsx;
	unsigned	ntapsy;

	val = AVI_READ(base + AVI_SCAL_NTAPS_NPHASES);

	ntapsx = 2 << ((val >> AVI_SCAL_NTAPSX_SHIFT) & 3);
	ntapsy = 2 << ((val >> AVI_SCAL_NTAPSY_SHIFT) & 3);

	seq_printf(s,
		   "NTAPS:   x: %u y: %u\n"
		   "NPHASES: x: %u y: %u\n",
		   ntapsx, ntapsy,
		   2 << ((val >> AVI_SCAL_NPHASESX_SHIFT) & 3),
		   2 << ((val >> AVI_SCAL_NPHASESY_SHIFT) & 3));

	seq_printf(s, "Horizontal coefficients:\n");
	avi_scal_dump_coeff(s, base + AVI_SCAL_HORIZ_DIM, ntapsx);
	seq_printf(s, "Vertical coefficients:\n");
	avi_scal_dump_coeff(s, base + AVI_SCAL_VERT_DIM, ntapsy);
}

static void avi_scal_show(struct seq_file *s, const struct avi_node *scal)
{
	unsigned long	base = avi_node_base(scal);
	u32		val;

	char const * const scal_inter[] = {
		[0] = "Unconnected",
		[1] = "VL Input",
		[2] = "Horizontal scaler",
		[3] = "Vertical scaler",
	};

	val = AVI_READ(base + AVI_SCAL_CONF);
	seq_printf(s,
		   "CFG:     OUTPUT_SRC:     %s\n"
		   "CFG:     HORIZONTAL_SRC: %s\n"
		   "CFG:     VERTICAL_SRC:   %s\n",
		   scal_inter[(val >> AVI_SCAL_CONF_OUTSRC_SHIFT) & 3],
		   scal_inter[(val >> AVI_SCAL_CONF_HORIZSRC_SHIFT) & 3],
		   scal_inter[(val >> AVI_SCAL_CONF_VERTSRC_SHIFT) & 3]);
	seq_printf(s,
		   "CFG:     SEMIPLANAR: %d FORMATOUT: %s\n",
		   (val >> AVI_SCAL_CONF_PLANAR_SHIFT) & 1,
		   ((val >> AVI_SCAL_CONF_OUTSRC_SHIFT) & 1) ?
		   "4:2:2" : "4:4:4");

	val = AVI_READ(base + AVI_SCAL_SIZEOUT);
	seq_printf(s,
		   "SIZEOUT: %u x %u\n", val & 0xffff, val >> 16);

	seq_printf(s, "AR or Y plane:\n");
	avi_scal_dump_plane(s, base + AVI_SCAL_PLANE0);
	seq_printf(s, "GB or UV plane:\n");
	avi_scal_dump_plane(s, base + AVI_SCAL_PLANE1);
}

#define MAKE_AVI_SCAL_SHOW(_nr)						 \
	static int avi_scal##_nr##_show(struct seq_file *s,		 \
					void *unused)			 \
	{								 \
		avi_scal_show(s, avi_ctrl.nodes[AVI_SCAL##_nr##0_NODE]); \
		return 0;						 \
	}

MAKE_AVI_SCAL_SHOW(0)
MAKE_AVI_SCAL_SHOW(1)

static void avi_rot_show(struct seq_file *s, const struct avi_node *rot)
{
	unsigned long	   base = avi_node_base(rot);
	u32		   val;
	char const * const angle[] = {
		[AVI_ROT_ANGLE_0]   = "0",
		[AVI_ROT_ANGLE_90]  = "90",
		[AVI_ROT_ANGLE_270] = "270",
		[AVI_ROT_ANGLE_180] = "180",
	};

	val = AVI_READ(base + AVI_ROT_ANGLE);

	seq_printf(s, "ANGLE: %s degrees HORIZONTAL FLIP: %d\n",
		   angle[val & AVI_ROT_ANGLE_MASK],
		   !!(val & AVI_ROT_HORIZ_FLIP));

	val = AVI_READ(base + AVI_ROT_SIZE);
	seq_printf(s, "HEIGHTOUT: %d NSTRIPE: %d\n", val >> 16, val & 0xffff);
}

#define MAKE_AVI_ROT_SHOW(_nr)						\
	static int avi_rot##_nr##_show(struct seq_file *s,		\
				       void *unused)			\
	{								\
		avi_rot_show(s, avi_ctrl.nodes[AVI_ROT##_nr##_NODE]);	\
		return 0;						\
	}

MAKE_AVI_ROT_SHOW(0)
MAKE_AVI_ROT_SHOW(1)

static void avi_isp_chain_bayer_inter_show(struct seq_file *s,
					   const struct avi_node *chain_bayer)
{
	struct avi_isp_chain_bayer_inter_regs regs;

	avi_isp_chain_bayer_inter_get_registers(chain_bayer, &regs);

	seq_printf(s, "PEDESTAL: %d\n", regs.module_bypass.pedestal_bypass);
	seq_printf(s, "GRIM:     %d\n", regs.module_bypass.grim_bypass);
	seq_printf(s, "DPC:      %d\n", regs.module_bypass.rip_bypass);
	seq_printf(s, "DENOISE:  %d\n", regs.module_bypass.denoise_bypass);
	seq_printf(s, "LSC:      %d\n", regs.module_bypass.lsc_bypass);
	seq_printf(s, "CAC:      %d\n", regs.module_bypass.chroma_aber_bypass);
	seq_printf(s, "BAYER:    %d\n", regs.module_bypass.bayer_bypass);
	seq_printf(s, "CC:       %d\n", regs.module_bypass.color_matrix_bypass);
}

#define MAKE_AVI_ISP_CHAIN_BAYER_INTER_SHOW(_nr)                             \
	static int avi_isp_chain_bayer_inter##_nr##_show(struct seq_file *s, \
							 void *unused)       \
	{                                                                    \
		avi_isp_chain_bayer_inter_show(s,                            \
			avi_ctrl.nodes[AVI_ISP_CHAIN_BAYER##_nr##_NODE]);    \
		return 0;                                                    \
	}

MAKE_AVI_ISP_CHAIN_BAYER_INTER_SHOW(0)
MAKE_AVI_ISP_CHAIN_BAYER_INTER_SHOW(1)

static const char *cfa_format_to_str[] = {
	[0] = "BGGR",
	[1] = "RGGB",
	[2] = "GBRG",
	[3] = "GRBG",
};

static void avi_isp_bayer_show(struct seq_file *s,
			       const struct avi_node *chain_bayer)
{
	struct avi_isp_bayer_regs regs;

	avi_isp_bayer_get_registers(chain_bayer, &regs);

	seq_printf(s, "CFA:        %s\n", cfa_format_to_str[regs.cfa.cfa]);
	seq_printf(s, "THRESHOLD1: %d\n", regs.threshold_1.threshold_1);
	seq_printf(s, "THRESHOLD2: %d\n", regs.threshold_2.threshold_2);
}

#define MAKE_AVI_ISP_BAYER_SHOW(_nr)					    \
	static int avi_isp_bayer##_nr##_show(struct seq_file *s,	    \
					     void *unused)		    \
	{								    \
		avi_isp_bayer_show(s,					    \
			avi_ctrl.nodes[AVI_ISP_CHAIN_BAYER##_nr##_NODE]);   \
		return 0;						    \
	}

MAKE_AVI_ISP_BAYER_SHOW(0)
MAKE_AVI_ISP_BAYER_SHOW(1)

static void avi_stats_bayer_show(struct seq_file *s,
				 struct avi_node *chain_bayer)
{
	struct avi_isp_statistics_bayer_regs	regs;
	unsigned				x_center;
	unsigned				y_center;
	unsigned				x_squared;
	unsigned				y_squared;
	int					squared_ok;

	avi_isp_statistics_bayer_get_registers(chain_bayer, &regs);

	seq_printf(s, "WINDOW:        %dx%d\n",
		   regs.window_x.x_width,
		   regs.window_y.y_width);
	seq_printf(s, "OFFSETS:       %dx%d\n",
		   regs.window_x.x_offset,
		   regs.window_y.y_offset);
	seq_printf(s, "MAX_NB_WIN:    %dx%d\n",
		   regs.max_nb_windows.x_window_count,
		   regs.max_nb_windows.y_window_count);
	seq_printf(s, "MEASURE_REQ:   CLEAR: %d\n",
		   regs.measure_req.clear);

	x_center = regs.circle_pos_x_center.x_center;
	y_center = regs.circle_pos_y_center.y_center;

	seq_printf(s, "CIRCLE:        CENTER: %dx%d\n", x_center, y_center);

	x_squared = regs.circle_pos_x_squared.x_squared;
	y_squared = regs.circle_pos_y_squared.y_squared;

	squared_ok = (x_center * x_center == x_squared) &&
		     (y_center * y_center == y_squared);

	seq_printf(s, "CIRCLE:        SQUARED: %dx%d [%s]\n",
		   regs.circle_pos_x_squared.x_squared,
		   regs.circle_pos_y_squared.y_squared,
		   squared_ok ? "OK" : "NOK");

	seq_printf(s, "INCR_LOG2:     %dx%d\n",
		   regs.increments_log2.x_log2_inc,
		   regs.increments_log2.y_log2_inc);
	seq_printf(s, "SAT_THRESHOLD: %d\n",
		   regs.sat_threshold.threshold);
	seq_printf(s, "CFA:           %s\n",
		   cfa_format_to_str[regs.cfa.cfa]);
}

#define MAKE_AVI_STATS_BAYER_SHOW(_nr)					    \
	static int avi_stats_bayer##_nr##_show(struct seq_file *s,	    \
					     void *unused)		    \
	{								    \
		avi_stats_bayer_show(s,					    \
			avi_ctrl.nodes[AVI_ISP_CHAIN_BAYER##_nr##_NODE]);   \
		return 0;						    \
	}

MAKE_AVI_STATS_BAYER_SHOW(0)
MAKE_AVI_STATS_BAYER_SHOW(1)

static void avi_isp_pedestal_show(struct seq_file *s,
				  struct avi_node *chain_bayer)
{
	struct avi_isp_pedestal_regs regs;

	avi_isp_pedestal_get_registers(chain_bayer, &regs);
	seq_printf(s, "CFA:  %s\n", cfa_format_to_str[regs.cfa.cfa]);
	seq_printf(s, "SUB_R:  %u\n", regs.sub_r.sub_r);
	seq_printf(s, "SUB_GB: %u\n", regs.sub_gb.sub_gb);
	seq_printf(s, "SUB_GR: %u\n", regs.sub_gr.sub_gr);
	seq_printf(s, "SUB_B:  %u\n", regs.sub_b.sub_b);
}

#define MAKE_AVI_ISP_PEDESTAL_SHOW(_nr) \
	static int avi_isp_pedestal##_nr##_show(struct seq_file *s, \
						void *unused) \
	{ \
		avi_isp_pedestal_show(s, \
				avi_ctrl.nodes[AVI_ISP_CHAIN_BAYER##_nr##_NODE]); \
		return 0; \
	}

MAKE_AVI_ISP_PEDESTAL_SHOW(0)
MAKE_AVI_ISP_PEDESTAL_SHOW(1)

static void avi_isp_grim_show(struct seq_file *s,
			      struct avi_node *chain_bayer)
{
	struct avi_isp_green_imbalance_regs regs;
	struct avi_isp_green_imbalance_green_red_coeff_mem_regs *gr_regs;
	struct avi_isp_green_imbalance_green_blue_coeff_mem_regs *gb_regs;
	int i;

	/* gr and gb regs make function exceed frame size so allocate them
	 * on the heap */
	gr_regs = kzalloc(sizeof(*gr_regs), GFP_KERNEL);
	gb_regs = kzalloc(sizeof(*gb_regs), GFP_KERNEL);

	avi_isp_grim_get_registers(chain_bayer, &regs, gr_regs, gb_regs);
	seq_printf(s, "CFA:           %s\n",
			cfa_format_to_str[regs.bayer_cfa.cfa]);
	seq_printf(s, "CELL SIZE:     %ux%u\n", regs.cell_w.cell_w,
			regs.cell_h.cell_h);
	seq_printf(s, "CELL SIZE INV: %ux%u\n", regs.cell_w_inv.w_inv,
			regs.cell_h_inv.h_inv);
	seq_printf(s, "CELL IDS:      %u,%u\n", regs.cell_id_x_y.cell_id_x,
			regs.cell_id_x_y.cell_id_y);
	seq_printf(s, "OFFSETS:       %u,%u\n", regs.offset_x_y.offset_x,
			regs.offset_x_y.offset_y);
	seq_printf(s, "ALPHA:         %u\n", regs.alpha.alpha);
	seq_printf(s, "BETA:          %u\n", regs.beta.beta);

	/* display Gr and Gb coefficients */
	seq_printf(s, "\n"
			"ID : GR GB\n");

	for (i = 0; i < 221; i++) {
		seq_printf(s, "%03x: %02x %02x\n", i,
				gr_regs->red_coeff_mem[i].gr_coeff_value,
				gb_regs->green_coeff_mem[i].gb_coeff_value);
	}

	kfree(gr_regs);
	kfree(gb_regs);
}

#define MAKE_AVI_ISP_GRIM_SHOW(_nr) \
	static int avi_isp_grim##_nr##_show(struct seq_file *s, \
					    void *unused) \
	{ \
		avi_isp_grim_show(s, \
				avi_ctrl.nodes[AVI_ISP_CHAIN_BAYER##_nr##_NODE]); \
		return 0; \
	}

MAKE_AVI_ISP_GRIM_SHOW(0)
MAKE_AVI_ISP_GRIM_SHOW(1)

static void avi_isp_dpc_rgrim_show(struct seq_file *s,
				   struct avi_node *chain_bayer)
{
	struct avi_isp_dead_pixel_correction_regs regs;
	struct avi_isp_dead_pixel_correction_list_mem_regs *list;
	int i;
	unsigned int column_size;

	/* list is too big to be allocated on the stack */
	list = kzalloc(sizeof(*list), GFP_KERNEL);

	avi_isp_dpc_rgrim_get_registers(chain_bayer, &regs, list);
	seq_printf(s, "CFA:              %s\n", cfa_format_to_str[regs.cfa.cfa]);
	seq_printf(s, "BYPASS_LIST:      %u\n", regs.bypass.list);
	seq_printf(s, "BYPASS_AUTO:      %u\n", regs.bypass.auto_detection);
	seq_printf(s, "BYPASS_RGRIM:     %u\n", regs.bypass.rgrim);
	seq_printf(s, "THRESHOLD:        %u\n", regs.threshold.threshold);
	seq_printf(s, "RGRIM IMBALANCE1: %u\n", regs.rgrim_conf.tim_1);
	seq_printf(s, "RGRIM IMBALANCE2: %u\n", regs.rgrim_conf.tim_2);
	seq_printf(s, "RGRIM SATURATION: %u\n", regs.rgrim_conf.tsat);
	seq_printf(s, "RGRIM CONTRAST:   %u\n", regs.rgrim_conf.tcon);
	seq_printf(s, "RGRIM GAIN:       %u\n", regs.rgrim_gain.gain);

	/* Display list mem on 4 columns */
	seq_printf(s, "\n"
			"ID :  X   Y  OP | "
			"ID :  X   Y  OP | "
			"ID :  X   Y  OP | "
			"ID :  X   Y  OP | \n");
	column_size = 256 / 4;

	for (i = 0; i < column_size; i++) {
		unsigned int c1 = i;
		unsigned int c2 = c1 + column_size;
		unsigned int c3 = c2 + column_size;
		unsigned int c4 = c3 + column_size;
		const union avi_isp_dead_pixel_correction_list_mem *l1;
		const union avi_isp_dead_pixel_correction_list_mem *l2;
		const union avi_isp_dead_pixel_correction_list_mem *l3;
		const union avi_isp_dead_pixel_correction_list_mem *l4;

		l1 = &list->list_mem[c1];
		l2 = &list->list_mem[c2];
		l3 = &list->list_mem[c3];
		l4 = &list->list_mem[c4];

		seq_printf(s,
			   "%03x: %03x %03x %02x | "
			   "%03x: %03x %03x %02x | "
			   "%03x: %03x %03x %02x | "
			   "%03x: %03x %03x %02x\n",
			   c1, l1->coord_x, l1->coord_y, l1->op_code,
			   c2, l2->coord_x, l2->coord_y, l2->op_code,
			   c3, l3->coord_x, l3->coord_y, l3->op_code,
			   c4, l4->coord_x, l4->coord_y, l4->op_code);
	}

	kfree(list);
}

#define MAKE_AVI_ISP_DPC_RGRIM_SHOW(_nr) \
	static int avi_isp_dpc_rgrim##_nr##_show(struct seq_file *s, \
					    void *unused) \
	{ \
		avi_isp_dpc_rgrim_show(s, \
				avi_ctrl.nodes[AVI_ISP_CHAIN_BAYER##_nr##_NODE]); \
		return 0; \
	}

MAKE_AVI_ISP_DPC_RGRIM_SHOW(0)
MAKE_AVI_ISP_DPC_RGRIM_SHOW(1)

static void avi_isp_denoising_show(struct seq_file *s,
				   struct avi_node *chain_bayer)
{
	struct avi_isp_denoising_regs regs;

	avi_isp_denoising_get_registers(chain_bayer, &regs);
	seq_printf(s, "CFA:   %s\n", cfa_format_to_str[regs.cfa.cfa]);
	seq_printf(s, "RED:   %u %u %u %u %u %u %u %u %u %u %u %u %u %u\n",
			regs.lumocoeff_r_03_00.lumocoeff_r_00,
			regs.lumocoeff_r_03_00.lumocoeff_r_01,
			regs.lumocoeff_r_03_00.lumocoeff_r_02,
			regs.lumocoeff_r_03_00.lumocoeff_r_03,
			regs.lumocoeff_r_07_04.lumocoeff_r_04,
			regs.lumocoeff_r_07_04.lumocoeff_r_05,
			regs.lumocoeff_r_07_04.lumocoeff_r_06,
			regs.lumocoeff_r_07_04.lumocoeff_r_07,
			regs.lumocoeff_r_11_08.lumocoeff_r_08,
			regs.lumocoeff_r_11_08.lumocoeff_r_09,
			regs.lumocoeff_r_11_08.lumocoeff_r_10,
			regs.lumocoeff_r_11_08.lumocoeff_r_11,
			regs.lumocoeff_r_13_12.lumocoeff_r_12,
			regs.lumocoeff_r_13_12.lumocoeff_r_13);
	seq_printf(s, "BLUE:  %u %u %u %u %u %u %u %u %u %u %u %u %u %u\n",
			regs.lumocoeff_b_03_00.lumocoeff_b_00,
			regs.lumocoeff_b_03_00.lumocoeff_b_01,
			regs.lumocoeff_b_03_00.lumocoeff_b_02,
			regs.lumocoeff_b_03_00.lumocoeff_b_03,
			regs.lumocoeff_b_07_04.lumocoeff_b_04,
			regs.lumocoeff_b_07_04.lumocoeff_b_05,
			regs.lumocoeff_b_07_04.lumocoeff_b_06,
			regs.lumocoeff_b_07_04.lumocoeff_b_07,
			regs.lumocoeff_b_11_08.lumocoeff_b_08,
			regs.lumocoeff_b_11_08.lumocoeff_b_09,
			regs.lumocoeff_b_11_08.lumocoeff_b_10,
			regs.lumocoeff_b_11_08.lumocoeff_b_11,
			regs.lumocoeff_b_13_12.lumocoeff_b_12,
			regs.lumocoeff_b_13_12.lumocoeff_b_13);
	seq_printf(s, "GREEN:  %u %u %u %u %u %u %u %u %u %u %u %u %u %u\n",
			regs.lumocoeff_g_03_00.lumocoeff_g_00,
			regs.lumocoeff_g_03_00.lumocoeff_g_01,
			regs.lumocoeff_g_03_00.lumocoeff_g_02,
			regs.lumocoeff_g_03_00.lumocoeff_g_03,
			regs.lumocoeff_g_07_04.lumocoeff_g_04,
			regs.lumocoeff_g_07_04.lumocoeff_g_05,
			regs.lumocoeff_g_07_04.lumocoeff_g_06,
			regs.lumocoeff_g_07_04.lumocoeff_g_07,
			regs.lumocoeff_g_11_08.lumocoeff_g_08,
			regs.lumocoeff_g_11_08.lumocoeff_g_09,
			regs.lumocoeff_g_11_08.lumocoeff_g_10,
			regs.lumocoeff_g_11_08.lumocoeff_g_11,
			regs.lumocoeff_g_13_12.lumocoeff_g_12,
			regs.lumocoeff_g_13_12.lumocoeff_g_13);
}

#define MAKE_AVI_ISP_DENOISING_SHOW(_nr) \
	static int avi_isp_denoising##_nr##_show(struct seq_file *s, \
					    void *unused) \
	{ \
		avi_isp_denoising_show(s, \
				avi_ctrl.nodes[AVI_ISP_CHAIN_BAYER##_nr##_NODE]); \
		return 0; \
	}

MAKE_AVI_ISP_DENOISING_SHOW(0)
MAKE_AVI_ISP_DENOISING_SHOW(1)

static void avi_isp_lsc_show(struct seq_file *s, struct avi_node *chain_bayer)
{
	struct avi_isp_lens_shading_correction_regs regs;
	struct avi_isp_lens_shading_correction_red_coeff_mem_regs *r_regs;
	struct avi_isp_lens_shading_correction_green_coeff_mem_regs *g_regs;
	struct avi_isp_lens_shading_correction_blue_coeff_mem_regs *b_regs;
	int i;

	r_regs = kzalloc(sizeof(*r_regs), GFP_KERNEL);
	g_regs = kzalloc(sizeof(*g_regs), GFP_KERNEL);
	b_regs = kzalloc(sizeof(*b_regs), GFP_KERNEL);

	avi_isp_lsc_get_registers(chain_bayer, &regs, r_regs, g_regs, b_regs);
	seq_printf(s, "CFA:           %s\n",
			cfa_format_to_str[regs.bayer_cfa.cfa]);
	seq_printf(s, "CELL SIZE:     %ux%u\n", regs.cell_w.cell_w,
			regs.cell_h.cell_h);
	seq_printf(s, "CELL SIZE INV: %ux%u\n", regs.cell_w_inv.w_inv,
			regs.cell_h_inv.h_inv);
	seq_printf(s, "CELL IDS:      %u,%u\n", regs.cell_id_x_y.cell_id_x,
			regs.cell_id_x_y.cell_id_y);
	seq_printf(s, "OFFSETS:       %u,%u\n", regs.offset_x_y.offset_x,
			regs.offset_x_y.offset_y);
	seq_printf(s, "ALPHA:         %u\n", regs.alpha.alpha);
	seq_printf(s, "BETA:          %u\n", regs.beta.beta);
	seq_printf(s, "THRESHOLD RGB: %u %u %u\n", regs.threshold.threshold_r,
			regs.threshold.threshold_g, regs.threshold.threshold_b);
	seq_printf(s, "GAIN RGB:      %u %u %u\n", regs.gain.gain_r,
			regs.gain.gain_g, regs.gain.gain_b);

	/* display RGB coefficients */
	seq_printf(s, "\n"
			"ID : R  G  B\n");

	for (i = 0; i < 221; i++) {
		seq_printf(s, "%03x: %02x %02x %02x\n", i,
				r_regs->red_coeff_mem[i].r_coeff_value,
				g_regs->green_coeff_mem[i].g_coeff_value,
				b_regs->blue_coeff_mem[i].b_coeff_value);
	}

	kfree(r_regs);
	kfree(g_regs);
	kfree(b_regs);
}

#define MAKE_AVI_ISP_LSC_SHOW(_nr) \
	static int avi_isp_lsc##_nr##_show(struct seq_file *s, \
					    void *unused) \
	{ \
		avi_isp_lsc_show(s, \
				avi_ctrl.nodes[AVI_ISP_CHAIN_BAYER##_nr##_NODE]); \
		return 0; \
	}

MAKE_AVI_ISP_LSC_SHOW(0)
MAKE_AVI_ISP_LSC_SHOW(1)

static void avi_isp_ca_correction_show(struct seq_file *s,
				       struct avi_node *chain_bayer)
{
	struct avi_isp_chromatic_aberration_regs regs;

	avi_isp_ca_correction_get_registers(chain_bayer, &regs);
	seq_printf(s, "CFA:                       %s\n",
			cfa_format_to_str[regs.cfa.cfa]);
	seq_printf(s, "CIRCLE POSITION (SQUARED): %ux%u (%ux%u)\n",
			regs.circle_pos_x_center.x_center,
			regs.circle_pos_y_center.y_center,
			regs.circle_pos_x_squared.x_squared,
			regs.circle_pos_y_squared.y_squared);
	seq_printf(s, "LOG2 INCREMENTS:           %ux%u\n",
			regs.increments_log2.x_log2_inc,
			regs.increments_log2.y_log2_inc);
	seq_printf(s, "RADIUS SQUARED:            %u\n",
			regs.radius_squared.radius_squared);
	seq_printf(s, "DISPLACEMENT_BLUE_COEFF:   %u\n",
			regs.displacement_coeffs.displacement_blue);
	seq_printf(s, "DISPLACEMENT_RED_COEFF:    %u\n",
			regs.displacement_coeffs.displacement_red);
	seq_printf(s, "GREEN VARIATION:           %u\n",
			regs.green_variation.green_var);
}

#define MAKE_AVI_ISP_CA_CORRECTION_SHOW(_nr) \
	static int avi_isp_ca_correction##_nr##_show(struct seq_file *s, \
					    void *unused) \
	{ \
		avi_isp_ca_correction_show(s, \
				avi_ctrl.nodes[AVI_ISP_CHAIN_BAYER##_nr##_NODE]); \
		return 0; \
	}

MAKE_AVI_ISP_CA_CORRECTION_SHOW(0)
MAKE_AVI_ISP_CA_CORRECTION_SHOW(1)

static void avi_isp_color_correction_show(struct seq_file *s,
					  struct avi_node *chain_bayer)
{
	struct avi_isp_color_correction_regs regs;

	avi_isp_color_correction_get_registers(chain_bayer, &regs);
	seq_printf(s, "[ %04x %04x %04x ]   / [RY]   [ %02x ] \\   [ %02x ];"
		   " %02x <= RY <= %02x\n",
		   regs.coeff_01_00.coeff_00,
		   regs.coeff_01_00.coeff_01,
		   regs.coeff_10_02.coeff_02,
		   regs.offset_ry.offset_in,
		   regs.offset_ry.offset_out,
		   regs.clip_ry.clip_min,
		   regs.clip_ry.clip_max);
	seq_printf(s, "[ %04x %04x %04x ] x | [GU] - [ %02x ] | + [ %02x ];"
		   " %02x <= GU <= %02x\n",
		   regs.coeff_10_02.coeff_10,
		   regs.coeff_12_11.coeff_11,
		   regs.coeff_12_11.coeff_12,
		   regs.offset_gu.offset_in,
		   regs.offset_gu.offset_out,
		   regs.clip_gu.clip_min,
		   regs.clip_gu.clip_max);
	seq_printf(s, "[ %04x %04x %04x ]   \\ [BV]   [ %02x ] /   [ %02x ];"
		   " %02x <= BV <= %02x\n",
		   regs.coeff_21_20.coeff_20,
		   regs.coeff_21_20.coeff_21,
		   regs.coeff_22.coeff_22,
		   regs.offset_bv.offset_in,
		   regs.offset_bv.offset_out,
		   regs.clip_bv.clip_min,
		   regs.clip_bv.clip_max);
}

#define MAKE_AVI_ISP_COLOR_CORRECTION_SHOW(_nr) \
	static int avi_isp_color_correction##_nr##_show(struct seq_file *s, \
							void *unused) \
	{ \
		avi_isp_color_correction_show(s, \
				avi_ctrl.nodes[AVI_ISP_CHAIN_BAYER##_nr##_NODE]); \
		return 0; \
	}

MAKE_AVI_ISP_COLOR_CORRECTION_SHOW(0)
MAKE_AVI_ISP_COLOR_CORRECTION_SHOW(1)

static void avi_cfg_display(struct seq_file *s, const char *n, u32 r1, u32 r2,
		u32 r3)
{
	seq_printf(s, "%s:", n);

	if (r1 & (1 <<  0)) seq_printf(s, " CFG");
	if (r1 & (1 <<  1)) seq_printf(s, " INTER");
	if (r1 & (1 <<  2)) seq_printf(s, " FIFO00");
	if (r1 & (1 <<  3)) seq_printf(s, " FIFO01");
	if (r1 & (1 <<  4)) seq_printf(s, " FIFO02");
	if (r1 & (1 <<  5)) seq_printf(s, " FIFO03");
	if (r1 & (1 <<  6)) seq_printf(s, " FIFO04");
	if (r1 & (1 <<  7)) seq_printf(s, " FIFO05");
	if (r1 & (1 <<  8)) seq_printf(s, " FIFO06");
	if (r1 & (1 <<  9)) seq_printf(s, " FIFO07");
	if (r1 & (1 << 10)) seq_printf(s, " FIFO08");
	if (r1 & (1 << 11)) seq_printf(s, " FIFO09");
	if (r1 & (1 << 12)) seq_printf(s, " FIFO10");
	if (r1 & (1 << 13)) seq_printf(s, " FIFO11");

	if (r2 & (1 <<  0)) seq_printf(s, " CONV0");
	if (r2 & (1 <<  1)) seq_printf(s, " CONV1");
	if (r2 & (1 <<  2)) seq_printf(s, " CONV2");
	if (r2 & (1 <<  3)) seq_printf(s, " CONV3");
	if (r2 & (1 <<  4)) seq_printf(s, " BLEND0");
	if (r2 & (1 <<  5)) seq_printf(s, " BLEND1");
	if (r2 & (1 <<  6)) seq_printf(s, " LCD0");
	if (r2 & (1 <<  7)) seq_printf(s, " LCD1");
	if (r2 & (1 <<  8)) seq_printf(s, " CAM0");
	if (r2 & (1 <<  9)) seq_printf(s, " CAM1");
	if (r2 & (1 << 10)) seq_printf(s, " CAM2");
	if (r2 & (1 << 11)) seq_printf(s, " CAM3");
	if (r2 & (1 << 12)) seq_printf(s, " CAM4");
	if (r2 & (1 << 13)) seq_printf(s, " CAM5");
	if (r2 & (1 << 14)) seq_printf(s, " SCAL0");
	if (r2 & (1 << 15)) seq_printf(s, " ROT0");
	if (r2 & (1 << 16)) seq_printf(s, " SCAL1");
	if (r2 & (1 << 17)) seq_printf(s, " ROT1");
	if (r2 & (1 << 18)) seq_printf(s, " GAM0");
	if (r2 & (1 << 19)) seq_printf(s, " GAM1");
	if (r2 & (1 << 20)) seq_printf(s, " FORK0");
	if (r2 & (1 << 21)) seq_printf(s, " FORK1");
	if (r2 & (1 << 22)) seq_printf(s, " FORK2");
	if (r2 & (1 << 23)) seq_printf(s, " FORK3");
	if (r2 & (1 << 24)) seq_printf(s, " SAT0");
	if (r2 & (1 << 25)) seq_printf(s, " SAT1");
	if (r2 & (1 << 26)) seq_printf(s, " STATS_YUV0");
	if (r2 & (1 << 27)) seq_printf(s, " STATS_YUV1");
	if (r2 & (1 << 28)) seq_printf(s, " I3D_LUT0");
	if (r2 & (1 << 29)) seq_printf(s, " I3D_LUT1");

	if (r3 & (1 <<  0)) seq_printf(s, " ISP_BAYER_INTER0");
	if (r3 & (1 <<  1)) seq_printf(s, " VLFORMAT_32TO40_0");
	if (r3 & (1 <<  2)) seq_printf(s, " PEDESTAL0");
	if (r3 & (1 <<  3)) seq_printf(s, " GRIM0");
	if (r3 & (1 <<  4)) seq_printf(s, " RIP0");
	if (r3 & (1 <<  5)) seq_printf(s, " DENOISE0");
	if (r3 & (1 <<  6)) seq_printf(s, " STATS_BAYER0");
	if (r3 & (1 <<  7)) seq_printf(s, " LSC0");
	if (r3 & (1 <<  8)) seq_printf(s, " CHROMA_ABER0");
	if (r3 & (1 <<  9)) seq_printf(s, " BAYER0");
	if (r3 & (1 << 10)) seq_printf(s, " COLOR_MATRIX0");
	if (r3 & (1 << 11)) seq_printf(s, " VLFORMAT_40TO32_0");
	if (r3 & (1 << 12)) seq_printf(s, " ISP_BAYER_INTER1");
	if (r3 & (1 << 13)) seq_printf(s, " VLFORMAT_32TO40_1");
	if (r3 & (1 << 14)) seq_printf(s, " PEDESTAL1");
	if (r3 & (1 << 15)) seq_printf(s, " GRIM1");
	if (r3 & (1 << 16)) seq_printf(s, " RIP1");
	if (r3 & (1 << 17)) seq_printf(s, " DENOISE1");
	if (r3 & (1 << 18)) seq_printf(s, " STATS_BAYER1");
	if (r3 & (1 << 19)) seq_printf(s, " LSC1");
	if (r3 & (1 << 20)) seq_printf(s, " CHROMA_ABER1");
	if (r3 & (1 << 21)) seq_printf(s, " BAYER1");
	if (r3 & (1 << 22)) seq_printf(s, " COLOR_MATRIX1");
	if (r3 & (1 << 23)) seq_printf(s, " VLFORMAT_40TO32_1");
	if (r3 & (1 << 24)) seq_printf(s, " ISP_YUV_INTER0");
	if (r3 & (1 << 25)) seq_printf(s, " EE_CRF0");
	if (r3 & (1 << 26)) seq_printf(s, " I3D_LUT0");
	if (r3 & (1 << 27)) seq_printf(s, " DROP0");
	if (r3 & (1 << 28)) seq_printf(s, " ISP_YUV_INTER1");
	if (r3 & (1 << 29)) seq_printf(s, " EE_CRF1");
	if (r3 & (1 << 30)) seq_printf(s, " I3D_LUT1");
	if (r3 & (1 << 31)) seq_printf(s, " DROP1");

	seq_printf(s, "\n");
}

static int avi_cfg_show(struct seq_file *s, void *unused)
{
	struct avi_cfg_regs regs;

	avi_cfg_get_registers(&regs);

#define DISPLAY(_n) avi_cfg_display(s, #_n, regs._n ##1 ._register, \
                                            regs._n ##2 ._register, \
                                            regs.isp_ ## _n ##3 ._register)
	DISPLAY(enable);
	DISPLAY(apply);
	DISPLAY(lock);
	DISPLAY(itflg);
	DISPLAY(iten);
#undef DISPLAY
	seq_printf(s, "DMACFG: FLAG_TIMEOUT:%u FLAG_NUMBER:%u\n",
			regs.dmacfg.dma_flag_timeout,regs.dmacfg.dma_flag_number);
	seq_printf(s, "DMACFG: MAX_BURST_NUMBER:%u MAX_BANDWIDTH:%u\n",
			regs.dmacfg.dma_max_burst_number,regs.dmacfg.dma_max_bandwidth);
	return 0;
}

static void avi_debugfs_show_segment_format(struct seq_file *seq,
                                            struct avi_segment_format *fmt,
                                            int print_dma)
{
	seq_printf(seq, "%ux%u%c %s",
	           fmt->width,
	           fmt->height,
	           fmt->interlaced ? 'i' : 'p',
	           avi_debug_colorspace_to_str(fmt->colorspace));

	if (print_dma)
		seq_printf(seq, " %s %uB/line@plane0 %uB/line@plane1",
		           avi_debug_pixfmt_to_str(fmt->pix_fmt),
		           fmt->plane0.line_size,
		           fmt->plane1.line_size);
}

int avi_cap_debugfs_show(struct seq_file *s, unsigned long caps)
{
	int		 n = 0;
	const char	*str;

	while ((str = avi_debug_caps_to_str(&caps))) {
		if (n++)
			seq_printf(s, " | ");

		seq_printf(s, "%s", str);
	}

	if (n == 0)
		seq_printf(s, "<none>");

	return 0;
}

static int avi_segment_show(struct avi_segment *s, void *priv)
{
	struct seq_file *seq	  = priv;
	struct avi_node *dma_in_planar;
	int		 i;
	int		 n;

	/* Make sure the segment won't be destroyed while we're accessing it */
	mutex_lock(&s->lock);

	seq_printf(seq, "\nSegment \"%.*s\" [%s]",
	           AVI_SEGMENT_ID_LEN,
	           s->id,
	           avi_debug_activation_to_str(s->active));

	if (s->owner)
		seq_printf(seq, " owned by %s.%d:\n",
		           dev_driver_string(s->owner),
		           s->owner->id);
	else
		seq_printf(seq, " orphaned:\n");

	seq_printf(seq, "  Source segments: ");

	for (i = 0, n = 0; i < AVI_SEGMENT_MAX_SRC; i++)
		if (s->source[i]) {
			if (n++)
				seq_printf(seq, ", ");
			seq_printf(seq, "\"%.*s\" [%d]",
			           AVI_SEGMENT_ID_LEN, s->source[i]->id, i);
		}

	if (n == 0)
		seq_printf(seq, "<none>");

	seq_printf(seq, "\n  Sink segments:   ");

	for (i = 0, n = 0; i < AVI_SEGMENT_MAX_SINK; i++)
		if (s->sink[i]) {
			if (n++)
				seq_printf(seq, ", ");
			seq_printf(seq, "\"%.*s\" [%d]",
			           AVI_SEGMENT_ID_LEN, s->sink[i]->id, i);
		}

	if (n == 0)
		seq_printf(seq, "<none>");

	seq_printf(seq, "\n  Layout:          x: %u, y: %u, alpha: %d, %s",
	           s->layout.x, s->layout.y,
	           s->layout.alpha,
	           s->layout.hidden ? "hidden" : "displayed");

	seq_printf(seq, "\n  Input format:    ");
	avi_debugfs_show_segment_format(seq,
	                                &s->input_format,
	                                !!(s->caps & AVI_CAP_DMA_IN));

	seq_printf(seq, "\n  Output format:   ");
	avi_debugfs_show_segment_format(seq,
	                                &s->output_format,
	                                !!(s->caps & AVI_CAP_DMA_OUT));

	if (s->stream_period_us)
		seq_printf(seq, "\n  Stream Period:   %ldus (~%ldHz)",
			   s->stream_period_us,
			   (USEC_PER_SEC + s->stream_period_us / 2) / s->stream_period_us);

	if (s->period_us)
		seq_printf(seq, "\n  Period:          %ldus (~%ldHz)",
			   s->period_us,
			   (USEC_PER_SEC + s->period_us / 2) / s->period_us);

	if (s->period_us)
		seq_printf(seq, "\n  Dropped frames:  %lu", s->frame_dropped);


	seq_printf(seq, "\n  Capabilities:    ");
	avi_cap_debugfs_show(seq, s->caps);

	seq_printf(seq, "\n  AVI Nodes:       ");

	/* Blenders are connected in reverse order */
	for (i = 1, n = 0; i >= 0; i--) {
		if (s->blender[i]) {
			if (n++)
				seq_printf(seq, " -> ");

			seq_printf(seq, s->blender[i]->name);
		}
	}

	/* Plane1 input for planar formats is not in the main node array */
	dma_in_planar = avi_segment_get_node(s, AVI_CAP_PLANAR);
	if (dma_in_planar)
		seq_printf(seq, "%s + ", dma_in_planar->name);

	for (i = 0; i < s->nodes_nr; i++) {
		if (n++)
			seq_printf(seq, " -> ");

		seq_printf(seq, s->nodes[i]->name);
	}

	if (n == 0)
		seq_printf(seq, "<none>");

	seq_printf(seq, "\n");

	mutex_unlock(&s->lock);

	return 0;
}

static int avi_segments_show(struct seq_file *seq, void *unused)
{
	unsigned count = avi_segment_count();

	seq_printf(seq, "%d registered segment%s:\n",
	           count, count == 1 ? "" : "s");

	return avi_segment_foreach(&avi_segment_show, seq);
}

static int avi_access_show(struct seq_file *seq, void *unused)
{
	return avi_log_display(seq);
}

static int avi_debugfs_open(struct inode *inode, struct file *file)
{
	/* inode->i_private contains the "show" callback */
	return single_open(file, inode->i_private, NULL);
}

static const struct file_operations avi_debugfs_fops = {
	.open		= avi_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init avi_debugfs_init(void)
{
	BUG_ON(!IS_ERR_OR_NULL(avi_debugfs_root));

	avi_debugfs_root = debugfs_create_dir("avi", NULL);

	if (IS_ERR_OR_NULL(avi_debugfs_root))
		return avi_debugfs_root ? PTR_ERR(avi_debugfs_root) : -ENOMEM;

	avi_isp0_debugfs_root = debugfs_create_dir("isp0", avi_debugfs_root);

	if (IS_ERR_OR_NULL(avi_isp0_debugfs_root))
		return avi_isp0_debugfs_root ? PTR_ERR(avi_isp0_debugfs_root) :
					       -ENOMEM;

	avi_isp1_debugfs_root = debugfs_create_dir("isp1", avi_debugfs_root);

	if (IS_ERR_OR_NULL(avi_isp1_debugfs_root))
		return avi_isp1_debugfs_root ? PTR_ERR(avi_isp1_debugfs_root) :
					       -ENOMEM;

#define AVI_DEBUGFS_REGISTER(_name, _cback)			\
	IS_ERR_OR_NULL(debugfs_create_file((_name), S_IRUGO,	\
					   avi_debugfs_root,	\
					   (_cback),		\
					   &avi_debugfs_fops))

#define AVI_ISP_DEBUGFS_REGISTER(_name, _nr, _cback)			\
	IS_ERR_OR_NULL(debugfs_create_file((_name), S_IRUGO,		\
					   avi_isp##_nr##_debugfs_root,	\
					   (_cback),			\
					   &avi_debugfs_fops))

	if (AVI_DEBUGFS_REGISTER("inter",    &avi_inter_show)	        ||
	    AVI_DEBUGFS_REGISTER("fifo00",   &avi_fifo00_show)	        ||
	    AVI_DEBUGFS_REGISTER("fifo01",   &avi_fifo01_show)	        ||
	    AVI_DEBUGFS_REGISTER("fifo02",   &avi_fifo02_show)	        ||
	    AVI_DEBUGFS_REGISTER("fifo03",   &avi_fifo03_show)	        ||
	    AVI_DEBUGFS_REGISTER("fifo04",   &avi_fifo04_show)	        ||
	    AVI_DEBUGFS_REGISTER("fifo05",   &avi_fifo05_show)	        ||
	    AVI_DEBUGFS_REGISTER("fifo06",   &avi_fifo06_show)	        ||
	    AVI_DEBUGFS_REGISTER("fifo07",   &avi_fifo07_show)	        ||
	    AVI_DEBUGFS_REGISTER("fifo08",   &avi_fifo08_show)	        ||
	    AVI_DEBUGFS_REGISTER("fifo09",   &avi_fifo09_show)	        ||
	    AVI_DEBUGFS_REGISTER("fifo10",   &avi_fifo10_show)	        ||
	    AVI_DEBUGFS_REGISTER("fifo11",   &avi_fifo11_show)	        ||
	    AVI_DEBUGFS_REGISTER("conv0",    &avi_conv0_show)	        ||
	    AVI_DEBUGFS_REGISTER("conv1",    &avi_conv1_show)	        ||
	    AVI_DEBUGFS_REGISTER("conv2",    &avi_conv2_show)	        ||
	    AVI_DEBUGFS_REGISTER("conv3",    &avi_conv3_show)	        ||
	    AVI_DEBUGFS_REGISTER("cam0",     &avi_cam0_show)	        ||
	    AVI_DEBUGFS_REGISTER("cam1",     &avi_cam1_show)	        ||
	    AVI_DEBUGFS_REGISTER("cam2",     &avi_cam2_show)	        ||
	    AVI_DEBUGFS_REGISTER("cam3",     &avi_cam3_show)	        ||
	    AVI_DEBUGFS_REGISTER("cam4",     &avi_cam4_show)	        ||
	    AVI_DEBUGFS_REGISTER("cam5",     &avi_cam5_show)	        ||
	    AVI_DEBUGFS_REGISTER("gam0",     &avi_gam0_show)	        ||
	    AVI_DEBUGFS_REGISTER("gam1",     &avi_gam1_show)	        ||
	    AVI_DEBUGFS_REGISTER("lcd0",     &avi_lcd0_show)	        ||
	    AVI_DEBUGFS_REGISTER("lcd1",     &avi_lcd1_show)	        ||
	    AVI_DEBUGFS_REGISTER("blend0",   &avi_blend0_show)	        ||
	    AVI_DEBUGFS_REGISTER("blend1",   &avi_blend1_show)	        ||
	    AVI_DEBUGFS_REGISTER("scal0",    &avi_scal0_show)	        ||
	    AVI_DEBUGFS_REGISTER("scal1",    &avi_scal1_show)	        ||
	    AVI_DEBUGFS_REGISTER("rot0",     &avi_rot0_show)	        ||
	    AVI_DEBUGFS_REGISTER("rot1",     &avi_rot1_show)	        ||
	    AVI_DEBUGFS_REGISTER("cfg",      &avi_cfg_show)	        ||
	    AVI_DEBUGFS_REGISTER("segments", &avi_segments_show)        ||
	    AVI_DEBUGFS_REGISTER("access",   &avi_access_show)          ||

	    AVI_ISP_DEBUGFS_REGISTER("inter", 0,
				     &avi_isp_chain_bayer_inter0_show)  ||
	    AVI_ISP_DEBUGFS_REGISTER("inter", 1,
				     &avi_isp_chain_bayer_inter1_show)  ||
	    AVI_ISP_DEBUGFS_REGISTER("bayer", 0,
				     &avi_isp_bayer0_show)              ||
	    AVI_ISP_DEBUGFS_REGISTER("bayer", 1,
				     &avi_isp_bayer1_show)              ||
	    AVI_ISP_DEBUGFS_REGISTER("stats_bayer", 0,
				     &avi_stats_bayer0_show)            ||
	    AVI_ISP_DEBUGFS_REGISTER("stats_bayer", 1,
				     &avi_stats_bayer1_show)            ||
	    AVI_ISP_DEBUGFS_REGISTER("pedestal", 0,
				     &avi_isp_pedestal0_show)           ||
	    AVI_ISP_DEBUGFS_REGISTER("pedestal", 1,
				     &avi_isp_pedestal1_show)           ||
	    AVI_ISP_DEBUGFS_REGISTER("grim", 0, &avi_isp_grim0_show)    ||
	    AVI_ISP_DEBUGFS_REGISTER("grim", 1, &avi_isp_grim1_show)    ||
	    AVI_ISP_DEBUGFS_REGISTER("dpc_rgrim", 0,
				     &avi_isp_dpc_rgrim0_show)          ||
	    AVI_ISP_DEBUGFS_REGISTER("dpc_rgrim", 1,
				     &avi_isp_dpc_rgrim1_show)          ||
	    AVI_ISP_DEBUGFS_REGISTER("denoising", 0,
				     &avi_isp_denoising0_show)          ||
	    AVI_ISP_DEBUGFS_REGISTER("denoising", 1,
				     &avi_isp_denoising1_show)          ||
	    AVI_ISP_DEBUGFS_REGISTER("lsc", 0, &avi_isp_lsc0_show)      ||
	    AVI_ISP_DEBUGFS_REGISTER("lsc", 1, &avi_isp_lsc1_show)      ||
	    AVI_ISP_DEBUGFS_REGISTER("cac", 0,
				     &avi_isp_ca_correction0_show)      ||
	    AVI_ISP_DEBUGFS_REGISTER("cac", 1,
				     &avi_isp_ca_correction1_show)      ||
	    AVI_ISP_DEBUGFS_REGISTER("color_correction", 0,
				     &avi_isp_color_correction0_show)   ||
	    AVI_ISP_DEBUGFS_REGISTER("color_correction", 1,
				     &avi_isp_color_correction1_show)) {
		debugfs_remove_recursive(avi_debugfs_root);
		avi_debugfs_root = NULL;
		return -ENOMEM;
	}

	printk(KERN_INFO "AVI DebugFS registered\n");

	return 0;
}
module_init(avi_debugfs_init);

static void __exit avi_debugfs_exit(void)
{
	BUG_ON(IS_ERR_OR_NULL(avi_debugfs_root));

	debugfs_remove_recursive(avi_debugfs_root);
	avi_debugfs_root = NULL;
}
module_exit(avi_debugfs_exit);

MODULE_DESCRIPTION("AVI debugfs module");
MODULE_AUTHOR("Lionel Flandrin <lionel.flandrin@parrot.com>");
MODULE_LICENSE("GPL");
