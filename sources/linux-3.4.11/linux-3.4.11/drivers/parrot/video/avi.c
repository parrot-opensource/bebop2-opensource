/*
 *  linux/drivers/parrot/video/avi.c
 *
 *  Copyright (C) 2011 Parrot S.A.
 *
 * @author  Gregor Boirie <gregor.boirie@parrot.com>
 * @author  Lionel Flandrin <lionel.flandrin@parrot.com>
 * @date    18-Feb-2011
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

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <mach/irqs.h>
#include <linux/stat.h>
#include <linux/module.h>
#include <linux/mutex.h>

#include "avi_compat.h"
#include "reg_avi.h"
#include "avi_logger.h"

/* If the AVI must share (some of) its IRQs */
/*#define AVI_IRQ_SHARED */

static struct avi_node avi_cfg = {
	.type       = AVI_CFG_NODE_TYPE,
	.src_id     = AVI_SRCSINK_NONE,
	.sink_id    = AVI_SRCSINK_NONE,
	.base_off   = AVI_CFG,
	.cfg_off    = 0,
	.cfg_mask   = 1UL << 0,
	.name       = "CFG",
	.irq_offset = 0,
	.node_id = AVI_CFG_NODE
};

static struct avi_node avi_inter = {
	.type       = AVI_INTER_NODE_TYPE,
	.src_id     = AVI_SRCSINK_NONE,
	.sink_id    = AVI_SRCSINK_NONE,
	.base_off   = AVI_INTER,
	.cfg_off    = 0,
	.cfg_mask   = 1UL << 1,
	.name       = "INTER",
	.irq_offset = 1,
	.node_id = AVI_INTER_NODE
};

/* Extend avi_node to add fifo capabilities */
struct avi_fifo
{
	struct avi_node avi_node;
	int             capabilities;
};

static __maybe_unused struct avi_fifo *avi_get_fifo(struct avi_node *n)
{
	BUG_ON(!(n->type == AVI_FIFO_NODE_TYPE));
	return container_of(n, struct avi_fifo, avi_node);
}

#define AVI_DEFINE_FIFO(_no)                                            \
	struct avi_node avi_fifo ## _no = {				\
		.type       = AVI_FIFO_NODE_TYPE,			\
		.src_id     = _no + 1,					\
		.sink_id    = _no,					\
		.base_off   = AVI_FIFO ## _no,				\
		.cfg_off    = 0,					\
		.cfg_mask   = 1UL << ((_no) + 2),			\
		.name = (_no < 10 ? ("FIFO0" #_no) : ("FIFO" #_no)),	\
		.irq_offset = 2 + (_no), 				\
		.node_id = AVI_FIFO00_NODE + _no			\
	}

static AVI_DEFINE_FIFO(0);
static AVI_DEFINE_FIFO(1);
static AVI_DEFINE_FIFO(2);
static AVI_DEFINE_FIFO(3);
static AVI_DEFINE_FIFO(4);
static AVI_DEFINE_FIFO(5);
static AVI_DEFINE_FIFO(6);
static AVI_DEFINE_FIFO(7);
static AVI_DEFINE_FIFO(8);
static AVI_DEFINE_FIFO(9);
static AVI_DEFINE_FIFO(10);
static AVI_DEFINE_FIFO(11);

#define AVI_DEFINE_CONV(_no)			    \
	struct avi_node avi_conv ## _no = {	    \
		.type       = AVI_CONV_NODE_TYPE,   \
		.src_id     = _no + 0xd,            \
		.sink_id    = _no + 0xc,            \
		.base_off   = AVI_CONV ## _no,	    \
		.cfg_off    = 1,                    \
		.cfg_mask   = 1UL << (_no),         \
		.name       = "CONV" #_no,	    \
		.irq_offset = 14 + (_no),	    \
		.node_id = AVI_CONV0_NODE + _no     \
	}

static AVI_DEFINE_CONV(0);
static AVI_DEFINE_CONV(1);
static AVI_DEFINE_CONV(2);
static AVI_DEFINE_CONV(3);

/* Each blender has 4 sinks. .sink_id takes the value of the first one, the
 * others follow.*/
#define AVI_DEFINE_BLEND(_no)					\
	struct avi_node avi_blend ## _no = {			\
		.type       = AVI_BLEND_NODE_TYPE,		\
		.src_id     = _no + 0x11,			\
		.sink_id    = ((_no) * 4) + 0x10,		\
		.base_off   = AVI_BLEND ## _no,			\
		.cfg_off    = 1,				\
		.cfg_mask   = 1UL << ((_no) + 4),		\
		.name       = "BLEND" #_no,			\
		.irq_offset = 18 + (_no),			\
		.node_id = AVI_BLEND0_NODE + _no		\
	}

static AVI_DEFINE_BLEND(0);
static AVI_DEFINE_BLEND(1);

/* Extend avi_node to add lcd pixclock info */
struct avi_lcd
{
	struct avi_node	 avi_node;
	struct clk	*pixclock;
	int		 clk_enabled;
};

/* The LCD nodes are not present in the interconnect of MPW1. They are hardwired
 * to their respective GAM node. In this case the sink_id points to a
 * non-existing register, but we can safely write to it (it's a NOP). So this
 * should be backward compatible. */
#define AVI_DEFINE_LCD(_no)						\
	struct avi_lcd avi_lcd ## _no = {				\
		.avi_node = {						\
			.type       = AVI_LCD_NODE_TYPE,		\
			.src_id     = AVI_SRCSINK_NONE,			\
			.sink_id    = _no + 0x20,			\
			.base_off   = AVI_LCD ## _no,			\
			.cfg_off    = 1,				\
			.cfg_mask   = 1UL << ((_no) + 6),		\
			.name       = "LCD" #_no,			\
			.irq_offset = 20 + (_no),			\
			.node_id = AVI_LCD0_NODE + _no			\
		},							\
	}

static AVI_DEFINE_LCD(0);
static AVI_DEFINE_LCD(1);

#define AVI_DEFINE_CAM(_no)			    \
	struct avi_node avi_cam ## _no = {	    \
		.type       = AVI_CAM_NODE_TYPE,    \
		.src_id     = _no + 0x13,           \
		.sink_id    = AVI_SRCSINK_NONE,     \
		.base_off   = AVI_CAM ## _no,	    \
		.cfg_off    = 1,                    \
		.cfg_mask   = 1UL << ((_no) + 8),   \
		.name       = "CAM" #_no,	    \
		.irq_offset = 22 + (_no),	    \
		.node_id = AVI_CAM0_NODE + _no	    \
	}

static AVI_DEFINE_CAM(0);
static AVI_DEFINE_CAM(1);
static AVI_DEFINE_CAM(2);
static AVI_DEFINE_CAM(3);
static AVI_DEFINE_CAM(4);
static AVI_DEFINE_CAM(5);

#define AVI_DEFINE_SCAL(_scalrot, _input)				  \
	struct avi_node avi_scal ## _scalrot ## _input = {		  \
		.type       = AVI_SCALROT_NODE_TYPE,			  \
		.src_id     = (_scalrot * 2) + 0x19,			  \
		.sink_id    = (_scalrot * 3) + _input + 0x1a,		  \
		.base_off   = AVI_SCAL ## _scalrot,			  \
		.cfg_off    = 1,					  \
		.cfg_mask   = 1UL << (((_scalrot) * 2) + 14),	  	  \
		.irq_offset = 28 + (_scalrot),				  \
		.name       = "SCAL" #_scalrot "[" #_input "]",		  \
		.node_id = (_scalrot==0)?AVI_SCAL00_NODE:AVI_SCAL10_NODE  \
				+ _input				  \
	}

static AVI_DEFINE_SCAL(0, 0);
static AVI_DEFINE_SCAL(0, 1);
static AVI_DEFINE_SCAL(1, 0);
static AVI_DEFINE_SCAL(1, 1);

#define AVI_DEFINE_ROT(_scalrot)					\
	struct avi_node avi_rot ## _scalrot = {				\
		.type       = AVI_SCALROT_NODE_TYPE,			\
		.src_id     = (_scalrot * 2) + 0x1a,			\
		.sink_id    = (_scalrot * 3) + 0x1c,			\
		.base_off   = AVI_ROT ## _scalrot,			\
		.cfg_off    = 1,					\
		.cfg_mask   = 1UL << (((_scalrot) * 2) + 15),		\
		.name       = "ROT" #_scalrot,				\
		.irq_offset = 28 + (_scalrot),				\
		.node_id = (_scalrot==0)?AVI_ROT0_NODE:AVI_ROT1_NODE	\
	}

static AVI_DEFINE_ROT(0);
static AVI_DEFINE_ROT(1);

/* GAMs don't have a src_id in MPW1, see comment on AVI_DEFINE_LCD above. */
#define AVI_DEFINE_GAM(_no)			    \
	struct avi_node avi_gam ## _no = {	    \
		.type       = AVI_GAM_NODE_TYPE,    \
		.src_id     = _no + 0x1d,	    \
		.sink_id    = _no + 0x18,           \
		.base_off   = AVI_GAM ## _no,	    \
		.cfg_off    = 1,                    \
		.cfg_mask   = 1UL << ((_no) + 18),  \
		.name       = "GAM" #_no,	    \
		.irq_offset = 30 + (_no),	    \
		.node_id = AVI_GAM0_NODE + _no	    \
	}

static AVI_DEFINE_GAM(0);
static AVI_DEFINE_GAM(1);

#define AVI_DEFINE_ISP_CHAIN_BAYER(_no)                       \
	struct avi_node avi_isp_chain_bayer ## _no = {        \
		.type       = AVI_ISP_CHAIN_BAYER_NODE_TYPE,  \
		.src_id     = _no + 0x2d,                     \
		.sink_id    = _no + 0x2a,                     \
		.base_off   = AVI_ISP_CHAIN_BAYER ## _no,     \
		.cfg_off    = 3,                              \
		.cfg_mask   = 0xFFFUL << ((_no) * 12),        \
		.name       = "ISP_CHAIN_BAYER" #_no,         \
		.irq_offset = 42 + 12 * (_no),                \
		.node_id    = AVI_ISP_CHAIN_BAYER0_NODE + _no \
	}

static AVI_DEFINE_ISP_CHAIN_BAYER(0);
static AVI_DEFINE_ISP_CHAIN_BAYER(1);

#define AVI_DEFINE_STATS_BAYER(_no)				\
	struct avi_node avi_stats_bayer ##_no = {		\
		.type       = AVI_STATS_BAYER_NODE_TYPE,	\
		.src_id     = _no + 0x2b,			\
		 /* Always implicitely connected to		\
		    the respective CHAIN_BAYER */		\
		.sink_id    = AVI_SRCSINK_NONE,			\
		.base_off   = 0,				\
		.cfg_off    = 3,				\
		.cfg_mask   = 0x40UL << ((_no) * 12),		\
		.name       = "ISP_STATS_BAYER" #_no,		\
		.irq_offset = -1,				\
		.node_id    = AVI_STATS_BAYER0_NODE + _no,	\
	}

static AVI_DEFINE_STATS_BAYER(0);
static AVI_DEFINE_STATS_BAYER(1);

#define AVI_DEFINE_STATS_YUV(_no)                       \
	struct avi_node avi_stats_yuv ## _no = {        \
		.type       = AVI_STATS_YUV_NODE_TYPE,  \
		.src_id     = _no + 0x29,               \
		.sink_id    = _no + 0x28,               \
		.base_off   = AVI_STATS_YUV ## _no,     \
		.cfg_off    = 1,                        \
		.cfg_mask   = 1UL << ((_no) + 26),      \
		.name       = "STATS_YUV" #_no,         \
		.irq_offset = 40 + (_no),               \
		.node_id    = AVI_STATS_YUV0_NODE + _no \
	}

static AVI_DEFINE_STATS_YUV(0);
static AVI_DEFINE_STATS_YUV(1);

#define AVI_DEFINE_ISP_CHAIN_YUV(_no)                       \
	struct avi_node avi_isp_chain_yuv ## _no = {        \
		.type       = AVI_ISP_CHAIN_YUV_NODE_TYPE,  \
		.src_id     = _no + 0x2f,                   \
		.sink_id    = _no + 0x2c,                   \
		.base_off   = AVI_ISP_CHAIN_YUV ## _no,     \
		.cfg_off    = 3,                            \
		.cfg_mask   = 0xFUL << ((_no) * 4 + 24),    \
		.name       = "ISP_CHAIN_YUV" #_no,         \
		.irq_offset = 66 + 4 * (_no),               \
		.node_id    = AVI_ISP_CHAIN_YUV0_NODE + _no \
	}

static AVI_DEFINE_ISP_CHAIN_YUV(0);
static AVI_DEFINE_ISP_CHAIN_YUV(1);


struct avi_controller avi_ctrl = {
	.cfg_lck    = __SPIN_LOCK_UNLOCKED(avi_ctrl.cfg_lck),
	.blend_lck  = __MUTEX_INITIALIZER(avi_ctrl.blend_lck),
	.node_lck    = __SPIN_LOCK_UNLOCKED(avi_ctrl.node_lck),
	.nodes      = {
		[AVI_FIFO00_NODE]           = &avi_fifo0,
		[AVI_FIFO01_NODE]           = &avi_fifo1,
		[AVI_FIFO02_NODE]           = &avi_fifo2,
		[AVI_FIFO03_NODE]           = &avi_fifo3,
		[AVI_FIFO04_NODE]           = &avi_fifo4,
		[AVI_FIFO05_NODE]           = &avi_fifo5,
		[AVI_FIFO06_NODE]           = &avi_fifo6,
		[AVI_FIFO07_NODE]           = &avi_fifo7,
		[AVI_FIFO08_NODE]           = &avi_fifo8,
		[AVI_FIFO09_NODE]           = &avi_fifo9,
		[AVI_FIFO10_NODE]           = &avi_fifo10,
		[AVI_FIFO11_NODE]           = &avi_fifo11,
		[AVI_CONV0_NODE]            = &avi_conv0,
		[AVI_CONV1_NODE]            = &avi_conv1,
		[AVI_CONV2_NODE]            = &avi_conv2,
		[AVI_CONV3_NODE]            = &avi_conv3,
		[AVI_BLEND0_NODE]           = &avi_blend0,
		[AVI_BLEND1_NODE]           = &avi_blend1,
		[AVI_LCD0_NODE]             = &avi_lcd0.avi_node,
		[AVI_LCD1_NODE]             = &avi_lcd1.avi_node,
		[AVI_CAM0_NODE]             = &avi_cam0,
		[AVI_CAM1_NODE]             = &avi_cam1,
		[AVI_CAM2_NODE]             = &avi_cam2,
		[AVI_CAM3_NODE]             = &avi_cam3,
		[AVI_CAM4_NODE]             = &avi_cam4,
		[AVI_CAM5_NODE]             = &avi_cam5,
		[AVI_SCAL00_NODE]           = &avi_scal00,
		[AVI_SCAL01_NODE]           = &avi_scal01,
		[AVI_ROT0_NODE]             = &avi_rot0,
		[AVI_SCAL10_NODE]           = &avi_scal10,
		[AVI_SCAL11_NODE]           = &avi_scal11,
		[AVI_ROT1_NODE]             = &avi_rot1,
		[AVI_GAM0_NODE]             = &avi_gam0,
		[AVI_GAM1_NODE]             = &avi_gam1,
		[AVI_ISP_CHAIN_BAYER0_NODE] = &avi_isp_chain_bayer0,
		[AVI_ISP_CHAIN_BAYER1_NODE] = &avi_isp_chain_bayer1,
		[AVI_STATS_BAYER0_NODE]     = &avi_stats_bayer0,
		[AVI_STATS_BAYER1_NODE]     = &avi_stats_bayer1,
		[AVI_STATS_YUV0_NODE]       = &avi_stats_yuv0,
		[AVI_STATS_YUV1_NODE]       = &avi_stats_yuv1,
		[AVI_ISP_CHAIN_YUV0_NODE]   = &avi_isp_chain_yuv0,
		[AVI_ISP_CHAIN_YUV1_NODE]   = &avi_isp_chain_yuv1,
		[AVI_INTER_NODE]            = &avi_inter,
		[AVI_CFG_NODE]              = &avi_cfg,
	}
};
EXPORT_SYMBOL(avi_ctrl);

/**
 * Copy reg_base to addr one 32bits at a time.
 */
void memcpy_from_registers(void *reg_base, unsigned long addr, size_t s)
{
	u32      *reg = reg_base;
	unsigned i;

	BUG_ON(s % sizeof(u32) != 0);
	s /= sizeof(u32); /* we read one register at a time */

	for (i = 0; i < s; i++)
		reg[i] = AVI_READ(addr + i * sizeof(u32));
	rmb();
}
EXPORT_SYMBOL(memcpy_from_registers);

void avi_cfg_get_registers(struct avi_cfg_regs *regs)
{
	memcpy_from_registers(regs, avi_node_base(&avi_cfg), sizeof(*regs));
}
EXPORT_SYMBOL(avi_cfg_get_registers);

void avi_enable_node(struct avi_node const* node)
{
	unsigned long flags;

	if (node->type != AVI_BLENDIN_NODE_TYPE) {
		unsigned long addr;

		/* ISP modules do not follow the same register map scheme. */
		if (node->cfg_off == 3)
			addr = avi_ctrl.base + AVI_CFG_ISP_ENABLE3;
		else
			addr = avi_ctrl.base + AVI_CFG_ENABLE1 +
			       (node->cfg_off * sizeof(u32));

		spin_lock_irqsave(&avi_ctrl.node_lck, flags);
		AVI_WRITE(node->cfg_mask | AVI_READ(addr), addr);
		spin_unlock_irqrestore(&avi_ctrl.node_lck, flags);
	}
}
EXPORT_SYMBOL(avi_enable_node);

void avi_disable_node(struct avi_node const* node)
{
	unsigned long flags;

	if (node->type != AVI_BLENDIN_NODE_TYPE) {
		unsigned long addr;

		/* ISP modules do not follow the same register map scheme. */
		if (node->cfg_off == 3)
			addr = avi_ctrl.base + AVI_CFG_ISP_ENABLE3;
		else
			addr = avi_ctrl.base + AVI_CFG_ENABLE1 +
			       (node->cfg_off * sizeof(u32));

		spin_lock_irqsave(&avi_ctrl.node_lck, flags);
		AVI_WRITE(AVI_READ(addr) & ~node->cfg_mask, addr);
		spin_unlock_irqrestore(&avi_ctrl.node_lck, flags);
	}
}
EXPORT_SYMBOL(avi_disable_node);

void avi_lock_node(struct avi_node const* node)
{
	unsigned long flags;

	/*
	 * Some nodes (aka AVI_BLENDIN_NODE_TYPE) are virtual nodes: they have
	 * no assigned clock.
	 */
	if (node->type != AVI_BLENDIN_NODE_TYPE) {
		unsigned long addr;

		/* ISP modules do not follow the same register map scheme. */
		if (node->cfg_off == 3)
			addr = avi_ctrl.base + AVI_CFG_ISP_LOCK3;
		else
			addr = avi_ctrl.base + AVI_CFG_LOCK1 +
			       (node->cfg_off * sizeof(u32));

		spin_lock_irqsave(&avi_ctrl.node_lck, flags);
		AVI_WRITE(node->cfg_mask | AVI_READ(addr), addr);
		spin_unlock_irqrestore(&avi_ctrl.node_lck, flags);
	}
}
EXPORT_SYMBOL(avi_lock_node);

void avi_unlock_node(struct avi_node const* node)
{
	unsigned long flags;

	if (node->type != AVI_BLENDIN_NODE_TYPE) {
		unsigned long addr;

		/* ISP modules do not follow the same register map scheme. */
		if (node->cfg_off == 3)
			addr = avi_ctrl.base + AVI_CFG_ISP_LOCK3;
		else
			addr = avi_ctrl.base + AVI_CFG_LOCK1 +
			       (node->cfg_off * sizeof(u32));

		spin_lock_irqsave(&avi_ctrl.node_lck, flags);
		AVI_WRITE(AVI_READ(addr) & ~node->cfg_mask, addr);
		spin_unlock_irqrestore(&avi_ctrl.node_lck, flags);
	}
}
EXPORT_SYMBOL(avi_unlock_node);

/**
 * Force apply the node's shadow registers configuration.
 */
void avi_apply_node(struct avi_node const* node)
{
	unsigned long flags;

	if (node->type != AVI_BLENDIN_NODE_TYPE) {
		unsigned long addr;

		/* ISP modules do not follow the same register map scheme. */
		if (node->cfg_off == 3)
			addr = avi_ctrl.base + AVI_CFG_ISP_APPLY3;
		else
			addr = avi_ctrl.base + AVI_CFG_APPLY1 +
			       (node->cfg_off * sizeof(u32));

		spin_lock_irqsave(&avi_ctrl.node_lck, flags);
		AVI_WRITE(node->cfg_mask, addr);
		spin_unlock_irqrestore(&avi_ctrl.node_lck, flags);
	}
}
EXPORT_SYMBOL(avi_apply_node);

/**************
 * FIFO config
 **************/

static inline void memcpy_to_registers(unsigned long addr,
				       const void *reg_base,
				       size_t s)
{
	const u32 *reg = reg_base;
	unsigned  i;

	BUG_ON(s % sizeof(u32) != 0);
	s /= sizeof(u32); /* we write one register at a time */

	for (i = 0; i < s; i++)
		AVI_WRITE(reg[i], addr + i * sizeof(u32));
	wmb();
}

void avi_fifo_get_registers(struct avi_node const *fifo,
			    struct avi_fifo_registers *regs)
{
	memcpy_from_registers(regs, avi_node_base(fifo), sizeof(*regs));
}
EXPORT_SYMBOL(avi_fifo_get_registers);

void avi_fifo_set_registers(struct avi_node const *fifo,
			    struct avi_fifo_registers const *regs)
{
	memcpy_to_registers(avi_node_base(fifo),
			    regs,
			    sizeof(*regs));
}
EXPORT_SYMBOL(avi_fifo_set_registers);

void avi_fifo_set_oneshot(struct avi_node const *fifo, bool oneshot)
{
	unsigned long const addr = avi_node_base(fifo) + AVI_FIFO_CFG;
	union avi_fifo_cfg cfg;

	cfg._register = AVI_READ(addr);
	cfg.single    = oneshot;
	AVI_WRITE(cfg._register, addr);
}
EXPORT_SYMBOL(avi_fifo_set_oneshot);

void avi_fifo_set_dmasa(struct avi_node const *fifo, u32 dmasa0, u32 dmasa1)
{
	unsigned long const base = avi_node_base(fifo);

	AVI_WRITE(dmasa0, base + AVI_FIFO_DMASA0);
	AVI_WRITE(dmasa1, base + AVI_FIFO_DMASA1);
}
EXPORT_SYMBOL(avi_fifo_set_dmasa);

void avi_fifo_set_dmalstr(struct avi_node const *fifo,
                          u32 dmalstr0,
                          u32 dmalstr1)
{
	unsigned long const base = avi_node_base(fifo);

	AVI_WRITE(dmalstr0, base + AVI_FIFO_DMALSTR0);
	AVI_WRITE(dmalstr1, base + AVI_FIFO_DMALSTR1);
}
EXPORT_SYMBOL(avi_fifo_set_dmalstr);

union avi_fifo_status avi_fifo_get_status(struct avi_node const *fifo)
{
	unsigned long const	addr = avi_node_base(fifo) + AVI_FIFO_STATUS;
	union avi_fifo_status	fifo_status;

	fifo_status._register = AVI_READ(addr);
	return fifo_status;
}
EXPORT_SYMBOL(avi_fifo_get_status);

static void avi_fifo_set_force_clear(struct avi_node const		*fifo,
				     enum avi_fifo_vlin_force_clear	 fc)
{
	unsigned long const addr = avi_node_base(fifo) + AVI_FIFO_VLIN;
	union avi_fifo_vlin vlin;

	vlin._register   = AVI_READ(addr);
	vlin.force_clear = fc;
	AVI_WRITE(vlin._register, addr);
}

void avi_fifo_force_clear(struct avi_node const *node)
{
	/* Force a clear sequence on the fifo. We force the clear high, then we
	 * return to the normal clear behaviour. */
	avi_fifo_set_force_clear(node, AVI_FIFO_VLIN_FORCE_CLEAR_ACTIVE);
	wmb();
	avi_fifo_set_force_clear(node, AVI_FIFO_VLIN_FORCE_CLEAR_NORMAL);
}
EXPORT_SYMBOL(avi_fifo_force_clear);

/***************************
 * Blenders low-level setup
 ***************************/

static inline int avi_blendin_id(const struct avi_node *blendin)
{
	return blendin->sink_id & 3;
}

static inline void avi_blendin_set_id(struct avi_node *blendin, int id)
{
	blendin->sink_id &= ~3;
	blendin->sink_id += id & 3;
}

/**
 * avi_blendin_2cfg() - Build blending configuration.
 * @config: pointer to configuration to apply
 * @blendin: pointer to AVI virtual blendin node
 * @xoffset: x coordinate of upper left corner of frame to blend
 * @yoffset: y coordinate of upper left corner of frame to blend
 * @alpha: alpha component used to blend plane with, or enable OSD mode if < 0
 */
void avi_blendin_2cfg(struct avi_blendin_cfg* config,
		      struct avi_node const* blendin,
		      u16 xoffset,
		      u16 yoffset,
		      int alpha)
{
	config->offset = (yoffset << 16) | xoffset;

	if (alpha < 0)
		config->alpha = 0x100; /* set AMODE to 1 (OSD mode) */
	else
		config->alpha = alpha & 0xff;
}
EXPORT_SYMBOL(avi_blendin_2cfg);

/**
 * avi_setup_blendin_cfg() - Apply previously built blending configuration.
 * @blendin: pointer to AVI virtual blendin node
 * @config: pointer to configuration to apply
 */
void avi_set_blendin_cfg(struct avi_node const* blendin,
                         struct avi_blendin_cfg const* config)
{
	unsigned long const	base = avi_node_base(blendin);
	int			id   = avi_blendin_id(blendin);

	AVI_WRITE(config->offset,
		  base + AVI_BLEND_OFFSET0 + (id * sizeof(u32) * 2));
	AVI_WRITE(config->alpha,
		  base + AVI_BLEND_ALPHA0 + (id * sizeof(u32) * 2));
}
EXPORT_SYMBOL(avi_set_blendin_cfg);

/* config->id must be the ID of the blendin */
void avi_get_blendin_cfg(struct avi_node const* blendin,
			 struct avi_blendin_cfg* config)
{
	unsigned long const	base = avi_node_base(blendin);
	int			id   = avi_blendin_id(blendin);

	config->offset = AVI_READ(base + AVI_BLEND_OFFSET0
				  + (id * sizeof(u32) * 2));
	config->alpha  = AVI_READ(base + AVI_BLEND_ALPHA0
				  + (id * sizeof(u32) * 2));
}
EXPORT_SYMBOL(avi_get_blendin_cfg);

void avi_setup_blender_input(struct avi_node const* blender,
                             unsigned input,
                             unsigned x,
                             unsigned y,
                             int alpha)
{
	unsigned long	base;
	u32		offset;

	base = avi_node_base(blender) + (input * sizeof(u32) * 2);

	if (alpha < 0 || alpha == 0x100)
		/* OSD mode */
		alpha = 0x100;
	else
		alpha &= 0xff;

	x &= 0xffff;
	y &= 0xffff;

	offset = (y << 16) | x;

	AVI_WRITE(offset, base + AVI_BLEND_OFFSET0);
	AVI_WRITE(alpha, base + AVI_BLEND_ALPHA0);
}
EXPORT_SYMBOL(avi_setup_blender_input);

/**
 * avi_setup_blendin() - Setup blending configuration
 * @blendin: pointer to AVI virtual blendin node
 * @xoffset: x coordinate of upper left corner of frame to blend
 * @yoffset: y coordinate of upper left corner of frame to blend
 * @alpha: alpha component used to blend plane with, or enable OSD mode if < 0
 */
void avi_setup_blendin(struct avi_node const* blendin,
		       u16 xoffset,
		       u16 yoffset,
		       int alpha)
{
	struct avi_blendin_cfg cfg;

	avi_blendin_2cfg(&cfg, blendin, xoffset, yoffset, alpha);
	avi_set_blendin_cfg(blendin, &cfg);
}
EXPORT_SYMBOL(avi_setup_blendin);

/**
 * avi_setup_blendin_alpha() - Setup blending alpha
 * @blendin: pointer to AVI virtual blendin node
 * @alpha: alpha component used to blend plane with, or enable OSD mode if < 0
 */
void avi_setup_blendin_alpha(struct avi_node const* blendin,
			     int alpha)
{
	unsigned long const	base = avi_node_base(blendin);
	int			id   = avi_blendin_id(blendin);
	struct avi_blendin_cfg	cfg;

	avi_blendin_2cfg(&cfg, blendin, 0, 0, alpha);

	AVI_WRITE(cfg.alpha,
		  base + AVI_BLEND_ALPHA0 + (id * sizeof(u32)) * 2);
}
EXPORT_SYMBOL(avi_setup_blendin_alpha);

/**
 * avi_setup_blend() - Setup blender configuration
 * @blender: pointer to AVI virtual blendin node
 * @background: background plane color
 * @width: blender output width in pixels
 * @height: blender output height in pixels
 */
void avi_setup_blend(struct avi_node const* blender,
		     u32 background,
		     u16 width,
		     u16 height)
{
	unsigned long const base = avi_node_base(blender);

	avi_blend_set_bg(blender, background);
	AVI_WRITE(((height - 1) << 16) | (width - 1),
		  base + AVI_BLEND_FRAMEOUTSIZE);
}
EXPORT_SYMBOL(avi_setup_blend);

u32 avi_blend_get_bg(struct avi_node const *blender)
{
	unsigned long const base = avi_node_base(blender);

	return AVI_READ(base + AVI_BLEND_BACKGROUND);
}
EXPORT_SYMBOL(avi_blend_get_bg);

void avi_blend_set_bg(struct avi_node const *blender, u32 bg)
{
	unsigned long const base = avi_node_base(blender);

	AVI_WRITE(bg & 0x00ffffffUL, base + AVI_BLEND_BACKGROUND);
}
EXPORT_SYMBOL(avi_blend_set_bg);

/************************
 * Gamma Corrector setup
 ***********************/

void avi_gam_setup(struct avi_node const* node,
		   int bypass, int palette, int ten_bits)
{
	unsigned long const			base = avi_node_base(node);
	union avi_isp_gamma_corrector_conf	conf = {{
		.bypass = !!bypass,
		.palette = !!palette,
		.comp_width = !!ten_bits,
	}};

	AVI_WRITE(conf._register, base + AVI_ISP_GAMMA_CORRECTOR_CONF);
}
EXPORT_SYMBOL(avi_gam_setup);

u32 avi_gam_get_config(struct avi_node const* node)
{
	unsigned long const base = avi_node_base(node);

	return AVI_READ(base + AVI_ISP_GAMMA_CORRECTOR_CONF);
}
EXPORT_SYMBOL(avi_gam_get_config);

void avi_gam_set_entry(struct avi_node const *gam,
		       u16 regno,
		       u8 red, u8 green, u8 blue)
{
	unsigned long const base = avi_node_base(gam);

	AVI_WRITE(red,
		  base + AVI_ISP_GAMMA_CORRECTOR_RY_LUT + ((u32)regno << 2));
	AVI_WRITE(green,
		  base + AVI_ISP_GAMMA_CORRECTOR_GU_LUT + ((u32)regno << 2));
	AVI_WRITE(blue,
		  base + AVI_ISP_GAMMA_CORRECTOR_BV_LUT + ((u32)regno << 2));
}
EXPORT_SYMBOL(avi_gam_set_entry);

void avi_gam_set_cmap(struct avi_node const *gam, struct avi_cmap const *cmap)
{
	unsigned i;

	for (i = 0; i < AVI_CMAP_SZ; i++)
		avi_gam_set_entry(gam, i,
				  cmap->red[i],
				  cmap->green[i],
				  cmap->blue[i]);
}
EXPORT_SYMBOL(avi_gam_set_cmap);

void avi_gam_get_cmap(struct avi_node const *gam, struct avi_cmap *cmap)
{
	unsigned long const	base = avi_node_base(gam);
	unsigned		i;

	for (i = 0; i < AVI_CMAP_SZ; i++) {
		cmap->red  [i] = AVI_READ(base + (i << 2)
					  + AVI_ISP_GAMMA_CORRECTOR_RY_LUT);
		cmap->green[i] = AVI_READ(base + (i << 2)
					  + AVI_ISP_GAMMA_CORRECTOR_GU_LUT);
		cmap->blue [i] = AVI_READ(base + (i << 2)
					  + AVI_ISP_GAMMA_CORRECTOR_BV_LUT);
	}
}
EXPORT_SYMBOL(avi_gam_get_cmap);

/**************
 * LCD config
 **************/

void avi_lcd_get_registers(struct avi_node const *lcd,
			   struct avi_lcd_regs *regs)
{
	memcpy_from_registers(regs, avi_node_base(lcd), sizeof(*regs));
}
EXPORT_SYMBOL(avi_lcd_get_registers);

void avi_lcd_set_registers(struct avi_node const *lcd,
			   struct avi_lcd_regs const *regs)
{
	memcpy_to_registers(avi_node_base(lcd), regs, sizeof(*regs));
}
EXPORT_SYMBOL(avi_lcd_set_registers);

enum avi_field avi_lcd_get_current_field(struct avi_node const *lcd)
{
		unsigned long const addr = avi_node_base(lcd) + AVI_LCD_STATUS;

		return (AVI_READ(addr) & 0x4) ? AVI_BOT_FIELD : AVI_TOP_FIELD;
}
EXPORT_SYMBOL(avi_lcd_get_current_field);

unsigned avi_lcd_get_colorbar(struct avi_node const *lcd)
{
		unsigned long const addr = avi_node_base(lcd) + AVI_LCD_DPD;
		union avi_lcd_dpd   ret;

		ret._register = AVI_READ(addr);

		return ret.colorbar;
}
EXPORT_SYMBOL(avi_lcd_get_colorbar);

/**
 * avi_lcd_get_colorbar: set the colorbar register for the given LCD. Only the
 * two LSB of colorbar will be used: (colorbar & 2) enables/disables the
 * colorbar, (colorbar & 1) when set means RGB mode, unset means YUV mode.
 */
void  avi_lcd_set_colorbar(struct avi_node const *lcd, unsigned colorbar)
{
		unsigned long const addr = avi_node_base(lcd) + AVI_LCD_DPD;
		union avi_lcd_dpd   dpd;

		dpd._register = AVI_READ(addr);

		dpd.colorbar = colorbar;
		AVI_WRITE(dpd._register, addr);
}
EXPORT_SYMBOL(avi_lcd_set_colorbar);

unsigned avi_lcd_get_force_clear(struct avi_node const *lcd)
{
	union avi_lcd_force_clear	clr;
	unsigned long const		addr =
		avi_node_base(lcd) + AVI_LCD_FORCE_CLEAR;

	clr._register	= AVI_READ(addr);

	return clr.force_clear;
}
EXPORT_SYMBOL(avi_lcd_get_force_clear);

void avi_lcd_set_force_clear(struct avi_node const *lcd, unsigned clear)
{
	union avi_lcd_force_clear	clr;
	unsigned long const		addr =
		avi_node_base(lcd) + AVI_LCD_FORCE_CLEAR;

	clr._register	= AVI_READ(addr);
	clr.force_clear = clear;

	AVI_WRITE(clr._register, addr);
}
EXPORT_SYMBOL(avi_lcd_set_force_clear);

union avi_lcd_status avi_lcd_get_status(struct avi_node const *lcd)
{
	unsigned long const  addr = avi_node_base(lcd) + AVI_LCD_STATUS;
	union avi_lcd_status ret;

	ret._register = AVI_READ(addr);

	return ret;
}
EXPORT_SYMBOL(avi_lcd_get_status);

u32 avi_lcd_get_dpd(struct avi_node const *lcd)
{
	unsigned long const  addr = avi_node_base(lcd) + AVI_LCD_DPD;
	union avi_lcd_dpd ret;

	ret._register = AVI_READ(addr);

	return ret.dpd;
}
EXPORT_SYMBOL(avi_lcd_get_dpd);

void avi_lcd_set_dpd(struct avi_node const *lcd, u32 dpd)
{
	unsigned long const	addr = avi_node_base(lcd) + AVI_LCD_DPD;
	union avi_lcd_dpd       v;

	v._register = AVI_READ(addr);

	v.dpd = dpd & 0xffffff;

	AVI_WRITE(v._register, addr);
}
EXPORT_SYMBOL(avi_lcd_set_dpd);

int avi_lcd_pixclock_enable(struct avi_node const *n)
{
	struct avi_lcd	*lcd;
	int		 ret = 0;

	BUG_ON(n->type != AVI_LCD_NODE_TYPE);

	lcd = container_of(n, struct avi_lcd, avi_node);

	if (lcd->clk_enabled)
		ret = -EBUSY;
	else {
		ret = clk_prepare_enable(lcd->pixclock);
		lcd->clk_enabled = (ret == 0);
	}

	return ret;
}
EXPORT_SYMBOL(avi_lcd_pixclock_enable);

int avi_lcd_pixclock_disable(struct avi_node const *n)
{
	struct avi_lcd	*lcd;
	int		 ret = 0;

	BUG_ON(n->type != AVI_LCD_NODE_TYPE);

	lcd = container_of(n, struct avi_lcd, avi_node);

	if (!lcd->clk_enabled)
		ret = -EINVAL;
	else {
		clk_disable_unprepare(lcd->pixclock);
		lcd->clk_enabled = 0;
	}

	return ret;
}
EXPORT_SYMBOL(avi_lcd_pixclock_disable);

unsigned long avi_lcd_pixclock_round_rate(struct avi_node const *n,
					  unsigned long r)
{
	struct avi_lcd	*lcd;

	BUG_ON(n->type != AVI_LCD_NODE_TYPE);

	lcd = container_of(n, struct avi_lcd, avi_node);

	return clk_round_rate(lcd->pixclock, r);
}
EXPORT_SYMBOL(avi_lcd_pixclock_round_rate);

int avi_lcd_pixclock_set_rate(struct avi_node const *n,
			      unsigned long r)
{
	struct avi_lcd	*lcd;

	BUG_ON(n->type != AVI_LCD_NODE_TYPE);

	lcd = container_of(n, struct avi_lcd, avi_node);

	return clk_set_rate(lcd->pixclock, r);
}
EXPORT_SYMBOL(avi_lcd_pixclock_set_rate);

unsigned long avi_lcd_pixclock_get_rate(struct avi_node const *n)
{
	struct avi_lcd	*lcd;

	BUG_ON(n->type != AVI_LCD_NODE_TYPE);

	lcd = container_of(n, struct avi_lcd, avi_node);

	return clk_get_rate(lcd->pixclock);
}
EXPORT_SYMBOL(avi_lcd_pixclock_get_rate);

void avi_vsync_gen_set_registers(struct avi_node const *lcd,
				 struct avi_lcd_avi_vsync_gen_regs const *regs)
{
	memcpy_to_registers(avi_node_base(lcd)
			    + AVI_LCD_AVI_VSYNC_GEN_VSYNC_GEN_ON_0,
			    regs,
			    sizeof(*regs));
}
EXPORT_SYMBOL(avi_vsync_gen_set_registers);

void avi_vsync_gen_get_registers(struct avi_node const *lcd,
				 struct avi_lcd_avi_vsync_gen_regs *regs)
{
	memcpy_from_registers(regs, avi_node_base(lcd)
			      + AVI_LCD_AVI_VSYNC_GEN_VSYNC_GEN_ON_0,
			      sizeof(*regs));
}
EXPORT_SYMBOL(avi_vsync_gen_get_registers);

extern void avi_vsync_gen_set_registers(struct avi_node const*,
					struct avi_lcd_avi_vsync_gen_regs const*);


void avi_cam_set_registers(struct avi_node const *cam,
			   struct avi_cam_registers const *regs)
{
		memcpy_to_registers(avi_node_base(cam), regs, sizeof(*regs));
}
EXPORT_SYMBOL(avi_cam_set_registers);

void avi_cam_get_registers(struct avi_node const *cam,
			   struct avi_cam_registers *regs)
{
		memcpy_from_registers(regs, avi_node_base(cam), sizeof(*regs));
}
EXPORT_SYMBOL(avi_cam_get_registers);

union avi_cam_status avi_cam_get_status(struct avi_node const *cam)
{
		unsigned long const addr = avi_node_base(cam) + AVI_CAM_STATUS;
		union avi_cam_status ret;

		ret._register = AVI_READ(addr);

		return ret;
}
EXPORT_SYMBOL(avi_cam_get_status);

void avi_cam_get_measure(struct avi_node const *cam,
						 struct avi_cam_measure_regs *regs)
{
		memcpy_from_registers(regs,
				      avi_node_base(cam) + AVI_CAM_MESURE_H_TIMING0,
				      sizeof(*regs));
}
EXPORT_SYMBOL(avi_cam_get_measure);

void avi_cam_set_itsource(struct avi_node const *cam,
						  union avi_cam_itsource itsrc)
{
		unsigned long const addr = avi_node_base(cam) + AVI_CAM_ITSOURCE;

		AVI_WRITE(itsrc._register, addr);
}
EXPORT_SYMBOL(avi_cam_set_itsource);

void avi_cam_set_timings(struct avi_node const *cam,
                         union avi_cam_h_timing ht,
                         union avi_cam_v_timing vt)
{
		unsigned long const haddr = avi_node_base(cam) + AVI_CAM_H_TIMING;
		unsigned long const vaddr = avi_node_base(cam) + AVI_CAM_V_TIMING;

		AVI_WRITE(ht._register, haddr);
		AVI_WRITE(vt._register, vaddr);
}
EXPORT_SYMBOL(avi_cam_set_timings);

void avi_cam_set_run(struct avi_node const *cam,
                     union avi_cam_run run)
{
		unsigned long const addr = avi_node_base(cam) + AVI_CAM_RUN;

		AVI_WRITE(run._register, addr);
}
EXPORT_SYMBOL(avi_cam_set_run);

void avi_cam_run(struct avi_node const *cam)
{
	unsigned long const addr = avi_node_base(cam) + AVI_CAM_RUN;
	union avi_cam_run run = {
		.run = 1,
	};

	AVI_WRITE(run._register, addr);
}
EXPORT_SYMBOL(avi_cam_run);

void avi_cam_freerun(struct avi_node const *cam)
{
	unsigned long const addr = avi_node_base(cam) + AVI_CAM_RUN;
	union avi_cam_run run = {
		.free_run = 1,
	};

	AVI_WRITE(run._register, addr);
}
EXPORT_SYMBOL(avi_cam_freerun);

void avi_inter_get_registers(u32 regs[AVI_INTER_NSINK])
{
	memcpy_from_registers(regs,
			      avi_node_base(&avi_inter),
			      AVI_INTER_NSINK * sizeof(u32));
}
EXPORT_SYMBOL(avi_inter_get_registers);

/***********************
 * Rotation handling
 ***********************/
void avi_rotator_setup(struct avi_node const* rot,
		       enum avi_rotation rotation,
		       bool flip,
		       unsigned height_out,
		       unsigned nstrips)
{
	unsigned long const angle = avi_node_base(rot) + AVI_ROT_ANGLE;
	unsigned long const size = avi_node_base(rot) + AVI_ROT_SIZE;

	switch(rotation)
	{
		case AVI_ROTATION_0:
			AVI_WRITE(AVI_ROT_ANGLE_0|(AVI_ROT_HORIZ_FLIP*flip),
				  angle);
			break;
		case AVI_ROTATION_90:
			AVI_WRITE(AVI_ROT_ANGLE_90|(AVI_ROT_HORIZ_FLIP*flip),
				  angle);
			break;
		case AVI_ROTATION_180:/* rotation is done by FIFOs */
			AVI_WRITE(AVI_ROT_ANGLE_0|(AVI_ROT_HORIZ_FLIP*~flip),
				  angle);
			break;
		case AVI_ROTATION_270:
			AVI_WRITE(AVI_ROT_ANGLE_270|(AVI_ROT_HORIZ_FLIP*flip),
				  angle);
			break;
		default:
			/* Unknown rotation, there is a problem ! */
			BUG_ON(1);
	}

	AVI_WRITE(((height_out & AVI_ROT_HEIGHTOUT_MASK)<<AVI_ROT_HEIGHTOUT_SHIFT)|
		  ((nstrips & AVI_ROT_NSTRIPE_MASK)<<AVI_ROT_NSTRIPE_SHIFT),
		  size);
}

/******
 * ISP
 ******/
/* Inter */
void avi_isp_chain_bayer_inter_set_registers(
		struct avi_node const *chain_bayer,
		struct avi_isp_chain_bayer_inter_regs const *regs)
{
	memcpy_to_registers(avi_node_base(chain_bayer) +
			    AVI_ISP_CHAIN_BAYER_INTER,
			    regs,
			    sizeof(*regs));
}
EXPORT_SYMBOL(avi_isp_chain_bayer_inter_set_registers);

void avi_isp_chain_bayer_inter_get_registers(
		struct avi_node const *chain_bayer,
		struct avi_isp_chain_bayer_inter_regs *regs)
{
	memcpy_from_registers(regs,
			      avi_node_base(chain_bayer) +
			      AVI_ISP_CHAIN_BAYER_INTER,
			      sizeof(*regs));
}
EXPORT_SYMBOL(avi_isp_chain_bayer_inter_get_registers);

void avi_isp_chain_yuv_inter_set_registers(
		struct avi_node const *chain_yuv,
		struct avi_isp_chain_yuv_inter_regs const *regs)
{
	memcpy_to_registers(avi_node_base(chain_yuv) +
			    AVI_ISP_CHAIN_YUV_INTER,
			    regs,
			    sizeof(*regs));
}
EXPORT_SYMBOL(avi_isp_chain_yuv_inter_set_registers);

void avi_isp_chain_yuv_inter_get_registers(
		struct avi_node const *chain_yuv,
		struct avi_isp_chain_yuv_inter_regs *regs)
{
	memcpy_from_registers(regs,
			      avi_node_base(chain_yuv) +
			      AVI_ISP_CHAIN_YUV_INTER,
			      sizeof(*regs));
}
EXPORT_SYMBOL(avi_isp_chain_yuv_inter_get_registers);

/* Bayer */
void avi_isp_bayer_set_registers(struct avi_node const *chain_bayer,
				 struct avi_isp_bayer_regs const *regs)
{
	memcpy_to_registers(avi_node_base(chain_bayer) + AVI_ISP_BAYER,
			    regs,
			    sizeof(*regs));
}
EXPORT_SYMBOL(avi_isp_bayer_set_registers);

void avi_isp_bayer_get_registers(struct avi_node const *chain_bayer,
			         struct avi_isp_bayer_regs *regs)
{
	memcpy_from_registers(regs,
			      avi_node_base(chain_bayer) + AVI_ISP_BAYER,
			      sizeof(*regs));
}
EXPORT_SYMBOL(avi_isp_bayer_get_registers);

/* Stats */
void avi_isp_statistics_bayer_set_registers(
	struct avi_node                            *chain_bayer,
	const struct avi_isp_statistics_bayer_regs *regs)
{
	memcpy_to_registers(avi_node_base(chain_bayer) +
			    AVI_ISP_STATS_BAYER,
			    regs,
			    sizeof(*regs));
}
EXPORT_SYMBOL(avi_isp_statistics_bayer_set_registers);

void avi_isp_statistics_bayer_get_registers(
	struct avi_node                      *chain_bayer,
	struct avi_isp_statistics_bayer_regs *regs)
{
	memcpy_from_registers(regs,
			      avi_node_base(chain_bayer) +
			      AVI_ISP_STATS_BAYER,
			      sizeof(*regs));
}
EXPORT_SYMBOL(avi_isp_statistics_bayer_get_registers);

void avi_isp_pedestal_get_registers(struct avi_node *chain_bayer,
				    struct avi_isp_pedestal_regs *regs)
{
	memcpy_from_registers(regs,
			avi_node_base(chain_bayer) + AVI_ISP_PEDESTAL,
			sizeof(*regs));
}
EXPORT_SYMBOL(avi_isp_pedestal_get_registers);

void avi_isp_grim_get_registers(struct avi_node *chain_bayer,
		struct avi_isp_green_imbalance_regs *regs,
		struct avi_isp_green_imbalance_green_red_coeff_mem_regs *gr_regs,
		struct avi_isp_green_imbalance_green_blue_coeff_mem_regs *gb_regs)
{
	unsigned long addr;

	addr = avi_node_base(chain_bayer) + AVI_ISP_GREEN_IMBALANCE;

	memcpy_from_registers(regs, addr, sizeof(*regs));
	memcpy_from_registers(gr_regs,
			addr + AVI_ISP_GREEN_IMBALANCE_GREEN_RED_COEFF_MEM,
			sizeof(*gr_regs));
	memcpy_from_registers(gb_regs,
			addr + AVI_ISP_GREEN_IMBALANCE_GREEN_BLUE_COEFF_MEM,
			sizeof(*gb_regs));
}
EXPORT_SYMBOL(avi_isp_grim_get_registers);

void avi_isp_dpc_rgrim_get_registers(struct avi_node *chain_bayer,
		struct avi_isp_dead_pixel_correction_regs *regs,
		struct avi_isp_dead_pixel_correction_list_mem_regs *list)
{
	unsigned long addr;

	addr = avi_node_base(chain_bayer) + AVI_ISP_DEAD_PIXEL_CORRECTION;

	memcpy_from_registers(regs, addr + AVI_ISP_DEAD_PIXEL_CORRECTION_CFA,
			      sizeof(*regs));
	memcpy_from_registers(list,
			      addr + AVI_ISP_DEAD_PIXEL_CORRECTION_LIST_MEM,
			      sizeof(*list));
}
EXPORT_SYMBOL(avi_isp_dpc_rgrim_get_registers);

void avi_isp_denoising_get_registers(struct avi_node *chain_bayer,
				     struct avi_isp_denoising_regs *regs)
{
	memcpy_from_registers(regs,
			      avi_node_base(chain_bayer) + AVI_ISP_DENOISING,
			      sizeof(*regs));
}
EXPORT_SYMBOL(avi_isp_denoising_get_registers);

void avi_isp_lsc_get_registers(struct avi_node *chain_bayer,
		struct avi_isp_lens_shading_correction_regs *regs,
		struct avi_isp_lens_shading_correction_red_coeff_mem_regs *r_regs,
		struct avi_isp_lens_shading_correction_green_coeff_mem_regs *g_regs,
		struct avi_isp_lens_shading_correction_blue_coeff_mem_regs *b_regs)
{
	unsigned long addr;

	addr = avi_node_base(chain_bayer) + AVI_ISP_LENS_SHADING_CORRECTION;

	memcpy_from_registers(regs, addr, sizeof (*regs));
	memcpy_from_registers(r_regs,
			      addr + AVI_ISP_LENS_SHADING_CORRECTION_RED_COEFF_MEM,
			      sizeof(*r_regs));
	memcpy_from_registers(g_regs,
			      addr + AVI_ISP_LENS_SHADING_CORRECTION_GREEN_COEFF_MEM,
			      sizeof(*g_regs));
	memcpy_from_registers(b_regs,
			      addr + AVI_ISP_LENS_SHADING_CORRECTION_BLUE_COEFF_MEM,
			      sizeof(*b_regs));
}
EXPORT_SYMBOL(avi_isp_lsc_get_registers);

void avi_isp_ca_correction_get_registers(struct avi_node *chain_bayer,
		struct avi_isp_chromatic_aberration_regs *regs)
{
	memcpy_from_registers(regs,
			      avi_node_base(chain_bayer) + AVI_ISP_CHROMATIC_ABERRATION,
			      sizeof(*regs));
}
EXPORT_SYMBOL(avi_isp_ca_correction_get_registers);

void avi_isp_color_correction_get_registers(struct avi_node *chain_bayer,
		struct avi_isp_color_correction_regs *regs)
{
	memcpy_from_registers(regs,
			      avi_node_base(chain_bayer) + AVI_ISP_COLOR_CORRECTION,
			      sizeof(*regs));
}
EXPORT_SYMBOL(avi_isp_color_correction_get_registers);

int avi_node_get_irq_flag(struct avi_node const *node)
{
	unsigned long addr;

	/* ISP modules do not follow the same register map scheme. */
	if (node->cfg_off == 3)
		addr = avi_ctrl.base + AVI_CFG_ISP_ITFLG3;
	else
		addr = avi_ctrl.base + AVI_CFG_ITFLG1 +
		       (node->cfg_off * sizeof(u32));

	return !!(node->cfg_mask & AVI_READ(addr));
}
EXPORT_SYMBOL(avi_node_get_irq_flag);

void avi_ack_node_irq(struct avi_node const *node)
{
	unsigned long addr;
	unsigned long flags;

	/* ISP modules do not follow the same register map scheme. */
	if (node->cfg_off == 3)
		addr = avi_ctrl.base + AVI_CFG_ISP_ITACK3;
	else
		addr = avi_ctrl.base + AVI_CFG_ITACK1 +
		       (node->cfg_off * sizeof(u32));

	spin_lock_irqsave(&avi_ctrl.node_lck, flags);
	AVI_WRITE(node->cfg_mask, addr);
	spin_unlock_irqrestore(&avi_ctrl.node_lck, flags);
}
EXPORT_SYMBOL(avi_ack_node_irq);

void avi_enable_node_irq(struct avi_node const *node)
{
	unsigned long addr;
	unsigned long flags;

	/* ISP modules do not follow the same register map scheme. */
	if (node->cfg_off == 3)
		addr = avi_ctrl.base + AVI_CFG_ISP_ITEN3;
	else
		addr = avi_ctrl.base + AVI_CFG_ITEN1 +
		       (node->cfg_off * sizeof(u32));

	/* Make sure we don't have a pending IRQ */
	avi_ack_node_irq(node);

	spin_lock_irqsave(&avi_ctrl.node_lck, flags);
	AVI_WRITE(node->cfg_mask | AVI_READ(addr), addr);
	spin_unlock_irqrestore(&avi_ctrl.node_lck, flags);

	avi_apply_node(&avi_cfg);
}
EXPORT_SYMBOL(avi_enable_node_irq);

void avi_disable_node_irq(struct avi_node const *node)
{
	unsigned long addr;
	unsigned long flags;

	/* ISP modules do not follow the same register map scheme. */
	if (node->cfg_off == 3)
		addr = avi_ctrl.base + AVI_CFG_ISP_ITEN3;
	else
		addr = avi_ctrl.base + AVI_CFG_ITEN1 +
		       (node->cfg_off * sizeof(u32));

	spin_lock_irqsave(&avi_ctrl.node_lck, flags);
	AVI_WRITE(~node->cfg_mask & AVI_READ(addr), addr);
	spin_unlock_irqrestore(&avi_ctrl.node_lck, flags);

	avi_apply_node(&avi_cfg);
}
EXPORT_SYMBOL(avi_disable_node_irq);

/***************************************************
 * Module initialization / shutdown implementation.
 ***************************************************/

static inline void memzero_regs(unsigned long base, size_t s)
{
	volatile u32 __iomem *reg = (u32 *)base;

	while (s--)
		AVI_WRITE(0, reg++);

#ifdef AVI_BACKWARD_COMPAT
	/* Because in P7R3 the ISP interconnect has been changed to 'bypass'
	 * the compatibility layer will set the P7R2 ISP interconnect to some
	 * values, so we override it here */
	if (avi_ctrl.revision == AVI_REVISION_2) {
		/* ISP Inter 0 */
		writel(0, base + 0x200);
		writel(0, base + 0x204);
		/* ISP Inter 1 */
		writel(0, base + 0x300);
		writel(0, base + 0x304);
	}
#endif /* AVI_BACKWARD_COMPAT */

}

/*
 * This initialization sequence makes sure the AVI is in a correct state
 * before we start anything. After a reset, the registers are left in a
 * non-deterministic state.
 */
int avi_dereset(void)
{
	union avi_cfg_dmacfg	dmacfg;
	int			ret;

	ret = clk_prepare_enable(avi_ctrl.clock);
	if (ret)
		return ret;

	/* Initialize all the registers to 0. */
	memzero_regs(avi_ctrl.base,
		     resource_size(avi_ctrl.iomem) / sizeof(u32));

	/* Enable all AVI nodes */
	AVI_WRITE(0xffffffff, avi_ctrl.base + AVI_CFG_ENABLE1);
	AVI_WRITE(0xffffffff, avi_ctrl.base + AVI_CFG_ENABLE2);
	AVI_WRITE(0xffffffff, avi_ctrl.base + AVI_CFG_ISP_ENABLE3);

	AVI_WRITE(0xffffffff, avi_ctrl.base + AVI_CFG_APPLY1);
	AVI_WRITE(0xffffffff, avi_ctrl.base + AVI_CFG_APPLY2);
	AVI_WRITE(0xffffffff, avi_ctrl.base + AVI_CFG_ISP_APPLY3);

	/* Disable all AVI nodes */
	AVI_WRITE(0, avi_ctrl.base + AVI_CFG_ENABLE1);
	AVI_WRITE(0, avi_ctrl.base + AVI_CFG_ENABLE2);
	AVI_WRITE(0, avi_ctrl.base + AVI_CFG_ISP_ENABLE3);

	AVI_WRITE(0xffffffff, avi_ctrl.base + AVI_CFG_APPLY1);
	AVI_WRITE(0xffffffff, avi_ctrl.base + AVI_CFG_APPLY2);
	AVI_WRITE(0xffffffff, avi_ctrl.base + AVI_CFG_ISP_APPLY3);

	/* DMA configuration. Those are arbitrary values known to work; they will
	 * probably need some adjusting after the ASIC make performance measurements
	 * for the AVI (and maybe depending on the configuration and the stress put
	 * on the AXI) */
	dmacfg.dma_flag_timeout	    = 0;
	dmacfg.dma_flag_number	    = 0;
	dmacfg.dma_max_burst_number = 3;
	dmacfg.dma_max_bandwidth    = 0x0f;
	AVI_WRITE(dmacfg._register, avi_ctrl.base + AVI_CFG_DMACFG);

	/* Scaler/Rotator memory reservation
	 * Reserve memory for each scaler
	 */
	/* Reserve SCAL0 memory slot */
	AVI_WRITE(0x1, avi_ctrl.base + AVI_ISP_SCAL_ROT_SCAL0_RESERVE);
	/* Reserve SCAL1 memory slot */
	AVI_WRITE(0x1, avi_ctrl.base + AVI_ISP_SCAL_ROT_SCAL1_RESERVE);

	/* Apply configuration, enable configuration and interconnect nodes */
	avi_enable_node(&avi_inter);
	avi_enable_node(&avi_cfg);
	avi_apply_node(&avi_inter);
	avi_apply_node(&avi_cfg);

	return 0;
}
EXPORT_SYMBOL(avi_dereset);

void avi_reset(void)
{
	clk_disable_unprepare(avi_ctrl.clock);
}
EXPORT_SYMBOL(avi_reset);

static int __devinit avi_probe(struct platform_device* pdev)
{
	struct resource*		 res;
	const struct avi_platform_data	*pdata;
	int				 ret;

	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata) {
		dev_err(&pdev->dev, "no platform data found\n");
		ret = -ENODATA;
		goto no_pdata;
	}

	avi_ctrl.dev      = &pdev->dev;
	avi_ctrl.revision = pdata->revision;
	dev_info(&pdev->dev, "Found AVI revision %d", avi_get_revision());

	avi_ctrl.iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!avi_ctrl.iomem) {
		dev_err(&pdev->dev, "failed to get I/O registers memory\n");
		ret = -ENXIO;
		goto no_iomem;
	}

	avi_ctrl.clock = clk_get(&pdev->dev, "ctrl");
	if (IS_ERR(avi_ctrl.clock)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		ret = PTR_ERR(avi_ctrl.clock);
		goto no_clk;
	}

	res = request_mem_region(avi_ctrl.iomem->start,
				 resource_size(avi_ctrl.iomem),
				 pdev->name);
	if (!res) {
		dev_err(&pdev->dev, "failed to request I/O register memory\n");
		ret = -EBUSY;
		goto request_mem_failed;
	}

	avi_log_init();

	avi_ctrl.irq_base = pdata->irq_base;

	avi_ctrl.base = (unsigned long) ioremap(res->start, resource_size(res));
	if (! avi_ctrl.base) {
		dev_err(&pdev->dev, "failed to remap I/O registers memory\n");
		ret = -ENOMEM;
		goto remap_failed;
	}

	ret = avi_dereset();
	if (ret)
		goto dereset_failed;

	avi_lcd0.pixclock = clk_get(&pdev->dev, "lcd0");
	if (IS_ERR(avi_lcd0.pixclock)) {
		dev_err(&pdev->dev, "no lcd0 clk\n");
		ret = PTR_ERR(avi_lcd0.pixclock);
		goto no_lcd0_clk;
	}

	avi_lcd1.pixclock = clk_get(&pdev->dev, "lcd1");
	if (IS_ERR(avi_lcd1.pixclock)) {
		dev_err(&pdev->dev, "no lcd1 clk\n");
		ret = PTR_ERR(avi_lcd1.pixclock);
		goto no_lcd1_clk;
	}

	dev_info(&pdev->dev, "AVI core driver initialized\n");

	return 0;

 no_lcd1_clk:
	clk_put(avi_lcd0.pixclock);
 no_lcd0_clk:
	avi_reset();
 dereset_failed:
	iounmap((void *)avi_ctrl.base);
 remap_failed:
	release_mem_region(avi_ctrl.iomem->start, resource_size(avi_ctrl.iomem));
 request_mem_failed:
	clk_put(avi_ctrl.clock);
 no_clk:
 no_iomem:
 no_pdata:
	/* Set the AVI base to 0 to indicate to the subdrivers that the avi
	 * didn't probe correctly */
	avi_ctrl.base = 0;
	return ret;
}

static int __devexit avi_remove(struct platform_device* pdev)
{
	clk_put(avi_lcd0.pixclock);
	clk_put(avi_lcd1.pixclock);

	avi_reset();

	iounmap((void *)avi_ctrl.base);
	release_mem_region(avi_ctrl.iomem->start, resource_size(avi_ctrl.iomem));
	clk_put(avi_ctrl.clock);

	return 0;
}

static int avi_suspend(struct device *dev)
{
       dev_info(dev, "AVI shutting down\n");

       avi_reset();

       return 0;
}

/* This function is only meant to be called from the core so I don't want to
 * declare it in the public headers. */
extern void avi_segment_resume(void);

static int avi_resume(struct device *dev)
{
       dev_info(dev, "AVI resuming\n");

       avi_dereset();

       avi_segment_resume();

       return 0;
}

static struct dev_pm_ops avi_dev_pm_ops = {
       /* I use the late/early callbacks to make sure the AVI functions are
        * sequenced correctly with regard to the AVI subdrivers (fb, cam...) */
       .suspend_late = &avi_suspend,
       .resume_early = &avi_resume,
};

static struct platform_driver avi_driver = {
	.driver         = {
		.name   = "avi",
		.owner  = THIS_MODULE,
		.pm     = &avi_dev_pm_ops,
	},
	.probe      = &avi_probe,
	.remove     = __devexit_p(&avi_remove),
};

static int __init avi_init(void)
{
	return platform_driver_register(&avi_driver);
}
module_init(avi_init);

static void __exit avi_exit(void)
{
	platform_driver_unregister(&avi_driver);
}
module_exit(avi_exit);

MODULE_AUTHOR("Gregor Boirie <gregor.boirie@parrot.com>");
MODULE_DESCRIPTION("Parrot Advanced Video Interface");
MODULE_LICENSE("GPL");
