#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include "avi_compat.h"
#include "avi_segment.h"
#include "avi_dma.h"
#include "avi_debug.h"
#include "avi_scaler.h"
#include "avi_pixfmt.h"

#ifdef DEBUG

#define dprintk(format_, args...) \
	pr_debug("avi segment: " format_, ##args)

#define dprintk_s(s_, format_, args...) \
	dev_dbg((s_)->owner, "[%s] " format_, (s_)->id, ##args)

#else /* DEBUG */
#define dprintk(format_, args...) do {} while(0)
#define dprintk_s(s_, format_, args...) (void)s_
#endif /* DEBUG */

struct avi_segment_index {
	struct mutex            lock;
	struct list_head        segment_list;
};

static struct avi_segment_index avi_segment_index = {
	.lock         = __MUTEX_INITIALIZER(avi_segment_index.lock),
	.segment_list = LIST_HEAD_INIT(avi_segment_index.segment_list),
};

/*
 * This function contains the heuristic used to build the segments. Hopefully it
 * should handle all the cases and the client application won't have to bother
 * tweaking the order of the nodes within a segment.
 *
 * This function is called with avi_segment_index.lock held.
 */
static int avi_segment_satisfy_caps(struct avi_segment *s)
{
	unsigned long   caps    = s->caps;
	unsigned long   missing = 0;
	int             ret     = 0;
	int             i       = 0;
	unsigned long   flags;
	struct avi_node *n;

	/* First some basic (and probably non-exhaustive) sanity checks on the
	 * capabilities. Some combinations make no sense (like having both a
	 * DMA_OUT and an LCD or multiple LCDs for instance) */
	BUG_ON(!caps);

	BUG_ON(avi_cap_check(caps));

	/* At the moment each capability has an associated node. This might
	 * change later on.
	 */

	/* Play nicely with legacy code. We don't want it to allocate nodes in
	 * our back. */
	spin_lock_irqsave(&avi_ctrl.cfg_lck, flags);

	/* Planar special case:
	 * If requested caps contain DMA_IN, SCALER and PLANAR
	 * we try to find a "PLANAR" node (DMA_IN)
	 */
	s->dma_in_planar = NULL;
	if (caps & AVI_CAP_PLANAR) {
		n = avi_cap_get_node(AVI_CAP_PLANAR);
		if (n) {
			n->busy = 1;
			n->assigned_caps = AVI_CAP_PLANAR;
		}
#ifdef AVI_BACKWARD_COMPAT
		else if ((avi_get_revision() < AVI_REVISION_3))
			/* for AVI revision 3, DMA IN FIFO works with
			 * semi-planar pixel formats instead of revision 1 and 2
			 * where they are bugged.  So in this case, a second DMA
			 * IN FIFO is mandatory for managing semi-planar */
			missing |= (AVI_CAP_PLANAR);
#endif /* AVI_BACKWARD_COMPAT */
		s->dma_in_planar = n;
		caps &=	~(AVI_CAP_PLANAR);
	}
	/* Unlock CFG to allow kzalloc() below with GFP_KERNEL. */
	spin_unlock_irqrestore(&avi_ctrl.cfg_lck, flags);

	/*
	 * The Hamming weight is the number of bits set in the int.
	 */
	s->nodes_nr = hweight_long(caps);
	s->nodes    = kzalloc(s->nodes_nr * sizeof(*s->nodes), GFP_KERNEL);
	if (!s->nodes) {
		if (s->dma_in_planar) {
			s->dma_in_planar->assigned_caps = 0;
			s->dma_in_planar->busy = 0;
		}
		ret = -ENOMEM;
		return ret;
	}

	/* Play nicely with legacy code. We don't want it to allocate nodes in
	 * our back. */
	spin_lock_irqsave(&avi_ctrl.cfg_lck, flags);

#define AVI_SEGMENT_FIND_NODE(cap) do {                         \
		if (caps & (cap)) {                             \
			n = avi_cap_get_node(cap);              \
			if (!n) {                               \
				missing |= (cap);               \
			} else {                                \
				n->busy = 1;                    \
				n->assigned_caps = cap;         \
			}                                       \
			s->nodes[i++] = n;                      \
			caps &=	~(cap);                         \
		}                                               \
	} while(0)

	AVI_SEGMENT_FIND_NODE(AVI_CAP_CAM_0);
	AVI_SEGMENT_FIND_NODE(AVI_CAP_CAM_1);
	AVI_SEGMENT_FIND_NODE(AVI_CAP_CAM_2);
	AVI_SEGMENT_FIND_NODE(AVI_CAP_CAM_3);
	AVI_SEGMENT_FIND_NODE(AVI_CAP_CAM_4);
	AVI_SEGMENT_FIND_NODE(AVI_CAP_CAM_5);

	AVI_SEGMENT_FIND_NODE(AVI_CAP_DMA_IN);

	AVI_SEGMENT_FIND_NODE(AVI_CAP_STATS_BAYER_0);
	AVI_SEGMENT_FIND_NODE(AVI_CAP_STATS_BAYER_1);

	/* Selecting one part of ISP (Bayer or YUV)
	 *  implies select of all modules
	 *  It does not make sense to take only a part of the ISP-chain
	 *  See AVI_CAPS_ISP in avi_cap.h
	 * For the ISP chain, the order is fixed
	 */
	if (caps & (AVI_CAP_ISP_CHAIN_BAYER|AVI_CAP_ISP_CHAIN_YUV)) {
		AVI_SEGMENT_FIND_NODE(AVI_CAP_ISP_CHAIN_BAYER);
		AVI_SEGMENT_FIND_NODE(AVI_CAP_GAM);
		AVI_SEGMENT_FIND_NODE(AVI_CAP_CONV);
		AVI_SEGMENT_FIND_NODE(AVI_CAP_STATS_YUV);
		AVI_SEGMENT_FIND_NODE(AVI_CAP_ISP_CHAIN_YUV);
	}

	AVI_SEGMENT_FIND_NODE(AVI_CAP_SCAL);
	AVI_SEGMENT_FIND_NODE(AVI_CAP_BUFFER);

	AVI_SEGMENT_FIND_NODE(AVI_CAP_CONV);

	/* In P7R1 the gamma and the LCD are directly connected in hardware,
	 * therefore we have to put the gamma just before the LCD here if we
	 * want to remain compatible with this version */
	AVI_SEGMENT_FIND_NODE(AVI_CAP_GAM);
	AVI_SEGMENT_FIND_NODE(AVI_CAP_LCD_0);
	AVI_SEGMENT_FIND_NODE(AVI_CAP_LCD_1);
	AVI_SEGMENT_FIND_NODE(AVI_CAP_DMA_OUT);

	/* We shouldn't have unhandled capabilities at that point. If we have it
	 * means our code is incomplete or the caller gave us garbage (undefined
	 * caps in the bitfield) */
	BUG_ON(caps);

	if (missing) {
		int j;

		for (j = 0; j < s->nodes_nr; j++)
			if (s->nodes[j]) {
				s->nodes[j]->assigned_caps = 0;
				s->nodes[j]->busy = 0;
			}
		kfree(s->nodes);
		if (s->dma_in_planar) {
			s->dma_in_planar->assigned_caps = 0;
			s->dma_in_planar->busy = 0;
		}
		/* update s->caps with the unsatisfied capabilities */
		s->caps = missing;
		ret = -ENODEV;
		goto unlock;
	}

	/* We should have filled the entire array by that point */
	BUG_ON(i != s->nodes_nr);

unlock:
	spin_unlock_irqrestore(&avi_ctrl.cfg_lck, flags);

	return ret;
}

/* Set sink's source to src_id in the AVI interconnect. If src_id is 0 it
 * disconnects the node. */
static void avi_segment_set_node_source(struct avi_node *inter,
                                        unsigned sink_id,
                                        unsigned src_id)
{
	unsigned        base;
	unsigned        shift;
	u32             reg;

	BUG_ON(src_id == AVI_SRCSINK_NONE);

	/* We have 4 src_ids per 32 bit registers. */
	base = avi_node_base(inter) + (sink_id & ~(3UL));
	/* Now that we have the base we need to find the layout within
	 * the register */
	shift = (sink_id % 4) * 8;

	reg = AVI_READ(base);

	/* We mask the previous value of the src id */
	reg &= ~(0xffUL << shift);
	reg |= src_id << shift;

	AVI_WRITE(reg, base);
}

static inline void avi_segment_set_blender_src_id(struct avi_node *inter,
                                                  struct avi_node *blender,
                                                  unsigned input,
                                                  unsigned src_id)
{
	unsigned        sink_id = blender->sink_id + input;

	BUG_ON(input >= 4);

	avi_segment_set_node_source(inter, sink_id, src_id);
}

/* Connect src to a blender input. If src is NULL it disconnects the input. */
static inline void avi_segment_set_blender_source(struct avi_node	*inter,
                                                  struct avi_node	*blender,
                                                  unsigned		 input,
                                                  struct avi_segment	*src)
{
	unsigned src_id  = 0;

	if (src)
		src_id = src->nodes[src->nodes_nr - 1]->src_id;

	if (!src || src->layout.hidden) {
		/* hide plane */
		avi_setup_blender_input(blender, input,
		                        0xffff, 0xffff,
		                        0);
	} else {
		unsigned blender_y = src->layout.y;

		if (src->output_format.interlaced)
			/* If the stream is interlaced we have to use the offset
			 * in the field, not the full frame */
			blender_y /= 2;

		avi_setup_blender_input(blender, input,
		                        src->layout.x, blender_y,
		                        src->layout.alpha);
	}

	avi_segment_set_blender_src_id(inter, blender, input, src_id);
}

/* must be called with avi_ctrl.cfg_lck held */
static void avi_segment_connect_nodes(struct avi_segment *s)
{
	struct avi_node *inter = avi_get_node(AVI_INTER_NODE);
	struct avi_node *scaler;
	int             i;

	avi_lock_node(inter);

	for (i = 1; i < s->nodes_nr; i++)
		avi_segment_set_node_source(inter,
		                            s->nodes[i]->sink_id,
		                            s->nodes[i - 1]->src_id);

	if (s->dma_in_planar) {
		scaler = avi_segment_get_node(s, AVI_CAP_SCAL);

		BUG_ON(!scaler);

		avi_segment_set_node_source(inter,
					    scaler->sink_id + 1,
					    s->dma_in_planar->src_id);
	}

	avi_unlock_node(inter);
}

/* Must be called with avi_ctrl.cfg_lck held */
static void avi_segment_disconnect_nodes(struct avi_segment *s)
{
	struct avi_node *inter = avi_get_node(AVI_INTER_NODE);
	int             i;

	avi_lock_node(inter);

	if (s->nodes_nr)
		for (i = 0; i < s->nodes_nr; i++)
			avi_segment_set_node_source(inter,
			                            s->nodes[i]->sink_id,
			                            0);

	if (s->dma_in_planar) {
		struct avi_node *scaler = avi_segment_get_node(s, AVI_CAP_SCAL);

		BUG_ON(!scaler);

		avi_segment_set_node_source(inter,
					    scaler->sink_id + 1,
					    0);
	}

	avi_unlock_node(inter);

	/* The node disconnection will only take place at the next clear, but
	 * since we're in the process of destroying the segment it won't happen
	 * any time soon. Therefore we force the interconnect change right
	 * here. */
	avi_apply_node(inter);
}

/* Must be called with the index lock held */
struct avi_segment *avi_segment_find(const char *id)
{
	struct avi_segment *s;

	BUG_ON(!id || !*id);

	list_for_each_entry(s, &avi_segment_index.segment_list, list)
		if (strncmp(s->id, id, ARRAY_SIZE(s->id)) == 0)
			return s;

	/* not found */
	return NULL;
}
EXPORT_SYMBOL(avi_segment_find);

/* Reconfigure all nodes in the segment to sane default values.
 *
 * Must be called with s->lock held. */
static void avi_segment_reset(struct avi_segment *s)
{
	struct avi_node *n;
	int              i;

	for (i = 0; i < s->nodes_nr; i++) {
		 n = s->nodes[i];

		switch (n->type) {
		case AVI_FIFO_NODE_TYPE:
		{
			struct avi_fifo_registers fifo_regs;

			memset(&fifo_regs, 0, sizeof(fifo_regs));

			if ((s->caps & AVI_CAP_DMA_IN) && i == 0)
				/* DMA IN fifo */
				fifo_regs.cfg.srctype = AVI_FIFO_CFG_DMA_TYPE;
			else
				fifo_regs.cfg.srctype = AVI_FIFO_CFG_VL_TYPE;


			if ((s->caps & AVI_CAP_DMA_OUT) &&
			    i == (s->nodes_nr - 1))
				/* DMA OUT fifo */
				fifo_regs.cfg.dsttype = AVI_FIFO_CFG_DMA_TYPE;
			else
				fifo_regs.cfg.dsttype = AVI_FIFO_CFG_VL_TYPE;

			fifo_regs.cfg.dstbitenc = 0x8;  /* 8888 */
			fifo_regs.cfg.srcbitenc = 0x8;  /* 8888 */
			fifo_regs.cfg.itline    = 0xf;
			fifo_regs.cfg.dithering = 1;    /* MSB filling */

			fifo_regs.swap0._register = AVI_FIFO_NULL_SWAP;
			fifo_regs.swap1._register = AVI_FIFO_NULL_SWAP;

			fifo_regs.timeout.pincr                 = 1;
			fifo_regs.timeout.issuing_capability    = 7;

			fifo_regs.dmafxycfg.dmafxnb = 1;
			fifo_regs.dmafxycfg.dmafynb = 1;

			avi_fifo_set_registers(n, &fifo_regs);

			if (s->dma_in_planar && i == 0)
				avi_fifo_set_registers(s->dma_in_planar, &fifo_regs);
		}
		break;
		case AVI_CONV_NODE_TYPE:
			/* Put the identity matrix in the converter */
			avi_setup_conv_colorspace(n,
			                          AVI_NULL_CSPACE,
			                          AVI_NULL_CSPACE);
		break;
		case AVI_GAM_NODE_TYPE:
			/* Bypass the gamma */
			avi_gam_setup(n, 1, 0, 0);
			break;
		case AVI_LCD_NODE_TYPE:
		{
			struct avi_lcd_regs lcd_regs;

			memset(&lcd_regs, 0, sizeof(lcd_regs));

			/* Default pixel colour. Ugly yellow. */
			lcd_regs.dpd.dpd = 0xd4e523,
			avi_lcd_set_registers(n, &lcd_regs);
		}
		break;
		case AVI_CAM_NODE_TYPE:
		{
			struct avi_cam_registers cam_regs;

			memset(&cam_regs, 0, sizeof(cam_regs));

			avi_cam_set_registers(n, &cam_regs);
		}
		break;
		case AVI_SCALROT_NODE_TYPE:
			switch(n->node_id)
			{
				case AVI_SCAL00_NODE:/* scaler */
				case AVI_SCAL10_NODE:
					/* Setup scaler in bypass by default */
					avi_scal_bypass(n);
				break;
				case AVI_ROT0_NODE:/* rotator */
				case AVI_ROT1_NODE:
					BUG();
				break;
				default:
					BUG();
				break;
			}
		break;

		case AVI_ISP_CHAIN_BAYER_NODE_TYPE:
			avi_isp_chain_bayer_bypass(n);
			break;

		case AVI_STATS_BAYER_NODE_TYPE:
			/* Nop */
			break;

		case AVI_STATS_YUV_NODE_TYPE:
			/* Nothing to be done, the module always forward the
			 * stream */
			break;

		case AVI_ISP_CHAIN_YUV_NODE_TYPE:
			avi_isp_chain_yuv_bypass(n);
			break;

		default:
			BUG();
		}
	}
}

/* Handle FIFO start address reconfiguration for interlaced video */
static inline void avi_segment_update_dma_field(struct avi_segment *s)
{
	if (s->input_format.interlaced && (s->caps & AVI_CAP_DMA_IN)) {
		struct avi_node *fifo0;
		struct avi_node *fifo1;
		dma_addr_t	 start0 = s->plane0_base;
		dma_addr_t	 start1 = s->plane1_base;

		if (s->cur_field == AVI_BOT_FIELD) {
			/* Offset the beginning of the video buffer by one line
			 * to get the bottom field. Since line_size must already
			 * contain the offset to skip one line we must divide it
			 * by 2
			 *
			 * XXX I'm not really satisfied with this method, maybe
			 * the client driver should tell us where the bottom
			 * field is? The problem of course is that if we need to
			 * use the scaler to drop the useless field we *must* be
			 * in INTERLACED mode, ALTERNATE can't work. That
			 * constrains what we can do here. */
			start0 += s->input_format.plane0.line_size / 2;
			start1 += s->input_format.plane1.line_size / 2;
		}

		fifo0 = avi_segment_get_node(s, AVI_CAP_DMA_IN);

		BUG_ON(fifo0 == NULL);

		avi_fifo_set_dmasa(fifo0, start0, start1);

		fifo1 = avi_segment_get_node(s, AVI_CAP_PLANAR);
		if (fifo1)
			avi_fifo_set_dmasa(fifo1, start0, start1);
	}
}

static void avi_segment_update_field(struct avi_segment *s, enum avi_field *field)
{
	s->cur_field = *field;

	if (*field == AVI_NONE_FIELD)
		/* Nothing left to do here */
		return;

	BUG_ON(*field != AVI_TOP_FIELD && *field != AVI_BOT_FIELD);

	/* If the segment is displayed on an odd line we have to inverse the
	 * field for interlaced streams*/
	if (s->layout.y % 2)
		s->cur_field = (*field == AVI_BOT_FIELD) ?
			AVI_TOP_FIELD : AVI_BOT_FIELD;

	if (!s->input_format.interlaced && s->output_format.interlaced) {
		/* We use the scaler to drop the useless field*/
		struct avi_node *scal = avi_segment_get_node(s, AVI_CAP_SCAL);

		BUG_ON(!scal);

		avi_scal_set_field(scal,
		                   s->input_format.height,
		                   s->output_format.height,
		                   s->cur_field);
	}

	avi_segment_update_dma_field(s);
}

/* I couldn't find a way to factorize these two function simply, so I use a
 * macro instead... */
#define AVI_SEGMENT_PROPAGATE_IRQ(_dir, _count, _member)                       \
	static void avi_segment_propagate_irq_##_dir(struct avi_segment *s,    \
	                                             enum avi_field field)     \
	{                                                                      \
		int i;                                                         \
                                                                               \
		for (i = 0; i < _count; i++) {                                 \
			struct avi_segment *next = s->_member[i];              \
                                                                               \
			if (next) {                                            \
				/* Each branch can have its own field it can   \
				 * manipulate  at will without impacting the   \
				 * other branches */                           \
				enum avi_field  nfield = field;                \
				unsigned        status = 0;                    \
                                                                               \
				avi_segment_update_field(next, &nfield);       \
									       \
				if (next->active == AVI_SEGMENT_ACTIVE_ONESHOT)\
					avi_segment_deactivate(next);          \
                                                                               \
				if (next->irq_handler_f)                       \
					status = next->irq_handler_f(next,     \
					                             &nfield); \
                                                                               \
				if (!(status & AVI_IRQ_HANDLED))               \
					avi_segment_propagate_irq_##_dir(      \
						next,                          \
						nfield);                       \
						                               \
			}                                                      \
		}                                                              \
	}

AVI_SEGMENT_PROPAGATE_IRQ(forward, AVI_SEGMENT_MAX_SINK, sink)
AVI_SEGMENT_PROPAGATE_IRQ(backward, AVI_SEGMENT_MAX_SRC, source)

static irqreturn_t avi_segment_irq_handler(int irq, void *priv)
{
	struct avi_segment      *s      = priv;
	enum avi_field           field  = AVI_NONE_FIELD;
	int                      status = 0;

	if (!avi_node_get_irq_flag(s->irq_node)) {
		printk("stray IRQ\n");
		return IRQ_NONE;
	}

	if (s->active == AVI_SEGMENT_ACTIVE_ONESHOT)
		avi_segment_deactivate(s);

	if (s->irq_handler_f)
		status = s->irq_handler_f(s, &field);

	if (!(status & AVI_IRQ_ACKED))
		avi_ack_node_irq(s->irq_node);

	if (!(status & AVI_IRQ_HANDLED)) {
		avi_segment_update_field(s, &field);

		avi_segment_propagate_irq_forward (s, field);
		avi_segment_propagate_irq_backward(s, field);
	}

	return IRQ_HANDLED;
}

static int avi_segment_setup_irq(struct avi_segment *s)
{
	BUG_ON(!s->irq_node);

	avi_disable_node_irq(s->irq_node);
	avi_ack_node_irq(s->irq_node);

	return request_irq(avi_node_irq(s->irq_node),
	                   &avi_segment_irq_handler,
	                   0,
	                   s->id, s);
}

static void avi_segment_free_irq(struct avi_segment *s)
{
	BUG_ON(!s->irq_node);

	avi_disable_node_irq(s->irq_node);
	free_irq(avi_node_irq(s->irq_node), s);
}

void avi_segment_enable_irq(struct avi_segment *s)
{
	BUG_ON(!s->irq_node);

	avi_enable_node_irq(s->irq_node);
}
EXPORT_SYMBOL(avi_segment_enable_irq);

void avi_segment_disable_irq(struct avi_segment *s)
{
	BUG_ON(!s->irq_node);

	avi_disable_node_irq(s->irq_node);
}
EXPORT_SYMBOL(avi_segment_disable_irq);

void avi_segment_register_irq(struct avi_segment *s,
                              avi_segment_irq_handler_t f)
{
	s->irq_handler_f = f;
}
EXPORT_SYMBOL(avi_segment_register_irq);

static const struct avi_segment_format avi_segment_default_format = {
	.width      = 0,
	.height     = 0,
	.colorspace = AVI_NULL_CSPACE,
	.pix_fmt    = AVI_PIXFMT_INVALID,
};

static void avi_segment_free_nodes(struct avi_segment *s)
{
	int i;

	/* Mark the nodes as unused */
	for (i = 0; i < s->nodes_nr; i++) {
		struct avi_node *n = s->nodes[i];

		n->assigned_caps = 0;
		n->busy		 = 0;
	}

	if (s->dma_in_planar) {
		s->dma_in_planar->assigned_caps = 0;
		s->dma_in_planar->busy		= 0;
	}
}


static struct avi_segment *avi_segment_do_build(unsigned long	*caps,
						const char	*id,
						struct device	*owner)
{
	struct avi_segment_index        *index = &avi_segment_index;
	struct avi_segment              *new;
	struct avi_node                 *out_node;
	int                              ret;

	/* Make sure the AVI has been succesfully probed, otherwise we will
	 * crash when we attempt to access the registers. */
	BUG_ON(!avi_probed());

	mutex_lock(&index->lock);

	if (avi_segment_find(id)) {
		dprintk("duplicate segment %s\n", id);
		ret = -EBUSY;
		goto unlock;
	}

	new = kzalloc(sizeof(*new), GFP_KERNEL);
	if (!new) {
		dprintk("can't allocate segment struct for %s", id);
		ret = -ENOMEM;
		goto unlock;
	}

	new->caps         = *caps;
	new->owner        = owner;
	new->input_format = avi_segment_default_format;
	new->layout.alpha = 0xff;
	new->cur_field    = AVI_NONE_FIELD;

	/* if the provided id is less than ARRAY_SIZE(new->id) it feels the
	 * remaining bytes with \0. If it's bigger however the buffer will not
	 * be \0 terminated. Since the buffer has a static size we could live
	 * with it but in order to save us some trouble later I force the last
	 * byte to \0. It means we effectively lose one usable byte for the
	 * id. */
	strncpy(new->id, id, ARRAY_SIZE(new->id));
	new->id[ARRAY_SIZE(new->id) - 1] = '\0';

	mutex_init(&new->lock);

	ret = avi_segment_satisfy_caps(new);
	if (ret) {
		if (ret == -ENODEV) {
#ifdef DEBUG
			char str[64];

			avi_debug_format_caps(new->caps, str, sizeof(str));

			dprintk_s(new, "can't satisfy caps: %s\n", str);
#endif /* DEBUG */

			/* Copy the unsatisfied capabilities back to caps */
			*caps = new->caps;
		} else {
			dprintk_s (new, "satisfy caps failed [%d]\n", ret);
		}
		goto free;
	}

	/* If we have a LCD, CAM or a DMA_OUT, let's setup its interrupt */
	out_node = avi_segment_get_node(new,
	                                AVI_CAPS_LCD_ALL |
	                                AVI_CAPS_CAM_ALL |
	                                AVI_CAP_DMA_OUT);
	if (out_node) {
		new->irq_node = out_node;
		ret = avi_segment_setup_irq(new);
		if (ret) {
			dprintk_s(new,
			          "can't setup interrupt for node %s\n",
			          new->irq_node->name);
			new->irq_node = NULL;
			goto free_nodes;
		}
	}

	/* Reconfigure all nodes to sane (and hopefully harmless) default values */
	avi_segment_reset(new);

	avi_segment_connect_nodes(new);

	list_add_tail(&new->list, &index->segment_list);

	mutex_unlock(&index->lock);

	return new;

free_nodes:
	avi_segment_free_nodes(new);
free:
	kfree(new);
unlock:
	mutex_unlock(&index->lock);

	return ERR_PTR(ret);
}

struct avi_segment *avi_segment_build(unsigned long	*caps,
				      const char	*type,
				      int		 major,
				      int		 minor,
				      struct device	*owner)
{
	char id[AVI_SEGMENT_ID_LEN];

	if (minor >= 0)
		snprintf(id, sizeof(id), "%s.%d:%d",
			 type, major, minor);
	else
		snprintf(id, sizeof(id), "%s.%d",
			 type, major);

	return avi_segment_do_build(caps, id, owner);
}
EXPORT_SYMBOL(avi_segment_build);

void avi_segment_gen_id(const char *base_id,
                        const char *nonce,
                        char        gen_id[AVI_SEGMENT_ID_LEN])
{
	size_t   base_len  = strlen(base_id);
	/* We need space for two more chars: the dash and the trailing \0 */
	size_t   nonce_len = strlen(nonce) + 2;
	size_t   dst_len;
	char    *p;

	/* limit the size of the nonce */
	if (nonce_len > (AVI_SEGMENT_ID_LEN / 2))
		nonce_len = (AVI_SEGMENT_ID_LEN / 2);

	dst_len = base_len + nonce_len;

	if (dst_len > AVI_SEGMENT_ID_LEN)
		dst_len = AVI_SEGMENT_ID_LEN;

	memset(gen_id, 0, AVI_SEGMENT_ID_LEN);

	p = gen_id;

	strncpy(p, base_id, AVI_SEGMENT_ID_LEN);

	p += dst_len - nonce_len;

	*p++ = '-';
	memcpy(p, nonce, nonce_len - 2);
}
EXPORT_SYMBOL(avi_segment_gen_id);

int avi_segment_connected(struct avi_segment *s)
{
	int i;

	for (i = 0; i < AVI_SEGMENT_MAX_SRC; i++)
		if (s->source[i])
			return 1;

	for (i = 0; i < AVI_SEGMENT_MAX_SINK; i++)
		if (s->sink[i])
			return 1;

	return 0;
}
EXPORT_SYMBOL(avi_segment_connected);

int avi_segment_teardown(struct avi_segment *s)
{
	int             ret = 0;

	/* Make sure the index is locked since we want to remove an element */
	mutex_lock(&avi_segment_index.lock);
	mutex_lock(&s->lock);

	if (avi_segment_connected(s)) {
		ret = -EBUSY;
		goto unlock;
	}

	avi_segment_deactivate(s);

	if (s->irq_node)
		avi_segment_free_irq(s);

	/* Reconfigure all nodes to sane (and hopefully harmless) default values */
	avi_segment_reset(s);

	spin_lock(&avi_ctrl.cfg_lck);

	avi_segment_disconnect_nodes(s);

	/* Mark the nodes as unused */
	avi_segment_free_nodes(s);

	list_del(&s->list);

	spin_unlock(&avi_ctrl.cfg_lck);

	mutex_unlock(&s->lock);
	mutex_unlock(&avi_segment_index.lock);

	kfree(s);

	return 0;

unlock:
	mutex_unlock(&s->lock);
	mutex_unlock(&avi_segment_index.lock);

	return ret;
}
EXPORT_SYMBOL(avi_segment_teardown);

int avi_take_segment(struct avi_segment *s, struct device *owner)
{
	mutex_lock(&s->lock);

	if (s->owner) {
		mutex_unlock(&s->lock);
		return -EBUSY;
	}

	s->owner = owner;

	mutex_unlock(&s->lock);

	return 0;
}
EXPORT_SYMBOL(avi_take_segment);

int avi_release_segment(struct avi_segment *s)
{
	mutex_lock(&s->lock);

	if (s->owner == NULL) {
		mutex_unlock(&s->lock);
		return -EINVAL;
	}

	/* Disown the segment and cleanup the old client context for good
	 * measure. */
	s->owner         = NULL;
	s->priv          = NULL;
	s->irq_handler_f = NULL;

	mutex_unlock(&s->lock);

	return 0;
}
EXPORT_SYMBOL(avi_release_segment);

/* This function attempt to reconnect all sources of s, adding and removing
 * blenders if necessary.
 *
 * Must be called with the segment's lock held and interrupts disabled */
static int avi_segment_reconfigure_connection(struct avi_segment *s)
{
	int			force_blender	= 0;
	int			nsources	= 0;
	int			ret		= 0;
	struct avi_node		*inter		= avi_get_node(AVI_INTER_NODE);
	unsigned		sink_id;
	int			nblenders	= 0;
	int			i;
	unsigned long		cfg_flags;
	struct avi_segment	*dense_sources[AVI_SEGMENT_MAX_SRC] = { NULL };

	BUG_ON(s->nodes_nr == 0);

	sink_id = s->nodes[0]->sink_id;

	/* count number of connected sources,
	 * compress segment sources into dense_sources */
	for (i = 0; i < AVI_SEGMENT_MAX_SRC; i++) {
		struct avi_segment *tmp;
		tmp = s->source[i];
		if (tmp != NULL) {
			dense_sources[nsources] = tmp;
			nsources ++;
		}
	}

	if ((nsources == 1) && (dense_sources[0] != NULL)) {
		/* Even if we only have one source we might need a
		 * blender if it's not fullscreen or needs to be moved
		 * around. */
		if (dense_sources[0]->layout.x != 0 ||
		    dense_sources[0]->layout.y != 0 ||
		    dense_sources[0]->layout.hidden ||
		    dense_sources[0]->layout.alpha != 0xff ||
		    dense_sources[0]->output_format.width  !=
		    s->input_format.width ||
		    dense_sources[0]->output_format.height !=
		    s->input_format.height)
			force_blender = 1;
	}

	if (nsources <= 1)
		nblenders = force_blender;
	else if (nsources <= 4)
		nblenders = 1;
	else
		nblenders = 2;

	spin_lock_irqsave(&avi_ctrl.cfg_lck, cfg_flags);

	for (i = 0; i < nblenders; i++)
		if (s->blender[i] == NULL) {
			/* We need a new blender node */
			s->blender[i] = avi_cap_get_blender();
			if (s->blender[i] == NULL) {
				/* no more blenders available! */
				ret = -ENODEV;
				break;
			}
			/* We mark a different busy state in order to
			 * distinguish freshly obtained blenders and old ones.
			 * This is used to revert to previous state if an error
			 * occured. */
			s->blender[i]->busy = 2;
		}

	if (ret) {
		/* We couldn't get the blenders we wanted, rollback and make
		 * sure we revert to the initial state. */
		while (i--)
			if (s->blender[i]->busy == 2) {
				s->blender[i]->busy = 0;
				s->blender[i] = NULL;
			}
		goto unlock;
	}
	else {
		/* All needed blenders are requested, so we no longer need to
		 * distinguish blenders */
		for (i = 0; i < nblenders; i++)
			if (s->blender[i]->busy == 2)
				s->blender[i]->busy = 1;
	}

	avi_lock_node(inter);

	if (nblenders == 0) {
		unsigned src_id = 0;

		if (dense_sources[0])
			src_id = dense_sources[0]->
				 nodes[dense_sources[0]->nodes_nr - 1]->
				 src_id;

		avi_segment_set_node_source(inter, sink_id, src_id);
	} else if (nblenders == 1) {
		/* Connect the blender */
		avi_segment_set_node_source(inter, sink_id, s->blender[0]->src_id);

		for (i = 0; i < 4; i++)
			/* The blender's input are ordered from bottom to top
			 * (input 0 is under input 1 etc...). Our zorder is the
			 * other way around so we connect them starting from 3
			 * downwards. */
			avi_segment_set_blender_source(inter,
			                               s->blender[0],
			                               3 - i,
			                               dense_sources[i]);

	} else if (nblenders == 2) {
		/* Connect the blenders */
		avi_segment_set_node_source(inter,
		                            sink_id,
		                            s->blender[0]->src_id);
		/* the 2nd blender's output is the first input of the other
		 * one */
		avi_segment_set_blender_src_id(inter,
		                               s->blender[0],
		                               0,
		                               s->blender[1]->src_id);

		/* Connect the first 3 planes on the first blender */
		for (i = 0; i < 3; i++)
			/* Same as above, we connect the planes in reverse
			 * order. */
			avi_segment_set_blender_source(inter,
			                               s->blender[0],
			                               3 - i,
			                               dense_sources[i]);

		/* Connect the last 4 planes on the 2nd blender */
		for (i = 3; i < 7; i++) {
			avi_segment_set_blender_source(inter,
			                               s->blender[1],
			                               6 - i,
			                               dense_sources[i]);
		}
	} else
		BUG();

	for (i = 0; i < nblenders; i++) {
		avi_setup_blend(s->blender[i],
		                s->background,
		                s->input_format.width,
		                s->input_format.height);

		avi_enable_node(s->blender[i]);
	}

	/* Free the no-longer needed blenders (if any) */
	for (i = nblenders; i < 2; i++)
		if (s->blender[i]) {
			int j;

			for (j = 0; j < 4; j++)
				avi_segment_set_blender_src_id(inter,
				                               s->blender[i],
				                               j,
				                               0);

			avi_disable_node(s->blender[i]);
			s->blender[i]->busy = 0;
			s->blender[i]->assigned_caps = 0;
			s->blender[i] = NULL;
		}

	/* Reconfiguration done! */
	avi_unlock_node(inter);

 unlock:
	spin_unlock_irqrestore(&avi_ctrl.cfg_lck, cfg_flags);
	return ret;
}

int avi_segment_connect(struct avi_segment *src,
                        struct avi_segment *dst,
                        int                 zorder)
{
	int		ret;

	mutex_lock(&src->lock);
	mutex_lock(&dst->lock);

	if (src->sink[0]) {
		/* Forks are not yet supported */
		ret = -EBUSY;
		goto unlock;
	}

	if (zorder == -1) {
		/* Look for an available zorder */
		for (zorder = 0; zorder < AVI_SEGMENT_MAX_SRC; zorder++)
			if (dst->source[zorder] == NULL)
				/* We found an available source */
				break;

		if (zorder >= AVI_SEGMENT_MAX_SRC) {
			/* No available sources remain */
			ret = -EBUSY;
			goto unlock;
		}

	} else if (dst->source[zorder] != NULL) {
		/* The requested zorder is already in use */
		ret = -EBUSY;
		goto unlock;
	}

	dst->source[zorder] = src;

	ret = avi_segment_reconfigure_connection(dst);

	if (ret) {
		/* reconfiguration failed, rollback */
		dst->source[zorder] = NULL;
		goto unlock;
	}

	src->sink[0] = dst;

	ret = 0;
 unlock:
	mutex_unlock(&dst->lock);
	mutex_unlock(&src->lock);

	return ret;
}
EXPORT_SYMBOL(avi_segment_connect);

int avi_segment_disconnect(struct avi_segment *src,
                           struct avi_segment *dst)
{
	int	ret;
	int	zorder;

	mutex_lock(&src->lock);
	mutex_lock(&dst->lock);

	for (zorder = 0; zorder < AVI_SEGMENT_MAX_SRC; zorder++)
		if (dst->source[zorder] == src)
			break;

	if (zorder >= AVI_SEGMENT_MAX_SRC) {
		ret = -ENODEV;
		goto unlock;
	}

	dst->source[zorder] = NULL;

	ret = avi_segment_reconfigure_connection(dst);
	if (ret) {
		/* reconfiguration failed, rollback */
		dst->source[zorder] = src;
		goto unlock;
	}

	/* XXX no fork support */
	src->sink[0] = NULL;
	ret = 0;
 unlock:
	mutex_unlock(&dst->lock);
	mutex_unlock(&src->lock);

	return ret;
}
EXPORT_SYMBOL(avi_segment_disconnect);

void avi_segment_set_background(struct avi_segment *s, u32 rgb)
{
	/* Convert the rgb color to the destination colorspace */
	s->background = avi_conv_convert_color(AVI_RGB_CSPACE,
					       s->output_format.colorspace,
					       rgb);

	if (s->blender[0])
		avi_blend_set_bg(s->blender[0], s->background);

	if (s->blender[1])
		avi_blend_set_bg(s->blender[1], s->background);
}
EXPORT_SYMBOL(avi_segment_set_background);

u32 avi_segment_get_background(struct avi_segment *s)
{
	/* Convert the rgb color to the destination colorspace */
	return avi_conv_convert_color(s->output_format.colorspace,
				      AVI_RGB_CSPACE,
				      s->background);
}
EXPORT_SYMBOL(avi_segment_get_background);

static int avi_segment_do_activate(struct avi_segment *s, bool oneshot)
{

	struct avi_node		*out_node;
	unsigned		i;

	if (s->active != AVI_SEGMENT_DISABLED)
		return -EBUSY;

	for (i = 0; i < s->nodes_nr; i++) {
		if (s->nodes[i]->type == AVI_FIFO_NODE_TYPE)
			avi_fifo_set_oneshot(s->nodes[i], oneshot);
		avi_enable_node(s->nodes[i]);
	}

	if (s->dma_in_planar) {
		avi_fifo_set_oneshot(s->dma_in_planar, oneshot);
		avi_enable_node(s->dma_in_planar);
	}

	out_node = avi_segment_get_node(s, AVI_CAPS_LCD_ALL | AVI_CAP_DMA_OUT);

	if (out_node)
		avi_apply_node(out_node);

#ifdef AVI_BACKWARD_COMPAT
	if ((avi_get_revision() == AVI_REVISION_1) && oneshot) {
		/* Pre-disable FIFO nodes to stop processing after one shot */
		for (i = 0; i < s->nodes_nr; i++) {
			if (s->nodes[i]->type == AVI_FIFO_NODE_TYPE)
				avi_disable_node(s->nodes[i]);
		}
		if (s->dma_in_planar) {
			avi_disable_node(s->dma_in_planar);
		}
	}
#endif /* AVI_BACKWARD_COMPAT */

	s->active = oneshot ? AVI_SEGMENT_ACTIVE_ONESHOT : AVI_SEGMENT_ACTIVE;

	return 0;
}

int avi_segment_activate(struct avi_segment *s) {
	return avi_segment_do_activate(s, 0);
}
EXPORT_SYMBOL(avi_segment_activate);

int avi_segment_activate_oneshot(struct avi_segment *s) {
	return avi_segment_do_activate(s, 1);
}
EXPORT_SYMBOL(avi_segment_activate_oneshot);

int avi_segment_deactivate(struct avi_segment *s)
{
	unsigned		i;

	if (s->active == AVI_SEGMENT_DISABLED)
		return -EINVAL;

	for (i = 0; i < s->nodes_nr; i++)
		avi_disable_node(s->nodes[i]);

	if (s->dma_in_planar)
		avi_disable_node(s->dma_in_planar);

	s->active = AVI_SEGMENT_DISABLED;

	return 0;
}

EXPORT_SYMBOL(avi_segment_deactivate);

/* Called with the segment's lock held */
static int avi_segment_check_transform(struct avi_segment              *s,
                                       const struct avi_segment_format *in,
                                       const struct avi_segment_format *out)
{
	dprintk_s(s,
	          "check_transform: %ux%u%c %s [%s %u %u] ->"
	          " %ux%u%c %s [%s %u %u]\n",
	          in->width, in->height, in->interlaced ? 'i' : 'p',
	          avi_debug_colorspace_to_str(in->colorspace),
	          avi_debug_pixfmt_to_str(in->pix_fmt),
	          in->plane0.line_size,
	          in->plane1.line_size,

	          out->width, out->height, out->interlaced ? 'i' : 'p',
	          avi_debug_colorspace_to_str(out->colorspace),
	          avi_debug_pixfmt_to_str(out->pix_fmt),
	          out->plane0.line_size,
	          out->plane1.line_size);

#define INVALID_TRANSFORM(msg_) do {                              \
		        dprintk_s(s, "bad transform: %s\n",  msg_); \
		        return -EINVAL;                           \
	        } while (0)

	if (in->width  == 0 || out->width  == 0 ||
	    in->height == 0 || out->height == 0)
		INVALID_TRANSFORM("null dimension in transform");

	if ((in->width != out->width || in->height != out->height)
	    && !(s->caps & AVI_CAP_SCAL))
		INVALID_TRANSFORM("can't rescale without scaler");

	if (in->interlaced && !out->interlaced)
		INVALID_TRANSFORM("can't deinterlace");

	if (!in->interlaced && out->interlaced && !(s->caps & AVI_CAP_SCAL))
		INVALID_TRANSFORM("can't drop field without scaler");

	/* For RAW input, need to check that the whole ISP chain is
	 * present and the dimensions */
	if (in->pix_fmt.raw) {
		if (!out->pix_fmt.raw &&
		    (s->caps & AVI_CAPS_ISP) != AVI_CAPS_ISP)
			INVALID_TRANSFORM("can't do ISP conversion");

		if (in->width % 2)
			INVALID_TRANSFORM("bad width for RAW colorspace");

		if (in->height % 2)
			INVALID_TRANSFORM("bad height for RAW colorspace");
	}

	if (in->colorspace != out->colorspace && !(s->caps & AVI_CAP_CONV))
		INVALID_TRANSFORM("can't do chroma conversion");

	if (in->pix_fmt.id != AVI_INVALID_FMTID && !(s->caps & AVI_CAP_DMA_IN))
		INVALID_TRANSFORM("pixel_format without DMA_IN");

	if (in->pix_fmt.id != AVI_INVALID_FMTID && in->plane0.line_size == 0)
		INVALID_TRANSFORM("bad line_size for DMA_IN");

	if (avi_pixfmt_get_packing(in->pix_fmt) != AVI_INTERLEAVED_444_PACKING
	    && (in->width % 2))
		INVALID_TRANSFORM("bad width for DMA_IN");

	if (avi_pixfmt_get_packing(in->pix_fmt) == AVI_SEMIPLANAR_YUV_420_PACKING
	    && (in->height % 2))
		INVALID_TRANSFORM("bad height for DMA_IN");

	if (out->pix_fmt.id != AVI_INVALID_FMTID &&
	    !(s->caps & AVI_CAP_DMA_OUT))
		INVALID_TRANSFORM("pixel_format without DMA_OUT");

	if (out->pix_fmt.id != AVI_INVALID_FMTID && out->plane0.line_size == 0)
		INVALID_TRANSFORM("bad line_size for DMA_OUT");

	if (avi_pixfmt_get_packing(out->pix_fmt) != AVI_INTERLEAVED_444_PACKING
	    && (out->width % 2))
		INVALID_TRANSFORM("bad width for DMA_OUT");

	if (avi_pixfmt_get_packing(out->pix_fmt) == AVI_SEMIPLANAR_YUV_420_PACKING
	    && (out->height % 2))
		INVALID_TRANSFORM("bad height for DMA_OUT");

	if ((avi_get_revision() < AVI_REVISION_3))
		if (avi_pixfmt_is_planar(in->pix_fmt) &&
			!(s->caps & AVI_CAP_PLANAR))
			INVALID_TRANSFORM("can't handle planar without CAP_PLANAR");

#undef INVALID_TRANSFORM

	/* All is good */
	return 0;
}

int avi_segment_try_format(struct avi_segment              *s,
                           const struct avi_segment_format *in,
                           const struct avi_segment_format *out,
                           const struct avi_segment_layout *layout)
{
	unsigned	out_width;
	unsigned	out_height;
	int		ret;
	int		i;

	ret = avi_segment_check_transform(s, in, out);
	if (ret)
		return ret;

	/* How big the sinks will have to be to accomodate the current
	 * format. */
	out_width  = layout->x + out->width;
	out_height = layout->y + out->height;

	for (i = 0; i < AVI_SEGMENT_MAX_SINK; i++) {
		struct avi_segment *sink = s->sink[i];

		if (sink &&
		    (sink->input_format.width  < out_width ||
		     sink->input_format.height < out_height)) {
			dprintk_s(s, "segment position is out "
			          "of the sink window: %ux%u @%u,%u -> %ux%u\n",
			          out->width, out->height,
			          layout->x, layout->y,
			          sink->input_format.width,
			          sink->input_format.height);
			return -ERANGE;
		}
	}

	return 0;
}
EXPORT_SYMBOL(avi_segment_try_format);

/* Must be called with interrupts disabled */
static inline int avi_segment_reconfigure_sinks(struct avi_segment *s)
{
	/* XXX no fork support */
	struct avi_segment *sink = s->sink[0];

	if (sink)
		return avi_segment_reconfigure_connection(s->sink[0]);

	return 0;
}

/* Configure the segment's node to match the current input and output
 * formats. This function cannot fail as avi_segment_try_format must have been
 * called beforehand to validate */
static int avi_segment_apply_format(struct avi_segment *s)
{
	const struct avi_segment_format *in  = &s->input_format;
	const struct avi_segment_format *out = &s->output_format;
	struct avi_node			*n;
	unsigned			 in_height;
	unsigned			 out_height;
	int				 ret;

	in_height  = in->height;
	out_height = out->height;

	if (in->interlaced)
		in_height /= 2;

	if (out->interlaced)
		out_height /= 2;

	ret = avi_segment_reconfigure_sinks(s);
	if (ret)
		return ret;

	if ((n = avi_segment_get_node(s, AVI_CAP_SCAL))) {

		/* We need to configure the scaler in planar if we have a planar
		 * pixel format and we're using two FIFOs */
		avi_scal_setup_oneshot(n,
		                       in->width, out->width,
		                       in_height, out_height,
		                       in->pix_fmt,
		                       (s->dma_in_planar != NULL));
	}

	/* configure only chroma converter if ISP CHAIN BAYER is not present
	 * in this case chroma converter is configured in userland
	 */
	if (!avi_segment_get_node(s, AVI_CAP_ISP_CHAIN_BAYER))
		if ((n = avi_segment_get_node(s, AVI_CAP_CONV)))
			avi_setup_conv_colorspace(n,
						  in->colorspace,
						  out->colorspace);

	if ((n = avi_segment_get_node(s, AVI_CAP_DMA_IN))) {
		struct avi_node	*planar_fifo;

		planar_fifo = avi_segment_get_node(s, AVI_CAP_PLANAR);

		avi_dma_setup_input(n, planar_fifo, in);
	}

	if ((n = avi_segment_get_node(s, AVI_CAP_BUFFER))) {
		struct avi_fifo_registers fifo_reg = {
			.cfg = {{
					.srctype   = AVI_FIFO_CFG_VL_TYPE,
					.dsttype   = AVI_FIFO_CFG_VL_TYPE,
					.srcbitenc = AVI_FIFO_CFG_8888_BENC,
					.dstbitenc = AVI_FIFO_CFG_8888_BENC,
				}},
			.framesize = {{
					.width  = out->width - 1,
					.height = out_height - 1,
				}},
			.dmafxycfg = {{
					.dmafxnb = 1,
					.dmafynb = 1,
					.dmafxymode = 0,
				}},
		};
		avi_fifo_set_registers(n, &fifo_reg);
	}

	if ((n = avi_segment_get_node(s, AVI_CAP_DMA_OUT)))
		avi_dma_setup_output(n, out);

	avi_segment_reconfigure_connection(s);

	return 0;
}

int avi_segment_set_format_and_layout(struct avi_segment              *s,
                                      const struct avi_segment_format *in_fmt,
                                      const struct avi_segment_format *out_fmt,
                                      const struct avi_segment_layout *layout)
{
	struct avi_segment_format	old_in_fmt;
	struct avi_segment_format	old_out_fmt;
	struct avi_segment_layout	old_layout;
	int				ret;

	ret = avi_segment_try_format(s, in_fmt, out_fmt, layout);
	if (ret)
		return ret;

	/* Roll the new configuration in */
	old_in_fmt	= s->input_format;
	old_out_fmt	= s->output_format;
	old_layout	= s->layout;

	s->input_format		= *in_fmt;
	s->output_format	= *out_fmt;
	s->layout		= *layout;

	ret = avi_segment_apply_format(s);
	if (ret) {
		/* Rollback */
		s->input_format  = old_in_fmt;
		s->output_format = old_out_fmt;
		s->layout        = old_layout;
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(avi_segment_set_format_and_layout);

int avi_segment_set_format(struct avi_segment              *s,
                           const struct avi_segment_format *in_fmt,
                           const struct avi_segment_format *out_fmt)
{
	return avi_segment_set_format_and_layout(s,
	                                         in_fmt,
	                                         out_fmt,
	                                         &s->layout);
}
EXPORT_SYMBOL(avi_segment_set_format);

int avi_segment_set_input_format(struct avi_segment              *s,
                                 const struct avi_segment_format *fmt)
{
	return avi_segment_set_format(s, fmt, &s->output_format);
}
EXPORT_SYMBOL(avi_segment_set_input_format);

int avi_segment_set_output_format(struct avi_segment *s,
                                  const struct avi_segment_format *fmt)
{
	return avi_segment_set_format(s, &s->input_format, fmt);
}
EXPORT_SYMBOL(avi_segment_set_output_format);

int avi_segment_hide(struct avi_segment *s)
{
	int		ret = 0;

	if (!s->layout.hidden) {
		s->layout.hidden = 1;

		ret = avi_segment_reconfigure_sinks(s);

		if (ret)
			s->layout.hidden = 0;
	}

	return ret;
}
EXPORT_SYMBOL(avi_segment_hide);

int avi_segment_unhide(struct avi_segment *s)
{
	int		ret = 0;

	if (s->layout.hidden) {
		s->layout.hidden = 0;

		ret = avi_segment_reconfigure_sinks(s);

		if (ret)
			s->layout.hidden = 1;
	}

	return ret;
}
EXPORT_SYMBOL(avi_segment_unhide);

inline int avi_segment_set_position(struct avi_segment *s,
                                    unsigned x,
                                    unsigned y)
{
	struct avi_segment		*sink = s->sink[0];
	struct avi_segment_layout	 layout;
	int				 ret  = 0;

	if (s->layout.x == x && s->layout.y == y)
		/* Nothing to do */
		return 0;

	layout = s->layout;

	layout.x = x;
	layout.y = y;

	ret = avi_segment_try_format(s,
	                             &s->input_format,
	                             &s->output_format,
	                             &layout);
	if (ret)
		return ret;

	s->layout = layout;

	if (sink)
		return avi_segment_reconfigure_connection(sink);

	return 0;
}
EXPORT_SYMBOL(avi_segment_set_position);

inline int avi_segment_set_alpha(struct avi_segment *s, int alpha)
{
	int ret = 0;

	if (s->layout.alpha != alpha) {
		struct avi_segment *sink = s->sink[0];

		s->layout.alpha = alpha;
		if (sink)
			ret = avi_segment_reconfigure_connection(sink);
	}

	return ret;
}
EXPORT_SYMBOL(avi_segment_set_alpha);

void avi_segment_get_input_format(struct avi_segment *s,
                                  struct avi_segment_format *fmt)
{
	*fmt = s->input_format;
}
EXPORT_SYMBOL(avi_segment_get_input_format);

void avi_segment_get_output_format(struct avi_segment *s,
                                   struct avi_segment_format *fmt)
{
	*fmt = s->output_format;
}
EXPORT_SYMBOL(avi_segment_get_output_format);

void avi_segment_get_layout(struct avi_segment		*s,
                            struct avi_segment_layout	*layout)
{
	*layout = s->layout;
}
EXPORT_SYMBOL(avi_segment_get_layout);

struct avi_node *avi_segment_get_node(struct avi_segment *s,
				      const unsigned long cap)
{
	int     pos;

	if ((s->caps & cap)) {
		for (pos = 0; pos < s->nodes_nr; pos++)
			if ((s->nodes[pos]->assigned_caps & cap))
				return s->nodes[pos];

		if (s->dma_in_planar && (s->dma_in_planar->assigned_caps & cap))
			return s->dma_in_planar;
	}

	/* Not found */
	return NULL;
}
EXPORT_SYMBOL(avi_segment_get_node);

void avi_segment_set_input_buffer(struct avi_segment *segment,
                                  const struct avi_dma_buffer *buff)
{
	struct avi_node			*fifo_plane0;
	struct avi_node			*fifo_plane1;
	const struct avi_segment_format	*in = &segment->input_format;

	fifo_plane0 = avi_segment_get_node(segment, AVI_CAP_DMA_IN);

	BUG_ON(fifo_plane0 == NULL);
	BUG_ON(buff->status != AVI_BUFFER_READY);
	BUG_ON(buff->plane0.dma_addr == 0);
	BUG_ON(avi_pixfmt_is_planar(in->pix_fmt) &&
	       buff->plane1.dma_addr == 0);

	segment->plane0_base = buff->plane0.dma_addr;
	segment->plane1_base = buff->plane1.dma_addr;

	fifo_plane1 = avi_segment_get_node(segment, AVI_CAP_PLANAR);
	avi_dma_set_input_buffer(fifo_plane0, fifo_plane1, in, buff);

	avi_segment_update_dma_field(segment);
}
EXPORT_SYMBOL(avi_segment_set_input_buffer);

void avi_segment_set_output_buffer(struct avi_segment *segment,
                                   const struct avi_dma_buffer *buff)
{
	struct avi_node			*fifo;
	const struct avi_segment_format	*out = &segment->output_format;

	fifo = avi_segment_get_node(segment, AVI_CAP_DMA_OUT);

	BUG_ON(fifo == NULL);
	BUG_ON(buff->status != AVI_BUFFER_READY);

	avi_dma_set_output_buffer(fifo, out, buff);
}
EXPORT_SYMBOL(avi_segment_set_output_buffer);

int avi_segment_foreach(int (*cback)(struct avi_segment *, void *), void *priv)
{
	struct avi_segment      *s;
	int                      ret = 0;

	mutex_lock(&avi_segment_index.lock);

	list_for_each_entry(s, &avi_segment_index.segment_list, list) {
		ret = cback(s, priv);

		if (ret)
			break;
	}

	mutex_unlock(&avi_segment_index.lock);

	return ret;
}
EXPORT_SYMBOL(avi_segment_foreach);

unsigned avi_segment_count(void)
{
	struct avi_segment      *s;
	unsigned		 count = 0;

	mutex_lock(&avi_segment_index.lock);

	list_for_each_entry(s, &avi_segment_index.segment_list, list) {
		count++;
	}

	mutex_unlock(&avi_segment_index.lock);

	return count;
}
EXPORT_SYMBOL(avi_segment_count);

static int avi_segment_reconfigure(struct avi_segment *s, void *unused)
{
       avi_segment_reset(s);
       avi_segment_connect_nodes(s);
       BUG_ON(avi_segment_apply_format(s));
       BUG_ON(avi_segment_reconfigure_connection(s));

       /* It's up to each individual driver (fb, cam...) to actually reactivate
        * the segments (enable, apply and IRQ) as well as reconfiguring the
        * DMA. */
       s->active = AVI_SEGMENT_DISABLED;

       return 0;
}

void avi_segment_resume(void)
{
       /* Iterate over each segment, reconfiguring and reconnecting
        * everything. */
       avi_segment_foreach(avi_segment_reconfigure, NULL);
}

MODULE_AUTHOR("Lionel Flandrin <lionel.flandrin@parrot.com>");
MODULE_DESCRIPTION("Parrot Advanced Video Interface Segment layer");
MODULE_LICENSE("GPL");
