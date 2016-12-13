#include <linux/module.h>
#include <linux/uaccess.h>
#include "avi_segment.h"
#include "avi_isp.h"

/* Bypass all ISP modules */
void avi_isp_chain_bayer_bypass(struct avi_node *node)
{
	struct avi_isp_chain_bayer_inter_regs inter_regs = {
		.module_bypass = {
			.pedestal_bypass     = 1,
			.grim_bypass         = 1,
			.rip_bypass          = 1,
			.denoise_bypass      = 1,
			.lsc_bypass          = 1,
			.chroma_aber_bypass  = 1,
			.bayer_bypass        = 1,
			.color_matrix_bypass = 1,
		},
	};

	avi_isp_chain_bayer_inter_set_registers(node, &inter_regs);
}
EXPORT_SYMBOL(avi_isp_chain_bayer_bypass);

void avi_isp_chain_yuv_bypass(struct avi_node *node)
{
	struct avi_isp_chain_yuv_inter_regs inter_regs = {
		.module_bypass = {
			.ee_crf_bypass  = 1,
			.i3d_lut_bypass = 1,
			.drop_bypass    = 1,
		},
	};

	avi_isp_chain_yuv_inter_set_registers(node, &inter_regs);
}
EXPORT_SYMBOL(avi_isp_chain_yuv_bypass);

void avi_statistics_bayer_configure(struct avi_node	*node,
				    unsigned		 width,
				    unsigned		 height,
				    unsigned		 thumb_width,
				    unsigned		 thumb_height)
{
	struct avi_isp_statistics_bayer_regs stats_cfg;

	avi_isp_statistics_bayer_get_registers(node, &stats_cfg);

	/* Update only window settings */
	stats_cfg.max_nb_windows.x_window_count = thumb_width;
	stats_cfg.max_nb_windows.y_window_count = thumb_height;

	/* All the stats config has to be even */
	stats_cfg.window_x.x_width = (width  / thumb_width) & (~1UL);
	stats_cfg.window_y.y_width = (height / thumb_height) & (~1UL);

	/* Center the grid */
	stats_cfg.window_x.x_offset = ((width  % thumb_width) / 2) & (~1UL);
	stats_cfg.window_y.y_offset = ((height % thumb_height) / 2) & (~1UL);

	avi_isp_statistics_bayer_set_registers(node, &stats_cfg);
}
EXPORT_SYMBOL(avi_statistics_bayer_configure);

/* Get all ISP modules offset */
int avi_isp_get_offsets(struct avi_segment *s, struct avi_isp_offsets *offsets)
{
       struct avi_node		*n;

       if (s == NULL)
               return -ENODEV;

#define GET_BASE(_off, _cap) do {                      \
               n = avi_segment_get_node(s, _cap);      \
               if (n == NULL)                          \
                       return -ENODEV;                 \
               _off = n->base_off;                     \
       } while(0)

       GET_BASE(offsets->chain_bayer,     AVI_CAP_ISP_CHAIN_BAYER);
       GET_BASE(offsets->gamma_corrector, AVI_CAP_GAM);
       GET_BASE(offsets->chroma,          AVI_CAP_CONV);
       GET_BASE(offsets->statistics_yuv,  AVI_CAP_STATS_YUV);
       GET_BASE(offsets->chain_yuv,       AVI_CAP_ISP_CHAIN_YUV);

#undef GET_BASE

       return 0;
}
EXPORT_SYMBOL(avi_isp_get_offsets);

unsigned long avi_isp_bayer_get_stats_cap(struct avi_segment *s)
{
	struct avi_node *bayer;

	bayer = avi_segment_get_node(s, AVI_CAP_ISP_CHAIN_BAYER);

	if (bayer == NULL)
		return 0;

	switch (bayer->node_id) {
	case AVI_ISP_CHAIN_BAYER0_NODE:
		return AVI_CAP_STATS_BAYER_0;
	case AVI_ISP_CHAIN_BAYER1_NODE:
		return AVI_CAP_STATS_BAYER_1;
	default:
		BUG();
	}
}
EXPORT_SYMBOL(avi_isp_bayer_get_stats_cap);
