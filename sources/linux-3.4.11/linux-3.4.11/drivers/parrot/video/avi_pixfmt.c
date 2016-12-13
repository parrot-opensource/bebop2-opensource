#include <linux/module.h>

#include "avi_pixfmt.h"

#define AVI_PIXEL_FORMAT_AS_LUT(_x) \
	[AVI_PIXEL_FORMAT_AS_ENUM(_x)] = AVI_PIXFMT_ ## _x
static const struct avi_dma_pixfmt avi_pixfmt_lut[AVI_PIXFMT_NR] = {
	AVI_PIXEL_FORMATS(AVI_PIXEL_FORMAT_AS_LUT),
};

const struct avi_dma_pixfmt avi_pixfmt_by_id(enum avi_dma_pixfmt_ids id)
{
	if (id >= AVI_PIXFMT_NR)
		id = AVI_INVALID_FMTID;

	return avi_pixfmt_lut[id];
}
EXPORT_SYMBOL(avi_pixfmt_by_id);

int avi_pixfmt_have_alpha(struct avi_dma_pixfmt pixfmt)
{
	enum avi_dma_pixfmt_ids id = pixfmt.id;
	return ((id == AVI_RGBA8888_FMTID) ||
		(id == AVI_BGRA8888_FMTID) ||
		(id == AVI_ARGB8888_FMTID) ||
		(id == AVI_ABGR8888_FMTID) ||
		(id == AVI_RGBA5551_FMTID) ||
		(id == AVI_AYUV_FMTID) );
}
EXPORT_SYMBOL(avi_pixfmt_have_alpha);
