#include <linux/module.h>
#include "avi_debug.h"

/* Look for entry v_ in string array arr_. If it's not found return string
 * not_found_ */
#define FIND_IN_ARRAY(arr_, v_, not_found_) do {                          \
		if ((v_) >= ARRAY_SIZE(arr_) || (arr_)[(v_)] == NULL)     \
			return not_found_;                                \
		return (arr_)[(v_)];                                      \
	} while(0)

static const char *avi_debug_colorspaces[] =
{
#define AVI_DEBUG_CSPACE_TO_STR(_c) [AVI_ ## _c ## _CSPACE] = # _c
	AVI_DEBUG_CSPACE_TO_STR(NULL),
	AVI_DEBUG_CSPACE_TO_STR(RGB),
	AVI_DEBUG_CSPACE_TO_STR(BT601),
	AVI_DEBUG_CSPACE_TO_STR(BT709),
	AVI_DEBUG_CSPACE_TO_STR(JFIF),
	AVI_DEBUG_CSPACE_TO_STR(GREY),
	AVI_DEBUG_CSPACE_TO_STR(Y10),
#undef AVI_DEBUG_CSPACE_TO_STR
};

const char *avi_debug_colorspace_to_str(enum avi_colorspace cspace)
{
	FIND_IN_ARRAY(avi_debug_colorspaces, cspace, "INVALID_COLORSPACE");
}
EXPORT_SYMBOL(avi_debug_colorspace_to_str);

#define AVI_PIXEL_FORMAT_AS_STR(_x) [AVI_PIXEL_FORMAT_AS_ENUM(_x)] = # _x
static const char *avi_debug_pixfmts[] =
{
	AVI_PIXEL_FORMATS(AVI_PIXEL_FORMAT_AS_STR),
};

const char *avi_debug_pixfmt_to_str(struct avi_dma_pixfmt pixfmt)
{
	FIND_IN_ARRAY(avi_debug_pixfmts, pixfmt.id, "INVALID_PIXFMT");
}
EXPORT_SYMBOL(avi_debug_pixfmt_to_str);

static const char *avi_debug_active_modes[] =
{
#define AVI_DEBUG_ACTIVE_TO_STR(_p) [AVI_SEGMENT_ ## _p] = # _p
	AVI_DEBUG_ACTIVE_TO_STR(DISABLED),
	AVI_DEBUG_ACTIVE_TO_STR(ACTIVE),
	AVI_DEBUG_ACTIVE_TO_STR(ACTIVE_ONESHOT),
#undef AVI_DEBUG_ACTIVE_TO_STR
};

const char *avi_debug_activation_to_str(enum avi_segment_activation active)
{
	FIND_IN_ARRAY(avi_debug_active_modes, active, "INVALID_ACTIVE");
}
EXPORT_SYMBOL(avi_debug_activation_to_str);

static const char *avi_debug_force_clear[] =
{
#define AVI_DEBUG_CLEAR_TO_STR(_p) [AVI_LCD_FORCE_CLEAR_ ## _p] = # _p
	AVI_DEBUG_CLEAR_TO_STR(NORMAL),
	AVI_DEBUG_CLEAR_TO_STR(ACTIVE),
	AVI_DEBUG_CLEAR_TO_STR(INACTIVE),
#undef AVI_DEBUG_ACTIVE_TO_STR
};

const char *avi_debug_force_clear_to_str(unsigned clear)
{
	FIND_IN_ARRAY(avi_debug_force_clear, clear, "INVALID_CLEAR");
}
EXPORT_SYMBOL(avi_debug_force_clear_to_str);

static const struct {
	unsigned long	 cap;
	const char	*str;
} avi_debug_caps[] = {
#define MAKE_CAP_STR(c_) { .cap = AVI_CAP_ ## c_, .str = # c_ }
 	MAKE_CAP_STR(DMA_IN),
	MAKE_CAP_STR(DMA_OUT),
	MAKE_CAP_STR(BUFFER),
	MAKE_CAP_STR(CONV),
	MAKE_CAP_STR(GAM),
	MAKE_CAP_STR(SCAL),
	MAKE_CAP_STR(ROT),
	MAKE_CAP_STR(PLANAR),
	MAKE_CAP_STR(ISP_CHAIN_BAYER),
	MAKE_CAP_STR(STATS_BAYER_0),
	MAKE_CAP_STR(STATS_BAYER_1),
	MAKE_CAP_STR(STATS_YUV),
	MAKE_CAP_STR(ISP_CHAIN_YUV),
	MAKE_CAP_STR(CAM_0),
	MAKE_CAP_STR(CAM_1),
	MAKE_CAP_STR(CAM_2),
	MAKE_CAP_STR(CAM_3),
	MAKE_CAP_STR(CAM_4),
	MAKE_CAP_STR(CAM_5),
	MAKE_CAP_STR(LCD_0),
	MAKE_CAP_STR(LCD_1),
#undef MAKE_CAP_STR
};

const char *avi_debug_caps_to_str(unsigned long *caps)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(avi_debug_caps); i++) {
		if (*caps & avi_debug_caps[i].cap) {
			/* Found a capability to return  */
			*caps &= ~(avi_debug_caps[i].cap);
			return avi_debug_caps[i].str;
		}
	}

	if (*caps) {
		/* caps is not empty but nothing matches in our translation
		 * table, return an error */
		*caps = 0;
		return "<UNKNOWN_CAPS>";
	}

	/* No caps remain to convert */
	return NULL;
}
EXPORT_SYMBOL(avi_debug_caps_to_str);

/* The pleasure of string handling in C... */
static inline int avi_debug_safe_strcpy(char **dst, size_t *s, const char *src)
{
	size_t l = strlen(src);

	if (*s < l + 1)
		/* Not enough space left */
		return 1;

	strcpy(*dst, src);

	*dst += l;
	*s   -= l;

	return 0;
}

char *avi_debug_format_caps(unsigned long caps, char *buf, size_t buf_size)
{
	const char	*str;
	char		*p = buf;
	size_t		 s = buf_size;
	int		 n = 0;

	while ((str = avi_debug_caps_to_str(&caps))) {
		if (n++ > 0)
			if (avi_debug_safe_strcpy(&p, &s, " | "))
				goto unfinished;

		if (avi_debug_safe_strcpy(&p, &s, str))
			goto unfinished;
	}

	return buf;

 unfinished:
	if (buf_size == 0)
		return NULL;

	if (buf_size >= 4) {
		if (buf_size - s < 4)
			s = buf_size - 4;

		strcpy(p - s, "...");
	} else {
		/* Not enough for the dots */
		*buf = '\0';
	}

	return buf;
}
EXPORT_SYMBOL(avi_debug_format_caps);
