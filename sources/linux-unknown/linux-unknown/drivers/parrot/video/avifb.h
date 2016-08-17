#ifndef _AVIFB_H_
#define _AVIFB_H_

#include <linux/fb.h>
#include <linux/ioctl.h>
#include "avi_segment.h"

struct avifb_layout {
	unsigned x;
	unsigned y;
	unsigned width;
	unsigned height;
	int      alpha;
	int      enabled;
};

struct avifb_overlay {
	/* Default layout on startup */
	struct avifb_layout	 layout;
	struct resource		 dma_memory;
	unsigned long		 caps;
	int			 zorder;
	unsigned		 color_depth;
	/* If cacheable is != 0 the framebuffer will be mapped in device
	 * cacheable in both kernel and userland. */
	int			 cacheable;
};

struct avifb_platform_data {
	/* Output format control as defined in reg_avi.h */
	unsigned			  lcd_format_control;
	const struct avi_videomode	 *lcd_default_videomode;
	const struct avi_videomode	**lcd_videomodes;
	union avi_lcd_interface		  lcd_interface;
	unsigned long			  caps;
	/* blender background colour. 24bit RGB no alpha */
	u32				  background;
	/* Output colorspace conversion. Only possible if we have a chroma
	 * converter in our channel. Input colorspace is always
	 * AVI_RGB_CSPACE. */
	enum avi_colorspace		  output_cspace;
	/* Enable itu656 mode where video synchronisation codes are embedded in
	 * the data stream (no need for external HS/VS/DE */
	struct avifb_overlay		 *overlays;
	unsigned			  overlay_nr;
	/* Default pixel data (color) for the LCD */
	u32                               dpd;

	/* panel size in mm */
	__u32                             height;
	__u32                             width;
};

extern struct fb_videomode *avifb_select_mode(const char         *id,
					      struct fb_monspecs *monspecs);

extern int avifb_set_screen_size(const char* id, unsigned width, unsigned height);

/** avifb_set_monspecs() - Set monitor specs for specific LCD
  * @monspecs:	monitor specs
  *
  * Copy all specs of fb_monspecs to an internal fb_monspecs except the
  * videomode databse (modedb) which stay empty.
  */
extern void avifb_set_monspecs(const char *id, struct fb_monspecs *monspecs);

#endif /* _AVIFB_H_ */
