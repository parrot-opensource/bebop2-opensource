/*
 *  Parrot AVI framebuffer driver.
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/pinctrl/consumer.h>
#include <asm/cacheflush.h>
#include "avi_scaler.h"
#include "avi_isp.h"
#include "avi_limit.h"
#include "avifb.h"

/* maximum possimble number of displays */
#define AVIFB_BUFFER_COUNT 2

#define DEVICE_NAME "avifb"

#ifdef DEBUG
#define dprintk(data, format_, args...) \
	dev_dbg((data)->dev, "%s: " format_, __func__, ##args)

#else /* DEBUG */
#define dprintk(data_, format_, args...) (void)data_
#endif /* DEBUG */

/* AVI LCD data. */
struct avifb_data {
	struct device                   *dev;
	struct avifb_platform_data	*pdata;
	/* Used to change the videomode/pixclock/gamma... atomically, avoid
	 * incoherent configurations */
	struct mutex			 lcd_lock;
	unsigned			 lcd_format_control;
	struct avi_videomode		 lcd_videomode;
	struct avi_videomode		 lcd_default_videomode;
	enum avi_colorspace		 output_cspace;
	union avi_lcd_interface		 lcd_interface;
	wait_queue_head_t		 wait_vbl;
	atomic_t			 vbl_count;
	struct sysfs_dirent             *vbl_dirent;
	enum avi_field			 next_field;
	struct avi_segment              *display_segment;
	const struct avi_cihan          *avi_chan;
	struct avi_cmap                  gamma;
	const struct avi_node		*gam;
	const struct avi_node		*lcdc;
	/* "Default Pixel Data", default color of the LCD output when there's no
	 * stream to display */
	u32                              dpd;
	/* For interlaced modes we need to change the scaler's config depending
	 * on which field we need to display. */
	struct avi_scal_cfg		 top_config;
	struct avi_scal_cfg		 bot_config;
	/* Number of clk ticks per frame */
	unsigned long			 clk_per_frame;
	struct pinctrl                  *pctl;
	/* Monitor specs */
	struct fb_monspecs		 monspecs;
	/* Panel size in mm */
	__u32                            height;
	__u32                            width;
	struct fb_info                  *infos[];
};

#define IS_INTERLACED(_data)  \
	(!!((data)->lcd_videomode.flags & AVI_VIDEOMODE_INTERLACED))
/*
 * AVI framebuffer private data. One instance per framebuffer
 */
struct avifb_par {
	unsigned                         id;
	struct avifb_data               *data;
	const struct avifb_overlay      *overlay;
	struct avi_segment		*segment;
	u32                              pseudo_palette[16];
	atomic_t                         pan_count;
	unsigned                         pan_waits_for_vbl;
	/* same as in avifb_data, in case we have to handle interlacing at the
	 * overlay level */
	struct avi_scal_cfg		 top_config;
	struct avi_scal_cfg		 bot_config;
	/* If we're interlaced and there's no scaler in the way we use the input
	 * fifo to only fetch the displayed field */
	u32                              top_fifo_addr;
	u32                              bot_fifo_addr;
};

/* Empirical timeout value: 3/10th of a sec */
#define AVIFB_VSYNC_TIMEOUT (3 * HZ / 10)

static int avifb_wait_for_vbl(struct avifb_data *data)
{
        atomic_t current_vbl;
        int      ret;

        atomic_set(&current_vbl, atomic_read(&data->vbl_count));
        ret = wait_event_interruptible_timeout(data->wait_vbl,
                                               atomic_read(&current_vbl)
                                               != atomic_read(&data->vbl_count),
                                               AVIFB_VSYNC_TIMEOUT);

        if (ret == 0)
                return -ETIMEDOUT;

	if (ret < 0)
		return ret;

        return 0;
}

/**
 *      avifb_setcolreg - Optional function. Sets a color register.
 *      @regno: Which register in the CLUT we are programming
 *      @red: The red value which can be up to 16 bits wide
 *      @green: The green value which can be up to 16 bits wide
 *      @blue:  The blue value which can be up to 16 bits wide.
 *      @transp: If supported, the alpha value which can be up to 16 bits wide.
 *      @info: frame buffer info structure
 *
 *      Set a single color register. The values supplied have a 16 bit
 *      magnitude which needs to be scaled in this function for the hardware.
 *      Things to take into consideration are how many color registers, if
 *      any, are supported with the current color visual. With truecolor mode
 *      no color palettes are supported. Here a pseudo palette is created
 *      which we store the value in pseudo_palette in struct fb_info. For
 *      pseudocolor mode we have a limited color palette. To deal with this
 *      we can program what color is displayed for a particular pixel value.
 *      DirectColor is similar in that we can program each color field. If
 *      we have a static colormap we don't need to implement this function.
 *
 *      Returns negative errno on error, or zero on success.
 */
static int avifb_setcolreg(unsigned regno, unsigned red, unsigned green,
                           unsigned blue, unsigned transp,
                           struct fb_info *info)
{
        struct avifb_par		*par  = info->par;
        struct fb_var_screeninfo	*var  = &info->var;
	struct avifb_data               *data = par->data;
	int				 ret  = 0;

        /* Store the pseudopalette */
        if (regno < 16) {
#define TRUNCATE_COL(_c) \
	        ((_c >> (16 - var->_c.length)) << var->_c.offset)
                /* colours are 16 bits in amplitude, so we truncate them to the
                 * hardware (i.e. TRUECOLOR) value*/
                ((u32 *)info->pseudo_palette)[regno] =
                        TRUNCATE_COL(red)   |
                        TRUNCATE_COL(green) |
                        TRUNCATE_COL(blue)  |
                        TRUNCATE_COL(transp);
#undef TRUNCATE_COL
        }

	mutex_lock(&data->lcd_lock);

	if (!data->gam) {
		if (regno >= 16)
			ret = -EINVAL;

		goto unlock;
	}

	/* We only handle 8bit colours */
	red   >>= 8;
	green >>= 8;
	blue  >>= 8;

	/* gamma correction (directcolor) cmap */
	regno &= 0xff;
	data->gamma.red  [regno] = red;
	data->gamma.green[regno] = green;
	data->gamma.blue [regno] = blue;
	avi_gam_set_entry(data->gam, regno, red, green, blue);

 unlock:
	mutex_unlock(&data->lcd_lock);

        return 0;
}

/*
 * Flush all mappings from the cache for the currently active portion of the
 * framebuffer.
 */
static void avifb_flush_cache(struct fb_info *info)
{
	struct avifb_par		*par = info->par;
	struct fb_var_screeninfo	*var = &info->var;
	unsigned long			 off;
	unsigned long			 sz;

	if (!par->overlay->cacheable)
		/* fb is mapped as non-cacheable, nothing to be done here */
		return;

	/* Compute the offset and the size of the active portion of the
	 * framebuffer */
	off = var->yoffset * var->xres_virtual + var->xoffset;
	off *= var->bits_per_pixel / 8;
	sz = var->xres_virtual * var->yres * var->bits_per_pixel / 8;

	/* Flush L1 cache. __cpuc_flush_dcache_area takes a virtual address and
	 * a size. Since the cache is physically tagged, we don't need to flush
	 * all virtual mappings independantly, instead we just use the kernel
	 * mapping.  */
	__cpuc_flush_dcache_area(info->screen_base + off, sz);

	/* Now that the L1 is clean, we can move on to the L2. outer_flush_range
	 * takes a physical addresse range. */
	outer_flush_range(info->fix.smem_start + off,
			  info->fix.smem_start + off + sz);
}

/**
 *      Pans the display
 *      @var: frame buffer variable screen structure
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *      Pan (or wrap, depending on the `vmode' field) the display using the
 *      `xoffset' and `yoffset' fields of the `var' structure.
 *      If the values don't fit, return -EINVAL.
 *
 *      In this implementation, the pan will only take effect in the next VBL,
 *      never immediately (the AVI FIFO will only sync their shadow registers on
 *      EOF)
 *
 *      Returns negative errno on error, or zero on success.
 */
static int avifb_pan_display(struct fb_var_screeninfo *pan_var,
                             struct fb_info *info)
{
	struct fb_var_screeninfo	*var  = &info->var;
        struct avifb_par		*par  = info->par;
	struct avifb_data		*data = par->data;
	struct avi_dma_buffer		 buf;

        if (pan_var->xoffset + var->xres > var->xres_virtual)
                return -EINVAL;
        if (pan_var->yoffset + var->yres > var->yres_virtual)
                return -EINVAL;

        if (var->rotate != FB_ROTATE_UR)
        {
		dev_err(data->dev,
			"rotation requested but not yet implemented\n");
                return -EINVAL;
        }

        var->xoffset = pan_var->xoffset;
        var->yoffset = pan_var->yoffset;

        buf.plane0.dma_addr  = var->yoffset * var->xres_virtual + var->xoffset;
        /* Convert offset from pixels into bytes */
        buf.plane0.dma_addr *= var->bits_per_pixel / 8;
        /* Finally get the real physical address of the buffer */
        buf.plane0.dma_addr += info->fix.smem_start;

        /* Framebuffer is never planar */
        buf.plane1.dma_addr = 0;

        buf.priv = info;

        buf.status = AVI_BUFFER_READY;

        avi_segment_set_input_buffer(par->segment, &buf);

        atomic_inc(&par->pan_count);

	/* Ensure the coherency of the portion of the framebuffer to be
	 * displayed */
	avifb_flush_cache(info);

        if (par->pan_waits_for_vbl)
                return avifb_wait_for_vbl(data);

        return 0;
}

static inline int avifb_pan_display_nowait(struct fb_var_screeninfo *pan_var,
					   struct fb_info *info)
{
	struct avifb_par	*par	  = info->par;
	unsigned		 wait_vbl = par->pan_waits_for_vbl;
	int			 ret;

	par->pan_waits_for_vbl = 0;
	ret = avifb_pan_display(pan_var, info);
	par->pan_waits_for_vbl = wait_vbl;

	return ret;
}

/**
 *      avifb_blank - NOT a required function. Blanks the display.
 *      @blank_mode: the blank mode we want.
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *      Blank the screen if blank_mode != FB_BLANK_UNBLANK, else unblank.
 *      Return 0 if blanking succeeded, != 0 if un-/blanking failed due to
 *      e.g. a video mode which doesn't support it.
 *
 *      Implements VESA suspend and powerdown modes on hardware that supports
 *      disabling hsync/vsync:
 *
 *      FB_BLANK_NORMAL = display is blanked, syncs are on.
 *      FB_BLANK_HSYNC_SUSPEND = hsync off
 *      FB_BLANK_VSYNC_SUSPEND = vsync off
 *      FB_BLANK_POWERDOWN =  hsync and vsync off
 *
 *      If implementing this function, at least support FB_BLANK_UNBLANK.
 *      Return !0 for any modes that are unimplemented.
 *
 */
static int avifb_blank(int blank_mode, struct fb_info *info)
{
	struct avifb_par *par = info->par;

	/* XXX: when do we actually blank the screen (i.e. disable the
	 * pixclock)? When all blends are deactivated? */
        switch (blank_mode) {
        case FB_BLANK_UNBLANK:
	        avi_segment_unhide(par->segment);
                break;
        case FB_BLANK_NORMAL:
        case FB_BLANK_HSYNC_SUSPEND:
        case FB_BLANK_VSYNC_SUSPEND:
        case FB_BLANK_POWERDOWN:
		avi_segment_hide(par->segment);
                break;
        default:
                return -EINVAL;
        }

        return 0;
}

/* We reimplement our own mmap routine to remap the framebuffer memory as
 * cacheable in userland. This is done in order to improve read/write
 * performances on the framebuffer (the osprey won't burst on the AXI on
 * non-cacheable memory)  */
static int avifb_mmap(struct fb_info *info, struct vm_area_struct * vma)
{
	struct avifb_par	*par = info->par;
	unsigned long		 off = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long		 startpfn;

	/* Check if the mapping is within the framebuffer memory */
	if ((vma->vm_end - vma->vm_start) > (info->fix.smem_len - off))
		return -EINVAL;
	/* Compute the absolute offset */
	startpfn = (info->fix.smem_start + off) >> PAGE_SHIFT;
	/* This is an IO map - tell maydump to skip this VMA */
	vma->vm_flags |= VM_IO | VM_RESERVED;

	vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
	if (par->overlay->cacheable)
		/* Set mapping as device cacheable */
		vma->vm_page_prot = __pgprot_modify(vma->vm_page_prot,
						    L_PTE_MT_MASK,
						    L_PTE_MT_DEV_CACHED);
	else
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	return io_remap_pfn_range(vma, vma->vm_start, startpfn,
				  vma->vm_end - vma->vm_start,
				  vma->vm_page_prot);
}

static unsigned avifb_interrupt(struct avi_segment *s, enum avi_field *field)
{
	struct avifb_data    *data = s->priv;
        union avi_lcd_status status;

        status = avi_lcd_get_status(data->lcdc);

        if (avi_segment_connected(data->display_segment) &&
            status.fuf                                   &&
            printk_ratelimit())
		dev_warn(data->dev, "LCD input underflow\n");

        if (status.done) {
                atomic_inc(&data->vbl_count);
		if (IS_INTERLACED(data)) {
#ifdef AVI_BACKWARD_COMPAT
			if (avi_get_revision() == AVI_REVISION_1)
				/* In MPW1 we can't trust the "FIELD" bit given
				 * by the hardware, so we just toggle a bit in
				 * software and pray that we're correctly
				 * synchronized */
				data->next_field ^=
					AVI_TOP_FIELD ^ AVI_BOT_FIELD;
			else
#endif /* AVI_BACKWARD_COMPAT */
				/* status.field is the field currently being
				 * displayed. We want to configure the next
				 * field. */
				data->next_field = status.field ?
					AVI_TOP_FIELD : AVI_BOT_FIELD;

			*field = data->next_field;
		} else
			*field = AVI_NONE_FIELD;

		/* Wake up any task waiting for the vsync */
                wake_up_interruptible(&data->wait_vbl);
		/* Notify any task polling the vbl_count sysfs entry */
		sysfs_notify_dirent(data->vbl_dirent);

                return 0;
        }

        /* Do not dispatch the interruption if it's not the EOF */
        return AVI_IRQ_HANDLED;
}

static int avifb_instance(struct avifb_data *data)
{
	struct platform_device *pdev;

	pdev = container_of(data->dev, struct platform_device, dev);

	return pdev->id;
}

static void avifb_set_gam_config(struct avifb_data *data)
{
	if (data->gam) {
		avi_gam_set_cmap(data->gam, &data->gamma);
		avi_gam_setup(data->gam, 0, 0, 0);
	}
}

/**
 * avifb_setup_lcd_segment
 *
 * Called by avifb_probe, checks channel coherency
 */
static int __devinit avifb_setup_lcd_segment(struct avifb_data *data)
{
	unsigned long	caps;
	int             i;

        caps = data->pdata->caps;

        if (!(caps & AVI_CAPS_LCD_ALL))
	        /* We need an LCD! */
	        return -EINVAL;

        data->display_segment = avi_segment_build(&caps,
                                                  "lcd",
						  avifb_instance(data),
						  -1,
                                                  data->dev);
        if (IS_ERR(data->display_segment))
	        return PTR_ERR(data->display_segment);

        data->lcdc = avi_segment_get_node(data->display_segment,
                                          AVI_CAPS_LCD_ALL);

	data->gam  = avi_segment_get_node(data->display_segment, AVI_CAP_GAM);

	/* Initialize gamma LUT to a 1:1 transformation. */
	for (i = 0; i < 0x100; i++)
		data->gamma.red[i]
			= data->gamma.green[i]
			= data->gamma.blue[i]
			= i;

	avifb_set_gam_config(data);

        init_waitqueue_head(&data->wait_vbl);

        data->display_segment->priv = data;

        avi_segment_register_irq(data->display_segment, &avifb_interrupt);
        avi_segment_set_background(data->display_segment, data->pdata->background);

        return 0;
}

/* Return the avi_pixel_format corresponding to the pixel structure in
 * var. The framebuffer can only be in RGB format (or palette).
 *
 * Returns AVI_INVALID_FMT if the format is invalid or unknown.
 */
struct avi_dma_pixfmt avifb_get_pixfmt(struct fb_info *info,
                                       struct fb_var_screeninfo *var)
{
	if (var->nonstd)
		return AVI_PIXFMT_INVALID;

	switch (var->bits_per_pixel) {
		/* For 32 and 24 bpp modes we have to take the endianess into
		 * account since fbdev's pixel format is defined as bit shifts
		 * instead of byte order. The AVI_PIXFMT however follows the
		 * V4L2 convention of using the byte order. */
	case 32:
		if (var->red.length    != 8 ||
		    var->green.length  != 8 ||
		    var->blue.length   != 8 ||
		    var->transp.length != 8)
			return AVI_PIXFMT_INVALID;

		if (var->transp.offset == 0) {
			if (var->red.offset   == 8  &&
			    var->green.offset == 16 &&
			    var->blue.offset  == 24)
				return AVI_PIXFMT_ARGB8888;

			if (var->blue.offset  == 8  &&
			    var->green.offset == 16 &&
			    var->red.offset   == 24)
				return AVI_PIXFMT_ABGR8888;
		}

		if (var->transp.offset == 24) {
			if (var->red.offset   == 0 &&
			    var->green.offset == 8 &&
			    var->blue.offset  == 16)
				return AVI_PIXFMT_RGBA8888;

			if (var->blue.offset  == 0 &&
			    var->green.offset == 8 &&
			    var->red.offset   == 16)
				return AVI_PIXFMT_BGRA8888;
		}

		return AVI_PIXFMT_INVALID;
	case 24:
		if (var->red.length    != 8 ||
		    var->green.length  != 8 ||
		    var->blue.length   != 8 ||
		    var->transp.length != 0)
			return AVI_PIXFMT_INVALID;


		if (var->red.offset   == 0 &&
		    var->green.offset == 8 &&
		    var->blue.offset  == 16)
			return AVI_PIXFMT_RGB888;

		if (var->blue.offset  == 0 &&
		    var->green.offset == 8 &&
		    var->red.offset   == 16)
			return AVI_PIXFMT_BGR888;

		return AVI_PIXFMT_INVALID;
	case 16:
		if (var->red.length   == 5	&&
		    var->green.length == 6	&&
		    var->blue.length  == 5) {
			if (var->red.offset   == 11	&&
			    var->green.offset == 5	&&
			    var->blue.offset  == 0)
				return AVI_PIXFMT_RGB565;

			if (var->red.offset   == 0	&&
			    var->green.offset == 5	&&
			    var->blue.offset  == 11)
				return AVI_PIXFMT_BGR565;
		}

		return AVI_PIXFMT_INVALID;
	default:
		return AVI_PIXFMT_INVALID;
	}
}

static void avifb_compute_format(struct fb_info *info,
                                 struct avi_segment_format *in,
                                 struct avi_segment_format *out)
{
	struct fb_var_screeninfo	*var  = &info->var;
	struct avifb_par		*par  = info->par;
	struct avifb_data		*data = par->data;

	avi_segment_get_input_format(data->display_segment, out);

	/* If the resolution does not match the LCD, we crop */
	out->width  = var->xres;
	out->height = var->yres;

	*in = *out;

	/* Framebuffer is always RGB */
	in->colorspace = AVI_RGB_CSPACE;

	in->plane0.line_size = var->xres_virtual * var->bits_per_pixel / 8;

	if (in->interlaced)
		/* Skip the unused field */
		in->plane0.line_size *= 2;

	/* No planar input format for avifb */
	in->plane1.line_size = 0;

	in->pix_fmt = avifb_get_pixfmt(info, var);
}

static int avifb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct avifb_par		*par  = info->par;
	struct avifb_data		*data = par->data;
	struct avi_segment_format	 in;
	struct avi_segment_format	 out;
        unsigned			 framebuffer_size;
        int				 ret;

        /*
         * Match the in-memory resolution with the FIFO alignment requirements.
         */
        if (!var->xres || !var->yres)
                return -EINVAL;
        if (var->xres > var->xres_virtual)
                var->xres_virtual = var->xres;
        /* Save us some trouble later (especially in interlaced mode) and make
         * sure we can pretend to have an even number of lines. */
        if (roundup(var->yres, 2) * AVIFB_BUFFER_COUNT > var->yres_virtual)
                var->yres_virtual = roundup(var->yres, 2) * AVIFB_BUFFER_COUNT;
        if (var->xoffset + var->xres > var->xres_virtual)
                return -EINVAL;
        if (var->yoffset + var->yres > var->yres_virtual)
                return -EINVAL;

        if (avifb_get_pixfmt(info, var).id == AVI_INVALID_FMTID) {
		/* Some utils just update bpp and expect us to fill in the rest
		 * (e.g. busybox's fbset when using the -depth option). So let's
		 * try to accommodate them. */
		switch (var->bits_per_pixel) {
		case 32:
			/* default to ARGB 8888 */
			var->transp.length = 8;
			var->red.length	   = 8;
			var->green.length  = 8;
			var->blue.length   = 8;

			var->blue.offset   = 0;
			var->green.offset  = 8;
			var->red.offset    = 16;
			var->transp.offset = 24;
			break;
		case 24:
			/* default to RGB 888 */
			var->transp.length = 0;
			var->red.length	   = 8;
			var->green.length  = 8;
			var->blue.length   = 8;

			var->transp.offset = 0;
			var->blue.offset   = 0;
			var->green.offset  = 8;
			var->red.offset    = 16;
			break;
		case 16:
			/* default to RGB 565 */
			var->transp.length = 0;
			var->red.length	   = 5;
			var->green.length  = 6;
			var->blue.length   = 5;

			var->transp.offset = 0;
			var->blue.offset   = 0;
			var->green.offset  = 5;
			var->red.offset    = 11;
			break;
		default:
			/* At least we tried... */
			return -EINVAL;
		}

	/* This one has to work since the timings above are known to be
		 * valid */
		BUG_ON(avifb_get_pixfmt(info, var).id == AVI_INVALID_FMTID);
	}

        avifb_compute_format(info, &in, &out);

        ret = avi_segment_try_format(par->segment,
                                     &in, &out,
                                     &par->segment->layout);
        if (ret)
	        return ret;

        framebuffer_size = var->xres_virtual * var->yres_virtual
                         * var->bits_per_pixel / 8;
	if (PAGE_ALIGN(framebuffer_size) >
	    resource_size(&par->overlay->dma_memory))
                return -ENOMEM;

	if (data->width > 0 && data->height > 0) {
		var->height = data->height;
		var->width = data->width;
	} else {
		var->height = -1;
		var->width = -1;
	}
        if (var->rotate != FB_ROTATE_UR) {
		dev_err(data->dev,
			"rotation requested but not yet implemented\n");
                return -EINVAL;
        }

        return 0;
}

/* This function is only called after a succesful avifb_check_var, we don't have
 * to recheck everything. */
static int avifb_set_par(struct fb_info *info)
{
	struct avifb_par		*par  = info->par;
	struct fb_var_screeninfo	*var  = &info->var;
	struct avifb_data		*data = par->data;
	struct avi_segment_format	 in;
	struct avi_segment_format	 out;
	unsigned			 line_size;
	unsigned			 framebuffer_size;
	int				 ret;

	avi_segment_get_input_format(data->display_segment, &out);

	avifb_compute_format(info, &in, &out);

	line_size	 = var->xres_virtual * var->bits_per_pixel / 8;
	framebuffer_size = line_size * var->yres_virtual;

	ret = avi_segment_set_format(par->segment, &in, &out);
	if (ret)
		return ret;

	info->fix.smem_len = PAGE_ALIGN(framebuffer_size);
        info->screen_size = 0;
        info->fix.line_length = line_size;

        return 0;
}

static int avifb_ioctl(struct fb_info *info, unsigned int cmd,
                       unsigned long arg)
{
        void __user		*argp = (void __user *)arg;
        struct avifb_par	*par  = info->par;
	struct avifb_data	*data = par->data;

        switch (cmd) {
        case FBIOGET_VBLANK:
        {
                struct fb_vblank vb = {
                        /* we return the vblcount and we support WAITFORVSYNC */
                        .flags = FB_VBLANK_HAVE_COUNT | FB_VBLANK_HAVE_VSYNC,
                        .count = atomic_read(&data->vbl_count),
                };
                if (copy_to_user(argp, &vb, sizeof(vb)))
                        return -EFAULT;
                return 0;
        }
        case FBIO_WAITFORVSYNC:
		avifb_flush_cache(info);
		return avifb_wait_for_vbl(data);
	case AVI_ISP_IOGET_OFFSETS:
		return avi_isp_get_offsets(par->segment,
				           (struct avi_isp_offsets *)arg);
        default:
                return -ENOIOCTLCMD;
        }
}

/*
 * sysfs: per-framebuffer entries
 */
static ssize_t avifb_show_pan_waits_for_vsync(struct device *device,
					      struct device_attribute *attr,
					      char *buf)
{
        const struct fb_info   *info = dev_get_drvdata(device);
        const struct avifb_par *par = info->par;

        return snprintf(buf, PAGE_SIZE, "%d\n", par->pan_waits_for_vbl);
}

static ssize_t avifb_store_pan_waits_for_vsync(struct device *device,
					       struct device_attribute *attr,
					       const char *buf, size_t count)
{
        struct fb_info   *info = dev_get_drvdata(device);
        struct avifb_par *par = info->par;
        char             *last;

        par->pan_waits_for_vbl = !!simple_strtoul(buf, &last, 0);

        return count;
}

static ssize_t avifb_show_fb_enabled(struct device *device,
				     struct device_attribute *attr,
				     char *buf)
{
        const struct fb_info   *info = dev_get_drvdata(device);
        const struct avifb_par *par = info->par;

        return snprintf(buf, PAGE_SIZE, "%d\n",
                        !par->segment->layout.hidden);
}

static ssize_t avifb_store_fb_enabled(struct device *device,
				      struct device_attribute *attr,
				      const char *buf,
                                      size_t count)
{
        struct fb_info		*info = dev_get_drvdata(device);
	int			 enable;
        char			*last;

        enable = !!simple_strtoul(buf, &last, 0);

	avifb_blank(enable ? FB_BLANK_UNBLANK : FB_BLANK_NORMAL, info);

        return count;
}

static ssize_t avifb_show_pan_count(struct device *device,
				    struct device_attribute *attr,
				    char *buf)
{
        const struct fb_info   *info = dev_get_drvdata(device);
        const struct avifb_par *par = info->par;

        return snprintf(buf, PAGE_SIZE, "%u\n", atomic_read(&par->pan_count));
}

static ssize_t avifb_show_position(struct device *device,
                                   struct device_attribute *attr,
                                   char *buf)
{
	const struct fb_info   *info = dev_get_drvdata(device);
        const struct avifb_par *par = info->par;

	return snprintf(buf, PAGE_SIZE, "%u,%u\n",
			par->segment->layout.x,
	                par->segment->layout.y);
}

static ssize_t avifb_store_position(struct device *device,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count)
{
	const struct fb_info	*info = dev_get_drvdata(device);
        const struct avifb_par	*par  = info->par;
	char			*last;
	unsigned		 x, y;
	int			 ret;

	x = simple_strtoul(buf, &last, 0);
	if (*last++ != ',')
		return -EINVAL;

	y = simple_strtoul(last, &last, 0);
	if (*last != '\0' && *last != '\n')
		return -EINVAL;

	ret = avi_segment_set_position(par->segment, x, y);

	if (ret)
		return ret;

	return count;
}

static ssize_t avifb_show_alpha(struct device *device,
				struct device_attribute *attr,
				char *buf)
{
	const struct fb_info	*info  = dev_get_drvdata(device);
        const struct avifb_par	*par   = info->par;
        int			 alpha = par->segment->layout.alpha;

        if (alpha == AVI_ALPHA_OSD)
		return snprintf(buf, PAGE_SIZE, "OSD (per-pixel alpha)\n");

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", alpha);
}

static ssize_t avifb_store_alpha(struct device *device,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct fb_info		*info = dev_get_drvdata(device);
        struct avifb_par	*par  = info->par;
	unsigned		 alpha;
        char			*last;

	if (*buf == 'O' || *buf == 'o')
		alpha = AVI_ALPHA_OSD;
	else {
		alpha = simple_strtoul(buf, &last, 0);
		if (alpha > 0xff || (*last != '\0' && *last != '\n'))
			return -EINVAL;
	}

	avi_segment_set_alpha(par->segment, alpha);

        return count;
}

static ssize_t avifb_show_timeout(struct device *device,
				  struct device_attribute *attr,
				  char *buf)
{
	const struct fb_info		*info = dev_get_drvdata(device);
        const struct avifb_par		*par  = info->par;
        struct avi_node			*in_fifo;
	struct avi_fifo_registers	 regs;

	in_fifo = avi_segment_get_node(par->segment, AVI_CAP_DMA_IN);
	if (in_fifo == NULL)
		return -ENODEV;

	avi_fifo_get_registers(in_fifo, &regs);
	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d\n",
			16 << regs.timeout.burst,
			regs.timeout.issuing_capability + 1,
			regs.timeout.pincr,
			regs.timeout.hblank,
			regs.timeout.vblank);
}

static ssize_t avifb_store_timeout(struct device *device,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct fb_info			*info = dev_get_drvdata(device);
        struct avifb_par		*par  = info->par;
        struct avi_node			*in_fifo;
	char				*last;
	unsigned			 tmp;
	struct avi_fifo_registers	 regs;

	in_fifo = avi_segment_get_node(par->segment, AVI_CAP_DMA_IN);
	if (in_fifo == NULL)
		return -ENODEV;

	avi_fifo_get_registers(in_fifo, &regs);

	tmp = simple_strtoul(buf, &last, 0);
	if (*last++ != ',')
		return -EINVAL;

	switch (tmp) {
	case 16:
		regs.timeout.burst = 0;
		break;
	case 32:
		regs.timeout.burst = 1;
		break;
	case 64:
		regs.timeout.burst = 2;
		break;
	default:
		return -EINVAL;
	}

	tmp = simple_strtoul(last, &last, 0);
	if (*last++ != ',')
		return -EINVAL;

	if (tmp == 0 || tmp > 8)
		return -EINVAL;

	regs.timeout.issuing_capability = tmp - 1;

	regs.timeout.pincr = simple_strtoul(last, &last, 0);
	if (*last++ != ',')
		return -EINVAL;

	if (regs.timeout.pincr == 0)
		return -EINVAL;

	regs.timeout.hblank = simple_strtoul(last, &last, 0);
	if (*last++ != ',')
		return -EINVAL;

	regs.timeout.vblank = simple_strtoul(last, &last, 0);
	if (*last != '\0' && *last != '\n')
		return -EINVAL;

	avi_fifo_set_registers(in_fifo, &regs);

        return count;
}

static struct device_attribute avifb_fb_sysfs_attrs[] = {
        __ATTR(pan_waits_for_vsync, S_IRUGO|S_IWUSR,
               avifb_show_pan_waits_for_vsync, avifb_store_pan_waits_for_vsync),
	__ATTR(enabled, S_IRUGO|S_IWUSR,
               avifb_show_fb_enabled, avifb_store_fb_enabled),
	__ATTR(pan_count, S_IRUGO, avifb_show_pan_count, NULL),
	__ATTR(position, S_IRUGO|S_IWUSR,
	       avifb_show_position, avifb_store_position),
	__ATTR(alpha, S_IRUGO|S_IWUSR, avifb_show_alpha, avifb_store_alpha),
	__ATTR(timeout, S_IRUGO|S_IWUSR,
	       avifb_show_timeout, avifb_store_timeout),
};

static void avifb_update_period(struct avifb_data *data, unsigned long pclk)
{
	static const unsigned long	scaling = 10000;
	unsigned long			period;

	/* To achieve the best precision we should be doing:
	 *
	 *   period = (clk_per_frame * USEC_PER_SEC) / pclk
	 *
	 * However the multiplication is very likely to oveflow, so instead we
	 * settle for:
	 *
	 *   period = clk_per_frame / (pclk / scaling) * (USEC_PER_SEC / scaling)
	 */

	pclk = (pclk + scaling / 2) / scaling;

	period	= (data->clk_per_frame + pclk / 2) / pclk;
	period *= USEC_PER_SEC / scaling;

	data->display_segment->period_us = period;
}

/* Should be called with lcd_lock held. */
static int avifb_set_clock_rate(struct avifb_data *data, unsigned long *rate)
{
	unsigned long   r = *rate;
	unsigned long	pclk;
	unsigned long	delta;
	unsigned long	delta_percent;
	int		ret;

	pclk = avi_lcd_pixclock_round_rate(data->lcdc, r);

	if (pclk == r)
		goto set_rate;

	if (r > pclk)
		delta = r - pclk;
	else
		delta = pclk - r;

	/* Compute deviation in percents, rounding up */
	delta_percent = (delta * 100 + r - 1) / r;

	dev_warn(data->dev,
		 "pixelclock rounded to %lukHz [%lu%%]\n",
		 pclk / 1000,
		 delta_percent);

	/* Allow up to 5% deviation */
	if (delta_percent > 5) {
		dev_err(data->dev,
			"clock rate deviation too big, bailing out\n");
		return -ERANGE;
	}

 set_rate:
	ret = avi_lcd_pixclock_set_rate(data->lcdc, pclk);
	if (ret)
		return ret;

	avifb_update_period(data, pclk);

	*rate = pclk;

	return 0;
}

static unsigned avifb_get_framerate(const struct avifb_data *data)
{
	return (USEC_PER_SEC + data->display_segment->period_us / 2) /
		data->display_segment->period_us;
}

static void avifb_vmode_to_registers(struct avi_lcd_regs	*lcd_regs,
				     const struct avi_videomode *vmode)
{
	/*
         * The timings configuration follows. We configure both fields all the
         * time to simplify the code a bit, but the LCDC will only use the
         * top_* timings in progressive mode.
         */

        /* The horizontal timings are expressed in pixclock ticks. TOP and BOT
         * horizontal timings are identical, exception made of the vsync/hsync
         * offsets */
        lcd_regs->top_h_timing0.top_hsync_off
		= lcd_regs->bot_h_timing0.bot_hsync_off
		= vmode->hsync_len;

        lcd_regs->top_h_timing0.top_hactive_on
		= lcd_regs->bot_h_timing0.bot_hactive_on
		= lcd_regs->top_h_timing0.top_hsync_off + vmode->left_margin;

        lcd_regs->top_h_timing1.top_hactive_off
		= lcd_regs->bot_h_timing1.bot_hactive_off
		= lcd_regs->top_h_timing0.top_hactive_on + vmode->xres;

	lcd_regs->top_h_timing1.top_htotal
		= lcd_regs->bot_h_timing1.bot_htotal
		= lcd_regs->top_h_timing1.top_hactive_off + vmode->right_margin;

        /* We offset the vsync_hon and hoff by half a line to denote the bottom
         * field to the LCD. */
	lcd_regs->top_v_timing0.top_vsync_hon
		= lcd_regs->top_v_timing1.top_vsync_hoff
		= 0;

	lcd_regs->bot_v_timing0.bot_vsync_hon
		= lcd_regs->bot_v_timing1.bot_vsync_hoff
		= lcd_regs->bot_h_timing1.bot_htotal / 2;

        /* The vertical timings are expressed in lines */
        lcd_regs->top_v_timing0.top_vsync_von
		= lcd_regs->bot_v_timing0.bot_vsync_von
		= 0;

	lcd_regs->top_v_timing1.top_vsync_voff
		= lcd_regs->bot_v_timing1.bot_vsync_voff
		= vmode->vsync_len;

	lcd_regs->top_v_timing2.top_vactive_on
		= lcd_regs->top_v_timing1.top_vsync_voff + vmode->upper_margin;

	if (vmode->flags & AVI_VIDEOMODE_INTERLACED)
		lcd_regs->top_v_timing2.top_vactive_off
			= lcd_regs->top_v_timing2.top_vactive_on
			+ roundup(vmode->yres, 2) / 2;
	else
		lcd_regs->top_v_timing2.top_vactive_off
			= lcd_regs->top_v_timing2.top_vactive_on
			+ vmode->yres;

        /* bottom field is offset by one line (at least, for PAL and NTSC that's
         * how it is, I'm not sure whether it's true for all the interlaced
         * screens. Maybe this should be made configurable, we'll see) */
	lcd_regs->bot_v_timing2.bot_vactive_on  = lcd_regs->top_v_timing2.top_vactive_on  + 1;
	lcd_regs->bot_v_timing2.bot_vactive_off = lcd_regs->top_v_timing2.top_vactive_off + 1;

	lcd_regs->top_v_timing3.top_vtotal
		= lcd_regs->top_v_timing2.top_vactive_off + vmode->lower_margin;
	lcd_regs->bot_v_timing3.bot_vtotal
		= lcd_regs->bot_v_timing2.bot_vactive_off + vmode->lower_margin;
}

static inline int avifb_ticks_per_pixel(struct avifb_data *data)
{
	switch (data->lcd_format_control >> 2) {
	case 0: /* RGB888/YUV422 1X24 */
	case 1:	/* YUV 4:2:2 1X16 */
		return 1;
	case 2: /* YUV 4:2:2 2X8 */
	case 5: /* RGB565 2X8 */
	case 6: /* RGB555 2X8 */
	case 7: /* RGB444 2X8 */
		/* Two clock periods per pixel */
		return 2;
	case 3: /* RGB888 3X8 */
		/* Three clock periods per pixel */
		return 3;
		break;
	case 4: /* RGBX8888 4X8 */
		/* Four clock periods per pixel */
		return 4;
	default:
		BUG();
	}
}

static int avifb_configure_timings(struct avifb_data *data)
{
	const struct avi_node		*lcd   = data->lcdc;
	const struct avi_videomode	*vmode = &data->lcd_videomode;
	unsigned long			 ticks_per_pixel;
	unsigned long			 pclk;
	unsigned long                    old_clk_per_frame;
	int				 ret;

	struct avi_lcd_regs lcd_reg = {
                .itsource = {{
                        .done_en = 1, /* EOF IRQ enabled */
                }},
                .interface = {{
                        .ivs = !(vmode->flags & AVI_VIDEOMODE_VSYNC_ACTIVE_HIGH),
                        .ihs = !(vmode->flags & AVI_VIDEOMODE_HSYNC_ACTIVE_HIGH),
                        .ipc = data->lcd_interface.ipc,
                        .ioe = data->lcd_interface.ioe,
                        .psync_en   = data->lcd_interface.psync_en,
                        .psync_rf   = data->lcd_interface.psync_rf,
                        .itu656     = data->lcd_interface.itu656,
                        .clip_en    = data->lcd_interface.clip_en,
                        .prog       = !(vmode->flags & AVI_VIDEOMODE_INTERLACED),
                        .free_run   = data->lcd_interface.free_run,
                        .pad_select = data->lcd_interface.pad_select,
                }},
                .top_format_ctrl = {{
                        .top_format_control = data->lcd_format_control,
                }},
                .bot_format_ctrl = {{
                        .bot_format_control = data->lcd_format_control,
                }},
                .dpd = {{
			.dpd = data->dpd,
                        .colorbar = 0,
                }},
        };

	avifb_vmode_to_registers(&lcd_reg, vmode);

	mutex_lock(&data->lcd_lock);

	pclk = vmode->pixclock * 1000UL;

	dev_info(data->dev, "Using vmode %s (PCLK at %lukHz)\n",
		 vmode->name, pclk / 1000);

	ticks_per_pixel = avifb_ticks_per_pixel(data);

	pclk *= ticks_per_pixel;

	old_clk_per_frame = data->clk_per_frame;

	data->clk_per_frame = lcd_reg.top_h_timing1.top_htotal
	        * lcd_reg.top_v_timing3.top_vtotal
	        * ticks_per_pixel;

	ret = avifb_set_clock_rate(data, &pclk);
	if (ret) {
		/* Rollback */
		data->clk_per_frame = old_clk_per_frame;
		goto unlock;
	}

        avi_lcd_set_registers(lcd, &lcd_reg);

	dev_info(data->dev,
		 "screen output configured: %dx%d%c@%dHz\n",
		 vmode->xres, vmode->yres,
		 (vmode->flags & AVI_VIDEOMODE_INTERLACED) ? 'i' : 'p',
		 avifb_get_framerate(data));

	ret = 0;

 unlock:
	mutex_unlock(&data->lcd_lock);

        return ret;
}

static int avifb_configure_lcd_segment(struct avifb_data *data)
{
	struct avi_segment_format lcd_fmt = {
		.width			  = data->lcd_videomode.xres,
		.height			  = data->lcd_videomode.yres,
		.interlaced		  = IS_INTERLACED(data),
		.colorspace		  = data->output_cspace,
	};
	struct avi_segment_format	in_fmt = lcd_fmt;
	int				ret;

	if (data->display_segment->caps & AVI_CAP_CONV)
		/* Force RGB input and do color conversion in the output segment
		 * if it has a CONV. This is pretty arbitrary but it mimicks the
		 * previous behaviour of this driver: all overlays were supposed
		 * to output in RGB. */
		in_fmt.colorspace = AVI_RGB_CSPACE;

	ret = avi_segment_set_format(data->display_segment, &in_fmt, &lcd_fmt);
	if (ret) {
		dev_err(data->dev, "failed to set LCD segment format\n");
		return ret;
	}

	ret = avifb_configure_timings(data);
	if (ret)
		return ret;

#ifdef AVI_BACKWARD_COMPAT
	data->next_field = AVI_BOT_FIELD;
#endif

	avi_lcd_pixclock_enable(data->lcdc);

	avi_segment_enable_irq(data->display_segment);

	ret = avi_segment_activate(data->display_segment);
	if (ret) {
		dev_err(data->dev, "couldn't activate LCD segment\n");
		return ret;
	}

	return 0;
}

/*
 * sysfs: per-LCD entries
 */
static ssize_t avifb_show_vbl_count(struct device *device,
				    struct device_attribute *attr,
				    char *buf)
{
	struct avifb_data       *data = dev_get_drvdata(device);

        return snprintf(buf, PAGE_SIZE, "%u\n", atomic_read(&data->vbl_count));
}

static ssize_t avifb_show_colorbar(struct device *device,
				   struct device_attribute *attr,
				   char *buf)
{
	struct avifb_data       *data = dev_get_drvdata(device);
        unsigned		 col;
        const char		*str;

        col = avi_lcd_get_colorbar(data->lcdc);

        switch (col) {
        case 0:
        case 1:
                str = "0";
                break;
        case 2:
                str = "YUV";
                break;
        case 3:
                str = "RGB";
                break;
        default:
                str = "?";
        }

        return snprintf(buf, PAGE_SIZE, "%s\n", str);
}

static ssize_t avifb_store_colorbar(struct device *device,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
        struct avifb_data       *data = dev_get_drvdata(device);
        unsigned		 frm  = data->lcd_format_control;
        unsigned		 rgb;

	/* We only consider the first letter to determine what's to be done:
	 * - If we get a '1' we try to determine which colorbar to use based on
	 *   the output colorspace and format.
	 * - If we get a 'Y' or 'y' we display the YUV colorbar.
	 * - If we get an 'R' or 'r' we display the RGB colorbar.
	 */
        switch (*buf) {
        case '1':
                /* Set the colorbar depending on format_control */
		switch (frm >> 2) {
                case 1:
		case 2:
                        /* YUV colourspace */
                        rgb = 0;
			break;
                case 0:
			/* This can be either 24bit RGB or 24bit YUV. Check the
			 * segment format. */
	                rgb = data->display_segment->output_format.colorspace
		                == AVI_RGB_CSPACE;
	                break;
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
                default:
                        rgb = 1;
			break;
		}
		break;
        case 'R':
        case 'r':
                /* force RGB colorbar */
                rgb = 1;
                break;
        case 'Y':
        case 'y':
                /* force YUV colorbar */
                rgb = 0;
                break;
        case '0':
        default:
                /* disable colorbar */
                avi_lcd_set_colorbar(data->lcdc, 0);
                return count;
        }

        avi_lcd_set_colorbar(data->lcdc, 0x2 | rgb);

        return count;
}

static ssize_t avifb_show_background(struct device *device,
				     struct device_attribute *attr,
				     char *buf)
{
	struct avifb_data       *data = dev_get_drvdata(device);

        return snprintf(buf, PAGE_SIZE, "0x%06x\n",
                        avi_segment_get_background(data->display_segment));
}

static ssize_t avifb_store_background(struct device *device,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
        struct avifb_data       *data = dev_get_drvdata(device);
	char			*last;
	unsigned long		 bg;

	bg = simple_strtoul(buf, &last, 0);

	if (bg & 0xff000000 || (*last != '\0' && *last != '\n'))
		return -EINVAL;

	avi_segment_set_background(data->display_segment, bg);

        return count;
}

static ssize_t avifb_show_framerate(struct device *device,
				    struct device_attribute *attr,
				    char *buf)
{
	const struct avifb_data *data = dev_get_drvdata(device);

        return snprintf(buf, PAGE_SIZE, "%uHz\n", avifb_get_framerate(data));
}

static ssize_t avifb_store_framerate(struct device *device,
				     struct device_attribute *attr,
				     const  char *buf,
				     size_t count)
{
	struct avifb_data	*data = dev_get_drvdata(device);
	unsigned long		 fps;
	unsigned long		 clk;
	char			*last;
	int			 ret;

	fps = simple_strtoul(buf, &last, 0);

	if (*last != '\0' && *last != '\n')
		return -EINVAL;

	clk = data->clk_per_frame * fps;

	dev_info(data->dev, "Setting PCLK to %lukHz\n", clk / 1000);

	mutex_lock(&data->lcd_lock);
	avi_lcd_pixclock_disable(data->lcdc);

	ret = avifb_set_clock_rate(data, &clk);

	avi_lcd_pixclock_enable(data->lcdc);
	mutex_unlock(&data->lcd_lock);

	if (ret)
		return ret;

	return count;
}

/* The clear register value is tristate:
 *
 * Normal:   usual clear behaviour. triggered by the LCD state machine.
 * Inactive: clear is forced low
 * Active:   clear is forced high
 */
static ssize_t avifb_store_clear(struct device *device,
                                 struct device_attribute *attr,
                                 const char *buf, size_t count)
{
        struct avifb_data       *data = dev_get_drvdata(device);
        unsigned		 clear;

        switch (*buf) {
        case 'a':
        case 'A':
        case '1': /* Active */
	        clear = AVI_LCD_FORCE_CLEAR_ACTIVE;
	        break;
        case 'i':
        case 'I': /* Inactive */
	        clear = AVI_LCD_FORCE_CLEAR_INACTIVE;
	        break;
        case 'n':
        case 'N':
        case '0':
	        clear = AVI_LCD_FORCE_CLEAR_NORMAL;
	        break;
        default:
	        return -EINVAL;
        }

        avi_lcd_set_force_clear(data->lcdc, clear);

        return count;
}

static ssize_t avifb_show_clear(struct device *device,
                                struct device_attribute *attr,
                                char *buf)
{
	const struct avifb_data *data = dev_get_drvdata(device);
	const char		*str;
	unsigned		 clear;

	clear = avi_lcd_get_force_clear(data->lcdc);

	switch (clear) {
	case AVI_LCD_FORCE_CLEAR_ACTIVE:
		str = "Active";
		break;
	case AVI_LCD_FORCE_CLEAR_INACTIVE:
		str = "Inactive";
		break;
	case AVI_LCD_FORCE_CLEAR_NORMAL:
		str = "Normal";
		break;
	default:
		str = "INVALID";
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", str);
}

static ssize_t avifb_show_vmode(struct device *device,
				struct device_attribute *attr,
				char *buf)
{
	const struct avifb_data		*data  = dev_get_drvdata(device);
	const struct avi_videomode	*vmode = &data->lcd_videomode;

	return snprintf(buf, PAGE_SIZE, "%s\n", vmode->name);
}

static int avifb_resize_overlay(struct fb_info *info,
				const struct avi_videomode *vmode)
{
	struct avifb_par		*par = info->par;
	struct fb_var_screeninfo        *var = &info->var;
	struct avi_segment_format	 in;
	struct avi_segment_format	 out;
	struct avi_segment_layout        layout;
	unsigned			 max_width;
	unsigned			 max_height;
	int				 ret;

	avi_segment_get_layout(par->segment, &layout);

	/* Check if the position is outside the screen */
	if (layout.x >= vmode->xres)
		layout.x = 0;

	if (layout.y >= vmode->yres)
		layout.y = 0;

	/* Here's our heuristic: we see if the current overlay at its current
	 * position still fits in the screen, if so we do nothing. Otherwise we
	 * leave the overlay at the current position but we adjust its size to
	 * fit. */
	max_width = vmode->xres - layout.x;
	var->xres = max_width;

	max_height = vmode->yres - layout.y;
	var->yres = max_height;

	/* Force a re-check of the variable configuration */
	ret = avifb_check_var(var, info);
	if (ret)
		return ret;

	/* Reconfigure private data and fb fix configuration */
	ret = avifb_set_par(info);
	if (ret)
		return ret;

	avifb_compute_format(info, &in, &out);

	ret = avi_segment_set_format_and_layout(par->segment,
						&in,
						&out,
						&layout);
	if (ret)
		return ret;

	ret = avifb_pan_display_nowait(&info->var, info);

	return ret;
}

static int avifb_change_vmode(struct avifb_data *data,
			      const struct avi_videomode *vmode)
{
	const unsigned			noverlays = data->pdata->overlay_nr;
	struct avi_segment_format	in_fmt;
	struct avi_segment_format	lcd_fmt;
	int				i;

	for (i = 0; i < noverlays; i++)
		BUG_ON(!lock_fb_info(data->infos[i]));

	avi_lcd_pixclock_disable(data->lcdc);

	/* Reconfigure the LCD */
	avi_segment_get_input_format(data->display_segment, &in_fmt);
	avi_segment_get_output_format(data->display_segment, &lcd_fmt);

	in_fmt.width  = lcd_fmt.width  = vmode->xres;
	in_fmt.height = lcd_fmt.height = vmode->yres;
	in_fmt.interlaced = lcd_fmt.interlaced =
		vmode->flags & AVI_VIDEOMODE_INTERLACED;

	BUG_ON(avi_segment_set_format(data->display_segment,
				      &in_fmt,
				      &lcd_fmt));

	data->lcd_videomode = *vmode;

	avifb_configure_timings(data);

	/* Reconfigure the FB overlays */
	for (i = 0; i < noverlays; i++)
		BUG_ON(avifb_resize_overlay(data->infos[i], vmode));

	avi_lcd_pixclock_enable(data->lcdc);

	for (i = 0; i < noverlays; i++)
		unlock_fb_info(data->infos[i]);

	return 0;
}

static void avifb_from_fb_videomode(struct avi_videomode	*vmode,
				    const struct fb_videomode	*fb_vmode)
{
	vmode->name	    = fb_vmode->name;
	vmode->xres	    = fb_vmode->xres;
	vmode->yres	    = fb_vmode->yres;
	vmode->pixclock	    = PICOS2KHZ(fb_vmode->pixclock);
	vmode->left_margin  = fb_vmode->left_margin;
	vmode->right_margin = fb_vmode->right_margin;
	vmode->upper_margin = fb_vmode->upper_margin;
	vmode->lower_margin = fb_vmode->lower_margin;
	vmode->hsync_len    = fb_vmode->hsync_len;
	vmode->vsync_len    = fb_vmode->vsync_len;

	vmode->flags	    = 0;

	if (fb_vmode->vmode & FB_VMODE_INTERLACED)
		vmode->flags |= AVI_VIDEOMODE_INTERLACED;

	if (fb_vmode->sync & FB_SYNC_VERT_HIGH_ACT)
		vmode->flags |= AVI_VIDEOMODE_VSYNC_ACTIVE_HIGH;

	if (fb_vmode->sync & FB_SYNC_HOR_HIGH_ACT)
		vmode->flags |= AVI_VIDEOMODE_HSYNC_ACTIVE_HIGH;
}

struct fb_videomode *avifb_select_mode(const char         *id,
				       struct fb_monspecs *monspecs)
{
	struct avi_segment	*s;
	struct avifb_data	*data;
	struct fb_videomode     *best = NULL;
	struct avi_videomode     vmode;
	unsigned long		 min_period;
	int			 i;
	int			 ret;
	int			 select_first_detailed;

	s = avi_segment_find(id);

	if (s == NULL)
		return ERR_PTR(-ENODEV);

	data = s->priv;
	if (data == NULL)
		return ERR_PTR(-ENODEV);

	/* struct fb_videomode stores the pixclock period in ns, not the
	 * frequency. */
	min_period = KHZ2PICOS(AVI_LIMIT_MAX_PHY_FREQ) *
		avifb_ticks_per_pixel(data);

	/* The FB_MISC_1ST_DETAIL flag is set to indicate that the screen
	 * prefered mode is the first detailed one */

	select_first_detailed = monspecs->misc & FB_MISC_1ST_DETAIL;

	for (i = 0; i < monspecs->modedb_len; i++) {
		struct fb_videomode *m = &monspecs->modedb[i];

		if (m->pixclock < min_period       ||
		    m->xres > AVI_LIMIT_MAX_WIDTH  ||
		    m->yres > AVI_LIMIT_MAX_HEIGHT) {
			/* We don't support this videomode */
			if (m->flag & FB_MODE_IS_DETAILED)
				/* We don't support the screen preered mode */
				select_first_detailed = 0;
			continue;
		}

		if (select_first_detailed && (m->flag & FB_MODE_IS_DETAILED)) {
			/* Select the prefered mode if possible */
			best = m;
			break;
		}

		if (best == NULL) {
			/* Better than nothing */
			best = m;
			continue;
		}

		if (m->xres > best->xres) {
			best = m;
			continue;
		}

		if (m->xres == best->xres && m->pixclock < best->pixclock) {
			best = m;
			continue;
		}
	}

#ifdef DEBUG
	dprintk(data, "Selecting mode:\n");

	for (i = 0; i < monspecs->modedb_len; i++) {
		struct fb_videomode *m = &monspecs->modedb[i];

		dprintk(data, "%c %dx%d @ %ldkHz\n",
			(m == best) ? '*' : ' ',
			m->xres, m->yres, PICOS2KHZ(m->pixclock));
	}
#endif

	if (best == NULL)
		return NULL;

	avifb_from_fb_videomode(&vmode, best);

	ret = avifb_change_vmode(data, &vmode);
	if (ret)
		return ERR_PTR(ret);

	return best;
}
EXPORT_SYMBOL(avifb_select_mode);

int avifb_set_screen_size(const char* id, unsigned width, unsigned height)
{
	struct avi_segment *s;
	struct avifb_data  *data;

	s = avi_segment_find(id);
	if (s == NULL)
		return -ENODEV;

	data = s->priv;
	if (data == NULL)
		return -ENODEV;

	data->width = width;
	data->height = height;

	return 0;
}
EXPORT_SYMBOL(avifb_set_screen_size);

static ssize_t avifb_store_vmode(struct device *device,
                                 struct device_attribute *attr,
                                 const char *buf, size_t count)
{
	struct avifb_data		*data = dev_get_drvdata(device);
	size_t				 l    = count;
	const struct avi_videomode	*vm;
	const struct avi_videomode	*vmode;
	int				 i;
	int				 ret;

	while (buf[l] == '\n' || buf[l] == '\0')
		l--;

	vmode = NULL;
	for (i = 0; (vm = data->pdata->lcd_videomodes[i]) != NULL; i++) {
		if (strncmp(vm->name, buf, l) == 0) {
			vmode = vm;
			break;
		}
	}

	if (!vmode) {
		dev_warn(data->dev,
			 "unknown vmode \"%s\"\n", buf);
		return -EINVAL;
	}

	ret = avifb_change_vmode(data, vmode);
	if (ret)
		return ret;

	return count;
}
static ssize_t avifb_show_default_vmode(struct device *device,
				     struct device_attribute *attr,
				     char *buf)
{
	const struct avifb_data         *data  = dev_get_drvdata(device);
	const struct avi_videomode      *vmode = &data->lcd_default_videomode;

        return snprintf(buf, PAGE_SIZE, "%s\n", vmode->name);
}

static ssize_t avifb_show_default_resolution(struct device *device,
				     struct device_attribute *attr,
				     char *buf)
{
	const struct avifb_data         *data  = dev_get_drvdata(device);
	const struct avi_videomode      *vmode = &data->lcd_default_videomode;

	return snprintf(buf, PAGE_SIZE, "%ux%u\n", vmode->xres, vmode->yres);
}

static ssize_t avifb_show_resolution(struct device *device,
				     struct device_attribute *attr,
				     char *buf)
{
	const struct avifb_data		*data  = dev_get_drvdata(device);
	const struct avi_videomode	*vmode = &data->lcd_videomode;

	return snprintf(buf, PAGE_SIZE, "%ux%u\n", vmode->xres, vmode->yres);
}

static ssize_t avifb_show_dpd(struct device *device,
			      struct device_attribute *attr,
			      char *buf)
{
	const struct avifb_data		*data  = dev_get_drvdata(device);

	return snprintf(buf, PAGE_SIZE, "0x%06x\n", data->dpd);
}

static ssize_t avifb_store_dpd(struct device *device,
			       struct device_attribute *attr,
			       const  char *buf,
			       size_t count)
{
	struct avifb_data	*data = dev_get_drvdata(device);
	unsigned long		 dpd;
	char			*last;

	dpd = simple_strtoul(buf, &last, 0);

	if (*last != '\0' && *last != '\n')
		return -EINVAL;

	if (dpd & ~0xFFFFFFfUL)
		return -ERANGE;

	data->dpd = dpd;

	if (data->lcdc)
		avi_lcd_set_dpd(data->lcdc, data->dpd);

	return count;
}

static ssize_t avifb_show_default_screen_size(struct device *device,
				      struct device_attribute *attr,
				      char *buf)
{
	const struct avifb_data *data = dev_get_drvdata(device);
	const struct avifb_platform_data *pdata = data->pdata;

	return snprintf(buf, PAGE_SIZE, "%ux%u\n", pdata->width, pdata->height);
}

static ssize_t avifb_show_screen_size(struct device *device,
                              struct device_attribute *attr,
                              char *buf)
{
	const struct avifb_data *data = dev_get_drvdata(device);

	return snprintf(buf, PAGE_SIZE, "%ux%u\n", data->width, data->height);
}

void avifb_set_monspecs(const char *id, struct fb_monspecs *monspecs)
{
	struct avi_segment	*seg;
	struct avifb_data	*data;

	/* Get avi_segment associated to LCD ID */
	seg = avi_segment_find(id);
	if (seg == NULL)
		return;

	/* Get avifb_data from avi_segment */
	data = seg->priv;
	if (data == NULL)
		return;

	/* Copy monitor specs */
	if (monspecs == NULL)
		memset(&data->monspecs, 0, sizeof(struct fb_monspecs));
	else
		memcpy(&data->monspecs, monspecs, sizeof(struct fb_monspecs));

	/* Reset modedb to an empty list */
	data->monspecs.modedb = NULL;
	data->monspecs.modedb_len = 0;
}
EXPORT_SYMBOL(avifb_set_monspecs);

static ssize_t avifb_show_monitor_manufacturer(struct device *device,
					       struct device_attribute *attr,
					       char *buf)
{
	const struct avifb_data *data = dev_get_drvdata(device);

	return snprintf(buf, PAGE_SIZE, "%s\n", data->monspecs.manufacturer);
}


static ssize_t avifb_show_monitor_name(struct device *device,
				       struct device_attribute *attr, char *buf)
{
	const struct avifb_data *data = dev_get_drvdata(device);

	return snprintf(buf, PAGE_SIZE, "%s\n", data->monspecs.monitor);
}

static ssize_t avifb_show_monitor_serial(struct device *device,
					 struct device_attribute *attr,
					 char *buf)
{
	const struct avifb_data *data = dev_get_drvdata(device);

	return snprintf(buf, PAGE_SIZE, "%s\n", data->monspecs.serial_no);
}

static ssize_t avifb_show_monitor_misc(struct device *device,
				       struct device_attribute *attr, char *buf)
{
	const struct avifb_data *data = dev_get_drvdata(device);

	return snprintf(buf, PAGE_SIZE, "%s\n", data->monspecs.ascii);
}

static struct device_attribute avifb_lcd_sysfs_attrs[] = {
        __ATTR(vbl_count, S_IRUGO, avifb_show_vbl_count, NULL),
        __ATTR(colorbar, S_IRUGO | S_IWUSR,
	       avifb_show_colorbar, avifb_store_colorbar),
	__ATTR(background, S_IRUGO | S_IWUSR,
	       avifb_show_background, avifb_store_background),
	__ATTR(framerate, S_IRUGO | S_IWUSR,
	       avifb_show_framerate, avifb_store_framerate),
	__ATTR(clear, S_IRUGO | S_IWUSR, avifb_show_clear, avifb_store_clear),
	__ATTR(vmode, S_IRUGO | S_IWUSR, avifb_show_vmode, avifb_store_vmode),
	__ATTR(default_vmode, S_IRUGO | S_IWUSR, avifb_show_default_vmode, NULL),
	__ATTR(resolution, S_IRUGO, avifb_show_resolution, NULL),
	__ATTR(default_resolution, S_IRUGO, avifb_show_default_resolution, NULL),
	__ATTR(dpd, S_IRUGO | S_IWUSR, avifb_show_dpd, avifb_store_dpd),
	// device physical size (mm)
	__ATTR(screen_size, S_IRUGO, avifb_show_screen_size, NULL),
	__ATTR(default_screen_size, S_IRUGO, avifb_show_default_screen_size, NULL),
	/* Monitor specs */
	__ATTR(monitor_manufacturer, S_IRUGO, avifb_show_monitor_manufacturer,
	       NULL),
	__ATTR(monitor_name, S_IRUGO, avifb_show_monitor_name, NULL),
	__ATTR(monitor_serial, S_IRUGO, avifb_show_monitor_serial, NULL),
	__ATTR(monitor_misc, S_IRUGO, avifb_show_monitor_misc, NULL),
};

/*
 *  Frame buffer operations
 */
static struct fb_ops avifb_ops = {
        .owner          = THIS_MODULE,
        .fb_check_var   = avifb_check_var,
        .fb_set_par     = avifb_set_par,
        .fb_setcolreg   = avifb_setcolreg,
        .fb_blank       = avifb_blank,
        .fb_pan_display = avifb_pan_display,
        .fb_ioctl       = avifb_ioctl,
	.fb_mmap        = avifb_mmap,

        /*
         * fillrect, copyarea and imageblit are mandatory. We use the generic
         * software based versions.
         */
        .fb_fillrect    = cfb_fillrect,
        .fb_copyarea    = cfb_copyarea,
        .fb_imageblit   = cfb_imageblit,
};

/* Used to set the basic "hardcoded" features of the hardware and driver */
static const struct fb_fix_screeninfo avifb_default_fix = {
        .id         = DEVICE_NAME,
        .type       = FB_TYPE_PACKED_PIXELS,
        .visual     = FB_VISUAL_TRUECOLOR,
        .xpanstep   = 0,
        .ypanstep   = 1,
        .ywrapstep  = 0,
        .accel      = FB_ACCEL_NONE,
        .mmio_start = 0,
        .mmio_len   = 0,
};

static int __devinit avifb_overlay_init(struct avifb_data *data, unsigned id)
{
	struct fb_info			*info;
	struct avifb_par		*par;
	struct resource			*vmem;
	struct avi_segment_format        lcd_fmt;
	unsigned long			 caps;
	unsigned			 i;
	int				 ret;

	info = framebuffer_alloc(sizeof(struct avifb_par), data->dev);
	if (!info) {
		ret = -ENOMEM;
		goto nomem;
	}

	data->infos[id] = info;
	par		= info->par;
	par->id		= id;
	par->data       = data;
	par->overlay    = &data->pdata->overlays[id];

	/* Force DMA input for the overlay, it makes no sense not to have it. */
	caps = par->overlay->caps | AVI_CAP_DMA_IN;

	avi_segment_get_input_format(data->display_segment, &lcd_fmt);

	if (lcd_fmt.colorspace != AVI_RGB_CSPACE)
		caps |= AVI_CAP_CONV;

	par->segment = avi_segment_build(&caps,
	                                 "fb",
					 avifb_instance(data),
					 id,
	                                 data->dev);
	if (IS_ERR(par->segment)) {
		ret = PTR_ERR(par->segment);
		goto no_segment;
	}

	/* The default value for pan_waits_for_vbl is defined in the KConfig. It
	 * can be overriden at runtime through the sysfs. */
#ifdef CONFIG_FB_AVI_PAN_WAITS_FOR_VSYNC
	par->pan_waits_for_vbl = 1;
#else
	par->pan_waits_for_vbl = 0;
#endif

	info->pseudo_palette = par->pseudo_palette;

	info->fbops = &avifb_ops;
	info->fix   = avifb_default_fix;

	if (data->gam)
		info->fix.visual = FB_VISUAL_DIRECTCOLOR;

	info->fix.smem_start = par->overlay->dma_memory.start;
	info->fix.smem_len   = resource_size(&par->overlay->dma_memory);

	vmem = request_mem_region(info->fix.smem_start,
	                          resource_size(&par->overlay->dma_memory),
				  dev_driver_string(data->dev));
	if (!vmem) {
		ret = -EBUSY;
		dev_err(data->dev,
			"request_mem_region %08lx:%08x failed\n",
			info->fix.smem_start,
			info->fix.smem_len);
		goto mem_request_failed;
	}

	if (par->overlay->cacheable)
		/* Map the framebuffer DMA memory as cached. This improves the
		 * write performances in certain cases (but also contributes to
		 * cache corruption). */
		info->screen_base = ioremap_cached(info->fix.smem_start,
						   info->fix.smem_len);
	else
		info->screen_base = ioremap(info->fix.smem_start,
					    info->fix.smem_len);

	if (!info->screen_base) {
		ret = -ENOMEM;
		dev_err(data->dev,
			"ioremap_cached %08lx:%08x failed\n",
			info->fix.smem_start,
			info->fix.smem_len);
		goto ioremap_failed;
	}

	/* Paint the framebuffer black */
	memset(info->screen_base, 0, info->fix.smem_len);

	/* Use the specified color depth of default to 16bit RGB */
	if (par->overlay->color_depth)
		info->var.bits_per_pixel = par->overlay->color_depth;
	else
		info->var.bits_per_pixel = 16;

	/* The default resolution is the display resolution */
	info->var.xres = data->display_segment->input_format.width;
	info->var.yres = data->display_segment->input_format.height;

	/* If the BSP requires a different resolution we override it here */
	if (par->overlay->layout.width)
		info->var.xres = par->overlay->layout.width;
	if (par->overlay->layout.height)
		info->var.yres = par->overlay->layout.height;

	info->var.yres_virtual = info->var.yres * 2;

	ret = avifb_check_var(&info->var, info);
	if (ret)
		goto check_var_failed;

	ret = avifb_set_par(info);
	if (ret)
		goto set_par_failed;

	ret = avi_segment_set_position(par->segment,
	                               par->overlay->layout.x,
	                               par->overlay->layout.y);
	if (ret)
		goto set_position_failed;

	ret = avifb_pan_display(&info->var, info);
	if (ret)
		goto pan_failed;

	avi_segment_set_alpha(par->segment, par->overlay->layout.alpha);
	if (!par->overlay->layout.enabled)
		avi_segment_hide(par->segment);

	avi_segment_activate(par->segment);

	ret = avi_segment_connect(par->segment,
	                          data->display_segment,
	                          par->overlay->zorder);
	if (ret) {
		dev_err(data->dev,
			"Couldn't connect segment %s to %s "
			"(zorder: %d)\n",
			par->segment->id,
			data->display_segment->id,
			par->overlay->zorder);
		goto segment_connect_failed;
	}

	ret = register_framebuffer(info);
	if (ret)
		goto register_failed;

	for (i = 0; i < ARRAY_SIZE(avifb_fb_sysfs_attrs); i++) {
                ret = device_create_file(info->dev, &avifb_fb_sysfs_attrs[i]);
                if (ret) {
                        while (i--)
                                device_remove_file(info->dev,
						   &avifb_fb_sysfs_attrs[i]);
			goto sysfs_failed;
                }
        }

	dev_info(data->dev,
		 "overlay fb%d initialized [%u x %u] (%s)\n",
		 info->node, info->var.xres, info->var.yres,
		 par->segment->id);

	return 0;

sysfs_failed:
	unregister_framebuffer(info);
register_failed:
	avi_segment_disconnect(par->segment, data->display_segment);
segment_connect_failed:
pan_failed:
set_position_failed:
set_par_failed:
check_var_failed:
	iounmap(info->screen_base);
ioremap_failed:
	release_mem_region(info->fix.smem_start,
			   resource_size(&par->overlay->dma_memory));
mem_request_failed:
	avi_segment_teardown(par->segment);
no_segment:
	kfree(info);
nomem:
	data->infos[id] = NULL;
	return ret;
}

static void avifb_overlay_destroy(struct avifb_data *data, unsigned id)
{
	struct fb_info		*info;
	struct avifb_par        *par;
	int			 i;

	info = data->infos[id];
	if (info == NULL)
		return;

	par = info->par;

	for (i = 0; i < ARRAY_SIZE(avifb_fb_sysfs_attrs); i++)
		device_remove_file(info->dev, &avifb_fb_sysfs_attrs[i]);

	unregister_framebuffer(info);

	iounmap(info->screen_base);
	release_mem_region(info->fix.smem_start,
	                   resource_size(&par->overlay->dma_memory));
	avi_segment_disconnect(par->segment, data->display_segment);
	avi_segment_teardown(par->segment);
	kfree(info);
	data->infos[id] = NULL;
}

/* This has to match with the pin configuration in the BSP. If the command line
 * requests RGB888_1X24 but only 8 data bits are enabled in the pinctrl there
 * will be a silent missmatch. */
static const struct
{
	const char *name;
	unsigned    value;
} avifb_busnames[] = {
#define AVIFB_BUS(_n) { #_n, AVI_FORMAT_CONTROL_ ##_n }
	AVIFB_BUS(RGB888_1X24),
	AVIFB_BUS(YUV888_1X24),
	AVIFB_BUS(YUYV_1X16),
	AVIFB_BUS(YVYU_1X16),
	AVIFB_BUS(UYVY_2X8),
	AVIFB_BUS(VYUY_2X8),
	AVIFB_BUS(YUYV_2X8),
	AVIFB_BUS(YVYU_2X8),
	AVIFB_BUS(RGB888_3X8),
	AVIFB_BUS(BGR888_3X8),
	AVIFB_BUS(RGBX8888_4X8),
	AVIFB_BUS(BGRX8888_4X8),
	AVIFB_BUS(RGB565_2X8),
	AVIFB_BUS(BGR565_2X8),
	AVIFB_BUS(RGB555_2X8),
	AVIFB_BUS(BGR555_2X8),
	AVIFB_BUS(RGB444_2X8),
	AVIFB_BUS(BGR444_2X8),
};

/**
 * avifb_parse_options: parse the kernel command line for framebuffer-specific options.
 */
static int __devinit avifb_parse_options(struct avifb_data *data)
{
	const struct avi_videomode **vmodes = data->pdata->lcd_videomodes;
	char			    *opts = NULL;
	const char		    *vmode;
	int 			     pad;
	const char		    *bus;
	char			    *opt;
	int			     i;

	if (fb_get_options(dev_name(data->dev), &opts)) {
		/* If fb_get_options returns 1 it means we were asked to turn
		 * off the screen (e.g. video=avifb:off) */
		dev_info(data->dev, "canceling probe due to command line\n");
		return -ENODEV;
	}

	if (opts == NULL)
		return 0;

	/* For all the parsing I assume that command line errors are not fatal:
	 * bogus entries are simply ignored (with a warning message) */
	while ((opt = strsep(&opts, ",")) != NULL) {
		if (!strcmp(opt, "yuv"))
			data->output_cspace = AVI_BT709_CSPACE;
		else if (!strcmp(opt, "rgb"))
			data->output_cspace = AVI_RGB_CSPACE;
		else if (!strcmp(opt, "bt656"))
			data->lcd_interface.itu656 = 1;
		else if (!strcmp(opt, "bt601"))
			data->lcd_interface.itu656 = 0;
		else if (!strncmp(opt, "pad=", strlen("pad="))) {
			pad = *(opt + strlen("pad=")) - '0';

			if (pad >= 0 && pad <= 2) {
				data->lcd_interface.pad_select = pad;
			} else
				dev_warn(data->dev, "bogus pad value\n");
		}
		else if (!strncmp(opt, "ipc=", strlen("ipc=")))
			data->lcd_interface.ipc = (*(opt + strlen("ipc=")) != '0');
		else if (!strncmp(opt, "vmode=", strlen("vmode="))) {
			vmode = opt + strlen("vmode=");

			if (vmodes) {
				for (i = 0; vmodes[i]; i++)
					if (!strcmp(vmode, vmodes[i]->name))
						break;

				if (vmodes[i]) {
					data->lcd_videomode = *vmodes[i];
					data->lcd_default_videomode = *vmodes[i];
					continue;
				}
			}

			dev_warn(data->dev, "unknown vmode %s", opt);
		}
		else if (!strncmp(opt, "bus=", strlen("bus="))) {
			bus = opt + strlen("bus=");

			for (i = 0; i < ARRAY_SIZE(avifb_busnames); i++)
				if (!strcmp(bus, avifb_busnames[i].name))
					break;

			if (i == ARRAY_SIZE(avifb_busnames))
				dev_warn(data->dev, "unknown bus type %s", bus);
			else
				data->lcd_format_control
					= avifb_busnames[i].value;
		}
		else
			dev_warn(data->dev, "unknown option %s", opt);
	}

	return 0;
}

static int __devinit avifb_probe(struct platform_device *pdev)
{
	struct avifb_data		*data;
	struct avifb_platform_data	*pdata;
	signed				 i;
	int				 ret;

	pdata = dev_get_platdata(&pdev->dev);

	if (!pdata) {
		ret = -ENODEV;
		goto nopdata;
	}

	/* Allocate avifb_data structure + enough memory to hold the pointers to
	 * the fb_info structs */
	data = kzalloc(sizeof(*data) +
		       pdata->overlay_nr * sizeof(*(data->infos)), GFP_KERNEL);

	if (!data) {
		ret = -ENOMEM;
		goto nomem;
	}

	data->dev   = &pdev->dev;
	data->pdata = pdata;

	mutex_init(&data->lcd_lock);

	/* Load the defaults, may be overriden by the command line */
	data->lcd_format_control 	= pdata->lcd_format_control;
	data->lcd_videomode      	= *pdata->lcd_default_videomode;
	data->lcd_default_videomode 	= *pdata->lcd_default_videomode;
	data->output_cspace      	= pdata->output_cspace;
	data->lcd_interface      	= pdata->lcd_interface;
	data->width              	= pdata->width;
	data->height             	= pdata->height;
	data->dpd                	= pdata->dpd;

	if (data->output_cspace == AVI_NULL_CSPACE)
		/* Default to RGB */
		data->output_cspace = AVI_RGB_CSPACE;

	ret = avifb_parse_options(data);
	if (ret)
		goto bad_option;

	ret = avifb_setup_lcd_segment(data);
	if (ret)
		goto bad_lcd_segment;

	data->pctl = pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(data->pctl)) {
		ret = PTR_ERR(data->pctl);
		goto nopin;
	}

	for (i = 0; i < ARRAY_SIZE(avifb_lcd_sysfs_attrs); i++) {
                ret = device_create_file(&pdev->dev, &avifb_lcd_sysfs_attrs[i]);
                if (ret && i) {
                        for (--i; i >= 0; i--)
                                device_remove_file(&pdev->dev,
						   &avifb_lcd_sysfs_attrs[i]);
			goto sysfs_err;
                }
        }

	/* Retrieve the struct sysfs_dirent of the vbl_count in order to call
	 * sysfs_notify_dirent from the irq handler */
	data->vbl_dirent = sysfs_get_dirent(pdev->dev.kobj.sd, NULL, "vbl_count");
	if (!data->vbl_dirent)
		goto vbl_dirent_err;

	ret = avifb_configure_lcd_segment(data);
	if (ret)
		goto lcd_config_failed;

	for (i = 0; i < pdata->overlay_nr; i++)
		if ((ret = avifb_overlay_init(data, i))) {
			while (i--)
				avifb_overlay_destroy(data, i);
			goto overlay_err;
		}

	dev_set_drvdata(&pdev->dev, data);

	dev_info(&pdev->dev, "avifb initialized, screen configured\n");

	return 0;

overlay_err:
	avi_lcd_pixclock_disable(data->lcdc);
lcd_config_failed:
	sysfs_put(data->vbl_dirent);
vbl_dirent_err:
	for (i = 0; i < ARRAY_SIZE(avifb_lcd_sysfs_attrs); i++)
		device_remove_file(&pdev->dev, &avifb_lcd_sysfs_attrs[i]);
sysfs_err:
	pinctrl_put(data->pctl);
nopin:
	/* It's safe to call this function even if the pixclock's already
	 * disabled.
	 * Note : we keep the original return value from 'PTR_ERR(data->pctl)' */
	avi_lcd_pixclock_disable(data->lcdc);
	avi_segment_teardown(data->display_segment);
bad_lcd_segment:
bad_option:
	kfree(data);
nomem:
nopdata:
	return ret;
}

static int __devexit avifb_remove(struct platform_device *pdev)
{
        const struct avifb_platform_data *pdata = dev_get_platdata(&pdev->dev);
        struct avifb_data                *data = dev_get_drvdata(&pdev->dev);
        int                               i;

	sysfs_put(data->vbl_dirent);
	for (i = 0; i < pdata->overlay_nr; i++)
		avifb_overlay_destroy(data, i);
	for (i = 0; i < ARRAY_SIZE(avifb_lcd_sysfs_attrs); i++)
		device_remove_file(&pdev->dev, &avifb_lcd_sysfs_attrs[i]);
	dev_set_drvdata(&pdev->dev, NULL);
	avi_lcd_pixclock_disable(data->lcdc);
	pinctrl_put(data->pctl);
	avi_segment_teardown(data->display_segment);
	kfree(data);

	return 0;
}

static int avifb_suspend(struct device *dev)
{
       struct avifb_data	*data = dev_get_drvdata(dev);
       int			 i;

       dev_info(dev, "framebuffer shutting down\n");

       BUG_ON(data == NULL);

       for (i = 0; i < data->pdata->overlay_nr; i++)
	       fb_set_suspend(data->infos[i], 1);

       avi_lcd_pixclock_disable(data->lcdc);

       return 0;
}

static int avifb_resume(struct device *dev)
{
	struct avifb_data       *data = dev_get_drvdata(dev);
	struct fb_info          *info;
	struct avifb_par        *par;
	int                      i;

	dev_info(dev, "framebuffer resuming\n");

	BUG_ON(data == NULL);

	for (i = 0; i < data->pdata->overlay_nr; i++) {

		info = data->infos[i];
		par  = info->par;

		avifb_pan_display_nowait(&info->var, info);

		avi_segment_activate(par->segment);

		fb_set_suspend(info, 0);
	}

	avifb_set_gam_config(data);

	avifb_configure_timings(data);
	avi_lcd_pixclock_enable(data->lcdc);
	avi_segment_enable_irq(data->display_segment);
	avi_segment_activate(data->display_segment);

	dev_info(dev, "framebuffer reconfigured\n");

	return 0;
}

static struct dev_pm_ops avifb_dev_pm_ops = {
       .suspend = &avifb_suspend,
       .resume  = &avifb_resume,
};

static struct platform_driver avifb_driver = {
        .driver         = {
                .name   = DEVICE_NAME,
                .owner  = THIS_MODULE,
		.pm     = &avifb_dev_pm_ops,
        },
        .probe          = avifb_probe,
        .remove         = __devexit_p(avifb_remove),
};

static int __init avifb_init(void)
{
        return platform_driver_register(&avifb_driver);
}
module_init(avifb_init);

static void __exit avifb_exit(void)
{
        platform_driver_unregister(&avifb_driver);
}
module_exit(avifb_exit);

MODULE_AUTHOR("Lionel Flandrin <lionel.flandrin@parrot.com>");
MODULE_DESCRIPTION("Framebuffer driver for Parrot Advanced Video Interface");
MODULE_LICENSE("GPL");
