#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include <video/avi_pixfmt.h>
#include "avi_capture.h"

static 	struct avi_capture_context test_cap_ctx;

static void avitest_next(struct avi_capture_context	*ctx,
                         struct avi_dma_buffer		*frame,
                         enum avi_field			 f)
{
	/* hardcoded fb address */
	const dma_addr_t fb_addr = 0xbf000000;

	frame->plane0.dma_addr = fb_addr;

	if (f == AVI_BOT_FIELD)
		frame->plane0.dma_addr += 1920 * 2;
}

static int __init avitest_init(void)
{
	struct avi_capture_context	*ctx = &test_cap_ctx;
	struct avi_capture_crop          crop;
	int				 ret;
	struct avi_segment_format	 fmt;

	printk("probe\n");

	ret = avi_capture_init(ctx, NULL, 0,
	                       AVI_CAP_CAM_0 | AVI_CAP_CONV,
	                       0);
	if (ret) {
		printk("init failed\n");
		goto init_failed;
	}

	ctx->interface.itu656         = 1;
	ctx->interface.format_control = AVI_FORMAT_CONTROL_UYVY_2X8;
	ctx->interface.ipc            = 0;
	ctx->interface.pad_select     = 1;

	ctx->capture_cspace = AVI_BT709_CSPACE;

	ctx->interlaced = 0;

	/* Set frame timeout to 1second */
	ctx->timeout_jiffies = HZ;

	ret = avi_capture_prepare(ctx);
	if (ret) {
		printk("prepare failed\n");
		goto prepare_failed;
	}

	avi_capture_resolution(ctx, &fmt.width, &fmt.height);

#if 1
	crop.x = 0;
	crop.y = 0;

	crop.width  = 1280;
	crop.height = 720;

	ret = avi_capture_set_crop(ctx, &crop);
	if (ret) {
		printk("crop failed\n");
		goto crop_failed;
	}

	fmt.width  = crop.width;
	fmt.height = crop.height;
#endif

	fmt.colorspace	     = AVI_RGB_CSPACE;
	fmt.interlaced	     = ctx->interlaced;
	fmt.pix_fmt	     = AVI_PIXFMT_RGB565;
	fmt.plane0.line_size = fmt.width * avi_pixel_size0(fmt.pix_fmt);
	fmt.plane1.line_size = fmt.width * avi_pixel_size1(fmt.pix_fmt);

	if (fmt.interlaced) {
		fmt.plane0.line_size *= 2;
		fmt.plane1.line_size *= 2;
	}

	ret = avi_capture_set_format(ctx, &fmt);
	if (ret) {
		printk("set_format failed\n");
		goto set_format_failed;
	}

	ctx->next = &avitest_next;

	ret = avi_capture_streamon(ctx);
	if (ret) {
		printk("streamon failed\n");
		goto streamon_failed;
	}

	printk("probe ok. Resolution: %ux%u\n", fmt.width, fmt.height);

#if 0
	for (;;) {
		msleep(10);

		crop.x += 2;
		crop.y += 2;

		if ((ret = avi_capture_set_crop(ctx, &crop))) {
			printk("crop failed (%u, %u) %d\n", crop.x, crop.y, ret);
			crop.x = crop.y = 0;
		}
	}
#endif

	return 0;

 streamon_failed:
 set_format_failed:
 crop_failed:
 prepare_failed:
	avi_capture_destroy(&test_cap_ctx);
 init_failed:
	return ret;
}
module_init(avitest_init);

static void __exit avitest_exit(void)
{
	struct avi_capture_context	*ctx = &test_cap_ctx;
	int				 ret;

	ret = avi_capture_streamoff(ctx);
	if (ret)
		printk("streamoff failed [%d]\n", ret);

	avi_capture_destroy(&test_cap_ctx);
}
module_exit(avitest_exit);

MODULE_AUTHOR("Lionel Flandrin <lionel.flandrin@parrot.com>");
MODULE_DESCRIPTION("Test driver");
MODULE_LICENSE("GPL");
