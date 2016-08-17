#ifndef _AVICAM_V4L2_H_
#define _AVICAM_V4L2_H_

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-core.h>

#include "avi_v4l2.h"
#include "avi_capture.h"
#include "avicam.h"

#ifdef CONFIG_AVICAM_USE_ISP
#include "avi_v4l2_isp.h"
#endif

#ifdef DEBUG
#define dprintk(avicam, format_, args...) \
	dev_dbg((avicam)->dev, "%s: " format_, __func__, ##args)

#define dprintk_l(avicam, format_, args...) do {		\
		if (printk_ratelimit())				\
			dprintk(avicam, format_, ##args);	\
	} while (0)

#else /* DEBUG */
#define dprintk(avicam_, format_, args...) (void)avicam_
#define dprintk_l(avicam_, format_, args...) (void)avicam_
#endif /* DEBUG */

#define DRIVER_NAME    "avicam"
#define DRIVER_VERSION KERNEL_VERSION(0, 2, 0)

struct avicam_vbuf
{
	/* *must* be first */
	struct vb2_buffer       vb;
	struct list_head        list;
	/* For interlaced streams we capture two fields in a single frame */
	int                     last_capture;
};

/* Context used when capturing bayer stats */
struct avicam_stats
{
        spinlock_t                       vbq_lock;
        struct list_head                 bufqueue;

        struct video_device             *vdev;
        struct vb2_queue                 vb_vidq;
        struct vb2_alloc_ctx            *alloc_ctx;

	/* serialization lock */
        struct mutex                     lock;
	int                              streaming;

	int                              use_count;
};

struct avicam
{
        struct device                   *dev;
        struct avi_capture_context       capture_ctx;
        struct avicam_platform_data     *pdata;

	spinlock_t                       vbq_lock;
	struct list_head                 bufqueue;

	struct v4l2_device              *v4l2_dev;
	struct v4l2_ctrl_handler         ctrl_handler;

	struct video_device             *vdev;
	struct vb2_queue                 vb_vidq;
	void                            *alloc_ctx;
	/* serialization lock */
	struct mutex                     lock;

	struct media_pad		 pad;
	struct media_pipeline		 pipe;

	int                              use_count;

	int                              streaming;

	unsigned                         frame_count;

	/* Output format */
        struct v4l2_pix_format           pix;
        struct v4l2_rect                 rect;
	struct v4l2_rect                 compose;

        struct avi_segment_format        segment_format;
        unsigned                         plane0_size;
        unsigned                         plane1_size;

	/* When storing interlaced video we have to store both fields in the
	 * same buffer */
	struct avicam_vbuf              *bottom_vbuf;

	struct avicam_stats              stats;

	struct platform_device          *dummy_pdev;

	struct pinctrl                  *pctl;

	/* Integrated V4L2 controls for ISP chain */
#ifdef CONFIG_AVICAM_USE_ISP
	struct avi_v4l2_isp             *ctrl_isp;
#endif
};

struct avicam_fh {
	struct v4l2_fh vfh;
};

#define to_avicam_fh(fh)	\
	container_of(fh, struct avicam_fh, vfh)

static inline int avicam_instance(struct avicam *avicam)
{
	struct platform_device *pdev;

	pdev = container_of(avicam->dev, struct platform_device, dev);

	return pdev->id;
}

static inline struct avicam_vbuf* to_avicam_vbuf(struct vb2_buffer* vb2)
{
	return container_of(vb2, struct avicam_vbuf, vb);
}

extern void avicam_stats_done(struct avi_capture_context *ctx,
                              struct avi_dma_buffer      *frame,
                              enum avi_field              f);

extern void avicam_stats_next(struct avi_capture_context *ctx,
                              struct avi_dma_buffer      *frame,
                              enum avi_field              f);



extern int  __devinit avicam_stats_init(struct avicam *avicam);
extern void __devexit avicam_stats_destroy(struct avicam *avicam);

#endif /* _AVICAM_V4L2_H_ */
