/*
 *      linux/drivers/parrot/video/avi_voc_core.c
 *
 *      Copyright (C) 2011 Parrot S.A.
 *
 * @author  Victor Lambret <victor.lambret.ext@parrot.com>
 * @date    03-Mar-2011
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

#ifndef _AVI_VOC_CORE_H_
#define _AVI_VOC_CORE_H_

#include <linux/list.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/gcd.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-common.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-ctrls.h>

#include "avi_v4l2.h"
#include "video/avi_limit.h"
#include "avi_voc.h"

#define DRIVER_NAME    "avi-voc"

#define AVI_VOC_ENUM_NB         3
#define AVI_VOC_MIN_BUFFERS     3

#ifdef DEBUG
#define dprintk(voc, format_, args...) \
	dev_dbg((voc)->dev, "%s: " format_, __func__, ##args)

#else /* DEBUG */
#define dprintk(voc_, format_, args...) (void)voc_
#endif /* DEBUG */


struct avi_voc_vbuf {
	/* common v4l buffer stuff -- must be first */
	struct vb2_buffer       vb;
	struct list_head        list;
	int in_queue;
};

struct avi_voc {
	struct device                   *dev;
	/* V4L2 video device - keep first for cast purpose ! */
	struct video_device             *vdev;

	struct v4l2_device              *v4l2_dev;

	struct avi_segment              *display;

	/* This lock protects the bufqueue, the displayed and expired frame
	 * pointers as well as the pix, win and crop configs */
	spinlock_t                       vbuf_lock;


	/* The overlay format selection is a bit tricky. A picture being worth a
	 * thousand words here's how it works:
	 *
	 *            Input frame                             Display
	 * ----------------------------------       ---------------------------
	 * |.pix.(fmt_vid_out)..............|       |~Display~segment~format~~|
	 * |................................|       |~~~~~~~~~~~~~~~~~~~~~~~~~|
	 * |................................|       |~~~~~~~~~~~~~~~~~~~~~~~~~|
	 * |....________________________....|       |~~_______________~~~~~~~~|
	 * |...| win (fmt_vid_overlay)  |...|       |~| crop (crop)   |~~~~~~~|
	 * |...|                        |...|  voc  |~|               |~~~~~~~|
	 * |...|                        |============>|               |~~~~~~~|
	 * |...|________________________|...|       |~|               |~~~~~~~|
	 * |................................|       |~|_______________|~~~~~~~|
	 * ----------------------------------       |~~~~~~~~~~~~~~~~~~~~~~~~~|
	 *                                          |~~~~~~~~~~~~~~~~~~~~~~~~~|
	 *                                          ---------------------------
	 */
	/* format of the input frame */
	struct v4l2_pix_format           pix;
	/* window within the input frame that we want to display */
	struct v4l2_window               win;
	/* position and size within the output window */
        struct v4l2_crop                 crop;

	struct vb2_queue                 vb2_vidq;
	struct list_head                 bufqueue;
	struct avi_voc_vbuf             *displayed_frame;
	struct avi_voc_vbuf             *expired_frame;
	/* serialization lock */
	struct mutex                     lock;
	/* VB2 callback lock.
	 * XXX: Have to be merge with another lock : not in vivi */
	struct mutex                     vb2_lock;

	/* Used when waiting for streamoff */
	wait_queue_head_t                waitq;
	/* frame period in microsecond. Updated in user context, read in
	 * interrupt context. To save us some locking I made this variable
	 * atomic */
	atomic_t                         period_us;
	unsigned                         frame_count;

	struct avi_segment              *segment;
	unsigned                         plane0_size;
	unsigned                         plane1_size;
	int                              user_count;

	/* Used for outputs enumerations */
	int                              out_nb;
	int                              out_id;
	u8                               output[32];
	u8                               outputs[2*AVI_VOC_ENUM_NB][32];

	/* Controls */
	struct v4l2_ctrl_handler         ctrl_handler;
	int                              no_scaler;
	int                              zorder;
	struct v4l2_ctrl                 *hide_ctrl;
	int                              hide;
	int                              hw_streaming;
};

struct avi_voc_fh {
	struct v4l2_fh vfh;
};

#endif
