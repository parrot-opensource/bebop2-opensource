/*
 * Lepton camera driver
 *
 * Copyright 2014 Parrot S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/videodev2.h>
#include <linux/hrtimer.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-dma-contig.h>

#include "lepton.h"

static int zero_copy = 0;
module_param(zero_copy, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(zero_copy, "hint to use zero copy mode if possible");

static int check_crc = 1;
module_param(check_crc, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(check_crc, "compute crc on every packet");

MODULE_DESCRIPTION("Lepton capture driver");
MODULE_AUTHOR("Damien Riegel <damien.riegel.ext@parrot.com>");
MODULE_LICENSE("GPL");

#define LEPTON_DRV_NAME "lepton"

#define LEPTON_SPI_XFER_SPEED      (20 * 1000 * 1000)
#define LEPTON_HEADER_LENGTH       (4)
#define LEPTON_DISCARD_PKT         (0x0FF0)
#define LEPTON_CRC_POLY            ((u32) 0x11021)
#define LEPTON_FRAME_TIMER_NS      (36000000)
#define LEPTON_RESYNCHRO_TIMER_NS  (200000000)
#define LEPTON_NB_VIDEO_PACKETS    (60)
#define LEPTON_FRAME_COUNT_OFFSET  (40)
#define LEPTON_NR_SPI_MESSAGES     (2)
#define LEPTON_SEND_NEXT_QUEUED_VB (0)
#define LEPTON_STOP_TRANSFER       (1)
#define LEPTON_MAX_FAILS           (40)

struct lepton {
	struct v4l2_device              v4l2_dev;

	struct video_device             *vdev;
	struct v4l2_subdev              *subdev;

	struct v4l2_ctrl_handler        ctrl_handler;

	void                            *alloc_ctx;

	struct spi_device               *spi;
	struct spi_message              spi_msg[LEPTON_NR_SPI_MESSAGES];
	struct spi_transfer             *spi_xfers;
	int                             n_pkts;

	void                            *virt_addr;
	dma_addr_t                      dma_addr;
	size_t                          buf_size;

	struct vb2_queue                vb2_queue;
	const struct lepton_format      *fmt;
	struct v4l2_pix_format          pix_fmt;

	struct list_head                vb_queued;

	unsigned long                   flags;

	struct mutex                    mutex;
	spinlock_t                      slock;

	struct hrtimer                  timer;
	ktime_t                         start_time;
	ktime_t                         frame_time;

	struct lepton_buffer            *buf;
	u32                             frame_count_current;
	u32                             frame_count_last;
};

struct lepton_buffer {
	struct vb2_buffer       vb;
	struct list_head        list;
	unsigned int            cnt_fail;
};

/*
 * format description
 *  zero_copy: this format can be used in zero copy mode. To compute this flag
 *  use this formula.
 *      line_per_pkt == 1 && (LEPTON_HEADER_LENGTH % (fmt->depth >> 3)) == 0
 *
 * Basically, it means that each line can be prefixed by its header and that
 * header can be consider as valid pixel(s). If any of these criterion is not
 * met, it means that we can't set correct values in the
 * v4l2_pix_format.bytesperline and/or width fields.
 */
struct lepton_format {
	char                    desc[32];
	u32                     v4l2_fmt;
	u32                     mbus_fmt;
	int                     depth;
	int                     line_per_pkt;
	bool                    zero_copy;
};

static struct lepton_format formats[] = {
	{
		.v4l2_fmt = V4L2_PIX_FMT_GREY,
		.mbus_fmt = V4L2_MBUS_FMT_Y8_1X8,
		.desc = "8bit Greyscale",
		.depth = 8,
		.line_per_pkt = 2,
		.zero_copy = false,
	},
	{
		.v4l2_fmt = V4L2_PIX_FMT_Y16,
		.mbus_fmt = V4L2_MBUS_FMT_Y14_2X8_PADHI_LE,
		.desc = "14bit Greyscale",
		.depth = 16,
		.line_per_pkt = 1,
		.zero_copy = true,
	},
	{
		.v4l2_fmt = V4L2_PIX_FMT_RGB24,
		.mbus_fmt = V4L2_MBUS_FMT_RGB888_1X24,
		.desc = "24bit RGB",
		.depth = 24,
		.line_per_pkt = 1,
		.zero_copy = false,
	},
};

#define LEPTON_DEBUG
#if defined(LEPTON_DEBUG)
#define dprintk(_dev, fmt_, args...) \
	dev_dbg((_dev)->v4l2_dev.dev, "%s: " fmt_, __func__, ##args)
#else
#define dprintk(_dev, fmt_, args...)
#endif

/*************************
 * Video buffer operations
 *************************/
static u16 calc_crc16(u8 *buf, size_t len)
{
	int i, j;
	u16 word, crc = 0;

	for(i = 0; i < len; i += 2) {
		word = buf[i] << 8 | buf[i + 1];

		crc ^= word;
		for(j = sizeof(crc) * 8; j; j--) {
			if (crc & 0x8000)
				crc = (crc << 1) ^ LEPTON_CRC_POLY;
			else
				crc = (crc << 1);
		}
	}

	return crc;
}

static bool lepton_check_crc(u8 *buf, size_t len)
{
	u16 expected_crc, calc_crc;
	u8 save[LEPTON_HEADER_LENGTH];

	/* this is very unlikely but let's prevent buffer overflow */
	if (len <= LEPTON_HEADER_LENGTH)
		return false;

	expected_crc = buf[2] << 8 | buf[3];

	/*
	 * CRC is computed on entire packet, including ID and CRC fields...
	 * For this computation, the T-number (some bits of the frame number)
	 * and the CRC are considered to be 0;
	 */
	memcpy(save, buf, LEPTON_HEADER_LENGTH);
	buf[0] &= 0x0F;
	buf[2] = buf[3] = 0;

	calc_crc = calc_crc16(buf, len);

	memcpy(buf, save, LEPTON_HEADER_LENGTH);

	return calc_crc == expected_crc;
}

static inline bool format_can_zero_copy(const struct lepton_format *lep_fmt)
{
	return zero_copy && lep_fmt->zero_copy;
}

static inline size_t spi_get_packet_length(struct lepton *lep)
{
	size_t len;

	/*
	 * If the format can be used in zero copy, then the
	 * header length is already included in the width field
	 * because it is considered to be valid data
	 */
	len = lep->pix_fmt.width * (lep->fmt->depth >> 3) *
		lep->fmt->line_per_pkt;
	if (!format_can_zero_copy(lep->fmt))
		len += LEPTON_HEADER_LENGTH;

	return len;
}

static inline size_t spi_align(size_t len)
{
	/*
	 * For SPI transfers, we must fetch one packet without pauses in
	 * the SPI clk. To follow this constraint, the P7 SPI master driver
	 * imposes that transfer starts aligned dma_get_cache_alignment().
	 * Wrap this constraint in this helper
	 */
	return roundup(len, dma_get_cache_alignment());
}

static void spi_fill_xfers(struct lepton *lep, void *virt, dma_addr_t dma)
{
	int                  pkt_len;
	int                  align_len;
	struct spi_transfer *xfer;

	pkt_len = spi_get_packet_length(lep);

	/* First line */
	xfer = lep->spi_xfers;
	xfer->len = pkt_len;
	xfer->rx_buf = virt;
	xfer->rx_dma = dma;

	/* Others packets */
	align_len = spi_align(pkt_len);

	xfer = lep->spi_xfers + 1;
	xfer->len = pkt_len * (lep->n_pkts - 1);
	xfer->rx_buf = virt + align_len;
	xfer->rx_dma = dma + align_len;
}

static int spi_async_first_line(struct lepton *lep, struct lepton_buffer *b)
{
	struct spi_message *msg = &lep->spi_msg[0];

	dprintk(lep, "buffer [%p] async first\n", b);
	msg->context = b;
	return spi_async(lep->spi, msg);
}

static int spi_async_remaining_lines(struct lepton *lep, struct lepton_buffer *b)
{
	struct spi_message *msg = &lep->spi_msg[1];

	dprintk(lep, "buffer [%p] async others\n", b);
	msg->context = b;
	return spi_async(lep->spi, msg);
}

static void spi_first_line_complete(void *context)
{
	struct lepton_buffer *buf = context;
	struct lepton *lep = vb2_get_drv_priv(buf->vb.vb2_queue);
	struct spi_transfer *xfer = lep->spi_xfers;
	u8 *pkt = (u8*)xfer->rx_buf;
	u16 pkt_nr = (pkt[0] & 0xF) << 8 | pkt[1];
	int err;

	if (test_bit(LEPTON_STOP_TRANSFER, &lep->flags)) {
		goto mark_buffer_error;
	}

	if (pkt_nr >= LEPTON_DISCARD_PKT ||
	    !lepton_check_crc(pkt, xfer->len)) {
		buf->cnt_fail++;
		if (buf->cnt_fail > LEPTON_MAX_FAILS) {
			dev_err(lep->v4l2_dev.dev, "TOO MUCH FAIL, RESET SENSOR\n");
			buf->cnt_fail = 0;
			/*
			 * Force re-synchronization :
			 * Deassert /CS and idle SCK for at least 5 frame periods (>185 msec)
			 */
			lep->buf = buf;
			hrtimer_start(&lep->timer, ktime_set(0, LEPTON_RESYNCHRO_TIMER_NS), HRTIMER_MODE_REL);
			return;
		}

		spi_async_first_line(lep, buf);
		return;
	}

	/*
	 * Save the start of the reception for configure the timer
	 * To receive the next frame
	 */
	lep->start_time = ktime_get();

	err = spi_async_remaining_lines(lep, buf);
	if (!err)
		return;

mark_buffer_error:
	vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	dprintk(lep, "buffer [%p] done\n", buf);
}

static void copy_to_vb_buffer(struct lepton *lep, struct lepton_buffer *buf)
{
	if (!format_can_zero_copy(lep->fmt)) {
		void *dst = vb2_plane_vaddr(&buf->vb, 0);
		void *src = lep->virt_addr;
		int pkt_len = spi_get_packet_length(lep);
		int align_len = spi_align(pkt_len);
		int copy_len = pkt_len - LEPTON_HEADER_LENGTH;
		int i;

		for (i = 0; i < lep->n_pkts; i++) {
			memcpy(dst, src + LEPTON_HEADER_LENGTH, copy_len);
			src += (i == 0) ? align_len : pkt_len;
			dst += copy_len;
		}
	}

	return;
}

/* Triggered by timer */
static enum hrtimer_restart lepton_timeout(struct hrtimer *timer)
{
	struct lepton *lep = container_of(timer, struct lepton, timer);

	spi_async_first_line(lep, lep->buf);

	return HRTIMER_NORESTART;
}

static void capture_frame_delay(struct lepton *lep, struct lepton_buffer *buf)
{
	if (format_can_zero_copy(lep->fmt)) {
		struct vb2_buffer *vb_buf = &buf->vb;

		/*
		 * If we support zero copy, receive sensor data directly in
		 * vb2_buffers. Prepare spi_transfers with the buffer addresses
		 * (dma and cpu).
		 */
		spi_fill_xfers(lep,
		               vb2_plane_vaddr(vb_buf, 0),
		               vb2_dma_contig_plane_dma_addr(vb_buf, 0));
	}

	lep->buf = buf;

	hrtimer_start(&lep->timer, ktime_add(lep->start_time, lep->frame_time), HRTIMER_MODE_ABS);
}

static int capture_frame(struct lepton *lep, struct lepton_buffer *buf)
{
	if (format_can_zero_copy(lep->fmt)) {
		struct vb2_buffer *vb_buf = &buf->vb;

		/*
		 * If we support zero copy, receive sensor data directly in
		 * vb2_buffers. Prepare spi_transfers with the buffer addresses
		 * (dma and cpu).
		 */
		spi_fill_xfers(lep,
		               vb2_plane_vaddr(vb_buf, 0),
		               vb2_dma_contig_plane_dma_addr(vb_buf, 0));
	}

	lep->buf = buf;

	return spi_async_first_line(lep, buf);
}

static int get_telemetry_from_sd(struct lepton *lep, int *val)
{
	struct v4l2_streamparm param;
	int err;

	err = v4l2_subdev_call(lep->subdev, video, g_parm, &param);
	if (err)
		return err;

	*val = param.parm.raw_data[0];

	/*
	 * 0 = telemetry disabled
	 * 1 = telemetry as header
	 * 2 = telemetry as footer
	 */

	return 0;
}

static void spi_frame_complete(void *context)
{
	struct lepton_buffer  *next = NULL, *buf = context;
	struct lepton         *lep = vb2_get_drv_priv(buf->vb.vb2_queue);
	int                   status = VB2_BUF_STATE_DONE;
	struct spi_transfer   *xfer = lep->spi_xfers + 1;
	int                   pkt_len = spi_get_packet_length(lep);
	int                   telemetry = 0;
	u8                    skip_frame = 0;
	bool                  crc = 1;
	int                   i;

	/*
	 * We skip duplicated frame when :
	 * - telemetry is enabled (to get frame count)
	 * - telemetry is set as footer
	 */
	if (get_telemetry_from_sd(lep, &telemetry) == 0 && (telemetry == 2)) {
		/*
		 * We look for the frame count, in telemetry lines (3 last lines)
		 * Total packets : 60 (video) + 3 (telemetry)
		 * We look to 59th position because it is on second transfert
		 * Counter (32 bits) begin to 20th 16 bits word
		 */

		int  frameOffset;
		u8   *data = (u8 *) xfer->rx_buf;

		skip_frame = 1;

		frameOffset = pkt_len * (LEPTON_NB_VIDEO_PACKETS-1) + LEPTON_HEADER_LENGTH + LEPTON_FRAME_COUNT_OFFSET;

		lep->frame_count_current = (data[frameOffset+2] << 24) +
			(data[frameOffset+3] << 16) +
			(data[frameOffset+0] << 8) +
			data[frameOffset+1];

		/*
		 * Each frame is duplicated 2 times
		 * Send to user only one
		 */
		if (lep->frame_count_current == lep->frame_count_last) {
			capture_frame_delay(lep, buf);
			return;
		}
	}

	spin_lock(&lep->slock);
	/*
	 * Try to get the next buffer to be sent. If there are no pending
	 * buffers, set a flag to send the next queued one (in buf_queue).
	 * Don't set this flag if we must stop capturing.
	 */
	if (!list_empty(&lep->vb_queued)) {
		next = list_first_entry(&lep->vb_queued,
				       struct lepton_buffer, list);
		list_del(&next->list);
	}
	else if (!test_bit(LEPTON_STOP_TRANSFER, &lep->flags))  {
		set_bit(LEPTON_SEND_NEXT_QUEUED_VB, &lep->flags);
		dprintk(lep, "setting send next queued vb flag\n");
	}
	spin_unlock(&lep->slock);

	if (check_crc) {
		/*
		 * Check crc on second transfert
		 * First line already received and crc checked
		 */
		for(i = 0; i < lep->n_pkts - 1; i++) {
			crc = lepton_check_crc(xfer->rx_buf + pkt_len * i, pkt_len);

			if (!crc) {
				dev_err(lep->v4l2_dev.dev, "CRC error on frame %u\n",
					(telemetry == 2) ? lep->frame_count_current : 0);
				status = VB2_BUF_STATE_ERROR;
				break;
			}
		}
	}

	if (skip_frame && crc)
		lep->frame_count_last = lep->frame_count_current;

	copy_to_vb_buffer(lep, buf);
	v4l2_get_timestamp(&buf->vb.v4l2_buf.timestamp);
	vb2_buffer_done(&buf->vb, status);

	if (next)
		capture_frame_delay(lep, next);
	else
		dev_err(lep->v4l2_dev.dev, "No buffer available !\n");

}

static int spi_init_messages(struct lepton *lep)
{
	struct spi_message *msg;
	struct spi_transfer *xfers;
	int n_pkts = lep->pix_fmt.height / lep->fmt->line_per_pkt;

	if (lep->spi_xfers)
		return 0;

	xfers = kcalloc(LEPTON_NR_SPI_MESSAGES, sizeof(struct spi_transfer), GFP_KERNEL);
	if (!xfers)
		return -ENOMEM;

	lep->spi_xfers = xfers;
	lep->n_pkts = n_pkts;

	msg = &lep->spi_msg[0];
	spi_message_init(msg);
	msg->complete = spi_first_line_complete;
	msg->is_dma_mapped = true;
	xfers->speed_hz = LEPTON_SPI_XFER_SPEED;
	spi_message_add_tail(xfers, msg);

	xfers++;

	msg = &lep->spi_msg[1];
	spi_message_init(msg);
	msg->complete = spi_frame_complete;
	msg->is_dma_mapped = true;
	xfers->speed_hz = LEPTON_SPI_XFER_SPEED;
	spi_message_add_tail(xfers, msg);

	return 0;
}

static void spi_free_messages(struct lepton *lep)
{
	kfree(lep->spi_xfers);
	lep->spi_xfers = NULL;
}

static int queue_setup(struct vb2_queue *q, const struct v4l2_format *fmt,
                       unsigned int *num_buffers, unsigned int *num_planes,
                       unsigned int sizes[], void *alloc_ctxs[])
{
	struct lepton *lep = vb2_get_drv_priv(q);
	unsigned int size;

	size = lep->pix_fmt.sizeimage;

	if (*num_buffers == 0)
		*num_buffers = 2;
	*num_planes = 1;

	sizes[0] = size;
	alloc_ctxs[0] = lep->alloc_ctx;

	dprintk(lep, "count=%d size=%u\n",
	        *num_buffers, size);

	return 0;
}

static void wait_prepare(struct vb2_queue *q)
{
	struct lepton *lep = vb2_get_drv_priv(q);
	mutex_unlock(&lep->mutex);
}

static void wait_finish(struct vb2_queue *q)
{
	struct lepton *lep = vb2_get_drv_priv(q);
	mutex_lock(&lep->mutex);
}

static int buf_init(struct vb2_buffer *vb)
{
	struct lepton *lep = vb2_get_drv_priv(vb->vb2_queue);
	dprintk(lep, "buf init\n");
	return 0;
}

static int buf_prepare(struct vb2_buffer *vb)
{
	struct lepton *lep = vb2_get_drv_priv(vb->vb2_queue);
	struct lepton_buffer *buf = container_of(vb, struct lepton_buffer, vb);
	unsigned int size = lep->pix_fmt.sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		dprintk(lep, "data will not fit into plane (%lu < %u)\n",
		        vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);

	buf->cnt_fail = 0;

	dprintk(lep, "buffer [%p] prepared\n", buf);

	return 0;
}

static int buf_finish(struct vb2_buffer *vb)
{
	struct lepton *lep = vb2_get_drv_priv(vb->vb2_queue);
	dprintk(lep, "buf finish\n");
	return 0;
}

static void buf_cleanup(struct vb2_buffer *vb)
{
	struct lepton *lep = vb2_get_drv_priv(vb->vb2_queue);
	dprintk(lep, "buf cleanup\n");
}

static int start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct lepton *lep = vb2_get_drv_priv(q);
	int err;

	err = spi_init_messages(lep);
	if (err)
		return err;

	if (!format_can_zero_copy(lep->fmt)) {
		/*
		 * If the format is not compatible with zero copy,
		 * we must use an intermediate buffer to receive data and
		 * then copy them in the vb2_buffer
		 */
		void *cpu;
		dma_addr_t dma;
		size_t size = spi_get_packet_length(lep);

		size = spi_align(size);
		size += (lep->n_pkts - 1) * (spi_get_packet_length(lep));

		cpu = dma_alloc_coherent(NULL, size, &dma, GFP_DMA);
		if (!cpu) {
			spi_free_messages(lep);
			return -ENOMEM;
		}

		lep->virt_addr = cpu;
		lep->dma_addr = dma;
		lep->buf_size = size;

		spi_fill_xfers(lep, cpu, dma);
	}

	spin_lock(&lep->slock);
	lep->flags = 0;
	if (count == 0) {
		set_bit(LEPTON_SEND_NEXT_QUEUED_VB, &lep->flags);
		dprintk(lep, "setting next queue vb flag, f 0x%.8lX\n", lep->flags);
	}
	else {
		struct lepton_buffer *buf;
		buf = list_first_entry(&lep->vb_queued,
				       struct lepton_buffer, list);

		list_del(&buf->list);
		capture_frame(lep, buf);
		dprintk(lep, "start capture frame\n");
	}
	spin_unlock(&lep->slock);

	dprintk(lep, "streaming started\n");

	return 0;
}

static int stop_streaming(struct vb2_queue *q)
{
	struct lepton *lep = vb2_get_drv_priv(q);

	set_bit(LEPTON_STOP_TRANSFER, &lep->flags);

	spin_lock(&lep->slock);
	while (!list_empty(&lep->vb_queued)) {
		struct lepton_buffer *buf;

		buf = list_first_entry(&lep->vb_queued,
				       struct lepton_buffer, list);
		list_del(&buf->list);

		spin_unlock(&lep->slock);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
		dprintk(lep, "buffer [%p] done\n", buf);
		spin_lock(&lep->slock);
	}
	spin_unlock(&lep->slock);

	/*
	 * wait for the buffer being filled
	 * by the SPI to be returned
	 */
	vb2_wait_for_all_buffers(q);
	if (!format_can_zero_copy(lep->fmt)) {
		dma_free_coherent(NULL, lep->buf_size,
		                  lep->virt_addr, lep->dma_addr);
	}
	lep->flags = 0;
	spi_free_messages(lep);
	dprintk(lep, "streaming stopped\n");

	return 0;
}

static void buf_queue(struct vb2_buffer *vb)
{
	struct lepton *lep = vb2_get_drv_priv(vb->vb2_queue);
	struct lepton_buffer *buf = container_of(vb, struct lepton_buffer, vb);

	dprintk(lep, "queue buf [%p]\n", buf);

	spin_lock(&lep->slock);
	if (test_bit(LEPTON_SEND_NEXT_QUEUED_VB, &lep->flags)) {
		clear_bit(LEPTON_SEND_NEXT_QUEUED_VB, &lep->flags);
		capture_frame(lep, buf);
		dprintk(lep, "start capture frame, f 0x%.8lX\n", lep->flags);
	}
	else {
		list_add_tail(&buf->list, &lep->vb_queued);
	}
	spin_unlock(&lep->slock);
}

static struct vb2_ops lepton_videobuf_ops = {
	.queue_setup            = queue_setup,
	.wait_prepare           = wait_prepare,
	.wait_finish            = wait_finish,
	.buf_init               = buf_init,
	.buf_prepare            = buf_prepare,
	.buf_finish             = buf_finish,
	.buf_cleanup            = buf_cleanup,
	.start_streaming        = start_streaming,
	.stop_streaming         = stop_streaming,
	.buf_queue              = buf_queue,
};

/***********************
 * lepton file operations
 ***********************/
static const struct v4l2_file_operations lepton_fops = {
	.owner          = THIS_MODULE,
	.open           = v4l2_fh_open,
	.release        = vb2_fop_release,
	.read           = vb2_fop_read,
	.mmap           = vb2_fop_mmap,
	.poll           = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
};

/*************
 * V4L2 IOCTLs
 *************/
static int vidioc_querycap(struct file *file, void *fh,
                           struct v4l2_capability *cap)
{
	struct lepton *lep = video_drvdata(file);

	strcpy(cap->driver, LEPTON_DRV_NAME);
	strcpy(cap->card, LEPTON_DRV_NAME);
	strlcpy(cap->bus_info, lep->v4l2_dev.name, sizeof(cap->bus_info));
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
	                   V4L2_CAP_READWRITE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void *fh,
                                   struct v4l2_fmtdesc *f)
{
	const struct lepton_format *fmt;

	if (f->index >= ARRAY_SIZE(formats))
		return -EINVAL;

	fmt = &formats[f->index];

	f->flags = 0;
	f->pixelformat = fmt->v4l2_fmt;
	strlcpy(f->description, fmt->desc, sizeof(f->description));

	return 0;
}

static const struct lepton_format* get_format(struct v4l2_format *f)
{
	struct lepton_format *fmt;
	int i;

	for (i = 0; i < ARRAY_SIZE(formats); i++) {
		fmt = &formats[i];
		if (fmt->v4l2_fmt == f->fmt.pix.pixelformat)
			break;
	}

	if (i == ARRAY_SIZE(formats))
		i = 0; /* default to first format */

	return &formats[i];
}

static const struct lepton_format* get_format_from_mbus(u32 mbus)
{
	struct lepton_format *fmt;
	int i;

	for (i = 0; i < ARRAY_SIZE(formats); i++) {
		fmt = &formats[i];
		if (fmt->mbus_fmt == mbus)
			break;
	}

	if (i == ARRAY_SIZE(formats))
		return NULL;

	return &formats[i];
}

static void lepton_fill_pix_format(const struct lepton_format *lepfmt,
                                   struct v4l2_pix_format *pixfmt,
                                   const struct v4l2_mbus_framefmt *mbusfmt)
{
	v4l2_fill_pix_format(pixfmt, mbusfmt);
	pixfmt->pixelformat = lepfmt->v4l2_fmt;

	if (format_can_zero_copy(lepfmt)) {
		/*
		 * In zero copy mode, we consider the header as being part of
		 * the data because there is no way to cut it out without
		 * memcpy. The V4L2 layer doesn't support left padding.
		 */
		pixfmt->width += LEPTON_HEADER_LENGTH / (lepfmt->depth >> 3);
		pixfmt->bytesperline =
			spi_align(pixfmt->width * (lepfmt->depth >> 3));
	}
	else {
		pixfmt->bytesperline =
			pixfmt->width * (lepfmt->depth >> 3);
	}

	pixfmt->sizeimage =
		pixfmt->height * pixfmt->bytesperline;
}

static int get_pixfmt_from_sd(struct lepton *lep, struct v4l2_pix_format *pixfmt)
{
	struct v4l2_mbus_framefmt sub_fmt;
	const struct lepton_format *fmt;
	int err;

	err = v4l2_subdev_call(lep->subdev, video, g_mbus_fmt, &sub_fmt);
	if (err)
		return err;

	fmt = get_format_from_mbus(sub_fmt.code);
	if (!fmt)
		return -EINVAL;

	lepton_fill_pix_format(fmt, pixfmt, &sub_fmt);

	return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *fh,
                                struct v4l2_format *f)
{
	struct lepton *lep = video_drvdata(file);
	struct v4l2_pix_format pixfmt = {0};
	int err;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	err = get_pixfmt_from_sd(lep, &pixfmt);
	if (err)
		return err;

	f->fmt.pix = lep->pix_fmt = pixfmt;
	return 0;
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *fh,
                                  struct v4l2_format *f)
{
	struct lepton *lep = video_drvdata(file);
	const struct lepton_format *fmt;
	struct v4l2_pix_format *pixfmt = &f->fmt.pix;
	struct v4l2_mbus_framefmt sub_fmt;
	int err;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	fmt = get_format(f);

	v4l2_fill_mbus_format(&sub_fmt, pixfmt, fmt->mbus_fmt);
	err = v4l2_subdev_call(lep->subdev, video, try_mbus_fmt, &sub_fmt);
	if (err)
		return err;

	if (sub_fmt.code != fmt->mbus_fmt) {
		/* subdevice doesn't want this format, damn... */
		fmt = get_format_from_mbus(sub_fmt.code);
		if (!fmt)
			return -EINVAL;
	}

	lepton_fill_pix_format(fmt, pixfmt, &sub_fmt);

	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *fh,
                                struct v4l2_format *f)
{
	struct lepton *lep = video_drvdata(file);
	struct vb2_queue *q = &lep->vb2_queue;
	struct v4l2_mbus_framefmt sub_fmt = {0};
	const struct lepton_format *lep_fmt;
	int err;

	if (vb2_is_streaming(q))
		return -EBUSY;

	err = vidioc_try_fmt_vid_cap(file, fh, f);
	if (err < 0)
		return err;

	lep_fmt = get_format(f);
	v4l2_fill_mbus_format(&sub_fmt, &f->fmt.pix, lep_fmt->mbus_fmt);
	err = v4l2_subdev_call(lep->subdev, video, s_mbus_fmt, &sub_fmt);
	if (err)
		return err;

	if (sub_fmt.code != lep_fmt->mbus_fmt) {
		lep_fmt = get_format_from_mbus(sub_fmt.code);
		if (!lep_fmt)
			return -EINVAL;
	}

	lepton_fill_pix_format(lep_fmt, &f->fmt.pix, &sub_fmt);

	lep->fmt = lep_fmt;
	lep->pix_fmt = f->fmt.pix;
	return 0;
}

static int vidioc_enum_input(struct file *file, void *fh,
                             struct v4l2_input *inp)
{
	if (inp->index != 0)
		return -EINVAL;

	inp->type = V4L2_INPUT_TYPE_CAMERA;
	inp->std = V4L2_STD_UNKNOWN;
	strlcpy(inp->name, "Camera", sizeof(inp->name));

	return 0;
}

static int vidioc_g_input(struct file *file, void *fh, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int vidioc_s_input(struct file *file, void *fh, unsigned int i)
{
	if (i > 0)
		return -EINVAL;

	return 0;
}

int vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct lepton *lep = video_drvdata(file);
	struct v4l2_pix_format fmt;
	int err;

	/*
	 * starting streaming on the subdev locks the telemetry
	 * control, preventing any change from happening. We can
	 * then check that subdev and device formats are the same
	 */
	err = v4l2_subdev_call(lep->subdev, video, s_stream, 1);
	if (err)
		goto error;

	err = get_pixfmt_from_sd(lep, &fmt);
	if (err)
		goto stop_stream;

	if (fmt.width != lep->pix_fmt.width ||
	    fmt.height != lep->pix_fmt.height ||
	    fmt.pixelformat != lep->pix_fmt.pixelformat ||
	    fmt.field != lep->pix_fmt.field ||
	    fmt.bytesperline != lep->pix_fmt.bytesperline ||
	    fmt.sizeimage != lep->pix_fmt.sizeimage ||
	    fmt.colorspace != lep->pix_fmt.colorspace) {
		err = -EPIPE;
		goto stop_stream;
	}

	err = vb2_ioctl_streamon(file, priv, i);
	if (err)
		goto stop_stream;

	return 0;

stop_stream:
	v4l2_subdev_call(lep->subdev, video, s_stream, 0);
error:
	return err;
}

int vidioc_streamoff(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct lepton *lep = video_drvdata(file);

	hrtimer_cancel(&lep->timer);

	v4l2_subdev_call(lep->subdev, video, s_stream, 0);
	return vb2_ioctl_streamoff(file, fh, i);
}

static const struct v4l2_ioctl_ops lepton_ioctl_ops = {
	.vidioc_querycap                = vidioc_querycap,
	.vidioc_enum_fmt_vid_cap        = vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap           = vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap         = vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap           = vidioc_s_fmt_vid_cap,
	//.vidioc_s_std                 = vidioc_s_std,
	.vidioc_enum_input              = vidioc_enum_input,
	.vidioc_g_input                 = vidioc_g_input,
	.vidioc_s_input                 = vidioc_s_input,
	.vidioc_reqbufs                 = vb2_ioctl_reqbufs,
	.vidioc_querybuf                = vb2_ioctl_querybuf,
	.vidioc_qbuf                    = vb2_ioctl_qbuf,
	.vidioc_dqbuf                   = vb2_ioctl_dqbuf,
	.vidioc_streamon                = vidioc_streamon,
	.vidioc_streamoff               = vidioc_streamoff,
};

/**************************
 * module setup and cleanup
 **************************/
static int lepton_init_ctrl(struct lepton *lep)
{
	lep->v4l2_dev.ctrl_handler = &lep->ctrl_handler;
	return v4l2_ctrl_handler_init(&lep->ctrl_handler, 0);
}

static void lepton_free_ctrl(struct lepton *lep)
{
	v4l2_ctrl_handler_free(&lep->ctrl_handler);
}

static int lepton_probe(struct spi_device *spi)
{
	struct lepton *lep;
	struct video_device *vdev;
	struct v4l2_subdev *subdev;
	struct vb2_queue *vb2_q;
	struct lepton_platform_data *data;
	struct i2c_adapter *adapter;
	struct vb2_dc_conf *alloc_ctx;
	int err;

	data = spi->dev.platform_data;
	if (!data)
		return -EINVAL;

	lep = kzalloc(sizeof(struct lepton), GFP_KERNEL);
	if (!lep)
		return -ENOMEM;

	mutex_init(&lep->mutex);
	spin_lock_init(&lep->slock);
	INIT_LIST_HEAD(&lep->vb_queued);

	err = -ENOMEM;
	alloc_ctx = (struct vb2_dc_conf *)vb2_dma_contig_init_ctx(&spi->dev);
	if (!alloc_ctx)
		goto free_lep;

	alloc_ctx->cache_flags = data->vb2_cache_flags;

	lep->alloc_ctx = alloc_ctx;

	err = lepton_init_ctrl(lep);
	if (err)
		goto free_alloc_ctx;

	spi_set_drvdata(spi, lep);
	err = v4l2_device_register(&spi->dev, &lep->v4l2_dev);
	if (err)
		goto free_ctrl;

	err = -ENODEV;
	adapter = i2c_get_adapter(data->i2c_adapter_nr);
	if (!adapter)
		goto unregister_v4l2_dev;
	subdev = v4l2_i2c_new_subdev_board(&lep->v4l2_dev, adapter,
	                                   &data->board_info, NULL);
	if (!subdev)
		goto put_i2c_adapter;
	lep->subdev = subdev;

	err = v4l2_device_register_subdev_nodes(&lep->v4l2_dev);
	if (err)
		goto put_i2c_adapter;

	vb2_q = &lep->vb2_queue;
	memset(vb2_q, 0, sizeof(*vb2_q));
	vb2_q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vb2_q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_READ; /* XXX: change that ? */
	vb2_q->drv_priv = lep;
	vb2_q->buf_struct_size = sizeof(struct lepton_buffer);
	vb2_q->ops = &lepton_videobuf_ops;
	vb2_q->mem_ops = &vb2_dma_contig_memops;
	vb2_q->lock = &lep->mutex;
	vb2_queue_init(vb2_q);

	err = -ENOMEM;
	vdev = video_device_alloc();
	if (!vdev)
		goto put_i2c_adapter;

	strlcpy(vdev->name, "lepton", sizeof(vdev->name)); /* XXX: change that ? */
	vdev->v4l2_dev = &lep->v4l2_dev;
	vdev->release = video_device_release;
	vdev->fops = &lepton_fops;
	vdev->ioctl_ops = &lepton_ioctl_ops;
	vdev->current_norm = V4L2_STD_UNKNOWN;
	vdev->tvnorms = V4L2_STD_UNKNOWN;
	vdev->lock = &lep->mutex;
	vdev->queue = vb2_q;

	video_set_drvdata(vdev, lep);

	err = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (err < 0)
		goto release_vdev;

	get_pixfmt_from_sd(lep, &lep->pix_fmt);

	v4l2_info(&lep->v4l2_dev, "V4L2 device registered as %s\n",
		  video_device_node_name(vdev));

	lep->vdev = vdev;
	lep->spi = spi;

	hrtimer_init(&lep->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	lep->timer.function = lepton_timeout;
	lep->frame_time = ktime_set(0, LEPTON_FRAME_TIMER_NS);

	lep->frame_count_current = 0;
	lep->frame_count_last = 0;

	return 0;

release_vdev:
	video_device_release(vdev);
put_i2c_adapter:
	i2c_put_adapter(adapter);
unregister_v4l2_dev:
	v4l2_device_unregister(&lep->v4l2_dev);
free_ctrl:
	lepton_free_ctrl(lep);
free_alloc_ctx:
	vb2_dma_contig_cleanup_ctx(lep->alloc_ctx);
free_lep:
	kfree(lep);
	return err;
}

static int lepton_remove(struct spi_device *spi)
{
	struct lepton *lep;

	lep = spi_get_drvdata(spi);

	video_unregister_device(lep->vdev);
	v4l2_device_unregister(&lep->v4l2_dev);
	lepton_free_ctrl(lep);
	vb2_dma_contig_cleanup_ctx(lep->alloc_ctx);
	kfree(lep);
	return 0;
}

static struct spi_driver lepton_driver = {
	.driver		= {
		.name   = LEPTON_DRV_NAME,
		.owner  = THIS_MODULE,
	},
	.probe          = lepton_probe,
	.remove         = __devexit_p(lepton_remove),
};

module_spi_driver(lepton_driver);

