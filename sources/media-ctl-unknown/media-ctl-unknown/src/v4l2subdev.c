/*
 * V4L2 subdev interface library
 *
 * Copyright (C) 2010-2011 Ideas on board SPRL
 *
 * Contact: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <ctype.h>

#include <linux/v4l2-subdev.h>

#include "mediactl.h"
#include "v4l2subdev.h"
#include "tools.h"

int v4l2_subdev_open(struct media_entity *entity)
{
	if (entity->fd != -1)
		return 0;

	entity->fd = open(entity->devname, O_RDWR);
	if (entity->fd == -1) {
		media_dbg(entity->media,
			  "%s: Failed to open subdev device node %s\n", __func__,
			  entity->devname);
		return -errno;
	}

	return 0;
}

void v4l2_subdev_close(struct media_entity *entity)
{
	close(entity->fd);
	entity->fd = -1;
}

int v4l2_subdev_get_format(struct media_entity *entity,
	struct v4l2_mbus_framefmt *format, unsigned int pad,
	enum v4l2_subdev_format_whence which)
{
	struct v4l2_subdev_format fmt;
	int ret;

	ret = v4l2_subdev_open(entity);
	if (ret < 0)
		return ret;

	memset(&fmt, 0, sizeof(fmt));
	fmt.pad = pad;
	fmt.which = which;

	ret = ioctl(entity->fd, VIDIOC_SUBDEV_G_FMT, &fmt);
	if (ret < 0)
		return -errno;

	*format = fmt.format;
	return 0;
}

int v4l2_subdev_set_format(struct media_entity *entity,
	struct v4l2_mbus_framefmt *format, unsigned int pad,
	enum v4l2_subdev_format_whence which)
{
	struct v4l2_subdev_format fmt;
	int ret;

	ret = v4l2_subdev_open(entity);
	if (ret < 0)
		return ret;

	memset(&fmt, 0, sizeof(fmt));
	fmt.pad = pad;
	fmt.which = which;
	fmt.format = *format;

	ret = ioctl(entity->fd, VIDIOC_SUBDEV_S_FMT, &fmt);
	if (ret < 0)
		return -errno;

	*format = fmt.format;
	return 0;
}

int v4l2_subdev_get_crop(struct media_entity *entity, struct v4l2_rect *rect,
			 unsigned int pad, enum v4l2_subdev_format_whence which)
{
	struct v4l2_subdev_crop crop;
	int ret;

	ret = v4l2_subdev_open(entity);
	if (ret < 0)
		return ret;

	memset(&crop, 0, sizeof(crop));
	crop.pad = pad;
	crop.which = which;

	ret = ioctl(entity->fd, VIDIOC_SUBDEV_G_CROP, &crop);
	if (ret < 0)
		return -errno;

	*rect = crop.rect;
	return 0;
}

int v4l2_subdev_set_crop(struct media_entity *entity, struct v4l2_rect *rect,
			 unsigned int pad, enum v4l2_subdev_format_whence which)
{
	struct v4l2_subdev_crop crop;
	int ret;

	ret = v4l2_subdev_open(entity);
	if (ret < 0)
		return ret;

	memset(&crop, 0, sizeof(crop));
	crop.pad = pad;
	crop.which = which;
	crop.rect = *rect;

	ret = ioctl(entity->fd, VIDIOC_SUBDEV_S_CROP, &crop);
	if (ret < 0)
		return -errno;

	*rect = crop.rect;
	return 0;
}

int v4l2_subdev_set_routing(struct media_entity *entity, struct v4l2_subdev_routing *routing)
{
	int ret;

	ret = v4l2_subdev_open(entity);
	if (ret < 0)
		return ret;

	ret = ioctl(entity->fd, VIDIOC_SUBDEV_S_ROUTING, routing);
	if (ret < 0)
		return -errno;

	return 0;
}

int v4l2_subdev_get_frame_interval(struct media_entity *entity,
				   struct v4l2_fract *interval)
{
	struct v4l2_subdev_frame_interval ival;
	int ret;

	ret = v4l2_subdev_open(entity);
	if (ret < 0)
		return ret;

	memset(&ival, 0, sizeof(ival));

	ret = ioctl(entity->fd, VIDIOC_SUBDEV_G_FRAME_INTERVAL, &ival);
	if (ret < 0)
		return -errno;

	*interval = ival.interval;
	return 0;
}

int v4l2_subdev_set_frame_interval(struct media_entity *entity,
				   struct v4l2_fract *interval)
{
	struct v4l2_subdev_frame_interval ival;
	int ret;

	ret = v4l2_subdev_open(entity);
	if (ret < 0)
		return ret;

	memset(&ival, 0, sizeof(ival));
	ival.interval = *interval;

	ret = ioctl(entity->fd, VIDIOC_SUBDEV_S_FRAME_INTERVAL, &ival);
	if (ret < 0)
		return -errno;

	*interval = ival.interval;
	return 0;
}

static int v4l2_subdev_parse_format(struct v4l2_mbus_framefmt *format,
				    const char *p, char **endp)
{
	enum v4l2_mbus_pixelcode code;
	unsigned int width, height;
	char *end;

	for (; isspace(*p); ++p);
	for (end = (char *)p; !isspace(*end) && *end != '\0'; ++end);

	code = v4l2_subdev_string_to_pixelcode(p, end - p);
	if (code == (enum v4l2_mbus_pixelcode)-1)
		return -EINVAL;

	for (p = end; isspace(*p); ++p);
	width = strtoul(p, &end, 10);
	if (*end != 'x')
		return -EINVAL;

	p = end + 1;
	height = strtoul(p, &end, 10);

	memset(format, 0, sizeof(*format));
	format->width = width;
	format->height = height;
	format->code = code;

	if(*end == 'i') {
		format->field = V4L2_FIELD_INTERLACED;
		end++;
	}

	*endp = end;

	return 0;
}

static int v4l2_subdev_parse_crop(
	struct v4l2_rect *crop, const char *p, char **endp)
{
	char *end;

	if (*p++ != '(')
		return -EINVAL;

	crop->left = strtoul(p, &end, 10);
	if (*end != ',')
		return -EINVAL;

	p = end + 1;
	crop->top = strtoul(p, &end, 10);
	if ((*end != '/') && (*end++ != ')'))
		return -EINVAL;

	p = end + 1;
	crop->width = strtoul(p, &end, 10);
	if (*end != 'x')
		return -EINVAL;

	p = end + 1;
	crop->height = strtoul(p, &end, 10);
	if (*end++ != ')')
		return -EINVAL;
	*endp = end;

	return 0;
}

static int v4l2_subdev_parse_frame_interval(struct v4l2_fract *interval,
					    const char *p, char **endp)
{
	char *end;

	for (; isspace(*p); ++p);

	interval->numerator = strtoul(p, &end, 10);

	for (p = end; isspace(*p); ++p);
	if (*p++ != '/')
		return -EINVAL;

	for (; isspace(*p); ++p);
	interval->denominator = strtoul(p, &end, 10);

	*endp = end;
	return 0;
}

static int v4l2_subdev_parse_routing(struct v4l2_subdev_routing *routing,
				     const char *p, char **endp)
{
	char *end;

	for (; isspace(*p); ++p);

	routing->in = strtoul(p, &end, 10);

	for (p = end; isspace(*p); ++p);
	if (*p++ != ':')
		return -EINVAL;

	for (; isspace(*p); ++p);
	routing->out = strtoul(p, &end, 10);

	*endp = end;
	return 0;
}

static struct media_pad *v4l2_subdev_parse_pad_format(
	struct media_device *media, struct v4l2_mbus_framefmt *format,
	struct v4l2_rect *crop, struct v4l2_fract *interval, 
	struct v4l2_subdev_routing *routing, const char *p, char **endp)
{
	struct media_pad *pad;
	char *end;
	int ret;

	for (; isspace(*p); ++p);

	pad = media_parse_pad(media, p, &end);
	if (pad == NULL)
		return NULL;

	for (p = end; isspace(*p); ++p);
	if (*p++ != '[')
		return NULL;

	for (; isspace(*p); ++p);

	if (isalnum(*p)) {
		ret = v4l2_subdev_parse_format(format, p, &end);
		if (ret < 0)
			return NULL;

		for (p = end; isspace(*p); p++);
	}

	if (*p == '(') {
		ret = v4l2_subdev_parse_crop(crop, p, &end);
		if (ret < 0)
			return NULL;

		for (p = end; isspace(*p); p++);
	}

	if (*p == '@') {
		ret = v4l2_subdev_parse_frame_interval(interval, ++p, &end);
		if (ret < 0)
			return NULL;

		for (p = end; isspace(*p); p++);
	}

	if(*p == 'r') {
		ret = v4l2_subdev_parse_routing(routing, ++p, &end);
		if(ret < 0)
			return NULL;
		for (p = end; isspace(*p); p++);
	}

	if (*p != ']')
		return NULL;

	*endp = (char *)p + 1;
	return pad;
}

static int set_format(struct media_pad *pad,
		      struct v4l2_mbus_framefmt *format)
{
	int ret;

	if (format->width == 0 || format->height == 0)
		return 0;

	media_dbg(pad->entity->media,
		  "Setting up format %s %ux%u%s on pad %s/%u\n",
		  v4l2_subdev_pixelcode_to_string(format->code),
		  format->width, format->height,
                  (format->field==V4L2_FIELD_INTERLACED)?"i":"p",
		  pad->entity->info.name, pad->index);

	ret = v4l2_subdev_set_format(pad->entity, format, pad->index,
				     V4L2_SUBDEV_FORMAT_ACTIVE);
	if (ret < 0) {
		media_dbg(pad->entity->media,
			  "Unable to set format: %s (%d)\n",
			  strerror(-ret), ret);
		return ret;
	}

	media_dbg(pad->entity->media,
		  "Format set: %s %ux%u%s\n",
		  v4l2_subdev_pixelcode_to_string(format->code),
                  format->width, format->height,
                  (format->field==V4L2_FIELD_INTERLACED)?"i":"p");

	return 0;
}

static int set_crop(struct media_pad *pad, struct v4l2_rect *crop)
{
	int ret;

	if (crop->left == -1 || crop->top == -1)
		return 0;

	media_dbg(pad->entity->media,
		  "Setting up crop rectangle (%u,%u)/%ux%u on pad %s/%u\n",
		  crop->left, crop->top, crop->width, crop->height,
		  pad->entity->info.name, pad->index);

	ret = v4l2_subdev_set_crop(pad->entity, crop, pad->index,
				   V4L2_SUBDEV_FORMAT_ACTIVE);
	if (ret < 0) {
		media_dbg(pad->entity->media,
			  "Unable to set crop rectangle: %s (%d)\n",
			  strerror(-ret), ret);
		return ret;
	}

	media_dbg(pad->entity->media,
		  "Crop rectangle set: (%u,%u)/%ux%u\n",
		  crop->left, crop->top, crop->width, crop->height);

	return 0;
}

static int set_routing(struct media_entity *entity, struct v4l2_subdev_routing *routing)
{
	int ret;

	media_dbg(entity->media,
		  "Setting up routing in = %u, out = %d",
		  routing->in, routing->out);

	ret = v4l2_subdev_set_routing(entity, routing);
	if (ret < 0) {
		media_dbg(entity->media,
			  "Unable to set routing: %s (%d)\n",
			  strerror(-ret), ret);
		return ret;
	}

	media_dbg(entity->media,
		  "Routing set: in = %u, out = %u\n",
		  routing->in, routing->out);

	return 0;
}

static int set_frame_interval(struct media_entity *entity,
			      struct v4l2_fract *interval)
{
	int ret;

	if (interval->numerator == 0)
		return 0;

	media_dbg(entity->media,
		  "Setting up frame interval %u/%u on entity %s\n",
		  interval->numerator, interval->denominator,
		  entity->info.name);

	ret = v4l2_subdev_set_frame_interval(entity, interval);
	if (ret < 0) {
		media_dbg(entity->media,
			  "Unable to set frame interval: %s (%d)",
			  strerror(-ret), ret);
		return ret;
	}

	media_dbg(entity->media, "Frame interval set: %u/%u\n",
		  interval->numerator, interval->denominator);

	return 0;
}


static int v4l2_subdev_parse_setup_format(struct media_device *media,
					  const char *p, char **endp)
{
	struct v4l2_mbus_framefmt format = { .width=0, .height=0, .code=0, .field=V4L2_FIELD_NONE };
	struct media_pad *pad;
	struct v4l2_rect crop = { -1, -1, -1, -1 };
	struct v4l2_fract interval = { 0, 0 };
	struct v4l2_subdev_routing routing = { /*in*/0, /*out*/0, /*reserved[9]*/{0} };
	unsigned int i;
	char *end;
	int ret;

	pad = v4l2_subdev_parse_pad_format(media, &format, &crop, &interval,
					   &routing, p, &end);
	if (pad == NULL) {
		media_dbg(media, "Unable to parse format\n");
		return -EINVAL;
	}

	ret = set_routing(pad->entity, &routing);

	if (pad->flags & MEDIA_PAD_FL_SOURCE) {
		ret = set_crop(pad, &crop);
		if (ret < 0)
			return ret;
	}

	ret = set_format(pad, &format);
	if (ret < 0)
		return ret;

	if (pad->flags & MEDIA_PAD_FL_SINK) {
		ret = set_crop(pad, &crop);
		if (ret < 0)
			return ret;
	}

	ret = set_frame_interval(pad->entity, &interval);
	if (ret < 0)
		return ret;

	/* If the pad is an output pad, automatically set the same format on
	 * the remote subdev input pads, if any.
	 */
	if (pad->flags & MEDIA_PAD_FL_SOURCE) {
		for (i = 0; i < pad->entity->num_links; ++i) {
			struct media_link *link = &pad->entity->links[i];
			struct v4l2_mbus_framefmt remote_format;

			if (!(link->flags & MEDIA_LNK_FL_ENABLED))
				continue;

			if (link->source == pad &&
			    link->sink->entity->info.type == MEDIA_ENT_T_V4L2_SUBDEV) {
				remote_format = format;
				set_format(link->sink, &remote_format);
			}
		}
	}

	*endp = end;
	return 0;
}

int v4l2_subdev_parse_setup_formats(struct media_device *media, const char *p)
{
	char *end;
	int ret;

	do {
		ret = v4l2_subdev_parse_setup_format(media, p, &end);
		if (ret < 0)
			return ret;

		p = end + 1;
	} while (*end == ',');

	return *end ? -EINVAL : 0;
}

static struct {
	const char *name;
	enum v4l2_mbus_pixelcode code;
} mbus_formats[] = {
	{ "Y8", V4L2_MBUS_FMT_Y8_1X8},
	{ "Y10", V4L2_MBUS_FMT_Y10_1X10 },
	{ "Y12", V4L2_MBUS_FMT_Y12_1X12 },
	{ "YUYV", V4L2_MBUS_FMT_YUYV8_1X16 },
	{ "YUYV2X8", V4L2_MBUS_FMT_YUYV8_2X8 },
	{ "UYVY", V4L2_MBUS_FMT_UYVY8_1X16 },
	{ "UYVY2X8", V4L2_MBUS_FMT_UYVY8_2X8 },
	{ "SBGGR8", V4L2_MBUS_FMT_SBGGR8_1X8 },
	{ "SGBRG8", V4L2_MBUS_FMT_SGBRG8_1X8 },
	{ "SGRBG8", V4L2_MBUS_FMT_SGRBG8_1X8 },
	{ "SRGGB8", V4L2_MBUS_FMT_SRGGB8_1X8 },
	{ "SBGGR10", V4L2_MBUS_FMT_SBGGR10_1X10 },
	{ "SGBRG10", V4L2_MBUS_FMT_SGBRG10_1X10 },
	{ "SGRBG10", V4L2_MBUS_FMT_SGRBG10_1X10 },
	{ "SRGGB10", V4L2_MBUS_FMT_SRGGB10_1X10 },
	{ "SBGGR10_DPCM8", V4L2_MBUS_FMT_SBGGR10_DPCM8_1X8 },
	{ "SGBRG10_DPCM8", V4L2_MBUS_FMT_SGBRG10_DPCM8_1X8 },
	{ "SGRBG10_DPCM8", V4L2_MBUS_FMT_SGRBG10_DPCM8_1X8 },
	{ "SRGGB10_DPCM8", V4L2_MBUS_FMT_SRGGB10_DPCM8_1X8 },
	{ "SBGGR12", V4L2_MBUS_FMT_SBGGR12_1X12 },
	{ "SGBRG12", V4L2_MBUS_FMT_SGBRG12_1X12 },
	{ "SGRBG12", V4L2_MBUS_FMT_SGRBG12_1X12 },
	{ "SRGGB12", V4L2_MBUS_FMT_SRGGB12_1X12 },
};

const char *v4l2_subdev_pixelcode_to_string(enum v4l2_mbus_pixelcode code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mbus_formats); ++i) {
		if (mbus_formats[i].code == code)
			return mbus_formats[i].name;
	}

	return "unknown";
}

enum v4l2_mbus_pixelcode v4l2_subdev_string_to_pixelcode(const char *string,
							 unsigned int length)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mbus_formats); ++i) {
		if (strncmp(mbus_formats[i].name, string, length) == 0)
			break;
	}

	if (i == ARRAY_SIZE(mbus_formats))
		return (enum v4l2_mbus_pixelcode)-1;

	return mbus_formats[i].code;
}
