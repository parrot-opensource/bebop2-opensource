/*
 *      linux/drivers/parrot/video/avi_voc_ctrl.c
 *
 *      Copyright (C) 2014 Parrot S.A.
 *
 * @author  Victor Lambret <victor.lambret.ext@parrot.com>
 * @date    20-Oct-2014
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

#include "avi_voc_ctrl.h"

static int avi_voc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct avi_voc *voc = container_of(ctrl->handler,
	                                   struct avi_voc,
	                                   ctrl_handler);
	int rt = 0;

	mutex_lock(&voc->lock);

	dprintk(voc, "id=%x, value=%d\n", ctrl->id, ctrl->val);

	switch (ctrl->id) {
	case V4L2_CID_VOC_NO_SCALER:
		if (voc->segment)
			rt = -EBUSY;
		else
			voc->no_scaler = ctrl->val;
		break;
	case V4L2_CID_VOC_ZORDER:
		if (voc->segment)
			rt = -EBUSY;
		else
			voc->zorder = ctrl->val;
		break;
	case V4L2_CID_VOC_HIDE:
		spin_lock_irq(&voc->vbuf_lock);
		dprintk(voc, "voc->hide=%d, voc->hw_streaming=%d\n",
		        voc->hide, voc->hw_streaming);
		voc->hide = ctrl->val;
		if (voc->hide && voc->hw_streaming)
			avi_segment_hide(voc->segment);
		else if (!voc->hide && voc->hw_streaming)
			avi_segment_unhide(voc->segment);
		spin_unlock_irq(&voc->vbuf_lock);
		break;
	default:
		dev_err(voc->dev, "Invalid set of control=%08x\n", ctrl->id);
		rt = -EACCES;
		break;
	}
	mutex_unlock(&voc->lock);
	return rt;
}

/* As we can also modify those value with sysfs we have to bypass v4l2 control
 * cached value by declaring those control volatiles. */
static int avi_voc_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct avi_voc *voc = container_of(ctrl->handler,
	                                   struct avi_voc,
	                                   ctrl_handler);
	int rt = 0;

	mutex_lock(&voc->lock);

	switch (ctrl->id) {
	case V4L2_CID_VOC_NO_SCALER:
		ctrl->val = voc->no_scaler;
		break;
	case V4L2_CID_VOC_ZORDER:
		ctrl->val = voc->zorder;
		break;
	case V4L2_CID_VOC_HIDE:
		ctrl->val = voc->hide;
		break;
	default:
		dev_err(voc->dev, "Invalid set of control=%08x\n", ctrl->id);
		rt = -EACCES;
		break;
	}

	dprintk(voc, "id=%x, value=%d\n", ctrl->id, ctrl->val);
	mutex_unlock(&voc->lock);
	return rt;
}

static const struct v4l2_ctrl_ops avi_voc_ctrl_ops = {
	.s_ctrl = avi_voc_s_ctrl,
};

static const struct v4l2_ctrl_ops avi_voc_ctrl_ops_volatile = {
	.g_volatile_ctrl = avi_voc_g_volatile_ctrl,
	.s_ctrl = avi_voc_s_ctrl,
};

static const struct v4l2_ctrl_config avi_voc_ctrls[] = {
	{
		.ops = &avi_voc_ctrl_ops_volatile,
		.id = V4L2_CID_VOC_NO_SCALER,
		.name = "Turn off scaling possibility",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
	},
	{
		.ops = &avi_voc_ctrl_ops_volatile,
		.id = V4L2_CID_VOC_ZORDER,
		.name = "blender zorder",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -1,
		.max = 7,
		.step = 1,
		.def = -1,
	},
	{
		.ops = &avi_voc_ctrl_ops,
		.id = V4L2_CID_MIN_BUFFERS_FOR_OUTPUT,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Minimum number of output buffer",
		.min = AVI_VOC_MIN_BUFFERS,
		.max = AVI_VOC_MIN_BUFFERS,
		.step = 1,
		.def = AVI_VOC_MIN_BUFFERS,
	},
};

static const struct v4l2_ctrl_config avi_voc_ctrl_hide = {
	.ops = &avi_voc_ctrl_ops,
	.id = V4L2_CID_VOC_HIDE,
	.name = "Hide VOC",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
};

void avi_voc_ctrl_unhide(struct avi_voc *voc)
{
	mutex_lock(voc->ctrl_handler.lock);
	voc->hide_ctrl->val = 0;
	voc->hide_ctrl->cur.val = 0;
	mutex_unlock(voc->ctrl_handler.lock);
}

int avi_voc_ctrl_create(struct avi_voc *voc)
{
	int rt;
	int i;
	struct v4l2_ctrl *ctrl;

	if ((rt = v4l2_ctrl_handler_init(&voc->ctrl_handler,
	                                 ARRAY_SIZE(avi_voc_ctrls)+1)))
		return rt;

	for (i = 0; i < ARRAY_SIZE(avi_voc_ctrls); ++i) {
		ctrl = v4l2_ctrl_new_custom(&voc->ctrl_handler,
		                            &avi_voc_ctrls[i], NULL);
		if (ctrl != NULL && avi_voc_ctrls[i].ops->g_volatile_ctrl)
			ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;
	}

	voc->hide_ctrl = v4l2_ctrl_new_custom(&voc->ctrl_handler,
	                                      &avi_voc_ctrl_hide, NULL);

	voc->vdev->ctrl_handler = &voc->ctrl_handler;

	return 0;
}

void avi_voc_ctrl_free(struct avi_voc *voc)
{
	v4l2_ctrl_handler_free(&voc->ctrl_handler);
}
