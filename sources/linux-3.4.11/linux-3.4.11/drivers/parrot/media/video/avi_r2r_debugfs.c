/**
 * @file avi_r2r_debugfs.c
 *  Parrot AVI RAM to RAM driver.
 *
 * Copyright (C) 2014 Parrot S.A.
 *
 * @author     didier.leymarie.ext@parrot.com
 * @date       2014-02-06
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

#include "avi_r2r_core.h"

/* debugfs user */
/*--------------*/
static int user_status_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_user_context *user = s->private;

	seq_printf(s,
		   "running=%d, stacked=%d\n",
		   atomic_read(&user->context->running),
		   atomic_read(&user->context->stacked_nr));

	return 0;
}

static int user_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, user_status_show ,inode->i_private);
}

static const struct file_operations user_status_debugfs_fops = {
	.open		= user_status_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*--------------*/
static int user_device_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_user_context *user = s->private;

	seq_printf(s, "%s\n", dev_name(user->context->dev));

	return 0;
}

static int user_device_open(struct inode *inode, struct file *file)
{
	return single_open(file, user_device_show ,inode->i_private);
}

static const struct file_operations user_device_debugfs_fops = {
	.open		= user_device_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*--------------*/
static int user_requirements_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_user_context	*user = s->private;
	unsigned long			caps = user->requirements;
	int				n = 0;
	const char			*str;

	while ((str = avi_debug_caps_to_str(&caps))) {
		if (n++)
			seq_printf(s, " | ");

		seq_printf(s, "%s", str);
	}

	if (n == 0)
		seq_printf(s, "<none>");

	seq_printf(s, "\n");

	return 0;
}

static int user_requirements_open(struct inode *inode, struct file *file)
{
	return single_open(file, user_requirements_show ,inode->i_private);
}

static const struct file_operations user_requirements_debugfs_fops = {
	.open		= user_requirements_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*--------------*/
static int user_channels_compliant_show(struct seq_file *s, void *unused)
{
	char buffer[64];
	struct avi_r2r_user_context *user = s->private;
	struct avi_r2r			*avi_r2r = user->avi_r2r;
	int c,i;

	c=0;
	if (user->channels_compliant) {
		for (i=0;i<avi_r2r->channels_nr;i++) {
			if (user->channels_compliant & (1<<i))
				c+=sprintf(&buffer[c],"%d ",i);
		}
	}
	else
		c+=sprintf(&buffer[c],"<none>");

	seq_printf(s, "%s\n", buffer);

	return 0;
}

static int user_channels_compliant_open(struct inode *inode, struct file *file)
{
	return single_open(file, user_channels_compliant_show ,inode->i_private);
}

static const struct file_operations user_channels_compliant_debugfs_fops = {
	.open		= user_channels_compliant_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*--------------*/
static int user_src_format_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_user_context *user = s->private;

	seq_printf(s, "%dx%d, %s, %s[%d], %d bytes/line@p0, %d bytes/line@p1\n",
		user->format.source.width,
		user->format.source.height,
		avi_debug_colorspace_to_str(user->format.source.colorspace),
		avi_debug_pixfmt_to_str(user->format.source.pix_fmt),
		user->format.source.pix_fmt.id,
		user->format.source.plane0.line_size,
		user->format.source.plane1.line_size);

	return 0;
}

static int user_src_format_open(struct inode *inode, struct file *file)
{
	return single_open(file, user_src_format_show ,inode->i_private);
}

static const struct file_operations user_src_format_debugfs_fops = {
	.open		= user_src_format_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*--------------*/
static int user_tgt_format_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_user_context *user = s->private;

	seq_printf(s, "%dx%d, %s, %s[%d], %d bytes/line@p0, %d bytes/line@p1\n",
		user->format.target.width,
		user->format.target.height,
		avi_debug_colorspace_to_str(user->format.source.colorspace),
		avi_debug_pixfmt_to_str(user->format.target.pix_fmt),
		user->format.target.pix_fmt.id,
		user->format.target.plane0.line_size,
		user->format.target.plane1.line_size);

	return 0;
}

static int user_tgt_format_open(struct inode *inode, struct file *file)
{
	return single_open(file, user_tgt_format_show ,inode->i_private);
}

static const struct file_operations user_tgt_format_debugfs_fops = {
	.open		= user_tgt_format_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*--------------*/
static int user_compose_format_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_user_context *user = s->private;

	if (user->format.use_compose)
		seq_printf(s, "%dx%d, %s, (%u, %u) alpha=0x%02X\n",
			user->format.compose.width,
			user->format.compose.height,
			avi_debug_colorspace_to_str(user->format.compose.colorspace),
			user->format.layout.x,
			user->format.layout.y,
			user->format.layout.alpha);
	else
		seq_printf(s, "no compositing\n");
	return 0;
}

static int user_compose_format_open(struct inode *inode, struct file *file)
{
	return single_open(file, user_compose_format_show ,inode->i_private);
}

static const struct file_operations user_compose_format_debugfs_fops = {
	.open		= user_compose_format_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void avi_r2r__user_setup_debugfs(struct avi_r2r_user_context *user)
{
	char		buffer[64];
	struct dentry	*file;
	struct avi_r2r	*avi_r2r=user->avi_r2r;

	sprintf(buffer, "user.%p", user);

#define MY_DEBUGFS_REGISTER(_name)				\
	file = debugfs_create_file(#_name,			\
			   S_IRWXUGO,				\
			   user->debugfs_root,			\
			   user,				\
			   &user_ ## _name ## _debugfs_fops);	\
	if (IS_ERR_OR_NULL(file)) {				\
		goto rm;					\
	}

	user->debugfs_root = debugfs_create_dir(buffer,
						avi_r2r->debugfs_root);
	if (IS_ERR_OR_NULL(user->debugfs_root)) {
		AVIR2R_LOG_WARN(avi_r2r,
				"cannot create debugfs directory "
				"for user %p",
				user);
		user->debugfs_root = NULL;
		return;
	}

	MY_DEBUGFS_REGISTER(status);
	MY_DEBUGFS_REGISTER(device);
	MY_DEBUGFS_REGISTER(requirements);
	MY_DEBUGFS_REGISTER(channels_compliant);
	MY_DEBUGFS_REGISTER(src_format);
	MY_DEBUGFS_REGISTER(tgt_format);
	MY_DEBUGFS_REGISTER(compose_format);
#undef MY_DEBUGFS_REGISTER
	return;

rm:
	if (user->debugfs_root)
		debugfs_remove_recursive(user->debugfs_root);
	user->debugfs_root = NULL;

	return;
}

void avi_r2r__user_destroy_debugfs(struct avi_r2r_user_context *user)
{
	if (user->debugfs_root)
		debugfs_remove_recursive(user->debugfs_root);
	user->debugfs_root = NULL;
}

/* debugfs channel */
/*-----------*/
static const char *avi_r2r_channel_status_str[AVI_R2R_CHANNEL_STATUS_NR]=
{
#define AVI_DEBUGFS_CHAN_STATUS_TO_STR(_p) [AVI_R2R_CHANNEL_ ## _p] = # _p
	AVI_DEBUGFS_CHAN_STATUS_TO_STR(UNUSED),
	AVI_DEBUGFS_CHAN_STATUS_TO_STR(INACTIVE),
	AVI_DEBUGFS_CHAN_STATUS_TO_STR(RUNNING)
#undef AVI_DEBUGFS_CHAN_STATUS_TO_STR
};

static int channel_status_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_channel_data	*channel = s->private;

	seq_printf(s, "%s segment in=%s out=%s\n",
	           avi_r2r_channel_status_str[atomic_read(&channel->status)],
	           avi_debug_activation_to_str(channel->input_segment->active),
	           avi_debug_activation_to_str(channel->output_segment->active));

	return 0;
}

static int channel_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, channel_status_show, inode->i_private);
}

static const struct file_operations channel_status_debugfs_fops = {
	.open		= channel_status_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int channel_work_queue_level_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_channel_data	*channel = s->private;

	seq_printf(s, "%d\n", avi_r2r__work_queue_level(&channel->work_queue));

	return 0;
}

static int channel_work_queue_level_open(struct inode *inode, struct file *file)
{
	return single_open(file, channel_work_queue_level_show, inode->i_private);
}

static const struct file_operations channel_work_queue_level_debugfs_fops = {
	.open		= channel_work_queue_level_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int channel_timespec_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_channel_data	*channel = s->private;

	seq_printf(s, "%20lld, %20lld, %20lld : %10lld microseconds\n",
		ktime_to_us(timespec_to_ktime(channel->ts_push)),
		ktime_to_us(timespec_to_ktime(channel->ts_start)),
		ktime_to_us(timespec_to_ktime(channel->ts_stop)),
		ktime_us_delta(timespec_to_ktime(channel->ts_stop),
				timespec_to_ktime(channel->ts_start))
		);

	return 0;
}

static int channel_timespec_open(struct inode *inode, struct file *file)
{
	return single_open(file, channel_timespec_show, inode->i_private);
}

static const struct file_operations channel_timespec_debugfs_fops = {
	.open		= channel_timespec_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int channel_timings_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_channel_data	*channel = s->private;
	struct timings prev;
	int i, n, c, cn;

	cn=channel->magic & ~_AVI_R2R_CHANNEL_MAGIC_MASK;

	n = channel->timings_nr;
	seq_printf(s, "%d/%d\n", n, channel->timings_counter);
	if (n>0) {
		i = channel->timings_index-n;
		if (i < 0) i += channel->nr_timings_saved;
		prev = channel->timings[i];
		for (c = 0; c < n; c++) {
			seq_printf(s,
				"%08X_%01d: %18lld, %18lld, %18lld :"
				" %10lld microseconds, "
				"%10lld, %10lld, %10lld, %10d\n",
				channel->timings[i].uuid, cn,
				ktime_to_us(timespec_to_ktime(channel->timings[i].ts_start)),
				ktime_to_us(timespec_to_ktime(channel->timings[i].ts_stop)),
				ktime_to_us(timespec_to_ktime(channel->timings[i].ts_push)),
				ktime_us_delta(timespec_to_ktime(channel->timings[i].ts_stop),
						timespec_to_ktime(channel->timings[i].ts_start)),
				ktime_us_delta(timespec_to_ktime(channel->timings[i].ts_start),
						timespec_to_ktime(prev.ts_stop)),
				ktime_us_delta(timespec_to_ktime(channel->timings[i].ts_start),
						timespec_to_ktime(prev.ts_start)),
				ktime_us_delta(timespec_to_ktime(channel->timings[i].ts_start),
						timespec_to_ktime(channel->timings[i].ts_push)),
				channel->timings[i].index);
			prev = channel->timings[i];
			i++;
			if (i >= channel->nr_timings_saved ) i = 0;
		}
	}

	return 0;
}

static int channel_timings_open(struct inode *inode, struct file *file)
{
	return single_open(file, channel_timings_show, inode->i_private);
}

static const struct file_operations channel_timings_debugfs_fops = {
	.open		= channel_timings_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int channel_available_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_channel_data	*channel = s->private;
	unsigned long			caps = channel->available;
	int				n = 0;
	const char			*str;

	while ((str = avi_debug_caps_to_str(&caps))) {
		if (n++)
			seq_printf(s, " | ");

		seq_printf(s, "%s", str);
	}

	if (n == 0)
		seq_printf(s, "<none>");

	seq_printf(s, "\n");

	return 0;
}

static int channel_available_open(struct inode *inode, struct file *file)
{
	return single_open(file, channel_available_show, inode->i_private);
}

static const struct file_operations channel_available_debugfs_fops = {
	.open		= channel_available_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int channel_current_user_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_channel_data	*channel = s->private;
	void				*uuid = 0, *prev_uuid = 0;
	bool				cur_ok = 0, prev_ok = 0;

	if (channel->current_work.user)
		if (avi_r2r__user_context_is_ok(channel->current_work.user)) {
			uuid = channel->current_work.user;
			cur_ok = 1;
		}

	prev_uuid = channel->previous_user;
	if (channel->previous_user)
	 	if (avi_r2r__user_context_is_ok(channel->previous_user)) {
	 		prev_ok = 1;
	 	}

	seq_printf(s,
		   "%p%c %p%c\n",
		   uuid, cur_ok?' ':'!',
		   prev_uuid, prev_ok?' ':'!');

	return 0;
}

static int channel_current_user_open(struct inode *inode, struct file *file)
{
	return single_open(file, channel_current_user_show, inode->i_private);
}

static const struct file_operations channel_current_user_debugfs_fops = {
	.open		= channel_current_user_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int channel_interrupts_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_channel_data	*channel = s->private;

	seq_printf(s,
		   "%u, "
		   "spurious %u, "
		   "missing source p0=%u p1=%u, "
		   "segment in=%u out=%u"
		   "\n",
		   channel->interrupts_counter,
		   channel->spurious_interrupts_counter,
		   channel->missing_source_interrupts_counter,
		   channel->missing_planar_interrupts_counter,
		   channel->input_segment_interrupts_counter,
		   channel->output_segment_interrupts_counter);

	return 0;
}

static int channel_interrupts_open(struct inode *inode, struct file *file)
{
	return single_open(file, channel_interrupts_show, inode->i_private);
}

static const struct file_operations channel_interrupts_debugfs_fops = {
	.open		= channel_interrupts_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int channel_apply_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_channel_data	*channel = s->private;

	seq_printf(s, "%u\n", channel->apply_counter);

	return 0;
}

static int channel_apply_open(struct inode *inode, struct file *file)
{
	return single_open(file, channel_apply_show , inode->i_private);
}

static const struct file_operations channel_apply_debugfs_fops = {
	.open		= channel_apply_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int channel_nodes_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_channel_data	*channel = s->private;
	int				i;

	seq_printf(s, "(%s) ", channel->input_segment->id);
	for (i=0; i < channel->input_segment->nodes_nr; i++) {
		seq_printf(s, "%s", channel->input_segment->nodes[i]->name);
		if (i<(channel->input_segment->nodes_nr-1))
			seq_printf(s, " -> ");
	}
	{
		struct avi_node *dma_in_planar =
			avi_segment_get_node(channel->input_segment,
					     AVI_CAP_PLANAR);
		struct avi_node *scaler_planar =
			avi_segment_get_node(channel->input_segment,
					      AVI_CAP_SCAL);
		if (dma_in_planar && scaler_planar)
		{
			scaler_planar = avi_get_node(scaler_planar->node_id+1);
			seq_printf(s, ", %s -> %s",
					dma_in_planar->name,
					scaler_planar->name);
		}
	}

	seq_printf(s, " >> OUT(%s) ", channel->output_segment->id);
	for (i=0; i < channel->output_segment->nodes_nr; i++)
	{
		seq_printf(s, "%s", channel->output_segment->nodes[i]->name);
		if (i<(channel->output_segment->nodes_nr-1))
			seq_printf(s, " -> ");
	}

	seq_printf(s, "\n");
	return 0;
}

static int channel_nodes_open(struct inode *inode, struct file *file)
{
	return single_open(file, channel_nodes_show ,inode->i_private);
}

static const struct file_operations channel_nodes_debugfs_fops = {
	.open		= channel_nodes_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int __devinit avi_r2r__channel_setup_debugfs
				(struct avi_r2r_channel_data	*channel)
{
#define MY_DEBUGFS_REGISTER(_name)				\
	file = debugfs_create_file(#_name,			\
			   S_IRWXUGO,				\
			   debugfs_chan_root,			\
			   channel,				\
			   &channel_ ## _name ## _debugfs_fops);	\
	if (IS_ERR_OR_NULL(file)) {				\
		ret = PTR_ERR(file) ?: -ENOMEM;			\
		goto rm;					\
	}

	char		buffer[64];
	struct avi_r2r	*avi_r2r = channel->avi_r2r;
	struct dentry	*debugfs_chan_root = 0;
	struct dentry	*file;
	int		ret = 0;

	channel->debugfs_root = 0;

	/* allocate timings data for channel */
	channel->timings=kzalloc(
		sizeof(struct timings)*channel->nr_timings_saved,
		GFP_KERNEL);
	if (!channel->timings) {
		AVIR2R_LOG_ERROR(avi_r2r,
			"couldn't allocate struct timings");
		ret =  -ENOMEM;
		goto rm;
	}

	sprintf(buffer,"channel.%d",channel->magic & ~_AVI_R2R_CHANNEL_MAGIC_MASK);
	debugfs_chan_root = debugfs_create_dir(buffer, avi_r2r->debugfs_root);
	if (IS_ERR_OR_NULL(debugfs_chan_root))
		return debugfs_chan_root ? PTR_ERR(debugfs_chan_root) : -ENOMEM;

	MY_DEBUGFS_REGISTER(status);
	MY_DEBUGFS_REGISTER(work_queue_level);
	MY_DEBUGFS_REGISTER(timespec);
	MY_DEBUGFS_REGISTER(timings);
	MY_DEBUGFS_REGISTER(available);
	MY_DEBUGFS_REGISTER(current_user);
	MY_DEBUGFS_REGISTER(interrupts);
	MY_DEBUGFS_REGISTER(apply);
	MY_DEBUGFS_REGISTER(nodes);

	channel->debugfs_root = debugfs_chan_root;
#undef MY_DEBUGFS_REGISTER
	return 0;

rm:
	if (channel->timings)
		kfree(channel->timings);
	if (debugfs_chan_root)
		debugfs_remove_recursive(debugfs_chan_root);

	return ret;
}

void avi_r2r__channel_destroy_debugfs(struct avi_r2r_channel_data *channel)
{
	if (channel->timings)
		kfree(channel->timings);
	if (channel->debugfs_root)
		debugfs_remove_recursive(channel->debugfs_root);
}

void avi_r2r__channel_add_timings(struct avi_r2r_channel_data *data)
{
	data->timings[data->timings_index].ts_start = data->ts_start;
	data->timings[data->timings_index].ts_stop  = data->ts_stop;
	data->timings[data->timings_index].ts_push  = data->ts_push;
	data->timings[data->timings_index].uuid     = (unsigned long)data->current_work.user;
	data->timings[data->timings_index].index    = data->timings_counter;

	data->timings_counter++;
	data->timings_index++;
	if (data->timings_index >= data->nr_timings_saved)
		data->timings_index = 0;

	if (data->timings_nr < data->nr_timings_saved)
		data->timings_nr++;
}
