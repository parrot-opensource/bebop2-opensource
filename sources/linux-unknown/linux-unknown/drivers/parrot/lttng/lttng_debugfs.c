
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/module.h>

#include <trace/events/parrot_trace.h>

struct lttng_debugfs {
	struct dentry	*dir;
};

static struct lttng_debugfs lttng_debugfs;

static ssize_t lttng_event_write(struct file *filp,
				 const char __user *ubuf,
				 size_t cnt,
				 loff_t *ppos,
				 unsigned long *event)
{
	char buf[32];
	int ret;

	cnt = min(cnt, sizeof(buf) - 1);
	if (copy_from_user(buf, ubuf, cnt))
		return -EFAULT;
	buf[cnt] = '\0';

	ret = kstrtoul(buf, 10, event);
	if (0 != ret)
		return ret;

	*ppos += cnt;
	return cnt;
}

static ssize_t lttng_event_start_write(struct file *filp,
				       const char __user *ubuf,
				       size_t cnt,
				       loff_t *ppos)
{
	unsigned long event = 0;
	int ret = lttng_event_write(filp, ubuf, cnt, ppos, &event);
	trace_user_kevent_start(event);
	return ret;
}

static ssize_t lttng_event_stop_write(struct file *filp,
				      const char __user *ubuf,
				      size_t cnt,
				      loff_t *ppos)
{
	unsigned long event = 0;
	int ret = lttng_event_write(filp, ubuf, cnt, ppos, &event);
	trace_user_kevent_stop(event);
	return ret;
}

static ssize_t lttng_event_msg_write(struct file *filp,
				     const char __user *ubuf,
				     size_t cnt,
				     loff_t *ppos)
{
	char buf[128];

	cnt = min(cnt, sizeof(buf) - 1);
	if (copy_from_user(buf, ubuf, cnt))
		return -EFAULT;
	buf[cnt] = '\0';

	trace_user_kmessage(buf);

	*ppos += cnt;
	return cnt;
}

static const struct file_operations lttng_event_start_fops = {
	.owner = THIS_MODULE,
	.write = lttng_event_start_write,
};

static const struct file_operations lttng_event_stop_fops = {
	.owner = THIS_MODULE,
	.write = lttng_event_stop_write,
};

static const struct file_operations lttng_event_msg_fops = {
	.owner = THIS_MODULE,
	.write = lttng_event_msg_write,
};

static int __init lttng_debugfs_init(void)
{
	int ret = 0;
	struct lttng_debugfs *debugfs = &lttng_debugfs;
	struct dentry *d;

	debugfs->dir = debugfs_create_dir("lttng", NULL);
	if (!debugfs->dir) {
		ret = -ENOMEM;
		goto exit;
	}

	d = debugfs_create_file("event_start", S_IWUGO, debugfs->dir,
				NULL, &lttng_event_start_fops);
	if (!d) {
		ret = -ENOMEM;
		goto exit;
	}

	d = debugfs_create_file("event_stop", S_IWUGO, debugfs->dir,
				NULL, &lttng_event_stop_fops);
	if (!d) {
		ret = -ENOMEM;
		goto exit;
	}

	d = debugfs_create_file("message", S_IWUGO, debugfs->dir,
				NULL, &lttng_event_msg_fops);
	if (!d) {
		ret = -ENOMEM;
		goto exit;
	}

exit:
	return ret;
}

static void __exit lttng_debugfs_exit(void)
{
	struct lttng_debugfs *debugfs = &lttng_debugfs;
	debugfs_remove_recursive(debugfs->dir);
}

module_init(lttng_debugfs_init);
module_exit(lttng_debugfs_exit);

MODULE_DESCRIPTION("LTTng simple debugfs interface");
MODULE_AUTHOR("Adrien Charruel, <adrien.charruel@parrot.com>");
MODULE_LICENSE("GPL");
