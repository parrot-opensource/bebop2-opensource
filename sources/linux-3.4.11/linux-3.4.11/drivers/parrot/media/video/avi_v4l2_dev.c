#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/device.h>

#include "avi_v4l2.h"
#include "avi_v4l2_dev.h"

#define DRIVER_NAME "avi_v4l2_dev"
#define DEVICE_NAME "avi"

struct avi_v4l2_dev
{
	struct device		*dev;
	struct mutex		 lock;
	struct class		*class;
	dev_t			 devno;
	struct cdev		 cdev;
	struct list_head	 segment_devs;
	int			 id;
};

struct segment_dev
{
	struct list_head	 list;
	struct avi_segment	*segment;
};

/* There should be only one instance of this driver so I put the data statically
 * here. I still pass pointers everywhere though, in order to make all functions
 * reentrant if we ever need to refactor the driver or add multiple instances
 * for some reason. */
static struct avi_v4l2_dev avi_v4l2_dev;

/* Called with avi_dev->lock held */
int avi_v4l2_dev_build(struct avi_v4l2_dev *avi_dev, struct avi_dev_segment *d)
{
	struct segment_dev *sd;

	sd = kzalloc(sizeof(*sd), GFP_KERNEL);
	if (!sd)
		return -ENOMEM;

	sd->segment = avi_segment_build(&d->caps,
					"v4l2",
					avi_dev->id,
					-1,
					avi_dev->dev);
	if (IS_ERR(sd->segment)) {
		int ret = PTR_ERR(sd->segment);

		kfree(sd);

		return ret;
	}

	avi_dev->id++;
	memcpy(d->id, sd->segment->id, sizeof(d->id));

	list_add_tail(&sd->list, &avi_dev->segment_devs);

	return 0;
}

long avi_v4l2_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user		*argp	 = (void __user *)arg;
	struct avi_v4l2_dev	*avi_dev = &avi_v4l2_dev;
	struct avi_dev_segment	 d;
	int			 ret;

	switch (cmd) {
	case AVI_BUILD_DEV:
		if (copy_from_user(&d, argp, sizeof(d)))
			return -EFAULT;

		mutex_lock(&avi_dev->lock);
		ret = avi_v4l2_dev_build(avi_dev, &d);
		mutex_unlock(&avi_dev->lock);

		if (copy_to_user(argp, &d,sizeof(d)))
			return -EFAULT;

		return ret;

		break;
	default:
		return -ENOIOCTLCMD;
	}
}

int avi_v4l2_dev_open(struct inode *inode, struct file *file)
{
	return 0;
}

int avi_v4l2_dev_release(struct inode *inode, struct file *file)
{
	return 0;
}

static struct file_operations const avi_v4l2_dev_fops = {
	.owner		= THIS_MODULE,
	.open           = avi_v4l2_dev_open,
	.release        = avi_v4l2_dev_release,
	.unlocked_ioctl = avi_v4l2_dev_ioctl,
};

static int __init avi_v4l2_dev_init(void)
{
	struct avi_v4l2_dev	*avi_dev = &avi_v4l2_dev;
	int			 ret;

	if (!avi_probed()) {
		ret = -ENODEV;
		goto no_avi;
	}

	memset(avi_dev, 0, sizeof(*avi_dev));

	mutex_init(&avi_dev->lock);
	INIT_LIST_HEAD(&avi_dev->segment_devs);

	ret = alloc_chrdev_region(&avi_dev->devno, 0, 1, DRIVER_NAME);
	if (ret)
		goto alloc_chrdev_failed;

	cdev_init(&avi_dev->cdev, &avi_v4l2_dev_fops);
	avi_dev->cdev.owner = THIS_MODULE;

	ret = cdev_add(&avi_dev->cdev, avi_dev->devno, 1);
	if (ret)
		goto cdev_add_failed;

	avi_dev->class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(avi_dev->class)) {
		ret = PTR_ERR(avi_dev->class);
		goto class_create_failed;
	}

	avi_dev->dev = device_create(avi_dev->class,
				     avi_dev->dev,
				     avi_dev->devno,
				     &avi_dev,
				     DEVICE_NAME);
	if (IS_ERR(avi_dev->dev)) {
		ret = PTR_ERR(avi_dev->dev);
		goto device_create_failed;
	}

	dev_info(avi_dev->dev, "successfully probed\n");

	return 0;

 device_create_failed:
	class_destroy(avi_dev->class);
 class_create_failed:
	cdev_del(&avi_dev->cdev);
 cdev_add_failed:
	unregister_chrdev_region(avi_dev->devno, 1);
 alloc_chrdev_failed:
	mutex_destroy(&avi_v4l2_dev.lock);
 no_avi:
	memset(avi_dev, 0, sizeof(*avi_dev));

	return ret;
}
module_init(avi_v4l2_dev_init);

static void __exit avi_v4l2_dev_exit(void)
{
	struct segment_dev      *sd, *next;
	struct avi_v4l2_dev	*avi_dev = &avi_v4l2_dev;

	list_for_each_entry_safe(sd, next, &avi_dev->segment_devs, list) {
		avi_segment_teardown(sd->segment);
		kfree(sd);
	}

	device_destroy(avi_dev->class, avi_dev->devno),
	class_destroy(avi_dev->class);
	cdev_del(&avi_dev->cdev);
	unregister_chrdev_region(avi_dev->devno, 1);
	mutex_destroy(&avi_v4l2_dev.lock);

	memset(&avi_v4l2_dev, 0, sizeof(avi_v4l2_dev));
}
module_exit(avi_v4l2_dev_exit);

MODULE_AUTHOR("Lionel Flandrin <lionel.flandrin@parrot.com>");
MODULE_DESCRIPTION("V4L2 device interface for the AVI");
MODULE_LICENSE("GPL");
