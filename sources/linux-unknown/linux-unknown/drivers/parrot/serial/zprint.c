/**
 * linux/drivers/parrot/zprint.c - Parrot7 ZPrint Zebu transactor
 *                                 implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * @author	Lionel Flandrin <lionel.flandrin@parrot.com>
 * @date  24-Nov-2011
 *
 * This file is released under the GPL
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/p7.h>
#include "zprint.h"

struct zprint {
	struct class    *class;
	dev_t           devno;
	struct cdev     cdev;
	struct device   *device;
	spinlock_t      lock;
};

/* The one instance of the zebu driver */
static struct zprint zprint = { 0 };

static inline void zprint_putc(unsigned c)
{
	writel(c, __MMIO_P2V(P7_ZPRINT));
}

static ssize_t zprint_write(struct file *filp,
                            const char __user *buf,
                            size_t count,
                            loff_t *f_pos)
{
	size_t cnt;

	for (cnt = 0; cnt < count; cnt++) {
		char        c;
		int const   err = get_user(c, &buf[cnt]);
		if (err)
			return err;

		zprint_putc((unsigned) c);
	}

	return count;
}

static struct file_operations const zprint_fops = {
	.owner  = THIS_MODULE,
	.write  = &zprint_write,
};

static void (*zprint_write_oldcon)(struct console*, char const*, unsigned);

static void zprint_write_con(struct console* con,
                             char const* buff,
                             unsigned int count)
{
	if (con->index)
		return (*zprint_write_oldcon)(con, buff, count);

	while (count--) {
		if (*buff == '\n')
			zprint_putc('\r');
		zprint_putc((unsigned) *buff++);
	}
}

#ifdef CONFIG_CONSOLE_POLL
static struct tty_operations zprint_old_ttyops;

static void zprint_poll_putc(struct tty_driver* driver, int line, char c)
{
	if (line) {
		if (zprint_old_ttyops.poll_put_char)
			zprint_old_ttyops.poll_put_char(driver, line, c);
		return;
	}

	zprint_putc((unsigned) c);
}
#endif

static void __init zprint_setup_con(void)
{
	struct console* con;

	for (con = console_drivers; con != NULL; con = con->next) {
		if (con->device) {
			int                         idx = -1;
			struct tty_driver* const    drv = con->device(con, &idx);

			if (! idx &&
			    drv &&
			    ((con->flags & (CON_ENABLED | CON_CONSDEV)) ==
			     (CON_ENABLED | CON_CONSDEV)) && con->write) {
				zprint_write_oldcon = con->write;
				con->write = &zprint_write_con;
#ifdef CONFIG_CONSOLE_POLL
				zprint_old_ttyops = *drv->ops;
				zprint_old_ttyops.poll_put_char = &zprint_poll_putc;
				drv->ops = &zprint_old_ttyops;
#endif
				pr_info("Zprint: %s0 console output redirected.\n", con->name);
				return;
			}
		}
	}
}

static int __devinit zprint_probe(struct platform_device* pdev)
{
	int res;

	spin_lock_init(&zprint.lock);

	res = alloc_chrdev_region(&zprint.devno, 0, 1, ZPRINT_DRV_NAME);
	if (res < 0)
		goto exit;

	cdev_init(&zprint.cdev, &zprint_fops);
	zprint.cdev.owner = THIS_MODULE;

	res = cdev_add(&zprint.cdev, zprint.devno, 1);
	if (res < 0)
		goto dealloc_chrdev;

	zprint.class = class_create(THIS_MODULE, ZPRINT_DRV_NAME);
	if (IS_ERR(zprint.class)) {
		res = PTR_ERR(zprint.class);
		goto del_cdev;
	}

	zprint.device = device_create(zprint.class,
	                              NULL,
	                              zprint.devno,
	                              NULL,
	                              ZPRINT_DRV_NAME);
	if (IS_ERR(zprint.device)) {
		res = PTR_ERR(zprint.device);
		goto class_destroy;
	}

	zprint_setup_con();
	dev_info(&pdev->dev, "ready\n");
	return 0;

class_destroy:
	class_destroy(zprint.class);
del_cdev:
	cdev_del(&zprint.cdev);
dealloc_chrdev:
	unregister_chrdev_region(zprint.devno, 1);
exit:
	return res;
}

static struct platform_driver zprint_driver = {
	.probe		= zprint_probe,
	.driver		= {
		.owner  = THIS_MODULE,
		.name	= ZPRINT_DRV_NAME
	},
};

static int __init zprint_init(void)
{
	return platform_driver_register(&zprint_driver);
}
late_initcall(zprint_init);

MODULE_AUTHOR("Lionel Flandrin <lionel.flandrin@parrot.com>");
MODULE_DESCRIPTION("Zebu zprint interface");
MODULE_LICENSE("GPL");
