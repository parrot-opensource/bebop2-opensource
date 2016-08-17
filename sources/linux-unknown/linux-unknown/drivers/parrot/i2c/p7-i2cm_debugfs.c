/**
 *  linux/drivers/parrot/i2c/p7-i2cm-debugfs.c
 *
 *  Copyright (C) 2011 Parrot S.A.
 *
 * @author  Lionel Flandrin <lionel.flandrin@parrot.com>
 *   @date  26-Sep-2011
 */

#include <linux/module.h>
#include <linux/seq_file.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "p7-i2cm_debugfs.h"
#include "p7-i2cm_regs.h"

static struct dentry *p7i2cm_debugfs_root = NULL;
static atomic_t       p7i2cm_debugfs_cnt  = ATOMIC_INIT(0);

static int p7i2cm_display_freq(struct seq_file *s,
                               struct p7i2cm_debugfs *d,
                               u32 prescale)
{
	seq_printf(s, "%luHz (prescale: 0x%04x)\n",
	           d->in_clk / (12 * (prescale + 1)),
	           prescale);

	return 0;
}

static ssize_t p7i2cm_read_prescale(struct p7i2cm_debugfs *d,
                                    const char __user *_buf,
                                    size_t size,
                                    loff_t *ppos)
{
	unsigned long    freq;
	char            *last;
	char             buf[16];
	unsigned         tmp;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(buf) - 1)
		return -ERANGE;

	if (copy_from_user(buf, _buf, size))
		return -EFAULT;

	buf[size] = '\0';

	freq = simple_strtoul(buf, &last, 0);

	switch (*last) {
	case 'M':
		freq *= 1000;
	case 'k':
		freq *= 1000;
	case '\0':
	case '\n':
		break;
	default:
		return -EINVAL;
	}

	/* Compute the value of prescale for the desired frequency */
	tmp = 12 * freq;

	if (tmp == 0)
		return -ERANGE;

	tmp = (d->in_clk + tmp - 1) / tmp - 1;
	if (tmp > 0xffff)
		return 0xffff;

	return tmp;
}

static int p7i2cm_speed_show(struct seq_file *s, void *unused)
{
	struct p7i2cm_debugfs *d = s->private;

	return p7i2cm_display_freq(s, d, __raw_readl(d->base + I2CM_PRESCALE));
}

static ssize_t p7i2cm_speed_store(struct file *file,
                                  const char __user *buf,
                                  size_t size,
                                  loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct p7i2cm_debugfs   *d = m->private;
	ssize_t                  prescale;

	prescale = p7i2cm_read_prescale(d, buf, size, ppos);

	if (prescale < 0)
		return prescale;

	__raw_writel(prescale, d->base + I2CM_PRESCALE);

	return size;
}

static int p7i2cm_speed_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, p7i2cm_speed_show, inode->i_private);
}

static const struct file_operations p7i2cm_speed_debug_ops = {
	.open           = p7i2cm_speed_debug_open,
	.read           = seq_read,
	.write          = p7i2cm_speed_store,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int p7i2cm_highspeed_show(struct seq_file *s, void *unused)
{
	struct p7i2cm_debugfs *d = s->private;

	return p7i2cm_display_freq(s, d,
	                           __raw_readl(d->base + I2CM_HIGH_PRESCALE));
}

static ssize_t p7i2cm_highspeed_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct p7i2cm_debugfs   *d = m->private;
	ssize_t                  prescale;

	prescale = p7i2cm_read_prescale(d, buf, size, ppos);

	if (prescale < 0)
		return prescale;

	__raw_writel(prescale, d->base + I2CM_HIGH_PRESCALE);

	return size;
}

static int p7i2cm_highspeed_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, p7i2cm_highspeed_show, inode->i_private);
}

static const struct file_operations p7i2cm_highspeed_debug_ops = {
	.open           = p7i2cm_highspeed_debug_open,
	.read           = seq_read,
	.write          = p7i2cm_highspeed_store,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int p7i2cm_dbglvl_show(struct seq_file *s, void *unused)
{
	struct p7i2cm_debugfs *d = s->private;

	seq_printf(s, "%d\n",d->dbglvl);

	return 0;
}

static ssize_t p7i2cm_dbglvl_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct p7i2cm_debugfs   *d = m->private;
	unsigned long            level;
	char                    *last;
	char                     _buf[16];

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	level = simple_strtoul(_buf, &last, 0);
	d->dbglvl = (unsigned int) level;

	return size;
}

static int p7i2cm_dbglvl_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, p7i2cm_dbglvl_show, inode->i_private);
}
static const struct file_operations p7i2cm_dbglvl_debug_ops = {
	.open           = p7i2cm_dbglvl_debug_open,
	.read           = seq_read,
	.write          = p7i2cm_dbglvl_store,
	.llseek         = seq_lseek,
	.release        = single_release,
};

#define VALIDATION_TESTS
#ifdef VALIDATION_TESTS

static int p7i2cm_test_clkstch_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "Write a value in this file to launch the clock stretching test\n");

	return 0;
}

static ssize_t p7i2cm_test_clkstch_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *s = file->private_data;
	struct p7i2cm_debugfs   *d = s->private;

	/* Read the 4 ID registers from P7MU (8 bytes at @ 0x1A) */
	__raw_writel(I2CM_WFIFO_START | I2CM_WFIFO_WR | (0x31 << 1),
	             d->base + I2CM_WFIFO);
	__raw_writel(I2CM_WFIFO_WR | 0x0,
	             d->base + I2CM_WFIFO);
	__raw_writel(I2CM_WFIFO_WR | 0x1A,
	             d->base + I2CM_WFIFO);

	__raw_writel(I2CM_WFIFO_START | I2CM_WFIFO_WR | (0x31 << 1) | 1,
	             d->base + I2CM_WFIFO);
	__raw_writel(I2CM_WFIFO_LAST_NACK | I2CM_WFIFO_RD | 8,
	             d->base + I2CM_WFIFO);
	__raw_writel(I2CM_WFIFO_STOP, d->base + I2CM_WFIFO);

	return size;
}

static int p7i2cm_test_clkstch_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, p7i2cm_test_clkstch_show, inode->i_private);
}
static const struct file_operations p7i2cm_test_clkstch_debug_ops = {
	.open           = p7i2cm_test_clkstch_debug_open,
	.read           = seq_read,
	.write          = p7i2cm_test_clkstch_store,
	.llseek         = seq_lseek,
	.release        = single_release,
};

#endif /* VALIDATION_TESTS */

int p7i2cm_debugfs_init(struct p7i2cm_debugfs *d,
                        void __iomem *base,
                        const char *name,
                        unsigned long in_clk)
{
	struct dentry   *file;
	int              cnt;
	int              ret = 0;

	/* We have to create the root debugfs directory if we're the first one
	 * using it */
	if (atomic_inc_return(&p7i2cm_debugfs_cnt) == 1)
		p7i2cm_debugfs_root = debugfs_create_dir("i2c", NULL);

	if (IS_ERR_OR_NULL(p7i2cm_debugfs_root)) {
		ret = PTR_ERR(p7i2cm_debugfs_root) ?: -ENOMEM;
		goto release;
	}

	d->base = base;
	d->in_clk = in_clk;
	d->dbglvl = 0;
	d->dir = debugfs_create_dir(name, p7i2cm_debugfs_root);
	if (IS_ERR_OR_NULL(d->dir)) {
		ret = PTR_ERR(d->dir) ?: -ENOMEM;
		goto release;
	}

	file = debugfs_create_file("speed", S_IRUGO, d->dir,
	                           d, &p7i2cm_speed_debug_ops);
	if (IS_ERR_OR_NULL(file)) {
		ret = PTR_ERR(file) ?: -ENOMEM;
		goto rm;
	}

	file = debugfs_create_file("highspeed", S_IRUGO, d->dir,
	                           d, &p7i2cm_highspeed_debug_ops);
	if (IS_ERR_OR_NULL(file)) {
		ret = PTR_ERR(file) ?: -ENOMEM;
		goto rm;
	}

	file = debugfs_create_file("dbglvl", S_IRUGO, d->dir,
	                           d, &p7i2cm_dbglvl_debug_ops);
	if (IS_ERR_OR_NULL(file)) {
		ret = PTR_ERR(file) ?: -ENOMEM;
		goto rm;
	}

#ifdef VALIDATION_TESTS
	file = debugfs_create_file("test1_clkstrch", S_IRUGO, d->dir,
	                           d, &p7i2cm_test_clkstch_debug_ops);
	if (IS_ERR_OR_NULL(file)) {
		ret = PTR_ERR(file) ?: -ENOMEM;
		goto rm;
	}
#endif /* VALIDATION_TESTS */

	printk(KERN_INFO "%s debugfs interface registered\n", name);

	return 0;

rm:
	debugfs_remove_recursive(d->dir);
release:
	cnt = atomic_dec_return(&p7i2cm_debugfs_cnt);
	if (cnt == 0 && !IS_ERR_OR_NULL(p7i2cm_debugfs_root))
		/* nobody is using the rootdir anymore, destroy it */
		debugfs_remove_recursive(p7i2cm_debugfs_root);

	d->base = NULL;

	return ret;
}
EXPORT_SYMBOL(p7i2cm_debugfs_init);

void p7i2cm_debugfs_remove(struct p7i2cm_debugfs *d)
{
	int cnt;

	if (d->base) {
		debugfs_remove_recursive(d->dir);

		cnt = atomic_dec_return(&p7i2cm_debugfs_cnt);

		if (cnt == 0 && !IS_ERR_OR_NULL(p7i2cm_debugfs_root))
			/* nobody is using the rootdir anymore, destroy it */
			debugfs_remove_recursive(p7i2cm_debugfs_root);
	}
}
EXPORT_SYMBOL(p7i2cm_debugfs_remove);

MODULE_DESCRIPTION("P7-I2CM debugfs module");
MODULE_AUTHOR("Lionel Flandrin <lionel.flandrin@parrot.com>");
MODULE_LICENSE("GPL");
