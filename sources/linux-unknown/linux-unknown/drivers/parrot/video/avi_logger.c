#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/vmalloc.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/seq_file.h>
#include <linux/module.h>

#include "avi_logger.h"

/* Must be a power of two */
#define AVI_LOGGER_BUF_SZ  0x4000

struct avi_register_access {
	/* I store the direction in reg's lsb */
	u32	reg;
	u32	val;
};

struct avi_logger {
	unsigned long			 head;
	unsigned long			 nwrap;
	spinlock_t			 lock;
	struct avi_register_access	*log;
};

static struct avi_logger avi_logger = {
	.head  = 0,
	.nwrap = 0,
	.lock  = __SPIN_LOCK_UNLOCKED(avi_logger.lock),
	.log   = NULL,
};

int avi_log_init(void)
{
	avi_logger.log =
		vmalloc(AVI_LOGGER_BUF_SZ * sizeof(*avi_logger.log));

	if (avi_logger.log == NULL)
		return -ENOMEM;

	memset(avi_logger.log,
	       0,
	       AVI_LOGGER_BUF_SZ * sizeof(*avi_logger.log));

	return 0;
}
EXPORT_SYMBOL(avi_log_init);

void avi_log_access(u32 reg, u32 val, enum avi_log_type type)
{
	unsigned long			flags;
	struct avi_register_access	*a;

	/* The register must be 32bit aligned */
	BUG_ON(reg & 3);
	BUG_ON(type > 3);

	/* Cram the type into the register value to save space */
	reg |= type;

	spin_lock_irqsave(&avi_logger.lock, flags);

	if (avi_logger.log == NULL)
		/* No log available */
		goto unlock;

	avi_logger.head = (avi_logger.head + 1) % AVI_LOGGER_BUF_SZ;

	if (avi_logger.head == 0)
		/* We wrapped around */
		avi_logger.nwrap++;

	a = &avi_logger.log[avi_logger.head];
	a->reg = reg;
	a->val = val;

 unlock:
	spin_unlock_irqrestore(&avi_logger.lock, flags);
}
EXPORT_SYMBOL(avi_log_access);

static void avi_log_format(struct seq_file *s,
                           struct avi_register_access *log,
                           unsigned long head,
                           unsigned long index)
{
	struct avi_register_access	*a;
	enum avi_log_type		 t;
	unsigned long			 i;

	for (i = (head + 1) % AVI_LOGGER_BUF_SZ;
	     i != head;
	     i = (i + 1) % AVI_LOGGER_BUF_SZ) {
		a = &log[i];

		t = a->reg & 3;
		a->reg &= ~3UL;

		if (t != AVI_LOG_NONE)
			seq_printf(s, "[%8lx] %c 0x%08x @ 0x%08x\n",
			           index,
			           (t == AVI_LOG_READ) ? 'R' : 'W',
			           a->val,
			           a->reg);

		index++;
	}
}

int avi_log_display(struct seq_file *s)
{
	struct avi_register_access	*log;
	unsigned long			 head;
	unsigned long			 index;
	unsigned long			 flags;
	int				 ret = 0;

	/* We make a copy of the log before formating it to limit the time we
	 * spend holding the lock */
	log = vmalloc(AVI_LOGGER_BUF_SZ * sizeof(*avi_logger.log));
	if (log == NULL)
		return -ENOMEM;

	memset(log, 0, AVI_LOGGER_BUF_SZ * sizeof(*log));

	spin_lock_irqsave(&avi_logger.lock, flags);

	if (avi_logger.log == NULL) {
		/* No log available */
		ret = -ENODEV;
		goto copy_done;
	}

	memcpy(log, avi_logger.log, AVI_LOGGER_BUF_SZ * sizeof(*log));
	head = avi_logger.head;
	index = avi_logger.nwrap * AVI_LOGGER_BUF_SZ + avi_logger.head - 1;

 copy_done:
	spin_unlock_irqrestore(&avi_logger.lock, flags);

	if (ret == 0)
		avi_log_format(s, log, head, index);

	vfree(log);
	return ret;
}
EXPORT_SYMBOL(avi_log_display);
