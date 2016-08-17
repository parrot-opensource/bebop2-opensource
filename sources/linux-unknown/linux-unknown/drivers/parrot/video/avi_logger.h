#ifndef _AVI_LOGGER_H_
#define _AVI_LOGGER_H_

#include <linux/errno.h>

enum avi_log_type {
	AVI_LOG_NONE  = 0,
	AVI_LOG_READ  = 1,
	AVI_LOG_WRITE = 2,
};

#ifdef CONFIG_AVI_LOGGER

extern int avi_log_init(void);
extern void avi_log_access(u32 reg, u32 val, enum avi_log_type type);
extern int avi_log_display(struct seq_file *s);

#else /* CONFIG_AVI_LOGGER */

static inline int avi_log_init(void)
{
	return -ENODEV;
}

static inline void avi_log_access(u32 reg, u32 val, enum avi_log_type type)
{
	return;
}

static inline int avi_log_display(struct seq_file *s)
{
	return -ENODEV;
}
#endif /* CONFIG_AVI_LOGGER */

#endif /* _AVI_LOGGER_H_ */
