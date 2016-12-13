#ifndef _P7_I2CM_DEBUGFS_H_
#define _P7_I2CM_DEBUGFS_H_

#include <linux/debugfs.h>

struct p7i2cm_debugfs {
	struct dentry   *dir;
	void __iomem    *base;
	unsigned long    in_clk;
	unsigned int     dbglvl; /* 0 : no debug message
	                          * 1 : Essential messages
	                          * other : All messages */
};

extern int p7i2cm_debugfs_init(struct p7i2cm_debugfs *, void __iomem *,
                               const char *, unsigned long);
extern void p7i2cm_debugfs_remove(struct p7i2cm_debugfs *);

#endif /* _P7_I2CM_DEBUGFS_H_ */
