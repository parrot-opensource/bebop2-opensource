#ifndef _BOARD_SYSFS_H_
#define _BOARD_SYSFS_H_

#include <linux/init.h>

#ifdef CONFIG_SYSFS

extern void __init p7brd_export_i2c_hw_infos(int i2c_bus,
					     unsigned short addr,
					     char *version,
					     const char *extra);

extern void __init p7brd_export_uart_hw_infos(int uart,
					      int rts_cts,
					      char *version);

extern int __init p7brd_export_mpmc_priorities(void);

#if defined(CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE)
extern int __init p7brd_export_fb_positions(void);
#else
static inline int __init p7brd_export_fb_positions(void) { return -ENOSYS; }
#endif

#else
/* No sysfs : info on dmesg only */
static inline void p7brd_export_i2c_hw_infos(int i2c_bus,
					     unsigned short addr,
					     char *version,
					     const char *extra)
{
	printk(KERN_INFO "hw_info : i2c-%d device 0x%02X (%s) : %s\n",
	       i2c_bus, addr, version, extra);
}

static inline void p7brd_export_uart_hw_infos(int uart,
					      int rts_cts,
					      char *version)
{
	printk(KERN_INFO "hw_info : uart %d (%s) rts/cts %d\n",
	       uart, version, rts_cts);
}

static inline int p7brd_export_mpmc_priorities()
{
	return -ENOSYS;
}

static inline int __init p7brd_export_fb_positions(void)
{
	return -ENOSYS;
}

#endif

#endif /* _BOARD_SYSFS_H_ */
