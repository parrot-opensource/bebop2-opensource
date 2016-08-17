#ifndef CONFIG_H
#define CONFIG_H

#ifndef le16toh
#  define le16toh(x) (x)
#endif

extern long int syscall(long int __sysno, ...);
static inline int inotify_init1(int flags) {return syscall(360, flags);}

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#define UDEV_DEV_PREFIX "/tmp/udev/"
#define SYSCONFDIR "/etc/"
#define LIBEXECDIR "/lib/udev"
#define VERSION "164"

#endif /* CONFIG_H */
