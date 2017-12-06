#ifndef CONFIG_H
#define CONFIG_H

#ifndef le16toh
#  define le16toh(x) (x)
#endif

#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

#ifdef __arm__
#include <unistd.h>
#include <sys/syscall.h>
static inline int inotify_init1(int flags) {return syscall(360, flags);}
#endif


#define ENABLE_DEBUG 1
#define ENABLE_LOGGING 1
#define UDEV_DEV_PREFIX ""
#define SYSCONFDIR "/etc/"
#define LIBEXECDIR "/lib/udev"
#define VERSION "164"

#ifdef ANDROID

#undef SYSCONFDIR
#undef LIBEXECDIR
#define SYSCONFDIR "/system/lib"
#define LIBEXECDIR "/system/lib/udev"

#include <stdio.h>
#include <string.h>
static inline void *mempcpy(void *dest, const void *src, size_t n)
{
	memcpy(dest, src, n);
	return ((char *)dest)+n;
}

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

static inline int readlinkat(int dirfd,
		const char *pathname, char *buf, size_t bufsiz)
{
	int result = -1;
	char* buffer = NULL;

	if (dirfd >=0 && pathname && pathname[0] != '/') {
		static const char procfd[] = "/proc/self/fd/%d/%s";
		size_t buflen = sizeof(procfd) + sizeof(int) * 3 + strlen(pathname);
		buffer = (char*)malloc(buflen);
		if(buffer) {
			snprintf(buffer, buflen, procfd, dirfd, pathname);
			result = readlink(buffer, buf, bufsiz);
			free(buffer);
		}
	}
	return result;
}

static inline int mknodat(int dirfd,
		const char *pathname, mode_t mode, dev_t dev)
{
	int result = -1;
	char* buffer = NULL;

	if (dirfd >=0 && pathname && pathname[0] != '/') {
		static const char procfd[] = "/proc/self/fd/%d/%s";
		size_t buflen = sizeof(procfd) + sizeof(int) * 3 + strlen(pathname);
		buffer = (char*)malloc(buflen);
		if(buffer) {
			snprintf(buffer, buflen, procfd, dirfd, pathname);
			result = mknod(buffer, mode, dev);
			free(buffer);
		}
	}
	return result;
}

static inline int symlinkat(const char *oldpath,
		int newdirfd, const char *newpath)
{
	int result = -1;
	char* buffer = NULL;

	if (newdirfd >= 0 && newpath && newpath[0] != '/') {
		static const char procfd[] = "/proc/self/fd/%d/%s";
		size_t buflen = sizeof(procfd) + sizeof(int) * 3 + strlen(newpath);
		buffer = (char*)malloc(buflen);
		if(buffer) {
			snprintf(buffer, buflen, procfd, newdirfd, newpath);
			result = symlink(oldpath, buffer);
			free(buffer);
		}
	}
	return result;
}

static inline void util_set_fd_cloexec(int fd)
{
	int flags;

	flags = fcntl(fd, F_GETFD);
	if (flags < 0)
		flags = FD_CLOEXEC;
	else
		flags |= FD_CLOEXEC;
	fcntl(fd, F_SETFD, flags);
}

#ifndef le16toh
#  define le16toh(x) (x)
#endif

#ifndef MIN
#define MIN(X,Y)				\
	({ typeof (X) x_ = (X);			\
		typeof (Y) y_ = (Y);		\
		(x_ < y_) ? x_ : y_; })
#endif

#ifndef MAX
#define MAX(X,Y)				\
	({ typeof (X) x_ = (X);			\
		typeof (Y) y_ = (Y);		\
		(x_ > y_) ? x_ : y_; })
#endif

#include <sys/stat.h>
#include <sys/syscall.h>
#ifndef UTIME_NOW
# define UTIME_NOW      ((1l << 30) - 1l)
# define UTIME_OMIT     ((1l << 30) - 2l)
static inline int utimensat(int dirfd, const char *pathname,
			    const struct timespec times[2], int flags)
{
	return syscall(348, dirfd, pathname, times, flags);
}
#endif

#include <signal.h>
#include <poll.h>
static inline int ppoll(struct pollfd *fds, nfds_t nfds,
			const struct timespec *timeout,
			const sigset_t *sigmask)
{
	int ms = -1, ready;
	sigset_t sigmask2[2]  = {*sigmask, 0};
	sigset_t origmask2[2];

	if (timeout) {
		ms = timeout->tv_sec*1000 + timeout->tv_nsec/1000000;
	}

	sigprocmask(SIG_SETMASK, &sigmask2[0], &origmask2[0]);
	ready = poll(fds, nfds, ms);
	sigprocmask(SIG_SETMASK, &origmask2[0], NULL);
	return ready;
}

#include <pwd.h>
static inline int _getpwnam_r(const char *name, struct passwd *pwd,
			      char *buf, size_t buflen, struct passwd **result)
{
	*result = NULL;
	return 0;
}
#define getpwnam_r _getpwnam_r
#include <grp.h>
static inline int _getgrnam_r(const char *name, struct group *grp,
			      char *buf, size_t buflen, struct group **result)
{
	*result = NULL;
	return 0;
}
#define getgrnam_r _getgrnam_r

#define SOCK_CLOEXEC 02000000
#define IN_CLOEXEC   02000000

#define BSG_PROTOCOL_SCSI		0

#define BSG_SUB_PROTOCOL_SCSI_CMD	0
#define BSG_SUB_PROTOCOL_SCSI_TMF	1
#define BSG_SUB_PROTOCOL_SCSI_TRANSPORT	2

struct sg_io_v4 {
	signed int guard;		/* [i] 'Q' to differentiate from v3 */
	unsigned int protocol;		/* [i] 0 -> SCSI , .... */
	unsigned int subprotocol;	/* [i] 0 -> SCSI command, 1 -> SCSI task
					   management function, .... */

	unsigned int request_len;	/* [i] in bytes */
	unsigned long long int request;		/* [i], [*i] {SCSI: cdb} */
	unsigned long long int request_tag;	/* [i] {SCSI: task tag (only if flagged)} */
	unsigned int request_attr;	/* [i] {SCSI: task attribute} */
	unsigned int request_priority;	/* [i] {SCSI: task priority} */
	unsigned int request_extra;	/* [i] {spare, for padding} */
	unsigned int max_response_len;	/* [i] in bytes */
	unsigned long long int response;		/* [i], [*o] {SCSI: (auto)sense data} */

        /* "dout_": data out (to device); "din_": data in (from device) */
	unsigned int dout_iovec_count;	/* [i] 0 -> "flat" dout transfer else
					   dout_xfer points to array of iovec */
	unsigned int dout_xfer_len;	/* [i] bytes to be transferred to device */
	unsigned int din_iovec_count;	/* [i] 0 -> "flat" din transfer */
	unsigned int din_xfer_len;	/* [i] bytes to be transferred from device */
	unsigned long long int dout_xferp;	/* [i], [*i] */
	unsigned long long int din_xferp;	/* [i], [*o] */

	unsigned int timeout;		/* [i] units: millisecond */
	unsigned int flags;		/* [i] bit mask */
	unsigned long long int usr_ptr;		/* [i->o] unused internally */
	unsigned int spare_in;		/* [i] */

	unsigned int driver_status;	/* [o] 0 -> ok */
	unsigned int transport_status;	/* [o] 0 -> ok */
	unsigned int device_status;	/* [o] {SCSI: command completion status} */
	unsigned int retry_delay;	/* [o] {SCSI: status auxiliary information} */
	unsigned int info;		/* [o] additional information */
	unsigned int duration;		/* [o] time to complete, in milliseconds */
	unsigned int response_len;	/* [o] bytes of response actually written */
	signed int din_resid;	/* [o] din_xfer_len - actual_din_xfer_len */
	signed int dout_resid;	/* [o] dout_xfer_len - actual_dout_xfer_len */
	unsigned long long int generated_tag;	/* [o] {SCSI: transport generated task tag} */
	unsigned int spare_out;	/* [o] */

	unsigned int padding;
};

#endif /* ANDROID */
#endif /* CONFIG_H */
