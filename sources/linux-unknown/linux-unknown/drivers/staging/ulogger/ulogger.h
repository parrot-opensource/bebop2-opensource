/* include/linux/ulogger.h
 *
 * A fork of the Android Logger.
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_ULOGGER_H
#define _LINUX_ULOGGER_H

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/version.h>

/*
 * The userspace structure for version 1 of the ulogger_entry ABI.
 * This structure is returned to userspace unless the caller requests
 * an upgrade to a newer ABI version.
 */
struct user_ulogger_entry_compat {
	__u16		len;	/* length of the payload */
	__u16		__pad;	/* no matter what, we get 2 bytes of padding */
	__s32		pid;	/* generating process's pid */
	__s32		tid;	/* generating process's tid */
	__s32		sec;	/* seconds since Epoch */
	__s32		nsec;	/* nanoseconds */
	char		msg[0];	/* the entry's payload */
};

/*
 * The structure for version 2 of the ulogger_entry ABI.
 * This structure is returned to userspace if ioctl(ULOGGER_SET_VERSION)
 * is called with version >= 2
 */
struct ulogger_entry {
	__u16		len;		/* length of the payload */
	__u16		hdr_size;	/* sizeof(struct ulogger_entry_v2) */
	__s32		pid;		/* generating process's pid */
	__s32		tid;		/* generating process's tid */
	__s32		sec;		/* seconds since Epoch */
	__s32		nsec;		/* nanoseconds */
	uid_t		euid;		/* effective UID of ulogger */
	char		msg[0];		/* the entry's payload */
};

#define ULOGGER_LOG_MAIN	"ulog_main"	/* everything else */

#define ULOGGER_ENTRY_MAX_PAYLOAD	4076

#define ULOGGER_MAX_LOGS                16

#define __ULOGGERIO	0xAE

#define ULOGGER_GET_LOG_BUF_SIZE	_IO(__ULOGGERIO, 21) /* size of log */
#define ULOGGER_GET_LOG_LEN		_IO(__ULOGGERIO, 22) /* used log len */
#define ULOGGER_GET_NEXT_ENTRY_LEN	_IO(__ULOGGERIO, 23) /* next entry len */
#define ULOGGER_FLUSH_LOG		_IO(__ULOGGERIO, 24) /* flush log */
#define ULOGGER_GET_VERSION		_IO(__ULOGGERIO, 25) /* abi version */
#define ULOGGER_SET_VERSION		_IO(__ULOGGERIO, 26) /* abi version */

/* Can we take advantage of misc_open() setting a pointer to our device ? */
static inline int file_private_data_has_miscdevice_pointer(void)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	return 1;
#else
	return 0;
#endif
}

static inline int ulogger_use_monotonic_clock(void)
{
#if defined(CONFIG_ULOGGER_USE_MONOTONIC_CLOCK)
	return 1;
#else
	return 0;
#endif /* defined(CONFIG_ULOGGER_USE_MONOTONIC_CLOCK) */
}

#endif /* _LINUX_ULOGGER_H */
