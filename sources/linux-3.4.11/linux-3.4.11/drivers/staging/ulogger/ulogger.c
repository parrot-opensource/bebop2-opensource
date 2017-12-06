/*
 * drivers/misc/ulogger.c
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
 */

#include <linux/sched.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>

#include "ulogger.h"

#include <asm/ioctls.h>

/*
 * struct ulogger_log - represents a specific log, such as 'main' or 'radio'
 *
 * This structure lives from module insertion until module removal, so it does
 * not need additional reference counting. The structure is protected by the
 * mutex 'mutex'.
 */
struct ulogger_log {
	unsigned char		*buffer;/* the ring buffer itself */
	struct miscdevice	misc;	/* misc device representing the log */
	wait_queue_head_t	wq;	/* wait queue for readers */
	struct list_head	readers; /* this log's readers */
	struct rt_mutex		mutex;	/* mutex protecting buffer */
	size_t			w_off;	/* current write head offset */
	size_t			head;	/* new readers start here */
	size_t			size;	/* size of the log */
	size_t			dropped;/* number of globally dropped entries */
};

/*
 * struct ulogger_reader - a logging device open for reading
 *
 * This object lives from open to release, so we don't need additional
 * reference counting. The structure is protected by log->mutex.
 */
struct ulogger_reader {
	struct ulogger_log	*log;	/* associated log */
	struct list_head	list;	/* entry in ulogger_log's list */
	size_t			r_off;	/* current read head offset */
	bool			r_all;	/* reader can read all entries */
	int			r_ver;	/* reader ABI version */
	size_t			r_dropped; /* dropped entries for reader */
};

/* ulogger_offset - returns index 'n' into the log via (optimized) modulus */
size_t ulogger_offset(struct ulogger_log *log, size_t n)
{
	return n & (log->size-1);
}


/*
 * file_get_log - Given a file structure, return the associated log
 *
 * This isn't aesthetic. We have several goals:
 *
 *	1) Need to quickly obtain the associated log during an I/O operation
 *	2) Readers need to maintain state (ulogger_reader)
 *	3) Writers need to be very fast (open() should be a near no-op)
 *
 * In the reader case, we can trivially go file->ulogger_reader->ulogger_log.
 * For a writer, we don't want to maintain a ulogger_reader, so we just go
 * file->ulogger_log. Thus what file->private_data points at depends on whether
 * or not the file was opened for reading. This function hides that dirtiness.
 */
static inline struct ulogger_log *file_get_log(struct file *file)
{
	if (file->f_mode & FMODE_READ) {
		struct ulogger_reader *reader = file->private_data;
		return reader->log;
	} else
		return file->private_data;
}

/*
 * get_entry_header - returns a pointer to the ulogger_entry header within
 * 'log' starting at offset 'off'. A temporary ulogger_entry 'scratch' must
 * be provided. Typically the return value will be a pointer within
 * 'ulogger->buf'.  However, a pointer to 'scratch' may be returned if
 * the log entry spans the end and beginning of the circular buffer.
 */
static struct ulogger_entry *get_entry_header(struct ulogger_log *log,
		size_t off, struct ulogger_entry *scratch)
{
	size_t len = min(sizeof(struct ulogger_entry), log->size - off);
	if (len != sizeof(struct ulogger_entry)) {
		memcpy(((void *) scratch), log->buffer + off, len);
		memcpy(((void *) scratch) + len, log->buffer,
			sizeof(struct ulogger_entry) - len);
		return scratch;
	}

	return (struct ulogger_entry *) (log->buffer + off);
}

/*
 * get_entry_msg_len - Grabs the length of the message of the entry
 * starting from from 'off'.
 *
 * An entry length is 2 bytes (16 bits) in host endian order.
 * In the log, the length does not include the size of the log entry structure.
 * This function returns the size including the log entry structure.
 *
 * Caller needs to hold log->mutex.
 */
static __u32 get_entry_msg_len(struct ulogger_log *log, size_t off)
{
	struct ulogger_entry scratch;
	struct ulogger_entry *entry;

	entry = get_entry_header(log, off, &scratch);
	return entry->len;
}

static size_t get_user_hdr_len(int ver)
{
	if (ver < 2)
		return sizeof(struct user_ulogger_entry_compat);
	else
		return sizeof(struct ulogger_entry);
}

static ssize_t copy_header_to_user(int ver, struct ulogger_entry *entry,
					 char __user *buf)
{
	void *hdr;
	size_t hdr_len;
	struct user_ulogger_entry_compat v1;

	if (ver < 2) {
		v1.len      = entry->len;
		v1.__pad    = 0;
		v1.pid      = entry->pid;
		v1.tid      = entry->tid;
		v1.sec      = entry->sec;
		v1.nsec     = entry->nsec;
		hdr         = &v1;
		hdr_len     = sizeof(struct user_ulogger_entry_compat);
	} else {
		hdr         = entry;
		hdr_len     = sizeof(struct ulogger_entry);
	}

	return copy_to_user(buf, hdr, hdr_len);
}

/*
 * do_read_drop_summary_to_user - provides a summary of dropped log entries to
 * user-space buffer 'buf'.
 *
 * Caller must hold log->mutex.
 */
static ssize_t do_read_drop_summary(struct ulogger_log *log,
				    struct ulogger_reader *reader,
				    char __user *buf,
				    size_t count)
{
	int ret;
	size_t hdrlen;
	char msgbuf[128];
	struct ulogger_entry *entry;
	struct ulogger_entry summary, scratch;

	/*
	 * First, copy the header to userspace, using the version of
	 * the header requested
	 */
	entry = get_entry_header(log, reader->r_off, &scratch);

	/* build a fake log entry indicating how many entries were dropped */
	memcpy(&summary, entry, sizeof(summary));
	summary.pid = 0;
	summary.tid = 0;

	ret = snprintf(msgbuf, sizeof(msgbuf),
		       /* <pname>\0<tname>\0<priority:4><tag>\0<message> */
		       "%c%c%c%c%culog%c%d log entries dropped\n",
		       '\0',       /* empty process name, no thread (pid=tid) */
		       4, 0, 0, 0, /* WARN prio level */
		       '\0',       /* tag trailing null byte */
		       (int)reader->r_dropped);

	if ((ret < 0) || (ret >= sizeof(msgbuf)))
		return -EFAULT;

	summary.len = ret+1; /* count trailing null byte */
	hdrlen = get_user_hdr_len(reader->r_ver);

	if (count < hdrlen + summary.len)
		return -EINVAL;

	if (copy_header_to_user(reader->r_ver, &summary, buf))
		return -EFAULT;

	buf += hdrlen;

	if (copy_to_user(buf, msgbuf, summary.len))
		return -EFAULT;

	reader->r_dropped = 0;
	return hdrlen + summary.len;
}

/*
 * do_read_log_to_user - reads exactly 'count' bytes from 'log' into the
 * user-space buffer 'buf'. Returns 'count' on success.
 *
 * Caller must hold log->mutex.
 */
static ssize_t do_read_log_to_user(struct ulogger_log *log,
				   struct ulogger_reader *reader,
				   char __user *buf,
				   size_t count)
{
	struct ulogger_entry scratch;
	struct ulogger_entry *entry;
	size_t len;
	size_t msg_start;

	/*
	 * First, copy the header to userspace, using the version of
	 * the header requested
	 */
	entry = get_entry_header(log, reader->r_off, &scratch);
	if (copy_header_to_user(reader->r_ver, entry, buf))
		return -EFAULT;

	count -= get_user_hdr_len(reader->r_ver);
	buf += get_user_hdr_len(reader->r_ver);
	msg_start = ulogger_offset(log,
		reader->r_off + sizeof(struct ulogger_entry));

	/*
	 * We read from the msg in two disjoint operations. First, we read from
	 * the current msg head offset up to 'count' bytes or to the end of
	 * the log, whichever comes first.
	 */
	len = min(count, log->size - msg_start);
	if (copy_to_user(buf, log->buffer + msg_start, len))
		return -EFAULT;

	/*
	 * Second, we read any remaining bytes, starting back at the head of
	 * the log.
	 */
	if (count != len)
		if (copy_to_user(buf + len, log->buffer, count - len))
			return -EFAULT;

	reader->r_off = ulogger_offset(log, reader->r_off +
		sizeof(struct ulogger_entry) + count);

	return count + get_user_hdr_len(reader->r_ver);
}

/*
 * get_next_entry_by_uid - Starting at 'off', returns an offset into
 * 'log->buffer' which contains the first entry readable by 'euid'
 */
static size_t get_next_entry_by_uid(struct ulogger_log *log,
		size_t off, uid_t euid)
{
	while (off != log->w_off) {
		struct ulogger_entry *entry;
		struct ulogger_entry scratch;
		size_t next_len;

		entry = get_entry_header(log, off, &scratch);

		if (entry->euid == euid)
			return off;

		next_len = sizeof(struct ulogger_entry) + entry->len;
		off = ulogger_offset(log, off + next_len);
	}

	return off;
}

/*
 * ulogger_read - our log's read() method
 *
 * Behavior:
 *
 *	- O_NONBLOCK works
 *	- If there are no log entries to read, blocks until log is written to
 *	- Atomically reads exactly one log entry
 *
 * Will set errno to EINVAL if read
 * buffer is insufficient to hold next entry.
 */
static ssize_t ulogger_read(struct file *file, char __user *buf,
			   size_t count, loff_t *pos)
{
	struct ulogger_reader *reader = file->private_data;
	struct ulogger_log *log = reader->log;
	ssize_t ret;
	DEFINE_WAIT(wait);

start:
	while (1) {
		rt_mutex_lock(&log->mutex);

		prepare_to_wait(&log->wq, &wait, TASK_INTERRUPTIBLE);

		ret = (log->w_off == reader->r_off);
		rt_mutex_unlock(&log->mutex);
		if (!ret)
			break;

		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			break;
		}

		if (signal_pending(current)) {
			ret = -EINTR;
			break;
		}

		schedule();
	}

	finish_wait(&log->wq, &wait);
	if (ret)
		return ret;

	rt_mutex_lock(&log->mutex);

	if (!reader->r_all)
		reader->r_off = get_next_entry_by_uid(log,
			reader->r_off, current_euid());

	/* is there still something to read or did we race? */
	if (unlikely(log->w_off == reader->r_off)) {
		rt_mutex_unlock(&log->mutex);
		goto start;
	}

	/*
	 * If this reader has missed dropped entries, return a fake generated
	 * log entry with drop information.
	 */
	if (unlikely((reader->r_dropped > 0))) {
		ret = do_read_drop_summary(log, reader, buf, count);
		goto out;
	}

	/* get the size of the next entry */
	ret = get_user_hdr_len(reader->r_ver) +
		get_entry_msg_len(log, reader->r_off);
	if (count < ret) {
		ret = -EINVAL;
		goto out;
	}

	/* get exactly one entry from the log */
	ret = do_read_log_to_user(log, reader, buf, ret);

out:
	rt_mutex_unlock(&log->mutex);

	return ret;
}

/*
 * get_next_entry - return the offset of the first valid entry at least 'len'
 * bytes after 'off'.
 *
 * Caller must hold log->mutex.
 */
static size_t get_next_entry(struct ulogger_log *log, size_t off, size_t len,
			     size_t *dropped)
{
	size_t count = 0, entries = 0;

	do {
		size_t nr = sizeof(struct ulogger_entry) +
			get_entry_msg_len(log, off);
		off = ulogger_offset(log, off + nr);
		count += nr;
		entries++;
	} while (count < len);

	*dropped = entries;
	return off;
}

/*
 * is_between - is a < c < b, accounting for wrapping of a, b, and c
 *    positions in the buffer
 *
 * That is, if a<b, check for c between a and b
 * and if a>b, check for c outside (not between) a and b
 *
 * |------- a xxxxxxxx b --------|
 *               c^
 *
 * |xxxxx b --------- a xxxxxxxxx|
 *    c^
 *  or                    c^
 */
static inline int is_between(size_t a, size_t b, size_t c)
{
	if (a < b) {
		/* is c between a and b? */
		if (a < c && c <= b)
			return 1;
	} else {
		/* is c outside of b through a? */
		if (c <= b || a < c)
			return 1;
	}

	return 0;
}

/*
 * fix_up_readers - walk the list of all readers and "fix up" any who were
 * lapped by the writer; also do the same for the default "start head".
 * We do this by "pulling forward" the readers and start head to the first
 * entry after the new write head.
 *
 * The caller needs to hold log->mutex.
 */
static void fix_up_readers(struct ulogger_log *log, size_t len)
{
	size_t old = log->w_off, dropped;
	size_t new = ulogger_offset(log, old + len);
	struct ulogger_reader *reader;

	if (is_between(old, new, log->head)) {
		log->head = get_next_entry(log, log->head, len, &dropped);
		log->dropped += dropped;
	}

	list_for_each_entry(reader, &log->readers, list)
		if (is_between(old, new, reader->r_off)) {
			reader->r_off = get_next_entry(log, reader->r_off, len,
						       &dropped);
			reader->r_dropped += dropped;
		}
}

/*
 * do_write_log - writes 'len' bytes from 'buf' to 'log'
 *
 * The caller needs to hold log->mutex.
 */
static void do_write_log(struct ulogger_log *log, const void *buf, size_t count)
{
	size_t len;

	len = min(count, log->size - log->w_off);
	memcpy(log->buffer + log->w_off, buf, len);

	if (count != len)
		memcpy(log->buffer, buf + len, count - len);

	log->w_off = ulogger_offset(log, log->w_off + count);

}

/*
 * do_write_log_user - writes 'len' bytes from the user-space buffer 'buf' to
 * the log 'log'
 *
 * The caller needs to hold log->mutex.
 *
 * Returns 'count' on success, negative error code on failure.
 */
static ssize_t do_write_log_from_user(struct ulogger_log *log,
				      const void __user *buf, size_t count)
{
	size_t len;

	len = min(count, log->size - log->w_off);
	if (len && copy_from_user(log->buffer + log->w_off, buf, len))
		return -EFAULT;

	if (count != len)
		if (copy_from_user(log->buffer, buf + len, count - len))
			/*
			 * Note that by not updating w_off, this abandons the
			 * portion of the new entry that *was* successfully
			 * copied, just above.  This is intentional to avoid
			 * message corruption from missing fragments.
			 */
			return -EFAULT;

	log->w_off = ulogger_offset(log, log->w_off + count);

	return count;
}

/*
 * ulogger_aio_write - our write method, implementing support for write(),
 * writev(), and aio_write(). Writes are our fast path, and we try to optimize
 * them above all else.
 */
ssize_t ulogger_aio_write(struct kiocb *iocb, const struct iovec *iov,
			 unsigned long nr_segs, loff_t ppos)
{
	struct ulogger_log *log = file_get_log(iocb->ki_filp);
	size_t orig = log->w_off;
	struct ulogger_entry header;
	struct timespec now;
	ssize_t ret = 0, prefix;
	const size_t commlen = sizeof(current->comm);
	char tcomm[commlen+4];
	char pcomm[commlen+4];
	size_t tlen, plen;

	if (ulogger_use_monotonic_clock())
		getrawmonotonic(&now);
	else
		now = current_kernel_time();

	header.pid = current->tgid;
	header.tid = current->pid;

	/* retrieve process (thread group leader) and thread names */
	memcpy(pcomm, current->group_leader->comm, commlen);
	pcomm[commlen] = '\0';
	plen = strlen(pcomm)+1;
	if (header.pid != header.tid) {
		memcpy(tcomm, current->comm, commlen);
		tcomm[commlen] = '\0';
		tlen = strlen(tcomm)+1;
	} else {
		tlen = 0;
	}

	header.sec = now.tv_sec;
	header.nsec = now.tv_nsec;
	header.euid = current_euid();
	header.len = min_t(size_t, plen+tlen+iocb->ki_left,
			   ULOGGER_ENTRY_MAX_PAYLOAD);
	header.hdr_size = sizeof(struct ulogger_entry);

	/* null writes succeed, return zero */
	if (unlikely(!header.len))
		return 0;

	rt_mutex_lock(&log->mutex);

	/*
	 * Fix up any readers, pulling them forward to the first readable
	 * entry after (what will be) the new write offset. We do this now
	 * because if we partially fail, we can end up with clobbered log
	 * entries that encroach on readable buffer.
	 */
	fix_up_readers(log, sizeof(struct ulogger_entry) + header.len);

	do_write_log(log, &header, sizeof(struct ulogger_entry));

	/* append null-terminated process and thread names */
	do_write_log(log, pcomm, plen);
	prefix = plen;
	if (tlen) {
		do_write_log(log, tcomm, tlen);
		prefix += tlen;
	}

	while (nr_segs-- > 0) {
		size_t len;
		ssize_t nr;

		/* figure out how much of this vector we can keep */
		len = min_t(size_t, iov->iov_len, header.len - ret - prefix);

		/* write out this segment's payload */
		nr = do_write_log_from_user(log, iov->iov_base, len);
		if (unlikely(nr < 0)) {
			log->w_off = orig;
			rt_mutex_unlock(&log->mutex);
			return nr;
		}

		iov++;
		ret += nr;
	}

	rt_mutex_unlock(&log->mutex);

	/* wake up any blocked readers */
	wake_up_interruptible(&log->wq);

	return ret;
}

static struct ulogger_log *get_log_from_minor(int minor);

/*
 * ulogger_open - the log's open() file operation
 *
 * Note how near a no-op this is in the write-only case. Keep it that way!
 */
static int ulogger_open(struct inode *inode, struct file *file)
{
	struct ulogger_log *log;
	int ret;

	ret = nonseekable_open(inode, file);
	if (ret)
		return ret;

	if (file_private_data_has_miscdevice_pointer()) {
		/* file->private_data points to our misc struct */
		log = container_of(file->private_data, struct ulogger_log,
				   misc);
	} else {
		/* slower version for older kernels, requires mutex locking */
		log = get_log_from_minor(MINOR(inode->i_rdev));
		if (!log)
			return -ENODEV;
	}

	if (file->f_mode & FMODE_READ) {
		struct ulogger_reader *reader;

		reader = kmalloc(sizeof(struct ulogger_reader), GFP_KERNEL);
		if (!reader)
			return -ENOMEM;

		reader->log = log;
		reader->r_ver = 2;
		reader->r_all = in_egroup_p(inode->i_gid) ||
			capable(CAP_SYS_ADMIN);

		INIT_LIST_HEAD(&reader->list);

		rt_mutex_lock(&log->mutex);
		reader->r_off = log->head;
		reader->r_dropped = log->dropped;
		list_add_tail(&reader->list, &log->readers);
		rt_mutex_unlock(&log->mutex);

		file->private_data = reader;
	} else
		file->private_data = log;

	return 0;
}

/*
 * ulogger_release - the log's release file operation
 *
 * Note this is a total no-op in the write-only case. Keep it that way!
 */
static int ulogger_release(struct inode *ignored, struct file *file)
{
	if (file->f_mode & FMODE_READ) {
		struct ulogger_reader *reader = file->private_data;
		struct ulogger_log *log = reader->log;

		rt_mutex_lock(&log->mutex);
		list_del(&reader->list);
		rt_mutex_unlock(&log->mutex);

		kfree(reader);
	}

	return 0;
}

/*
 * ulogger_poll - the log's poll file operation, for poll/select/epoll
 *
 * Note we always return POLLOUT, because you can always write() to the log.
 * Note also that, strictly speaking, a return value of POLLIN does not
 * guarantee that the log is readable without blocking, as there is a small
 * chance that the writer can lap the reader in the interim between poll()
 * returning and the read() request.
 */
static unsigned int ulogger_poll(struct file *file, poll_table *wait)
{
	struct ulogger_reader *reader;
	struct ulogger_log *log;
	unsigned int ret = POLLOUT | POLLWRNORM;

	if (!(file->f_mode & FMODE_READ))
		return ret;

	reader = file->private_data;
	log = reader->log;

	poll_wait(file, &log->wq, wait);

	rt_mutex_lock(&log->mutex);
	if (!reader->r_all)
		reader->r_off = get_next_entry_by_uid(log,
			reader->r_off, current_euid());

	if (log->w_off != reader->r_off)
		ret |= POLLIN | POLLRDNORM;
	rt_mutex_unlock(&log->mutex);

	return ret;
}

static long ulogger_set_version(struct ulogger_reader *reader, void __user *arg)
{
	int version;
	if (copy_from_user(&version, arg, sizeof(int)))
		return -EFAULT;

	if ((version < 1) || (version > 2))
		return -EINVAL;

	reader->r_ver = version;
	return 0;
}

static long ulogger_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct ulogger_log *log = file_get_log(file);
	struct ulogger_reader *reader;
	long ret = -EINVAL;
	void __user *argp = (void __user *) arg;

	rt_mutex_lock(&log->mutex);

	switch (cmd) {
	case ULOGGER_GET_LOG_BUF_SIZE:
		ret = log->size;
		break;
	case ULOGGER_GET_LOG_LEN:
		if (!(file->f_mode & FMODE_READ)) {
			ret = -EBADF;
			break;
		}
		reader = file->private_data;
		if (log->w_off >= reader->r_off)
			ret = log->w_off - reader->r_off;
		else
			ret = (log->size - reader->r_off) + log->w_off;
		break;
	case ULOGGER_GET_NEXT_ENTRY_LEN:
		if (!(file->f_mode & FMODE_READ)) {
			ret = -EBADF;
			break;
		}
		reader = file->private_data;

		if (!reader->r_all)
			reader->r_off = get_next_entry_by_uid(log,
				reader->r_off, current_euid());

		if (log->w_off != reader->r_off)
			ret = get_user_hdr_len(reader->r_ver) +
				get_entry_msg_len(log, reader->r_off);
		else
			ret = 0;
		break;
	case ULOGGER_FLUSH_LOG:
		if (!(file->f_mode & FMODE_WRITE)) {
			ret = -EBADF;
			break;
		}
		list_for_each_entry(reader, &log->readers, list) {
			reader->r_off = log->w_off;
			reader->r_dropped = 0;
		}
		log->head = log->w_off;
		log->dropped = 0;
		ret = 0;
		break;
	case ULOGGER_GET_VERSION:
		if (!(file->f_mode & FMODE_READ)) {
			ret = -EBADF;
			break;
		}
		reader = file->private_data;
		ret = reader->r_ver;
		break;
	case ULOGGER_SET_VERSION:
		if (!(file->f_mode & FMODE_READ)) {
			ret = -EBADF;
			break;
		}
		reader = file->private_data;
		ret = ulogger_set_version(reader, argp);
		break;
	}

	rt_mutex_unlock(&log->mutex);

	return ret;
}

static const struct file_operations ulogger_fops = {
	.owner = THIS_MODULE,
	.read = ulogger_read,
	.aio_write = ulogger_aio_write,
	.poll = ulogger_poll,
	.unlocked_ioctl = ulogger_ioctl,
	.compat_ioctl = ulogger_ioctl,
	.open = ulogger_open,
	.release = ulogger_release,
};

/*
 * Defines a static main log structure.
 * Buffer size must be a power of two, and greater than
 * (ULOGGER_ENTRY_MAX_PAYLOAD + sizeof(struct ulogger_entry)).
 */
static unsigned char _buf_log_main[1U << CONFIG_ULOGGER_BUF_SHIFT];

static struct ulogger_log log_main = {
	.buffer = _buf_log_main,
	.misc = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = ULOGGER_LOG_MAIN,
		.fops = &ulogger_fops,
		.parent = NULL,
		.mode = 0666,
	},
	.wq = __WAIT_QUEUE_HEAD_INITIALIZER(log_main.wq),
	.readers = LIST_HEAD_INIT(log_main.readers),
	.mutex = __RT_MUTEX_INITIALIZER(log_main.mutex),
	.w_off = 0,
	.head = 0,
	.dropped = 0,
	.size = sizeof(_buf_log_main),
};

/*
 * ulogger_logs - the array of currently available logging devices
 *
 * New devices can be added by writing to a sysfs attribute of log_main.
 */
static struct ulogger_log *ulogger_logs[ULOGGER_MAX_LOGS] = {&log_main};

/* logs_mutex - protects access to ulogger_logs */
static DEFINE_MUTEX(logs_mutex);

static struct ulogger_log *get_log_from_minor(int minor)
{
	unsigned int i;
	struct ulogger_log *log = NULL;

	if (log_main.misc.minor == minor)
		return &log_main;

	mutex_lock(&logs_mutex);

	for (i = 0; i < ARRAY_SIZE(ulogger_logs); i++) {
		if (ulogger_logs[i] == NULL)
			break;
		if (ulogger_logs[i]->misc.minor == minor) {
			log = ulogger_logs[i];
			break;
		}
	}

	mutex_unlock(&logs_mutex);

	return log;
}

static int init_log(struct ulogger_log *log)
{
	int ret;

	ret = misc_register(&log->misc);
	if (unlikely(ret)) {
		pr_err("failed to register misc device for log '%s'!\n",
		       log->misc.name);
		return ret;
	}

	pr_info("created %luK log '%s'\n",
		(unsigned long) log->size >> 10, log->misc.name);

	return 0;
}

/*
 * List all log buffers.
 */
static ssize_t ulogger_show_logs(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	unsigned int i;
	ssize_t count = 0;
	const unsigned int length = PAGE_SIZE/ARRAY_SIZE(ulogger_logs);

	mutex_lock(&logs_mutex);

	for (i = 0; i < ARRAY_SIZE(ulogger_logs); i++) {
		if (ulogger_logs[i] == NULL)
			break;
		count += scnprintf(&buf[count], length, "%s %u\n",
				   ulogger_logs[i]->misc.name,
				   ffs(ulogger_logs[i]->size)-1);
	}

	mutex_unlock(&logs_mutex);

	return count;
}

/*
 * Dynamically allocate and register a new log device.
 */
static ssize_t ulogger_add_log(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	char namebuf[16];
	int i, len, slot, ret = -EINVAL;
	unsigned int size;
	char *name = NULL;
	unsigned char *buffer = NULL;
	struct ulogger_log *log = NULL;

	/* parse buffer specification */
	if (sscanf(buf, "%15s %u", namebuf, &size) != 2)
		goto bad_spec;

	/* log name should start with prefix ulog_ */
	if (strncmp(namebuf, "ulog_", 5) != 0)
		goto bad_spec;

	/* log name should contain readable lowercase alpha characters only */
	len = strlen(namebuf);
	for (i = 5; i < len; i++)
		if (!islower(namebuf[i]))
			goto bad_spec;

	/* sanity check: 16 kB <= size <= 1 MB */
	if ((size < 14) || (size > 20))
		goto bad_spec;

	name = kstrdup(namebuf, GFP_KERNEL);
	size = (1U << size);

	log = kzalloc(sizeof(*log), GFP_KERNEL);
	buffer = vmalloc(size);

	if (!log || !buffer) {
		pr_err("failed to allocate log '%s' size %u\n", name, size);
		ret = -ENOMEM;
		goto fail;
	}

	log->misc.minor = MISC_DYNAMIC_MINOR;
	log->misc.name = name;
	log->misc.fops = &ulogger_fops;
	log->misc.mode = 0666;
	log->size = size;
	log->buffer = buffer;
	init_waitqueue_head(&log->wq);
	INIT_LIST_HEAD(&log->readers);
	rt_mutex_init(&log->mutex);

	/* find a slot for this log device */
	slot = -1;

	mutex_lock(&logs_mutex);
	for (i = 0; i < ARRAY_SIZE(ulogger_logs); i++) {
		if (ulogger_logs[i] == NULL) {
			if (init_log(log))
				break;
			ulogger_logs[i] = log;
			slot = i;
			break;
		}
		if (strcmp(ulogger_logs[i]->misc.name, name) == 0)
			/* we already have a buffer with this name */
			break;
	}
	mutex_unlock(&logs_mutex);

	if (slot < 0)
		/* device registration failed */
		goto fail;

	return count;

bad_spec:
	pr_err("invalid buffer specification\n");
fail:
	kfree(name);
	kfree(log);
	vfree(buffer);
	return ret;
}

static DEVICE_ATTR(logs, S_IWUSR|S_IRUGO, ulogger_show_logs, ulogger_add_log);

static int __init ulogger_init(void)
{
	int ret;

	/* static device 'main' is always present */
	ret = init_log(&log_main);
	if (unlikely(ret))
		goto out;

	/* write-only attribute for dynamically adding new buffers */
	ret = device_create_file(log_main.misc.this_device, &dev_attr_logs);
out:
	return ret;
}
device_initcall(ulogger_init);

MODULE_LICENSE("GPL");
