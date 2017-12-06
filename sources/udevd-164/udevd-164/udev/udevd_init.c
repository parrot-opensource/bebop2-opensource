/*
 * udevd startup helpers
 *
 * Author: Ivan Djelic <ivan.djelic@parrot.com>
 * Copyright (C) 2013 Parrot S.A.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/vfs.h>
#include <libgen.h>

#include "udev.h"

/* check if dev path is on a tmpfs filesystem */
int udev_check_dev_path(struct udev *udev)
{
	int ret;
	struct statfs fs;
	char path[UTIL_PATH_SIZE];

	util_strscpyl(path, sizeof(path), udev_get_dev_path(udev), "/.udev",
		      NULL);

	/* if path does not exist, create it */
	ret = statfs(path, &fs);
	if ((ret < 0) && (errno == ENOENT)) {
		util_create_path(udev, path);
		ret = statfs(dirname(path), &fs);
	}
	if (ret < 0) {
		err(udev, "cannot statfs '%s': %s\n", path, strerror(errno));
		return -1;
	}
	if (fs.f_type != 0x01021994 /* TMPFS_MAGIC */) {
		err(udev, "'%s' is not on a tmpfs filesystem\n", path);
		return -1;
	}

	return 0;
}

/* read the first line of a file */
static const char *read_line(const char *filename)
{
	FILE *fp;
	static char buf[256];
	const char *ret = NULL;

	fp = fopen(filename, "r");
	if (fp) {
		if (fgets(buf, sizeof(buf), fp))
			ret = buf;
		fclose(fp);
	}
	return ret;
}

/* write a single line in a file */
static int write_line(const char *filename, const char *line)
{
	FILE *fp;
	int ret = -1;

	fp = fopen(filename, "w");
	if (fp) {
		if (fputs(line, fp) >= 0)
			ret = 0;
		fchmod(fileno(fp), S_IRWXU|S_IRGRP|S_IROTH);
		fclose(fp);
	}
	return ret;
}

/* write uevent kernel seqnum in udev cold seqnum file */
static int write_udev_cold_seqnum(struct udev *udev, const char *filename)
{
	const char *seqnum;

	/* get current seqnum */
	seqnum = read_line("/sys/kernel/uevent_seqnum");
	if (seqnum == NULL) {
		err(udev, "cannot read current seqnum\n");
		return -1;
	}

	if (write_line(filename, seqnum) < 0) {
		err(udev, "cannot write cold seqnum in file %s\n", filename);
		return -1;
	}
	return 0;
}

static void *xrealloc(struct udev *udev, void *ptr, size_t size)
{
	void *ret;
	ret = realloc(ptr, size);
	if (!ret) {
		err(udev, "realloc: %s\n", strerror(errno));
		exit(1);
	}
	return ret;
}

static char **make_trigger_argv(struct udev *udev, int *out_argc)
{
	FILE *fp;
	char *s;
	char **argv = NULL;
	int i, argc = 0;
	char buf[512];
	static const char * const trigger_argv[] = {
		"trigger",
		"--action=add",
		/* default subsystems on which we trigger uevents */
		"--subsystem-match=hidraw",
		"--subsystem-match=mmc",
		"--subsystem-match=usb",
		"--subsystem-match=tty",
		"--subsystem-match=net",
		"--subsystem-match=block",
		"--subsystem-match=sound",
		"--subsystem-match=firmware",
	};

	/* do we have custom trigger args ? */
	fp = fopen("/etc/udev_trigger_args.conf", "r");
	if (fp) {
		while (fgets(buf, sizeof(buf), fp)) {
			/* squash line return */
			s = strchr(buf, '\n');
			if (s)
				*s = '\0';
			argv = xrealloc(udev, argv, (++argc)*sizeof(*argv));
			argv[argc-1] = strdup(buf);
		}
		fclose(fp);
	}

	/* if not, use default args */
	if (argc == 0) {
		for (i = 0; i < (int)ARRAY_SIZE(trigger_argv); i++) {
			argv = xrealloc(udev, argv, (++argc)*sizeof(*argv));
			argv[argc-1] = strdup(trigger_argv[i]);
		}
	}
	*out_argc = argc;
	return argv;
}

static void destroy_trigger_argv(char **argv, int argc)
{
	int i;
	for (i = 0; i < argc; i++)
		free(argv[i]);
	free(argv);
}

void udev_spawn_trigger(struct udev *udev)
{
	pid_t pid;
	int argc;
	char **argv;
	char filename[UTIL_PATH_SIZE];

	/* where to store udevd cold seqnum */
	snprintf(filename, sizeof(filename), "%s/.udevd_cold_seqnum",
		 udev_get_dev_path(udev));

	/* do trigger only once, not when we restart */
	if (read_line(filename)) {
		dbg(udev, "detected cold seqnum file, skipping trigger...\n");
		return;
	}

	pid = fork();
	if (pid < 0) {
		err(udev, "cannot fork: %s\n", strerror(errno));
		return;
	}
	if (pid != 0)
		/* nothing more to do in parent */
		return;

	/* get custom list of arguments to udevadm trigger */
	argv = make_trigger_argv(udev, &argc);
	dbg(udev, "starting trigger...\n");
	udevadm_trigger(udev, argc, argv);
	destroy_trigger_argv(argv, argc);

	/* trigger phase completed, store current seqnum as cold seqnum */
	dbg(udev, "completed trigger\n");
	write_udev_cold_seqnum(udev, filename);

	/* udevd will catch SIGCHLD and ignore us (we are not in worker list) */
	udev_unref(udev);
	exit(0);
}
