#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <getopt.h>
#include <ctype.h>
#include <sys/poll.h>
#include <sys/types.h>
#include <dirent.h>
#ifdef ANDROID
#include <cutils/properties.h>
#endif

#include "udev.h"

/* udev directory permissions */
#define UDEV_DIR_PERMS (S_IRWXU|S_IRGRP|S_IXGRP|S_IROTH|S_IXOTH)

#ifdef dbg
#undef dbg
#endif

static void  __attribute__ ((format (printf, 1, 2)))
dbg(const char *fmt, ...)
{
	struct timeval tm;
	va_list args;

	if (gettimeofday(&tm, NULL) == 0)
		fprintf(stderr, "%ld.%06ld: ", tm.tv_sec, tm.tv_usec);

	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
	va_end(args);
}

static void log_udev(struct udev *udev, int priority,
		   const char *file, int line, const char *fn,
		   const char *format, va_list args)
{
	struct timeval tm;

	if (gettimeofday(&tm, NULL) == 0)
		fprintf(stderr, "%ld.%06ld: ", tm.tv_sec, tm.tv_usec);

	fprintf(stderr, "%s: ", fn);
	vfprintf(stderr, format, args);
}


/**
 * format directory path and create directory with given permission
 * @param perm directory permission
 * @param fmt directory path format
 * @return 0 on success
 */
static int create_dir(int perm, const char *fmt, ...)
{
	int ret;
	struct stat path_stat;
	va_list args;
	char *path, *p;

	va_start(args, fmt);

	/* format directory path */
	ret = vasprintf(&path, fmt, args);
	if (ret < 0) {
		dbg("%s", "can't format path\n");
		ret = -1;
		goto free_valist;
	}

	/* check dir exists */
	if (lstat(path, &path_stat) == 0) {
		/* dir already exists do a chmod */
		chmod(path, perm);
		ret = 0;
		goto free_path;
	}

	/* create each directory from root */
	p = path;
	while (*p != '\0') {
		if (*p == '/' && p != path) {
			/* create dir */
			*p = '\0';
			ret = mkdir(path, perm);
			if (ret < 0 && errno != EEXIST) {
				dbg("mkdir(%s, %d) error(%d):%s\n", path, perm,
				    errno, strerror(errno));
				ret = -1;
				goto free_path;
			}
			*p = '/';
		}
		p++;
	}


	/* create final dir */
	ret = mkdir(path, perm);
	if (ret < 0 && errno != EEXIST) {
		dbg("mkdir(%s, %d) error(%d):%s\n", path, perm, errno,
		    strerror(errno));
		ret = -1;
	} else {
		ret = 0;
	}


free_path:
	free(path);
free_valist:
	va_end(args);
	return ret;
}

/**
 * check a filesystem type is mounted at specific mount point
 *
 * @param fstype filesystem type
 * @param mpoint mount point
 * @return 1 if filesystem is mounted, 0 else
 */
static int is_fs_mounted(const char *fstype, const char *mpoint)
{
	FILE *fp;
	char buf[512];
	const char *p;

	/* parse /proc/self/mountinfo */
	fp = fopen("/proc/self/mountinfo", "r");
	if (!fp)
		return -errno;

	while (fgets(buf, sizeof(buf), fp) != NULL) {
		(void)strtok(buf, " ");
		(void)strtok(NULL, " ");
		(void)strtok(NULL, " ");
		(void)strtok(NULL, " ");

		/* check mount point */
		p = strtok(NULL, " ");
		if (p && (strcmp(p, mpoint) == 0)) {
			(void)strtok(NULL, " ");
			(void)strtok(NULL, " ");
			/* check fs type */
			p = strtok(NULL, " ");
			if (p && (strcmp(p, fstype) == 0)) {
				fclose(fp);
				return 1;
			}
		}
	}
	fclose(fp);
	return 0;
}

/**
 * read board name using Hardware info in /proc/cpuinfo
 *
 * @param board pointer to string allocated by function on success
 * @return 0 on success (string must be freed by user)
 */
static int read_board_name(char **board)
{
	FILE *fp;
	char buf[512];
	static const char hw[] = "Hardware\t: ";
	size_t length, i;
	const char *name;
	int ret;

	/* parse /proc/cpuinfo */
	fp = fopen("/proc/cpuinfo", "r");
	if (!fp) {
		dbg("can't open '/proc/cpuinfo', error(%d):%s\n", errno,
		    strerror(errno));
		return -errno;
	}

	length = strlen(hw);
	while (fgets(buf, sizeof(buf), fp) != NULL) {
		if (strncmp(buf, hw, length) == 0) {
			name = &buf[length];
			i = 0;
			while (name[i] != '\0' && !isblank(name[i]))
				i++;

			*board = strndup(name, i);
			ret = (*board) ? 0 : -ENOMEM;
			goto out;
		}
	}
	ret = -ENOENT;
out:
	fclose(fp);
	return ret;
}

/**
 * read board name and export it as "JUBA_BOARD" udev property
 * @param udev_root udev root directory
 * @return 0 on success
 */
static int export_udev_board(const char *udev_root)
{
	FILE *fp;
	struct stat st;
	char *file_name, *board;
	int ret;

	/* format udev board file name */
	ret = asprintf(&file_name, "%s/.udevd_board", udev_root);
	if (ret < 0) {
		dbg("%s", "can't allocate file path\n");
		return -ENOMEM;
	}

	/* check if file exists */
	if (lstat(file_name, &st) == 0) {
		/* file already exists,
		 * (board name cannot changed on runtime) */
		ret = 0;
		goto free_filename;
	}

	/* read board name  */
	board = NULL;
	ret = read_board_name(&board);
	if (ret < 0)
		dbg("%s", "unable to read board name\n");

	/* create board file */
	fp = fopen(file_name, "w");
	if (!fp) {
		dbg("can't create file '%s', error(%d):%s\n",
			file_name, errno, strerror(errno));
		ret = -errno;
		goto free_board;
	}

	/* export juba board property*/
	fprintf(fp, "JUBA_BOARD=%s\n", board ? board : "native");

	/* update file permissions */
	chmod(file_name, S_IRWXU|S_IRGRP|S_IROTH);

	/* cleanup */
	fclose(fp);

	ret = 0;

free_board:
	free(board);
free_filename:
	free(file_name);
	return ret;
}
/**
 * get all running process id matching process name
 * @param name process name
 * @param pids pointer to array of pids allocated by function
 *        (must be freed by caller)
 * @return number of pids or n egative value on error
 */
extern int get_pids_by_name(const char *name, int **pids)
{
	DIR *dir;
	FILE *fp;
	struct dirent *entry;
	char path[32];
	char buf[128];
	char *end, *pname;
	int count = 0;
	int *array = NULL;


	/* open /proc dir */
	dir = opendir("/proc");
	if (!dir) {
		dbg("can't open /proc, error(%d):%s\n", errno,
		    strerror(errno));
		return -errno;
	}

	/* read /proc dir */
	while ((entry = readdir(dir)) != NULL) {

		/* only read digit (PID) entry */
		if (!isdigit(*entry->d_name))
			continue;

		/* compute process stat file */
		snprintf(path, sizeof(path), "/proc/%s/stat", entry->d_name);

		/* open /proc/[pid]/stat */
		fp = fopen(path, "r");
		if (!fp) {
			dbg("can't open '%s', error(%d):%s\n", path, errno,
			    strerror(errno));
			continue;
		}

		/* read first 128 bytes
		 * process name is the 2 field */
		if (!fgets(buf, sizeof(buf), fp)) {
			fclose(fp);
			continue;
		}

		fclose(fp);

		/* extract process name
		 * process name is only field in parentheses.
		 */
		pname = strchr(buf, '(');
		if (!pname)
			continue;
		pname++;

		end = strchr(pname, ')');
		if (!end)
			continue;
		*end = '\0';

		/* compare process name */
		if (strcmp(pname, name) == 0) {
			/* add pid in array */
			array = (int *)realloc(array,
					sizeof(int)*(1 + count));
			if (!array) {
				closedir(dir);
				return -ENOMEM;
			}
			array[count++] = atoi(entry->d_name);
		}
	}

	closedir(dir);
	*pids = array;
	return count;
}

/**
 * write uevent kerenel seqname in udev cold seqnum file
 * @param udev_root udev root directory
 * @return 0 on success
 */
static int write_udev_seqnum(const char *udev_root)
{
	FILE *fp;
	char seqnum[255];
	char *file_name;
	int ret;

	/* open kernel uevent seqnum file */
	fp = fopen("/sys/kernel/uevent_seqnum", "r");
	if (!fp) {
		dbg("can't open '/sys/kernel/uevent_seqnum', error(%d):%s\n",
		    errno, strerror(errno));
		return -errno;
	}

	/* read seqnum */
	if (!fgets(seqnum, sizeof(seqnum), fp)) {
		dbg("unable to read '/sys/kernel/uevent_seqnum', "
		    "error(%d):%s\n", errno, strerror(errno));
		fclose(fp);
		return -errno;
	}
	fclose(fp);

	/* format udevd seqnum file name */
	ret = asprintf(&file_name, "%s/.udevd_cold_seqnum", udev_root);
	if (ret < 0) {
		dbg("%s", "can't allocate file path\n");
		return -ENOMEM;
	}

	/* create udevd seqnum file name */
	fp = fopen(file_name, "w");
	if (!fp) {
		dbg("can't create '%s', error(%d):%s\n", file_name, errno,
		    strerror(errno));
		free(file_name);
		return -errno;
	}

	/* export seqnum property*/
	fputs(seqnum, fp);

	/* update file permissions */
	chmod(file_name, S_IRWXU|S_IRGRP|S_IROTH);

	/* cleanup */
	fclose(fp);
	free(file_name);
	return 0;
}

#define SIZEOF_ARRAY(x) (sizeof(x)/sizeof(x[0]))

static int version(struct udev *udev, int argc, char *argv[])
{
	printf("%s\n", VERSION);
	return 0;
}

static int help(struct udev *udev, int argc, char *argv[])
{
	printf("Usage: udevd_init [--help] [--version] [--debug] "
	       "[udevadm trigger options]\n");
	printf("\n");
	return 0;
}

static char *trigger_argv[] = {
	"trigger",
	"--action=add",
	"--subsystem-match=hidraw",
	"--subsystem-match=mmc",
	"--subsystem-match=usb",
	"--subsystem-match=tty",
	"--subsystem-match=net",
	"--subsystem-match=block",
	"--subsystem-match=sound",
	"--subsystem-match=firmware",
};

static char *settle_argv[] = {
	"settle"
};

int main(int argc, char *argv[])
{
	struct udev *udev = NULL;
	struct udev_ctrl *uctrl = NULL;
	const char *udev_root = NULL;
	int ret, rc, i, count;
	int *pids;
	int priority = LOG_ERR;
	static const struct option options[] = {
		{ "debug", no_argument, NULL, 'd' },
		{ "help", no_argument, NULL, 'h' },
		{ "version", no_argument, NULL, 'V' },
		{ NULL, 0, NULL, 0}
	};


	dbg("%s started\n", argv[0]);

	for (;;) {
		int option;
		option = getopt_long(argc, argv, "+dhV", options, NULL);
		if (option == -1)
			break;

		switch (option) {
		case 'd':
			priority = LOG_DEBUG;
			break;
		case 'h':
			rc = help(udev, argc, argv);
			return EXIT_SUCCESS;
		case 'V':
			rc = version(udev, argc, argv);
			return EXIT_SUCCESS;
		default:
			return EXIT_FAILURE;
		}
	}

	/* next arguments are for udevadm trigger */
	argc -= optind;
	argv += optind;
	optind = 0;

	/* make sure no udevd instance is running */

#ifdef ANDROID
	/* stop udevd service (if already started)*/
	property_set("ctl.stop", "udevd");
#endif

	/* list all udevd process running */
	count = get_pids_by_name("udevd", &pids);
	if (count < 0)
		goto error;

	/* kill all udevd */
	if (count > 0)
		dbg("%d udevd process running, shoot them up ...\n", count);

	for (i = 0; i < count; i++)
		kill(pids[i], SIGTERM);

	free(pids);

	udev = udev_new();
	if (!udev)
		goto error;

	udev_log_init("udev-init");
	udev_set_log_fn(udev, &log_udev);

	/* make sure we have a tmpfs mounted on /tmp */
	if (!is_fs_mounted("tmpfs", "/tmp")) {
		dbg("%s", "no tmpfs fs mounted on /tmp\n");
		goto error;
	}

	/* get udev root directory */
	udev_root = udev_get_dev_path(udev);
	if (!udev_root) {
		dbg("%s", "can't find udev root directory\n");
		goto error;
	}

	/* create udev root directory */
	if (create_dir(UDEV_DIR_PERMS, "%s", udev_root) < 0)
		goto error;

	/* create udev_root/.udev/queue directory */
	if (create_dir(UDEV_DIR_PERMS, "%s/.udev/queue", udev_root) < 0)
		goto error;

	/* create udev_root/.udev/rules.d directory */
	if (create_dir(UDEV_DIR_PERMS, "%s/.udev/rules.d", udev_root) < 0)
		goto error;

	/* export udev board */
	if (export_udev_board(udev_root) < 0)
		goto error;

#ifdef ANDROID
	/* tell init process to spawn a new instance of udevd */
	dbg("%s", "starting udevd service\n");
	/* start udevd */
	property_set("ctl.start", "udevd");
#else
	fprintf(stderr, "echo starting udevd daemon\n");
	/* spawn by ourselves udevd*/
	ret = system("udevd --daemon");
	if (ret < 0) {
		dbg("%s", "failed to launch udev deamon\n");
		goto error;
	}
#endif
	/* open udev control socket */
	uctrl = udev_ctrl_new_from_socket(udev, UDEV_CTRL_SOCK_PATH);
	if (!uctrl)
		goto error;

	/* set udev priority log error
	 * this is a poor synchronization mechanism to ensure udevd init
	 * is completed */
	do {
		ret = udev_ctrl_send_set_log_level(uctrl, priority);
		if (ret < 0) {
			/* failure means udevd init not completed !*/
			int timeout = 50*1000;
			dbg("wait %d us\n", timeout);
			usleep(timeout);
		}
	} while (ret < 0);

	udev_ctrl_unref(uctrl);
	dbg("%s", "udevd start completed !\n");

	/* synthesize events for cold plugged device */
	optind = 0;
	if (argc > 0) {
		/* trigger with user filter args */
		dbg("%s", "udev custom trigger\n");
		udevadm_trigger(udev, argc, argv);
	} else {
		dbg("%s", "udev default trigger\n");
		/* trigger with default filter args*/
		udevadm_trigger(udev, SIZEOF_ARRAY(trigger_argv),
				trigger_argv);
	}

	/* wait for udevd */
	dbg("%s","udev settle\n");
	optind = 0;
	udevadm_settle(udev, SIZEOF_ARRAY(settle_argv), settle_argv);

	/* write cold seqnum stamp*/
	dbg("%s", "write cold seqnum\n");
	if (write_udev_seqnum(udev_root) < 0)
		goto error;

	dbg("%s", "udevd successfully started\n");
	udev_unref(udev);
	udev_log_close();
	return EXIT_SUCCESS;

error:
	dbg("%s", "udevd start failed !");
	if (udev)
		udev_unref(udev);
	udev_log_close();
	return EXIT_FAILURE;
}
