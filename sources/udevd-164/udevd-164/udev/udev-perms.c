/*
 * Copyright (C) 2014 Parrot S.A.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <unistd.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <stdlib.h>
#include <ctype.h>

#include "udev.h"

#define UDEV_PERMS_CONFIG_FILE       "/etc/udev_perms.conf"
#define UDEV_PERMS_HW_CONFIG_FILE    "/etc/udev_perms.%s.conf"

/*
  We always put our stamp in /dev, so that our .rc file that does the
  waiting does not depend on udev root directory
*/
#define UDEV_PERMS_STAMP             "/dev/.udev_perms_done"

struct perm_rule {
	char             *name; /* device path without /dev/ prefix */
	mode_t            mode;
	unsigned int      uid;
	unsigned int      gid;
	int               prefix;
	struct perm_rule *next;
};

static struct perm_rule *perms;
static const char *dev_path;
static int dev_path_len;

static void parse_line(struct udev *udev, char *line)
{
	const char * const delim = " \t";
	char *name, *perm, *user, *group, *saveptr, *endptr;
	mode_t mode;
	uid_t uid;
	gid_t gid;
	int len, prefix = 0;
	struct perm_rule *rule;

	/* eat whitespace */
	while ((line[0] == ' ') || (line[0] == '\t'))
		line++;

	/* skip comments and empty lines */
	if ((line[0] == '#') || (line[0] == '\n'))
		return;

	name  = strtok_r(line, delim, &saveptr);
	perm  = strtok_r(NULL, delim, &saveptr);
	user  = strtok_r(NULL, delim, &saveptr);
	group = strtok_r(NULL, delim, &saveptr);

	if (!name || !perm || !user || !group) {
		err(udev, "invalid line '%s' in perm rule\n", line);
		return;
	}
	/* squash trailing newline */
	saveptr = strchr(group, '\n');
	if (saveptr)
		*saveptr = '\0';

	if (strncmp(name, "/dev/", 5) != 0) {
		err(udev, "invalid device '%s' in perm rule\n", name);
		return;
	}
	name += 5;

	len = strlen(name);
	if (name[len-1] == '*') {
		prefix = 1;
		name[len-1] = '\0';
	}

	mode = strtoul(perm, &endptr, 8);
	if (!endptr || (*endptr != '\0')) {
		err(udev, "invalid mode '%s' in perm rule\n", perm);
		return;
	}

	uid = util_lookup_user(udev, user);
	gid = util_lookup_group(udev, group);

	rule = malloc(sizeof(*rule));
	if (!rule)
		return;

	rule->name = strdup(name);
	if (!rule->name) {
		free(rule);
		return;
	}

	rule->mode   = mode;
	rule->uid    = uid;
	rule->gid    = gid;
	rule->prefix = prefix;
	rule->next   = perms;

	/* insert in first position */
	perms = rule;

	dbg(udev, "added perm rule '%s%s' mode %#o uid %d (%s) gid %d (%s)\n",
	    rule->name, rule->prefix ? "*" : "", rule->mode,
	    rule->uid, user, rule->gid, group);
}

static int parse_file(struct udev *udev, const char *filename)
{
	FILE *fp;
	char buf[128];

	fp = fopen(filename, "r");
	if (fp == NULL)
		return -errno;

	while (fgets(buf, sizeof(buf), fp))
		parse_line(udev, buf);

	fclose(fp);
	return 0;
}

void udev_perms_load(struct udev *udev)
{
	int i, ret;
	const char *board;
	char hardware[32], tmp[64];

	/* get lowercase hardware name */
	board = util_get_board_name();
	for (i = 0; board[i] && (i < (int)sizeof(hardware)-1); i++)
		hardware[i] = tolower(board[i]);
	hardware[sizeof(hardware)-1] = '\0';

	/* generic permissions */
	ret = parse_file(udev, UDEV_PERMS_CONFIG_FILE);
	if (ret < 0) {
		info(udev, "could not load '%s': %s\n", UDEV_PERMS_CONFIG_FILE,
		     strerror(-ret));
	}

	/* hardware-specific overrides (à la Android) , optional */
	snprintf(tmp, sizeof(tmp), UDEV_PERMS_HW_CONFIG_FILE, hardware);
	(void)parse_file(udev, tmp);

	dev_path = udev_get_dev_path(udev);
	dev_path_len = strlen(dev_path);
}

void udev_perms_unload(struct udev *udev)
{
	struct perm_rule *rule, *r;

	rule = perms;

	while (rule) {
		r = rule;
		rule = r->next;
		free(r->name);
		free(r);
	}
	perms = NULL;
}

static struct perm_rule *udev_perms_get(struct udev *udev, const char *path)
{
	struct perm_rule *rule;
	char first;

	if (path == NULL)
		return NULL;

	first = path[0];

	/* hardware-specific rules are scanned before generic rules */
	for (rule = perms; rule; rule = rule->next) {
		/* skip as early as possible */
		if (rule->name[0] != first)
			continue;

		if (rule->prefix) {
			if (strncmp(path, rule->name, strlen(rule->name)))
				continue;
		} else {
			if (strcmp(path, rule->name))
				continue;
		}
		return rule;
	}

	return NULL;
}

void udev_perms_apply_to_event(struct udev_event *event)
{
	struct perm_rule *rule;
	const char *devname;

	devname = udev_device_get_knodename(event->dev);
	if (devname == NULL)
		devname = udev_device_get_sysname(event->dev);

	rule = udev_perms_get(event->udev, devname);
	if (rule) {
		info(event->udev, "fixing '%s' mode %#o uid %d gid %d\n",
		     devname, rule->mode, rule->uid, rule->gid);

		event->mode = rule->mode;
		event->uid  = rule->uid;
		event->gid  = rule->gid;
	}
}

static void udev_perms_fix_dir(struct udev *udev, const char *path)
{
	DIR *dir;
	char dbuf[256], buf[256];
	const char *dpath;
	struct dirent *dent;
	struct stat st;
	struct perm_rule *rule;

	if (path[0]) {
		snprintf(dbuf, sizeof(dbuf), "%s/%s", dev_path, path);
		dpath = dbuf;
	} else {
		dpath = dev_path;
	}

	dir = opendir(dpath);
	if (dir == NULL)
		return;

	for (dent = readdir(dir); dent != NULL; dent = readdir(dir)) {
		if (dent->d_name[0] == '.')
			continue;
		snprintf(buf, sizeof(buf), "%s/%s", dpath, dent->d_name);

		/* only process device nodes, skip all other types */
		if (lstat(buf, &st) ||
		    (!S_ISBLK(st.st_mode) && !S_ISCHR(st.st_mode)))
			continue;

		rule = udev_perms_get(udev, buf+dev_path_len+1);
		if (!rule)
			continue;

		if ((rule->mode != (st.st_mode & 0777)) ||
		    (rule->uid  != st.st_uid)  ||
		    (rule->gid  != st.st_gid)) {

			info(udev, "fixing '%s' mode %#o uid %d gid %d\n",
			     buf, rule->mode, rule->uid, rule->gid);

			(void)chown(buf, rule->uid, rule->gid);
			(void)chmod(buf, rule->mode);
		}
	}
	closedir(dir);
}

void udev_perms_fix_static_devices(struct udev *udev)
{
	int fd;
	char *p;
	struct perm_rule *rule;
	struct stat st;
	struct udev_list_node dirs;
	struct udev_list_entry *entry_loop = NULL;

	if (perms == NULL)
		/* no rules to apply */
		goto finish;

	/* skip this step on restart */
	if (stat(UDEV_PERMS_STAMP, &st) == 0)
		return;

	/* build a list a sub-directories to visit */
	udev_list_init(&dirs);
	udev_list_entry_add(udev, &dirs, "", NULL, 0, 0);

	for (rule = perms; rule; rule = rule->next) {
		p = strrchr(rule->name, '/');
		if (p == NULL)
			continue;
		*p = '\0';
		udev_list_entry_add(udev, &dirs, rule->name, NULL,
				    1, /* unique */
				    0  /* no sorting */);
		*p = '/';
	}

	/* fix permissions in each directory */
	udev_list_entry_foreach(entry_loop, udev_list_get_entry(&dirs)) {
		udev_perms_fix_dir(udev, udev_list_entry_get_name(entry_loop));
	}

	udev_list_cleanup_entries(udev, &dirs);

finish:
	fd = open(UDEV_PERMS_STAMP, O_WRONLY|O_CREAT, 0000);
	if (fd >= 0)
		close(fd);
}
