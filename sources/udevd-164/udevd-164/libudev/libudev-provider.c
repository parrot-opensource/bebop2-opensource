/*
 * libudev - interface to udev device information
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <ctype.h>
#include <dirent.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <sys/param.h>

#include "libudev.h"
#include "libudev-private.h"

#define UDEV_PROVIDER_SERVER_SOCK_PATH	"@/org/kernel/udev/juba-provider-server"
#define UDEV_PROVIDER_DEVICE_SOCK_PATH  "@/org/kernel/udev/juba-provider-device"
#define UDEV_PROVIDER_SEQNUM_BASE       (10000000000000000000ULL)

/**
 * SECTION:libudev-provider
 * @short_description: external device event source
 *
 * Allows to asynchronously send device events to udev from an external process.
 */

/**
 * udev_provider:
 *
 * Opaque object handling one event provider.
 */
struct udev_provider {
	struct udev *udev;
	int refcount;
	/* socket descriptor for client permanent connection to server */
	int fd;
	/* this uniquely identifies this provider */
	char *key;
	/* monitor for sending devices from a client process */
	struct udev_monitor *monitor;
};

struct udev_provider_server {
	struct udev *udev;
	int refcount;
	int fd;
	/*
	 * Provider seqnum values start from a large offset to distinguish them
	 * from kernel ones.
	 */
	unsigned long long seqnum;
	/* monitor for receiving devices from client processes */
	struct udev_monitor *monitor;
	/* array of poll descriptors, polling is done in udevd.c:main() */
	struct pollfd *pfd;
	struct udev_provider *providers[UDEV_MAX_PROVIDERS];
	struct udev_list_node devices;
};

static struct udev_provider *_udev_provider_new(struct udev *udev)
{
	struct udev_provider *udev_provider;

	udev_provider = calloc(1, sizeof(struct udev_provider));
	if (udev_provider == NULL)
		return NULL;
	udev_provider->refcount = 1;
	udev_provider->udev = udev;
	return udev_provider;
}

static int udev_provider_connect(struct udev_provider *udev_provider)
{
	int fd = -1, ret = -1;
	char buf[64];
	int count;
	socklen_t addrlen;
	struct sockaddr_un sun;
	struct udev *udev = udev_provider->udev;

	udev_provider->fd = -1;
	udev_provider->key = NULL;

	sun.sun_family = AF_LOCAL;
	/* translate leading '@' to abstract namespace */
	util_strscpy(sun.sun_path, sizeof(sun.sun_path),
		     UDEV_PROVIDER_SERVER_SOCK_PATH);
	sun.sun_path[0] = '\0';
	addrlen = offsetof(struct sockaddr_un, sun_path) +
		strlen(UDEV_PROVIDER_SERVER_SOCK_PATH);

	fd = socket(AF_LOCAL, SOCK_STREAM|SOCK_CLOEXEC, 0);
	if (fd == -1) {
		ret = -errno;
		err(udev, "error getting socket: %m\n");
		goto fail;
	}

	/* connect to udevd */
	if (connect(fd, (const struct sockaddr *)&sun, addrlen) < 0) {
		ret = -errno;
		goto fail;
	}
	/* get identification key */
	count = recv(fd, buf, sizeof(buf), 0);
	if (count == 0) {
		err(udev, "connection to provider server rejected\n");
		goto fail;
	}
	if (count < 0) {
		ret = -errno;
		err(udev, "error receiving provider key: %s\n",strerror(errno));
		goto fail;
	}
	buf[sizeof(buf)-1] = '\0';

	udev_provider->fd = fd;
	udev_provider->key = strdup(buf);

	dbg(udev, "provider %p connected with '%s'\n", udev_provider,
	    udev_provider->key);
	return 0;
fail:
	if (fd >= 0)
		close(fd);
	return ret;
}

static int udev_provider_reconnect(struct udev_provider *udev_provider)
{
	struct pollfd pfd;

	if (udev_provider->fd >= 0) {
		/* check if connection is still up */
		pfd.fd = udev_provider->fd;
		pfd.events = POLLHUP;
		if (poll(&pfd, 1, 0) == 0)
			/* OK, still connected */
			return 0;
		/* assume we were disconnected */
		dbg(udev_provider->udev, "provider %p disconnected\n",
		    udev_provider);
		close(udev_provider->fd);
		free(udev_provider->key);
	}
	/* attempt to reconnect */
	return udev_provider_connect(udev_provider);
}

/**
 * udev_provider_new:
 * @udev: udev library context
 *
 * Create a new device provider, and connect to provider server.
 * You should have proper credentials for this call to succeed: either uid 0,
 * or the same group membership as udevd.
 *
 * Once the provider has been successfully created, you can immediately
 * start sending 'add', 'remove' or 'change' device events.
 *
 * Before releasing your provider, you should send 'remove' events for all
 * your currently attached virtual devices. If you fail to do so, or if your
 * process crashes, udevd will automatically generate the remove events.
 *
 * The initial refcount is 1, and needs to be decremented to
 * release the resources of the udev provider.
 *
 * Returns: a new udev provider, or #NULL, in case of an error
 **/
struct udev_provider *udev_provider_new(struct udev *udev)
{
	struct udev_provider *udev_provider = NULL;
	struct udev_monitor *monitor = NULL;

	udev_provider = _udev_provider_new(udev);
	if (udev_provider == NULL)
		goto fail;

	monitor = udev_monitor_new_from_socket(udev,
					       UDEV_PROVIDER_DEVICE_SOCK_PATH);
	if (monitor == NULL) {
		err(udev, "cannot create provider client monitor\n");
		goto fail;
	}
	udev_provider->monitor = monitor;

	/* connect to udevd (connect failures will be detected later) */
	(void)udev_provider_connect(udev_provider);

	dbg(udev, "provider %p created\n", udev_provider);
	return udev_provider;
fail:
	udev_monitor_unref(monitor);
	free(udev_provider);
	return NULL;
}

static int is_string_valid(const char *s)
{
	while (*s) {
		if (!isalnum(*s) && !strchr("_-:.", *s))
			return 0;
		s++;
	}
	return 1;
}

static struct udev_device *
udev_device_new_from_provider(struct udev *udev, const char *key,
			      const char *sysname, const char **properties)
{
	int i;
	struct udev_device *dev;
	char buf[256];
	const char *p;

	/* sanity checks on input strings */
	if (!is_string_valid(sysname)) {
		err(udev, "invalid sysname: %s\n", sysname);
		return NULL;
	}

	dev = udev_device_new(udev);
	if (dev == NULL)
		return NULL;
	udev_device_set_info_loaded(dev);

	util_strscpyl(buf, sizeof(buf), "DEVPATH=/juba-", key, "/juba-",
		      sysname, NULL);
	udev_device_add_property_from_string_parse(dev, buf);
	util_strscpyl(buf, sizeof(buf), "SUBSYSTEM=juba-", key, NULL);
	udev_device_add_property_from_string_parse(dev, buf);
	udev_device_add_property_from_string_parse(dev, "DEVTYPE=juba");
	udev_device_add_property_from_string_parse(dev, "TAGS=:JUBA:");

	for (i = 0; properties[i] != NULL; i++) {
		if (strncmp("JUBA_", properties[i], 5) != 0) {
			err(udev, "invalid property prefix (not JUBA_) '%s'\n",
			    properties[i]);
			udev_device_unref(dev);
			return NULL;
		}
		p = strchr(properties[i], '=');
		if (p == NULL) {
			err(udev, "invalid property '%s'\n", properties[i]);
			udev_device_unref(dev);
			return NULL;
		}
		if (p[1] == '\0') {
			info(udev, "skipping no-value property '%s'\n",
			     properties[i]);
			continue;
		}
		udev_device_add_property_from_string_parse(dev, properties[i]);
	}
	if (udev_device_add_property_from_string_parse_finish(dev) < 0) {
		err(udev, "missing values, invalid device\n");
		udev_device_unref(dev);
		dev = NULL;
	}

	return dev;
}

static int udev_provider_send_device(struct udev_provider *udev_provider,
				     const char *sysname,
				     const char **properties,
				     const char *action)
{
	int ret;
	struct udev_device *dev;

	if (udev_provider == NULL)
		return -EINVAL;

	/* check that udevd is still up */
	ret = udev_provider_reconnect(udev_provider);
	if (ret < 0)
		return ret;

	dev = udev_device_new_from_provider(udev_provider->udev,
					    udev_provider->key,
					    sysname, properties);
	if (dev == NULL)
		return -EINVAL;

	udev_device_set_action(dev, action);
	ret = udev_monitor_send_device(udev_provider->monitor, NULL, dev);
	udev_device_unref(dev);

	return (ret < 0) ? ret : 0;
}

/**
 * udev_provider_add_device:
 * @udev_provider: udev provider
 * @sysname: unique device name
 * @properties: #NULL terminated list of 'key=value' property strings
 *
 * Add a virtual device to the system. This will generate a fake 'add' uevent
 * in the udev daemon, with the following properties:
 * - DEVPATH=/juba-&lt;pid&gt;-&lt;id&gt;/juba-@sysname
 * - SUBSYSTEM=juba-&lt;pid&gt;-&lt;id&gt;
 * - DEVTYPE=juba
 * - @properties (your properties)
 *
 * Where &lt;pid&gt; is the PID of the client process, &lt;id&gt; identifies
 * the client connection.
 *
 * @sysname must uniquely identify your device in the system, it can contain
 * only the following characters: "a-zA-Z0-9.-_:". Example: "ipod0".
 * Note that @sysname will be prepended with the string "juba-" to avoid any
 * clash with kernel namespace.
 *
 * @properties is a list of "key=values" properties. Each 'key' name should
 * start with "JUBA_", otherwise the device will be rejected. Again this is to
 * protect against kernel namespace collisions (including wrong matches in
 * rules, etc).
 *
 * The virtual device event will be processed as usual, with rules processing,
 * tagging, broadcasting to libudev listeners, db storage, etc. The device
 * will be included in enumeration scans like a regular device. No device node
 * will be created, and it will not support sysfs attributes or uevent triggers.
 *
 * Returns: 0 upon success, a negative errno value is case of error
 **/
int udev_provider_add_device(struct udev_provider *udev_provider,
			     const char *sysname,
			     const char **properties)
{
	return udev_provider_send_device(udev_provider, sysname, properties,
					 "add");
}

/**
 * udev_provider_change_device:
 * @udev_provider: udev provider
 * @sysname: unique device name
 * @properties: #NULL terminated list of 'key=value' property strings
 *
 * Change the properties of a virtual device. This will generate a fake 'change'
 * uevent in the udev daemon. @sysname should match a device previously added
 * with udev_provider_add_device(). No verification is done that this is the
 * case.
 * Note that you should provide the entire set of properties, not just the ones
 * that need an update, as all properties are wiped out during a 'change' event.
 *
 * Returns: 0 upon success, a negative errno value is case of error
 **/
int udev_provider_change_device(struct udev_provider *udev_provider,
				const char *sysname,
				const char **properties)
{
	return udev_provider_send_device(udev_provider, sysname, properties,
					 "change");
}

/**
 * udev_provider_remove_device:
 * @udev_provider: udev provider
 * @sysname: unique device name
 *
 * Remove a virtual device from the system. This will generate a fake 'remove'
 * uevent in the udev daemon. @sysname should match a device previously
 * added with udev_provider_add_device(). No verification is done that this is
 * the case.
 *
 * Returns: 0 upon success, a negative errno value is case of error
 **/
int udev_provider_remove_device(struct udev_provider *udev_provider,
				const char *sysname)
{
	const char *properties[] = {NULL};
	return udev_provider_send_device(udev_provider, sysname, properties,
					 "remove");
}

/* iterate through all provider devices in udev db */
void udev_provider_enumerate(struct udev *udev,
			     void (*pfn)(void *cookie, struct udev_device *dev),
			     void *cookie)
{
	DIR *dir;
	struct dirent *dent;
	struct udev_device *dev;
	const char *prop[] = {NULL};
	char path[UTIL_PATH_SIZE];

	util_strscpyl(path, sizeof(path), udev_get_dev_path(udev), "/.udev/db",
		      NULL);

	dir = opendir(path);
	if (dir == NULL)
		return;

	for (dent = readdir(dir); dent != NULL; dent = readdir(dir)) {
		char buf[UTIL_PATH_SIZE];
		char *s;

		if (strncmp(dent->d_name, "juba-", 5) != 0)
			continue;

		util_strscpyl(buf, sizeof(buf), dent->d_name, NULL);
		s = strchr(buf, ':');
		if ((s == NULL) || strncmp(s+1, "juba-", 5))
			continue;
		*s++ = '\0';

		dev = udev_device_new_from_provider(udev, &buf[5], &s[5], prop);
		if (dev != NULL)
			(*pfn)(cookie, dev);
	}
	closedir(dir);
}

int udev_provider_device_is_virtual(struct udev_device *dev)
{
	if (dev == NULL)
		return 0;
	return (udev_device_get_seqnum(dev) >= UDEV_PROVIDER_SEQNUM_BASE);
}

/**
 * udev_provider_ref:
 * @udev_provider: udev provider
 *
 * Take a reference of a udev provider.
 *
 * Returns: the passed udev provider
 **/
struct udev_provider *udev_provider_ref(struct udev_provider *udev_provider)
{
	if (udev_provider == NULL)
		return NULL;
	udev_provider->refcount++;
	return udev_provider;
}

/**
 * udev_provider_unref:
 * @udev_provider: udev provider
 *
 * Drop a reference of a udev provider. If the refcount reaches zero,
 * the bound sockets will be closed, and the resources of the provider
 * will be released.
 *
 **/
void udev_provider_unref(struct udev_provider *udev_provider)
{
	if (udev_provider == NULL)
		return;
	udev_provider->refcount--;
	if (udev_provider->refcount > 0)
		return;
	if (udev_provider->fd >= 0)
		close(udev_provider->fd);
	udev_monitor_unref(udev_provider->monitor);

	dbg(udev_provider->udev, "provider %p released\n", udev_provider);

	free(udev_provider->key);
	free(udev_provider);
}

static void delete_stale_db_entry(void *cookie, const char *syspath)
{
	struct udev *udev = (struct udev *)cookie;
	char *s, buf[UTIL_PATH_SIZE];

	util_strscpyl(buf, sizeof(buf),
		      &syspath[strlen(udev_get_sys_path(udev))+1], NULL);
	s = strrchr(buf, '/');
	if (s == NULL)
		return;
	*s = ':';

	info(udev, "deleting stale provider db entry '%s'\n", buf);
	(void)unlink(buf);
}

/*
 * Allocate new provider server, bind server socket and enable receiving on
 * device monitor.
 */
struct udev_provider_server *udev_provider_server_new(struct udev *udev,
						      struct pollfd *pfd)
{
	int err, i, fd = -1;
	socklen_t addrlen;
	struct sockaddr_un sun;
	struct udev_provider_server *server;
	struct udev_monitor *monitor = NULL;

	server = calloc(1, sizeof(struct udev_provider_server));
	if (server == NULL)
		goto fail;

	server->refcount = 1;
	server->udev = udev;
	server->seqnum = UDEV_PROVIDER_SEQNUM_BASE;
	server->pfd = pfd;

	sun.sun_family = AF_LOCAL;
	/* translate leading '@' to abstract namespace */
	util_strscpy(sun.sun_path, sizeof(sun.sun_path),
		     UDEV_PROVIDER_SERVER_SOCK_PATH);
	sun.sun_path[0] = '\0';
	addrlen = offsetof(struct sockaddr_un, sun_path) +
		strlen(UDEV_PROVIDER_SERVER_SOCK_PATH);

	fd = socket(AF_LOCAL, SOCK_STREAM|SOCK_CLOEXEC, 0);
	if (fd == -1) {
		err(udev, "error getting socket: %m\n");
		goto fail;
	}

	err = bind(fd, (struct sockaddr *)&sun, addrlen);
	if (err < 0) {
		err(udev, "provider server bind: %s\n", strerror(errno));
		goto fail;
	}

	err = listen(fd, UDEV_MAX_PROVIDERS);
	if (err < 0) {
		err(udev, "provider server listen: %s\n", strerror(errno));
		goto fail;
	}
	server->fd = fd;

	monitor = udev_monitor_new_from_socket(udev,
					       UDEV_PROVIDER_DEVICE_SOCK_PATH);
	if (monitor == NULL) {
		err(udev, "error initializing provider monitor socket");
		goto fail;
	}
	if (udev_monitor_enable_receiving(monitor) < 0) {
		err(udev, "error binding provider monitor socket\n");
		goto fail;
	}
	/* 1 MB for buffering virtual devices, udevd uses 128 MB */
	udev_monitor_set_receive_buffer_size(monitor, 1*1024*1024);
	server->monitor = monitor;

	for (i = 0; i < UDEV_MAX_PROVIDERS; i++) {
		server->pfd[i].fd = -1;
		server->pfd[i].events = POLLHUP;
	}
	udev_list_init(&server->devices);

	dbg(udev, "provider server %p created\n", server);
	return server;
fail:
	if (fd >= 0)
		close(fd);

	udev_monitor_unref(monitor);
	free(server);
	return NULL;
}

void udev_provider_server_unref(struct udev_provider_server *server)
{
	int i;

	if (server == NULL)
		return;
	server->refcount--;
	if (server->refcount > 0)
		return;

	if (server->fd >= 0)
		close(server->fd);

	udev_monitor_unref(server->monitor);

	for (i = 0; i < UDEV_MAX_PROVIDERS; i++)
		if (server->pfd[i].fd >= 0)
			udev_provider_unref(server->providers[i]);

	udev_list_cleanup_entries(server->udev, &server->devices);

	dbg(server->udev, "provider server %p released\n", server);
	free(server);
}

/* Accept client connection and create a new provider */
void udev_provider_server_accept(struct udev_provider_server *server)
{
	int i, id, fd;
	struct ucred cred;
	socklen_t clen = sizeof(cred);
	char buf[64];
	struct udev *udev = server->udev;
	struct udev_provider *udev_provider = NULL;

	/* this connection is only used to track peer state */
	fd = accept(server->fd, NULL, NULL);
	if (fd < 0) {
		err(udev, "provider accept: %s\n", strerror(errno));
		goto fail;
	}

	/* find a slot for a new provider */
	id = -1;
	for (i = 0; i < UDEV_MAX_PROVIDERS; i++)
		if (server->pfd[i].fd < 0) {
			id = i;
			break;
		}

	if (id < 0) {
		err(udev, "too many providers connected\n");
		goto fail;
	}

	if (getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &cred, &clen) < 0) {
		err(udev, "provider getsockopt: %s\n", strerror(errno));
		goto fail;
	}

	if ((cred.uid != 0) && (cred.gid != getegid())) {
		err(udev, "provider lacks credentials\n");
		goto fail;
	}

	udev_provider = _udev_provider_new(udev);
	if (udev_provider == NULL)
		goto fail;

	/* create identification key and send it to client */
	snprintf(buf, sizeof(buf), "provider-%d-%d", cred.pid, id);

	if (send(fd, buf, strlen(buf)+1, MSG_NOSIGNAL) < 0) {
		err(udev, "send to provider client: %s\n", strerror(errno));
		goto fail;
	}

	udev_provider->fd = fd;
	udev_provider->key = strdup(buf);

	dbg(udev, "provider %p created with '%s'\n", udev_provider,
	    udev_provider->key);

	server->pfd[id].fd = udev_provider->fd;
	server->providers[id] = udev_provider;
	return;

fail:
	if (fd >= 0)
		close(fd);
	free(udev_provider);
}

struct device_removal_info {
	struct udev_provider_server *server;
	void (*insert_event)(struct udev_device *dev);
};

static void force_provider_device_removal(void *cookie, struct udev_device *dev)
{
	struct device_removal_info *info = (struct device_removal_info *)cookie;
	udev_device_set_seqnum(dev, ++info->server->seqnum);
	udev_device_set_action(dev, "remove");
	info->insert_event(dev);
}

/* Remove provider devices in udev db */
void udev_provider_server_remove_stale(struct udev_provider_server *server,
				       void (*pfn)(struct udev_device *d))
{
	struct device_removal_info info = {
		.server = server,
		.insert_event = pfn,
	};
	struct udev *udev = server->udev;

	/* insert remove events for all provider devices */
	udev_provider_enumerate(udev, force_provider_device_removal, &info);
}

/* Remove devices belonging to a provider and release it */
void udev_provider_server_handle_disconnect(struct udev_provider_server *server,
					    int id,
					    void (*pfn)(struct udev_device *d))
{
	int pid, cid;
	char key[64];
	struct udev *udev = server->udev;
	const char *prop[] = {NULL};
	struct udev_list_entry *entry_loop;
	struct udev_list_entry *entry_tmp;
	const char *name, *devpath;
	struct udev_device *dev;

	/* iterate through all attached devices and generate remove events */
	udev_list_entry_foreach_safe(entry_loop, entry_tmp,
				     udev_list_get_entry(&server->devices)) {

		devpath = udev_list_entry_get_name(entry_loop);

		/* parse pid and connection id */
		if (sscanf(devpath, "/juba-provider-%d-%d/", &pid, &cid) != 2)
			continue;

		/* filter by id */
		if (cid != id)
			continue;

		/* identification key */
		snprintf(key, sizeof(key), "provider-%d-%d", pid, cid);

		/* name */
		name = strrchr(devpath, '/');
		if (strncmp(name, "/juba-", 6) != 0)
			continue;
		name += 6;

		dev = udev_device_new_from_provider(udev, key, name, prop);
		if (dev) {
			udev_device_set_seqnum(dev, ++server->seqnum);
			udev_device_set_action(dev, "remove");
			/* event injector */
			(*pfn)(dev);
		}
		udev_list_entry_delete(entry_loop);
	}

	udev_provider_unref(server->providers[id]);
	server->pfd[id].fd = -1;
}

struct udev_device *
udev_provider_server_receive_device(struct udev_provider_server *server)
{
	int i, match;
	char buf[256];
	const char *devpath, *action;
	struct udev_device *dev;
	struct udev_list_entry *entry;
	struct udev *udev = server->udev;

	dev = udev_monitor_receive_device(server->monitor);
	if (dev == NULL)
		return NULL;

	/* check if we have a provider matching this device */
	devpath = udev_device_get_devpath(dev);

	match = 0;
	for (i = 0; i < UDEV_MAX_PROVIDERS; i++)
		if (server->pfd[i].fd >= 0) {
			snprintf(buf, sizeof(buf), "/juba-%s/",
				 server->providers[i]->key);
			if (strncmp(devpath, buf, strlen(buf)) == 0)
				match = 1;
		}

	if (!match) {
		dbg(udev, "rejecting device '%s' from unknown provider\n",
		    devpath);
		udev_device_unref(dev);
		return NULL;
	}

	udev_device_set_seqnum(dev, ++server->seqnum);

	/* maintain a list of currently attached devices */
	action = udev_device_get_action(dev);
	if (action) {
		if (strcmp(action, "add") == 0) {
			udev_list_entry_add(udev, &server->devices,
					    devpath, NULL, 1, 0);
		} else if (strcmp(action, "remove") == 0) {
			entry = udev_list_get_entry(&server->devices);
			entry = udev_list_entry_get_by_name(entry, devpath);
			if (entry)
				udev_list_entry_delete(entry);
		}
	}
	return dev;
}

/* Return server socket descriptor */
int udev_provider_server_get_fd(struct udev_provider_server *server)
{
	return server->fd;
}

/* Return device socket descriptor */
int udev_provider_server_get_device_fd(struct udev_provider_server *server)
{
	return udev_monitor_get_fd(server->monitor);
}
