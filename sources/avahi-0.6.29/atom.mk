LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := avahi
LOCAL_DESCRIPTION := Avahi
LOCAL_CATEGORY_PATH := network

LOCAL_AUTOTOOLS_VERSION := 0.6.29
LOCAL_AUTOTOOLS_ARCHIVE := avahi-$(LOCAL_AUTOTOOLS_VERSION).tar.gz
LOCAL_AUTOTOOLS_SUBDIR := avahi-$(LOCAL_AUTOTOOLS_VERSION)

LOCAL_AUTOTOOLS_CONFIGURE_ARGS := \
	--localstatedir=/var \
	--disable-nls \
	--disable-glib \
	--disable-gobject \
	--disable-qt3 \
	--disable-qt4 \
	--disable-gtk \
	--disable-gtk3 \
	--disable-gdbm \
	--disable-python \
	--disable-python-dbus \
	--disable-pygtk \
	--disable-mono \
	--disable-monodoc \
	--disable-stack-protector \
	--with-distro=none \
	--with-avahi-user=avahi \
	--with-avahi-group=avahi \
	--enable-libdaemon \
	--enable-shared=yes \
	--enable-static=no \
	--disable-dbus  ## DBus is potentially covered by GPL, which is forbidden

LOCAL_LIBRARIES := libdaemon expat libnss-mdns # dbus

#LOCAL_EXPORT_LDLIBS := -lavahi-client -lavahi-common
LOCAL_AUTOTOOLS_PATCHES := \
			iface-linux.patch \
			so_reuseport.patch \
			avahi_daemon_services.patch

include $(BUILD_AUTOTOOLS)
