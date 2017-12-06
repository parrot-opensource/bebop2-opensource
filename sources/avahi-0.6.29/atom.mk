LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := avahi
LOCAL_DESCRIPTION := Avahi service discovery suite
LOCAL_CATEGORY_PATH := network

LOCAL_AUTOTOOLS_VERSION := 0.6.29
LOCAL_AUTOTOOLS_ARCHIVE := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION).tar.gz
LOCAL_AUTOTOOLS_SUBDIR := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION)

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
	--disable-dbus

LOCAL_LIBRARIES := libdaemon expat libnss-mdns

LOCAL_EXPORT_LDLIBS := -lavahi-common -lavahi-core

LOCAL_AUTOTOOLS_CONFIGURE_ENV := ac_cv_header_sys_capability_h=no

LOCAL_AUTOTOOLS_PATCHES := \
	so_reuseport.patch \
	iface-linux.patch \
	avahi_daemon_services.patch \
	netlink-unique-nl_pid.patch \
	filter-denied-virtual-interfaces-when-adding-address.patch

LOCAL_COPY_FILES := \
	90-avahi-daemon.rc:etc/boxinit.d/

include $(BUILD_AUTOTOOLS)

########################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := libavahi
LOCAL_DESCRIPTION := Avahi libraries (common, core)
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
	--disable-libdaemon \
	--disable-dbus \
	--with-xml=none

LOCAL_EXPORT_LDLIBS := -lavahi-common -lavahi-core

LOCAL_AUTOTOOLS_CONFIGURE_ENV := ac_cv_header_sys_capability_h=no

LOCAL_AUTOTOOLS_PATCHES := \
	so_reuseport.patch \
	iface-linux.patch \
	netlink-unique-nl_pid.patch

include $(BUILD_AUTOTOOLS)

