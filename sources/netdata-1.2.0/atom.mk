LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := netdata
LOCAL_DESCRIPTION := Real-time performance monitoring daemon
LOCAL_CATEGORY_PATH := system

LOCAL_AUTOTOOLS_CONFIGURE_ARGS := \
	--localstatedir=/var

# util-linux-ng is needed only for libuuid
LOCAL_LIBRARIES := \
	util-linux-ng \
	zlib

LOCAL_COPY_FILES := \
	etc/boxinit.d/99-netdata.rc:etc/boxinit.d/ \
	etc/netdata/apps_groups.conf:etc/netdata/ \
	etc/netdata/netdata.conf:etc/netdata/

include $(BUILD_AUTOTOOLS)
