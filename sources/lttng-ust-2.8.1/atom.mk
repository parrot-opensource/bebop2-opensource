
LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE := lttng-ust
LOCAL_DESCRIPTION := Userspace tracing library.
LOCAL_CATEGORY_PATH := devel/lttng

LOCAL_AUTOTOOLS_VERSION := 2.8.1
LOCAL_AUTOTOOLS_ARCHIVE := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION).tar.bz2
LOCAL_AUTOTOOLS_SUBDIR := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION)
LOCAL_AUTOTOOLS_PATCHES := configure_remove_tests_and_doc.patch
LOCAL_LIBRARIES := util-linux-ng userspace-rcu

LOCAL_AUTOTOOLS_CONFIGURE_ARGS := \
	--disable-extras \
	--disable-man-pages

LTTNG_UST_BUILD_DIR := $(call local-get-build-dir)
LTTNG_UST_SRC_DIR := $(LTTNG_UST_BUILD_DIR)/$(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION)

LOCAL_EXPORT_LDLIBS := -llttng-ust -ldl

# liblttng-ust-ctl/ustctl.c is GPL v2
define LOCAL_AUTOTOOLS_CMD_POST_UNPACK
	$(Q) cp -af $(PRIVATE_PATH)/.MODULE_LICENSE_GPL.ustctl.c \
		$(PRIVATE_SRC_DIR)/liblttng-ust-ctl/
endef

include $(BUILD_AUTOTOOLS)
