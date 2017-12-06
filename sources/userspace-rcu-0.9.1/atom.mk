
LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE := userspace-rcu
LOCAL_DESCRIPTION := The userspace RCU library.
LOCAL_CATEGORY_PATH := libs

LOCAL_AUTOTOOLS_VERSION := 0.9.1
LOCAL_AUTOTOOLS_ARCHIVE := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION).tar.bz2
LOCAL_AUTOTOOLS_SUBDIR := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION)

LOCAL_AUTOTOOLS_CONFIGURE_ENV := ac_cv_func_malloc_0_nonnull=yes
LOCAL_AUTOTOOLS_PATCHES := configure-remove-tests-and-doc.patch

LOCAL_EXPORT_LDLIBS := \
	-lurcu			\
	-lurcu-bp		\
	-lurcu-cds		\
	-lurcu-common	\
	-lurcu-mb		\
	-lurcu-qsbr		\
	-lurcu-signal

include $(BUILD_AUTOTOOLS)
