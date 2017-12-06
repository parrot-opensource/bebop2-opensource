LOCAL_PATH := $(call my-dir)

###############################################################################
# memtester
###############################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := memtester
LOCAL_DESCRIPTION := A userspace utility for testing the memory subsystem for \
	faults.
LOCAL_CATEGORY_PATH := devel

LOCAL_ARCHIVE_VERSION := 4.2.2
LOCAL_ARCHIVE := $(LOCAL_MODULE)-$(LOCAL_ARCHIVE_VERSION).tar.gz
LOCAL_ARCHIVE_SUBDIR := $(LOCAL_MODULE)-$(LOCAL_ARCHIVE_VERSION)

LOCAL_GENERATED_SRC_FILES := \
	$(LOCAL_ARCHIVE_SUBDIR)/memtester.c \
	$(LOCAL_ARCHIVE_SUBDIR)/tests.c

include $(BUILD_EXECUTABLE)
