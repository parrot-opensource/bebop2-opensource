LOCAL_PATH := $(call my-dir)

###############################################################################
# devmem
###############################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := devmem
LOCAL_DESCRIPTION := Simple program to read/write from/to any location in memory.
LOCAL_CATEGORY_PATH := devel

LOCAL_SRC_FILES := \
	$(call all-c-files-under,.,.c)

include $(BUILD_EXECUTABLE)
