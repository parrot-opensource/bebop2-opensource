LOCAL_PATH := $(call my-dir)

# Not with bionic
ifneq ("$(TARGET_LIBC)","bionic")

###############################################################################
# lib media-ctl
###############################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := libmedia-ctl

LOCAL_SRC_FILES := \
  src/v4l2subdev.c \
  src/mediactl.c

LOCAL_CATEGORY_PATH := video/media-ctl

LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/src

include $(BUILD_SHARED_LIBRARY)


###############################################################################
# media-ctl
###############################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := media-ctl
LOCAL_DESCRIPTION := Media controller control application
LOCAL_CATEGORY_PATH := video

LOCAL_SRC_FILES := \
	$(call all-c-files-under,src,.c)

include $(BUILD_EXECUTABLE)

endif
