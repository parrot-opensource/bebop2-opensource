LOCAL_PATH := $(call my-dir)

#=============================================================================
# android libmedia-ctl for media-ctl, libvideosubdev and libcamerapipeline
#-----------------------------------------------------------------------------

include $(CLEAR_VARS)

# LOCAL_PRELINK_MODULE set to false for compilation on Android 2.3
LOCAL_PRELINK_MODULE := false

LOCAL_MODULE := libmedia-ctl

LOCAL_MODULE_TAGS := optional

LOCAL_SRC_FILES := \
  src/v4l2subdev.c \
  src/mediactl.c

LOCAL_C_INCLUDES := \
  $(LOCAL_PATH)/src

include $(BUILD_SHARED_LIBRARY)


#=============================================================================
# android media-ctl
#-----------------------------------------------------------------------------

include $(CLEAR_VARS)

LOCAL_MODULE := media-ctl

LOCAL_MODULE_TAGS := tests

LOCAL_SRC_FILES := \
  src/v4l2subdev.c \
  src/mediactl.c \
  src/options.c \
  src/main.c

LOCAL_C_INCLUDES := \
  $(LOCAL_PATH)/src

include $(BUILD_EXECUTABLE)
