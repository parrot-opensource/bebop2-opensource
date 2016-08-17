
LOCAL_PATH := $(call my-dir)

###############################################################################
# yavta
# Original upstream project :
#     http://git.ideasonboard.org/git/yavta.git
###############################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := yavta
LOCAL_DESCRIPTION := Yet another video application
LOCAL_CATEGORY_PATH := video

LOCAL_SRC_FILES := \
	yavta.c

include $(BUILD_EXECUTABLE)

