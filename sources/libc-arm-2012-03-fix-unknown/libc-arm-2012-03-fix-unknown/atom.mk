###############################################################################
## libc-arm-2012-03-fix atom.mk for arm-2012.03 toolchain fix
###############################################################################
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libc-arm-2012-03-fix
LOCAL_DESTDIR := lib
LOCAL_DESCRIPTION := arm-2012.03 eglibc fixed shared library
LOCAL_CATEGORY_PATH := alchemy/toolchains/arm-2012.03
LOCAL_SRC_FILES := strstr.c

# force package selection
CONFIG_ALCHEMY_BUILD_$(call get-define,$(LOCAL_MODULE)):=y

# copy ld.so.preload containing libstrstr.so path
LOCAL_COPY_FILES := ld.so.preload:etc/ld.so.preload

include $(BUILD_LIBRARY)

ifdef TARGET_TEST

# test libstrstr.so
include $(CLEAR_VARS)

LOCAL_MODULE := tst_libc-arm-2012-03-fix
LOCAL_DESCRIPTION := arm-2012.03 eglibc fixed shared library test
LOCAL_CATEGORY_PATH := alchemy/toolchains/arm-2012.03/tests
LOCAL_CFLAGS := -fno-builtin-strstr
LOCAL_SRC_FILES := test.c

include $(BUILD_EXECUTABLE)

# test libstrstr.a
include $(CLEAR_VARS)

LOCAL_MODULE := tst_libc-arm-2012-03-fix-static
LOCAL_DESCRIPTION := arm-2012.03 eglibc fixed static library test
LOCAL_CATEGORY_PATH := alchemy/toolchains/arm-2012.03/tests
LOCAL_CFLAGS := -fno-builtin-strstr
LOCAL_LDFLAGS := -static
LOCAL_SRC_FILES := test.c

include $(BUILD_EXECUTABLE)

endif
