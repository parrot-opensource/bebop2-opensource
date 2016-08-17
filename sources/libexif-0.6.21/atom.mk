ifneq ($(TARGET_PRODUCT),sfx1) # libexif already exists in SFX1 project
ifneq ($(TARGET_PRODUCT),thermoMAP) # libexif already exists in thermoMAP project
ifneq ($(TARGET_PRODUCT),galilei) # libexif already exists in Galilei project

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libexif
LOCAL_DESCRIPTION := libexif
LOCAL_CATEGORY_PATH := external

LOCAL_AUTOTOOLS_VERSION := 0.6.21
LOCAL_AUTOTOOLS_ARCHIVE := libexif-$(LOCAL_AUTOTOOLS_VERSION).tar.gz
LOCAL_AUTOTOOLS_SUBDIR := libexif-$(LOCAL_AUTOTOOLS_VERSION)

ifeq ("$(TARGET_PBUILD_FORCE_STATIC)","1")
#LOCAL_AUTOTOOLS_CONFIGURE_ARGS += --disable-shared
$(warning LibEXIF cannot be compiled as a static library for licensing issues)
else
# Force building shared libraries because of the LGPL license
LOCAL_AUTOTOOLS_CONFIGURE_ARGS += --enable-shared=yes --enable-static=no 
define LOCAL_AUTOTOOLS_CMD_POST_BUILD
	$(Q) cd $(PRIVATE_SRC_DIR)/$(LOCAL_AUTOTOOLS_SUBDIR) && find . -name "*.a" -exec echo " --> removing {}" \; -exec rm {} \;
endef
endif

#LOCAL_AUTOTOOLS_CONFIGURE_ARGS := \

# User define command to be launch before configure step.
# Generates files used by configure
define LOCAL_AUTOTOOLS_CMD_POST_UNPACK
	$(Q) cd $(PRIVATE_SRC_DIR)/$(LOCAL_AUTOTOOLS_SUBDIR) && echo "===========>" && pwd 
endef

LOCAL_EXPORT_LDLIBS := -lexif

include $(BUILD_AUTOTOOLS)

endif
endif
endif
