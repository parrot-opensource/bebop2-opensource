
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := procps
LOCAL_DESCRIPTION := CPU/ressource usage monitoring
LOCAL_CATEGORY_PATH := devel

LOCAL_LIBRARIES := ncurses
LOCAL_AUTOTOOLS_VERSION := 3.2.8
LOCAL_AUTOTOOLS_ARCHIVE := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION).tar.gz
LOCAL_AUTOTOOLS_SUBDIR := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION)

LOCAL_AUTOTOOLS_PATCHES := \
	procps-mips-define-pagesize.patch \
	procps-remove-index.patch \
	procps-fix-includes-order.patch \
	procps-fix-redefinition.patch

LOCAL_AUTOTOOLS_MAKE_BUILD_ENV := \
	$(AUTOTOOLS_CONFIGURE_ENV)

# Do not force 64-bit flags
LOCAL_AUTOTOOLS_MAKE_BUILD_ARGS := \
	$(AUTOTOOLS_CONFIGURE_ENV) \
	m64= vmstat top ps/ps

# Skip the configure step. The empty macro call is to make sure the step does
# nothing while shill being defined to something.
define LOCAL_AUTOTOOLS_CMD_CONFIGURE
	$(empty)
endef

define LOCAL_AUTOTOOLS_CMD_INSTALL
	$(Q) mkdir -p $(TARGET_OUT_STAGING)/usr/lib
	$(Q) mkdir -p $(TARGET_OUT_STAGING)/usr/bin
	$(Q) install -p $(PRIVATE_SRC_DIR)/proc/libproc-3.2.8.so $(TARGET_OUT_STAGING)/usr/lib
	$(Q) install -p $(PRIVATE_SRC_DIR)/vmstat $(TARGET_OUT_STAGING)/usr/bin/vmstat
	$(Q) install -p $(PRIVATE_SRC_DIR)/top $(TARGET_OUT_STAGING)/usr/bin/top2
	$(Q) install -p $(PRIVATE_SRC_DIR)/ps/ps $(TARGET_OUT_STAGING)/usr/bin/ps_procps
endef

define LOCAL_AUTOTOOLS_CMD_POST_CLEAN
	$(Q) rm -f $(TARGET_OUT_STAGING)/usr/lib/libproc-3.2.8.so
	$(Q) rm -f $(TARGET_OUT_STAGING)/usr/bin/vmstat
	$(Q) rm -f $(TARGET_OUT_STAGING)/usr/bin/top2
	$(Q) rm -f $(TARGET_OUT_STAGING)/usr/bin/ps_procps
endef

include $(BUILD_AUTOTOOLS)

