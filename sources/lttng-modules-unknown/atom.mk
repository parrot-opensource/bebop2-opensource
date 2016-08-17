
LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LTTNG_MODULES_VERSION := 2.1.0
LOCAL_MODULE := lttng-modules
LOCAL_MODULE_FILENAME := lttng-modules.done
LOCAL_DESCRIPTION := Kernel modules (requires Linux >= 2.6.38).
LOCAL_CATEGORY_PATH := devel/lttng
LOCAL_LIBRARIES := linux

LTTNG_MODULES_BUILD_DIR := $(call local-get-build-dir)
LTTNG_MODULES_SRC_DIR := $(LTTNG_MODULES_BUILD_DIR)/$(LOCAL_MODULE)-$(LTTNG_MODULES_VERSION)

LOCAL_ARCHIVE := $(LOCAL_MODULE)-$(LTTNG_MODULES_VERSION).tar.bz2
LOCAL_ARCHIVE_SUBDIR := $(LOCAL_MODULE)-$(LTTNG_MODULES_VERSION)
LOCAL_ARCHIVE_PATCHES := remove_ARM_set_tls_system_call_override.patch parrot_trace.patch fix_syscall_ret.patch

# Make sure the -C parameter come after
# $(LINUX_MAKE_ARGS) to override default value
$(LTTNG_MODULES_BUILD_DIR)/$(LOCAL_MODULE_FILENAME):
	@echo "Building LTTng modules"
	$(Q) KERNELDIR=$(LINUX_DIR) make $(LINUX_MAKE_ARGS) -C $(LTTNG_MODULES_SRC_DIR)
	@echo "Installing LTTng modules"
	$(Q) KERNELDIR=$(LINUX_DIR) make $(LINUX_MAKE_ARGS) -C $(LTTNG_MODULES_SRC_DIR) modules_install
	@touch $@

$(LOCAL_MODULE)-clean:
	$(Q) if [ -d $(LTTNG_MODULES_SRC_DIR) ]; then \
			KERNELDIR=$(LINUX_DIR) make $(LINUX_MAKE_ARGS) -C $(LTTNG_MODULES_SRC_DIR) clean \
				|| echo "Ignoring clean errors"; \
		fi

include $(BUILD_CUSTOM)
