
# Linux only
ifeq ("$(TARGET_OS)","linux")

# Disable under bionic if prebuilt busydroid is used
ifneq ("$(USE_ALCHEMY_ANDROID_BUSYDROID)","1")

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := busybox
LOCAL_MODULE_FILENAME := busybox.done
LOCAL_CATEGORY_PATH := system

# Variables
BUSYBOX_DIR := $(LOCAL_PATH)
BUSYBOX_VERSION := 1.25.0
BUSYBOX_ARCHIVE_FILE := $(LOCAL_PATH)/busybox-$(BUSYBOX_VERSION).tar.bz2

# Build directory and source sirectory inside build directory (after unpacking)
BUSYBOX_BUILD_DIR := $(call local-get-build-dir)
BUSYBOX_SRC_DIR := $(BUSYBOX_BUILD_DIR)/busybox-$(BUSYBOX_VERSION)
BUSYBOX_UNPACKED_FILE := $(BUSYBOX_BUILD_DIR)/busybox.unpacked

# Patches
BUSYBOX_PATCHES := \
	busybox-1.25.0-bin_init.patch \
	busybox-1.25.0-clang.patch

ifeq ("$(TARGET_LIBC)","bionic")
BUSYBOX_PATCHES += \
	busybox-1.25.0-bionic-mount-umount-fsck-df.patch \
	busybox-1.25.0-bionic-route-sys-socket.patch \
	busybox-1.25.0-bionic-link.patch \
	busybox-1.25.0-bionic-force-install-bin-dir.patch
endif

# Busybox configuration file
BUSYBOX_CONFIG_FILE := $(call module-get-config,$(LOCAL_MODULE))
ifeq ("$(wildcard $(BUSYBOX_CONFIG_FILE))","")
    ifeq ("$(TARGET_LIBC)","bionic")
        BUSYBOX_CONFIG_FILE := $(LOCAL_PATH)/busybox-bionic.config
    else
        BUSYBOX_CONFIG_FILE := $(LOCAL_PATH)/busybox.config
    endif
endif

BUSYBOX_CFLAGS := \
	$(TARGET_GLOBAL_CFLAGS) \
	-Wno-sign-compare -Wno-error=format-security -Wno-unused-result \
	$(call normalize-c-includes,$(TARGET_GLOBAL_C_INCLUDES))

# Make arguments
BUSYBOX_MAKE_ARGS := \
	ARCH=$(TARGET_ARCH) \
	CC="$(CCACHE) $(TARGET_CC)" \
	AR="$(TARGET_AR)" \
	NM="$(TARGET_NM)" \
	CROSS_COMPILE="$(TARGET_CROSS)" \
	CROSS="$(TARGET_CROSS)" \
	CONFIG_PREFIX="$(TARGET_OUT_STAGING)" \
	PREFIX="$(TARGET_OUT_STAGING)" \
	CFLAGS="$(BUSYBOX_CFLAGS)" \
	LDFLAGS="$(filter-out -pie,$(TARGET_GLOBAL_LDFLAGS)) -Wl,--no-fatal-warnings" \
	V=$(V)

# To link correctly with bionic, a patch is needed to specify aditional flags
ifeq ("$(TARGET_LIBC)","bionic")
BUSYBOX_MAKE_ARGS += \
	LDFLAGS_busybox="$(TARGET_GLOBAL_LDLIBS) -pie"
endif

# Extract the archive + apply patches
$(BUSYBOX_UNPACKED_FILE): $(BUSYBOX_ARCHIVE_FILE) $(addprefix $(LOCAL_PATH)/,$(BUSYBOX_PATCHES))
	@echo "Unpacking busybox"
	@mkdir -p $(BUSYBOX_BUILD_DIR)
	$(Q)tar -C $(BUSYBOX_BUILD_DIR) -xf $(BUSYBOX_ARCHIVE_FILE)
	$(Q)$(BUILD_SYSTEM)/scripts/apply-patches.sh $(BUSYBOX_SRC_DIR) $(BUSYBOX_DIR) $(BUSYBOX_PATCHES)
	$(Q)cp -af $(BUSYBOX_DIR)/.MODULE_LICENSE_GPL $(BUSYBOX_SRC_DIR)
	$(Q)cp -af $(BUSYBOX_DIR)/.MODULE_NAME_busybox $(BUSYBOX_SRC_DIR)
	@touch $@

# Copy config in build dir
$(BUSYBOX_SRC_DIR)/.config: $(BUSYBOX_CONFIG_FILE) $(BUSYBOX_UNPACKED_FILE)
	@mkdir -p $(dir $@)
	@cp -af $< $@

# Build
$(BUSYBOX_BUILD_DIR)/$(LOCAL_MODULE_FILENAME): $(BUSYBOX_SRC_DIR)/.config
	@echo "Checking busybox config: $(BUSYBOX_CONFIG_FILE)"
	$(Q)yes "" 2>/dev/null | $(MAKE) $(BUSYBOX_MAKE_ARGS) -C $(BUSYBOX_SRC_DIR) oldconfig
	@echo "Building busybox"
	$(Q)$(MAKE) $(BUSYBOX_MAKE_ARGS) -C $(BUSYBOX_SRC_DIR) SKIP_STRIP=y
	@echo "Installing busybox"
	$(Q) $(MAKE) $(BUSYBOX_MAKE_ARGS) -C $(BUSYBOX_SRC_DIR) install
	@rm -rf $(TARGET_OUT_STAGING)/linuxrc
	@touch $@

# Busybox configuration
.PHONY: busybox-menuconfig
busybox-menuconfig: $(BUSYBOX_SRC_DIR)/.config
	@echo "Configuring busybox: $(BUSYBOX_CONFIG_FILE)"
	$(Q)$(MAKE) $(BUSYBOX_MAKE_ARGS) -C $(BUSYBOX_SRC_DIR) menuconfig
	@cp -af $(BUSYBOX_SRC_DIR)/.config $(BUSYBOX_CONFIG_FILE)

.PHONY: busybox-xconfig
busybox-xconfig: $(BUSYBOX_SRC_DIR)/.config
	@echo "Configuring busybox: $(BUSYBOX_CONFIG_FILE)"
	$(Q)$(MAKE) $(BUSYBOX_MAKE_ARGS) -C $(BUSYBOX_SRC_DIR) xconfig
	@cp -af $(BUSYBOX_SRC_DIR)/.config $(BUSYBOX_CONFIG_FILE)

.PHONY: busybox-config
busybox-config: busybox-xconfig

# Custom clean rule. LOCAL_MODULE_FILENAME already deleted by common rule
.PHONY: busybox-clean
busybox-clean:
	$(Q)if [ -d $(BUSYBOX_SRC_DIR) ]; then \
		$(MAKE) $(BUSYBOX_MAKE_ARGS) -C $(BUSYBOX_SRC_DIR) uninstall \
			|| echo "Ignoring uninstall errors"; \
		$(MAKE) $(BUSYBOX_MAKE_ARGS) -C $(BUSYBOX_SRC_DIR) clean \
			|| echo "Ignoring clean errors"; \
	fi

# Copy udhcpd related files if udhcpd selected in busybox config file
ifneq ("$(wildcard $(BUSYBOX_CONFIG_FILE))","")
ifneq ("$(shell grep "^CONFIG_UDHCPD=y" $(BUSYBOX_CONFIG_FILE))","")
LOCAL_COPY_FILES += \
	20-udhcpd.rc:etc/boxinit.d/ \
	udhcpd.leases:etc/
endif
endif

include $(BUILD_CUSTOM)

endif

endif

