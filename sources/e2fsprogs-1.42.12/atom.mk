LOCAL_PATH := $(call my-dir)

###############################################################################
# e2fsprogs
###############################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := e2fsprogs
LOCAL_DESCRIPTION := Filesystem utilities for use with the extX filesystems
LOCAL_CATEGORY_PATH := fs

LOCAL_AUTOTOOLS_PATCHES := \
	noinline_getclock_toolchain_2010.patch \

LOCAL_AUTOTOOLS_VERSION := 1.42.12
LOCAL_AUTOTOOLS_ARCHIVE := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION).tar.gz
LOCAL_AUTOTOOLS_SUBDIR := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION)

E2FSPROGS_DIR := $(call local-get-build-dir)/$(LOCAL_AUTOTOOLS_SUBDIR)

LOCAL_DESCRIPTION := Ext2/3/4 Filesystem Utilities

# define police files, with different licenses for some submodules, hence
# created where the source files are un-tared
define LOCAL_AUTOTOOLS_CMD_POST_INSTALL
	$(Q) touch $(E2FSPROGS_DIR)/.MODULE_LICENSE_GPL2
	$(Q) touch $(E2FSPROGS_DIR)/.MODULE_NAME_e2fsprogs

	$(Q) touch $(E2FSPROGS_DIR)/lib/ext2fs/.MODULE_LICENSE_LGPL2
	$(Q) touch $(E2FSPROGS_DIR)/lib/ext2fs/.MODULE_NAME_ext2fs
	$(Q) touch $(E2FSPROGS_DIR)/lib/e2p/.MODULE_LICENSE_LGPL2
	$(Q) touch $(E2FSPROGS_DIR)/lib/e2p/.MODULE_NAME_e2p

	$(Q) cp $(E2FSPROGS_DIR)/lib/uuid/COPYING \
		$(E2FSPROGS_DIR)/lib/uuid/.MODULE_LICENSE_BSD
	$(Q) touch $(E2FSPROGS_DIR)/lib/uuid/.MODULE_NAME_uuid

	$(Q) touch $(E2FSPROGS_DIR)/lib/et/.MODULE_LICENSE_MIT_LIKE
	$(Q) touch $(E2FSPROGS_DIR)/lib/et/.MODULE_NAME_et
	$(Q) touch $(E2FSPROGS_DIR)/lib/ss/.MODULE_LICENSE_MIT_LIKE
	$(Q) touch $(E2FSPROGS_DIR)/lib/ss/.MODULE_NAME_ss
endef

LOCAL_AUTOTOOLS_CONFIGURE_ARGS := \
--enable-symlink-install

include $(BUILD_AUTOTOOLS)


###############################################################################
# e2fsprogs-static
###############################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := e2fsprogs-static
LOCAL_ARCHIVE := e2fsprogs
LOCAL_DESCRIPTION := Filesystem utilities for use with the extX filesystems in \
static compilation
LOCAL_CATEGORY_PATH := fs

LOCAL_AUTOTOOLS_PATCHES := \
	noinline_getclock_toolchain_2010.patch \

LOCAL_AUTOTOOLS_VERSION := 1.42.12
LOCAL_AUTOTOOLS_ARCHIVE := $(LOCAL_ARCHIVE)-$(LOCAL_AUTOTOOLS_VERSION).tar.gz
LOCAL_AUTOTOOLS_SUBDIR := $(LOCAL_ARCHIVE)-$(LOCAL_AUTOTOOLS_VERSION)
LOCAL_AUTOTOOLS_CONFIGURE_ARGS := LDFLAGS=-static

E2FSPROGS_STATIC_DIR := $(call local-get-build-dir)/$(LOCAL_AUTOTOOLS_SUBDIR)

LOCAL_DESCRIPTION := Ext2/3/4 Filesystem Utilities

# define police files, with different licenses for some submodules, hence
# created where the source files are un-tared
define LOCAL_AUTOTOOLS_CMD_POST_INSTALL
	$(Q) touch $(E2FSPROGS_STATIC_DIR)/.MODULE_LICENSE_GPL2
	$(Q) touch $(E2FSPROGS_STATIC_DIR)/.MODULE_NAME_e2fsprogs

	$(Q) touch $(E2FSPROGS_STATIC_DIR)/lib/ext2fs/.MODULE_LICENSE_LGPL2
	$(Q) touch $(E2FSPROGS_STATIC_DIR)/lib/ext2fs/.MODULE_NAME_ext2fs
	$(Q) touch $(E2FSPROGS_STATIC_DIR)/lib/e2p/.MODULE_LICENSE_LGPL2
	$(Q) touch $(E2FSPROGS_STATIC_DIR)/lib/e2p/.MODULE_NAME_e2p

	$(Q) cp $(E2FSPROGS_STATIC_DIR)/lib/uuid/COPYING \
		$(E2FSPROGS_STATIC_DIR)/lib/uuid/.MODULE_LICENSE_BSD
	$(Q) touch $(E2FSPROGS_STATIC_DIR)/lib/uuid/.MODULE_NAME_uuid

	$(Q) touch $(E2FSPROGS_STATIC_DIR)/lib/et/.MODULE_LICENSE_MIT_LIKE
	$(Q) touch $(E2FSPROGS_STATIC_DIR)/lib/et/.MODULE_NAME_et
	$(Q) touch $(E2FSPROGS_STATIC_DIR)/lib/ss/.MODULE_LICENSE_MIT_LIKE
	$(Q) touch $(E2FSPROGS_STATIC_DIR)/lib/ss/.MODULE_NAME_ss
endef

include $(BUILD_AUTOTOOLS)
