
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := util-linux-ng
LOCAL_DESCRIPTION := Miscellaneous utility programs
LOCAL_CATEGORY_PATH := system

LOCAL_EXPORT_LDLIBS += -lfdisk

LOCAL_AUTOTOOLS_VERSION := 2.28
LOCAL_AUTOTOOLS_ARCHIVE := util-linux-$(LOCAL_AUTOTOOLS_VERSION).tar.xz
LOCAL_AUTOTOOLS_SUBDIR := util-linux-$(LOCAL_AUTOTOOLS_VERSION)

LOCAL_AUTOTOOLS_PATCHES := \
	fsync_gpt.patch \
	police.patch

# Update some licenses
define LOCAL_AUTOTOOLS_CMD_POST_UNPACK
	$(Q) cp -af $(PRIVATE_PATH)/.MODULE_LICENSE_BSD.libuuid \
		$(PRIVATE_SRC_DIR)/libuuid/.MODULE_LICENSE_BSD
	$(Q) cp -af $(PRIVATE_PATH)/.MODULE_LICENSE_LGPL2.libblkid \
		$(PRIVATE_SRC_DIR)/libblkid/.MODULE_LICENSE_LGPL2
	$(Q) cp -af $(PRIVATE_PATH)/.MODULE_LICENSE_LGPL2.libmount \
		$(PRIVATE_SRC_DIR)/libmount/.MODULE_LICENSE_LGPL2
	$(Q) cp -af $(PRIVATE_PATH)/.MODULE_LICENSE_LGPL2.libfdisk \
		$(PRIVATE_SRC_DIR)/libfdisk/.MODULE_LICENSE_LGPL2
	$(Q) cp -af $(PRIVATE_PATH)/.MODULE_LICENSE_PUBLIC_DOMAIN.blkdev.c        $(PRIVATE_SRC_DIR)/lib
	$(Q) cp -af $(PRIVATE_PATH)/.MODULE_LICENSE_PUBLIC_DOMAIN.crc32.c         $(PRIVATE_SRC_DIR)/lib
	$(Q) cp -af $(PRIVATE_PATH)/.MODULE_LICENSE_PUBLIC_DOMAIN.env.c           $(PRIVATE_SRC_DIR)/lib
	$(Q) cp -af $(PRIVATE_PATH)/.MODULE_LICENSE_PUBLIC_DOMAIN.linux_version.c $(PRIVATE_SRC_DIR)/lib
	$(Q) cp -af $(PRIVATE_PATH)/.MODULE_LICENSE_PUBLIC_DOMAIN.md5.c           $(PRIVATE_SRC_DIR)/lib
	$(Q) cp -af $(PRIVATE_PATH)/.MODULE_LICENSE_PUBLIC_DOMAIN.sysfs.c         $(PRIVATE_SRC_DIR)/lib
	$(Q) cp -af $(PRIVATE_PATH)/.MODULE_LICENSE_LGPL2.canonicalize.c          $(PRIVATE_SRC_DIR)/lib
	$(Q) cp -af $(PRIVATE_PATH)/.MODULE_LICENSE_LGPL2.fileutils.c             $(PRIVATE_SRC_DIR)/lib
	$(Q) cp -af $(PRIVATE_PATH)/.MODULE_LICENSE_LGPL2.randutils.c             $(PRIVATE_SRC_DIR)/lib
	$(Q) cp -af $(PRIVATE_PATH)/.MODULE_LICENSE_LGPL2.strutils.c              $(PRIVATE_SRC_DIR)/lib
endef

#list generated with ./configure --help | grep "do not build" | awk '{print "\t"$1" \\" }'
LOCAL_AUTOTOOLS_CONFIGURE_ARGS := \
        --disable-mount \
        --disable-losetup \
        --disable-cytune \
	--enable-fsck \
	--enable-sfdisk \
        --disable-partx \
        --disable-uuidd \
        --disable-mountpoint \
        --disable-fallocate \
        --disable-unshare \
        --enable-nsenter \
        --disable-setpriv \
        --disable-eject \
        --disable-agetty \
        --disable-cramfs \
        --disable-bfs \
        --disable-fdformat \
        --disable-hwclock \
        --disable-wdctl \
        --disable-switch_root \
        --disable-pivot_root \
        --disable-kill \
        --disable-last \
        --disable-utmpdump \
        --disable-mesg \
        --disable-raw \
        --disable-rename \
        --disable-login \
        --disable-nologin \
        --disable-sulogin \
        --disable-su \
        --disable-runuser \
        --disable-ul \
        --disable-more \
        --disable-pg \
        --disable-setterm \
        --disable-schedutils \
        --disable-wall \
        --disable-rpath \
        --without-ncurses \
        --without-python \
        --without-udev

#        --disable-libuuid \
#        --disable-libblkid \
#        --disable-libmount \

LOCAL_AUTOTOOLS_CONFIGURE_ARGS += \
	--prefix=/ \
	--bindir=/bin \
	--libdir=/lib \
	--includedir=/usr/include \
	--datarootdir=/usr/share

# Only install a subset
# use a local DESTDIR to not overide busybox binary
LOCAL_AUTOTOOLS_MAKE_INSTALL_ARGS := \
	DESTDIR=$(call local-get-build-dir)/$(LOCAL_AUTOTOOLS_SUBDIR)/install

define LOCAL_AUTOTOOLS_CMD_POST_INSTALL
	$(Q) install -p -m755 -d $(TARGET_OUT_STAGING)/sbin
	$(Q) install -p -m755 -d $(TARGET_OUT_STAGING)/usr/sbin
	$(Q) install -p -m755 -d $(TARGET_OUT_STAGING)/lib
	$(Q) install -p -m755 -d $(TARGET_OUT_STAGING)/lib/pkgconfig
	$(Q) install -p -m755 -d $(TARGET_OUT_STAGING)/usr
	$(Q) install -p -m755 -d $(TARGET_OUT_STAGING)/usr/include
	$(Q) install -p -m755 -d $(TARGET_OUT_STAGING)/usr/include/blkid
	$(Q) install -p -m755 -d $(TARGET_OUT_STAGING)/usr/include/libmount
	$(Q) install -p -m755 -d $(TARGET_OUT_STAGING)/usr/include/libfdisk
	$(Q) install -p -m755 -d $(TARGET_OUT_STAGING)/usr/include/libsmartcols
	$(Q) install -p -m755 -d $(TARGET_OUT_STAGING)/usr/include/uuid
	$(Q) for lib in blkid mount fdisk smartcols uuid; do \
			__src_dir=$(PRIVATE_SRC_DIR)/install/lib; \
			__dest_dir=$(TARGET_OUT_STAGING)/lib; \
			if [ -e "$${__src_dir}/lib$${lib}.a" ]; then \
				install -p -m755 $${__src_dir}/lib$${lib}.a $${__dest_dir}/; \
			fi; \
			install -p -m755 $${__src_dir}/pkgconfig/$${lib}.pc $${__dest_dir}/pkgconfig/; \
			if ls $${__src_dir}/lib$${lib}.so.*.*.* &>/dev/null; then \
				__libname=$$(basename $${__src_dir}/lib$${lib}.so.*.*.*); \
				install -p -m755 $${__src_dir}/$${__libname} $${__dest_dir}/; \
				ln -sf $${__libname} $${__dest_dir}/$${__libname%.*.*}; \
				ln -sf $${__libname} $${__dest_dir}/$${__libname%%.*}.so; \
			fi; \
		done
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/usr/include/blkid/blkid.h $(TARGET_OUT_STAGING)/usr/include/blkid/
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/usr/include/libmount/libmount.h $(TARGET_OUT_STAGING)/usr/include/libmount/
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/usr/include/libfdisk/libfdisk.h $(TARGET_OUT_STAGING)/usr/include/libfdisk/
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/usr/include/libsmartcols/libsmartcols.h $(TARGET_OUT_STAGING)/usr/include/libsmartcols/
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/usr/include/uuid/uuid.h $(TARGET_OUT_STAGING)/usr/include/uuid/
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/bin/dmesg $(TARGET_OUT_STAGING)/sbin/dmesg-ng
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/sbin/blkid $(TARGET_OUT_STAGING)/sbin/blkid-ng
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/sbin/fstrim $(TARGET_OUT_STAGING)/sbin/fstrim-ng
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/sbin/fdisk $(TARGET_OUT_STAGING)/sbin/fdisk-ng
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/bin/nsenter $(TARGET_OUT_STAGING)/usr/bin/nsenter-ng
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/sbin/fsck $(TARGET_OUT_STAGING)/usr/sbin/fsck-ng
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/sbin/sfdisk $(TARGET_OUT_STAGING)/usr/sbin/sfdisk-ng
endef

# Clean what we manually installed
# Libraries are cleaned because the uninstall is done on staging...
define LOCAL_AUTOTOOLS_CMD_POST_CLEAN
	$(Q) rm -f $(TARGET_OUT_STAGING)/sbin/blkid-ng
	$(Q) rm -f $(TARGET_OUT_STAGING)/sbin/dmesg-ng
	$(Q) rm -f $(TARGET_OUT_STAGING)/sbin/fstrim-ng
	$(Q) rm -f $(TARGET_OUT_STAGING)/usr/bin/nsenter-ng
endef

LOCAL_CREATE_LINKS := etc/mtab:/proc/mounts

include $(BUILD_AUTOTOOLS)

