#############################################################
#
# mtp-probe : tool for mtp device detection
#
#############################################################
PACKAGE_NAME = mtp_probe
PACKAGE_DEPS = libusb_1_0 udev

LIBMTP_VERSION := 1.1.5
my-dir = $(abspath $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST)))))
MTP_PROBE_DIR := $(call my-dir)/..
MTP_PROBE_BUILDDIR := $(BUILD_DIR)/$(PACKAGE_NAME)

MTP_PROBE_BIN := mtp-probe
MTP_PROBE_PATH := /lib/udev/

$(MTP_PROBE_BUILDDIR)/.configured:
	$(Q) mkdir -p $(@D)
	$(Q) touch $@
	@echo $@ done

$(PACKAGE_NAME): $(PACKAGE_DEPS) $(MTP_PROBE_BUILDDIR)/.configured
	@echo building $@
	$(Q) (cd $(MTP_PROBE_BUILDDIR); \
			SRC_DIR=$(MTP_PROBE_DIR) \
			STAGING=$(STAGING_DIR) \
			$(TARGET_CONFIGURE_OPTS) \
			make -f $(MTP_PROBE_DIR)/lucie/Makefile;)
	@echo installing $@
	$(Q) mkdir -p $(TARGET_DIR)/$(MTP_PROBE_PATH)
	$(Q) install -m0755 $(MTP_PROBE_BUILDDIR)/$(MTP_PROBE_BIN) $(TARGET_DIR)/$(MTP_PROBE_PATH)

$(PACKAGE_NAME)-source:
	$(Q) mkdir -p $(SOURCES_DIR)
	$(Q) (cd $(MTP_PROBE_DIR)/ && \
		tar -czf $(SOURCES_DIR)/$(PACKAGE_NAME)-$(LIBMTP_VERSION).tar.gz *)
	@echo $@ done.

$(PACKAGE_NAME)-clean:
	cd $(MTP_PROBE_BUILDDIR); rm -f $(MTP_PROBE_OBJ); rm -f $(MTP_PROBE_BIN)

$(PACKAGE_NAME)-dirclean: $(PACKAGE_NAME)-clean
	@echo $@
	$(Q) $(RM) -Rf $(MTP_PROBE_BUILDDIR)
	@echo $@ done.

#############################################################
#
# Toplevel Makefile options
#
#############################################################
TARGETS-$(BR2_PACKAGE_MTP_PROBE)+=$(PACKAGE_NAME)

SOURCES-$(BR2_PACKAGE_MTP_PROBE)+=$(PACKAGE_NAME)-source
