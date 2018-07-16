# iproute2

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)


LOCAL_MODULE := iproute2
LOCAL_DESCRIPTION := Iproute2 is a collection of utilities for controlling TCP / IP networking and traffic control in Linux.
LOCAL_CATEGORY_PATH := network
IPROUTE2_TARGET_SBINS := bridge  ctstat  genl  ifcfg  ifstat  ip  lnstat  nstat  routef  routel  rtacct  rtmon  rtpr  rtstat  ss  tc

LOCAL_AUTOTOOLS_VERSION := 3.9.0
LOCAL_AUTOTOOLS_ARCHIVE := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION).tar.gz
LOCAL_AUTOTOOLS_SUBDIR := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION)/

$(call load-config)
ifdef CONFIG_ALCHEMY_BUILD_IPTABLES
define	IPROUTE2_WITH_IPTABLES
	# em_ipset needs xtables, but configure misdetects it
	echo "TC_CONFIG_IPSET:=n" >>$(PRIVATE_SRC_DIR)/Config
endef
endif
define	IPROUTE2_WITH_NETEM
	echo "TC_CONFIG_NETEM:=y" >>$(PRIVATE_SRC_DIR)/Config
endef

LOCAL_LIBRARIES := iptables

# If both iproute2 and busybox are selected, make certain we win
# the fight over who gets to have their utils actually installed.
ifdef CONFIG_ALCHEMY_BUILD_BUSYBOX
LOCAL_LIBRARIES += busybox
endif

LOCAL_AUTOTOOLS_PATCHES := \
	iproute2-3.9.0-ss_build_fix.patch \
	append_to_environment_cflags.patch \
	include-stdint.h-explicitly-for-UINT16_MAX.patch

define LOCAL_AUTOTOOLS_CMD_POST_CONFIGURE
	sed -i "s/-Werror//g" $(PRIVATE_SRC_DIR)/Makefile
	# arpd needs berkeleydb: disable arpd build
	sed -i "/^TARGETS=/s: arpd : :" $(PRIVATE_SRC_DIR)/misc/Makefile
	echo "IPT_LIB_DIR:=/usr/lib/xtables" >>$(PRIVATE_SRC_DIR)/Config
	$(IPROUTE2_WITH_IPTABLES)
	sed -i "/TCSO :=/a \
		ifeq ($(TC_CONFIG_NETEM),y) \
		  TCSO += q_netem.so \
		endif" $(PRIVATE_SRC_DIR)/Makefile
	$(IPROUTE2_WITH_NETEM)

endef

LOCAL_AUTOTOOLS_MAKE_BUILD_ENV := \
	$(AUTOTOOLS_CONFIGURE_ENV)

LOCAL_AUTOTOOLS_MAKE_BUILD_ARGS := \
	CC="$(CCACHE) $(TARGET_CC)"

LOCAL_AUTOTOOLS_MAKE_INSTALL_ENV := \
	$(AUTOTOOLS_CONFIGURE_ENV)

LOCAL_AUTOTOOLS_MAKE_INSTALL_ARGS := \
	CC="$(CCACHE) $(TARGET_CC)" \
	DESTDIR="$(TARGET_OUT_STAGING)" \
	BINDIR="/usr/sbin"

define LOCAL_AUTOTOOLS_CMD_POST_CLEAN
	$(Q) rm -rf $(TARGET_OUT_STAGING)/usr/lib/tc
	$(Q) rm -rf $(TARGET_OUT_STAGING)/etc/iproute2
	$(Q) rm -rf $(TARGET_OUT_STAGING)/var/lib/arpd
	$(Q) rm -f $(addprefix $(TARGET_OUT_STAGING)/sbin/, $(IPROUTE2_TARGET_SBINS))
endef


include $(BUILD_AUTOTOOLS)

