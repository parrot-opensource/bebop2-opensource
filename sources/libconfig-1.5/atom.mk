LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libconfig
LOCAL_DESCRIPTION := Simple library for processing structured configuration files
LOCAL_CATEGORY_PATH := libs

LOCAL_AUTOTOOLS_VERSION := 1.5
LOCAL_AUTOTOOLS_ARCHIVE := libconfig-$(LOCAL_AUTOTOOLS_VERSION).tar.gz
LOCAL_AUTOTOOLS_SUBDIR := libconfig-$(LOCAL_AUTOTOOLS_VERSION)

LOCAL_AUTOTOOLS_CONFIGURE_ARGS := --disable-examples

LOCAL_EXPORT_LDLIBS := -lconfig

# The c++ api requires exceptions
ifeq ("$(TARGET_USE_CXX_EXCEPTIONS)","1")
  LOCAL_AUTOTOOLS_CONFIGURE_ARGS += --enable-cxx
  LOCAL_EXPORT_LDLIBS += -lconfig++
else
  LOCAL_AUTOTOOLS_CONFIGURE_ARGS += --disable-cxx
endif


include $(BUILD_AUTOTOOLS)

###############################################################################
## Build for host too
###############################################################################
include $(CLEAR_VARS)

LOCAL_HOST_MODULE := libconfig
LOCAL_DESCRIPTION := Simple library for processing structured configuration files
LOCAL_CATEGORY_PATH := libs

LOCAL_AUTOTOOLS_VERSION := 1.5
LOCAL_AUTOTOOLS_ARCHIVE := libconfig-$(LOCAL_AUTOTOOLS_VERSION).tar.gz
LOCAL_AUTOTOOLS_SUBDIR := libconfig-$(LOCAL_AUTOTOOLS_VERSION)

LOCAL_AUTOTOOLS_CONFIGURE_ARGS := --disable-examples

LOCAL_EXPORT_LDLIBS := -lconfig

# The c++ api requires exceptions
ifeq ("$(TARGET_USE_CXX_EXCEPTIONS)","1")
  LOCAL_AUTOTOOLS_CONFIGURE_ARGS += --enable-cxx
  LOCAL_EXPORT_LDLIBS += -lconfig++
else
  LOCAL_AUTOTOOLS_CONFIGURE_ARGS += --disable-cxx
endif

include $(BUILD_AUTOTOOLS)

###############################################################################
## Host tool for checking and rewriting libconfig configuration files
###############################################################################
include $(CLEAR_VARS)

LOCAL_HOST_MODULE := libconfig_rewrite
LOCAL_DEPENDS_HOST_MODULES := host.libconfig
LOCAL_DESCRIPTION := Tool to check libconfig input files offline and rewrite them without comments

LIBCONFIG_REWRITE_BUILD_DIR := $(call local-get-build-dir)
LIBCONFIG_REWRITE_SRC_FILE := $(LOCAL_PATH)/tools/config_rewrite.c
LIBCONFIG_REWRITE_OUT_FILE := $(LIBCONFIG_REWRITE_BUILD_DIR)/$(LOCAL_HOST_MODULE)
LIBCONFIG_REWRITE_STAGED_FILE := $(HOST_OUT_STAGING)/usr/bin/$(LOCAL_HOST_MODULE)

$(LIBCONFIG_REWRITE_OUT_FILE): $(LIBCONFIG_REWRITE_SRC_FILE)
	$(Q) $(HOST_CC) -I$(HOST_OUT_STAGING)/usr/include $^ -o $@ -L$(HOST_OUT_STAGING)/usr/lib -lconfig 

LOCAL_EXPORT_PREREQUISITES := $(LIBCONFIG_REWRITE_STAGED_FILE)
LOCAL_CUSTOM_TARGETS := $(LIBCONFIG_REWRITE_OUT_FILE)
LOCAL_COPY_FILES := $(LIBCONFIG_REWRITE_OUT_FILE):$(LIBCONFIG_REWRITE_STAGED_FILE)

include $(BUILD_CUSTOM)

###############################################################################
## Custom macro that can be used in LOCAL_CUSTOM_MACROS of a module
##
## Rewrite a libconfig configuration file without comments and transform it
## into a C string. The output is a C header file. The string is suitable for
## being parsed with config_read_string.
##
## The purpose is to embed sensible default settings in an application or
## library, while not resorting to C macros. The original configuration file
## can serve as an example for a site specific configuration.
##
## Note : in the context of the macro, LOCAL_XXX variables refer to the module
## that use the macro, not this module defining the macro.
## As the content of the macro is 'eval'-ed after, most of variable ref shall be
## escaped (hence the $$). Only $1, $2... variables can be used directly.
## Note : no 'global' variable shall be used except the ones defined by
## alchemy (TARGET_XXX and HOST_XXX variables). Otherwise the macro will not
## work when integrated in a SDK (using local-register-custom-macro).
## Note : rules shoud NOT use any variables defined in the context of the
## macro (for the same reason PRIVATE_XXX variables shall be used in place of
## LOCAL_XXX variables).
## Note : if you need a script or a binary, please install it in host staging
## directory and execute it from there. This way it will also work in the
## context of a SDK.
###############################################################################

# $1: input config file, absolute path
# $2: output header file, file name only
define libconfig_to_c-macro

# Setup some internal variables
libconfig_in_file := $1
libconfig_module_build_dir := $(call local-get-build-dir)
libconfig_out_file := $$(libconfig_module_build_dir)/$2
libconfig_temp_file := $$(libconfig_module_build_dir)/$2.temp
libconfig_done_file := $$(libconfig_module_build_dir)/$$(notdir $$(libconfig_in_file)).done

# Actual generation rules
$$(libconfig_temp_file): $$(libconfig_in_file)
	@mkdir -p $$(dir $$@)
	$(Q) $(HOST_OUT_STAGING)/usr/bin/libconfig_rewrite $$< $$@

## sed command line does the following:
## Escape existing '\'
## Escape quotes
## Remove indentation
## Escape EOL
$$(libconfig_out_file): $$(libconfig_temp_file)
	$$(call print-banner1,"Generating",$$(call path-from-top,$$@))
	$(Q) echo -n '"' > $$@
	$(Q) cat $$< | sed -e 's/\\/\\\\/g' -e 's/"/\\"/g' -re 's/^\s+//' | sed -ze 's/\n/\\n/g' >> $$@
	$(Q) echo -n '"' >> $$@

$$(libconfig_done_file): $$(libconfig_out_file)
	@touch $$@

## Update LOCAL_C_INCLUDES
LOCAL_C_INCLUDES += $$(libconfig_module_build_dir)

# Update alchemy variables for the module
LOCAL_CLEAN_FILES += $$(libconfig_done_file) $$(libconfig_temp_file) $$(libconfig_out_file)
LOCAL_EXPORT_PREREQUISITES += $$(libconfig_out_file)
LOCAL_CUSTOM_TARGETS += $$(libconfig_done_file) $$(libconfig_temp_file)
LOCAL_DEPENDS_HOST_MODULES += host.libconfig_rewrite

endef

# Register the macro in alchemy so it will be integrated in generated sdk
$(call local-register-custom-macro,libconfig_to_c-macro)

