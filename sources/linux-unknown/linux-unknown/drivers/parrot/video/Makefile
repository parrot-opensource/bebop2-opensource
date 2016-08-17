ccflags-$(CONFIG_AVI_DEBUG_LOG) += -DDEBUG

avicore-objs := avi.o avi_compat.o avi_scaler.o avi_isp.o
avicore-objs += avi_segment.o avi_cap.o avi_chroma.o
avicore-objs += avi_dma.o avi_pixfmt.o
avicore-objs += avi_limit.o

ifeq ($(CONFIG_AVI_DEBUG),y)
avicore-objs += avi_debug.o
endif

ifeq ($(CONFIG_AVI_LOGGER),y)
avicore-objs += avi_logger.o
endif

obj-$(CONFIG_AVI) += avicore.o

obj-$(CONFIG_AVI_DEBUGFS) += avi_debugfs.o
obj-$(CONFIG_FB_AVI) += avifb.o
obj-$(CONFIG_VSYNC_GEN_AVI) += avi_vsync_gen.o
