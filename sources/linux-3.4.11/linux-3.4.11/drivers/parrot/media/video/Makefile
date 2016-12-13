ccflags-y                   += -I$(srctree)/drivers/parrot
ccflags-$(CONFIG_AVI_DEBUG_LOG) += -DDEBUG

obj-$(CONFIG_AVI_V4L2)    += avi_v4l2.o

obj-$(CONFIG_AVI_V4L2_ISP) += avi_v4l2_isp.o

avicam-objs := avicam_v4l2.o avicam_stats.o avi_capture.o

obj-$(CONFIG_CAM_AVI_DUMMY) += avicam_dummy_dev.o
obj-$(CONFIG_CAM_AVI)       += avicam.o
obj-$(CONFIG_AVICAM_SUBDEV) += avicam_subdev.o

avivoc-objs := avi_voc_ctrl.o avi_voc.o
obj-$(CONFIG_VIDOUT_AVI)    += avivoc.o

avi_r2r-objs := avi_r2r_core.o
ifeq ($(CONFIG_AVI_DEBUG),y)
avi_r2r-objs += avi_r2r_debugfs.o
endif
obj-$(CONFIG_R2R_AVI)            += avi_r2r.o

obj-$(CONFIG_R2R_AVI_TEST_MULTI) += avi_r2r_test_multi.o

obj-$(CONFIG_M2M_AVI)  += avi_m2m.o avi_stats.o

obj-$(CONFIG_AVI_CAPTURE_TEST) += avi_capture_test.o

obj-$(CONFIG_CAM_LEPTON)     += lepton.o lepton_dev.o

obj-$(CONFIG_AVI_MULTICAPTURE) += avi_multicapture.o

obj-$(CONFIG_AVI_V4L2_DEV) += avi_v4l2_dev.o
