obj-$(CONFIG_SND_AAI) += aai.o

aai-objs := \
	aai_alsa.o	\
	aai_group.o	\
	aai_hw.o	\
	aai_ioctl.o	\
	aai_irq.o	\
	aai_module.o	\
	aai_ops.o	\
	aai_clk.o	\
	aai_regs.o	\
	aai_rules.o

ifeq ($(CONFIG_SND_AAI_DEBUG), y)
EXTRA_CFLAGS += -DDEBUG
endif
