ifeq ($(CONFIG_ARCH_PARROT7),y)
	obj-$(CONFIG_PWM_PARROT7) += p7_pwm.o
endif

ifeq ($(CONFIG_PWM_P7MU_DEBUG),y)
	CFLAGS_p7mu-pwm.o   += -DDEBUG
endif
CFLAGS_p7mu-pwm.o       += -I$(srctree)/drivers/parrot
obj-$(CONFIG_PWM_P7MU)  += p7mu-pwm.o
