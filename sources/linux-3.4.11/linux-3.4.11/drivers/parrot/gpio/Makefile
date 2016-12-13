ccflags-$(CONFIG_DEBUG_GPIO)    += -DDEBUG

obj-$(CONFIG_GPIO_PARROT7)      += p7-gpio.o

CFLAGS_p7mu-gpio.o              += -I$(srctree)/drivers/parrot
obj-$(CONFIG_GPIO_P7MU)         += p7mu-gpio.o
