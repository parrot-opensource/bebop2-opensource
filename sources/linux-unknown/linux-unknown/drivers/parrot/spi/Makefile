ccflags-y                   += -I$(srctree)/drivers/parrot
ccflags-$(CONFIG_SPI_DEBUG) += -DDEBUG

obj-$(CONFIG_SPI_PARROT7)           += p7-spi-common.o
p7-spi-common-objs := p7-swb.o p7-spi.o
obj-$(CONFIG_SPI_MASTER_PARROT7)    += p7-spim.o
obj-$(CONFIG_SPI_SLAVE_PARROT7)     += p7-spis.o

obj-$(CONFIG_SPI_MASTER_PARROT7_TEST)   += p7-spim_test.o
obj-$(CONFIG_SPI_MASTER_SLAVE_TEST)     += p7-spis_test_p7mu.o

obj-$(CONFIG_SPI_MASTER_PARROT7_LOOPBACK_TEST)   += spimtest_loopback.o
obj-$(CONFIG_SPI_SLAVE_PARROT7_LOOPBACK_TEST)    += spistest_loopback.o
