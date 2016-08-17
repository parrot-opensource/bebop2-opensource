#
# Makefile for industrial I/O ADC drivers for PARROT
#

ccflags-y += -I$(srctree)/drivers/parrot
#ccflags-y += -I$(srctree)/drivers/staging

obj-$(CONFIG_P7_ADC)                  += p7adc.o
obj-$(CONFIG_P7MU_ADC)                += p7muadc.o
obj-$(CONFIG_P7_TEMPERATURE)          += p7temp.o
obj-$(CONFIG_P7_TEMPERATURE_FC7100)   += p7temp_hw08.o

p7adc-y                               := p7-adc_core.o

p7muadc-y                              := p7mu-adc_core.o
p7muadc-$(CONFIG_P7MU_ADC_RING_BUFFER) += p7mu-adc_ring.o

