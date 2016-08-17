
#ifndef _WAU8822_H_
#define _WAU8822_H_

/* wau8822 register space */
#define WAU8822_RESET		0x00
#define WAU8822_POWER1		0x01
#define WAU8822_POWER2		0x02
#define WAU8822_POWER3		0x03
#define WAU8822_IFACE		0x04
#define WAU8822_COMP		0x05
#define WAU8822_CLOCK1		0x06
#define WAU8822_CLOCK2		0x07
#define WAU8822_GPIO		0x08
#define WAU8822_JACK1		0x09
#define WAU8822_DAC		0x0a
#define WAU8822_DACVOLL		0x0b
#define WAU8822_DACVOLR		0x0c
#define WAU8822_JACK2		0x0d
#define WAU8822_ADC		0x0e
#define WAU8822_ADCVOLL		0x0f
#define WAU8822_ADCVOLR		0x10
/* 0x11 is reserved */
#define WAU8822_EQ1		0x12
#define WAU8822_EQ2		0x13
#define WAU8822_EQ3		0x14
#define WAU8822_EQ4		0x15
#define WAU8822_EQ5		0x16
/* 0x17 is reserved */
#define WAU8822_DACLIM1		0x18
#define WAU8822_DACLIM2		0x19
/* 0x1a is reserved */
#define WAU8822_NOTCH1		0x1b
#define WAU8822_NOTCH2		0x1c
#define WAU8822_NOTCH3		0x1d
#define WAU8822_NOTCH4		0x1e
/* 0x1f is reserved */
#define WAU8822_ALC1		0x20
#define WAU8822_ALC2		0x21
#define WAU8822_ALC3		0x22
#define WAU8822_NGATE		0x23
#define WAU8822_PLLN		0x24
#define WAU8822_PLLK1		0x25
#define WAU8822_PLLK2		0x26
#define WAU8822_PLLK3		0x27
/* 0x28 is reserved */
#define WAU8822_3D		0x29
/* 0x2a is reserved */
#define WAU8822_RSPKSUBMIX	0x2b
#define WAU8822_INPUT		0x2c
#define WAU8822_INPGAL		0x2d
#define WAU8822_INPGAR		0x2e
#define WAU8822_ADCBOOSTL	0x2f
#define WAU8822_ADCBOOSTR	0x30
#define WAU8822_OUTPUT		0x31
#define WAU8822_MIXL		0x32
#define WAU8822_MIXR		0x33
#define WAU8822_HPVOLL		0x34
#define WAU8822_HPVOLR		0x35
#define WAU8822_SPKVOLL		0x36
#define WAU8822_SPKVOLR		0x37
#define WAU8822_AUX2MIX		0x38
#define WAU8822_AUX1MIX		0x39
#define WAU8822_POWER4		0x3a
#define WAU8822_TIMESLOTL	0x3b
#define WAU8822_MISC		0x3c
#define WAU8822_TIMESLOTR	0x3d
#define WAU8822_REV		0x3e
#define WAU8822_ID		0x3f

#define WAU8822_CACHEREG_NUM	64

/* Clock divider Id's */
#define WAU8822_OPCLKDIV	0
#define WAU8822_MCLKDIV		1
#define WAU8822_ADCCLK		2
#define WAU8822_DACCLK		3
#define WAU8822_BCLKDIV		4

#endif

