/*
 * adv7391.h
 *
 * Driver for Analog device's 7391
 *
 * Author : Nicolas Laclau <nicolas.laclau@parrot.com>
 *
 * Date : 22 Jan. 2015
 *
 */

#ifndef ADV7391_H_
#define ADV7391_H_

#define ADV7391_NAME "adv7391"

/* registers list */
#define ADV7391_REG_PWR_MODE       (0x00)
#define ADV7391_REG_MODE_SEL       (0x01)
#define ADV7391_REG_DAC_OUT_LVL    (0x0B)
#define ADV7391_REG_SW_RESET       (0x17)
#define ADV7391_REG_SD_MODE1       (0x80)
#define ADV7391_REG_SD_MODE2       (0x82)
#define ADV7391_REG_SD_MODE3       (0x83)
#define ADV7391_REG_SD_MODE6       (0x87)
#define ADV7391_REG_SD_BRIGHT      (0xA1)
#define ADV7391_REG_SD_SCL_LSB     (0x9C)
#define ADV7391_REG_SD_Y_SCL       (0x9D)
#define ADV7391_REG_SD_CB_SCL      (0x9E)
#define ADV7391_REG_SD_CR_SCL      (0x9F)

/* flags for ADV7391_REG_PWR_MODE */
#define ADV7391_FLG_DAC1_PWR       (0x10)
#define ADV7391_FLG_DAC2_PWR       (0x08)
#define ADV7391_FLG_DAC3_PWR       (0x04)

/* flags for ADV7391_REG_SW_RESET */
#define ADV7391_FLG_SW_RESET       (0x02)

/* flags for ADV7391_REG_SD_MODE1 */
#define ADV7391_FLG_LUMA_SSAF      (0x10)

/* flags for ADV7391_REG_SD_MODE2 */
#define ADV7391_FLG_PRPB_SSAF_FILT (0x01)
#define ADV7391_FLG_DAC_OUT1       (0x02)
#define ADV7391_FLG_PEDESTAL       (0x08)
#define ADV7391_FLG_SQ_PIXEL       (0x10)
#define ADV7391_FLG_VCR_FF_FR_SYNC (0x20)
#define ADV7391_FLG_PIX_DT_VALID   (0x40)
#define ADV7391_FLG_VID_EDG_CTL    (0x80)

/* flags for ADV7391_REG_SD_MODE3 */
#define ADV7391_FLG_Y_OUT_LVL      (0x02) /*(dflt:700/300mV, set:714/286mV)*/

struct adv7391_platform_data {
	int dac_pwr;
	int dac_gain;
	int sd_mode1;
	int sd_mode2;
	int sd_mode3;
	int scale_lsb;
};

#endif /* ADV7391_H_ */
