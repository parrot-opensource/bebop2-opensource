/**
 * Kernel IIO driver for the TSL2591 illuminance I2C chip.
 * Datasheet : http://www.adafruit.com/datasheets/TSL25911_Datasheet_EN_v1.pdf
 *
 * Author : Maxime Jourdan <maxime.jourdan@parrot.com>
 * Date   : 14/01/2015
 * Author : Luis Mario Domenzain <ld@airinov.fr>
 * Date   : 12/10/2015
 */
#ifndef __H_TSL2591
#define __H_TSL2591

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/iio/iio.h>

//I2C device address:
#define TSL2591_ADDR 0x29

#define TSL2591_DEVICE_ID_VALUE 0x50

#define TSL2591_COMMAND (0x1 << 7)

//TYPES OF COMMAND
#define TSL2591_ADDRESS 0x20
#define TSL2591_SPECIAL 0xC0

//ADDRESS REGISTER COMMANDS
#define TSL2591_ENABLE_RW  (TSL2591_COMMAND | TSL2591_ADDRESS | 0x00)
#define TSL2591_CONFIG_RW  (TSL2591_COMMAND | TSL2591_ADDRESS | 0x01)
//ALS Interrupt Threshold Register
#define TSL2591_AILTL_RW   (TSL2591_COMMAND | TSL2591_ADDRESS | 0x04)
#define TSL2591_AILTH_RW   (TSL2591_COMMAND | TSL2591_ADDRESS | 0x05)
#define TSL2591_AIHTL_RW   (TSL2591_COMMAND | TSL2591_ADDRESS | 0x06)
#define TSL2591_AIHTH_RW   (TSL2591_COMMAND | TSL2591_ADDRESS | 0x07)
#define TSL2591_NPAILTL_RW (TSL2591_COMMAND | TSL2591_ADDRESS | 0x08)
#define TSL2591_NPAILTH_RW (TSL2591_COMMAND | TSL2591_ADDRESS | 0x09)
#define TSL2591_NPAIHTL_RW (TSL2591_COMMAND | TSL2591_ADDRESS | 0x0A)
#define TSL2591_NPAIHTH_RW (TSL2591_COMMAND | TSL2591_ADDRESS | 0x0B)
#define TSL2591_PERSIST_RW (TSL2591_COMMAND | TSL2591_ADDRESS | 0x0C)
#define TSL2591_PID_R      (TSL2591_COMMAND | TSL2591_ADDRESS | 0x11)
#define TSL2591_ID_R       (TSL2591_COMMAND | TSL2591_ADDRESS | 0x12)
//ADC data registers, with extra convenience address fot the whole block.
#define TSL2591_STATUS_R   (TSL2591_COMMAND | TSL2591_ADDRESS | 0x13)
#define TSL2591_C0DATAL_R  (TSL2591_COMMAND | TSL2591_ADDRESS | 0x14)
#define TSL2591_C0DATAH_R  (TSL2591_COMMAND | TSL2591_ADDRESS | 0x15)
#define TSL2591_C1DATAL_R  (TSL2591_COMMAND | TSL2591_ADDRESS | 0x16)
#define TSL2591_C1DATAH_R  (TSL2591_COMMAND | TSL2591_ADDRESS | 0x17)
#define TSL2591_ADCDATA_R  (TSL2591_COMMAND | TSL2591_ADDRESS | 0x13)

//SPECIAL FUNCTION COMMANDS
#define TSL2591_SET_INT    (TSL2591_COMMAND | TSL2591_SPECIAL | 0x04)
#define TSL2591_CLR_ALSINT (TSL2591_COMMAND | TSL2591_SPECIAL | 0x06)
#define TSL2591_CLR_ALLINT (TSL2591_COMMAND | TSL2591_SPECIAL | 0x07)
#define TSL2591_CLR_NPINT  (TSL2591_COMMAND | TSL2591_SPECIAL | 0x0A)

//ENABLE REGISTER FIELDS
#define TSL2591_ENABLE_NPIEN (0x1 << 7)
#define TSL2591_ENABLE_SAI   (0x1 << 6)
#define TSL2591_ENABLE_AIEN  (0x1 << 4)
#define TSL2591_ENABLE_AEN   (0x1 << 1)
#define TSL2591_ENABLE_PON   (0x1 << 0)
#define TSL2591_ENABLE_OFF   0x00

//CONTROL REGISTER FIELDS
#define TSL2591_CONFIG_SRESET          (0x1          << 7)
#define TSL2591_CONFIG_SET_AGAIN(gain) ((gain&0x3)   << 4)
#define TSL2591_CONFIG_SET_ATIME(time) ((time/100-1) << 0)

//CONTROL REGISTER MASKS
#define TSL2591_CONFIG_AGAIN  (0x3 << 4) /*Two bits in the middle*/
#define TSL2591_CONFIG_ATIME  (0x7 << 0) /*Three bits at LSB*/

//STATUS REGISTER MASKS
#define TSL2591_STATUS_NPINT  (0x1 << 5)
#define TSL2591_STATUS_AINT   (0x1 << 4)
#define TSL2591_STATUS_AVALID (0x1 << 0)

//PERSIST REGISTER OPTIONS
#define TSL2591_APERS_1  0x0
#define TSL2591_APERS_2  0x1
#define TSL2591_APERS_3  0x2
#define TSL2591_APERS_5  0x3
#define TSL2591_APERS_10 0x4
#define TSL2591_APERS_15 0x5
#define TSL2591_APERS_20 0x6
#define TSL2591_APERS_25 0x7
#define TSL2591_APERS_30 0x8
#define TSL2591_APERS_35 0x9
#define TSL2591_APERS_40 0xA
#define TSL2591_APERS_45 0xB
#define TSL2591_APERS_50 0xC
#define TSL2591_APERS_55 0xD
#define TSL2591_APERS_60 0xF

//Persist register mask
#define TSL2591_APERS 0xF

#endif
