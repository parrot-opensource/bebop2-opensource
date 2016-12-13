/* tw8836.c
 *
 * Driver for Techwell tw8836
 * It handles firmware updater and composite input
 *
 * Author : Nicolas Laclau <nicolas.laclau@parrot.com>
 *
 * Date : 20 June 2013
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <media/mx25l6435e.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>

/* ----------------------------------------------------------------------- */
/* Registers Definition : */
/* Control registers */
#define TW8836_MODE_CTRL_REG            (0x4C2)
/* I²C to XMEM registers */
#define TW8836_I2C_XMEM_DMA_HIGH_ADDR   (0x4DB)
#define TW8836_I2C_XMEM_DMA_LOW_ADDR    (0x4DC)
#define TW8836_I2C_XMEM_DMA_DATA_REG    (0x4DD)
/* Registers addresses for MCU management */
#define TW8836_EN_MCU                   (0x4EC)
#define TW8836_ACCESS_REG               (0x4ED)
#define TW8836_CRC16_H_REG              (0x4EE)
#define TW8836_CRC16_L_REG              (0x4EF)
/* Registers addresses for I2C-DMA programming */
#define TW8836_DMA_CTRL_REG             (0X4F3)
#define TW8836_DMA_FLASH_BUSY_CTRL_REG  (0X4F4)
#define TW8836_PAGE_REG                 (0X4FF)

/* Others : */
#define TW8836_FIRMWARE_ADDR            (0x0)
#define TW8836_FIRMWARE_NAME            "tw8836.fw"

#define TW8836_XRAM_WORKAREA            (0x0400)
#define TW8836_MAX_CACHE                (256)

#define TW8836_MCU_MAGIC_PATTERN_1      (0x55)
#define TW8836_MCU_MAGIC_PATTERN_2      (0xAA)
#define TW8836_MCU_HALT                 (0x00)
#define TW8836_MCU_RERUN                (0x01)

/* SPI-EEPROM commands : */
#define TW8836_DMA_CMD_CHIP_ERASE       (MX25L6435E_CE_2)

/* Macro definitions: */
/* - Basic */
#define TW_PAGE_REG                (0xFF)
#define TW_BIT_SHIFT_R(val, shift) (val>>shift)
#define TW_BIT_SHIFT_L(val, shift) (val<<shift)
#define TW_MASK(bits)              (TW_BIT_SHIFT_L(1, bits)-1)
#define TW_SHIFT(val, shift, mask) (TW_BIT_SHIFT_R(val, shift) & TW_MASK(mask))

/* - Techwell addresses */
#define TW_PAGE(reg)               (TW_BIT_SHIFT_R(reg, 8) & TW_MASK(8))
#define TW_INDEX(reg)              (reg & TW_MASK(8))

/* - Others */
#define TW8836_CHECK_RETURN(ret_val) ({if (ret_val < 0) return ret_val; })

/* ----------------------------------------------------------------------- */
enum {
	TW8836_OUT_PAD = 0,
	TW8836_NUM_PADS,
};

struct tw8836 {
	struct i2c_client *client;
	const struct firmware *tw8836_fw;
	struct v4l2_subdev subdev;
	struct media_pad pad[TW8836_NUM_PADS];
	struct v4l2_mbus_framefmt format[TW8836_NUM_PADS];
};
/* ----------------------------------------------------------------------- */

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-2)");

/*
 * CRC table provided by Instersil
 * Used for computing CRC16 on firmware file and compare it with
 * the CRC16 given by tw8836 chip.
 */
static u16 crctab[256] = {
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
	0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
	0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

/* ----------------------------------------------------------------------- */
/* basic i2c methods */
s32 tw_i2c_read_byte(struct i2c_client *client, u8 reg)
{
	union i2c_smbus_data data;

	if (!i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			I2C_SMBUS_READ, reg,
			I2C_SMBUS_BYTE_DATA, &data))
		return data.byte;
	else
		return -EIO;
}

s32 tw_update_page(struct i2c_client *client, u16 reg)
{
	int res;
	u8 buf[2] = { TW_PAGE_REG, TW_PAGE(reg) };
	u8 value = 0;
	struct i2c_msg xfer[2] = {
	/* 1st message: Request Register page value */
	{ .addr = client->addr, .flags = 0, .len = 1, .buf = &buf[0], },
	/* 2nd message: Read requested value */
	{ .addr = client->addr, .flags = I2C_M_RD, .len = 1, .buf = &value } };

	/* Read register page set */
	res = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
	if (res != ARRAY_SIZE(xfer)) {
		v4l_err(client, "%s: i2c transfer failed (%d)\n",
			__func__, res);
		return -EIO;
	}

	/* if already on the wanted page, then returns (nothing to do) */
	if (value == buf[1])
		return 0;

	/* Set the new page */
	if (i2c_master_send(client, buf, ARRAY_SIZE(buf)) != ARRAY_SIZE(buf)) {
		v4l_err(client, "%s:%d: i2c send failed\n",
			__func__, __LINE__);
		return -EIO;
	}
	return 0;
}

/* ----------------------------------------------------------------------- */
/* Firmware update Methods */

/*
 * MCU control allow to start or stop tw8836's MCU
 * note: ctrl_word is either TW8836_MCU_HALT or TW8836_MCU_RERUN
 */
static s32 tw8836_mcu_ctrl(struct i2c_client *client, u8 ctrl_word)
{
	s32 res;
	char page[2] = {
		TW_INDEX(TW8836_PAGE_REG),
		4
	};
	char magic1[2] = {
		TW_INDEX(TW8836_ACCESS_REG),
		TW8836_MCU_MAGIC_PATTERN_1
	};
	char magic2[2] = {
		TW_INDEX(TW8836_ACCESS_REG),
		TW8836_MCU_MAGIC_PATTERN_2
	};
	char dma_ctlw[2] = {
		TW_INDEX(TW8836_EN_MCU),
		ctrl_word
	};
	struct i2c_msg xfer[4] = {
		{.addr = client->addr, .flags = 0, .len = 2, .buf = page},
		{.addr = client->addr, .flags = 0, .len = 2, .buf = magic1},
		{.addr = client->addr, .flags = 0, .len = 2, .buf = magic2},
		{.addr = client->addr, .flags = 0, .len = 2, .buf = dma_ctlw},
	};

	res = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
	if (res != ARRAY_SIZE(xfer)) {
		v4l_err(client, "%s: i2c transfer failed (%d)\n",
			__func__, res);
		return res;
	}
	return 0;
}

/*
 * tw8836_index_mode() setup index handling mode on tw8836 side :
 * 0x00 = auto increment
 * 0x20 = fixed (don't increment)
 */
static s32 tw8836_index_mode(struct i2c_client *client, u8 val)
{
	s32 res;
	u8 cmds[6] = {
		TW_INDEX(TW8836_PAGE_REG), 0x00, /* 0 - page 0 */
		0x06, val,
		TW_INDEX(TW8836_PAGE_REG), 0x04, /* 4 - page 4 */
	};

	struct i2c_msg xfer[3] = {
		{.addr = client->addr, .flags = 0, .len = 2, .buf = &cmds[0]},
		{.addr = client->addr, .flags = 0, .len = 2, .buf = &cmds[2]},
		{.addr = client->addr, .flags = 0, .len = 2, .buf = &cmds[4]},
	};

	res = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
	/* if read fails, then */
	if (res != ARRAY_SIZE(xfer)) {
		v4l_err(client, "%s: i2c transfer failed (%d)\n",
			__func__, res);
		return res;
	}
	return 0;
}

/*
 * tw8836_dma_cfg() method allow to configure tw8836's DMA
 */
static s32 tw8836_dma_cfg(struct i2c_client *client,
		u16 xram, u32 flash, u32 size, u8 crtl, u8 cmd, u8 busy)
{
	s32 res;
	u8 dma_busy[2] = { TW_INDEX(TW8836_DMA_FLASH_BUSY_CTRL_REG), busy };
	u8 buf_magic[2] = { 0x0A, 0x00 };
	u8 dma_cfg[13] = {
	/*INDEX*/TW_INDEX(TW8836_DMA_CTRL_REG),
	/*F3*/crtl,
	/*F4*/0x00,
	/*F5*/TW_SHIFT(size, 16, 8),
	/*F6*/TW_SHIFT(xram, 8, 8),
	/*F7*/TW_SHIFT(xram, 0, 8),
	/*F8*/TW_SHIFT(size, 8, 8),
	/*F9*/TW_SHIFT(size, 0, 8),
	/*FA*/cmd,
	/*FB*/TW_SHIFT(flash, 16, 8),
	/*FC*/TW_SHIFT(flash, 8, 8),
	/*FD*/TW_SHIFT(flash, 0, 8),
	/*FE*/0x00, };
	struct i2c_msg xfer[3] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = ARRAY_SIZE(dma_cfg),
			.buf = dma_cfg },
		{
			.addr = client->addr,
			.flags = 0,
			.len = ARRAY_SIZE(buf_magic),
			.buf = buf_magic },
		{
			.addr = client->addr,
			.flags = 0,
			.len = ARRAY_SIZE(dma_busy),
			.buf = dma_busy },
		};

	/* Configure DMA and start it */
	res = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
	if (res != ARRAY_SIZE(xfer)) {
		v4l_err(client, "%s: i2c transfer failed (%d)\n",
			__func__, res);
		return res;
	}
	return 0;
}

/*
 * tw8836_write_enable() method allow to command tw8836 to send WEN
 * to SPI-EEPROM (connected to tw8836)
 */
static s32 tw8836_write_enable(struct i2c_client *client)
{
	s32 res;

	v4l_dbg(1, debug, client, "%s send WREN\n", __func__);
	res = tw8836_dma_cfg(client, TW8836_XRAM_WORKAREA, 0, 0,
			0xC1, MX25L6435E_WREN, 0x03);
	TW8836_CHECK_RETURN(res);

	/* Wait until flashing block is over :
	 * read TW8836_DMA_FLASH_BUSY_CTRL_REG[0] until it's set to '0'
	 */
	do {
		res = tw_i2c_read_byte(
				client,
				(u8)TW8836_DMA_FLASH_BUSY_CTRL_REG);
		TW8836_CHECK_RETURN(res);

		v4l_dbg(1, debug, client,
			" %s, busy=0x%x (wait for reg[0] = 0)\n",
			__func__, res);
	} while ((res & 0x01) != 0);

	return 0;
}

/*
 * tw8836_xram_write() method allow to write data into tw8836's XRAM
 */
static s32 tw8836_xram_write(struct i2c_client *client,
		u8 *pData, u16 addr, u16 iLen)
{
	s32 res;
	u32 count = 1;
	u8 bit1;
	u8 buf_data[TW8836_MAX_CACHE + 1];
	u8 xram_high[2] = {
			TW_INDEX(TW8836_I2C_XMEM_DMA_HIGH_ADDR),
			TW_SHIFT(addr, 8, 8) };
	u8 xram_low[2] = {
			TW_INDEX(TW8836_I2C_XMEM_DMA_LOW_ADDR),
			TW_SHIFT(addr, 0, 8) };
	u8 ctrl_reg[2] = {
			TW_INDEX(TW8836_MODE_CTRL_REG),
			0x00 };
	u8 xram_access[2] = {
			TW_INDEX(TW8836_MODE_CTRL_REG),
			0x01 };
	u8 commands[6] = {
			TW_INDEX(TW8836_PAGE_REG), 0x00, /* 0 - page 0 */
			0x06, 0x20, /* index mode (0x00:auto-inc, 0x20:fix) */
			TW_INDEX(TW8836_PAGE_REG), 0x04, /* 4 - page 4 */
	};

	struct i2c_msg xfer[7] = {
	{.addr = client->addr, .flags = 0, .len = 2, .buf = xram_high},
	{.addr = client->addr, .flags = 0, .len = 2, .buf = xram_low},
	{.addr = client->addr, .flags = 0, .len = 2, .buf = buf_data},
	{.addr = client->addr, .flags = 0, .len = 2, .buf = ctrl_reg},
	{.addr = client->addr, .flags = 0, .len = 2, .buf = &commands[0]},
	{.addr = client->addr, .flags = 0, .len = 2, .buf = &commands[2]},
	{.addr = client->addr, .flags = 0, .len = 2, .buf = &commands[4]},
	};

	struct i2c_msg xfer_xram[6] = {
	{.addr = client->addr, .flags = 0, .len = 2, .buf = &commands[0]},
	{.addr = client->addr, .flags = 0, .len = 2, .buf = &commands[2]},
	{.addr = client->addr, .flags = 0, .len = 2, .buf = &commands[4]},
	{.addr = client->addr, .flags = 0, .len = 2, .buf = &xram_access[0]},
	{.addr = client->addr, .flags = 0, .len = 1, .buf = &xram_access[0]},
	{.addr = client->addr, .flags = I2C_M_RD, .len = 1, .buf = &bit1 },
	};

	v4l_dbg(1, debug, client, " %s enter\n", __func__);
	v4l_dbg(1, debug, client, " %s access to xram...\n", __func__);

	res = i2c_transfer(client->adapter, xfer_xram, ARRAY_SIZE(xfer_xram));
	if (res != ARRAY_SIZE(xfer_xram)) {
		v4l_err(client, "%s: i2c transfer failed (%d)\n",
			__func__, res);
		return res;
	}

	/* Wait until I2C can access to XMEM:
	 * read TW8836_MODE_CTRL_REG[1] until it's set
	 */
	res = 0;
	while ((res & 0x02) == 0) {
		v4l_dbg(1, debug, client,
				" wait XRAM ( read=0x%x, wait until != 0)\n",
				res);
		res = tw_i2c_read_byte(client, (u8) TW8836_MODE_CTRL_REG);
		TW8836_CHECK_RETURN(res);
	}
	v4l_dbg(1, debug, client, " %s ... done\n", __func__);

	/* fix mode */
	v4l_dbg(1, debug, client, " %s set fix mode...\n", __func__);
	tw8836_index_mode(client, 0x20);
	v4l_dbg(1, debug, client, " %s ... done\n", __func__);

	/* Prepare buffer to send over i2c */
	v4l_dbg(1, debug, client, " %s prepare buffer to send...\n", __func__);
	buf_data[0] = TW_INDEX(TW8836_I2C_XMEM_DMA_DATA_REG);
	count = 1;
	while ((count < (TW8836_MAX_CACHE + 1)) && (count < (iLen + 1))) {
		buf_data[count] = pData[count - 1];
		count++;
	}
	xfer[2].len = count;
	v4l_dbg(1, debug, client, " %s ... done\n", __func__);

	v4l_dbg(1, debug, client, " %s send %d bytes...\n", __func__, count);
	commands[3] = 0;

	res = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
	if (res != ARRAY_SIZE(xfer)) {
		v4l_err(client, "%s: i2c transfer failed (%d)\n",
			__func__, res);
		return res;
	}

	v4l_dbg(1, debug, client, " %s ...done\n", __func__);

	/* auto inc */
	v4l_dbg(1, debug, client, " %s set to auto inc index\n", __func__);
	tw8836_index_mode(client, 0x00);

	v4l_dbg(1, debug, client, " %s leave\n", __func__);
	return 0;
}

/*
 * tw8836_program_by_dma() allow to flash data block
 * into SPI-EEPROM connected to tw8836.
 */
static s32 tw8836_program_by_dma(struct i2c_client *client,
		u16 src, u32 dest, u32 iLen)
{
	s32 res;

	v4l_dbg(1, debug, client, " configure dma...\n %s send PP\n",
		__func__);

	res = tw8836_dma_cfg(client, TW8836_XRAM_WORKAREA, dest, iLen,
			0xC4, MX25L6435E_PP, 0x07);
	TW8836_CHECK_RETURN(res);

	v4l_dbg(1, debug, client, " dma started !\n");
	/*
	 * Wait until flashing block is over :
	 * read TW8836_DMA_FLASH_BUSY_CTRL_REG[0] until it's set to '0'
	 */
	do {
		res = tw_i2c_read_byte(client,
				(u8) TW8836_DMA_FLASH_BUSY_CTRL_REG);
		TW8836_CHECK_RETURN(res);
		v4l_dbg(1, debug, client,
			" wait flash done (value:0x%x, expected bit0 = 0)\n",
			res);
	} while ((res & 0x01) != 0);

	return 0;
}

/*
 * tw8836_erase() method allow to erase SPI-EEPROM connected to tw8836.
 */
static s32 tw8836_erase(struct i2c_client *client, u8 erase_cmd, u32 addr)
{
	s32 res;

	v4l_dbg(1, debug, client, " %s enter\n", __func__);

	/* auto inc */
	res = tw8836_index_mode(client, 0x00);
	TW8836_CHECK_RETURN(res);

	/* WREN command */
	res = tw8836_write_enable(client);

	/* Chip erase command */
	v4l_dbg(1, debug, client, " %s send Chip erase\n", __func__);
	res = tw8836_dma_cfg(client, TW8836_XRAM_WORKAREA, 0, 0,
			0xC1, MX25L6435E_CE_2, 0x07);
	TW8836_CHECK_RETURN(res);

	/* Wait until flashing block is over :
	 * read TW8836_DMA_FLASH_BUSY_CTRL_REG[0] until it's set to '0'
	 */
	do {
		res = tw_i2c_read_byte(client,
				(u8) TW8836_DMA_FLASH_BUSY_CTRL_REG);
		TW8836_CHECK_RETURN(res);
		v4l_dbg(1, debug, client,
			" %s wait for complete erase "
			"(value:0x%x, expected bit0: 0)\n",
			__func__, res);
	} while ((res & 0x01) != 0);

	return 0;
}

/*
 * tw8836_crc_check() method allow to retrieve crc16 computed by tw8836
 * on SPI-EEPROM data range.
 */
static s32 tw8836_crc_check(struct i2c_client *client,
		u32 addr, u32 size, u16 *crc16)
{
	s32 res;
	u8 crc_msb, crc_lsb;
	u8 reg_crc_h = TW_INDEX(TW8836_CRC16_H_REG);
	u8 reg_crc_l = TW_INDEX(TW8836_CRC16_L_REG);

	struct i2c_msg xfer_crc[4] = {
	{.addr = client->addr, .flags = 0, .len = 1, .buf = &reg_crc_h},
	{.addr = client->addr, .flags = I2C_M_RD, .len = 1, .buf = &crc_msb},
	{.addr = client->addr, .flags = 0, .len = 1, .buf = &reg_crc_l },
	{.addr = client->addr, .flags = I2C_M_RD, .len = 1, .buf = &crc_lsb },
	};

	v4l_dbg(1, debug, client, " %s enter\n", __func__);

	/* auto inc */
	tw8836_index_mode(client, 0x00);

	/* configure dma and start it */
	res = tw8836_dma_cfg(client, TW8836_XRAM_WORKAREA, addr, size,
			0xE4, MX25L6435E_READ, 0x05);
	TW8836_CHECK_RETURN(res);

	/* Wait until read is finished :
	 * Wait until TW8836_DMA_FLASH_BUSY_CTRL_REG[0] is set to '0' */
	do {
		res = tw_i2c_read_byte(client,
				(u8) TW8836_DMA_FLASH_BUSY_CTRL_REG);
		TW8836_CHECK_RETURN(res);
		v4l_dbg(1, debug, client, " %s wait for busy done ! (0x%x)\n",
			__func__, res);
	} while ((res & 0x01) != 0);

	/* Read CRC result*/
	res = i2c_transfer(client->adapter, xfer_crc, ARRAY_SIZE(xfer_crc));
	if (res != ARRAY_SIZE(xfer_crc)) {
		v4l_err(client, "%s: i2c transfer failed (%d)\n",
			__func__, res);
		return res;
	}

	(*crc16) = ((crc_msb << 8) & 0xFF00);
	(*crc16) |= ((crc_lsb) & 0x00FF);

	v4l_info(client, " %s leave with 0x%x\n", __func__, *crc16);

	/* return crc16_val */
	return 0;
}

/* ----------------------------------------------------------------------- */
/*
 * ISP (In System Programming) through I2C of TW8836:
 *
 * TW8836 does not have an internal ROM code But it provides I2C DMA
 * for SPI flash programming.
 * Here is the sequence to use for programming external flash :
 *
 * MCU Halt                                                         MCU Rerun
 *   |                                                                 |
 *   +->SPI Erase-+->XRAM Write(256B max)-->I²C SPI DMA-+->CRC16 check-+
 *                |                                     |
 *                +-<---<---<---<---<---<---<---<---<---+
 *                Loop for each 256 Bytes block to flash.
 *
 * - 'MCU Halt' done through tw8836_mcu_ctrl(TW8836_MCU_HALT) call.
 * - 'SPI Erase' done through tw8836_erase(TW8836_DMA_CMD_CHIP_ERASE) call.
 * - 'XRAM Write' done through tw8836_xram_write() with data up to 256 bytes.
 * - 'I²C SPI DMA' done through tw8836_program_by_dma() with data length
 *    given in 'XRAM Write'.
 * - 'CRC16 check' done through tw8836_crc_check() call.
 * - 'MCU Rerun' done through tw8836_mcu_ctrl(TW8836_MCU_RERUN) call.
 */
static int tw8836_upgrade_firmware(struct tw8836 *data)
{
	int error, idx, remain, size;
	u16 crc16_in_chip = 0;

	v4l_info(data->client, "Upgrade firmware");

	error = tw8836_mcu_ctrl(data->client, (u8) TW8836_MCU_HALT);
	TW8836_CHECK_RETURN(error);

	error = tw8836_erase(data->client, (u8) TW8836_DMA_CMD_CHIP_ERASE, 0);
	TW8836_CHECK_RETURN(error);

	v4l_dbg(1, debug, data->client, " firmware length %d (0x%x)\n",
			data->tw8836_fw->size, data->tw8836_fw->size);

	for (idx = 0; idx < data->tw8836_fw->size; idx += TW8836_MAX_CACHE) {
		remain = data->tw8836_fw->size - idx;
		if (remain < TW8836_MAX_CACHE)
			size = remain;
		else
			size = TW8836_MAX_CACHE;

		/* Write 256 max bytes in XRAM (XDATA memory) */
		error = tw8836_xram_write(data->client,
				(u8 *) (data->tw8836_fw->data + idx),
				TW8836_XRAM_WORKAREA, size);
		TW8836_CHECK_RETURN(error);

		error = tw8836_write_enable(data->client);
		TW8836_CHECK_RETURN(error);

		/* Proceed DMA transfer data block from XRAM to SPI-Flash */
		error = tw8836_program_by_dma(data->client,
		TW8836_XRAM_WORKAREA,
		TW8836_FIRMWARE_ADDR + idx, size);
		TW8836_CHECK_RETURN(error);

		v4l_dbg(1, debug, data->client, " progression = %d %%\n",
				((idx * 100) / data->tw8836_fw->size));
	}

	error = tw8836_crc_check(data->client,
			TW8836_FIRMWARE_ADDR,
			data->tw8836_fw->size,
			&crc16_in_chip);
	TW8836_CHECK_RETURN(error);

	error = tw8836_mcu_ctrl(data->client, TW8836_MCU_RERUN);
	TW8836_CHECK_RETURN(error);

	return 0;
}

static s32 tw8836_initialize(struct tw8836 *data)
{
	s32 error = 0;
	u16 iCRC16_inChip = 0, iCRC16_fw = 0, count = 0, index = 0;
	struct device *dev = &data->client->dev;

	/* Get firmware */
	error = request_firmware(&data->tw8836_fw, TW8836_FIRMWARE_NAME, dev);
	if (error < 0) {
		v4l_err(data->client, "Failed to request firmware %s\n",
			TW8836_FIRMWARE_NAME);
		return error;
	}

	/* Check firmware CRC16 */
	error = tw_update_page(data->client, TW8836_CRC16_H_REG);
	if (error != 0)
		goto rlse_firmware;

	error = tw8836_crc_check(data->client, TW8836_FIRMWARE_ADDR,
			data->tw8836_fw->size, &iCRC16_inChip);
	if (error != 0)
		goto rlse_firmware;

	v4l_dbg(1, debug, data->client, " Compute firmware CRC16 ...\n");
	/* Compute CRC on firmware (given by Intersil) */
	count = data->tw8836_fw->size;
	while (count--) {
		iCRC16_fw =
			(iCRC16_fw << 8) ^
			crctab[(iCRC16_fw >> 8) ^
			data->tw8836_fw->data[index++]];
	}
	v4l_dbg(1, debug, data->client, " ... Done !\n");

	/* if different CRC, then upgrade is needed */
	if (iCRC16_inChip != iCRC16_fw) {
		v4l_err(data->client,
			"CRC16 in chips : 0x%04x\tCRC16 firmware : 0x%04x\n",
			iCRC16_inChip, iCRC16_fw);
		error = tw8836_upgrade_firmware(data);
	} else {
		v4l_info(data->client, "firmware already up-to-date (0x%04x)",
			iCRC16_inChip);
	}

rlse_firmware:
	release_firmware(data->tw8836_fw);
	return error;
}

/* ----------------------------------------------------------------------- */
/* V4L2 dummy ops */
static int tw8836_dummy_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int tw8836_g_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_frame_interval *fi)
{
	if (!fi)
		return -EINVAL;

	fi->interval.numerator = 1;
	fi->interval.denominator = 30;
	return 0;
}

static struct v4l2_mbus_framefmt *
__tw8836_get_fmt(struct tw8836 *tw8836, struct v4l2_subdev_fh *fh,
		unsigned int pad, enum v4l2_subdev_format_whence which)
{
	if (pad > TW8836_NUM_PADS)
		return NULL;

	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(fh, pad);
	else
		return &tw8836->format[pad];
}

static int tw8836_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *fmt)
{

	struct tw8836 *tw8836 = container_of(sd, struct tw8836, subdev);
	struct v4l2_mbus_framefmt *format;

	format = __tw8836_get_fmt(tw8836, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;
	return 0;
}

/* dummy set format */
static int tw8836_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *fmt)
{

	struct tw8836 *tw8836 = container_of(sd, struct tw8836, subdev);
	struct v4l2_mbus_framefmt *format;

	format = __tw8836_get_fmt(tw8836, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	*format = fmt->format;
	return 0;
}

static int tw8836_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_fh *fh,
		struct v4l2_subdev_mbus_code_enum *code)
{
	struct tw8836 *tw8836 = container_of(sd, struct tw8836, subdev);

	if (code->index)
		return -EINVAL;

	code->code = tw8836->format[code->pad].code;
	return 0;
}

static struct v4l2_subdev_core_ops tw8836_core_ops;

static struct v4l2_subdev_video_ops tw8836_video_ops = {
	.s_stream = tw8836_dummy_s_stream,
	.g_frame_interval = tw8836_g_frame_interval,
};

static struct v4l2_subdev_pad_ops tw8836_pad_ops = {
	.get_fmt = tw8836_get_fmt,
	.set_fmt = tw8836_set_fmt,
	.enum_mbus_code = tw8836_enum_mbus_code,
};

static const struct v4l2_subdev_ops tw8836_ops = {
	.core = &tw8836_core_ops,
	.video = &tw8836_video_ops,
	.pad = &tw8836_pad_ops,
};

/* ----------------------------------------------------------------------- */
static int __devinit tw8836_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct tw8836 *tw8836;
	struct v4l2_subdev *sd;
	struct v4l2_mbus_framefmt *format;
	struct v4l2_mbus_framefmt *pdata = client->dev.platform_data;
	int ret;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		ret = -EIO;
		goto ei2c;
	}

	/* allocate context memory */
	tw8836 = kzalloc(sizeof(*tw8836), GFP_KERNEL);
	if (!tw8836) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto enomem;
	}

	/* Keep client in context */
	tw8836->client = client;

	/* Initialize tw8836 driver */
	ret = tw8836_initialize(tw8836);
	if (ret != 0)
		goto efwfail;

	/* initialize V4L2 sub device */
	format = &tw8836->format[TW8836_OUT_PAD];
	if (pdata) {
		format->width = pdata->width;
		format->height = pdata->height;
		format->field = pdata->field;
		format->code = pdata->code;
		format->colorspace = pdata->colorspace;
	} else {
		format->width = 720;
		format->height = 480;
		format->field = V4L2_FIELD_INTERLACED;
		format->code = V4L2_MBUS_FMT_UYVY8_2X8;
		format->colorspace = V4L2_COLORSPACE_SMPTE170M;
	}
	sd = &tw8836->subdev;
	v4l2_i2c_subdev_init(sd, client, &tw8836_ops);

	/* create node */
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	/* pad supports sourcing data */
	tw8836->pad[TW8836_OUT_PAD].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_init(&sd->entity, TW8836_NUM_PADS, tw8836->pad, 0);
	if (ret < 0) {
		v4l_err(client, "failed to init media entity\n");
		goto emedia;
	}

	v4l_info(client, "entity initialized, dummy\n");
	return ret;

emedia:
efwfail:
	kfree(tw8836);
enomem:
ei2c:
	return ret;
}

static int __devexit tw8836_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tw8836 *tw8836 = container_of(sd, struct tw8836, subdev);

	v4l2_device_unregister_subdev(&tw8836->subdev);
	media_entity_cleanup(&tw8836->subdev.entity);
	kfree(tw8836);
	return 0;
}

/* ----------------------------------------------------------------------- */
static struct i2c_device_id tw8836_id[] = { { "tw8836", 0 }, { } };

MODULE_DEVICE_TABLE(i2c, tw8836_id);

static struct i2c_driver tw8836_driver = { .driver = { .owner = THIS_MODULE,
		.name = "tw8836", }, .id_table = tw8836_id, .probe =
		tw8836_probe, .remove = tw8836_remove, };

module_i2c_driver(tw8836_driver);

MODULE_AUTHOR("Nicolas Laclau <nicolas.laclau@parrot.com>");
MODULE_DESCRIPTION("Techwell 8836 driver");
MODULE_LICENSE("GPL");
