/* mt9v117.c
 *
 * Driver for aptina mt9v117
 *
 * Author : Julien BERAUD <julien.beraud@parrot.com>
 *
 * Date : 23/04/2014
 *
 */
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/gcd.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/delay.h>
#include <media/mt9v117.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include "mt9v117_patches.h"

MODULE_AUTHOR("Julien BERAUD <julien.beraud@parrot.com>");
MODULE_DESCRIPTION("kernel driver for mt9v117");
MODULE_LICENSE("GPL");

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-2)");

#define MT9V117_frameperiod_ns_DEF	36353853 //10000÷275074 ~= 27Fps
#define MIN_HORIZONTAL_BLANKING		210
#define MIN_VERTICAL_BLANKING		31

/* mt9v117 data structure */
struct mt9v117 {
	struct v4l2_subdev 		sd;
	struct i2c_client 		*i2c_client;
	struct media_pad 		pad;
	struct v4l2_mbus_framefmt 	format;

	struct  v4l2_ctrl_handler 	ctrls;
	struct 	v4l2_ctrl 		*gains[4];

	int 				input;
	struct mt9v117_platform_data 	*pdata;
	uint32_t 			frameperiod_ns;		/* user desired frame rate */
	uint32_t 			real_frameperiod_ns;	/* real framerate */
	uint16_t 			line_length;
	uint16_t 			frame_length;

	int 				isStreaming;
	bool				timings_uptodate;
	int (*set_power)(int on);
};

static inline struct mt9v117 *to_mt9v117(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9v117, sd);
}

/* write a register */
static int mt9v117_write8(struct mt9v117 *mt9v117, u16 reg, u8 val)
{
	struct i2c_client *client = mt9v117->i2c_client;
	struct i2c_msg msg;
	struct {
		u16 reg;
		u8 val;
	} __packed buf;
	int ret;

	reg = swab16(reg);

	buf.reg = reg;
	buf.val = val;

	msg.addr	= client->addr;
	msg.flags	= 0;
	msg.len		= 3;
	msg.buf		= (u8 *)&buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

/* read a register */
static int mt9v117_read16(struct mt9v117 *mt9v117, u16 reg, u16 *val)
{
	struct i2c_client *client = mt9v117->i2c_client;
	int ret;
	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= (u8 *)&reg,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 2,
			.buf	= (u8 *)val,
		},
	};

	reg = swab16(reg);

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "Failed reading register 0x%04x!\n", reg);
		return ret;
	}

	*val = swab16(*val);

	return 0;
}

/* write a register */
static int mt9v117_write16(struct mt9v117 *mt9v117, u16 reg, u16 val)
{
	struct i2c_client *client = mt9v117->i2c_client;
	struct i2c_msg msg;
	struct {
		u16 reg;
		u16 val;
	} __packed buf;
	int ret;

	buf.reg = swab16(reg);
	buf.val = swab16(val);

	msg.addr	= client->addr;
	msg.flags	= 0;
	msg.len		= 4;
	msg.buf		= (u8 *)&buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

/* write a register */
static int mt9v117_write32(struct mt9v117 *mt9v117, u16 reg, u32 val)
{
	struct i2c_client *client = mt9v117->i2c_client;
	struct i2c_msg msg;
	struct {
		u16 reg;
		u32 val;
	} __packed buf;
	int ret;

	buf.reg = swab16(reg);
	buf.val = swab32(val);

	msg.addr	= client->addr;
	msg.flags	= 0;
	msg.len		= 6;
	msg.buf		= (u8 *)&buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to write register 0x%04x!\n",
				reg);
		return ret;
	}

	return 0;
}

static int mt9v117_write_var8(struct mt9v117 *mt9v117, u16 var,
			      u16 offset, u16 value)
{
	struct i2c_client *client = mt9v117->i2c_client;
	u16 addr = 0x8000 | (var << 10) | offset;

	if(var > 0x200) {
		dev_err(&client->dev, "write var offset too high\n");
		return -EINVAL;
	} else
		return mt9v117_write8(mt9v117, addr, value);
}

static int mt9v117_read_var16(struct mt9v117 *mt9v117, u16 var,
			      u16 offset, u16 *value)
{
	struct i2c_client *client = mt9v117->i2c_client;
	u16 addr = 0x8000 | (var << 10) | offset;

	if(var > 0x200) {
		dev_err(&client->dev, "read var offset too high\n");
		return -EINVAL;
	} else
		return mt9v117_read16(mt9v117, addr, value);
}

static int mt9v117_write_var16(struct mt9v117 *mt9v117, u16 var,
			       u16 offset, u16 value)
{
	struct i2c_client *client = mt9v117->i2c_client;
	u16 addr = 0x8000 | (var << 10) | offset;

	if(var > 0x200) {
		dev_err(&client->dev, "write var offset too high\n");
		return -EINVAL;
	} else
		return mt9v117_write16(mt9v117, addr, value);
}

static int mt9v117_write_var32(struct mt9v117 *mt9v117, u16 var,
			       u16 offset, u32 value)
{
	struct i2c_client *client = mt9v117->i2c_client;
	u16 addr = 0x8000 | (var << 10) | offset;

	if(var > 0x200) {
		dev_err(&client->dev, "write var offset too high\n");
		return -EINVAL;
	} else
		return mt9v117_write32(mt9v117, addr, value);
}

static int mt9v117_write_patch(struct mt9v117 *mt9v117,
			       u8 *patch, u16 patch_len)
{
	struct i2c_client *client = mt9v117->i2c_client;
	struct i2c_msg msg;
	int ret;

	msg.addr	= client->addr;
	msg.flags	= 0;
	msg.len		= patch_len;
	msg.buf		= patch;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to write patch\n");
		return ret;
	}

	return 0;
}

/* setup functions */
static int mt9v117_config_change(struct mt9v117 *mt9v117)
{
	u16 value;

	mt9v117_write_var8(mt9v117, 23, 0x0000, 0x28);//SYSMGR_NEXT_STATE
	mt9v117_write16(mt9v117,  0x0040, 0x8002);//COMMAND_REGISTER
	msleep(500);
	mt9v117_read16(mt9v117, 0x0040, &value);

	if ((value & 0x0002) != 0)
		v4l2_err(&mt9v117->sd,"command not complete...\n");

	msleep(500);
	mt9v117_read16(mt9v117, 0x0040, &value);

	if ((value & 0x8000) == 0) {
		v4l2_err(&mt9v117->sd,"config change failed...%x\n",value);
		return -EPIPE;
	}

	return 0;
}

static int mt9v117_soft_reset(struct mt9v117 *mt9v117)
{
	if (mt9v117_write16(mt9v117, 0x001A, 0x0001) < 0)
		return -EPIPE;

	if (mt9v117_write16(mt9v117, 0x001A, 0x0000) < 0)
		return -EPIPE;

	msleep (50);

	return 0;
}

static int mt9v117_patch1(struct mt9v117 *mt9v117)
{
	u16 value;

	// correct image aberration due to low illumination conditions
	// Errata item 2
	mt9v117_write16(mt9v117, 0x301A, 0x10D0);
	mt9v117_write16(mt9v117, 0x31C0, 0x1404);
	mt9v117_write16(mt9v117, 0x3ED8, 0x879C);
	mt9v117_write16(mt9v117, 0x3042, 0x20E1);
	mt9v117_write16(mt9v117, 0x30D4, 0x8020);
	mt9v117_write16(mt9v117, 0x30C0, 0x0026);
	mt9v117_write16(mt9v117, 0x301A, 0x10D4);

	// Errata item 6
	mt9v117_write_var16(mt9v117, 10, 0x0002, 0x00D3);
	mt9v117_write_var16(mt9v117, 18, 0x0078, 0x00A0);
	mt9v117_write_var16(mt9v117, 18, 0x0076, 0x0140);

	// Errata item 8
	mt9v117_write_var16(mt9v117, 15, 0x0004, 0x00FC);
	mt9v117_write_var16(mt9v117, 15, 0x0038, 0x007F);
	mt9v117_write_var16(mt9v117, 15, 0x003A, 0x007F);
	mt9v117_write_var16(mt9v117, 15, 0x003C, 0x007F);
	mt9v117_write_var16(mt9v117, 15, 0x0004, 0x00F4);

	// Load Patch 0403
	mt9v117_write16(mt9v117,  0x0982, 0x0001);	//ACCESS_CTL_STAT
	mt9v117_write16(mt9v117,  0x098A, 0x7000);	// PHYSICAL_ADDRESS_ACCESS

	mt9v117_write_patch(mt9v117,patch_line1, sizeof(patch_line1));
	mt9v117_write_patch(mt9v117,patch_line2, sizeof(patch_line2));
	mt9v117_write_patch(mt9v117,patch_line3, sizeof(patch_line3));
	mt9v117_write_patch(mt9v117,patch_line4, sizeof(patch_line4));
	mt9v117_write_patch(mt9v117,patch_line5, sizeof(patch_line5));
	mt9v117_write_patch(mt9v117,patch_line6, sizeof(patch_line6));
	mt9v117_write_patch(mt9v117,patch_line7, sizeof(patch_line7));
	mt9v117_write_patch(mt9v117,patch_line8, sizeof(patch_line8));
	mt9v117_write_patch(mt9v117,patch_line9, sizeof(patch_line9));
	mt9v117_write_patch(mt9v117,patch_line10, sizeof(patch_line10));
	mt9v117_write_patch(mt9v117,patch_line11, sizeof(patch_line11));
	mt9v117_write_patch(mt9v117,patch_line12, sizeof(patch_line12));
	mt9v117_write_patch(mt9v117,patch_line13, sizeof(patch_line13));
	mt9v117_write16(mt9v117,  0x098E, 0x0000);	//LOGICAL_ADDRESS_ACCESS

	// apply patch 0403
	mt9v117_write_var16(mt9v117, 0x18, 0x0000, 0x05d8); //PATCHLDR_LOADER_ADDRESS
	mt9v117_write_var16(mt9v117, 0x18, 0x0002, 0x0403); // PATCHLDR_PATCH_ID
	mt9v117_write_var32(mt9v117, 0x18, 0x0004, 0x00430104); // PATCHLDR_FIRMWARE_ID doute


	mt9v117_write16(mt9v117,  0x0040, 0x8001);				//apply patch
	msleep(500);
	mt9v117_read16(mt9v117, 0x0040, &value);
	if ((value & 0x8001) != 0x8000) {
		v4l2_err(&mt9v117->sd,"mt9v117 patch apply failed...%x\n",value);
		return -ENODEV;
	}

	return 0;
}


static int mt9v117_set_basic_settings(struct mt9v117 *mt9v117)
{
	u32 ths = 50000;
	u16 value;

	//awb_pixel_threshold_count
	mt9v117_write_var32(mt9v117,11,0x0040,ths);

	// set ae_rule_algo to average
	mt9v117_write_var16(mt9v117,9,0x4,0x0);

	// pad slew more agressive
	mt9v117_read16(mt9v117, 0x0030, &value);
	value |= 0x0001; // data slew rate = 1  (default value 0)
	value |= 0x0600; // clock slew rate = 6 (default value 4)
	mt9v117_write16(mt9v117, 0x0030, value);

	return 0;
}

static int mt9v117_update_timings(struct mt9v117 *mt9v117)
{
	struct v4l2_mbus_framefmt *fmt = &mt9v117->format;

	// find the best couple Line Length&frame_length to reach desired framerate
	uint32_t lineLenght, frameLength;
	uint32_t bestLineLenght = 1;
	uint32_t bestFrameLength = 1;
	int64_t error, bestError = -1;
	uint64_t tmp64;
	uint32_t frameperiod_ns;
	uint32_t bestFramePeriod = 0;
	uint32_t pixelClock = mt9v117->pdata->ext_clk_freq_hz / 2;

	for (lineLenght = 640 + MIN_HORIZONTAL_BLANKING; lineLenght < 2048; lineLenght++) {
		for (frameLength = fmt->height + MIN_VERTICAL_BLANKING; frameLength < 600; frameLength++) {
			// compute framerate
			tmp64 = (u64)lineLenght * frameLength * NSEC_PER_SEC;
			frameperiod_ns = div_u64((u64)tmp64, pixelClock);
			// compute error
			error = abs64((u64)frameperiod_ns - mt9v117->frameperiod_ns);
			if (((error < bestError) || (bestError < 0))) {
				bestError = error;
				bestLineLenght = lineLenght;
				bestFrameLength = frameLength;
				bestFramePeriod = frameperiod_ns;
			}
			if (frameperiod_ns > mt9v117->frameperiod_ns)
				break;
		}
	}

	/* check a solution has been found */
	if (bestError < 0) {
		v4l2_err(&mt9v117->sd, "error setting framerate\n");
		return -1;
	}

	mt9v117->real_frameperiod_ns = bestFramePeriod;
	mt9v117->line_length = bestLineLenght;
	mt9v117->frame_length = bestFrameLength;

	v4l2_info(&mt9v117->sd, "pixelClock %u\n", mt9v117->pdata->ext_clk_freq_hz);
	v4l2_info(&mt9v117->sd, "setting framerate: frame period %u->%u, line length %u, frame length %u\n", mt9v117->frameperiod_ns, bestFramePeriod, bestLineLenght, bestFrameLength);

	mt9v117->timings_uptodate = 1;

	return 0;
}

static int mt9v117_blanking_write(struct mt9v117 *mt9v117)
{
	uint32_t pixelClock = mt9v117->pdata->ext_clk_freq_hz / 2;
	uint64_t tmp64;
	uint32_t flicker_periods;

	mt9v117_write_var16(mt9v117, 18, 0x10, mt9v117->line_length);
	mt9v117_write_var16(mt9v117, 18, 0xE, mt9v117->frame_length);

	// configure FD zone
	mt9v117_write_var16(mt9v117, 18, 0x16,
			pixelClock / mt9v117->line_length / 120); //FD Period 60Hz
	mt9v117_write_var16(mt9v117, 18, 0x18,
			pixelClock / mt9v117->line_length / 100); //FD Period 50Hz

	tmp64 =  ((u64)mt9v117->real_frameperiod_ns * 120);
	flicker_periods = div_u64(tmp64, NSEC_PER_SEC);
	mt9v117_write_var16(mt9v117, 18, 0x1A,
			flicker_periods); //Max FD Zone 60Hz
	mt9v117_write_var16(mt9v117, 18, 0x1E,
			flicker_periods); //Target FD Zone 60Hz

	tmp64 =  ((u64)mt9v117->real_frameperiod_ns * 100);
	flicker_periods = div_u64(tmp64, NSEC_PER_SEC);
	mt9v117_write_var16(mt9v117, 18, 0x1C,
			 flicker_periods); //Max FD Zone 50Hz
	mt9v117_write_var16(mt9v117, 18, 0x20,
			flicker_periods); //Target FD Zone 50Hz

	return 0;
}

static int mt9v117_set_VGA(struct mt9v117 *mt9v117)
{
	// QVGA, NO Y-skipping, framerate up to 35 FPS
	mt9v117_write_var16(mt9v117,  18, 0x0, 0x000C);	//Row Start = 12
	mt9v117_write_var16(mt9v117,  18, 0x2, 0x0010);	//Column Start = 16
	mt9v117_write_var16(mt9v117,  18, 0x4, 0x01F3);	//Row End = 499
	mt9v117_write_var16(mt9v117,  18, 0x6, 0x0297);	//Column End = 663
	mt9v117_write_var16(mt9v117,  18, 0x8, 0x0111);	//Row Speed = 273
	mt9v117_write_var16(mt9v117,  18, 0xA, 0x00A4);	//Sensor_fine_IT_min = 164
	mt9v117_write_var16(mt9v117,  18, 0xC, 0x02FA);	//Sensor_fine_IT_max = 762
	mt9v117_write_var16(mt9v117,  18, 0x12, 0x0031);//Sensor_fine_correction = 49
	mt9v117_write_var16(mt9v117,  18, 0x14, 0x01E3);//Cpipe Last Row = 483
	mt9v117_write_var16(mt9v117,  18, 0x28, 0x0000);//Read Mode = 3 - Mirror + Flip
	mt9v117_write_var16(mt9v117,  18, 0x4C, 0x0280);//Crop Width = 640
	mt9v117_write_var16(mt9v117,  18, 0x4E, 0x01E0);//Crop Height = 480
	mt9v117_write_var8(mt9v117,  18, 0x50, 0x03);	//Crop Mode = 3
	mt9v117_write_var16(mt9v117,  18, 0x54, 0x0280);//Output Width = 640
	mt9v117_write_var16(mt9v117,  18, 0x56, 0x01E0);//Output Height = 480
	mt9v117_write_var16(mt9v117,  18, 0xEC, 0x0000);//AWB Window Xstart = 0
	mt9v117_write_var16(mt9v117,  18, 0xEE, 0x0000);//AWB Window Ystart = 0
	mt9v117_write_var16(mt9v117,  18, 0xF0, 0x027F);//AWB Window Xend = 639
	mt9v117_write_var16(mt9v117,  18, 0xF2, 0x01DF);//AWB Window Yend = 479
	mt9v117_write_var16(mt9v117,  18, 0xF4, 0x0002);//AE Window Xstart = 2
	mt9v117_write_var16(mt9v117,  18, 0xF6, 0x0002);//AE Window Ystart = 2
	mt9v117_write_var16(mt9v117,  18, 0xF8, 0x007F);//AE Window Xend = 127
	mt9v117_write_var16(mt9v117,  18, 0xFA, 0x005F);//AE Window Yend = 95

	return 0;
}

static int mt9v117_set_QVGA(struct mt9v117 *mt9v117)
{
	// QVGA, Y-skipping, framerate up to 60 FPS
	mt9v117_write_var16(mt9v117, 18, 0x0, 0x0008);	//Row Start = 8
	mt9v117_write_var16(mt9v117, 18, 0x2, 0x0010);	//Column Start = 16
	mt9v117_write_var16(mt9v117, 18, 0x4, 0x01F5);	//Row End = 501
	mt9v117_write_var16(mt9v117, 18, 0x6, 0x0297);	//Column End = 663
	mt9v117_write_var16(mt9v117, 18, 0x8, 0x0111);	//Row Speed = 273
	mt9v117_write_var16(mt9v117, 18, 0xA, 0x00A4);	//Sensor_fine_IT_min = 164
	mt9v117_write_var16(mt9v117, 18, 0xC, 0x02FA);	//Sensor_fine_IT_max = 762
	mt9v117_write_var16(mt9v117, 18, 0x12, 0x0031);	//Sensor_fine_correction = 49
	mt9v117_write_var16(mt9v117, 18, 0x14, 0x00F3);	//Cpipe Last Row = 243
	mt9v117_write_var16(mt9v117, 18, 0x28, 0x0007);	//Read Mode = 6 - skipping + Mirror + Flip
	mt9v117_write_var16(mt9v117, 18, 0x4C, 0x0280);	//Crop Width = 640
	mt9v117_write_var16(mt9v117, 18, 0x4E, 0x00F0);	//Crop Height = 240
	mt9v117_write_var8(mt9v117, 18, 0x50, 0x03);	//Crop Mode = 3
	mt9v117_write_var16(mt9v117, 18, 0x54, 0x0140);	//Output Width = 320
	mt9v117_write_var16(mt9v117, 18, 0x56, 0x00F0);	//Output Height = 240
	mt9v117_write_var16(mt9v117, 18, 0xEC, 0x0000);	//AWB Window Xstart = 0
	mt9v117_write_var16(mt9v117, 18, 0xEE, 0x0000);	//AWB Window Ystart = 0
	mt9v117_write_var16(mt9v117, 18, 0xF0, 0x013F);	//AWB Window Xend = 319
	mt9v117_write_var16(mt9v117, 18, 0xF2, 0x00EF);	//AWB Window Yend = 239
	mt9v117_write_var16(mt9v117, 18, 0xF4, 0x0002);	//AE Window Xstart = 2
	mt9v117_write_var16(mt9v117, 18, 0xF6, 0x0002);	//AE Window Ystart = 2
	mt9v117_write_var16(mt9v117, 18, 0xF8, 0x003F);	//AE Window Xend = 63
	mt9v117_write_var16(mt9v117, 18, 0xFA, 0x002F);	//AE Window Yend = 47

	return 0;
}

static int mt9v117_set_QCIF(struct mt9v117 *mt9v117)
{
	// QCIF, Y-skipping, framerate up to 60 FPS
	mt9v117_write_var16(mt9v117, 18, 0x0, 0x0008);//Row Start = 8
	mt9v117_write_var16(mt9v117, 18, 0x2, 0x0010);//Column Start = 16
	mt9v117_write_var16(mt9v117, 18, 0x4, 0x01F5);//Row End = 501
	mt9v117_write_var16(mt9v117, 18, 0x6, 0x0297);//Column End = 663
	mt9v117_write_var16(mt9v117, 18, 0x8, 0x0111);//Row Speed = 273
	mt9v117_write_var16(mt9v117, 18, 0xA, 0x00A4);//Sensor_fine_IT_min = 164
	mt9v117_write_var16(mt9v117, 18, 0xC, 0x02FA);//Sensor_fine_IT_max = 762
	mt9v117_write_var16(mt9v117, 18, 0x12, 0x0031);	//Sensor_fine_correction = 49
	mt9v117_write_var16(mt9v117, 18, 0x14, 0x00F3);	//Cpipe Last Row = 243
	mt9v117_write_var16(mt9v117, 18, 0x28, 0x0007);	//Read Mode = 7 Skipping + Mirror + Flip
	mt9v117_write_var16(mt9v117, 18, 0x4C, 0x0280);	//Crop Width = 640
	mt9v117_write_var16(mt9v117, 18, 0x4E, 0x00F0);	//Crop Height = 240
	mt9v117_write_var8(mt9v117, 18, 0x50, 0x03);	//Crop Mode = 3
	mt9v117_write_var16(mt9v117, 18, 0x54, 0x00A0);	//Output Width = 160
	mt9v117_write_var16(mt9v117, 18, 0x56, 0x0078);	//Output Height = 120
	mt9v117_write_var16(mt9v117, 18, 0xEC, 0x0000);	//AWB Window Xstart = 0
	mt9v117_write_var16(mt9v117, 18, 0xEE, 0x0000);	//AWB Window Ystart = 0
	mt9v117_write_var16(mt9v117, 18, 0xF0, 0x00AF);	//AWB Window Xend = 175
	mt9v117_write_var16(mt9v117, 18, 0xF2, 0x008F);	//AWB Window Yend = 143
	mt9v117_write_var16(mt9v117, 18, 0xF4, 0x0002);	//AE Window Xstart = 2
	mt9v117_write_var16(mt9v117, 18, 0xF6, 0x0002);	//AE Window Ystart = 2
	mt9v117_write_var16(mt9v117, 18, 0xF8, 0x0022);	//AE Window Xend = 34
	mt9v117_write_var16(mt9v117, 18, 0xFA, 0x001B);	//AE Window Yend = 27

	return 0;
}

static int mt9v117_set_itu_bt656(struct mt9v117 *mt9v117, int itu_bt656)
{
	u16 value;

	mt9v117_read_var16(mt9v117,18,0x0058, &value);

	if (itu_bt656)
		value |= (1<<3);
	else
		value &= ~(1<<3);

	mt9v117_write_var16(mt9v117,18,0x0058,value);

	return 0;
}

/* v4l2 ops */
static struct v4l2_mbus_framefmt *
__mt9v117_get_fmt(struct mt9v117 *mt9v117,
		 struct v4l2_subdev_fh *fh,
		 unsigned int pad,
		 enum v4l2_subdev_format_whence which)
{
	if(pad)
		return NULL;

	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(fh, pad);
	else
		return &mt9v117->format;
}

static int mt9v117_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			  struct v4l2_subdev_format *fmt)
{
	struct mt9v117 *mt9v117 = to_mt9v117(sd);
	struct v4l2_mbus_framefmt *format;

	format = __mt9v117_get_fmt(mt9v117, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;

	return 0;
}

static int mt9v117_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			  struct v4l2_subdev_format *fmt)
{
	struct mt9v117 *mt9v117 = to_mt9v117(sd);
	struct v4l2_mbus_framefmt *format;

	format = __mt9v117_get_fmt(mt9v117, fh, fmt->pad, fmt->which);
	if(!format)
		return -EINVAL;

	if((fmt->format.width == 640 && fmt->format.height == 480) ||
			(fmt->format.width == 320 && fmt->format.height == 240) ||
			(fmt->format.width == 160 && fmt->format.height == 120)) {
		format->width = fmt->format.width;
		format->height = fmt->format.height;
		fmt->format = *format;
	} else
		return -EINVAL;

	mt9v117->timings_uptodate = 0;

	return 0;
}

static int mt9v117_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct mt9v117 *mt9v117 = to_mt9v117(sd);

	if (code->index)
		return -EINVAL;

	code->code = mt9v117->format.code;

	return 0;
}

static int mt9v117_apply_config(struct mt9v117 *mt9v117)
{
	struct v4l2_mbus_framefmt *format = &mt9v117->format;
	int ret = 0;

	ret = mt9v117_soft_reset(mt9v117);
	mt9v117_patch1(mt9v117);
	mt9v117_set_basic_settings(mt9v117);
	ret = mt9v117_update_timings(mt9v117);
	if (ret < 0) {
		v4l2_err(&mt9v117->sd, "Unable to calculate Video Timing\n");
		return ret;
	}

	switch (format->height) {
		case 480:
			ret = mt9v117_set_VGA(mt9v117);
			break;
		case 240:
			ret = mt9v117_set_QVGA(mt9v117);
			break;
		case 120:
			ret = mt9v117_set_QCIF(mt9v117);
			break;
		default:
			v4l2_err(&mt9v117->sd, "resolution not supported\n");
			return -EINVAL;
	}

	if(ret < 0) {
		v4l2_err(&mt9v117->sd, "error setting resolution\n");
		return ret;
	}

	mt9v117_blanking_write(mt9v117);

	mt9v117_set_itu_bt656(mt9v117,1);

	// apply settings
	ret = mt9v117_config_change(mt9v117);

	return ret;
}

static int mt9v117_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mt9v117 *mt9v117 = to_mt9v117(sd);
	int ret = 0;

	if(enable) {
		if(mt9v117->set_power)
			mt9v117->set_power(MT9V117_POWER_ON);
		msleep(500); /* time has to be confirmed */
		ret = mt9v117_apply_config(mt9v117);
		if (ret < 0) {
			v4l2_err(&mt9v117->sd, "failed to apply config\n");
			goto power_off;
		}

		mt9v117->isStreaming = true;

	} else {
		mt9v117->isStreaming = false;
		mt9v117->set_power(MT9V117_POWER_OFF);
	}

	return 0;

power_off:
	mt9v117->isStreaming = false;
	mt9v117->set_power(MT9V117_POWER_OFF);
	return ret;
}

static const struct v4l2_subdev_core_ops mt9v117_core_ops = {
};

static int mt9v117_g_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_frame_interval *fi)
{
	struct mt9v117 *mt9v117 = to_mt9v117(sd);
	unsigned g;

	memset(fi, 0, sizeof(*fi));
	fi->interval.denominator = NSEC_PER_SEC;
	fi->interval.numerator = mt9v117->frameperiod_ns;

	/* I could return there but I prefer to reduce the fraction first */
	g = gcd(fi->interval.numerator, fi->interval.denominator);

	fi->interval.numerator   /= g;
	fi->interval.denominator /= g;

	return 0;
}

static int mt9v117_s_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_frame_interval *fi)
{
	struct mt9v117 *mt9v117 = to_mt9v117(sd);
	uint64_t tmp64;
	int ret;

	/* Protect against division by 0. */
	if (fi->interval.denominator == 0)
		fi->interval.denominator = 30;

	if (fi->interval.numerator == 0)
		fi->interval.numerator = 1;

	tmp64 = (u64)fi->interval.numerator * NSEC_PER_SEC;
	mt9v117->frameperiod_ns = div_u64((u64)tmp64, fi->interval.denominator);

	mt9v117->timings_uptodate = 0;
	if (!mt9v117->isStreaming) {
		ret = mt9v117_update_timings(mt9v117);
		if (ret < 0) {
			v4l2_err(&mt9v117->sd,
					"Unable to calculate Video Timing\n");
			return ret;
		}
	}

	return 0;
}

static const struct v4l2_subdev_video_ops mt9v117_video_ops = {
	.s_stream = mt9v117_s_stream,
	.g_frame_interval = mt9v117_g_frame_interval,
	.s_frame_interval = mt9v117_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops mt9v117_pad_ops = {
	.get_fmt = mt9v117_get_fmt,
	.set_fmt = mt9v117_set_fmt,
	.enum_mbus_code = mt9v117_enum_mbus_code,
};

static const struct v4l2_subdev_ops mt9v117_ops = {
	.core = &mt9v117_core_ops,
	.video = &mt9v117_video_ops,
	.pad = &mt9v117_pad_ops,
};

static int mt9v117_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct mt9v117 *mt9v117;
	struct mt9v117_platform_data *pdata = NULL;
	struct v4l2_subdev *sd;
	int ret = -ENODEV;
	struct v4l2_mbus_framefmt *format;
	u16 model_id;
	pdata = client->dev.platform_data;

	if(!pdata)
		return -EINVAL;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		ret = -EIO;
		goto ei2c;
	}

	dev_info(&client->dev, "detecting mt9v117 client on address 0x%x\n",
			client->addr << 1);

	mt9v117 = kzalloc(sizeof(*mt9v117), GFP_KERNEL);
	if(!mt9v117) {
		v4l_err(client, "alloc failed for data structure\n");
		ret = -ENOMEM;
		goto enomem;
	}

	mt9v117->set_power = pdata->set_power;
	mt9v117->pdata = client->dev.platform_data;
	mt9v117->frameperiod_ns = MT9V117_frameperiod_ns_DEF;
	mt9v117->i2c_client = client;

	format = &mt9v117->format;
	format->width = 640;
	format->height = 480;
	format->field = V4L2_FIELD_NONE;
	format->code = V4L2_MBUS_FMT_UYVY8_2X8;
	format->colorspace = V4L2_COLORSPACE_SMPTE170M;

	sd = &mt9v117->sd;
	v4l2_i2c_subdev_init(sd, client, &mt9v117_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	mt9v117->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&sd->entity, 1, &mt9v117->pad, 0);
	if(ret < 0) {
		v4l_err(client, "failed to init media entity\n");
		goto emedia;
	}

	/*see if chip is present */
	if(pdata->set_power) {
		pdata->set_power(MT9V117_POWER_ON);
		msleep(500);
	}

	mt9v117_read16(mt9v117, 0x3000, &model_id);

	if(model_id != 0x2282) {
		v4l2_err(sd, "Error Model ID = 0x%04X\n", model_id);
		ret = -ENODEV;
		goto echipident;
	}

	if(pdata->set_power)
		pdata->set_power(MT9V117_POWER_OFF);

	ret = mt9v117_update_timings(mt9v117);
	if (ret < 0) {
		v4l2_err(&mt9v117->sd, "Unable to calculate Video Timing\n");
		goto echipident;
	}

	return ret;
echipident:
	if(pdata->set_power)
		pdata->set_power(MT9V117_POWER_OFF);
	media_entity_cleanup(&sd->entity);
emedia:
	kfree(mt9v117);
enomem:
ei2c:
	return ret;
}

static int mt9v117_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mt9v117 *mt9v117 = to_mt9v117(sd);

	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	kfree(mt9v117);
	return 0;
}

/* ----------------------------------------------------------------------- */

static const struct i2c_device_id mt9v117_id[] = {
	{ "mt9v117", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, mt9v117_id);

static struct i2c_driver mt9v117_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "mt9v117",
	},
	.probe = mt9v117_probe,
	.remove = mt9v117_remove,
	.id_table = mt9v117_id,
};

module_i2c_driver(mt9v117_driver);
