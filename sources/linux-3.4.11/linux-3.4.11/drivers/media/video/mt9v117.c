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

#define DEFAULT_FRAME_PERIOD_NS		33333333 /* = 30fps */
#define MIN_HORIZONTAL_BLANKING		210
#define MIN_VERTICAL_BLANKING		31

/* Timeout for command completion: MT9V117_CMD_WAIT_COUNT * MT9V117_CMD_WAIT ms.
 * MT9V117_CMD_WAIT is the period of I2C poll.
 */
#define MT9V117_CMD_WAIT_COUNT		50
#define MT9V117_CMD_WAIT		10

struct mt9v117_timings {
	uint16_t line_length;
	uint16_t frame_length;
	uint16_t fd_60;
	uint16_t fd_50;
	uint16_t fd_60_max;
	uint16_t fd_50_max;
};

/* mt9v117 data structure */
struct mt9v117 {
	struct mt9v117_platform_data	*pdata;
	struct i2c_client		*i2c_client;
	/* V4L2 device */
	struct v4l2_subdev		 sd;
	struct media_pad		 pad;
	/* V4L2 controls */
	struct v4l2_ctrl_handler	 ctrls;
	/* Format */
	struct v4l2_mbus_framefmt	 format;
	struct v4l2_rect		 crop;
	/* Sensor timings */
	struct mt9v117_timings		 timings;
	uint32_t			 frameperiod_ns;
	uint32_t			 max_frameperiod_ns;
	uint32_t			 real_frameperiod_ns;
	bool				 timings_uptodate;
	/* Anti-flickering */
	bool				 flicker_is_50hz;
	/* Streaming control */
	int				 isStreaming;
	/* Callbacks */
	int (*set_power)(int on);

};

enum mt9v117_test_patterns {
	MT9V117_TEST_PATTERN_NONE = 0,
	MT9V117_TEST_PATTERN_FLAT_FIELD = 1,
	MT9V117_TEST_PATTERN_VERTICAL_RAMP = 2,
	MT9V117_TEST_PATTERN_COLOR_BAR = 3,
	MT9V117_TEST_PATTERN_VERTICAL_BAR = 4,
	MT9V117_TEST_PATTERN_PSEUDO_RANDOM = 5,
	MT9V117_TEST_PATTERN_HORIZONTAL_BAR = 6,
	MT9V117_TEST_PATTERN_NB = 7
};

static const char * const mt9v117_test_pattern_labels[] = {
	[MT9V117_TEST_PATTERN_NONE] = "No test pattern",
	[MT9V117_TEST_PATTERN_FLAT_FIELD] = "Flat field",
	[MT9V117_TEST_PATTERN_VERTICAL_RAMP] = "Vertical Ramp",
	[MT9V117_TEST_PATTERN_COLOR_BAR] = "Color bar",
	[MT9V117_TEST_PATTERN_VERTICAL_BAR] = "Vertical bars",
	[MT9V117_TEST_PATTERN_PSEUDO_RANDOM] = "Pseudo random",
	[MT9V117_TEST_PATTERN_HORIZONTAL_BAR] = "Horizontal bars"
};

/* Module parameters */

static int test_mode = 0;
static int tm_r10    = 512;
static int tm_g10    = 512;
static int tm_b10    = 512;
static int mirror    = 1;
static int flip      = 1;

module_param(test_mode, int, 0644);
module_param(tm_r10   , int, 0644);
module_param(tm_g10   , int, 0644);
module_param(tm_b10   , int, 0644);
module_param(mirror   , int, 0644);
module_param(flip     , int, 0644);


static inline void mt9v117_reset_sleep(struct mt9v117 *mt9v117)
{
	/* Wait 3.5 ms to 35 ms (respectively for 54 MHz to 6 MHz ext clock) */
	msleep((34 * 6000000 / mt9v117->pdata->ext_clk_freq_hz) + 1);
}

static inline struct mt9v117 *to_mt9v117(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9v117, sd);
}

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

static int mt9v117_issue_cmd(struct mt9v117 *mt9v117, uint16_t cmd)
{
	u16 val;
	int i;

	/* Force reset of OK flag */
	cmd |= MT9V117_HOST_COMMAND_OK;

	/* Issue command */
	mt9v117_write16(mt9v117, MT9V117_REG_HOST_COMMAND, cmd);

	/* Wait until completion of the command */
	for (i = 0; i < MT9V117_CMD_WAIT_COUNT; i++) {
		/* Read command register */
		mt9v117_read16(mt9v117, MT9V117_REG_HOST_COMMAND, &val);

		/* Wait if not completed */
		if ((val & cmd) == MT9V117_HOST_COMMAND_OK)
			break;

		/* Wait before next read */
		msleep(MT9V117_CMD_WAIT);
	}

	/* Command failed */
	if ((val & cmd) != MT9V117_HOST_COMMAND_OK) {
		v4l2_err(&mt9v117->sd, "Command %x failed!\n", cmd);
		return -EPIPE;
	}

	return 0;
}

static int mt9v117_config_change(struct mt9v117 *mt9v117)
{
	/* Apply config
	 *    SYSMGR_NEXT_STATE
	 *    COMMAND_REGISTER
	 */
	mt9v117_write_var8(mt9v117, MT9V117_VAR_SYSMGR_NEXT_STATE, 0x28);
	return mt9v117_issue_cmd(mt9v117, MT9V117_HOST_COMMAND_1);
}

static int mt9v117_refresh(struct mt9v117 *mt9v117)
{
	/* Refreshes device state */
	return mt9v117_issue_cmd(mt9v117, MT9V117_HOST_COMMAND_2);
}


static int mt9v117_soft_reset(struct mt9v117 *mt9v117)
{
	if (mt9v117_write16(mt9v117, 0x001A, 0x0001) < 0)
		return -EPIPE;

	if (mt9v117_write16(mt9v117, 0x001A, 0x0000) < 0)
		return -EPIPE;

	mt9v117_reset_sleep(mt9v117);

	return 0;
}

static int mt9v117_patch1(struct mt9v117 *mt9v117)
{
	/* Errata item 2:
	 * Correct image aberration due to low illumination conditions.
	 */
	mt9v117_write16(mt9v117, 0x301A, 0x10D0);
	mt9v117_write16(mt9v117, 0x31C0, 0x1404);
	mt9v117_write16(mt9v117, 0x3ED8, 0x879C);
	mt9v117_write16(mt9v117, 0x3042, 0x20E1);
	mt9v117_write16(mt9v117, 0x30D4, 0x8020);
	mt9v117_write16(mt9v117, 0x30C0, 0x0026);
	mt9v117_write16(mt9v117, 0x301A, 0x10D4);

	/* Errata item 6 */
	mt9v117_write_var16(mt9v117, 10, 0x0002, 0x00D3);
	mt9v117_write_var16(mt9v117, 18, 0x0078, 0x00A0);
	mt9v117_write_var16(mt9v117, 18, 0x0076, 0x0140);

	/* Errata item 8 */
	mt9v117_write_var16(mt9v117, 15, 0x0004, 0x00FC);
	mt9v117_write_var16(mt9v117, 15, 0x0038, 0x007F);
	mt9v117_write_var16(mt9v117, 15, 0x003A, 0x007F);
	mt9v117_write_var16(mt9v117, 15, 0x003C, 0x007F);
	mt9v117_write_var16(mt9v117, 15, 0x0004, 0x00F4);

	/* Load Patch 0403
	 *    ACCESS_CTL_STAT
	 *    PHYSICAL_ADDRESS_ACCESS
	 *    Patches 1 to 13
	 *    LOGICAL_ADDRESS_ACCESS
	 */
	mt9v117_write16(mt9v117,  0x0982, 0x0001);
	mt9v117_write16(mt9v117,  0x098A, 0x7000); 
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
	mt9v117_write16(mt9v117,  0x098E, 0x0000);

	/* Apply patch 0403
	 *    PATCHLDR_LOADER_ADDRESS
	 *    PATCHLDR_PATCH_ID
	 *    PATCHLDR_FIRMWARE_ID doute
	 */
	mt9v117_write_var16(mt9v117, 0x18, 0x0000, 0x05d8);
	mt9v117_write_var16(mt9v117, 0x18, 0x0002, 0x0403);
	mt9v117_write_var32(mt9v117, 0x18, 0x0004, 0x00430104);

	/* Apply patch */
	return mt9v117_issue_cmd(mt9v117,  0x8001);
}

static int mt9v117_set_basic_settings(struct mt9v117 *mt9v117)
{
	u16 value;

	/* Set awb_pixel_threshold_count */
	mt9v117_write_var32(mt9v117, 11, 0x0040, 50000);

	/* Set ae_rule_algo to average */
	mt9v117_write_var16(mt9v117, 9, 0x4, 0x0);

	/* Get pad configuration */
	mt9v117_read16(mt9v117, 0x0030, &value);

	/* Set Pad slew more agressive:
	 *    data slew rate = 1 (default value 0)
	 *    clock slew rate = 6 (default value 4)
	 */
	value |= 0x0001;
	value |= 0x0600;

	/* Update pad configuration */
	mt9v117_write16(mt9v117, 0x0030, value);

	return 0;
}

static int mt9v117_update_frame_period(struct mt9v117 *mt9v117)
{
	uint32_t height = mt9v117->format.height > 240 ? 488 : 248;
	uint32_t pix_clock = mt9v117->pdata->ext_clk_freq_hz / 2;
	uint64_t tmp64;

	/* Calculate constants */
	tmp64 = (uint64_t) NSEC_PER_SEC * (648 + MIN_HORIZONTAL_BLANKING) *
		(height + MIN_VERTICAL_BLANKING);
	mt9v117->max_frameperiod_ns = div_u64(tmp64, pix_clock);

	/* Update frame period */
	if (mt9v117->frameperiod_ns < mt9v117->max_frameperiod_ns) {
		mt9v117->frameperiod_ns = mt9v117->max_frameperiod_ns;
		mt9v117->timings_uptodate = 0;
	}

	return 0;
}

static int mt9v117_update_timings(struct mt9v117 *mt9v117)
{
	struct mt9v117_timings *timings = &mt9v117->timings;
	uint32_t height = mt9v117->format.height > 240 ? 488 : 248;
	uint32_t pix_clock = mt9v117->pdata->ext_clk_freq_hz / 2;
	uint32_t best_line_length = 0, best_frame_length = 0;
	uint32_t rows, lines, max_rows, tmp;
	uint64_t minError = ULLONG_MAX, error;
	uint64_t num, den, tmp64;

	/* Update timings if necessary */
	if (mt9v117->timings_uptodate)
		return 0;

	/* Calculate constants */
	num = (u64) mt9v117->frameperiod_ns * pix_clock;
	tmp64 = ((uint64_t) height + MIN_VERTICAL_BLANKING) * NSEC_PER_SEC;
	max_rows = div64_u64(num, tmp64);

	/* Find best value for each line length (do at least one loop) */
	rows = 648 + MIN_HORIZONTAL_BLANKING;
	do {
		/* Find best frame length */
		den = (u64) rows * NSEC_PER_SEC;
		lines = div64_u64(num + (den / 2), den);

		/* Calculate error */
		error = abs64(((u64) lines * rows * NSEC_PER_SEC) - num);

		/* A new best has been found */
		if (error < minError) {
			minError = error;
			best_line_length = rows;
			best_frame_length = lines;
		}

		/* Check next line length */
		rows++;
	} while (rows <= max_rows);

	/* check a solution has been found */
	if ((int64_t) minError < 0) {
		v4l2_err(&mt9v117->sd, "error setting framerate\n");
		return -1;
	}

	/* Save blanking */
	timings->line_length = best_line_length;
	timings->frame_length = best_frame_length;

	/* Calculate real frame period */
	mt9v117->real_frameperiod_ns = div_u64((uint64_t) timings->line_length *
					       (uint64_t) timings->frame_length *
					       NSEC_PER_SEC, pix_clock);

	/* Calculate FD lines */
	tmp = (uint32_t) timings->line_length * 120;
	timings->fd_60 = (pix_clock + (tmp / 2)) / tmp;
	tmp = (uint32_t) timings->line_length * 100;
	timings->fd_50 = (pix_clock + (tmp / 2)) / tmp;

	/* Calculate FD max */
	tmp64 = ((uint64_t) mt9v117->real_frameperiod_ns + 10000) * 120;
	timings->fd_60_max = div_u64(tmp64, NSEC_PER_SEC);
	tmp64 = ((uint64_t) mt9v117->real_frameperiod_ns + 10000) * 100;
	timings->fd_50_max = div_u64(tmp64, NSEC_PER_SEC);

	/* FD max cannot be null */
	if (timings->fd_60_max < 1)
		timings->fd_60_max = 1;
	if (timings->fd_50_max < 1)
		timings->fd_50_max = 1;

	/* Display debug informations */
	v4l2_info(&mt9v117->sd, "pixel_clock: %u\n", pix_clock * 2);
	v4l2_info(&mt9v117->sd, "setting framerate:\n"
		  "   frame_period: %u -> %u\n"
		  "   line_length:  %u\n"
		  "   frame_length: %u\n"
		  "   fd_60:        %u / %u\n"
		  "   fd_50:        %u / %u\n",
		  mt9v117->frameperiod_ns, mt9v117->real_frameperiod_ns,
		  timings->line_length,
		  timings->frame_length,
		  timings->fd_60, timings->fd_60_max,
		  timings->fd_50, timings->fd_50_max);

	/* Timings are uptodate */
	mt9v117->timings_uptodate = 1;

	return 0;
}

static int mt9v117_blanking_write(struct mt9v117 *mt9v117)
{
	struct mt9v117_timings *timings = &mt9v117->timings;

	/* Write line/frame length */
	mt9v117_write_var16(mt9v117, 18, 0x10, timings->line_length);
	mt9v117_write_var16(mt9v117, 18, 0x0E, timings->frame_length);

	/* Write FD zones */
	mt9v117_write_var16(mt9v117, 18, 0x16, timings->fd_60);
	mt9v117_write_var16(mt9v117, 18, 0x18, timings->fd_50);
	mt9v117_write_var16(mt9v117, 18, 0x1A, timings->fd_60_max);
	mt9v117_write_var16(mt9v117, 18, 0x1C, timings->fd_50_max);
	mt9v117_write_var16(mt9v117, 18, 0x1E, timings->fd_60_max);
	mt9v117_write_var16(mt9v117, 18, 0x20, timings->fd_50_max);

	return 0;
}

static int mt9v117_set_format(struct mt9v117 *mt9v117)
{
	struct v4l2_mbus_framefmt *fmt = &mt9v117->format;
	struct v4l2_rect crop = mt9v117->crop;
	uint16_t skip = 0;

	/* Set bayer mode (no AWB and AE) */
	if (fmt->code == V4L2_MBUS_FMT_SRGGB8_1X8 ||
	    fmt->code == V4L2_MBUS_FMT_SRGGB10_1X10) {
		/* Use Y-skipping when sensor height crop is x4 or more to
		 * output height.
		 */
		if (crop.height / 2 >= fmt->height)
			skip = 1;

		/* Set read mode:
		 *    [2] Y-Skipping: skip
		 *    [1] Mirror: enabled
		 *    [0] Flip: enabled
		 */
		mt9v117_write_var16(mt9v117, 18, 0x28, (skip << 2) |
						       (mirror ? 0x02 : 0x00) |
						       (flip ? 0x01 : 0x00));

		/* Set output format to Bayer */
		mt9v117_write_var16(mt9v117, 18, 0x58, 0x0210);

		/* Disable AE and AWB */
		mt9v117_write_var16(mt9v117, 10, 0x04, 0);
		mt9v117_write_var16(mt9v117, 11, 0x04, 0);

		/* Set Digital gain (R, G, G, B gains) */
		mt9v117_write16(mt9v117, 0x32d4, 0x0080);
		mt9v117_write16(mt9v117, 0x32d6, 0x0080);
		mt9v117_write16(mt9v117, 0x32d8, 0x0080);
		mt9v117_write16(mt9v117, 0x32da, 0x0080);

		/* Calculate sensor crop
		 * /!\ In bayer mode, the sensor height crop is lower than
		 *     expected as specified: we substract 2 to final output
		 *      height.
		 */
		crop.left += 16;
		crop.top += skip ? 8 : 12;
		crop.width += crop.left + 7;
		crop.height += crop.top - 2;
		v4l2_info(&mt9v117->sd, "Sensor crop: %d %d %d %d\n",
			  crop.left, crop.top, crop.width, crop.height);

		/* Set sensor crop */
		mt9v117_write_var16(mt9v117, 18, 0x00, crop.top);
		mt9v117_write_var16(mt9v117, 18, 0x02, crop.left);
		mt9v117_write_var16(mt9v117, 18, 0x04, crop.height);
		mt9v117_write_var16(mt9v117, 18, 0x06, crop.width);

		/* Set output size (width, height)
		 * /!\ In bayer mode, the output height is bigger as specified:
		 *     9 lines are added. So we substract 9 to final height.
		 */
		mt9v117_write_var16(mt9v117, 18, 0x54, fmt->width);
		mt9v117_write_var16(mt9v117, 18, 0x56, fmt->height - 9);

		return 0;
	}

	/* Use Y-skipping when height is lower or equal to 240 */
	if (fmt->height <= 240)
		skip = 1;

	/* Set sensor window:
	 *  - y_start
	 *  - x_start
	 *  - y_end
	 *  - x_end
	 */
	mt9v117_write_var16(mt9v117, 18, 0x00, skip ? 8 : 12);
	mt9v117_write_var16(mt9v117, 18, 0x02, 16);
	mt9v117_write_var16(mt9v117, 18, 0x04, skip ? 501 : 499);
	mt9v117_write_var16(mt9v117, 18, 0x06, 663);

	/* Set default values:
	 *     Row Speed = 273
	 *     Sensor_fine_IT_min = 164
	 *     Sensor_fine_IT_max = 762
	 *     Sensor_fine_correction = 49
	 *     Cpipe Last Row = 243 or 483
	 */
	mt9v117_write_var16(mt9v117, 18, 0x08, 273);
	mt9v117_write_var16(mt9v117, 18, 0x0A, 164);
	mt9v117_write_var16(mt9v117, 18, 0x0C, 762);
	mt9v117_write_var16(mt9v117, 18, 0x12, 49);
	mt9v117_write_var16(mt9v117, 18, 0x14, skip ? 243 : 483);

	/* Set read mode:
	 *    [2] Y-Skipping: skip
	 *    [1] Mirror: enabled
	 *    [0] Flip: enabled
	 */
	mt9v117_write_var16(mt9v117, 18, 0x28, (skip << 2) |
					       (mirror ? 0x02 : 0x00) |
					       (flip ? 0x01 : 0x00));

	/* Set output format to YUV */
	mt9v117_write_var16(mt9v117, 18, 0x58, 0x00);

	/* Enable AE and AWB */
	mt9v117_write_var16(mt9v117, 10, 0x04, 0x0F);
	mt9v117_write_var16(mt9v117, 11, 0x04, 0xBE);

	/* Set sensor crop (width, height, mode) */
	mt9v117_write_var16(mt9v117, 18, 0x4C, 640);
	mt9v117_write_var16(mt9v117, 18, 0x4E, skip ? 240 : 480);
	mt9v117_write_var8(mt9v117, 18, 0x50, 0x03);

	/* Set output size (width, height) */
	mt9v117_write_var16(mt9v117, 18, 0x54, fmt->width);
	mt9v117_write_var16(mt9v117, 18, 0x56, fmt->height);

	/* Set AWB window (x_start, y_start, x_end, y_end)) */
	mt9v117_write_var16(mt9v117, 18, 0xEC, 0);
	mt9v117_write_var16(mt9v117, 18, 0xEE, 0);
	mt9v117_write_var16(mt9v117, 18, 0xF0, fmt->width - 1);
	mt9v117_write_var16(mt9v117, 18, 0xF2, fmt->height - 1);

	/* Set AE window (x_start, y_start, x_end, y_end)) */
	mt9v117_write_var16(mt9v117, 18, 0xF4, 2);
	mt9v117_write_var16(mt9v117, 18, 0xF6, 2);
	mt9v117_write_var16(mt9v117, 18, 0xF8, (fmt->width / 5) - 1);
	mt9v117_write_var16(mt9v117, 18, 0xFA, (fmt->height / 5) - 1);

	return 0;
}

static int mt9v117_set_itu_bt656(struct mt9v117 *mt9v117, int itu_bt656)
{
	u16 value;

	/* Get output pads configuration */
	mt9v117_read_var16(mt9v117, 18, 0x0058, &value);

	/* Enable/Disable BT656 output */
	if (itu_bt656)
		value |= (1<<3);
	else
		value &= ~(1<<3);

	/* Update output pads configuration */
	mt9v117_write_var16(mt9v117, 18, 0x0058, value);

	return 0;
}

static void mt9v117_test_pattern_setup(struct mt9v117 *mt9v117)
{
	/* Set cam_input_source to test pattern generator */
	mt9v117_write_var8(mt9v117, MT9V117_VAR_CAM_INPUT_SOURCE, 0x01);

	/* Turn off AE */
	mt9v117_write_var16(mt9v117, MT9V117_VAR_AE_TRACK_ALGO, 0x0000);

	/* Turn off AWB */
	mt9v117_write_var16(mt9v117, MT9V117_VAR_AWB_ALGO, 0x0000);

	/* Update sensor configuration */
	mt9v117_config_change(mt9v117);

	/* Set digital gains to unity */
	mt9v117_write16(mt9v117, 0x3210, 0x0010);
	mt9v117_write16(mt9v117, 0x32d4, 0x0080);
	mt9v117_write16(mt9v117, 0x32d6, 0x0080);
	mt9v117_write16(mt9v117, 0x32d8, 0x0080);
	mt9v117_write16(mt9v117, 0x32da, 0x0080);
}

static void mt9v117_exit_pattern_setup(struct mt9v117 *mt9v117)
{
	/* Set cam_input_source to sensor */
	mt9v117_write_var8(mt9v117, MT9V117_VAR_CAM_INPUT_SOURCE, 0x00);

	/* Turn on AE */
	mt9v117_write_var16(mt9v117, MT9V117_VAR_AE_TRACK_ALGO, 0x000f);

	/* Turn on AWB */
	mt9v117_write_var16(mt9v117, MT9V117_VAR_AWB_ALGO, 0x00be);
}

static void mt9v117_apply_test_mode(struct mt9v117 *mt9v117)
{
	/* Restore normal mode */
	if (test_mode == 0 || test_mode >= MT9V117_TEST_PATTERN_NB) {
		mt9v117_exit_pattern_setup(mt9v117);
		return;
	}

	dev_info(&mt9v117->i2c_client->dev, "Enabling test pattern %s\n",
		 mt9v117_test_pattern_labels[test_mode]);

	/* Setup pattern test */
	mt9v117_test_pattern_setup(mt9v117);

	/* Set the desired patterni and gain */
	mt9v117_write_var8(mt9v117, MT9V117_VAR_CAM_INPUT_TEST_PATTERN_SELECT,
			   test_mode);
	mt9v117_write_var16(mt9v117, MT9V117_VAR_CAM_INPUT_TEST_PATTERN_RED,
			    tm_r10);
	mt9v117_write_var16(mt9v117, MT9V117_VAR_CAM_INPUT_TEST_PATTERN_GREEN,
			    tm_g10);
	mt9v117_write_var16(mt9v117, MT9V117_VAR_CAM_INPUT_TEST_PATTERN_BLUE,
			    tm_b10);

	/* Refresh sensor configuration */
	mt9v117_refresh(mt9v117);
}

static int mt9v117_apply_config(struct mt9v117 *mt9v117)
{
	int ret = 0;

	/* Do a soft reset */
	ret = mt9v117_soft_reset(mt9v117);
	if (ret < 0) {
		v4l2_err(&mt9v117->sd, "Unable to reset sensor\n");
		return ret;
	}

	/* Set patch and basic settings */
	mt9v117_patch1(mt9v117);
	mt9v117_set_basic_settings(mt9v117);

	/* Set a test pattern if required by module parameter */
	mt9v117_apply_test_mode(mt9v117);

	/* Update timings */
	ret = mt9v117_update_timings(mt9v117);
	if (ret < 0) {
		v4l2_err(&mt9v117->sd, "Unable to calculate Video Timing\n");
		return ret;
	}

	/* Set format */
	mt9v117_set_format(mt9v117);

	/* Write blanking */
	if (!test_mode) {
		mt9v117_blanking_write(mt9v117);
	}

	/* Set BT656 output */
	mt9v117_set_itu_bt656(mt9v117, mt9v117->pdata->enable_bt656);

	/* Set anti-flickering mode */
	ret = mt9v117_write_var16(mt9v117, 0x12, 0x003E,
				  mt9v117->flicker_is_50hz & 0x01);

	/* Apply settings */
	return mt9v117_config_change(mt9v117);
}

static int mt9v117_get_ae_gain(struct mt9v117 *mt9v117, uint16_t *val)
{
	int ret;

	/* Get AE gain in YUV mode */
	if (mt9v117->format.code == V4L2_MBUS_FMT_UYVY8_2X8) {
		return mt9v117_read_var16(mt9v117, 0x0A, 0x002E, val);
	}

	/* Get analog gain */
	ret = mt9v117_read16(mt9v117, 0x3028, val);
	*val <<= 9;

	return ret;
}

static uint16_t mt9v117_set_gain(struct mt9v117 *mt9v117, uint16_t val)
{
	uint16_t gain;

	/* Do not set gain in YUV mode */
	if (mt9v117->format.code == V4L2_MBUS_FMT_UYVY8_2X8)
		return val;

	/* Analog gain control is from 0 to 127, so convert to 7bit value */
	gain = (val >> 9) & 0x7F;

	/* Write analog gain */
	mt9v117_write16(mt9v117, 0x3028, gain);

	return (val << 9);
}

static int mt9v117_get_ae_exposure(struct mt9v117 *mt9v117, uint16_t *exp)
{
	/* Get AE gain in YUV mode */
	if (mt9v117->format.code == V4L2_MBUS_FMT_UYVY8_2X8) {
		return mt9v117_read_var16(mt9v117, 0x12, 0x002E, exp);
	}

	/* Get coarse time */
	return mt9v117_read16(mt9v117, 0x3012, exp);
}

static uint16_t mt9v117_set_exposure(struct mt9v117 *mt9v117, uint16_t exp)
{
	/* Do not set exposure in YUV mode */
	if (mt9v117->format.code == V4L2_MBUS_FMT_UYVY8_2X8)
		return exp;

	/* Exposure cannot be greater than frame length */
	if (exp > mt9v117->timings.frame_length)
		exp = mt9v117->timings.frame_length;

	/* Set coarse time (fine is not set here) */
	mt9v117_write16(mt9v117, 0x3012, exp);

	return exp;
}

static int mt9v117_get_ae_exposure_us(struct mt9v117 *mt9v117, uint32_t *exp_us)
{
	uint16_t exp_coarse, exp_fine;
	uint32_t val;
	int ret;

	/* Get exposure values */
	if (mt9v117->format.code == V4L2_MBUS_FMT_UYVY8_2X8) {
		ret = mt9v117_read_var16(mt9v117, 0x12, 0x002E, &exp_coarse);
		ret |= mt9v117_read_var16(mt9v117, 0x12, 0x0030, &exp_fine);
	} else {
		ret = mt9v117_read16(mt9v117, 0x3012, &exp_coarse);
		exp_fine = 0;
	}

	/* Bad read on sensor */
	if (ret)
		return ret;

	/* Get final exposure in pixel count */
	val = (uint32_t) exp_coarse * mt9v117->timings.line_length +
	      (uint32_t) exp_fine;

	/* Convert to us */
	*exp_us = div_u64((uint64_t) val * 2 * 1000 * 1000,
			  mt9v117->pdata->ext_clk_freq_hz);

	return 0;
}

static uint32_t mt9v117_set_exposure_us(struct mt9v117 *mt9v117,
				        uint32_t exp_us)
{
	uint16_t val;

	/* Do not set exposure in YUV mode */
	if (mt9v117->format.code == V4L2_MBUS_FMT_UYVY8_2X8)
		return exp_us;

	val = div64_u64((uint64_t) exp_us * mt9v117->pdata->ext_clk_freq_hz,
			(uint64_t) mt9v117->timings.line_length * 2000000);

	/* Set exposure */
	val = mt9v117_set_exposure(mt9v117, val);

	/* Update value */
	exp_us = div64_u64(
			(uint64_t) val * mt9v117->timings.line_length * 2000000,
			(uint64_t)  mt9v117->pdata->ext_clk_freq_hz);

	return exp_us;
}

static int mt9v117_get_ae_zone(struct mt9v117 *mt9v117)
{
	uint16_t val;

	if (mt9v117_read_var16(mt9v117, 0x0A, 0x002A, &val))
		return -1;

	return (int) (val & 0x03);
}

static int mt9v117_set_anti_flickering(struct mt9v117 *mt9v117, bool is_50hz)
{
	int ret;

	/* Keep flickering value */
	mt9v117->flicker_is_50hz = is_50hz;

	/* Set anti-flickering mode:
	 *    0 for 60Hz,
	 *    1 for 50Hz.
	 */
	ret = mt9v117_write_var16(mt9v117, 0x12, 0x003E, is_50hz & 0x01);

	/* Apply when streaming */
	if (mt9v117->isStreaming)
		return mt9v117_config_change(mt9v117);

	return 0;
}

/*
 * V4L2 video ops
 */
static int mt9v117_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mt9v117 *mt9v117 = to_mt9v117(sd);
	int ret = 0;

	/* Streamon/off */
	if(enable) {
		/* Power on */
		if(mt9v117->set_power)
			mt9v117->set_power(MT9V117_POWER_ON);

		/* Wait before configuring sensor */
		mt9v117_reset_sleep(mt9v117);

		/* Apply configuration on sensor */
		ret = mt9v117_apply_config(mt9v117);
		if (ret < 0) {
			v4l2_err(&mt9v117->sd, "failed to apply config\n");
			goto power_off;
		}

		/* Streaming */
		mt9v117->isStreaming = true;
	} else {
		/* Power off */
		if(mt9v117->set_power)
			mt9v117->set_power(MT9V117_POWER_OFF);

		/* Not streaming */
		mt9v117->isStreaming = false;
	}

	return 0;

power_off:
	mt9v117->isStreaming = false;
	if(mt9v117->set_power)
		mt9v117->set_power(MT9V117_POWER_OFF);
	return ret;
}

static int mt9v117_enum_mbus(struct v4l2_subdev *sd, unsigned int index,
			     enum v4l2_mbus_pixelcode *code)
{
	/* Only support:
	 *    - YUV422-8bit
	 *    - RAW-8bit (bayer)
	 *    - RAW-10bit (bayer)
	 */
	switch (index) {
	case 0:
		*code = V4L2_MBUS_FMT_UYVY8_2X8;
		break;
	case 1:
		*code = V4L2_MBUS_FMT_SRGGB8_1X8;
		break;
	case 2:
		*code = V4L2_MBUS_FMT_SRGGB10_1X10;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int mt9v117_g_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *mf)
{
	struct mt9v117 *mt9v117 = to_mt9v117(sd);

	/* Get format */
	*mf = mt9v117->format;

	return 0;
}

static int mt9v117_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *mf)
{
	/* Max resolution supported */
	if (mf->width > 640)
		mf->width = 640;
	if (mf->height > 480)
		mf->height = 480;

	/* Set YUV by default*/
	if (mf->code != V4L2_MBUS_FMT_UYVY8_2X8 &&
	    mf->code != V4L2_MBUS_FMT_SRGGB8_1X8 &&
	    mf->code != V4L2_MBUS_FMT_SRGGB10_1X10)
		mf->code = V4L2_MBUS_FMT_UYVY8_2X8;

	mf->colorspace	= V4L2_COLORSPACE_SMPTE170M;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int mt9v117_s_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *mf)
{
	struct mt9v117 *mt9v117 = to_mt9v117(sd);
	struct v4l2_rect *crop = &mt9v117->crop;
	int ret;

	/* Try format */
	ret = mt9v117_try_mbus_fmt(sd, mf);
	if (ret)
		return ret;

	/* Update format */
	mt9v117->format = *mf;
	mt9v117->timings_uptodate = 0;

	/* Set a centered crop in bayer mode */
	if (mf->code == V4L2_MBUS_FMT_SRGGB8_1X8 ||
	    mf->code == V4L2_MBUS_FMT_SRGGB10_1X10) {
		crop->left = (640 - mf->width) / 2;
		crop->top = (480 - mf->height) / 2;
		crop->width = mf->width;
		crop->height = mf->height;
	}

	/* Update frame period */
	mt9v117_update_frame_period(mt9v117);

	return 0;
}

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

	/* Calculate frame period */
	tmp64 = (u64)fi->interval.numerator * NSEC_PER_SEC;
	mt9v117->frameperiod_ns = div_u64((u64)tmp64, fi->interval.denominator);

	/* Check frame rete */
	mt9v117_update_frame_period(mt9v117);

	/* Update frame rate */
	mt9v117_g_frame_interval(sd, fi);

	/* Update frame rate */
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

static int mt9v117_g_dv_timings(struct v4l2_subdev *sd,
				struct v4l2_dv_timings *timings)
{
	struct mt9v117 *mt9v117 = to_mt9v117(sd);
	struct mt9v117_timings *tmg = &mt9v117->timings;

	/* Reset timings */
	memset(timings, 0, sizeof(struct v4l2_dv_timings));

	/* Set timings value */
	timings->bt.width = mt9v117->format.width;
	timings->bt.height = mt9v117->format.height;
	timings->bt.hsync = tmg->line_length - mt9v117->format.width;
	timings->bt.vsync = tmg->frame_length - mt9v117->format.height;
	timings->bt.pixelclock = mt9v117->pdata->ext_clk_freq_hz;

	/* In bayer mode, pixel clock is divided by 2 */
	if (mt9v117->format.code == V4L2_MBUS_FMT_SRGGB8_1X8 ||
	    mt9v117->format.code == V4L2_MBUS_FMT_SRGGB10_1X10)
		timings->bt.pixelclock /= 2;

	return 0;
}

/* V4L2 pad ops */
static struct v4l2_mbus_framefmt *__mt9v117_get_fmt(
					   struct mt9v117 *mt9v117,
					   struct v4l2_subdev_fh *fh,
					   unsigned int pad,
					   enum v4l2_subdev_format_whence which)
{
	/* Only one pad */
	if(pad)
		return NULL;

	/* Get format */
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

	/* Get format to return */
	format = __mt9v117_get_fmt(mt9v117, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	/* Copy format */
	fmt->format = *format;

	return 0;
}

static int mt9v117_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			  struct v4l2_subdev_format *fmt)
{
	struct mt9v117 *mt9v117 = to_mt9v117(sd);
	struct v4l2_rect *crop = &mt9v117->crop;
	struct v4l2_mbus_framefmt *format;
	int ret;

	/* Get format to set */
	format = __mt9v117_get_fmt(mt9v117, fh, fmt->pad, fmt->which);
	if(!format)
		return -EINVAL;

	/* Try format */
	ret = mt9v117_try_mbus_fmt(sd, &fmt->format);
	if (ret)
		return ret;

	/* Update format */
	*format = fmt->format;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		/* Timings must be recalculated */
		mt9v117->timings_uptodate = 0;

		/* Update framerate */
		mt9v117_update_frame_period(mt9v117);
	}

	/* Set a centered crop in bayer mode */
	if (format->code == V4L2_MBUS_FMT_SRGGB8_1X8 ||
	    format->code == V4L2_MBUS_FMT_SRGGB10_1X10) {
		crop->left = (640 - format->width) / 2;
		crop->top = (480 - format->height) / 2;
		crop->width = format->width;
		crop->height = format->height;
	}

	return 0;
}

static int mt9v117_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	return mt9v117_enum_mbus(sd, code->index, &code->code);
}

static const struct v4l2_subdev_video_ops mt9v117_video_ops = {
	.s_stream = mt9v117_s_stream,
	.enum_mbus_fmt = mt9v117_enum_mbus,
	.try_mbus_fmt = mt9v117_try_mbus_fmt,
	.s_mbus_fmt = mt9v117_s_mbus_fmt,
	.g_mbus_fmt = mt9v117_g_mbus_fmt,
	.g_frame_interval = mt9v117_g_frame_interval,
	.s_frame_interval = mt9v117_s_frame_interval,
	.g_dv_timings = mt9v117_g_dv_timings,
};

static const struct v4l2_subdev_pad_ops mt9v117_pad_ops = {
	.get_fmt = mt9v117_get_fmt,
	.set_fmt = mt9v117_set_fmt,
	.enum_mbus_code = mt9v117_enum_mbus_code,
};

static const struct v4l2_subdev_ops mt9v117_ops = {
	.video = &mt9v117_video_ops,
	.pad = &mt9v117_pad_ops,
};

/*
 * Control handler
 */
static int mt9v117_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mt9v117 *mt9v117 = container_of(ctrl->handler, struct mt9v117,
					       ctrls);
	uint16_t val16;
	uint32_t val;
	int ret;

	/* Get value only when streaming */
	if (!mt9v117->isStreaming)
		return 0;

	/* Get control values */
	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		/* Get AE analog gain */
		if (!mt9v117_get_ae_gain(mt9v117, &val16))
			ctrl->val = val16;
		break;
	case V4L2_CID_EXPOSURE:
		/* Get AE exposure value (frame length) */
		if (!mt9v117_get_ae_exposure(mt9v117, &val16))
			ctrl->val = val16;
		break;
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		/* Get AE exposure time value (in us) */
		if (!mt9v117_get_ae_exposure_us(mt9v117, &val))
			ctrl->val = val;
		break;
	case MT9V117_CID_AUTO_EXPOSURE_ZONE:
		/* Get AE zone */
		ret = mt9v117_get_ae_zone(mt9v117);
		if (ret >= 0 && ret < 3)
			ctrl->val = ret;
		break;
	}

	return 0;
}

static int mt9v117_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mt9v117 *mt9v117 = container_of(ctrl->handler, struct mt9v117,
					       ctrls);
	int ret = 0;

	/* Set control */
	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		/* Set analog gain */
		ctrl->val = mt9v117_set_gain(mt9v117, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		/* Set exposure value (frame length) */
		ctrl->val = mt9v117_set_exposure(mt9v117, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		/* Set AE exposure time value (in us) */
		ctrl->val = mt9v117_set_exposure_us(mt9v117, ctrl->val);
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		/* Set anti-flickering mode */
		ret = mt9v117_set_anti_flickering(mt9v117,
		       ctrl->val == V4L2_CID_POWER_LINE_FREQUENCY_50HZ ? 1 : 0);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static struct v4l2_ctrl_ops mt9v117_ctrl_ops = {
	.g_volatile_ctrl	= mt9v117_g_volatile_ctrl,
	.s_ctrl			= mt9v117_s_ctrl,
};

static const char * const mt9v117_auto_exposure_zone[] = {
	"Bright",
	"Normal",
	"Dark",
	NULL
};

static struct v4l2_ctrl_config mt9v117_ctrls[] = {
	{
		.ops = &mt9v117_ctrl_ops,
		.id = V4L2_CID_GAIN,
		.name = "AE analog gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = (uint16_t)(~0U),
		.def = 0,
		.step = 1,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
	},
	{
		.ops = &mt9v117_ctrl_ops,
		.id = V4L2_CID_EXPOSURE,
		.name = "AE exposure (frame length)",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = (uint16_t)(~0U),
		.def = 0,
		.step = 1,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
	},
	{
		.ops = &mt9v117_ctrl_ops,
		.id = V4L2_CID_EXPOSURE_ABSOLUTE,
		.name = "AE exposure (time in us)",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = (int32_t)((uint32_t)(~0U) >> 1),
		.def = 0,
		.step = 1,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
	},
	{
		.ops = &mt9v117_ctrl_ops,
		.id = MT9V117_CID_AUTO_EXPOSURE_ZONE,
		.name = "AE zone",
		.type = V4L2_CTRL_TYPE_MENU,
		.min = MT9V117_AUTO_EXPOSURE_BRIGHT,
		.max = MT9V117_AUTO_EXPOSURE_DARK,
		.qmenu = mt9v117_auto_exposure_zone,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
	},
};

/*
 * Module probe
 */
static int mt9v117_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct mt9v117 *mt9v117;
	struct mt9v117_platform_data *pdata = NULL;
	struct v4l2_subdev *sd;
	int ret = -ENODEV;
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;
	u16 model_id;
	int i;

	/* Get platfotm data */
	pdata = client->dev.platform_data;

	/* Check platform data */
	if(!pdata)
		return -EINVAL;

	/* Check I2C */
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
		ret = -EIO;
		goto ei2c;
	}

	/* Sensor detected */
	dev_info(&client->dev, "detecting mt9v117 client on address 0x%x\n",
			client->addr << 1);

	/* Allocate private structure */
	mt9v117 = kzalloc(sizeof(*mt9v117), GFP_KERNEL);
	if(!mt9v117) {
		v4l_err(client, "alloc failed for data structure\n");
		ret = -ENOMEM;
		goto enomem;
	}

	/* Fill private structure */
	mt9v117->set_power = pdata->set_power;
	mt9v117->pdata = client->dev.platform_data;
	mt9v117->i2c_client = client;

	/* Set default format: 640x480 @ 30 fps */
	mt9v117->frameperiod_ns = DEFAULT_FRAME_PERIOD_NS;
	format = &mt9v117->format;
	format->width = 640;
	format->height = 480;
	format->field = V4L2_FIELD_NONE;
	format->code = V4L2_MBUS_FMT_UYVY8_2X8;
	format->colorspace = V4L2_COLORSPACE_SMPTE170M;
	crop = &mt9v117->crop;
	crop->left = 0;
	crop->top = 0;
	crop->width = format->width;
	crop->height = format->height;

	/* Init V4L2 subdev */
	sd = &mt9v117->sd;
	v4l2_i2c_subdev_init(sd, client, &mt9v117_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	/* Init media pad */
	mt9v117->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&sd->entity, 1, &mt9v117->pad, 0);
	if(ret < 0) {
		v4l_err(client, "failed to init media entity\n");
		goto emedia;
	}

	/* Init control handler */
	v4l2_ctrl_handler_init(&mt9v117->ctrls, 5);

	/* Create new controls:
	 *    Power Line frequency (only 50Hz or 60Hz)
	 */
	v4l2_ctrl_new_std_menu(&mt9v117->ctrls, &mt9v117_ctrl_ops,
			      V4L2_CID_POWER_LINE_FREQUENCY,
			      V4L2_CID_POWER_LINE_FREQUENCY_60HZ, 1,
			      V4L2_CID_POWER_LINE_FREQUENCY_60HZ);

	/* Add standard controls with volatile behavior:
	 *    Gain control
	 *    Exposure control (frame length)
	 *    Exposure absolute control (time in us)
	 * Add specific MT9V117 controls:
	 *    Auto exposure zone
	 */
	for (i = 0; i < ARRAY_SIZE(mt9v117_ctrls); i++)
		v4l2_ctrl_new_custom(&mt9v117->ctrls, &mt9v117_ctrls[i], NULL);

	/* Add control handler to subdev */
	mt9v117->sd.ctrl_handler = &mt9v117->ctrls;

	/* Power on */
	if(pdata->set_power) {
		pdata->set_power(MT9V117_POWER_ON);
		mt9v117_reset_sleep(mt9v117);
	}

	/* Read model ID of sensor */
	mt9v117_read16(mt9v117, 0x3000, &model_id);

	/* Check model ID */
	if(model_id != 0x2282) {
		v4l2_err(sd, "Error Model ID = 0x%04X\n", model_id);
		ret = -ENODEV;
		goto echipident;
	}

	/* Power off */
	if(pdata->set_power)
		pdata->set_power(MT9V117_POWER_OFF);

	/* Calculate timings for first time */
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

MODULE_AUTHOR("Julien BERAUD <julien.beraud@parrot.com>");
MODULE_AUTHOR("Alexandre Dilly <alexandre.dilly@parrot.com>");
MODULE_DESCRIPTION("Aptina MT9V117 camera driver");
MODULE_LICENSE("GPL");
