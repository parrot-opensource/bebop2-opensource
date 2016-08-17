/**
 ************************************************
 * @file mt9f002.c
 * @brief Driver for Aptina mt9f002
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * @author Karl Leplat <karl.leplat@parrot.com>
 * @date 2014-09-19
 ************************************************
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/gcd.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/log2.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/v4l2-mediabus.h>

#include <media/mt9f002.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-event.h>

#include "mt9f002_regs.h"

#define MT9F002_NEVENTS 8
/* /!\ This value is calculated with a 400kHz clock for I2C */
#define MT9F002_CROP_WRITE_US 3000
#define MT9F002_DEFAULT_USER_US 4000

static inline struct mt9f002 *to_mt9f002(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9f002, subdev);
}

/* write a register */
static int mt9f002_write8(struct mt9f002 *mt9f002, u16 reg, u8 val)
{
	struct i2c_client *client = mt9f002->i2c_client;
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
static int mt9f002_read16(struct mt9f002 *mt9f002, u16 reg, u16 *val)
{
	struct i2c_client *client = mt9f002->i2c_client;
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

static int mt9f002_read8(struct mt9f002 *mt9f002, u16 reg, u8 *val)
{
	struct i2c_client *client = mt9f002->i2c_client;
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
			.len	= 1,
			.buf	= val,
		},
	};

	reg = swab16(reg);

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "Failed reading register 0x%04x!\n", reg);
		return ret;
	}


	return 0;
}

/* write a register */
static int mt9f002_write16(struct mt9f002 *mt9f002, u16 reg, u16 val)
{
	struct i2c_client *client = mt9f002->i2c_client;
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
static int mt9f002_blanking_init(struct mt9f002 *mt9f002)
{
	int res = 0;
	struct mt9f002_context *ctx = &mt9f002->ctx;
	struct mt9f002_blanking *blanking = &ctx->blanking;

	if (blanking->x_odd_inc > 1) {
		if (blanking->y_odd_inc > 1)
		{
			blanking->min_line_blanking_pck =
					       BINNING_XY_MIN_LINE_BLANKING_PCK;
			blanking->min_line_length_pck =
						 BINNING_XY_MIN_LINE_LENGTH_PCK;
			blanking->min_line_fifo_pck =
					  BINNING_XY_MIN_LINE_FIFO_BLANKING_PCK;
		} else {
			blanking->min_line_blanking_pck =
						BINNING_X_MIN_LINE_BLANKING_PCK;
			blanking->min_line_length_pck =
						  BINNING_X_MIN_LINE_LENGTH_PCK;
			blanking->min_line_fifo_pck =
					   BINNING_X_MIN_LINE_FIFO_BLANKING_PCK;
		}
	} else {

		if (ctx->scaler != 16) {
			blanking->min_line_blanking_pck
				= SCALER_MIN_LINE_BLANKING_PCK;
			blanking->min_line_length_pck
				= SCALER_MIN_LINE_LENGTH_PCK;
			blanking->min_line_fifo_pck
				= SCALER_MIN_LINE_FIFO_BLANKING_PCK;
		} else {
			blanking->min_line_blanking_pck
				= MIN_LINE_BLANKING_PCK;
			blanking->min_line_length_pck
				= MIN_LINE_LENGTH_PCK;
			blanking->min_line_fifo_pck
				= MIN_LINE_FIFO_BLANKING_PCK;
		}
	}

	return res;
}

static int mt9f002_pll_check(struct mt9f002 *mt9f002)
{
	uint64_t pll_ip_clk_freq, vco_freq, clk_pixel, clk_op;
	struct mt9f002_platform_data *pdata = mt9f002->pdata;
	struct mt9f002_context *ctx = &mt9f002->ctx;
	struct mt9f002_pll *pll = &ctx->pll;
	int res = 0;

	/*
	 *  check ext_clk_freq_mhz (inputClk) is in the range
	 */
	if (pdata->ext_clk_freq_mhz < EXT_CLK_MIN
			|| pdata->ext_clk_freq_mhz > EXT_CLK_MAX) {
		v4l2_err(&mt9f002->subdev, "error,input clock (%llu MHz) \
				not in the allowed range\n",
				pdata->ext_clk_freq_mhz);
		res = -1;
	}

	/*
	 * check pll_ip_clk_freq is in the range
	 */
	if (pll->pre_pll_clk_div < PRE_PLL_CLK_DIV_MIN
			|| pll->pre_pll_clk_div > PRE_PLL_CLK_DIV_MAX) {
		v4l2_err(&mt9f002->subdev, "error, pre_pll_clk_div (%u) \
				not in the allowed range\n",
				pll->pre_pll_clk_div);
		res = -1;
	}

	pll_ip_clk_freq = div_u64((u64)pdata->ext_clk_freq_mhz,
			pll->pre_pll_clk_div);//(EQ 1)
	if (pll_ip_clk_freq < PLL_INPUT_CLK_MIN
			|| pll_ip_clk_freq > PLL_INPUT_CLK_MAX) {
		v4l2_err(&mt9f002->subdev, "error, pll_ip_clk_freq (%llu) \
				not in the allowed range\n",
				pll_ip_clk_freq);
		res = -1;
	}

	/*
	 *  check VCO is in the range
	 */
	if (pll->pll_multiplier % 2) {
		// odd case
		if (pll->pll_multiplier < PLL_MUL_ODD_MIN ||
				pll->pll_multiplier > PLL_MUL_ODD_MAX) {
			v4l2_err(&mt9f002->subdev, "error, odd pre_pll_clk_div \
					(%d) not in the allowed range\n",
					pll->pll_multiplier);
			res = -1;
		}
	} else {
		// even case
		if (pll->pll_multiplier < PLL_MUL_EVEN_MIN ||
				pll->pll_multiplier > PLL_MUL_EVEN_MAX) {
			v4l2_err(&mt9f002->subdev, "error, even pre_pll_clk_div \
					(%d) not in the allowed range\n",
					pll->pll_multiplier);
			res = -1;
		}
	}

	vco_freq = (u64)pll_ip_clk_freq * pll->pll_multiplier; // (EQ 2)
	if (vco_freq < VCO_CLK_MIN || vco_freq > VCO_CLK_MAX) {
		v4l2_err(&mt9f002->subdev, "error, vco_freq (%llu) \
				not in the allowed range\n",
				vco_freq);
		res = -1;
	}

	/*
	 *  check clk_op < clk_pixel
	 */
	clk_pixel = (u64)pdata->ext_clk_freq_mhz * pll->pll_multiplier *
		(1 + pll->shift_vt_pix_clk_div);

	clk_pixel = div_u64((u64)clk_pixel,
			pll->pre_pll_clk_div * pll->vt_sys_clk_div
		* pll->vt_pix_clk_div * 2 * pll->row_speed_2_0); // (EQ 4)

	clk_op = (u64)pdata->ext_clk_freq_mhz * pll->pll_multiplier;
	clk_op = div_u64((u64)clk_op, pll->pre_pll_clk_div * pll->op_sys_clk_div
		* pll->op_pix_clk_div * pll->row_speed_10_8); // (EQ 5)

	if (clk_op > clk_pixel) {
		v4l2_err(&mt9f002->subdev, "error, clk_op (%llu) runs faster \
				than clk_pixel (%llu)\n",
				clk_op, clk_pixel);
		res = -1;
	}

	/*
	 *  When the serial interface is used the clk_op divider
	 *  cannot be used; row_speed[10:8] must equal 1
	 */
	if (pdata->interface == MT9F002_MIPI ||
			pdata->interface == MT9F002_HiSPi) {
		if (pll->row_speed_10_8 != 1) {
			v4l2_err(&mt9f002->subdev, "error, wrong row_speed_10_8 \
					value %u) intead of 1\n",
					pll->row_speed_10_8);
			res = -1;
		}
	}

	/*
	   The value of op_sys_clk_div must match the bit-depth of the image
	   when using serial interface. R0x0112-3 controls whether the pixel
	   data interface will generate 12, 10, or 8 bits per pixel. When the
	   pixel data interface is generating 8 bits per-pixel, op_pix_clk_div
	   must be programmed with the value 8. When the pixel data interface is
	   generating 10 bits per pixel, op_pix_clk_div must be programmed with
	   the value 10. And when the pixel data interface is generating 12 bits
	   per pixel, op_pix_clk_div must be programmed with the value 12.
	   This is not required when using the parallel inter-face.
	   */
	if (pdata->interface == MT9F002_MIPI
			|| pdata->interface == MT9F002_HiSPi) {
		if (pll->op_pix_clk_div != pdata->pixel_depth) {
			v4l2_err(&mt9f002->subdev, "error, wrong op_pix_clk_div\
				       	expected 0x%x get 0x%x\n",
				       pdata->pixel_depth, pll->op_pix_clk_div);
			res = -1;
		}
	}

	return res;
}

static int mt9f002_pll_conf(struct mt9f002 *mt9f002)
{
	uint32_t pre_pll_clk_div, pll_multiplier;
	uint32_t best_pre_pll_clk_div = 0, best_pll_multiplier = 0;
	uint32_t best_vt_sys_clk_div = 0, best_vt_pix_clk_div = 0;
	uint32_t best_op_sys_clk_div = 0;
	uint32_t best_op_pix_clk_div = 0;
	uint32_t best_row_speed_10_8 = 0;
	uint32_t vt_sys_clk_div_index, vt_pix_clk_div_index,
		 op_sys_clk_div_index;
	uint32_t op_pix_clk_div_index, row_speed_10_8_index;
	int64_t minError = -1;
	struct mt9f002_platform_data *pdata = mt9f002->pdata;
	struct mt9f002_context *ctx = &mt9f002->ctx;
	struct mt9f002_pll *pll = &ctx->pll;
	struct mt9f002_clock *clock = &ctx->clock;

	uint32_t vt_sys_clk_div_tab[] = {1, 2, 4, 6, 8};
	uint32_t vt_pix_clk_div_tab[] = {2, 3, 4, 5, 6, 7, 8};
	uint32_t op_sys_clk_div_tab[] = {1, 2, 4, 6, 8};
	uint32_t op_pix_clk_div_tab[] = {8, 10, 12};
	uint32_t row_speed_10_8_tab[] = {1, 2, 4};

	uint64_t virtual_pll_ip_clk_freq, virtualVCOfreq, virtual_clk_pixel,
		virtual_clk;
	int64_t error;
	uint64_t tmp64;

	// fill constant value
	pll->shift_vt_pix_clk_div = 1;
	// no documentation, set to 1 in the excel sheet
	pll->row_speed_2_0 = 1;
	// don't understand what it is, fix it to 1

	if (pdata->interface == MT9F002_MIPI ||
			pdata->interface == MT9F002_HiSPi) {
		// MIPI/HiSPi has some forbiden values
		row_speed_10_8_tab[0] = row_speed_10_8_tab[1]
			= row_speed_10_8_tab[2] = 1;
		op_pix_clk_div_tab[0] = op_pix_clk_div_tab[1]
			= op_pix_clk_div_tab[2]
			= pdata->pixel_depth;
	}

	// explore all possibilities
	for (pre_pll_clk_div = PRE_PLL_CLK_DIV_MIN;
			pre_pll_clk_div <= PRE_PLL_CLK_DIV_MAX;
			pre_pll_clk_div++) {

		// if pll input clock is out of range, cancel this option
		virtual_pll_ip_clk_freq = div_u64((u64)pdata->ext_clk_freq_mhz,
				pre_pll_clk_div);

		if (virtual_pll_ip_clk_freq > PLL_INPUT_CLK_MAX
			|| virtual_pll_ip_clk_freq < PLL_INPUT_CLK_MIN) {
			continue;
		}

		for (pll_multiplier = PLL_MUL_MIN;
			pll_multiplier <= PLL_MUL_MAX; pll_multiplier++) {
			if (pll_multiplier % 2 == 0) {
				// even mul, check range
				if (pll_multiplier < PLL_MUL_EVEN_MIN
					|| pll_multiplier > PLL_MUL_EVEN_MAX) {
					// out of range
					continue;
				}
			} else {
				// odd mul, check range
				if (pll_multiplier < PLL_MUL_ODD_MIN
					|| pll_multiplier > PLL_MUL_ODD_MAX) {
					// out of range
					continue;
				}
			}

			// check that we remain in the VCO range
			virtualVCOfreq = (u64)virtual_pll_ip_clk_freq
				* pll_multiplier;

			if (virtualVCOfreq < VCO_CLK_MIN
					|| virtualVCOfreq > VCO_CLK_MAX) {
				// VCO out of range, cancel this option
				continue;
			}

			for (vt_sys_clk_div_index = 0; vt_sys_clk_div_index < ARRAY_SIZE(vt_sys_clk_div_tab); vt_sys_clk_div_index++) {
				for (vt_pix_clk_div_index = 0; vt_pix_clk_div_index < ARRAY_SIZE(vt_pix_clk_div_tab); vt_pix_clk_div_index++) {
					for (op_sys_clk_div_index = 0; op_sys_clk_div_index < ARRAY_SIZE(op_sys_clk_div_tab); op_sys_clk_div_index++) {
						for (op_pix_clk_div_index = 0; op_pix_clk_div_index < ARRAY_SIZE(op_pix_clk_div_tab); op_pix_clk_div_index++) {
							for (row_speed_10_8_index = 0; row_speed_10_8_index < ARRAY_SIZE(row_speed_10_8_tab); row_speed_10_8_index++) {

								// check that clk_op <= clk_pixel (from EQ 5 and EQ 4)
								if ((vt_sys_clk_div_tab[vt_sys_clk_div_index]
											* vt_pix_clk_div_tab[vt_pix_clk_div_index] * pll->row_speed_2_0) > (op_sys_clk_div_tab[op_sys_clk_div_index] * op_pix_clk_div_tab[op_pix_clk_div_index] * row_speed_10_8_tab[row_speed_10_8_index])) {
									// with this settings clk_op would be greater than clk_pixel, forget it !
									continue;
								}

								// check that we remain in clk_pixel range
								// try to run clk_pixel as close as possible to 120MHz (max value)
								virtual_clk_pixel = div_u64((u64)virtualVCOfreq,
									vt_sys_clk_div_tab[vt_sys_clk_div_index] * vt_pix_clk_div_tab[vt_pix_clk_div_index]
									* pll->row_speed_2_0); // (EQ 4)

								if (virtual_clk_pixel > PIXEL_CLK_MAX) {
									continue;
								}

								if (pdata->interface == MT9F002_Parallel) {
									// compute pixel clock (named clk_op in the datasheet)
									virtual_clk = div_u64((u64)virtualVCOfreq,
											op_sys_clk_div_tab[op_sys_clk_div_index] * op_pix_clk_div_tab[op_pix_clk_div_index]
											* row_speed_10_8_tab[row_speed_10_8_index]);
								} else {
									// compute serial clock (named op_sys_clk in the datasheet)
									virtual_clk = div_u64((u64)virtualVCOfreq,
											op_sys_clk_div_tab[op_sys_clk_div_index]);
								}

								// if this solution is best we've seen so far, save it
								error = abs64((u64)virtual_clk - pdata->output_clk_freq_mhz);
								if (error < minError || minError < 0) {
									minError = error;
									best_vt_sys_clk_div =  vt_sys_clk_div_index;
									best_vt_pix_clk_div =  vt_pix_clk_div_index;
									best_op_sys_clk_div =  op_sys_clk_div_index;
									best_op_pix_clk_div =  op_pix_clk_div_index;
									best_row_speed_10_8 = row_speed_10_8_index;
									best_pre_pll_clk_div = pre_pll_clk_div;
									best_pll_multiplier = pll_multiplier;
								}
							}
						}
					}
				}
			}
		}
	}

	// check that an option has been found
	if (minError < 0) {
		v4l2_err(&mt9f002->subdev, "pll configuration failure\n");
		return -1;
	}

	// save values
	pll->op_sys_clk_div = op_sys_clk_div_tab[best_op_sys_clk_div];
	pll->vt_sys_clk_div = vt_sys_clk_div_tab[best_vt_sys_clk_div];
	pll->vt_pix_clk_div = vt_pix_clk_div_tab[best_vt_pix_clk_div];
	pll->op_pix_clk_div = op_pix_clk_div_tab[best_op_pix_clk_div];
	pll->row_speed_10_8 = row_speed_10_8_tab[best_row_speed_10_8];
	pll->pll_multiplier = best_pll_multiplier;
	pll->pre_pll_clk_div = best_pre_pll_clk_div;

	// compute&save some relevant clock values
	// compute vt_pix_clk (EQ 3)
	tmp64 = (u64)pdata->ext_clk_freq_mhz * pll->pll_multiplier *
		(1 + pll->shift_vt_pix_clk_div);
	tmp64 = div_u64((u64)tmp64,
			pll->pre_pll_clk_div * pll->vt_sys_clk_div *
			pll->vt_pix_clk_div);
	clock->vt_pix_clk = div_u64((u64)tmp64, 1000000);

	/* Update pixel rate control */
	*mt9f002->pixel_rate->p_cur.p_s64 = div_u64((u64)tmp64, 1000L);

	// compute op_pix_clk
	tmp64 = (u64)pdata->ext_clk_freq_mhz * pll->pll_multiplier;
	tmp64 = div_u64((u64)tmp64,
			pll->pre_pll_clk_div * pll->op_sys_clk_div *
			pll->op_pix_clk_div);
	clock->op_pix_clk = div_u64((u64)tmp64, 1000000);

	return 0;
}

static int mt9f002_pll_write(struct mt9f002 *mt9f002)
{
	int res = 0;
	uint16_t smia;
	uint16_t row_speed;
	struct mt9f002_context *ctx = &mt9f002->ctx;
	struct mt9f002_pll *pll = &ctx->pll;

	res |= mt9f002_write16(mt9f002, MT9F002_VT_PIX_CLK_DIV,
			pll->vt_pix_clk_div);
	res |= mt9f002_write16(mt9f002, MT9F002_VT_SYS_CLK_DIV,
			pll->vt_sys_clk_div);
	res |= mt9f002_write16(mt9f002, MT9F002_PRE_PLL_CLK_DIV,
			pll->pre_pll_clk_div);
	res |= mt9f002_write16(mt9f002, MT9F002_PLL_MULTIPLIER,
			pll->pll_multiplier);
	res |= mt9f002_write16(mt9f002, MT9F002_OP_PIX_CLK_DIV,
			pll->op_pix_clk_div);
	res |= mt9f002_write16(mt9f002, MT9F002_OP_SYS_CLK_DIV,
			pll->op_sys_clk_div);

	res |= mt9f002_read16(mt9f002, MT9F002_SMIA_TEST, &smia);
	smia = (smia & 0xFFBF) | ((pll->shift_vt_pix_clk_div & 0x01) << 6);
	res |= mt9f002_write16(mt9f002, MT9F002_SMIA_TEST  , smia);

	res |= mt9f002_read16(mt9f002, MT9F002_ROW_SPEED, &row_speed);
	row_speed = (row_speed & 0xFFF8) | (pll->row_speed_2_0 & 0x07);
	row_speed = (row_speed & 0xF8FF) | ((pll->row_speed_10_8 & 0x07) << 8);
	// from ASIC team : change opclk_delay
	row_speed = (row_speed & (~0x70)) | (0x2 << 4);
	res |= mt9f002_write16(mt9f002, MT9F002_ROW_SPEED  , row_speed);

	return res;
}

static int mt9f002_parallel_stage1_conf(struct mt9f002 *mt9f002)
{
	int res = 0;

	res |= mt9f002_write16(mt9f002, MT9F002_RESET_REGISTER,		0x0010);
	res |= mt9f002_write16(mt9f002, MT9F002_RESET_REGISTER,		0x0010);
	res |= mt9f002_write16(mt9f002, MT9F002_RESET_REGISTER,		0x0010);
	res |= mt9f002_write16(mt9f002, MT9F002_RESET_REGISTER,		0x0010);
	res |= mt9f002_write16(mt9f002, MT9F002_DAC_LD_14_15,		0xE525);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_CONTROL_REG,	0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xF873);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG, 	0x08AA);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG, 	0x3219);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG, 	0x3219);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x3219);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x3200);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x3200);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x3200);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x3200);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x3200);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x1769);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xA6F3);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xA6F3);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xA6F3);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xA6F3);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xA6F3);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xA6F3);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xA6F3);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xAFF3);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xFA64);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xFA64);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xFA64);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xF164);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xFA64);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xFA64);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xFA64);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xF164);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x276E);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x18CF);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x18CF);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x18CF);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x28CF);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x18CF);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x18CF);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x18CF);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x18CF);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x2363);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x2363);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x2352);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x2363);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x2363);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x2363);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x2352);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x2352);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xA394);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xA394);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x8F8F);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xA3D4);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xA394);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xA394);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x8F8F);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x8FCF);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xDC23);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xDC63);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xDC63);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xDC23);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xDC23);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xDC63);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xDC63);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0xDC23);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x0F73);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x85C0);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x85C0);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x85C0);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x85C0);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x85C0);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x85C0);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x85C0);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x85C4);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_WR_DATA_REG,	0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_ANALOG_CONTROL4,	0x8000);
	res |= mt9f002_write16(mt9f002, MT9F002_DAC_LD_14_15   ,	0xE525);
	res |= mt9f002_write16(mt9f002, MT9F002_DATA_PEDESTAL_ ,	0x00A8);
	res |= mt9f002_write16(mt9f002, MT9F002_RESET_REGISTER ,	0x0090);
	res |= mt9f002_write16(mt9f002, MT9F002_SERIAL_FORMAT  ,	0x0301);
	res |= mt9f002_write16(mt9f002, MT9F002_RESET_REGISTER ,	0x1090);
	res |= mt9f002_write16(mt9f002, MT9F002_SMIA_TEST      ,	0x0845);
	res |= mt9f002_write16(mt9f002, MT9F002_RESET_REGISTER ,	0x1080);
	res |= mt9f002_write16(mt9f002, MT9F002_DATAPATH_SELECT,	0xD880);
	res |= mt9f002_write16(mt9f002, MT9F002_RESET_REGISTER ,	0x9080);
	res |= mt9f002_write16(mt9f002, MT9F002_DATAPATH_SELECT,	0xD880);
	res |= mt9f002_write16(mt9f002, MT9F002_RESET_REGISTER ,	0x10C8);
	res |= mt9f002_write16(mt9f002, MT9F002_DATAPATH_SELECT,	0xD880);

	return res;
}

static int mt9f002_parallel_stage2_conf(struct mt9f002 *mt9f002)
{
	int res = 0;
	struct mt9f002_context *ctx = &mt9f002->ctx;
	struct mt9f002_blanking *blanking = &ctx->blanking;

	res |= mt9f002_write16(mt9f002, MT9F002_ANALOG_CONTROL4,	0x8000);
	res |= mt9f002_write16(mt9f002, MT9F002_READ_MODE,		0x0041);
	res |= mt9f002_write16(mt9f002, MT9F002_READ_MODE      ,	0x04C3);
	res |= mt9f002_write16(mt9f002, MT9F002_READ_MODE      ,	0x04C3);
	res |= mt9f002_write16(mt9f002, MT9F002_ANALOG_CONTROL5,	0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_ANALOG_CONTROL5,	0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_ANALOG_CONTROL5,	0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_ANALOG_CONTROL5,	0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_DAC_LD_28_29   ,	0x0047);
	res |= mt9f002_write16(mt9f002, MT9F002_COLUMN_CORRECTION,	0xB080);
	res |= mt9f002_write16(mt9f002, MT9F002_COLUMN_CORRECTION,	0xB100);
	res |= mt9f002_write16(mt9f002, MT9F002_DARK_CONTROL3  ,	0x0020);
	res |= mt9f002_write16(mt9f002, MT9F002_DAC_LD_24_25   ,	0x6349);
	res |= mt9f002_write16(mt9f002, MT9F002_ANALOG_CONTROL7,	0x800A);
	res |= mt9f002_write16(mt9f002, MT9F002_RESET_REGISTER ,	0x90C8);
	res |= mt9f002_write16(mt9f002, MT9F002_CTX_CONTROL_REG,	0x8005);
	res |= mt9f002_write16(mt9f002, MT9F002_ANALOG_CONTROL7,	0x800A);
	res |= mt9f002_write16(mt9f002, MT9F002_DAC_LD_28_29   ,	0x0047);
	res |= mt9f002_write16(mt9f002, MT9F002_DAC_LD_30_31   ,	0x15F0);
	res |= mt9f002_write16(mt9f002, MT9F002_DAC_LD_30_31   ,	0x15F0);
	res |= mt9f002_write16(mt9f002, MT9F002_DAC_LD_30_31   ,	0x15F0);
	res |= mt9f002_write16(mt9f002, MT9F002_DAC_LD_28_29   ,	0x0047);
	res |= mt9f002_write16(mt9f002, MT9F002_DAC_LD_28_29   ,	0x0047);
	res |= mt9f002_write16(mt9f002, MT9F002_RESET_REGISTER ,	0x10C8);
	res |= mt9f002_write16(mt9f002, MT9F002_COARSE_INTEGRATION_TIME,0x08C3);
	res |= mt9f002_write16(mt9f002, MT9F002_DIGITAL_TEST   ,	0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_DATAPATH_SELECT,	0xd880);

	if (blanking->x_odd_inc > 1 &&
			blanking->y_odd_inc > 1) {
		res |= mt9f002_write16(mt9f002 , MT9F002_READ_MODE,
				0x00441);
		res |= mt9f002_write16(mt9f002 , MT9F002_X_ODD_INC,
				blanking->x_odd_inc);
		res |= mt9f002_write16(mt9f002 , MT9F002_Y_ODD_INC,
				blanking->y_odd_inc);

		// bayer resampling
		res |= mt9f002_write16(mt9f002,
					MT9F002_SCALING_MODE,
					2);
		res |= mt9f002_write16(mt9f002,
					MT9F002_DATAPATH_SELECT,
					0xd8b0);

	} else {
		res |= mt9f002_write16(mt9f002,
				MT9F002_READ_MODE,
				0x0041);
		res |= mt9f002_write16(mt9f002,
				MT9F002_X_ODD_INC,
				0x0001);
		res |= mt9f002_write16(mt9f002,
			       	MT9F002_Y_ODD_INC,
				0x0001);
	}

	res |= mt9f002_write8(mt9f002,
			MT9F002_MASK_CORRUPTED_FRAME,
			0x01);

	return res;
}

static int mt9f002_mipi_stage1_conf(struct mt9f002 *mt9f002)
{
	uint32_t serialFormat, dataFormat;
	struct mt9f002_platform_data *pdata = mt9f002->pdata;
	int res = 0;

	res |= mt9f002_write16(mt9f002, MT9F002_RESET_REGISTER   , 0x0118);
	res |= mt9f002_write8(mt9f002, MT9F002_MODE_SELECT      , 0x00);
	if (pdata->interface == MT9F002_HiSPi) {
		serialFormat = (3 << 8) | pdata->number_of_lanes;
	} else {
		serialFormat = (2 << 8) | pdata->number_of_lanes;
	}

	res |= mt9f002_write16(mt9f002, MT9F002_SERIAL_FORMAT, serialFormat);
	dataFormat = (pdata->pixel_depth << 8) | pdata->pixel_depth;
	res |= mt9f002_write16(mt9f002, MT9F002_CCP_DATA_FORMAT  , dataFormat);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D00, 0x0435);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D02, 0x435D);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D04, 0x6698);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D06, 0xFFFF);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D08, 0x7783);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D0A, 0x101B);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D0C, 0x732C);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D0E, 0x4230);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D10, 0x5881);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D12, 0x5C3A);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D14, 0x0140);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D16, 0x2300);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D18, 0x815F);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D1A, 0x6789);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D1C, 0x5920);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D1E, 0x0C20);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D20, 0x21C0);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D22, 0x4684);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D24, 0x4892);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D26, 0x1A00);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D28, 0xBA4C);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D2A, 0x8D48);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D2C, 0x4641);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D2E, 0x408C);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D30, 0x4784);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D32, 0x4A87);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D34, 0x561A);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D36, 0x00A5);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D38, 0x1A00);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D3A, 0x5693);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D3C, 0x4D8D);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D3E, 0x4A47);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D40, 0x4041);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D42, 0x8200);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D44, 0x24B7);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D46, 0x0024);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D48, 0x8D4F);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D4A, 0x831A);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D4C, 0x00B4);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D4E, 0x4684);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D50, 0x49CE);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D52, 0x4946);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D54, 0x4140);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D56, 0x9247);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D58, 0x844B);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D5A, 0xCE4B);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D5C, 0x4741);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D5E, 0x502F);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D60, 0xBD3A);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D62, 0x5181);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D64, 0x5E73);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D66, 0x7C0A);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D68, 0x7770);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D6A, 0x8085);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D6C, 0x6A82);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D6E, 0x6742);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D70, 0x8244);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D72, 0x831A);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D74, 0x0099);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D76, 0x44DF);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D78, 0x1A00);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D7A, 0x8542);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D7C, 0x8567);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D7E, 0x826A);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D80, 0x857C);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D82, 0x6B80);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D84, 0x7000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D86, 0xB831);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D88, 0x40BE);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D8A, 0x6700);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D8C, 0x0CBD);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D8E, 0x4482);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D90, 0x7898);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D92, 0x7480);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D94, 0x5680);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D96, 0x9755);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D98, 0x8057);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D9A, 0x8056);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D9C, 0x9256);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3D9E, 0x8057);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DA0, 0x8055);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DA2, 0x817C);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DA4, 0x969B);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DA6, 0x56A6);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DA8, 0x44BE);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DAA, 0x000C);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DAC, 0x867A);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DAE, 0x9474);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DB0, 0x8A79);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DB2, 0x9367);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DB4, 0xBF6A);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DB6, 0x816C);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DB8, 0x8570);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DBA, 0x836C);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DBC, 0x826A);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DBE, 0x8245);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DC0, 0xFFFF);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DC2, 0xFFD6);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DC4, 0x4582);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DC6, 0x6A82);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DC8, 0x6C83);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DCA, 0x7000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DCC, 0x8024);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DCE, 0xB181);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DD0, 0x6859);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DD2, 0x732B);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DD4, 0x4030);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DD6, 0x4982);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DD8, 0x101B);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DDA, 0x4083);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DDC, 0x6785);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DDE, 0x3A00);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DE0, 0x8820);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DE2, 0x0C59);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DE4, 0x8546);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DE6, 0x8348);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DE8, 0xD04C);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DEA, 0x8B48);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DEC, 0x4641);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DEE, 0x4083);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DF0, 0x1A00);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DF2, 0x8347);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DF4, 0x824A);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DF6, 0x9A56);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DF8, 0x1A00);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DFA, 0x951A);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DFC, 0x0056);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3DFE, 0x914D);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E00, 0x8B4A);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E02, 0x4700);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E04, 0x0300);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E06, 0x2492);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E08, 0x0024);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E0A, 0x8A1A);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E0C, 0x004F);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E0E, 0xB446);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E10, 0x8349);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E12, 0xB249);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E14, 0x4641);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E16, 0x408B);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E18, 0x4783);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E1A, 0x4BDB);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E1C, 0x4B47);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E1E, 0x4180);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E20, 0x502B);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E22, 0x4C3A);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E24, 0x4180);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E26, 0x737C);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E28, 0xD124);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E2A, 0x9068);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E2C, 0x8A20);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E2E, 0x2170);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E30, 0x8081);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E32, 0x6A67);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E34, 0x4257);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E36, 0x5544);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E38, 0x8644);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E3A, 0x9755);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E3C, 0x5742);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E3E, 0x676A);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E40, 0x807D);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E42, 0x3180);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E44, 0x7000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E46, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E48, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E4A, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E4C, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E4E, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E50, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E52, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E54, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E56, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E58, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E5A, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E5C, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E5E, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E60, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E62, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E64, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E66, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E68, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E6A, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E6C, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E6E, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E70, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E72, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E74, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E76, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E78, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E7A, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E7C, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E7E, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E80, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E82, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E84, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E86, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E88, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E8A, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E8C, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E8E, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E90, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E92, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E94, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E96, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E98, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E9A, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E9C, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3E9E, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EA0, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EA2, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EA4, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EA6, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EA8, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EAA, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EAC, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EAE, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EB0, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EB2, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EB4, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EB6, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EB8, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EBA, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EBC, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EBE, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EC0, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EC2, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EC4, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EC6, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EC8, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3ECA, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3176, 0x4000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_317C, 0xA00A);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EE6, 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3ED8, 0xE0E0);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EE8, 0x0001);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3064, 0x0005);

	return res;
}

static int mt9f002_mipi_stage2_conf(struct mt9f002 *mt9f002)
{
	int res = 0;

	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3064, 0x0045);
	return res;
}

static int mt9f002_mipi_stage3_conf(struct mt9f002 *mt9f002)
{
	int res = 0;

	res |= mt9f002_write16(mt9f002, MT9F002_EXTRA_DELAY      , 0x0000);
	res |= mt9f002_write16(mt9f002, MT9F002_RESET_REGISTER   , 0x0118);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EDC, 0x68CF);
	res |= mt9f002_write16(mt9f002, MT9F002_RESERVED_MFR_3EE2, 0xE363);

	return res;
}

static void mt9f002_power_on(struct mt9f002 *mt9f002)
{
	if (mt9f002->set_power) {
		mt9f002->set_power(1);
		msleep(500);
	}
}

static void mt9f002_power_off(struct mt9f002 *mt9f002)
{
	if (mt9f002->set_power) {
		mt9f002->set_power(0);
	}
}

static int __mt9f002_set_power(struct mt9f002 *mt9f002, bool on)
{
	int ret;

	if (!on) {
		mt9f002_power_off(mt9f002);
		return 0;
	}

	mt9f002_power_on(mt9f002);

	/* Reset the chip and stop data read out */
	ret = mt9f002_write8(mt9f002, MT9F002_SOFTWARE_RESET, 1);
	msleep(500);
	if (ret < 0) {
		v4l2_err(&mt9f002->subdev, "Failed to reset the camera\n");
		return ret;
	}

	return mt9f002_write8(mt9f002, MT9F002_SOFTWARE_RESET, 0);
}

static int mt9f002_set_power(struct v4l2_subdev *subdev, int on)
{
	struct mt9f002 *mt9f002 = to_mt9f002(subdev);
	int ret = 0;

	mutex_lock(&mt9f002->power_lock);
	/* If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (mt9f002->power_count == !on) {
		ret = __mt9f002_set_power(mt9f002, !!on);
		if (ret < 0)
			goto done;
	}

	/* Update the power count. */
	mt9f002->power_count += on ? 1 : -1;
	WARN_ON(mt9f002->power_count < 0);

done:
	mutex_unlock(&mt9f002->power_lock);

	return ret;
}

static int mt9f002_pll_setup(struct mt9f002 *mt9f002)
{
	int ret;
	ret = mt9f002_pll_conf(mt9f002);
	if (ret < 0) {
		v4l2_err(&mt9f002->subdev, "failed to configure sensor pll\n");
		return -ENODEV;
	}

	ret = mt9f002_pll_check(mt9f002);
	if (ret < 0) {
		v4l2_err(&mt9f002->subdev, "pll config checking failed\n");
		return -ENODEV;
	}

	return 0;
}

static int mt9f002_pos_write(struct mt9f002 *mt9f002)
{
	struct v4l2_rect *crop = &mt9f002->crop;
	int ret;

	//Set window pos
	ret = mt9f002_write16(mt9f002, MT9F002_X_ADDR_START, crop->left);
	if (ret < 0)
		return ret;
	ret = mt9f002_write16(mt9f002, MT9F002_Y_ADDR_START, crop->top);
	if (ret < 0)
		return ret;
	ret = mt9f002_write16(mt9f002, MT9F002_X_ADDR_END,
			crop->left + crop->width - 1);
	if (ret < 0)
		return ret;
	ret = mt9f002_write16(mt9f002, MT9F002_Y_ADDR_END,
			crop->top + crop->height - 1);

	return ret;
}

static void mt9f002_exposure_max(struct mt9f002 *mt9f002)
{
	u64 tmp64;
	struct mt9f002_context *ctx = &mt9f002->ctx;
	struct mt9f002_exposure *exposure = &ctx->exposure;
	struct mt9f002_blanking *blanking = &ctx->blanking;
	struct mt9f002_clock *clock = &ctx->clock;

	uint32_t max_fine_integration_time =
		(blanking->line_length
		 - exposure->fine_integration_time_max_margin);
	uint32_t max_coarse_integration_time =
		(blanking->frame_length
		 - exposure->coarse_integration_time_max_margin);

	tmp64 = ((u64)max_coarse_integration_time
			* blanking->line_length
			+ max_fine_integration_time) * 1000;
	ctx->maxExposure_us = div_u64((u64)tmp64,  clock->vt_pix_clk);
}

static int mt9f002_res_write(struct mt9f002 *mt9f002)
{
	struct mt9f002_context *ctx = &mt9f002->ctx;
	struct v4l2_mbus_framefmt *format = &mt9f002->format;
	int ret;

	//Set output resolution
	ret = mt9f002_write16(mt9f002, MT9F002_X_OUTPUT_SIZE, format->width);
	if (ret < 0)
		return ret;
	ret = mt9f002_write16(mt9f002, MT9F002_Y_OUTPUT_SIZE, format->height);
	/* scaler */
	if (ctx->scaler != 16) {
		// enable scaling mode
		ret |= mt9f002_write16(mt9f002, MT9F002_SCALING_MODE, 2);
		ret |= mt9f002_write16(mt9f002, MT9F002_SCALE_M, ctx->scaler);
	}

	return ret;
}

static int mt9f002_blanking_conf(struct mt9f002 *mt9f002)
{
	int i, res = 0;
	struct mt9f002_context *ctx = &mt9f002->ctx;
	struct mt9f002_blanking *blanking = &ctx->blanking;
	struct mt9f002_timing *timings = &ctx->timings;
	struct mt9f002_clock *clock = &ctx->clock;
	struct v4l2_rect *crop = &mt9f002->crop;
	struct v4l2_mbus_framefmt *format = &mt9f002->format;
	struct mt9f002_platform_data *pdata = mt9f002->pdata;
	uint32_t clkRatio_num, clkRatio_den;
	uint32_t minHBlkStep, fpgaCorrection;
	uint64_t tmp64;

	uint32_t subsamplingX_factor = (uint32_t)(1 + blanking->x_odd_inc)
		/ (uint32_t)2;
	uint32_t subsamplingY_factor = (uint32_t)(1 + blanking->y_odd_inc)
		/ (uint32_t)2;
	uint32_t minimum_line_length;

	// compute minimum line length
	minimum_line_length = blanking->min_line_length_pck;
	minimum_line_length = max(minimum_line_length, (crop->width - 1 +
					blanking->x_odd_inc)
				/ subsamplingX_factor
				+ blanking->min_line_blanking_pck);

	if (pdata->interface == MT9F002_MIPI ||
			pdata->interface == MT9F002_HiSPi) {
		minimum_line_length = max(minimum_line_length,
					((uint32_t)((uint32_t)format->width
						* clock->vt_pix_clk
						/ clock->op_pix_clk)
					/ pdata->number_of_lanes)
					+ blanking->min_line_fifo_pck);
	} else {
		minimum_line_length = max(minimum_line_length,
			((uint32_t)((uint32_t)format->width
				* clock->vt_pix_clk / clock->op_pix_clk))
			+ blanking->min_line_fifo_pck);

	}

	// first we compute clkRatio fraction
	clkRatio_num = ctx->pll.op_sys_clk_div * ctx->pll.op_pix_clk_div
		* ctx->pll.row_speed_10_8
		* (1 + ctx->pll.shift_vt_pix_clk_div);
	clkRatio_den = ctx->pll.vt_sys_clk_div * ctx->pll.vt_pix_clk_div;

	// reduce fraction (lazy way)
	i = clkRatio_den;
	while (i > 1) {
		if (((clkRatio_den % i) == 0) && ((clkRatio_num % i) == 0)) {
			clkRatio_den = clkRatio_den / i;
			clkRatio_num = clkRatio_num / i;
		}
		i--;
	}

	// then we adjust length_line to meet all the requirement
	// fpga length_line must me divisible by 2 (fpga constraint)
	// and sensor lenght_line by clkRatio_den
	// (so that length_line is an integer in vt based and op based)
	minHBlkStep = clkRatio_num;
	if (clkRatio_den % 2 != 0) {
		minHBlkStep *= 2;
	}

	fpgaCorrection = (minimum_line_length%minHBlkStep);
	if (fpgaCorrection) {
		minimum_line_length += minHBlkStep-fpgaCorrection;
	}

	// save  minimum line length
	blanking->minimum_line_length = minimum_line_length;

	// compute minimum frame length
	blanking->minimum_frame_length_lines = crop->height
		/ subsamplingY_factor + blanking->min_frame_blanking_lines;
	// (EQ 10)
	// compute max framerate
	tmp64 = (u64)blanking->minimum_line_length *
		blanking->minimum_frame_length_lines * NSEC_PER_SEC;
	ctx->max_frameperiod_ns = div_u64((u64)tmp64, clock->vt_pix_clk * 1000);

	if (ctx->frameperiod_ns < ctx->max_frameperiod_ns) {
		blanking->frame_length = blanking->minimum_frame_length_lines;
		blanking->line_length = blanking->minimum_line_length;
	} else {
#ifdef CONFIG_VIDEO_MT9F002_ACCURATE_FRAME_RATE
		// look for the best line_length/frame_length_lines couple to
		// reach desired framerate
		uint32_t best_line_length = 0, best_frame_length_lines = 0;
		uint32_t line_length, frame_length_lines;
		uint64_t max_length_line, num, den;
		uint64_t minError = ULLONG_MAX;
		uint64_t error;

		/* Calculate constants */
		num = (u64) ctx->frameperiod_ns * clock->vt_pix_clk;
		max_length_line = div64_u64((u64) num,
				    (u64) blanking->minimum_frame_length_lines *
					  USEC_PER_SEC);

		/* Find best value for each line length */
		for (line_length = blanking->minimum_line_length;
		     line_length <= max_length_line;
		     line_length += minHBlkStep) {
			/* Find best frame length */
			den = (u64) line_length * USEC_PER_SEC;
			frame_length_lines = div64_u64(num + (den / 2), den);

			/* Calculate error */
			error = abs64(((u64) frame_length_lines * line_length *
					     USEC_PER_SEC) - num);

			/* A new best has been found */
			if (error < minError) {
				minError = error;
				best_line_length = line_length;
				best_frame_length_lines = frame_length_lines;
			}
		}

		// check that an option has been found, should not happen
		if (minError < 0) {
			v4l2_err(&mt9f002->subdev, "error, failed to configure fps\n");
			return -1;
		}

		// save config
		blanking->line_length = best_line_length;
		blanking->frame_length = best_frame_length_lines;
#else
		/* Calculate frame length with minimal HBLANK */
		blanking->line_length = blanking->minimum_line_length;
		blanking->frame_length = div64_u64((u64) clock->vt_pix_clk *
							 ctx->frameperiod_ns,
						   (u64) blanking->line_length *
							 USEC_PER_SEC);
#endif
	}

	/* Calculate blanking length in frames */
	ctx->prev_timings = *timings;
	tmp64 = (u64) blanking->line_length * 1000LL *
		      (u64) (crop->height / subsamplingY_factor);
	timings->frameLength_us = (u32) div_u64(tmp64, clock->vt_pix_clk);
	tmp64 = (u64) blanking->line_length * 1000LL *
		      MT9F002_MIN_FRAME_BLANKING_LINES_DEF;
	timings->blkStartHV_us = (u32) div_u64(tmp64, clock->vt_pix_clk);
	tmp64 = (u64) blanking->line_length * 1000LL *
		      (u64) (blanking->frame_length -
			     (crop->height / subsamplingY_factor) -
			     MT9F002_MIN_FRAME_BLANKING_LINES_DEF);
	timings->blkExtraHV_us = (u32) div_u64(tmp64, clock->vt_pix_clk);

	/* Update H/VBLANK controls */
	mt9f002->hblank->cur.val = blanking->line_length -
				   (crop->width / subsamplingX_factor);
	mt9f002->vblank->cur.val = blanking->frame_length -
				   (crop->height / subsamplingY_factor);

	mt9f002_exposure_max(mt9f002);

	return res;
}

static int mt9f002_exposure_init(struct mt9f002 *mt9f002)
{
	int res = 0;
	struct mt9f002_context *ctx = &mt9f002->ctx;
	struct mt9f002_exposure *exposure = &ctx->exposure;
	struct mt9f002_blanking *blanking = &ctx->blanking;

	if (blanking->x_odd_inc > 1) {
		if (blanking->y_odd_inc > 1) {
			exposure->fine_integration_time_min =
					   BINNING_XY_FINE_INTEGRATION_TIME_MIN;
			exposure->fine_integration_time_max_margin =
				    BINNING_XY_FINE_INTEGRATION_TIME_MAX_MARGIN;
		} else {
			exposure->fine_integration_time_min =
					    BINNING_X_FINE_INTEGRATION_TIME_MIN;
			exposure->fine_integration_time_max_margin =
				     BINNING_X_FINE_INTEGRATION_TIME_MAX_MARGIN;
		}
	} else {

		if (ctx->scaler != 16) {
			exposure->fine_integration_time_max_margin =
				SCALER_FINE_INTEGRATION_TIME_MAX_MARGIN;
			exposure->fine_integration_time_min =
				SCALER_FINE_INTEGRATION_TIME_MIN;
		} else {
			exposure->fine_integration_time_min =
				FINE_INTEGRATION_TIME_MIN;
			exposure->fine_integration_time_max_margin =
				FINE_INTEGRATION_TIME_MAX_MARGIN;
		}
	}

	exposure->coarse_integration_time_min =
						COARSE_INTEGRATION_TIME_MIN_VAL;
	exposure->coarse_integration_time_max_margin =
					 COARSE_INTEGRATION_TIME_MAX_MARGIN_VAL;

	return res;
}

static int mt9f002_blanking_write(struct mt9f002 *mt9f002)
{
	int res;
	struct mt9f002_context *ctx = &mt9f002->ctx;
	struct mt9f002_blanking *blanking = &ctx->blanking;

	// write blanking
	res = mt9f002_write16(mt9f002,
			MT9F002_LINE_LENGTH_PCK,
			blanking->line_length);
	res |= mt9f002_write16(mt9f002,
			MT9F002_FRAME_LENGTH_LINES,
			blanking->frame_length);

	return res;
}

static int mt9f002_exposure_conf(struct mt9f002 *mt9f002)
{
	int res = 0;
	u64 tmp64;
	uint32_t integration;
	struct mt9f002_context *ctx = &mt9f002->ctx;
	struct mt9f002_exposure *exposure = &ctx->exposure;
	struct mt9f002_blanking *blanking = &ctx->blanking;
	struct mt9f002_clock *clock = &ctx->clock;
	u32 *exposure_us = &mt9f002->target_exposure_us->val;
	u32 real_exposure_us;

	// compute fine and coarse integration time
	tmp64 = (u64)*exposure_us * clock->vt_pix_clk;
	integration = div_u64((u64)tmp64, 1000);
	exposure->coarse_integration_time = integration / blanking->line_length;
	exposure->fine_integration_time = integration % blanking->line_length;

	// fine integration must be in a specific range, round exposure if it's not the case
	if ((exposure->fine_integration_time_min > exposure->fine_integration_time) ||
			(exposure->fine_integration_time >
			 (blanking->line_length -
			  exposure->fine_integration_time_max_margin))) {
		// test upper and lower value, take the closest
		int32_t upper_coarse_integration_time =
			exposure->coarse_integration_time + 1;
		int32_t upper_fine_integration_time =
			exposure->fine_integration_time_min;

		int32_t lower_coarse_integration_time =
			exposure->coarse_integration_time - 1;
		int32_t lower_fine_integration_time =
			blanking->line_length -
			exposure->fine_integration_time_max_margin;

		// test saturation
		if (lower_coarse_integration_time < 0) {
			// lower case is negative, reject it
			exposure->coarse_integration_time =
				upper_coarse_integration_time;
			exposure->fine_integration_time =
				upper_fine_integration_time;
		}
		else if (upper_coarse_integration_time >
				(blanking->frame_length -
				 exposure->coarse_integration_time_max_margin)) {
			// upper case is to high, reject it
			exposure->coarse_integration_time = lower_coarse_integration_time;
			exposure->fine_integration_time = lower_fine_integration_time;
		} else {
			// both case are correct values

			// which one is better, upper or lower case ?
			int32_t upper_integration = blanking->line_length *
				upper_coarse_integration_time +
				upper_fine_integration_time;
			int32_t lower_integration = blanking->line_length *
				lower_coarse_integration_time +
				lower_fine_integration_time;

			// compute error
			int32_t upper_error = upper_integration - integration;
			int32_t lower_error = integration - lower_integration;

			// choose the best
			if (upper_error < lower_error) {
				// take upper case
				exposure->coarse_integration_time =
					upper_coarse_integration_time;
				exposure->fine_integration_time =
					upper_fine_integration_time;
			} else {
				// take lower case
				exposure->coarse_integration_time =
					lower_coarse_integration_time;
				exposure->fine_integration_time =
					lower_fine_integration_time;
			}
		}
	}

	// check limit
	if (exposure->fine_integration_time_min > exposure->fine_integration_time) {
		v4l2_warn(&mt9f002->subdev, "warning, fine_integration_time too low (bottom limit %d)\n",
				exposure->fine_integration_time_min);
		exposure->fine_integration_time = exposure->fine_integration_time_min;
	}

	if (exposure->fine_integration_time >
			(blanking->line_length -
			 exposure->fine_integration_time_max_margin)) {
		v4l2_warn(&mt9f002->subdev, "warning, fine_integration_time too high (upper limit %d)\n",
				(blanking->line_length -
				 exposure->fine_integration_time_max_margin));
		exposure->fine_integration_time = (blanking->line_length -
				exposure->fine_integration_time_max_margin);
	}

	if (exposure->coarse_integration_time_min > exposure->coarse_integration_time) {
		v4l2_warn(&mt9f002->subdev, "warning, coarse_integration_time too low (bottom limit %d)\n",
				exposure->coarse_integration_time_min);
		exposure->coarse_integration_time = exposure->coarse_integration_time_min;
	}

	if (exposure->coarse_integration_time > (blanking->frame_length -
				exposure->coarse_integration_time_max_margin)) {
		v4l2_warn(&mt9f002->subdev, "warning, coarse_integration_time too high (upper limit %d) blanking->frame_length(%d) - exposure->coarse_integration_time_max_margin(%d)\n",
				(blanking->frame_length -
				 exposure->coarse_integration_time_max_margin),
				blanking->frame_length,
				exposure->coarse_integration_time_max_margin);
		exposure->coarse_integration_time = (blanking->frame_length -
				exposure->coarse_integration_time_max_margin);
	}

	// compute final time exposure
	tmp64 = ((u64)exposure->coarse_integration_time *
			blanking->line_length +
			exposure->fine_integration_time) * 1000;
	real_exposure_us = div_u64((u64)tmp64, clock->vt_pix_clk);

	if (res == -1) {
		v4l2_warn(&mt9f002->subdev, "warning, exposure saturation, ask for %d us, final time exposure %d us\n",
				*exposure_us, real_exposure_us);
	}

	*exposure_us = real_exposure_us;
	mt9f002->target_exposure_us->cur.val = *exposure_us;

#ifdef CONFIG_VIDEO_MT9F002_RESTART_BAD_FRAME
	/* Update timings */
	ctx->prev_timings.exposure_us = ctx->timings.exposure_us;
	ctx->timings.exposure_us = real_exposure_us;
#endif

	return res;
}

static int mt9f002_exposure_write(struct mt9f002 *mt9f002)
{
	int res;
	struct mt9f002_context *ctx = &mt9f002->ctx;
	struct mt9f002_exposure *exposure = &ctx->exposure;

	// write values
	res = mt9f002_write16(mt9f002,
			MT9F002_COARSE_INTEGRATION_TIME,
			exposure->coarse_integration_time);
	res |= mt9f002_write16(mt9f002,
			MT9F002_FINE_INTEGRATION_TIME,
			exposure->fine_integration_time);

	return res;
}

static int mt9f002_calc_vt(struct mt9f002 *mt9f002,
			   struct v4l2_mbus_framefmt *format,
			   struct v4l2_rect *crop, int update)
{
	struct mt9f002_context *ctx = &mt9f002->ctx;
	struct mt9f002_blanking *blanking = &ctx->blanking;
	struct v4l2_rect rect;
	unsigned int x_odd_inc;
	unsigned int y_odd_inc;
	unsigned int hratio;
	unsigned int vratio;
	unsigned int width;
	unsigned int height;
	unsigned int ratio;
	unsigned int xMultiple;
	unsigned int div;

	static const u8 xy_odd_inc_tab[] = {1, 1, 3, 3, 7};

	/* Clamp the crop rectangle boundaries and align them to a multiple of 2
	 * pixels to ensure a GRBG Bayer pattern.
	 */
	rect.left = clamp(ALIGN(crop->left, 2),
			MT9F002_X_ADDR_MIN,
			MT9F002_X_ADDR_MAX);
	rect.top = clamp(ALIGN(crop->top, 2),
			MT9F002_Y_ADDR_MIN,
			MT9F002_Y_ADDR_MAX);
	rect.width = clamp(ALIGN(crop->width, 2), 1,
			MT9F002_PIXEL_ARRAY_WIDTH);
	rect.height = clamp(ALIGN(crop->height, 2), 1,
			MT9F002_PIXEL_ARRAY_HEIGHT);

	rect.width = min(rect.width, MT9F002_PIXEL_ARRAY_WIDTH - rect.left);
	rect.height = min(rect.height, MT9F002_PIXEL_ARRAY_HEIGHT - rect.top);

	/* Clamp the width and height to avoid dividing by zero. */
	width = clamp_t(unsigned int, ALIGN(format->width, 2),
			max(rect.width / 8, (__s32) MT9F002_WINDOW_WIDTH_MIN),
			rect.width);
	height = clamp_t(unsigned int, ALIGN(format->height, 2),
			max((rect.height / 8), (__s32) MT9F002_WINDOW_HEIGHT_MIN),
			rect.height);

	/* Calculate binning / skipping for X */
	div = rect.width / width;
	div = clamp_t(unsigned short, div,
			1U, 4U);
	x_odd_inc = xy_odd_inc_tab[div];

	/* Calculate binning / skipping for Y */
	div = rect.height / height;
	div = clamp_t(unsigned short, div,
			1U, 4U);
	y_odd_inc = xy_odd_inc_tab[div];

	/* Align left offset to 8 */
	xMultiple = 8 * ((x_odd_inc + 1) / 2);
	if (rect.left % xMultiple)
		rect.left -= (rect.left % xMultiple);

	/* Calculate remaining scaling not handled by binning / skipping */
	hratio = ((rect.width / ((x_odd_inc + 1) / 2) * SCALER_N) + width - 1) /
		 width;
	vratio = ((rect.height / ((y_odd_inc + 1) / 2) * SCALER_N) + height -
		  1) / height;
	ratio = min(hratio, vratio);

	/* Check ratio */
	if (ratio > SCALER_M_MAX)
	{
		/* Fix ratio to maximum and adjust the crop window */
		ratio = SCALER_M_MAX;
		rect.width = format->width * ratio * ((x_odd_inc + 1) / 2) /
			     SCALER_N;
		rect.height = format->height * ratio * ((y_odd_inc + 1) / 2) /
			      SCALER_N;
	}

	/* Update crop */
	*crop = rect;

	/* Check if scaling configuration has changed */
	if (blanking->x_odd_inc != x_odd_inc ||
	    blanking->y_odd_inc != y_odd_inc || ctx->scaler != ratio) {
		/* Update values */
		if (update) {
			blanking->x_odd_inc = x_odd_inc;
			blanking->y_odd_inc = y_odd_inc;
			ctx->scaler = ratio;
		}
		return 1;
	}

	return 0;
}

static int mt9f002_update_timings(struct mt9f002 *mt9f002)
{
	int ret;

	ret = mt9f002_calc_vt(mt9f002, &mt9f002->format, &mt9f002->crop, 1);
	if (ret < 0) {
		v4l2_err(&mt9f002->subdev, "Unable to compute vt timing and binning\n");
		return ret;
	}

	ret = mt9f002_blanking_init(mt9f002);
	if (ret < 0) {
		v4l2_err(&mt9f002->subdev, "Blanking initialisation failed\n");
		return ret;
	}

	ret = mt9f002_exposure_init(mt9f002);
	if (ret < 0) {
		v4l2_err(&mt9f002->subdev, "Exposition initialization failed\n");
		return ret;
	}

	ret = mt9f002_blanking_conf(mt9f002);
	if (ret < 0) {
		v4l2_err(&mt9f002->subdev, "Blanking configuration failed\n");
		return ret;
	}

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev video operations
 */

static struct v4l2_mbus_framefmt *
__mt9f002_get_pad_format(struct mt9f002 *mt9f002, struct v4l2_subdev_fh *fh,
			 unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &mt9f002->format;
	default:
		return NULL;
	}
}

static struct v4l2_rect *
__mt9f002_get_pad_crop(struct mt9f002 *mt9f002, struct v4l2_subdev_fh *fh,
		       unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &mt9f002->crop;
	default:
		return NULL;
	}
}

static int mt9f002_get_format(struct v4l2_subdev *subdev,
			      struct v4l2_subdev_fh *fh,
			      struct v4l2_subdev_format *format)
{
	struct mt9f002 *mt9f002 = to_mt9f002(subdev);

	format->format = *__mt9f002_get_pad_format(mt9f002, fh, format->pad,
						   format->which);
	return 0;
}

static int mt9f002_set_format(struct v4l2_subdev *subdev,
			      struct v4l2_subdev_fh *fh,
			      struct v4l2_subdev_format *format)
{
	struct mt9f002 *mt9f002 = to_mt9f002(subdev);
	struct v4l2_mbus_framefmt *__format;

	/* Get format */
	__format = __mt9f002_get_pad_format(mt9f002, fh, format->pad,
					    format->which);

	/* Align format and fix format.: scaling is done with croping */
	__format->width = clamp_t(unsigned int, ALIGN(format->format.width, 2),
				  MT9F002_WINDOW_WIDTH_MIN,
				  MT9F002_PIXEL_ARRAY_WIDTH),
	__format->height = clamp_t(unsigned int,
				   ALIGN(format->format.height, 2),
				   MT9F002_WINDOW_HEIGHT_MIN,
				   MT9F002_PIXEL_ARRAY_HEIGHT),

	/* Update format */
	format->format = *__format;

	/* Update crop when not streaming */
	if (format->which != V4L2_SUBDEV_FORMAT_TRY && !mt9f002->isStreaming)
		mt9f002_update_timings(mt9f002);

	return 0;
}

static int mt9f002_get_selection(struct v4l2_subdev *subdev,
		struct v4l2_subdev_fh *fh,
		struct v4l2_subdev_selection *sel)
{
	struct mt9f002 *mt9f002 = to_mt9f002(subdev);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left   = 0;
		sel->r.top    = 0;
		sel->r.width  = MT9F002_PIXEL_ARRAY_WIDTH;
		sel->r.height = MT9F002_PIXEL_ARRAY_HEIGHT;
		break;
	case V4L2_SEL_TGT_CROP:
		sel->r = *__mt9f002_get_pad_crop(mt9f002, fh, sel->pad,
					     sel->which);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int mt9f002_set_selection(struct v4l2_subdev *subdev,
		struct v4l2_subdev_fh *fh,
		struct v4l2_subdev_selection *sel)
{
	struct mt9f002 *mt9f002 = to_mt9f002(subdev);
	struct i2c_client *client = mt9f002->i2c_client;
	struct mt9f002_context *ctx = &mt9f002->ctx;
	struct mt9f002_blanking *blanking = &ctx->blanking;
	struct mt9f002_exposure *exposure = &ctx->exposure;
	struct mt9f002_timing *timings = &ctx->timings;
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	struct v4l2_rect old_crop;
	struct mt9f002_timer *timer;
	struct i2c_msg msg[16];
	struct {
		u16 reg;
		u16 val;
	} __packed msg_buffer[16];
	int msg_len = 0;
	u16 read_mode = 0x0041; /* No binning / skipping */
	ktime_t ktime;
	int32_t delay;
	int recalc_blk;
	int ret;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		old_crop = mt9f002->crop;
		break;
	default:
		v4l2_err(&mt9f002->subdev, "selection target (%d) not supported yet\n",
				sel->target);
		return -EINVAL;
	}

	/* Get format and crop */
	__format = __mt9f002_get_pad_format(mt9f002, fh, sel->pad, sel->which);
	__crop = __mt9f002_get_pad_crop(mt9f002, fh, sel->pad, sel->which);
	*__crop = sel->r;

	/* Calculate cpature window for crop */
	recalc_blk = mt9f002_calc_vt(mt9f002, __format, __crop,
				  sel->which == V4L2_SUBDEV_FORMAT_TRY ? 0 : 1);
	sel->r = *__crop;

	/* Do not update crop config if try */
	if (sel->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	/* Sensor is not streaming */
	if (!mt9f002->isStreaming) {
		/* Recalculate blanking only */
		if (recalc_blk || old_crop.width != mt9f002->crop.width ||
		    old_crop.height != mt9f002->crop.height) {
			/* Reconfigure blanking */
			mt9f002_blanking_init(mt9f002);
			mt9f002_blanking_conf(mt9f002);

			/* Reconfigure exposure */
			mt9f002_exposure_init(mt9f002);
			mt9f002_exposure_conf(mt9f002);
		}

		return 0;
	}

#define ADD(r, v, l) do { \
		msg_buffer[msg_len].reg = swab16(r); \
		msg_buffer[msg_len].val = swab16(v); \
		msg[msg_len].addr = client->addr; \
		msg[msg_len].flags = 0; \
		msg[msg_len].len = 2 + (l / 8); \
		msg[msg_len].buf = (u8 *)&msg_buffer[msg_len]; \
		msg_len++; \
	} while(0)
#define ADD8(r, v) ADD(r, v, 8)
#define ADD16(r, v) ADD(r, v, 16)

	/* Prepare new configuration */
	ADD8(MT9F002_GROUPED_PARAMETER_HOLD, 1);

	/* Set position */
	ADD16(MT9F002_X_ADDR_START, mt9f002->crop.left);
	ADD16(MT9F002_Y_ADDR_START, mt9f002->crop.top);
	ADD16(MT9F002_X_ADDR_END, mt9f002->crop.left + mt9f002->crop.width - 1);
	ADD16(MT9F002_Y_ADDR_END, mt9f002->crop.top + mt9f002->crop.height - 1);

	/* Sensor binning and scaler configuration not changed */
	if (!recalc_blk)
		goto end;

	/* Reconfigure blanking */
	ret |= mt9f002_blanking_init(mt9f002);
	ret |= mt9f002_blanking_conf(mt9f002);
	ADD16(MT9F002_LINE_LENGTH_PCK, blanking->line_length);
	ADD16(MT9F002_FRAME_LENGTH_LINES, blanking->frame_length);

	/* Reconfigure exposure */
	ret |= mt9f002_exposure_init(mt9f002);
	ret |= mt9f002_exposure_conf(mt9f002);
	ADD16(MT9F002_FINE_INTEGRATION_TIME, exposure->fine_integration_time);
	ADD16(MT9F002_COARSE_INTEGRATION_TIME,
	      exposure->coarse_integration_time);

	/* Set scaling */
	ADD16(MT9F002_SCALING_MODE, 2);
	ADD16(MT9F002_SCALE_M, ctx->scaler);

	/* Binning activated for X/Y or only X */
	if (blanking->y_odd_inc > 1)
		read_mode = 0x0441;
	else if (blanking->x_odd_inc > 1)
		read_mode = 0x0841;

	/* Set binning / skipping */
	ADD16(MT9F002_READ_MODE, read_mode);
	ADD16(MT9F002_X_ODD_INC, blanking->x_odd_inc);
	ADD16(MT9F002_Y_ODD_INC, blanking->y_odd_inc);

end:
	/* Apply new configuration */
	ADD8(MT9F002_GROUPED_PARAMETER_HOLD, 0);

#undef ADD16
#undef ADD8
#undef ADD

	/* Send I2C messages in one shot */
	ret = i2c_transfer(client->adapter, msg, msg_len);
	if (ret < 0) {
		v4l2_err(&mt9f002->subdev, "Position configuration failed\n");
		return ret;
	}

	/* When the Y position or size of crop window is changed, the next
	 * incoming frame is considered as bad by sensor. By default, this frame
	 * is masked by inhibition of V/HSYNC signals during this frame.
	 * This implies that next FRAME_SYNC event must be delayed from one
	 * frame. */
	if (old_crop.top != mt9f002->crop.top ||
	    old_crop.width != mt9f002->crop.width ||
	    old_crop.height != mt9f002->crop.height) {
		/* Get timer */
		timer = &mt9f002->fsync_timers[!mt9f002->next_timer];
		if (!hrtimer_active(&timer->hrtimer)) {
			/* A bad frame will occur and must be added in next
			 * timer start (on next VSYNC BOTTOM) */
			mt9f002->drop = 1;
			return ret;
		}

#ifdef CONFIG_VIDEO_MT9F002_RESTART_BAD_FRAME
		/* Restart capture */
		mt9f002_write16(mt9f002, MT9F002_RESET_REGISTER, 0x12CE);

		/* Add a delay of exposure time instead of a frame */
		delay = timings->exposure_us + timings->frameLength_us +
			timings->blkExtraHV_us - MT9F002_CROP_WRITE_US -
			mt9f002->fsync_delay_us;
#else
		/* Add a delay of one frame to timer */
		delay = (timings->blkExtraHV_us + timings->blkStartHV_us +
			 ctx->timings.frameLength_us) * 1000;
#endif

		/* Cancel timer */
		hrtimer_cancel(&timer->hrtimer);

#ifdef CONFIG_VIDEO_MT9F002_RESTART_BAD_FRAME
		/* Calculate new timer delay */
		ktime = ktime_set(0, delay * 1000);
#else
		/* Calculate new timer delay */
		ktime = ktime_add(ktime_set(0, delay),
				  hrtimer_get_remaining(&timer->hrtimer));
#endif

		/* Restart timer */
		hrtimer_start(&timer->hrtimer, ktime, HRTIMER_MODE_REL);
	}

	return ret;
}

#define V4L2_CID_GAIN_RED		(V4L2_CTRL_CLASS_CAMERA | 0x1001)
#define V4L2_CID_GAIN_GREEN_RED		(V4L2_CTRL_CLASS_CAMERA | 0x1002)
#define V4L2_CID_GAIN_GREEN_BLUE	(V4L2_CTRL_CLASS_CAMERA | 0x1003)
#define V4L2_CID_GAIN_BLUE		(V4L2_CTRL_CLASS_CAMERA | 0x1004)
#define V4L2_CID_EXPOSURE_MAX		(V4L2_CTRL_CLASS_CAMERA | 0x1005)

/* Private controls
 *  MT9F002_CID_FRAME_SYNC_DELAY: this control allows to set the delay that user
 *				  needs before end of safe window during which
 *				  set_crop will immediatly apply (in us).
 */
#define MT9F002_CID_FRAME_SYNC_DELAY	(V4L2_CTRL_CLASS_CAMERA | 0x1006)

static int mt9f002_gain_value(struct mt9f002 *mt9f002, u16 reg, s32 gain)
{
	int res = 0;

	pixelGain_t pixelGain;
	uint32_t analogGain;

	if (gain < 10) {
		v4l2_err(&mt9f002->subdev, "error, gain (%d) too low (<1.0)\n", gain);
		res = -1;
	}

	if (!res) {
		// table 19 p56
		if (gain < 15) {
			// warning, gain < 1.5 are not recommended
			v4l2_warn(&mt9f002->subdev, "warning, gain (%d) <1.5 is not recommended\n", gain);
			pixelGain.colamp_gain = 0;
			pixelGain.analog_gain3 = 0;
			pixelGain.digital_gain = 1;
		} else if (gain < 30) {
			pixelGain.colamp_gain = 1;
			pixelGain.analog_gain3 = 0;
			pixelGain.digital_gain = 1;
		} else if (gain < 60) {
			pixelGain.colamp_gain = 2;
			pixelGain.analog_gain3 = 0;
			pixelGain.digital_gain = 1;
		} else if (gain < 160) {
			pixelGain.colamp_gain = 3;
			pixelGain.analog_gain3 = 0;
			pixelGain.digital_gain = 1;
		} else if (gain < 320) {
			pixelGain.colamp_gain = 3;
			pixelGain.analog_gain3 = 0;
			pixelGain.digital_gain = 2;
		} else {
			pixelGain.colamp_gain = 3;
			pixelGain.analog_gain3 = 0;
			pixelGain.digital_gain = 4;
		}

		// compute pixelGain.analog_gain2
		analogGain = (gain * 1000000)/(pixelGain.digital_gain)
			/ (1<<pixelGain.colamp_gain)
			/ (1<<pixelGain.analog_gain3)
			* 64
			/ 10
			/ 1000000;

		if (analogGain == 0) {
			v4l2_err(&mt9f002->subdev, "error, analogGain (%d) too low (<1) \n", analogGain);
			analogGain = 1;
			res = -1;
		} else if (analogGain > 127) {
			v4l2_err(&mt9f002->subdev, "error, analogGain (%d) too high (>127) \n", analogGain);
			analogGain = 127;
			res = -1;
		}

		if (analogGain < 48) {
			v4l2_warn(&mt9f002->subdev, "warning, analogGain (%d) <48 is not recommended\n", analogGain);
		}

		pixelGain.analog_gain2 = analogGain;
		res = mt9f002_write16(mt9f002, reg, pixelGain.val);
	}

	return res;
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev control operations
 */
static int mt9f002_s_ctrl(struct v4l2_ctrl *ctrl)
{
	u8 data;
	static const u16 gains[4] = {
		MT9F002_RED_GAIN, MT9F002_GREEN1_GAIN,
		MT9F002_GREEN2_GAIN, MT9F002_BLUE_GAIN
	};

	struct mt9f002 *mt9f002 =
		container_of(ctrl->handler, struct mt9f002, ctrls);
	unsigned int i;
	int ret;

	/* If not streaming, just keep interval structures up-to-date */
	if (!mt9f002->isStreaming)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		ret = mt9f002_read8(mt9f002,
				MT9F002_IMAGE_ORIENTATION,
				&data);
		if (ret < 0)
			return -EIO;

		if (ctrl->val)
			ret = mt9f002_write8(mt9f002,
					MT9F002_IMAGE_ORIENTATION,
					data | (1 << 1));
		else
			ret = mt9f002_write8(mt9f002,
					MT9F002_IMAGE_ORIENTATION,
					data & ~(1 << 1));

		return mt9f002_read8(mt9f002,
				MT9F002_IMAGE_ORIENTATION,
				&data);

	case V4L2_CID_HFLIP:
		ret = mt9f002_read8(mt9f002,
				MT9F002_IMAGE_ORIENTATION,
				&data);
		if (ret < 0)
			return -EIO;

		if (ctrl->val)
			ret = mt9f002_write8(mt9f002,
					MT9F002_IMAGE_ORIENTATION,
					data | (1 << 0));
		else
			ret = mt9f002_write8(mt9f002,
					MT9F002_IMAGE_ORIENTATION,
					data & ~(1 << 0));

		return mt9f002_read8(mt9f002,
				MT9F002_IMAGE_ORIENTATION,
				&data);

	case V4L2_CID_GAIN_RED:
	case V4L2_CID_GAIN_GREEN_RED:
	case V4L2_CID_GAIN_GREEN_BLUE:
	case V4L2_CID_GAIN_BLUE:

		/* Update the gain controls. */
		for (i = 0; i < 4; ++i) {
			struct v4l2_ctrl *gain = mt9f002->gains[i];

			ret = mt9f002_gain_value(mt9f002, gains[i], gain->val);
			if (ret < 0)
				return -EIO;
		}

		return 0;

	case V4L2_CID_EXPOSURE_ABSOLUTE:

		ret = mt9f002_exposure_conf(mt9f002);
		if (ret < 0) {
			v4l2_err(&mt9f002->subdev, "Exposure configuration failed\n");
			return ret;
		}

		ret = mt9f002_exposure_write(mt9f002);
		if (ret < 0)
		{
			v4l2_err(&mt9f002->subdev, "fatal error, exposure configuration failed\n");
			return ret;
		}

		return 0;

	case V4L2_CID_TEST_PATTERN:

		if (!ctrl->val) {
			return mt9f002_write16(mt9f002,
					MT9F002_TEST_PATTERN,
					0);
		}

		ret = mt9f002_write16(mt9f002, MT9F002_TEST_PATTERN_GREENR,
					0x05a0);
		if (ret < 0)
			return ret;
		ret = mt9f002_write16(mt9f002, MT9F002_TEST_PATTERN_RED,
					0x0a50);
		if (ret < 0)
			return ret;
		ret = mt9f002_write16(mt9f002, MT9F002_TEST_PATTERN_BLUE,
					0x0aa0);
		if (ret < 0)
			return ret;
		ret = mt9f002_write16(mt9f002, MT9F002_TEST_PATTERN_GREENB,
					0x0aa0);
		if (ret < 0)
			return ret;

		ret = mt9f002_write16(mt9f002, MT9F002_TEST_PATTERN_GREENB,
					0x0aa0);
		if (ret < 0)
			return ret;

		return mt9f002_write16(mt9f002,
				MT9F002_TEST_PATTERN,
				ctrl->val);

	case MT9F002_CID_FRAME_SYNC_DELAY:
		mt9f002->fsync_delay_us = ctrl->val;
		break;
	}

	return 0;
}

static int mt9f002_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mt9f002 *mt9f002 =
		container_of(ctrl->handler, struct mt9f002, ctrls);
	struct mt9f002_context *ctx = &mt9f002->ctx;

	/* If not streaming, just default value */
	if (!mt9f002->isStreaming) {
		ctrl->val = ctrl->default_value;
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE_MAX:
		ctrl->val = ctx->maxExposure_us;
		break;
	}

	return 0;
}

static struct v4l2_ctrl_ops mt9f002_ctrl_ops = {
	.g_volatile_ctrl = mt9f002_g_volatile_ctrl,
	.s_ctrl = mt9f002_s_ctrl,
};

static const struct v4l2_ctrl_config mt9f002_gains[] = {
	{
		.ops		= &mt9f002_ctrl_ops,
		.id		= V4L2_CID_GAIN_RED,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Gain, Red x10",
		.min		= MT9F002_GLOBAL_GAIN_MIN,
		.max		= MT9F002_GLOBAL_GAIN_MAX,
		.step		= 1,
		.def		= MT9F002_GLOBAL_GAIN_DEF,
		.flags		= 0,
	}, {
		.ops		= &mt9f002_ctrl_ops,
		.id		= V4L2_CID_GAIN_GREEN_RED,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Gain, Green (R) x10",
		.min		= MT9F002_GLOBAL_GAIN_MIN,
		.max		= MT9F002_GLOBAL_GAIN_MAX,
		.step		= 1,
		.def		= MT9F002_GLOBAL_GAIN_DEF,
		.flags		= 0,
	}, {
		.ops		= &mt9f002_ctrl_ops,
		.id		= V4L2_CID_GAIN_GREEN_BLUE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Gain, Green (B) x10",
		.min		= MT9F002_GLOBAL_GAIN_MIN,
		.max		= MT9F002_GLOBAL_GAIN_MAX,
		.step		= 1,
		.def		= MT9F002_GLOBAL_GAIN_DEF,
		.flags		= 0,
	}, {
		.ops		= &mt9f002_ctrl_ops,
		.id		= V4L2_CID_GAIN_BLUE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Gain, Blue x10",
		.min		= MT9F002_GLOBAL_GAIN_MIN,
		.max		= MT9F002_GLOBAL_GAIN_MAX,
		.step		= 1,
		.def		= MT9F002_GLOBAL_GAIN_DEF,
		.flags		= 0,
	},
};

static const char * const mt9f002_test_pattern_menu[] = {
	"Disabled",
	"Solid color",
	"100% color bar",
	"Fade-to-gray color bar",
};

static const struct v4l2_ctrl_config mt9f002_ctrls[] = {
	{
		.ops		= &mt9f002_ctrl_ops,
		.id		= V4L2_CID_TEST_PATTERN,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Test Pattern",
		.min		= 0,
		.max		= ARRAY_SIZE(mt9f002_test_pattern_menu) - 1,
		.step		= 0,
		.def		= 0,
		.flags		= 0,
		.menu_skip_mask	= 0,
		.qmenu		= mt9f002_test_pattern_menu,
	}, {
		.ops		= &mt9f002_ctrl_ops,
		.id		= V4L2_CID_EXPOSURE_MAX,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Exposure max",
		.min		= 0,
		.max		= 60000,
		.step		= 1,
		.def		= MT9F002_EXPOSURE_DEF,
		.flags		= V4L2_CTRL_FLAG_VOLATILE,
	}, {
		.ops		= &mt9f002_ctrl_ops,
		.id		= MT9F002_CID_FRAME_SYNC_DELAY,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "User delay for Frame sync",
		.min		= 0,
		.max		= USEC_PER_SEC,
		.step		= 1,
		.def		= MT9F002_DEFAULT_USER_US,
		.flags		= 0,
	},
};

/* -----------------------------------------------------------------------------
 * V4L2 subdev internal operations
 */
static int mt9f002_registered(struct v4l2_subdev *subdev)
{
	struct mt9f002 *mt9f002 = to_mt9f002(subdev);
	struct i2c_client *client = mt9f002->i2c_client;
	u16 data;
	int ret = 0;

	dev_info(&client->dev, "Probing mt9f002 at address 0x%02x\n",
			client->addr);

	mt9f002_power_on(mt9f002);

	/* Read and check the sensor version */
	mt9f002_read16(mt9f002, MT9F002_CHIP_VERSION, &data);
	if (data != MT9F002_CHIP_ID) {
		dev_err(&client->dev, "mt9f002 not detected, wrong version "
				"0x%04x\n", data);
		return -ENODEV;
	}

	mt9f002_power_off(mt9f002);

	dev_info(&client->dev, "mt9f002 detected at address 0x%02x\n",
			client->addr);

	return ret;
}

static int mt9f002_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct mt9f002 *mt9f002 = to_mt9f002(sd);

	if (code->index)
		return -EINVAL;

	code->code = mt9f002->format.code;
	return 0;
}

static int mt9f002_s_stream(struct v4l2_subdev *subdev, int enable)
{
	int ret;
	struct mt9f002 *mt9f002 = to_mt9f002(subdev);
	struct mt9f002_platform_data *pdata = mt9f002->pdata;

	if (!enable) {
		mt9f002->isStreaming = false;
		mt9f002_write8(mt9f002, MT9F002_MODE_SELECT, 0);
		return mt9f002_set_power(subdev, 0);
	}

	mt9f002->frame = 0;

	ret = mt9f002_set_power(subdev, 1);
	if (ret < 0) {
		v4l2_err(&mt9f002->subdev, "power failure\n");
		return ret;
	}

	if (pdata->interface == MT9F002_MIPI ||
			pdata->interface == MT9F002_HiSPi ) {
		ret = mt9f002_mipi_stage1_conf(mt9f002);
		if (ret < 0) {
			v4l2_err(&mt9f002->subdev, "mipi conf stage 1 failed\n");
			goto power_off;
		}
	} else {
		ret =  mt9f002_parallel_stage1_conf(mt9f002);
		if (ret < 0) {
			v4l2_err(&mt9f002->subdev, "parallel conf stage 1 failed\n");
			goto power_off;
		}
	}

	ret = mt9f002_pll_write(mt9f002);
	if (ret < 0) {
		v4l2_err(&mt9f002->subdev, "Could not write pll configuration\n");
		goto power_off;
	}

	if (pdata->interface == MT9F002_MIPI ||
		pdata->interface == MT9F002_HiSPi ) {
		ret = mt9f002_mipi_stage2_conf(mt9f002);
		if (ret < 0) {
			v4l2_err(&mt9f002->subdev, "mipi conf stage 2 failed\n");
			goto power_off;
		}
	} else {
		ret =  mt9f002_parallel_stage2_conf(mt9f002);
		if (ret < 0) {
			v4l2_err(&mt9f002->subdev, "parallel conf stage 2 failed\n");
			goto power_off;
		}
	}

	ret = mt9f002_update_timings(mt9f002);
	if (ret < 0) {
		v4l2_err(&mt9f002->subdev,
			 "Unable to calculate Video Timing\n");
		goto power_off;
	}

	ret = mt9f002_exposure_conf(mt9f002);
	if (ret < 0) {
		v4l2_err(&mt9f002->subdev, "Exposure configuration failed\n");
		goto power_off;
	}

	ret = mt9f002_res_write(mt9f002);
	if (ret < 0) {
		v4l2_err(&mt9f002->subdev, "Resolution configuration failed\n");
		goto power_off;
	}

	ret = mt9f002_pos_write(mt9f002);
	if (ret < 0) {
		v4l2_err(&mt9f002->subdev, "Position configuration failed\n");
		goto power_off;
	}

	ret = mt9f002_blanking_write(mt9f002);
	if (ret < 0) {
		v4l2_err(&mt9f002->subdev, "Writing blanking configuration failed\n");
		goto power_off;
	}

	ret = mt9f002_exposure_write(mt9f002);
	if (ret < 0)
	{
		v4l2_err(&mt9f002->subdev, "fatal error, exposure configuration failed\n");
		goto power_off;
	}

	if (pdata->interface == MT9F002_MIPI ||
			pdata->interface == MT9F002_HiSPi ) {
		ret = mt9f002_mipi_stage3_conf(mt9f002);
		if (ret < 0) {
			v4l2_err(&mt9f002->subdev, "Mipi conf stage 3 failed\n");
			goto power_off;
		}
	}

	mt9f002_write8(mt9f002, MT9F002_MODE_SELECT, 1);

	mt9f002->isStreaming = true;

	ret = v4l2_ctrl_handler_setup(&mt9f002->ctrls);
	if (ret) {
		v4l2_err(&mt9f002->subdev, "v4l2_ctrl_handler_setup failure\n");
		goto power_off;
	}

	return 0;

power_off:
	mt9f002_write8(mt9f002, MT9F002_MODE_SELECT, 0);
	mt9f002->isStreaming = false;
	mt9f002_set_power(&mt9f002->subdev, 0);

	return ret;
}

static int mt9f002_g_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_frame_interval *fi)
{
	struct mt9f002 *mt9f002 = to_mt9f002(sd);
	struct mt9f002_context *ctx = &mt9f002->ctx;
	unsigned g;

	memset(fi, 0, sizeof(*fi));
	fi->interval.denominator = NSEC_PER_SEC;
	fi->interval.numerator = ctx->frameperiod_ns;

	/* I could return there but I prefer to reduce the fraction first */
	g = gcd(fi->interval.numerator, fi->interval.denominator);

	fi->interval.numerator   /= g;
	fi->interval.denominator /= g;

	return 0;
}

static int mt9f002_s_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_frame_interval *fi)
{
	struct mt9f002 *mt9f002 = to_mt9f002(sd);
	struct mt9f002_context *ctx = &mt9f002->ctx;
	uint64_t tmp64;
	int ret;

	/* Protect against division by 0. */
	if (fi->interval.denominator == 0)
		fi->interval.denominator = 30;

	if (fi->interval.numerator == 0)
		fi->interval.numerator = 1;

	tmp64 = (u64)fi->interval.numerator * NSEC_PER_SEC;
	ctx->frameperiod_ns = div_u64((u64)tmp64, fi->interval.denominator);

	if (!mt9f002->isStreaming) {
		ret = mt9f002_update_timings(mt9f002);
		if (ret < 0) {
			v4l2_err(&mt9f002->subdev,
					"Unable to calculate Video Timing\n");
			return ret;
		}
	}

	return 0;
}

static struct v4l2_subdev_video_ops mt9f002_subdev_video_ops = {
	.s_stream	= mt9f002_s_stream,
	.g_frame_interval = mt9f002_g_frame_interval,
	.s_frame_interval = mt9f002_s_frame_interval,
};

static struct v4l2_subdev_pad_ops mt9f002_subdev_pad_ops = {
	.get_fmt = mt9f002_get_format,
	.set_fmt = mt9f002_set_format,
	.get_selection  = mt9f002_get_selection,
	.set_selection  = mt9f002_set_selection,
	.enum_mbus_code = mt9f002_enum_mbus_code,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int mt9f002_get_register(struct v4l2_subdev *sd,
				 struct v4l2_dbg_register *reg)
{
	struct mt9f002 *mt9f002 = to_mt9f002(sd);
	int              ret;
	u8               val;

	if (reg->size == 2)
		ret = mt9f002_read16(mt9f002, reg->reg, &val);
	else if (reg->size == 1)
		ret = mt9f002_read8(mt9f002, reg->reg, &val);
	else
		return -EINVAL;

	if (ret)
		return ret;

	reg->val = (__u64)val;

	return 0;
}

static int mt9f002_set_register(struct v4l2_subdev *sd,
				 struct v4l2_dbg_register *reg)
{
	struct mt9f002 *mt9f002 = to_mt9f002(sd);

	if (reg->size == 2)
		return mt9f002_write16(mt9f002, reg->reg, reg->val);
	else if (reg->size == 1)
		return mt9f002_write8(mt9f002, reg->reg, reg->val);
	else
		return -EINVAL;
}
#endif

static int mt9f002_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				   struct v4l2_event_subscription *sub)
{
	if (sub->type != V4L2_EVENT_VSYNC && sub->type != V4L2_EVENT_FRAME_SYNC)
		return -EINVAL;

	return v4l2_event_subscribe(fh, sub, MT9F002_NEVENTS, NULL);
}

static int mt9f002_unsubscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				     struct v4l2_event_subscription *sub)
{
	return v4l2_event_unsubscribe(fh, sub);
}

static enum hrtimer_restart mt9f002_fsync_callback(struct hrtimer *_timer)
{
	struct mt9f002_timer *timer = container_of(_timer, struct mt9f002_timer,
						   hrtimer);
	struct v4l2_event event = {
		.type = V4L2_EVENT_FRAME_SYNC,
		.u.frame_sync = {
			.frame_sequence = timer->frame,
		},
	};

	/* Queue V4L2 frame_sync event */
	v4l2_event_queue(timer->vdev, &event);

	return HRTIMER_NORESTART;
}

#ifdef CONFIG_VIDEO_MT9F002_RESTART_BAD_FRAME
static void mt9f002_restart_callback(struct kthread_work *work)
{
	struct mt9f002 *mt9f002 = container_of(work, struct mt9f002,
					       fsync_work);
	struct mt9f002_context *ctx = &mt9f002->ctx;
	struct mt9f002_timing *timings = &ctx->timings;
	struct video_device *vdev = mt9f002->subdev.devnode;
	struct mt9f002_timer *timer;
	int32_t delay = 0;
	ktime_t ktime;

	/* Restart capture */
	mt9f002_write16(mt9f002, MT9F002_RESET_REGISTER, 0x12CE);

	/* Calculate delay of exposure time */
	delay = timings->exposure_us + timings->frameLength_us +
		timings->blkExtraHV_us - MT9F002_CROP_WRITE_US -
		mt9f002->fsync_delay_us;

	/* Start a new timer */
	if (delay < 0)
		return;

	/* Get next timer */
	timer = &mt9f002->fsync_timers[mt9f002->next_timer];
	if (hrtimer_active(&timer->hrtimer))
		return;

	/* Setup timer */
	ktime = ktime_set(0, delay * 1000);
	timer->hrtimer.function = &mt9f002_fsync_callback;
	timer->vdev = vdev;
	timer->frame = mt9f002->frame++;
	hrtimer_start(&timer->hrtimer, ktime, HRTIMER_MODE_REL);

	/* Set next timer */
	mt9f002->next_timer ^= 1;
}
#endif

static int mt9f002_irq_handler(struct v4l2_subdev *sd, u32 status,
			       bool *handled)
{
	struct mt9f002 *mt9f002 = container_of(sd, struct mt9f002, subdev);
	struct mt9f002_context *ctx = &mt9f002->ctx;
	struct mt9f002_timing *timings = &ctx->timings;
	struct video_device *vdev = sd->devnode;
	static const struct v4l2_event event = {
		.type = V4L2_EVENT_VSYNC,
		.u.vsync.field = V4L2_FIELD_BOTTOM,
	};
	struct mt9f002_timer *timer;
	int32_t delay;
	ktime_t ktime;

	/* Queue V4L2 Vsync event (BOTTOM) */
	if (vdev)
		v4l2_event_queue(vdev, &event);

	/* Start a new timer for next safe window */
	if (vdev) {
		/* Calculate delay before beginning of next zone during which a
		 * new sensor conbfiguration will be delayed of one frame.
		 * This time is arround minimal blanking time of current
		 * frame + time needed for I2C configuration + defined user
		 * delay.
		 */
		delay = (ctx->prev_timings.blkExtraHV_us +
			 timings->blkStartHV_us + timings->frameLength_us +
			 timings->blkExtraHV_us - MT9F002_CROP_WRITE_US -
			 mt9f002->fsync_delay_us);

		/* Next frame is a bad frame: add a complete frame in order to
		 * do not have two consecutive bad frames.
		 * When restart is enabled, frame capture is restarted in order
		 * to reduce time before next frame.
		 */
		if (mt9f002->drop) {
			mt9f002->drop = 0;
#ifdef CONFIG_VIDEO_MT9F002_RESTART_BAD_FRAME
			/* Queue restart task in kthread worker */
			queue_kthread_work(&mt9f002->fsync_worker, &mt9f002->fsync_work);

			/* Timer will be started in restart task */
			goto end;
#else
			delay += (timings->blkExtraHV_us +
				  timings->blkStartHV_us +
				  timings->frameLength_us);
#endif
		}

		/* Too late: skip window */
		if (delay < 0)
			goto end;

		/* Get next timer */
		timer = &mt9f002->fsync_timers[mt9f002->next_timer];
		if (hrtimer_active(&timer->hrtimer))
			goto end;

		/* Setup timer */
		ktime = ktime_set(0, delay * 1000);
		timer->hrtimer.function = &mt9f002_fsync_callback;
		timer->vdev = vdev;
		timer->frame = mt9f002->frame++;
		hrtimer_start(&timer->hrtimer, ktime, HRTIMER_MODE_REL);

		/* Set next timer */
		mt9f002->next_timer ^= 1;
	}

end:
	*handled = true;
	return 0;
}

static const struct v4l2_subdev_core_ops mt9f002_core_ops = {
	.g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = mt9f002_get_register,
	.s_register = mt9f002_set_register,
#endif
	.subscribe_event = mt9f002_subscribe_event,
	.unsubscribe_event = mt9f002_unsubscribe_event,
	.interrupt_service_routine = mt9f002_irq_handler,
};

static struct v4l2_subdev_ops mt9f002_subdev_ops = {
	.core	= &mt9f002_core_ops,
	.video	= &mt9f002_subdev_video_ops,
	.pad	= &mt9f002_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops mt9f002_subdev_internal_ops = {
	.registered = mt9f002_registered,
};

/* -----------------------------------------------------------------------------
 * Driver initialization and probing
 */
static int mt9f002_probe(struct i2c_client *client,
		const struct i2c_device_id *did)
{
#ifdef CONFIG_VIDEO_MT9F002_RESTART_BAD_FRAME
	struct sched_param param = {
		.sched_priority = 99,
	};
#endif
	struct mt9f002 *mt9f002;
	struct mt9f002_context *ctx;
	struct mt9f002_blanking *blanking;
	struct mt9f002_platform_data *pdata = NULL;
	unsigned int i;
	int ret;
	pdata = client->dev.platform_data;

	if (!pdata)
		return -EINVAL;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	dev_info(&client->dev, "detecting mt9f002 client on address 0x%x\n",
			client->addr << 1);

	mt9f002 = kzalloc(sizeof(*mt9f002), GFP_KERNEL);
	if (!mt9f002)
		return -ENOMEM;

	ctx = &mt9f002->ctx;
	blanking = &ctx->blanking;
	mutex_init(&mt9f002->power_lock);
	mt9f002->pdata = client->dev.platform_data;

	v4l2_ctrl_handler_init(&mt9f002->ctrls, ARRAY_SIZE(mt9f002_gains) + ARRAY_SIZE(mt9f002_ctrls) + 6);

	mt9f002->target_exposure_us = v4l2_ctrl_new_std(&mt9f002->ctrls, &mt9f002_ctrl_ops,
			  V4L2_CID_EXPOSURE_ABSOLUTE, 0,
			  60000, 1,
			  MT9F002_EXPOSURE_DEF);

	v4l2_ctrl_new_std(&mt9f002->ctrls,
			&mt9f002_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&mt9f002->ctrls,
			&mt9f002_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	mt9f002->hblank = v4l2_ctrl_new_std(&mt9f002->ctrls, &mt9f002_ctrl_ops,
			  V4L2_CID_HBLANK, 1, LINE_LENGTH_MAX, 1, 1);
	mt9f002->vblank = v4l2_ctrl_new_std(&mt9f002->ctrls, &mt9f002_ctrl_ops,
			  V4L2_CID_VBLANK, MT9F002_MIN_FRAME_BLANKING_LINES_DEF,
			  FRAME_LENGTH_MAX - 2, 1,
			  MT9F002_MIN_FRAME_BLANKING_LINES_DEF);
	mt9f002->pixel_rate = v4l2_ctrl_new_std(&mt9f002->ctrls,
						&mt9f002_ctrl_ops,
						V4L2_CID_PIXEL_RATE,
						1000000L, 220000000LL, 1,
						220000000LL);

	for (i = 0; i < ARRAY_SIZE(mt9f002_gains); ++i)
		mt9f002->gains[i] = v4l2_ctrl_new_custom(&mt9f002->ctrls,
			&mt9f002_gains[i], NULL);

	for (i = 0; i < ARRAY_SIZE(mt9f002_ctrls); ++i)
		v4l2_ctrl_new_custom(&mt9f002->ctrls, &mt9f002_ctrls[i], NULL);

	mt9f002->subdev.ctrl_handler = &mt9f002->ctrls;

	if (mt9f002->ctrls.error)
		v4l2_err(client, "%s: control initialization error %d\n",
		       __func__, mt9f002->ctrls.error);

	mt9f002->set_power = pdata->set_power;
	mt9f002->i2c_client = client;

	v4l2_i2c_subdev_init(&mt9f002->subdev, client, &mt9f002_subdev_ops);
	mt9f002->subdev.internal_ops = &mt9f002_subdev_internal_ops;

	mt9f002->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&mt9f002->subdev.entity, 1, &mt9f002->pad, 0);
	if (ret < 0) {
		v4l_err(client, "failed to init media entity\n");
		goto emedia;
	}

	mt9f002->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
				 V4L2_SUBDEV_FL_HAS_EVENTS;
	mt9f002->format.code = V4L2_MBUS_FMT_SGRBG10_1X10;
	mt9f002->format.field = V4L2_FIELD_NONE;

	/* Max sensor crop into 1920p30 */
	mt9f002->format.width = 1920;
	mt9f002->format.height = 1080;

	mt9f002->crop.left = 1330; //MT9F002_X_ADDR_MIN;
	mt9f002->crop.top = 1120; //MT9F002_Y_ADDR_MIN;
	mt9f002->crop.width = 1920; //MT9F002_PIXEL_ARRAY_WIDTH;
	mt9f002->crop.height = 1080; //MT9F002_PIXEL_ARRAY_HEIGHT;

	blanking->min_frame_blanking_lines
		= MT9F002_MIN_FRAME_BLANKING_LINES_DEF;

	/* 30 FPS */
	ctx->frameperiod_ns = MT9F002_FRAMEPERIOD_NS_DEF;

	ret = mt9f002_pll_setup(mt9f002);
	if (ret < 0) {
		v4l2_err(&mt9f002->subdev, "Unable to setup pll\n");
		return ret;
	}

	ret = mt9f002_update_timings(mt9f002);
	if (ret < 0) {
		v4l2_err(&mt9f002->subdev,
				"Unable to calculate Video Timing\n");
		return ret;
	}

	/* Init internal timers */
	hrtimer_init(&mt9f002->fsync_timers[0].hrtimer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	hrtimer_init(&mt9f002->fsync_timers[1].hrtimer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	mt9f002->fsync_delay_us = MT9F002_DEFAULT_USER_US;

#ifdef CONFIG_VIDEO_MT9F002_RESTART_BAD_FRAME
	/* Create a new kthread worker for restart handling */
	init_kthread_worker(&mt9f002->fsync_worker);
	mt9f002->fsync_worker_task = kthread_run(kthread_worker_fn,
						 &mt9f002->fsync_worker,
						 mt9f002->subdev.name);
	if (mt9f002->fsync_worker_task == NULL) {
		v4l2_err(&mt9f002->subdev, "Unable to create worker kthread\n");
		return -1;
	}

	/* Use FIFO scheduler for realtime purpose */
	sched_setscheduler(mt9f002->fsync_worker_task, SCHED_FIFO, &param);

	/* Initialize restart work in case of high framerate */
	init_kthread_work(&mt9f002->fsync_work, mt9f002_restart_callback);
#endif

	dev_info(&client->dev, "mt9f002 client on address 0x%x name(%s) registered\n",
			client->addr << 1, did->name);
	return 0;

emedia:
	v4l2_ctrl_handler_free(&mt9f002->ctrls);
	mutex_destroy(&mt9f002->power_lock);
	kfree(mt9f002);

	return ret;
}

static int mt9f002_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct mt9f002 *mt9f002 = to_mt9f002(subdev);

#ifdef CONFIG_VIDEO_MT9F002_RESTART_BAD_FRAME
	flush_kthread_worker(&mt9f002->fsync_worker);
	kthread_stop(mt9f002->fsync_worker_task);
#endif
	v4l2_ctrl_handler_free(&mt9f002->ctrls);
	v4l2_device_unregister_subdev(subdev);
	media_entity_cleanup(&subdev->entity);
	mutex_destroy(&mt9f002->power_lock);
	kfree(mt9f002);
	return 0;
}

static const struct i2c_device_id mt9f002_id[] = {
	{ "mt9f002", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mt9f002_id);

static struct i2c_driver mt9f002_driver = {
	.driver = {
		.name = "mt9f002",
	},
	.probe		= mt9f002_probe,
	.remove		= mt9f002_remove,
	.id_table	= mt9f002_id,
};

module_i2c_driver(mt9f002_driver);

MODULE_DESCRIPTION("Aptina mt9f002 Camera driver");
MODULE_AUTHOR("Karl Leplat <karl.leplat@parrot.com>");
MODULE_LICENSE("GPL");

