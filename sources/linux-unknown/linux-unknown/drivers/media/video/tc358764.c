/* Driver for the TC358764 Parallel to MIPI bridge
 *
 *  Copyright (C) 2014 Parrot S.A.
 *
 * @author  Lionel Flandrin <lionel.flandrin@parrot.com>
 * @date    12-Dec-2014
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <media/tc358764.h>

#define DEVICE_NAME "tc358764"

#define TC358764_REG_VERSION                   0x0000
/* Expected chip revision */
#define TC358764_VERSION                       0x4401

/* These registers can be written using tc358764_write_dsi_config */
#define TC358764_REG_DSI_CONTROL               0x040C
#define TC358764_REG_DSI_INT_ENA               0x0418
#define TC358764_REG_DSI_ACKERR_INTENA         0x0438
#define TC358764_REG_DSI_ACKERR_HALT           0x043C
#define TC358764_REG_DSI_RXERR_INTENA          0x0444
#define TC358764_REG_DSI_RXERR_HALT            0x0448
#define TC358764_REG_DSI_ERR_INTENA            0x0450
#define TC358764_REG_DSI_ERR_HALT              0x0454

#define TC358764_REG_DSI_CONFW                 0x0500

#define TC358764_REG_DSICMD_TX                 0x0600
#define TC358764_REG_DSICMD_TYPE               0x0602
#define TC358764_REG_DSICMD_WC                 0x0604
#define TC358764_REG_DSICMD_WD0                0x0610
#define TC358764_REG_DSICMD_WD1                0x0612
#define TC358764_REG_DSICMD_WD2                0x0614
#define TC358764_REG_DSICMD_WD3                0x0616

struct tc358764 {
	struct i2c_client *client;
};

static int tc358764_read16(struct tc358764 *tc358764, u16 reg, u16 *val)
{
	struct i2c_client *client = tc358764->client;
	u8                 regb[2];
	u8                 valb[2];
	int                ret;

	struct i2c_msg msg[] = {
		[0] = {
			.addr  = client->addr,
			.flags = 0,
			.len   = ARRAY_SIZE(regb),
			.buf   = regb,
		},
		[1] = {
			.addr  = client->addr,
			.flags = I2C_M_RD,
			.len   = ARRAY_SIZE(valb),
			.buf   = valb,
		},
	};

	regb[0] = (u8)(reg >> 8);
	regb[1] = (u8)(reg);

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		dev_err(&client->dev, "I2C read failed with error %d", ret);
		return ret;
	}

	*val = (((u16)valb[0]) << 8) | ((u16)valb[1]);

	return 0;
}

static int tc358764_write16(struct tc358764 *tc358764, u16 reg, u16 val)
{
	struct i2c_client *client = tc358764->client;
	u8                 data[4];
	int                ret;

	struct i2c_msg msg[] = {
		[0] = {
			.addr  = client->addr,
			.flags = 0,
			.len   = ARRAY_SIZE(data),
			.buf   = data,
		},
	};

	data[0] = (u8)(reg >> 8);
	data[1] = (u8)(reg);

	data[2] = (u8)(val >> 8);
	data[3] = (u8)(val);

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		dev_err(&client->dev, "I2C write failed with error %d", ret);
		return ret;
	}

	return 0;
}

static int tc358764_write32(struct tc358764 *tc358764, u16 reg, u32 val)
{
	struct i2c_client *client = tc358764->client;
	u8                 data[6];
	int                ret;

	struct i2c_msg msg[] = {
		[0] = {
			.addr  = client->addr,
			.flags = 0,
			.len   = ARRAY_SIZE(data),
			.buf   = data,
		},
	};

	data[0] = (u8)(reg >> 8);
	data[1] = (u8)(reg);

	// The chip uses this weird "PDP-endian" style byte organisation for
	// 32byte writes.
	data[2] = (u8)(val >> 8);
	data[3] = (u8)(val);
	data[4] = (u8)(val >> 24);
	data[5] = (u8)(val >> 16);

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		dev_err(&client->dev, "I2C write failed with error %d", ret);
		return ret;
	}

	return 0;
}

/* Certain registers have to be written indirectly using the DSI_CONFW
 * register. It lets us set or clear individual bits (depending on whether set
 * is true or false) data in reg */
static int tc358764_write_dsi_config(struct tc358764 *tc358764,
				     u16              reg,
				     int              set,
				     u16              data)
{
	u32	id = 0;
	u32     r;
	int	i;

	/* The DSI_CONFW register can be used to write to a specific list of
	 * registers selected with a 5bit identifier */
	static const struct { u16 reg; u32 id; } id_list[] = {
		{ TC358764_REG_DSI_CONTROL,       0x03 },
		{ TC358764_REG_DSI_INT_ENA,       0x06 },
		{ TC358764_REG_DSI_ACKERR_INTENA, 0x0e },
		{ TC358764_REG_DSI_ACKERR_HALT,   0x0f },
		{ TC358764_REG_DSI_RXERR_INTENA,  0x11 },
		{ TC358764_REG_DSI_RXERR_HALT,    0x12 },
		{ TC358764_REG_DSI_ERR_INTENA,    0x14 },
		{ TC358764_REG_DSI_ERR_HALT,      0x15 },
	};

	for (i = 0; i < ARRAY_SIZE(id_list); i++) {
		if (id_list[i].reg == reg) {
			id = id_list[i].id;
			break;
		}
	}

	/* Crash if we're called with an invalid register */
	BUG_ON(id == 0);

	/* Build the DSI_CONFW command value */
	r  = id << 24;
	r |= (set ? 0x5 : 0x6) << 29;
	r |= (u32)data;

	return tc358764_write32(tc358764, TC358764_REG_DSI_CONFW, r);
}

/* Send a short command with no parameters */
static int tc358764_dsi_write_no_param(struct tc358764 *tc358764, u8 cmd)
{
	int ret;

	/* Short write without parameter */
	ret = tc358764_write16(tc358764, TC358764_REG_DSICMD_TYPE, 0x1005);
	if (ret)
		goto err;

	/* Word count is always 0 for short writes */
	ret = tc358764_write16(tc358764, TC358764_REG_DSICMD_WC, 0);
	if (ret)
		goto err;

	/* Command byte to be sent */
	ret = tc358764_write16(tc358764, TC358764_REG_DSICMD_WD0, cmd);
	if (ret)
		goto err;

	/* Start DSI transfer */
	ret = tc358764_write16(tc358764, TC358764_REG_DSICMD_TX, 1);
	if (ret)
		goto err;

 err:
	return ret;
}

/* Send a short command with one parameters */
static int tc358764_dsi_write_one_param(struct tc358764 *tc358764,
					u8               cmd,
					u8
               param)
{
	int ret;

	/* Short write with one parameter */
	ret = tc358764_write16(tc358764, TC358764_REG_DSICMD_TYPE, 0x1015);
	if (ret)
		goto err;

	/* Word count is always 0 for short writes */
	ret = tc358764_write16(tc358764, TC358764_REG_DSICMD_WC, 0);
	if (ret)
		goto err;

	/* Command and parameter to be sent */
	ret = tc358764_write16(tc358764,
			       TC358764_REG_DSICMD_WD0,
			       (((u16)param) << 8) | (u16)cmd);
	if (ret)
		goto err;

	/* Start DSI transfer */
	ret = tc358764_write16(tc358764, TC358764_REG_DSICMD_TX, 1);
	if (ret)
		goto err;

 err:
	return ret;
}

/* Send a long command with up to 7 parameters */
static int tc358764_dsi_write_long(struct tc358764 *tc358764,
				   u8               cmd,
				   size_t           nparams,
				   u8              *params)
{
	int ret;

	BUG_ON(nparams > 7);

	/* DCS long write */
	ret = tc358764_write16(tc358764, TC358764_REG_DSICMD_TYPE, 0x4039);
	if (ret)
		goto err;

	/* Set number of parameters + 1 for the command byte */
	ret = tc358764_write16(tc358764, TC358764_REG_DSICMD_WC, nparams + 1);
	if (ret)
		goto err;

	/* Store the command + parameters to be sent. Unused parameters are
	 * padded with 0s */

#define GET_PARAM(n)   ((n) < nparams ? params[(n)] : 0)
#define INTO_U16(l, h) ((u16)(l) | (((u16)(h)) << 8))

	ret = tc358764_write16(tc358764,
			       TC358764_REG_DSICMD_WD0,
			       INTO_U16(cmd, GET_PARAM(0)));
	if (ret)
		goto err;

	ret = tc358764_write16(tc358764,
			       TC358764_REG_DSICMD_WD1,
			       INTO_U16(GET_PARAM(1), GET_PARAM(2)));
	if (ret)
		goto err;


	ret = tc358764_write16(tc358764,
			       TC358764_REG_DSICMD_WD2,
			       INTO_U16(GET_PARAM(3), GET_PARAM(4)));
	if (ret)
		goto err;

	ret = tc358764_write16(tc358764,
			       TC358764_REG_DSICMD_WD3,
			       INTO_U16(GET_PARAM(5), GET_PARAM(6)));
	if (ret)
		goto err;

#undef INTO_U16
#undef GET_PARAM

	/* Start DSI transfer */
	ret = tc358764_write16(tc358764, TC358764_REG_DSICMD_TX, 1);
	if (ret)
		goto err;

 err:
	return ret;
}

/// Send DSI command with a variable amount of parameters
static int tc358764_dsi_write(struct tc358764 *tc358764,
			      u8               cmd,
			      size_t           nparams,
			      u8              *params)
{
	if (nparams == 0) {
		return tc358764_dsi_write_no_param(tc358764, cmd);
	} else if (nparams == 1) {
		return tc358764_dsi_write_one_param(tc358764, cmd, params[0]);
	} else if (nparams <= 7) {
		return tc358764_dsi_write_long(tc358764, cmd, nparams, params);
	} else {
		/* Sending commands with more parameters is possible with this
		 * bridge but it's tricky. You need to use the video RAM to
		 * store the command (so you have to stop the video streaming)
		 * and we never managed to find the correct sequence to get it
		 * to work reliably. */
		dev_err(&tc358764->client->dev,
			"DSI command with more than 7 "
			"parameters are not supported");
		return -EINVAL;
	}

	/* Not sure if that's entirely necessary but it shouldn't hurt */
	msleep(1);
}

/* XXX Hardcoded init for RNB6 */
static int tc358764_rnb6_init(struct tc358764 *tc358764) {
	int ret;

#define TRY(exp) do { ret = (exp); if (ret < 0) { goto err; } } while (0)

	// Disable Parallel Input and activate Automatic I2C address increment
	TRY(tc358764_write16(tc358764, 0x04, 0x04));

	// Software reset
	TRY(tc358764_write16(tc358764, 0x2, 0x1));
	msleep(10);
	TRY(tc358764_write16(tc358764, 0x2, 0x0));

	// PLL config
	TRY(tc358764_write16(tc358764, 0x16, 0x507e));
	TRY(tc358764_write16(tc358764, 0x18, 0x0203));
	msleep(1);
	TRY(tc358764_write16(tc358764, 0x18, 0x0213));

	TRY(tc358764_write16(tc358764, 0x06, 0x0D0));
	TRY(tc358764_write16(tc358764, 0x08, 0x37)); //?
	TRY(tc358764_write16(tc358764, 0x50, 0x3e)); //?

	TRY(tc358764_write32(tc358764, 0x140, 0));
	TRY(tc358764_write32(tc358764, 0x144, 0));
	TRY(tc358764_write32(tc358764, 0x148, 0));
	TRY(tc358764_write32(tc358764, 0x14c, 0));
	TRY(tc358764_write32(tc358764, 0x150, 0));

	TRY(tc358764_write32(tc358764, 0x210, 0x2c88));
	TRY(tc358764_write32(tc358764, 0x214, 0x5));
	TRY(tc358764_write32(tc358764, 0x218, 0x1f06));
	TRY(tc358764_write32(tc358764, 0x21c, 0x3));
	TRY(tc358764_write32(tc358764, 0x220, 0x606));
	TRY(tc358764_write32(tc358764, 0x224, 0x4a88));
	TRY(tc358764_write32(tc358764, 0x228, 0xb));


	TRY(tc358764_write32(tc358764, 0x22c, 0x4));

	TRY(tc358764_write32(tc358764, 0x234, 0x1f));

	TRY(tc358764_write32(tc358764, 0x238, 0x1));
	TRY(tc358764_write32(tc358764, 0x23c, 0x00050005));
	TRY(tc358764_write32(tc358764, 0x204, 0x1));

	TRY(tc358764_write16(tc358764, 0x620, 0x1));
	TRY(tc358764_write16(tc358764, 0x622, 0x11));
	TRY(tc358764_write16(tc358764, 0x626, 0x500));
	TRY(tc358764_write16(tc358764, 0x628, 0xd8));
	TRY(tc358764_write16(tc358764, 0x62c, 0x870));

	//DSI Start
	TRY(tc358764_write32(tc358764, 0x518, 0x1));

	//Set bit DSI CTRL
	TRY(tc358764_write_dsi_config(tc358764,
				      TC358764_REG_DSI_CONTROL,
				      1,
				      0x0126));
	//Clear bit DSI CTRL
	TRY(tc358764_write_dsi_config(tc358764,
				      TC358764_REG_DSI_CONTROL,
				      0,
				      0xfe81));

	// INIT LCD

	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x00}));
	TRY(tc358764_dsi_write(tc358764, 0xff, 3, (u8[]){0x12, 0x84, 0x01}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x80}));
	TRY(tc358764_dsi_write(tc358764, 0xff, 2, (u8[]){0x12, 0x84}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x80}));
	TRY(tc358764_dsi_write(tc358764, 0xc0, 7, (u8[]){0x00, 0x64, 0x00, 0x10, 0x10, 0x00, 0x64}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x87}));
	TRY(tc358764_dsi_write(tc358764, 0xc0, 2, (u8[]){0x10, 0x10}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x90}));
	TRY(tc358764_dsi_write(tc358764, 0xc0, 6, (u8[]){0x00, 0x5c, 0x00, 0x01, 0x00, 0x04}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xb3}));
	TRY(tc358764_dsi_write(tc358764, 0xc0, 2, (u8[]){0x00, 0x55}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x81}));
	TRY(tc358764_dsi_write(tc358764, 0xc1, 1, (u8[]){0x55}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xa0}));
	TRY(tc358764_dsi_write(tc358764, 0xc4, 7, (u8[]){0x05, 0x10, 0x06, 0x02, 0x05, 0x15, 0x10}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xa7}));
	TRY(tc358764_dsi_write(tc358764, 0xc4, 7, (u8[]){0x05, 0x10, 0x07, 0x02, 0x05, 0x15, 0x10}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xb0}));
	TRY(tc358764_dsi_write(tc358764, 0xc4, 2, (u8[]){0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x91}));
	TRY(tc358764_dsi_write(tc358764, 0xc5, 2, (u8[]){0x46, 0x42}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x00}));
	TRY(tc358764_dsi_write(tc358764, 0xd8, 2, (u8[]){0xc7, 0xc7}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x00}));
	TRY(tc358764_dsi_write(tc358764, 0xd9, 1, (u8[]){0x68}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xb3}));
	TRY(tc358764_dsi_write(tc358764, 0xc5, 1, (u8[]){0x84}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xbb}));
	TRY(tc358764_dsi_write(tc358764, 0xc5, 1, (u8[]){0x8a}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x82}));
	TRY(tc358764_dsi_write(tc358764, 0xC4, 1, (u8[]){0x0a}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xc6}));
	TRY(tc358764_dsi_write(tc358764, 0xB0, 1, (u8[]){0x03}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xc2}));
	TRY(tc358764_dsi_write(tc358764, 0xf5, 1, (u8[]){0x40}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xc3}));
	TRY(tc358764_dsi_write(tc358764, 0xf5, 1, (u8[]){0x85}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x00}));
	TRY(tc358764_dsi_write(tc358764, 0xd0, 1, (u8[]){0x40}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x00}));
	TRY(tc358764_dsi_write(tc358764, 0xd1, 2, (u8[]){0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x80}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x87}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 4, (u8[]){0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x90}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x97}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x9e}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 1, (u8[]){0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xa0}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xa7}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xae}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 1, (u8[]){0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xb0}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xb7}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xbe}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 1, (u8[]){0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xc0}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 7, (u8[]){0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xc7}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 7, (u8[]){0x00, 0x00, 0x00, 0x05, 0x05, 0x00, 0x05}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xce}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 1, (u8[]){0x05}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xd0}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xd7}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 7, (u8[]){0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xde}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 1, (u8[]){0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xe0}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 7, (u8[]){0x00, 0x00, 0x05, 0x05, 0x00, 0x05, 0x05}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xe7}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xf0}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 7, (u8[]){0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xf7}));
	TRY(tc358764_dsi_write(tc358764, 0xcb, 4, (u8[]){0xff, 0xff, 0xff, 0xff}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x80}));
	TRY(tc358764_dsi_write(tc358764, 0xcc, 7, (u8[]){0x0e, 0x10, 0x0a, 0x0c, 0x02, 0x04, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x87}));
	TRY(tc358764_dsi_write(tc358764, 0xcc, 7, (u8[]){0x00, 0x00, 0x00, 0x2e, 0x2d, 0x00, 0x29}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x8e}));
	TRY(tc358764_dsi_write(tc358764, 0xcc, 1, (u8[]){0x2a}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x90}));
	TRY(tc358764_dsi_write(tc358764, 0xcc, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x97}));
	TRY(tc358764_dsi_write(tc358764, 0xcc, 7, (u8[]){0x0d, 0x0f, 0x09, 0x0b, 0x01, 0x03, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x9e}));
	TRY(tc358764_dsi_write(tc358764, 0xcc, 1, (u8[]){0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xa0}));
	TRY(tc358764_dsi_write(tc358764, 0xcc, 7, (u8[]){0x00, 0x00, 0x2e, 0x2d, 0x00, 0x29, 0x2a}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xa7}));
	TRY(tc358764_dsi_write(tc358764, 0xcc, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xb0}));
	TRY(tc358764_dsi_write(tc358764, 0xcc, 7, (u8[]){0x0b, 0x09, 0x0f, 0x0d, 0x03, 0x01, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xb7}));
	TRY(tc358764_dsi_write(tc358764, 0xcc, 7, (u8[]){0x00, 0x00, 0x00, 0x2d, 0x2e, 0x00, 0x29}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xbe}));
	TRY(tc358764_dsi_write(tc358764, 0xcc, 1, (u8[]){0x2a}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xc0}));
	TRY(tc358764_dsi_write(tc358764, 0xcc, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xc7}));
	TRY(tc358764_dsi_write(tc358764, 0xcc, 7, (u8[]){0x0c, 0x0a, 0x10, 0x0e, 0x04, 0x02, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xce}));
	TRY(tc358764_dsi_write(tc358764, 0xcc, 1, (u8[]){0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xd0}));
	TRY(tc358764_dsi_write(tc358764, 0xcc, 7, (u8[]){0x00, 0x00, 0x2d, 0x2e, 0x00, 0x29, 0x2a}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xd7}));
	TRY(tc358764_dsi_write(tc358764, 0xcc, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x80}));
	TRY(tc358764_dsi_write(tc358764, 0xce, 7, (u8[]){0x8b, 0x03, 0x18, 0x8a, 0x03, 0x18, 0x89}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x87}));
	TRY(tc358764_dsi_write(tc358764, 0xce, 5, (u8[]){0x03, 0x18, 0x88, 0x03, 0x18}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x90}));
	TRY(tc358764_dsi_write(tc358764, 0xce, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x97}));
	TRY(tc358764_dsi_write(tc358764, 0xce, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xa0}));
	TRY(tc358764_dsi_write(tc358764, 0xce, 7, (u8[]){0x38, 0x07, 0x05, 0x00, 0x00, 0x18, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xa7}));
	TRY(tc358764_dsi_write(tc358764, 0xce, 7, (u8[]){0x38, 0x06, 0x05, 0x01, 0x00, 0x18, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xb0}));
	TRY(tc358764_dsi_write(tc358764, 0xce, 7, (u8[]){0x38, 0x05, 0x05, 0x02, 0x00, 0x18, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xb7}));
	TRY(tc358764_dsi_write(tc358764, 0xce, 7, (u8[]){0x38, 0x04, 0x05, 0x03, 0x00, 0x18, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xc0}));
	TRY(tc358764_dsi_write(tc358764, 0xce, 7, (u8[]){0x38, 0x03, 0x05, 0x04, 0x00, 0x18, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xc7}));
	TRY(tc358764_dsi_write(tc358764, 0xce, 7, (u8[]){0x38, 0x02, 0x05, 0x05, 0x00, 0x18, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xd0}));
	TRY(tc358764_dsi_write(tc358764, 0xce, 7, (u8[]){0x38, 0x01, 0x05, 0x06, 0x00, 0x18, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xd7}));
	TRY(tc358764_dsi_write(tc358764, 0xce, 7, (u8[]){0x38, 0x00, 0x05, 0x07, 0x00, 0x18, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x80}));
	TRY(tc358764_dsi_write(tc358764, 0xcf, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x87}));
	TRY(tc358764_dsi_write(tc358764, 0xcf, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x90}));
	TRY(tc358764_dsi_write(tc358764, 0xcf, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x97}));
	TRY(tc358764_dsi_write(tc358764, 0xcf, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xa0}));
	TRY(tc358764_dsi_write(tc358764, 0xcf, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xa7}));
	TRY(tc358764_dsi_write(tc358764, 0xcf, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xb0}));
	TRY(tc358764_dsi_write(tc358764, 0xcf, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xb7}));
	TRY(tc358764_dsi_write(tc358764, 0xcf, 7, (u8[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xc0}));
	TRY(tc358764_dsi_write(tc358764, 0xcf, 7, (u8[]){0x3d, 0x02, 0x15, 0x20, 0x00, 0x00, 0x01}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xc7}));
	TRY(tc358764_dsi_write(tc358764, 0xcf, 4, (u8[]){0x81, 0x00, 0x03, 0x08}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xb5}));
	TRY(tc358764_dsi_write(tc358764, 0xc5, 6, (u8[]){0x3f, 0x6f, 0xff, 0x00, 0x6f, 0xff}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x90}));
	TRY(tc358764_dsi_write(tc358764, 0xf5, 4, (u8[]){0x02, 0x11, 0x02, 0x15}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x90}));
	TRY(tc358764_dsi_write(tc358764, 0xc5, 1, (u8[]){0x50}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x94}));
	TRY(tc358764_dsi_write(tc358764, 0xc5, 1, (u8[]){0x66}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xb2}));
	TRY(tc358764_dsi_write(tc358764, 0xf5, 2, (u8[]){0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xb4}));
	TRY(tc358764_dsi_write(tc358764, 0xf5, 2, (u8[]){0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xb6}));
	TRY(tc358764_dsi_write(tc358764, 0xf5, 2, (u8[]){0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xb8}));
	TRY(tc358764_dsi_write(tc358764, 0xf5, 2, (u8[]){0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x94}));
	TRY(tc358764_dsi_write(tc358764, 0xf5, 2, (u8[]){0x00, 0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xd2}));
	TRY(tc358764_dsi_write(tc358764, 0xf5, 2, (u8[]){0x06, 0x15}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0xb4}));
	TRY(tc358764_dsi_write(tc358764, 0xc5, 1, (u8[]){0xcc}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x00}));
	TRY(tc358764_dsi_write(tc358764, 0xE1, 1, (u8[]){0x00}));
	TRY(tc358764_dsi_write(tc358764, 0xE1, 1, (u8[]){0x25}));
	TRY(tc358764_dsi_write(tc358764, 0xE1, 1, (u8[]){0x34}));
	TRY(tc358764_dsi_write(tc358764, 0xE1, 1, (u8[]){0x41}));
	TRY(tc358764_dsi_write(tc358764, 0xE1, 1, (u8[]){0x52}));
	TRY(tc358764_dsi_write(tc358764, 0xE1, 1, (u8[]){0x5f}));
	TRY(tc358764_dsi_write(tc358764, 0xE1, 1, (u8[]){0x60}));
	TRY(tc358764_dsi_write(tc358764, 0xE1, 1, (u8[]){0x88}));
	TRY(tc358764_dsi_write(tc358764, 0xE1, 1, (u8[]){0x76}));
	TRY(tc358764_dsi_write(tc358764, 0xE1, 1, (u8[]){0x8d}));
	TRY(tc358764_dsi_write(tc358764, 0xE1, 1, (u8[]){0x77}));
	TRY(tc358764_dsi_write(tc358764, 0xE1, 1, (u8[]){0x63}));
	TRY(tc358764_dsi_write(tc358764, 0xE1, 1, (u8[]){0x63}));
	TRY(tc358764_dsi_write(tc358764, 0xE1, 1, (u8[]){0x4e}));
	TRY(tc358764_dsi_write(tc358764, 0xE1, 1, (u8[]){0x4a}));
	TRY(tc358764_dsi_write(tc358764, 0xE1, 1, (u8[]){0x3a}));
	TRY(tc358764_dsi_write(tc358764, 0xE1, 1, (u8[]){0x2b}));
	TRY(tc358764_dsi_write(tc358764, 0xE1, 1, (u8[]){0x1b}));
	TRY(tc358764_dsi_write(tc358764, 0xE1, 1, (u8[]){0x10}));
	TRY(tc358764_dsi_write(tc358764, 0xE1, 1, (u8[]){0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x00}));
	TRY(tc358764_dsi_write(tc358764, 0xE2, 1, (u8[]){0x00}));
	TRY(tc358764_dsi_write(tc358764, 0xE2, 1, (u8[]){0x25}));
	TRY(tc358764_dsi_write(tc358764, 0xE2, 1, (u8[]){0x34}));
	TRY(tc358764_dsi_write(tc358764, 0xE2, 1, (u8[]){0x41}));
	TRY(tc358764_dsi_write(tc358764, 0xE2, 1, (u8[]){0x52}));
	TRY(tc358764_dsi_write(tc358764, 0xE2, 1, (u8[]){0x5f}));
	TRY(tc358764_dsi_write(tc358764, 0xE2, 1, (u8[]){0x60}));
	TRY(tc358764_dsi_write(tc358764, 0xE2, 1, (u8[]){0x88}));
	TRY(tc358764_dsi_write(tc358764, 0xE2, 1, (u8[]){0x76}));
	TRY(tc358764_dsi_write(tc358764, 0xE2, 1, (u8[]){0x8d}));
	TRY(tc358764_dsi_write(tc358764, 0xE2, 1, (u8[]){0x77}));
	TRY(tc358764_dsi_write(tc358764, 0xE2, 1, (u8[]){0x63}));
	TRY(tc358764_dsi_write(tc358764, 0xE2, 1, (u8[]){0x74}));
	TRY(tc358764_dsi_write(tc358764, 0xE2, 1, (u8[]){0x4e}));
	TRY(tc358764_dsi_write(tc358764, 0xE2, 1, (u8[]){0x4a}));
	TRY(tc358764_dsi_write(tc358764, 0xE2, 1, (u8[]){0x3a}));
	TRY(tc358764_dsi_write(tc358764, 0xE2, 1, (u8[]){0x2b}));
	TRY(tc358764_dsi_write(tc358764, 0xE2, 1, (u8[]){0x1b}));
	TRY(tc358764_dsi_write(tc358764, 0xE2, 1, (u8[]){0x10}));
	TRY(tc358764_dsi_write(tc358764, 0xE2, 1, (u8[]){0x00}));
	TRY(tc358764_dsi_write(tc358764, 0x00, 1, (u8[]){0x00}));
	TRY(tc358764_dsi_write(tc358764, 0xff, 3, (u8[]){0xff, 0xff, 0xff}));

	msleep(1);

	TRY(tc358764_dsi_write(tc358764, 0x11, 0, NULL));

	msleep(1);

	TRY(tc358764_dsi_write(tc358764, 0x29, 0, NULL));

	msleep(1);

	TRY(tc358764_write_dsi_config(tc358764,
				      TC358764_REG_DSI_CONTROL,
				      1,
				      0x00a7));

	TRY(tc358764_write_dsi_config(tc358764,
				      TC358764_REG_DSI_CONTROL,
				      0,
				      0x8000));

	TRY(tc358764_write16(tc358764, 0x4, 0x44));

#undef TRY

 err:
	return ret;
}

static int tc358764_probe(struct i2c_client          *client,
			  const struct i2c_device_id *id)
{
	struct tc358764_pdata	*pdata = client->dev.platform_data;
	struct tc358764         *tc358764;
	u16                      version;
	int                      ret;

	tc358764 = kzalloc(sizeof(*tc358764), GFP_KERNEL);
	if (tc358764 == NULL) {
		ret = -ENOMEM;
		goto kzalloc_failed;
	}

	tc358764->client = client;

	i2c_set_clientdata(client, tc358764);

	if (pdata->set_power) {
		ret = pdata->set_power(1);
		if (ret) {
			goto set_power_failed;
		}
	}

	ret = tc358764_read16(tc358764, TC358764_REG_VERSION, &version);
	if (ret) {
		goto read_failed;
	}

	if (version != TC358764_VERSION) {
		ret = -EINVAL;
		dev_err(&client->dev,
			"Unexpected device version: expected %04x got %04x\n",
			TC358764_VERSION, version);
		goto bad_version;
	}


	ret = tc358764_rnb6_init(tc358764);
	if (ret) {
		goto init_failed;
	}

	dev_info(&client->dev, "bridge succesfully initialized\n");

	return 0;

 init_failed:
 bad_version:
 read_failed:
	pdata->set_power(0);
 set_power_failed:
	kfree(tc358764);
 kzalloc_failed:
	i2c_set_clientdata(client, NULL);
	return ret;
}

static int tc358764_remove(struct i2c_client *client)
{
	struct tc358764         *tc358764 = i2c_get_clientdata(client);
	struct tc358764_pdata	*pdata    = client->dev.platform_data;

	if (tc358764 == NULL) {
		return 0;
	}

	if (pdata->set_power) {
		pdata->set_power(0);
	}

	kfree(tc358764);

	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id tc358764_id[] = {
	{ DEVICE_NAME, 0 },
	{ /* EOT */ }
};

MODULE_DEVICE_TABLE(i2c, tc358764_id);

static struct i2c_driver tc358764_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_NAME,
	},
	.probe = tc358764_probe,
	.remove = tc358764_remove,
	.id_table = tc358764_id,
};

module_i2c_driver(tc358764_driver);

MODULE_AUTHOR("Lionel Flandrin <lionel.flandrin@parrot.com>");
MODULE_DESCRIPTION("kernel driver for tc358764");
MODULE_LICENSE("GPL");
