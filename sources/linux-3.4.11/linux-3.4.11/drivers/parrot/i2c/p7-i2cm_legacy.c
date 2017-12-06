/**
 * @file parrot5_i2cm.c
 *
 * @brief Parrot I2C Master driver
 *
 * Copyright (C) 2008 Parrot S.A.
 *
 * @author     david.guilloteau@parrot.com
 * @author     matthieu.castet@parrot.com
 * @date       2008-08-28
 *
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

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/i2c.h>

#include <linux/pinctrl/consumer.h>

#include <asm/io.h>
#include "p7-i2cm.h"
#include "p7-i2cm_regs.h"

/* compat value */
#define I2CM_ITEN_ITDIS 0

#define I2CM_COMMAND_STA I2CM_COMMAND_START
#define I2CM_COMMAND_STO I2CM_COMMAND_STOP

#include "p7-i2cm_debugfs.h"

#define MULTI_MASTER_SUPPORT 0

#if 0
#define dprintk(...) dev_dbg(drv_data->dev, __VA_ARGS__)
#else
static int trace_addr = -1;
module_param(trace_addr, int, 0644);
#define dprintk(...) do { \
	if ((drv_data->msgs && trace_addr == drv_data->msgs[drv_data->msgs_idx].addr) || trace_addr == 0xffff) \
		dev_info(drv_data->dev,__VA_ARGS__); \
	} while (0)

#endif

#define DEFAULT_TIMEOUT	20000 //in us
#define parrot5_i2cm_wait_transfert(a,b) parrot5_i2cm_wait_transfert_timeout(a,b,DEFAULT_TIMEOUT)

#define PARROT5_I2CM_NAME P7I2CM_DRV_NAME

#define PARROT5_I2CM_TIMEOUT 10000 // in ms

enum {
	DELAY_DEBUG_IRQ_BACK = 1,
	DELAY_DEBUG_IRQ_AACK = 2,
};
static int test_driver = 0;

static int useirq = 1;
module_param(useirq, int, 0644);

static int usefifo = 1;
module_param(usefifo, int, 0644);

enum parrot5_i2cm_pm_mode {
	RUNNING,
	SUSPENDING,
};

struct parrot5_i2cm_data {

	//spinlock_t		        lock;
	wait_queue_head_t	        wait;
	int wait_up;
	int stop_sent;
	int irq;

	struct i2c_msg* msgs;
	int msgs_num;
	int msgs_idx;
	int data_pos;
	int state;
	unsigned int                    status;

	struct i2c_adapter	        adapter;
	struct p7i2cm_plat_data	*pdata;

	struct clk		        *clk;

	struct resource		        *ioarea;
	void __iomem		        *base;

	int rmsgs_idx;
	int rdata_pos;

	int usefifo;
	int hs_mode;

	struct device *dev;

	struct pinctrl                  *pctl;
	enum parrot5_i2cm_pm_mode        pm_mode;
#ifdef CONFIG_I2CM_PARROT7_DEBUGFS
		struct p7i2cm_debugfs            debug;
#endif /* CONFIG_I2CM_PARROT7_DEBUGFS */
};

enum {
	RESTART,
	STOP,
	DATA_W,
	DATA_R,
	HS_MCODE,
};

/**
 * Interrupt functions
 */
static int atomic_context(struct parrot5_i2cm_data *drv_data)
{
	return in_interrupt() || in_atomic() ||
		/*oops_in_progress ||*/ drv_data->pm_mode != RUNNING;
}
static int irq_on(struct parrot5_i2cm_data *drv_data)
{
	if (atomic_context(drv_data))
		return 0;
	return useirq;
}

static void parrot5_i2cm_show_debug(struct parrot5_i2cm_data *drv_data)
{
	u32 reg = readl(drv_data->base + I2CM_COMMAND);
	dev_warn(drv_data->dev, "debug cmd : SDA(%sB%d) SCL(%sB%dP7%d) %x\n",
			reg & (1 << 13) ? "S" : "M", /* 0 : p7 drive the line */
			!!(reg & (1 << 14)), /* unmasked real value of sda */
			reg & (1 <<  9) ? "S" : "M", /* 0 : p7 drive the line */
			!!(reg & (1 << 11)), /* unmasked real value of scl */
			!!(reg & (1 << 10)), /* Internal value of the P7 SCL output */
			reg);
	if ((reg & (1 << 13)) && (reg & (1 <<  9))) {
		if (!(reg & (1 << 14)))
			dev_warn(drv_data->dev, "a slave keep sda to 0!!!\n");
		if (!(reg & (1 << 11)))
			dev_warn(drv_data->dev, "a slave keep scl to 0!!!\n");
	}
}

static void parrot5_i2cm_acknowledge_irq(struct parrot5_i2cm_data *drv_data)
{
	writel(I2CM_ITACK_ACK, drv_data->base + I2CM_ITACK);
}

static void parrot5_i2cm_enable_irq(struct parrot5_i2cm_data *drv_data, int clean_status)
{
	u32 tmp;
	/* we need to clear irq before enable otherwise we got
	   spurious irq, if transfert were done in polling mode
	 */
	if (clean_status)
		parrot5_i2cm_acknowledge_irq(drv_data);

	tmp = I2CM_ITEN_ITEN;
	/* the I2CM_ITEN_DBG_FIFO irq will make the hw stop cmd fifo
	   in case of a nak during write
	 */
	if (drv_data->usefifo)
		tmp |= I2CM_ITEN_DBG_FIFO;
	writel(tmp, drv_data->base + I2CM_ITEN);
}

static void parrot5_i2cm_disable_irq(struct parrot5_i2cm_data *drv_data)
{
	writel(I2CM_ITEN_ITDIS, drv_data->base + I2CM_ITEN);
}

static int parrot5_i2cm_irq_active(struct parrot5_i2cm_data *drv_data)
{
	return readl(drv_data->base + I2CM_ITEN);
}

static int parrot5_i2cm_get_addr(struct i2c_msg *msg)
{
	int addr = (msg->addr & 0x7f) << 1;

	if (msg->flags & I2C_M_RD) {
		addr |= 1;
	}

	return addr;
}

static void parrot5_i2cm_write_byte_fifo(struct parrot5_i2cm_data *drv_data,
				   char byte, int stop)
{
	int ctrl;

	dprintk("write byte fifo %x (stop %d)\n", byte, stop);
	ctrl = I2CM_WFIFO_WR | byte;

	if (stop) {
		ctrl |= I2CM_WFIFO_STOP;
		drv_data->stop_sent = 1;
	}

	if (drv_data->hs_mode)
		ctrl |= I2CM_WFIFO_HIGH_SPEED;

	writel(ctrl, drv_data->base + I2CM_WFIFO);
}

static void parrot5_i2cm_read_byte_fifo(struct parrot5_i2cm_data *drv_data,
					struct i2c_msg* msg, int stop)
{
	int ctrl;
	int len;
	int noack = 0;
	len = msg->len - drv_data->data_pos;
	if (len > 255)
		len = 255;

	ctrl = I2CM_WFIFO_RD;

	if (len > 1)
		ctrl |= len;

	if (msg->len == len + drv_data->data_pos) {
		if (len == 1)
			ctrl |= I2CM_WFIFO_NACK;
		else
			ctrl |= I2CM_WFIFO_LAST_NACK;
		noack = 1;
	}

	if (stop && noack) {
		if (len == 1)
			ctrl |= I2CM_WFIFO_STOP;
		else
			ctrl |= I2CM_WFIFO_LAST_STOP;
		drv_data->stop_sent = 1;
	}

	if (drv_data->hs_mode)
		ctrl |= I2CM_WFIFO_HIGH_SPEED;

	drv_data->data_pos+= len;

	dprintk("read byte fifo len %d (noack %d, stop %d)\n", len, noack, stop);
	writel(ctrl, drv_data->base + I2CM_WFIFO);
}

static void parrot5_i2cm_start_message_fifo(struct parrot5_i2cm_data *drv_data,
				      struct i2c_msg* msg, int stop)
{
	int addr, ctrl;

	ctrl = (I2CM_WFIFO_START | I2CM_WFIFO_WR);
	/* send stop only if msg len is 0 (no data) */
	if (stop && msg->len == 0) {
		ctrl |= I2CM_WFIFO_STOP;
		drv_data->stop_sent = 1;
	}

	addr = parrot5_i2cm_get_addr(msg);

	ctrl |= addr;

	if (drv_data->state == HS_MCODE)
		ctrl |= I2CM_WFIFO_NACK;
	else if (drv_data->hs_mode)
		ctrl |= I2CM_WFIFO_HIGH_SPEED;

	dprintk("start fifo addr %x (stop %d)\n", addr, stop && msg->len == 0);
	writel(ctrl, drv_data->base + I2CM_WFIFO);
}


static void parrot5_i2cm_write_byte(struct parrot5_i2cm_data *drv_data,
				   char byte, int stop)
{
	int ctrl;

	if (drv_data->usefifo) {
		parrot5_i2cm_write_byte_fifo(drv_data, byte, stop);
		return;
	}

	dprintk("write byte %x (stop %d)\n", byte, stop);
	ctrl = I2CM_COMMAND_WR;

	if (stop) {
		ctrl |= I2CM_COMMAND_STO;
		if (drv_data->pdata->muxed)
			ctrl |= I2CM_COMMAND_MASK_SDA|I2CM_COMMAND_MASK_SCL;
		drv_data->stop_sent = 1;
	}

	if (drv_data->hs_mode)
		ctrl |= I2CM_COMMAND_HIGH_SPEED;

	writel(byte, drv_data->base + I2CM_TRANSMIT);
	writel(ctrl, drv_data->base + I2CM_COMMAND);
}

static void parrot5_i2cm_read_byte(struct parrot5_i2cm_data *drv_data,
					struct i2c_msg* msg,
				  int noack, int stop)
{
	int ctrl;
	dprintk("read byte (noack %d, stop %d)\n", noack, stop);

	ctrl = I2CM_COMMAND_RD;

	if (stop) {
		ctrl |= I2CM_COMMAND_STO;
		if (drv_data->pdata->muxed)
			ctrl |= I2CM_COMMAND_MASK_SDA|I2CM_COMMAND_MASK_SCL;
		drv_data->stop_sent = 1;
	}

	if (noack) {
		ctrl |= I2CM_COMMAND_NACK;
	}
	else if (!(msg->flags & I2C_M_NO_RD_ACK)) {
		ctrl |= I2CM_COMMAND_ACK; /* XXX do nothing : bit already set to 0 */
	}

	if (drv_data->hs_mode)
		ctrl |= I2CM_COMMAND_HIGH_SPEED;

	writel(ctrl, drv_data->base + I2CM_COMMAND);
}

static void parrot5_i2cm_start_message(struct parrot5_i2cm_data *drv_data,
				      struct i2c_msg* msg, int stop)
{
	int addr, ctrl;

	if (drv_data->usefifo) {
		parrot5_i2cm_start_message_fifo(drv_data, msg, stop);
		return;
	}

	ctrl = (I2CM_COMMAND_STA | I2CM_COMMAND_WR);
	/* send stop only if msg len is 0 (no data) */
	if (stop && msg->len == 0) {
		ctrl |= I2CM_COMMAND_STO;
		if (drv_data->pdata->muxed)
			ctrl |= I2CM_COMMAND_MASK_SDA|I2CM_COMMAND_MASK_SCL;
		drv_data->stop_sent = 1;
	}

	addr = parrot5_i2cm_get_addr(msg);

	if (drv_data->hs_mode && drv_data->state != HS_MCODE)
		ctrl |= I2CM_COMMAND_HIGH_SPEED;

	dprintk("start addr %x (stop %d)\n", addr, stop && msg->len == 0);
	writel(addr, drv_data->base + I2CM_TRANSMIT);
	writel(ctrl, drv_data->base + I2CM_COMMAND);
}

static void parrot5_i2cm_send_stop(struct parrot5_i2cm_data *drv_data)
{
	int ctrl;
	dprintk("stop\n");
	drv_data->stop_sent = 1;

	ctrl = I2CM_COMMAND_STO;
	if (drv_data->pdata->muxed)
		ctrl |= I2CM_COMMAND_MASK_SDA|I2CM_COMMAND_MASK_SCL;

	if (drv_data->hs_mode)
		ctrl |= I2CM_COMMAND_HIGH_SPEED;

	/* write to this register also clear the fifo */
	writel(ctrl, drv_data->base + I2CM_COMMAND);
}

static int parrot5_i2cm_wait_transfert_timeout(struct parrot5_i2cm_data *drv_data,
				u32 flags, unsigned long wait)
{
	u32 status;
	int ret = -1;
	//in case of no irq, jiffies won't update.
	long timeout = 1000000;

	/* wait the end of current transfert */
	do {
		status = readl(drv_data->base + I2CM_STATUS);
		if ((status & flags) == 0) {
			ret = 0;
			break;
		}
		if (atomic_context(drv_data)) {
			cpu_relax();
			timeout -= 10;
		}
		else {
			if (wait < 100) // < 100 us
				udelay(wait); //active wait
			else if (wait < 10000) // 10 ms
				usleep_range(wait, wait);
			else
				msleep(wait);

			timeout -= wait;
		}
	} while (timeout > 0);
	return ret;
}

static int parrot5_i2cm_badstatus(u32 status, int busy_check)
{
	if (status & (I2CM_STATUS_TIP|I2CM_STATUS_IF|I2CM_STATUS_FIFO_FILLED))
		return 1;
	if (busy_check && (status & I2CM_STATUS_BUSY))
		return 1;
	if ((status & I2CM_STATUS_CMD_EMPTY) != I2CM_STATUS_CMD_EMPTY)
		return 1;
	return 0;
}

/* reset controller state :
 wait end of TIP, [end of busy].
 clean irq [and busy]

 busy is touched only is clean_busy is true (we should own the bus)
 */
static int parrot5_i2cm_reset(struct parrot5_i2cm_data *drv_data, int clean_busy)
{
	u32 status, bus_state;
	int ret;
	int err = -1;
	int irq_on = parrot5_i2cm_irq_active(drv_data);
	/* disable irq */
	if (irq_on)
		parrot5_i2cm_disable_irq(drv_data);

	/* wait a stable state */
	ret = parrot5_i2cm_wait_transfert(drv_data, I2CM_STATUS_TIP);
	if (ret) {
		dev_warn(drv_data->dev,
				"timeout while waiting the end of current transfert\n");
		goto exit;
	}
#if 0
	status = readl(drv_data->base + I2CM_STATUS);
	if (clean_busy && drv_data->stop_sent && (status & I2CM_STATUS_BUSY)) {
		ret = parrot5_i2cm_wait_transfert(drv_data, I2CM_STATUS_BUSY);
		if (ret) {
			dev_warn(drv_data->dev, "timeout while waiting busy\n");
		}
	}
#endif
	bus_state = readl(drv_data->base + I2CM_COMMAND);

	/* P7 don't drive SDA but it is 0 */
	if ((bus_state & (1 << 13)) && !(bus_state & (1 << 14))) {
		dev_warn(drv_data->dev, "try bus defreeze\n");
		/* try to defreeze sda */
		writel(I2CM_COMMAND_BUS_CLEAR, drv_data->base + I2CM_COMMAND);
		ret = parrot5_i2cm_wait_transfert_timeout(drv_data, I2CM_STATUS_TIP, 25);
		if (ret) {
			dev_warn(drv_data->dev, "timeout while clearing bus\n");
			goto exit;
		}

		/* check if it worked */
		bus_state = readl(drv_data->base + I2CM_COMMAND);
		if ((bus_state & (1 << 13)) && !(bus_state & (1 << 14))) {
			dev_warn(drv_data->dev, "try bus defreeze #2\n");
			/* try to defreeze sda */
			writel(I2CM_COMMAND_BUS_CLEAR, drv_data->base + I2CM_COMMAND);
			ret = parrot5_i2cm_wait_transfert_timeout(drv_data, I2CM_STATUS_TIP, 25);
			if (ret) {
				dev_warn(drv_data->dev, "timeout while clearing bus #2\n");
				goto exit;
			}
		}
	}

	status = readl(drv_data->base + I2CM_STATUS);

	/* send stop if needed */
	if (clean_busy && (status & I2CM_STATUS_BUSY)) {
		parrot5_i2cm_send_stop(drv_data);
		ret = parrot5_i2cm_wait_transfert_timeout(drv_data, I2CM_STATUS_TIP, 10);
		if (ret) {
			dev_warn(drv_data->dev, "timeout while sending stop bit\n");
			goto exit;
		}
	}

	err = 0;
exit:

	if (irq_on)
		parrot5_i2cm_enable_irq(drv_data, 1);
	return err;
}

static void parrot5_i2cm_read_fifo(struct parrot5_i2cm_data *drv_data)
{
	u32 data;
	while (1) {
		struct i2c_msg* msg;
		dprintk("i2cm_read_fifo msg idx %d, data pos %d\n",
				drv_data->rmsgs_idx, drv_data->rdata_pos);

		if (drv_data->rmsgs_idx >= drv_data->msgs_num) {
			/* we will check that fifo is empty at the end
			   of transaction
			 */
			break;
		}

		msg = &drv_data->msgs[drv_data->rmsgs_idx];

		if (!(msg->flags & I2C_M_RD)) {
			/* find next read descriptor */
			drv_data->rmsgs_idx++;
			continue;
		}
		data = readl(drv_data->base + I2CM_RFIFO);
		if (data & I2CM_RFIFO_EMPTY)
			break;
		if (drv_data->rdata_pos >= msg->len) {
			dev_warn(drv_data->dev, "i2c read fifo overlfow\n");
			break;
		}
		msg->buf[drv_data->rdata_pos] = data & 0xff;
		dprintk("i2c read 0x%x\n", msg->buf[drv_data->rdata_pos]);
		drv_data->rdata_pos++;

		if (drv_data->rdata_pos >= msg->len) {
			/* go to next descriptor */
			drv_data->rmsgs_idx++;
			drv_data->rdata_pos = 0;
		}
	}
}

static void parrot5_i2cm_irq_(struct parrot5_i2cm_data *drv_data, unsigned int status1, const char *mode)
{
	struct i2c_msg* msg = &drv_data->msgs[drv_data->msgs_idx];
	int noack;
	int last_msg;
	/* NOTE : we need to read status after ack to not miss event,
	but some even are clear by ack (BAD_x) ... */
	unsigned int status = readl(drv_data->base + I2CM_STATUS) | status1;

	dprintk("%s status %x state %d\n", mode,
			status, drv_data->state);

	/* BAD_STOP is buggy in case we send start stop in the same command */
	if (status1 & ~(I2CM_STATUS_BAD_ACK|I2CM_STATUS_AL|I2CM_STATUS_BAD_STOP)) {
		dev_warn(drv_data->dev, "protocol error : %s%s%s%s%s%s %x\n", 
				status1 & I2CM_STATUS_BAD_CMD ? "CMD":"",
				status1 & I2CM_STATUS_BAD_WR ? "WR":"",
				status1 & I2CM_STATUS_BAD_RD ? "RD":"",
				status1 & I2CM_STATUS_BAD_STOP ? "STOP":"",
				status1 & I2CM_STATUS_BAD_START ? "START":"",
				status1 & I2CM_STATUS_BAD_SCL ? "SCL":"",
				status1);
		parrot5_i2cm_show_debug(drv_data);
	}
	if (drv_data->wait_up) {
		/* If we are here we got a spurious irq. This should not
		   happen anymore : we are ack again irq when setting wait_up
		 */
		dev_warn(drv_data->dev, "irq after transfert end, status %x\n", status);
		return;
	}

	if (!drv_data->usefifo && (status & I2CM_STATUS_TIP)) {
		dev_warn(drv_data->dev, "TIP in irq ??? %x\n", status);
		return;
	}

	if (status & I2CM_STATUS_AL) {
		dev_warn(drv_data->dev, "ab lost detected \n");
		parrot5_i2cm_show_debug(drv_data);
		/* abitration lost */
		drv_data->status = I2CM_STATUS_AL;
		/* abort transfert without generating stop */
		drv_data->wait_up = 1;
	}
	else if (drv_data->usefifo) {
		/* I2C_M_IGNORE_NAK is not supported in this mode */
		if (status & I2CM_STATUS_BAD_ACK) {
			/* TODO in hs mode, we should detect master code failure */
			drv_data->status = I2CM_STATUS_RXACK;
			/* do we need to sent a stop ?
			   we don't know...
			   if the stop was after the aborted command it won't be sent
			   if the stop command was in the aborted command it will be sent (case of scaning with a command that do start write stop)

			   we assume that if we have tip, we are sending the stop command ...
			 */
			if (1 || ((status & I2CM_STATUS_BUSY) && !(status & I2CM_STATUS_TIP))) {
				drv_data->stop_sent = 0;
				drv_data->state = DATA_W;
			}
			status &= ~I2CM_STATUS_BAD_ACK;
		}
		if (status & I2CM_STATUS_FIFO_FILLED)
			parrot5_i2cm_read_fifo(drv_data);
	}
	else {
		if (drv_data->state == DATA_R) {
			WARN_ON(!(msg->flags & I2C_M_RD));
			/* read received data */
			msg->buf[drv_data->data_pos] = readl(drv_data->base + I2CM_RECEIVE);
			dprintk("i2c read 0x%x\n", msg->buf[drv_data->data_pos]);
			drv_data->data_pos++;
		}
		else if (drv_data->state == HS_MCODE) {
			if (!(status & I2CM_STATUS_RXACK)) {
				drv_data->status = I2CM_STATUS_RXACK;
			}
		}
		else if ((status & I2CM_STATUS_RXACK) && !(msg->flags & I2C_M_IGNORE_NAK)) {
			/* nak : a stop we be sent to finish the transfert */
			drv_data->status = I2CM_STATUS_RXACK;
		}
	}

	/* if it was last cmd and transfert is finished, return */
	if (drv_data->stop_sent || drv_data->wait_up) {
		if (!(status & I2CM_STATUS_TIP) /* not in transfert and no cmd in fifo */
			&& (!(drv_data->usefifo)
				|| (drv_data->usefifo
				&& (status & I2CM_STATUS_CMD_EMPTY)))) {

			drv_data->wait_up = 1;
			/* TODO if we do multimaster, we need to mask FIFOCMD_ERROR
			   when transfert is finished, otherwise we will get
			   BAD_SCL each time others master use the clock
			 */
			/* NOTE re ack to clear spurious irq */
			parrot5_i2cm_acknowledge_irq(drv_data);
			wake_up(&drv_data->wait);
		}
		goto exit;
	}

resubmit:
	if (drv_data->usefifo && ((status & I2CM_STATUS_CMD_FULL) ||
				(status & I2CM_STATUS_BAD_ACK)))
		goto exit;

	BUG_ON(drv_data->state == STOP);

	/* after here we will sumbit a new command */

	if (drv_data->status) {
		/* nak, abort transfert : we didn't submit stop if we are here */
		drv_data->state = STOP;
	}
	else if (drv_data->state == HS_MCODE) {
		drv_data->state = RESTART;
	}
	else if (drv_data->state == RESTART && msg->len) {
		if (msg->flags & I2C_M_RD)
			drv_data->state = DATA_R;
		else
			drv_data->state = DATA_W;
	}
	else {
		if (drv_data->data_pos >= msg->len) {
			/* we are at the end of the data buffer,
			   check next operation */
			if (drv_data->msgs_idx + 1 >= drv_data->msgs_num) {
				/* no more operation */
				drv_data->state = STOP;
				WARN_ON(1);
			}
			else {
				drv_data->data_pos = 0;
				drv_data->msgs_idx++;
				msg = &drv_data->msgs[drv_data->msgs_idx];
#if 0
				/* XXX we could avoid restart between some transaction
				   I2C_M_NOSTART but we don't support it ATM
				 */
				if ((msg->flags & I2C_M_NOSTART) && (
						((msg->flags & I2C_M_RD) && drv_data->state == DATA_R) ||
						(!(msg->flags & I2C_M_RD) && drv_data->state == DATA_W)
				   ))
					dprintk("restart skip\n");
				else
#endif
				/* restart */
					if (!((msg->flags & I2C_M_NOSTART) && msg->len))
					drv_data->state = RESTART;
			}
		}
	}
	dprintk("data pos %d, len %d\n", drv_data->data_pos, msg->len);
	dprintk("msg idx %d, num %d\n", drv_data->msgs_idx, drv_data->msgs_num);

	/* for read nack the last read */
	noack = (drv_data->data_pos + 1 == msg->len);
	/* send stop when last data (noack) and last cmd */
	last_msg = (drv_data->msgs_idx + 1 == drv_data->msgs_num);

	switch (drv_data->state) {
		case DATA_W:
			parrot5_i2cm_write_byte(drv_data,
				msg->buf[drv_data->data_pos], noack && last_msg);
			drv_data->data_pos++;
		break;
		case DATA_R:
			if (drv_data->usefifo)
				parrot5_i2cm_read_byte_fifo(drv_data, msg, last_msg);
			else
				parrot5_i2cm_read_byte(drv_data, msg, noack, noack && last_msg);
		break;
		case RESTART:
			parrot5_i2cm_start_message(drv_data, msg, last_msg);
		break;
		case STOP:
			parrot5_i2cm_send_stop(drv_data);
		break;
	}

	if (drv_data->usefifo && !drv_data->stop_sent) {
		status = readl(drv_data->base + I2CM_STATUS);
		dprintk("resubmit status %x state %d\n", status, drv_data->state);
		goto resubmit;
	}

exit:
	return;
}

static irqreturn_t parrot5_i2cm_irq(int irq, void *dev_id)
{
	struct parrot5_i2cm_data *drv_data = dev_id;
	unsigned int status = readl(drv_data->base + I2CM_STATUS);

	if (!useirq)
		dev_warn(drv_data->dev, "i2c irq handler %x in poll mode\n", status);

	if ((status & I2CM_STATUS_IF) == 0)
		return IRQ_NONE;

	if (test_driver & DELAY_DEBUG_IRQ_BACK)
		udelay(100);
	/* acknowledge interrupt */
	parrot5_i2cm_acknowledge_irq(drv_data);
	if (test_driver & DELAY_DEBUG_IRQ_AACK)
		udelay(100);

	parrot5_i2cm_irq_(drv_data, status&(I2CM_STATUS_ERROR_MASK|I2CM_STATUS_AL), "irq");
	return IRQ_HANDLED;
}

/*
 * Calculate the correct value of the prescale registers.
 * the formula is: i2c_clk = in_clk / (12 * (PRESCALE + 1))
 *
 * The result is rounded *up* since prescale is a divider and we don't want the
 * output frequency to be superior to the specified speed.
 *
 * i2c_clk is in kHz
 */
static inline unsigned p7_i2cm_get_prescale(unsigned long in_clk,
		unsigned long i2c_clk)
{
	unsigned tmp;

	tmp = 12 * i2c_clk * 1000;

	tmp = (in_clk + tmp - 1) / tmp - 1;

	if (tmp == 0)
		tmp = 1;
	return tmp;
}
/**
 * Hardware init
 */
static int parrot5_i2cm_hw_init(struct parrot5_i2cm_data *drv_data, struct platform_device *pdev)
{
	int prescale = 0;
	int ret = 0;
	const char *const clk_name = "i2cm";

	/* enable I2C master clock */
	drv_data->clk = clk_get(&pdev->dev, clk_name);
	if (IS_ERR(drv_data->clk)) {
		ret = -EINVAL;
		goto exit;
	}
	ret = clk_prepare_enable(drv_data->clk);
	if (ret) {
		clk_put(drv_data->clk);
		ret = -EINVAL;
		goto exit;
	}

	/* Workaround for HW bug: RX FIFO might be left in an undefined state
	 * post RESET. Should only affect MPW1. */
	if (drv_data->pdata->revision == I2CM_REVISION_1) {
		writel(1, drv_data->base + I2CM_PRESCALE);
		clk_disable(drv_data->clk);
		clk_enable(drv_data->clk);
	}

	/* If the PRESCALE register is programmed with 0, the I2C master
	   is reset as if the chip was just powered on
	 */
	writel(0, drv_data->base + I2CM_PRESCALE);

	/* program prescale register */
	prescale = p7_i2cm_get_prescale(clk_get_rate(drv_data->clk),
			drv_data->pdata->bus_freq);
	/* Warning: we have to write a non-0 value to the I2CM_PRESCALE register
	   before we configure any other register */
	writel(prescale, drv_data->base + I2CM_PRESCALE);
	if (readl(drv_data->base + I2CM_PRESCALE) != prescale) {
		writel(0xffffffff, drv_data->base + I2CM_PRESCALE);

		WARN(1, KERN_ERR"i2cm prescaler is too big %d (max %d)\n",
				prescale, readl(drv_data->base + I2CM_PRESCALE));
	}

	if (drv_data->pdata->high_speed) {
		unsigned int high_prescale;
		high_prescale = p7_i2cm_get_prescale(clk_get_rate(drv_data->clk),
				drv_data->pdata->bus_high_freq);

		/* ignore if hs speed is low than normal speed */
		if (high_prescale >= prescale) {
			drv_data->hs_mode = 0;
		}
		else {
			writel(high_prescale, drv_data->base + I2CM_HIGH_PRESCALE);
			drv_data->hs_mode = 1;
		}
	}

	/* interrupt mode  */
	parrot5_i2cm_enable_irq(drv_data, 1);

exit:
	return ret;
}

static int parrot5_i2cm_master_start(struct parrot5_i2cm_data *drv_data,
				    struct i2c_msg *msgs,
				    int num)
{
	unsigned long flags;
	int time_left;
	u32 status;

	drv_data->msgs = msgs;
	drv_data->msgs_num = num;
	drv_data->msgs_idx = 0;
	drv_data->data_pos = 0;
	drv_data->rmsgs_idx = 0;
	drv_data->rdata_pos = 0;
	drv_data->state = RESTART;
	drv_data->status = 0;
	drv_data->stop_sent = 0;
	drv_data->wait_up = 0;
	drv_data->usefifo = usefifo;

	if (drv_data->usefifo && irq_on(drv_data) == 0) {
		/* we need error status that are available only with irq ! In theory
		 we could disable irq at interrupt controller level like it is done
		 in fifo case */
		dev_err(drv_data->dev, "i2c fifo mode without irq is not supported\n");
		drv_data->usefifo = 0;
	}

	if (irq_on(drv_data) == 0) {
		/* irq off, no fifo */
		parrot5_i2cm_disable_irq(drv_data);
	}
	else {
		/* irq on */
		parrot5_i2cm_enable_irq(drv_data, 0);
	}

	/* if another master take the bus before we issue the start condition we
	   have a race. We should make the busy check + sending start in less
	   than 4 µs @ 100Khz. That with we disable irq.

	   an disassembling show the lock is taken for 30 instructions.
	 */
	local_irq_save(flags);
	status = readl(drv_data->base + I2CM_STATUS);
	if (status & I2CM_STATUS_BUSY) {
		local_irq_restore(flags);
		dev_info(drv_data->dev, "i2c bus is busy. Waiting...\n");
		return -EBUSY;
	}

	if (irq_on(drv_data) && drv_data->usefifo)
		/* we need error status that are available only with irq enable in IP
		   so don't disable irq on IP level but on interrupt controller level */
		disable_irq(drv_data->irq);

	if (drv_data->hs_mode) {
		/* In high speed mode we have to send a master code for
		 * arbitration. This transfer is done in fast-mode (or slower). */
		struct i2c_msg master_code = {
			.addr = (0x8 | (drv_data->pdata->master_id & 0x7)) >> 1,
			.flags = (drv_data->pdata->master_id & 1) ? I2C_M_RD : 0,
			.len = 0,
			.buf = NULL,
		};
		drv_data->state = HS_MCODE;
		parrot5_i2cm_start_message(drv_data, &master_code, 0);
	}
	else {
		parrot5_i2cm_start_message(drv_data, &msgs[0], num == 1);
	}
	/* we sent the start, we can release the irq lock */
	local_irq_restore(flags);
	if (drv_data->usefifo) {
		/* fill the fifo with irq off */
		parrot5_i2cm_irq_(drv_data, 0, "fifo fill");
		/* fifo is full or fill all our msgs, enable irq */
		if (irq_on(drv_data))
			enable_irq(drv_data->irq);
	}

	if (irq_on(drv_data)) {
		time_left = wait_event_timeout(drv_data->wait,
				drv_data->wait_up,
				drv_data->adapter.timeout);
	}
	else {
		/* polling mode no fifo */
		do {
			time_left = parrot5_i2cm_wait_transfert(drv_data, I2CM_STATUS_TIP);
			if (time_left) {
				dev_warn(drv_data->dev, "timeout while waiting the end of current transfert\n");
				time_left = 0;
				break;
			}
			time_left = 1;

			parrot5_i2cm_irq_(drv_data, 0, "poll");
		} while (!drv_data->wait_up);

		parrot5_i2cm_enable_irq(drv_data, 1);
	}

	if (time_left <= 0) {
		dev_warn(drv_data->dev, "xfer timeout %d, wake %d,%d\n", time_left,
				drv_data->stop_sent, drv_data->wait_up);
		/* clear fifo */
		writel(0, drv_data->base + I2CM_TRANSMIT);
		return -ETIMEDOUT;
	}

	if (drv_data->status == I2CM_STATUS_AL) {
		/* clear fifo */
		writel(0, drv_data->base + I2CM_TRANSMIT);
		dev_info(drv_data->dev, "i2c arbitration lost. Waiting...\n");
		return -EBUSY;
	}
	return num;
}

/**
 *  I2C transfer main function
 */
static int parrot5_i2cm_master_xfer(struct i2c_adapter *i2c_adap,
				    struct i2c_msg *msgs,
				    int num)
{
	struct parrot5_i2cm_data *drv_data = i2c_adap->algo_data;
	int ret;
	u32 status;
	int retry = 0;

retry_transfert:
	/* first check if i2c state look sane */
	status = readl(drv_data->base + I2CM_STATUS);

	/* i2c busy is checked later */
	if (parrot5_i2cm_badstatus(status, 0)) {
		dev_warn(drv_data->dev, "bad status before transfert %x, aborting\n", status);
		return -EREMOTEIO;
	}

	ret = parrot5_i2cm_master_start(drv_data, msgs, num);
	if (ret == -EINVAL)
		return -EINVAL;

	if (ret == -EBUSY)
		goto arbitration_error;

	/* check i2c state after transfert */
	status = readl(drv_data->base + I2CM_STATUS);

	if (parrot5_i2cm_badstatus(status, 1)) {
		dev_warn(drv_data->dev, "xfer end  hw status %x, status %d\n",
			status,
			drv_data->status);

		if (ret >= 0) {
			dev_warn(drv_data->dev, "i2c bus is bad but transfert did not report"
					"any error");
		}
		/* we own the bus and need to release it */
		parrot5_i2cm_reset(drv_data, 1);

		/* recheck i2c state after clean */
		status = readl(drv_data->base + I2CM_STATUS);

		if (parrot5_i2cm_badstatus(status, 1)) {
			dev_warn(drv_data->dev, "bad status after reset %x\n",
					status);
			parrot5_i2cm_show_debug(drv_data);
		}
	}

	if (ret == -ETIMEDOUT) {
		dev_warn(drv_data->dev, "i2c timeout\n");
		ret = -EREMOTEIO;
	}
	else if (drv_data->status) {
		dprintk("bad status : drv_data->status=%x\n", drv_data->status);
		ret = -EREMOTEIO;
	}

	return ret;

arbitration_error:
	/* an other master is driving the bus, wait for the stop condition */
	if (!MULTI_MASTER_SUPPORT || parrot5_i2cm_wait_transfert(drv_data, I2CM_STATUS_BUSY)) {
		dev_info(drv_data->dev, "i2c : ab timeout while waiting busy\n");

		/* we suppose we own the bus */
		parrot5_i2cm_reset(drv_data, 1);
		/* recheck i2c state after clean */
		status = readl(drv_data->base + I2CM_STATUS);

		if (parrot5_i2cm_badstatus(status, 1)) {
			dev_warn(drv_data->dev, "bad status after reset %x\n", status);
			parrot5_i2cm_show_debug(drv_data);
		}
		goto xfer_error;
	}
	if (retry >= 3) {
		dev_info(drv_data->dev, "i2c too much retry. Aborting.\n");
		goto xfer_error;
	}
	retry++;

	goto retry_transfert;

xfer_error:
	return -EREMOTEIO;
}

static u32 parrot5_i2cm_funct(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_PROTOCOL_MANGLING;
}

static const struct i2c_algorithm parrot5_i2cm_algorithm = {
	.master_xfer	= parrot5_i2cm_master_xfer,
	.functionality	= parrot5_i2cm_funct,
};

/* debug stuff */
static	ssize_t set_tests(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct platform_device *pdev;
	struct parrot5_i2cm_data *drv_data;

	pdev = container_of(dev, struct platform_device, dev);
	drv_data = platform_get_drvdata(pdev);

	if (!strncmp(buf, "hs_emu", count)) {
		/* set hs mode with speed = 2 * normal speed */
		u32 tmp = readl(drv_data->base + I2CM_PRESCALE);
		writel(tmp/2, drv_data->base + I2CM_HIGH_PRESCALE);
		drv_data->hs_mode = 1;
	}
	else if (!strncmp(buf, "hs_off", count)) {
		drv_data->hs_mode = 0;
		writel(0, drv_data->base + I2CM_HIGH_PRESCALE);
	}
	else if (!strncmp(buf, "bus_stop", count)) {
		int ret;
		u32 status = readl(drv_data->base + I2CM_STATUS);

		/* send stop if needed */
		if (status & I2CM_STATUS_BUSY) {
			parrot5_i2cm_send_stop(drv_data);
			ret = parrot5_i2cm_wait_transfert(drv_data, I2CM_STATUS_BUSY);
			if (ret) {
				dev_warn(drv_data->dev, "timeout while sending stop bit\n");
			}
		}
	}
	else if (!strncmp(buf, "delay_bef_ackno", count)) {
		test_driver = DELAY_DEBUG_IRQ_BACK;
	}
	else if (!strncmp(buf, "delay_aft_ackno", count)) {
		test_driver = DELAY_DEBUG_IRQ_AACK;
	}
	else if (!strncmp(buf, "nodelay_bef_ackno", count)) {
		test_driver &= ~DELAY_DEBUG_IRQ_BACK;
	}
	else if (!strncmp(buf, "nodelay_aft_ackno", count)) {
		test_driver &= ~DELAY_DEBUG_IRQ_AACK;
	}
	else {
		return -EINVAL;
	}

	return count;
}

static ssize_t get_tests(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	/* TODO */
	return scnprintf(buf, PAGE_SIZE, "hs_emu hs_off bus_stop delay_bef_ackno delay_bef_ackno nodelay_bef_ackno nodelay_bef_ackno\n");
}

static DEVICE_ATTR(tests, 0644, get_tests, set_tests);
/**
 * probe function
 */
int parrot5_i2cm_probe(struct platform_device *pdev)
{
	struct parrot5_i2cm_data *drv_data;
	struct resource *res;
	int ret;

	drv_data = kzalloc(sizeof(struct parrot5_i2cm_data), GFP_KERNEL);
	if (!drv_data) {
		ret = -ENOMEM;
		goto no_mem;
	}

	drv_data->dev = &pdev->dev;
	drv_data->pdata = pdev->dev.platform_data;
	if (!drv_data->pdata) {
		dev_err(&pdev->dev, "no driver data\n");
		ret = -ENODEV;
		goto no_pdata;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "get memory region resource failed\n");
		ret = -ENOENT;
		goto no_res;
	}

	drv_data->ioarea = request_mem_region(res->start,
					      res->end - res->start + 1,
					      pdev->name);
	if (!drv_data->ioarea) {
		dev_err(&pdev->dev, "request memory region failed\n");
		ret = -ENOENT;
		goto no_res;
	}

	drv_data->base = ioremap(res->start, res->end - res->start + 1);
	if (!drv_data->base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto no_map;
	}

	if (!drv_data->pdata->muxed) {
		drv_data->pctl = pinctrl_get_select_default(&pdev->dev);
		if (IS_ERR(drv_data->pctl)) {
			ret = PTR_ERR(drv_data->pctl);
			clk_put(drv_data->clk);
			goto no_pinctrl;
		}
	}
	else
		/* masking scl and sda does not work in fifo mode. To activate it
		   we need to program COMMAND register in parallel of fifo, and we
		   need to do an operation...
		   Programming COMMAND with MASK_SDA|MASK_SCL and writing to
		   I2CM_TRANSMIT do not work.
		   We assume that share i2c bus do not need fifo.
		 */
		usefifo = 0;

	ret = parrot5_i2cm_hw_init(drv_data, pdev);
	if (ret) {
		dev_err(&pdev->dev, "Hardware init failed (%d)\n", ret);
		goto err_hw;
	}

	init_waitqueue_head(&drv_data->wait);

	platform_set_drvdata(pdev, drv_data);

	ret = request_irq(platform_get_irq(pdev, 0),
			  parrot5_i2cm_irq, IRQF_SHARED,
			  pdev->name, drv_data);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed (%d)\n", ret);
		goto no_irq;
	}
	drv_data->irq = platform_get_irq(pdev, 0);


	/* setup info for the i2c core */
	memcpy(drv_data->adapter.name, pdev->name, strlen(pdev->name));
	drv_data->adapter.algo = &parrot5_i2cm_algorithm;
	drv_data->adapter.algo_data = drv_data;
	drv_data->adapter.dev.parent = &pdev->dev;
	drv_data->adapter.owner = THIS_MODULE;
	drv_data->adapter.timeout = msecs_to_jiffies(PARROT5_I2CM_TIMEOUT);

	/* we don't use i2c_add_adapter to allow
	   board config to use i2c_register_board_info
	 */
	drv_data->adapter.nr = pdev->id;
	i2c_add_numbered_adapter(&drv_data->adapter);

	dev_info(&pdev->dev, "controller probe successfully\n");

#ifdef CONFIG_I2CM_PARROT7_DEBUGFS
	p7i2cm_debugfs_init(&drv_data->debug, drv_data->base,
			dev_name(&pdev->dev),
			clk_get_rate(drv_data->clk));
#endif /* CONFIG_I2CM_PARROT7_DEBUGFS */
	device_create_file(&pdev->dev, &dev_attr_tests);

	drv_data->pm_mode = RUNNING;

	if (drv_data->pdata->revision == I2CM_REVISION_1)
		usefifo = 0;

	return 0;

no_irq:
err_hw:
no_pinctrl:
no_map:
	iounmap(drv_data->base);
	release_resource(drv_data->ioarea);
no_res:
no_pdata:
	kfree(drv_data);

no_mem:
	return ret;

}

/**
 * remove function
 */
int parrot5_i2cm_remove(struct platform_device *pdev)
{
	struct parrot5_i2cm_data *drv_data = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_tests);
#ifdef CONFIG_I2CM_PARROT7_DEBUGFS
	p7i2cm_debugfs_remove(&drv_data->debug);
#endif /* CONFIG_I2CM_PARROT7_DEBUGFS */

	i2c_del_adapter(&drv_data->adapter);
	parrot5_i2cm_disable_irq(drv_data);
	clk_disable_unprepare(drv_data->clk);
	clk_put(drv_data->clk);
	free_irq(platform_get_irq(pdev, 0), drv_data);

	if (drv_data->pctl)
		pinctrl_put(drv_data->pctl);

	iounmap(drv_data->base);

	release_resource(drv_data->ioarea);

	kfree(drv_data);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

void parrot5_i2cm_shutdown(struct platform_device *pdev)
{
	struct parrot5_i2cm_data        *drv_data = platform_get_drvdata(pdev);
	dev_info(&pdev->dev, "shutdown\n");
	drv_data->pm_mode = SUSPENDING;
}

#ifdef CONFIG_PM_SLEEP
int parrot5_i2cm_suspend_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct parrot5_i2cm_data        *drv_data = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "suspend\n");
	drv_data->pm_mode = SUSPENDING;
	clk_disable_unprepare(drv_data->clk);

	return 0;
}

int parrot5_i2cm_resume_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct parrot5_i2cm_data        *drv_data = platform_get_drvdata(pdev);
	int ret;

	drv_data->pm_mode = RUNNING;
	dev_info(&pdev->dev, "resuming\n");

	ret = parrot5_i2cm_hw_init(drv_data, pdev);
	if (ret) {
		dev_err(&pdev->dev, "Hardware init failed (%d)\n", ret);
		return ret;
	}

	return 0;
}

static struct dev_pm_ops parrot5_i2cm_dev_pm_ops = {
	.suspend_noirq = parrot5_i2cm_suspend_noirq,
	.resume_noirq = parrot5_i2cm_resume_noirq,
};
#endif

/* This code is commented because p7-i2cm_probe does the job of selecting the
    * correct implementation */
#if 1
/**
 * platform driver structure
 */
static struct platform_driver parrot5_i2cm_driver = {
	.probe		= parrot5_i2cm_probe,
	.remove		= parrot5_i2cm_remove,
	.shutdown    = parrot5_i2cm_shutdown,
	.driver		= {
		.owner	= THIS_MODULE,
		.name   = P7I2CM_DRV_NAME,
#ifdef CONFIG_PM_SLEEP
		.pm = &parrot5_i2cm_dev_pm_ops,
#endif
	},
};

/**
 * init function
 */
static int __init parrot5_i2cm_init(void)
{
	return platform_driver_register(&parrot5_i2cm_driver);
}
#ifdef MODULE
module_init(parrot5_i2cm_init);
#else
postcore_initcall(parrot5_i2cm_init);
#endif

/**
 * exit function
 */
static void __exit parrot5_i2cm_exit(void)
{
	platform_driver_unregister(&parrot5_i2cm_driver);
}
module_exit(parrot5_i2cm_exit);

MODULE_AUTHOR("Parrot SA by David Guilloteau, Matthieu CASTET");
MODULE_DESCRIPTION("Parrot I2C Master driver");
MODULE_LICENSE("GPL");
#endif
