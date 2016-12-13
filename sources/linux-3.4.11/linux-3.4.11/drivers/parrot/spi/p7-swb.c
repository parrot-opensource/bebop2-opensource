/**
 * p7-swb.c - Switchbox implementation
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Damien Riegel <damien.riegel.ext@parrot.com>
 * date:    26-Jul-2013
 *
 * This file is released under the GPL
 */

#include <linux/module.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include "p7-swb.h"
#include "p7-spi.h"
#include "p7-spi_priv.h"

static int pad_out_values[] = {
	[0 ... P7_SWB_FUNCTION_LEN] = -1,
	[P7_SWB_SPI_CLK]        = 0x0,
	[P7_SWB_SPI_SS]         = 0x1,
	[P7_SWB_SPI_DATA0]      = 0x2,
	[P7_SWB_SPI_DATA1]      = 0x3,
	[P7_SWB_SPI_DATA2]      = 0x4,
	[P7_SWB_SPI_DATA3]      = 0x5,
	[P7_SWB_LOW]            = 0x18,
	[P7_SWB_HIGH]           = 0x19,
	[P7_SWB_Z]              = 0x1a,
};

static int pad_in_registers[] = {
	[0 ... P7_SWB_FUNCTION_LEN] = -1,
	[P7_SWB_SPI_CLK]     = 0x00,
	[P7_SWB_SPI_SS]      = 0x04,
	[P7_SWB_SPI_DATA0]   = 0x08,
	[P7_SWB_SPI_DATA1]   = 0x0C,
	[P7_SWB_SPI_DATA2]   = 0x10,
	[P7_SWB_SPI_DATA3]   = 0x14,
	[P7_SWB_MPEG_CLK]    = 0x00,
	[P7_SWB_MPEG_DATA]   = 0x04,
	[P7_SWB_MPEG_VALID]  = 0x08,
	[P7_SWB_MPEG_SYNC]   = 0x0C,
};

static char* p7swb_function2str(enum p7swb_function func)
{
	char* str = NULL;
	switch(func) {
	case P7_SWB_SPI_CLK:
		str = "SPI_CLK";
		break;
	case P7_SWB_SPI_SS:
		str = "SPI_SS";
		break;
	case P7_SWB_SPI_DATA0:
		str = "SPI_DATA0";
		break;
	case P7_SWB_SPI_DATA1:
		str = "SPI_DATA1";
		break;
	case P7_SWB_SPI_DATA2:
		str = "SPI_DATA2";
		break;
	case P7_SWB_SPI_DATA3:
		str = "SPI_DATA3";
		break;
	case P7_SWB_MPEG_CLK:
		str = "MPEG_CLK";
		break;
	case P7_SWB_MPEG_DATA:
		str = "MPEG_DATA";
		break;
	case P7_SWB_MPEG_VALID:
		str = "MPEG_VALID";
		break;
	case P7_SWB_MPEG_SYNC:
		str = "MPEG_SYNC";
		break;
	default:
		str = "";
		break;
	}

	return str;
}

static bool p7swb_func_available(enum p7swb_function func,
                                 enum p7spi_core_type type)
{
	if (func > P7_SWB_FIRST_STATE &&
	    func < P7_SWB_LAST_STATE)
		return true;
	else if (type == P7SPI_CORE_SPI)
		return (func > P7_SWB_SPI_FIRST_FUNC &&
		        func < P7_SWB_SPI_LAST_FUNC);
	else if (type == P7SPI_CORE_MPEG)
		return (func > P7_SWB_MPEG_FIRST_FUNC &&
		        func < P7_SWB_MPEG_LAST_FUNC);
	else
		return false;
}

int p7swb_core_offset(enum p7spi_core_type type)
{
	if (type == P7SPI_CORE_SPI)
		return (P7_SWB_SPI_LAST_FUNC - P7_SWB_SPI_FIRST_FUNC - 1);
	else
		return (P7_SWB_MPEG_LAST_FUNC - P7_SWB_MPEG_FIRST_FUNC - 1);
}

static int p7swb_funcout2val(int core_id,
                             enum p7spi_core_type type,
                             enum p7swb_function func)
{
	int val = pad_out_values[func];

	if (val < 0) {
		dev_err(p7spi_kern.dev,
		        "switchbox: function %d is not available as output\n",
		        func);
		return -EINVAL;
	}

	if (func > P7_SWB_FIRST_STATE && func < P7_SWB_LAST_STATE) {
		if (p7spi_kern.rev != SPI_REVISION_1)
			/*
			 *            low      high   Z
			 * R1         0x18     0x19  0x1A
			 * R2 / R3  0x18-0x20  0x21  0x22
			 * val is the correct value for R1, so for latter rev
			 * add +2 to get the right value
			 */
			val += 2;
		return val;
	}
	else {
		return val + core_id * p7swb_core_offset(P7SPI_CORE_SPI);
	}
}

static int p7swb_funcin2reg(int core_id,
                            enum p7spi_core_type type,
                            enum p7swb_function func)
{
	int core_index, i;
	int address = 0;
	int val = pad_in_registers[func];

	if (val < 0) {
		dev_err(p7spi_kern.dev,
		        "switchbox: function %d is not available as input\n",
		       func);
		return -EINVAL;
	}

	core_index = p7spi_core_index(core_id, type);
	if (core_index < 0) {
		dev_err(p7spi_kern.dev,
			"switchbox: can't find core n°%d of type %d\n",
			core_id, type);
		return -EINVAL;
	}

	for(i = 0; i < core_index; i++) {
		address += p7swb_core_offset(p7spi_kern.cores[i]) * 4;
	}

	return address + val;
}

static struct p7swb_desc* p7swb_get_swb(unsigned int pad)
{
	if (pad >= p7spi_kern.swb_sz)
		return NULL;
	else
		return &p7spi_kern.swb[pad];
}

static int p7swb_claim(void const * const owner, unsigned int pad, bool in_pad)
{
	struct p7swb_desc* swb;

	swb = p7swb_get_swb(pad);
	if (!swb) {
		/*
		 * Maybe P7SPI_SWB_LAST is missing or the pad configuration
		 * is incorrect in the BSP.
		 */
		dev_err(p7spi_kern.dev,
		        "switchbox: can't get pad %d.\n", pad);
		return -EINVAL;
	}

	if (! swb->owner) {
		swb->owner = owner;
	}
	else if (swb->owner != owner) {
		dev_err(p7spi_kern.dev,
		        "pad %d already claimed\n", pad);
		return -EBUSY;
	}

	if (in_pad && !(swb->in_used))
		swb->in_used = true;

	if (!in_pad && !(swb->out_used))
		swb->out_used = true;

	return 0;
}

static void p7swb_unclaim(void const * const owner, unsigned int pad, bool in_pad)
{
	struct p7swb_desc* swb;

	swb = p7swb_get_swb(pad);
	if (!swb || swb->owner != owner)
		return;

	if (in_pad)
		swb->in_used = false;
	else
		swb->out_used = false;

	dev_dbg(p7spi_kern.dev,
	        "unclaiming pad %d (%s)\n",
	        pad, in_pad ? "in" : "out");

	if ((!swb->in_used) && (!swb->out_used)) {
		swb->owner = NULL;
		dev_dbg(p7spi_kern.dev,
		        "releasing pad %d\n", pad);
	}
}

int p7swb_request_pad(int core_id, enum p7spi_core_type type,
                        void const * const owner, unsigned int pad,
                        enum p7swb_direction dir, enum p7swb_function func)
{
	int val, err = -EINVAL;

	mutex_lock(&p7spi_kern.lck);

	if (!p7swb_func_available(func, type))
		goto err;
	
	if (dir == P7_SWB_DIR_OUT || dir == P7_SWB_DIR_BOTH) {
		val = p7swb_funcout2val(core_id, type, func);
		if (val >= 0) {
			err = p7swb_claim(owner, pad, false);
			if (err)
				goto err;
		
			__raw_writel(val, p7spi_kern.vaddr + P7SPI_SWB_OPAD(pad));
			dev_dbg(p7spi_kern.dev,
			        "claiming pad %d (out) for function %s\n",
			        pad, p7swb_function2str(func));
		}
		else {
			err = val;
			goto err;
		}

	}

	if (dir == P7_SWB_DIR_IN || dir == P7_SWB_DIR_BOTH) {
		val = p7swb_funcin2reg(core_id, type, func);
		if (val >= 0) {
			err = p7swb_claim(owner, pad, true);
			if (err)
				goto release_pad;

			__raw_writel(pad, p7spi_kern.vaddr +
			                     P7SPI_SWB_IPAD_BASE + val);
			dev_dbg(p7spi_kern.dev,
				"claiming pad %d (in) for function %s\n",
				pad, p7swb_function2str(func));
		}
		else {
			err = val;
			goto release_pad;
		}
	}

	if (!err)
		goto err;
	
release_pad:
	if (dir == P7_SWB_DIR_OUT || dir == P7_SWB_DIR_BOTH)
		p7swb_unclaim(owner, pad, false);
err:
	mutex_unlock(&p7spi_kern.lck);
	return err;

}
EXPORT_SYMBOL(p7swb_request_pad);

void p7swb_release_pad(void const * const owner, unsigned int pad,
                         enum p7swb_direction dir)
{
	mutex_lock(&p7spi_kern.lck);
	switch (dir) {
	case P7_SWB_DIR_OUT:
		p7swb_unclaim(owner, pad, false);
		break;
	case P7_SWB_DIR_IN:
		p7swb_unclaim(owner, pad, true);
		break;
	case P7_SWB_DIR_BOTH:
		p7swb_unclaim(owner, pad, false);
		p7swb_unclaim(owner, pad, true);
		break;
	}
	mutex_unlock(&p7spi_kern.lck);
}
EXPORT_SYMBOL(p7swb_release_pad);
