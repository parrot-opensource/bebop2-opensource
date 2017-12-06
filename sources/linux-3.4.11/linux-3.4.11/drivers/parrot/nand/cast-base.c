/**
 * CAST Nand Controller Driver
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:	G.Tian <guangye.tian@parrot.com>
 * date:	2012-10-24
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/jiffies.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/jiffies.h>
#include <linux/pinctrl/consumer.h>
#include "cast-regs.h"
#include "cast-nand.h"
#include "cast-timings.h"

#define NF_TRANSFER_WRITE	0
#define NF_TRANSFER_READ	1

/* bad-block marker bytes */
#define BBM_BYTES		2

static int usedma = 1;
module_param(usedma, int, 0644);

static int synmode = 0;
module_param(synmode, int, 0644);

static int eccbits = 0;
module_param(eccbits, int, 0644);

static int cast_tim_asyn_wr;
static int cast_tim_asyn_rd;

static int cast_wait_ready(struct mtd_info *mtd, int irq);

struct cast_nand_host {
	struct nand_chip        nand_chip;
	struct mtd_info         mtd;
	void __iomem            *regs;

	/* First 5 bytes of read id 0x00 command */
	u8			id[8];

	/* page per block, complementing mtd_info */
	u32                     ppb;
	int                     ecc_bits;

	/*
	 * data is read word by word from cast
	 * unused bytes should be saved.
	 */
	u8                      data_buf[4];
	u32                     data_flag; /* 1: data_buf contains data 0: no */
	u32                     byte_idx;

	/* DMA operations */
	dma_addr_t              dmaaddr;
	unsigned char           *dmabuf;

	u32                     cmd_precd;
	struct completion       complete;

	int                     mode_curr;
	struct onfi_par_page    par_page;

	int                     chip_nr;
	struct clk*             clk_ctrl;
	struct clk*             clk_phy;
	struct pinctrl          *pctl;
	struct resource         *area;

	/*
	 * programmed-page marker size in spare area
	 * ppm follows bad block marker (offset 2)
	 */
	uint32_t ppm;

	uint32_t ecc_threshold;

	int benand;
};

#define cast_readl(_host, _reg) readl(_host->regs + _reg)
#define cast_writel(_val, _host, _reg) writel(_val, _host->regs + _reg)

static struct nand_ecclayout cast_ecc_oob_layout;

static ssize_t cast_read_ctrl(struct device *dev, char *buf, int reg)
{
	struct platform_device *pdev;
	struct cast_nand_host *host;
	u32 reg_val;

	pdev = container_of(dev, struct platform_device, dev);
	host = platform_get_drvdata(pdev);
	reg_val = cast_readl(host, reg);

	return scnprintf(buf, PAGE_SIZE, "0x%08x\n", reg_val);
}

static ssize_t get_rom_ctrl_reg(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct platform_device *pdev;
	struct cast_nand_host *host;
	u32 reg_val;

	pdev = container_of(dev, struct platform_device, dev);
	host = platform_get_drvdata(pdev);

	reg_val = cast_readl(host, CAST_CONTROL);
	reg_val &= ~(CAST_CTRL_SPARE_EN       |
		     CAST_CTRL_INT_EN         |
		     CAST_CTRL_CUSTOM_SIZE_EN |
		     CAST_CTRL_IO_WIDTH_16    |
		     CAST_CTRL_SYNC_MODE);
	reg_val |= CAST_CTRL_ECC_EN;

	return scnprintf(buf, PAGE_SIZE, "0x%08x\n", reg_val);
}

static ssize_t get_rom_eccctrl_reg(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	return cast_read_ctrl(dev, buf, CAST_ECC_CTRL);
}

static ssize_t get_rom_eccoff_reg(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	return cast_read_ctrl(dev, buf, CAST_ECC_OFFSET);
}

static ssize_t get_rom_gen_seq_reg(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	/* customized nand read/write sequence for rom */
	u32 reg_val = 0x30053;
	return scnprintf(buf, PAGE_SIZE, "0x%08x\n", reg_val);
}

static ssize_t get_rom_time_seq0_reg(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	u32 reg_val = 0x090d0d1f;
	return scnprintf(buf, PAGE_SIZE, "0x%08x\n", reg_val);
}

static ssize_t get_rom_time_seq1_reg(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	u32 reg_val = 0x0000081f;
	return scnprintf(buf, PAGE_SIZE, "0x%08x\n", reg_val);
}

static ssize_t get_rom_time_asyn_reg(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	u32 reg_val = 0x00000084;
	return scnprintf(buf, PAGE_SIZE, "0x%08x\n", reg_val);
}

static DEVICE_ATTR(rom_ctrl, 0644, get_rom_ctrl_reg, NULL);
static DEVICE_ATTR(rom_ecc_ctrl, 0644, get_rom_eccctrl_reg, NULL);
static DEVICE_ATTR(rom_ecc_offset, 0644, get_rom_eccoff_reg, NULL);
static DEVICE_ATTR(rom_gen_seq_ctrl, 0644, get_rom_gen_seq_reg, NULL);
static DEVICE_ATTR(rom_time_seq_0, 0644, get_rom_time_seq0_reg, NULL);
static DEVICE_ATTR(rom_time_seq_1, 0644, get_rom_time_seq1_reg, NULL);
static DEVICE_ATTR(rom_time_asyn, 0644, get_rom_time_asyn_reg, NULL);

static irqreturn_t cast_irq(int irq, void *dev_id)
{
	struct cast_nand_host *host = dev_id;
	u32 status = cast_readl(host, CAST_INT_STATUS);
	if (status == 0)
		return IRQ_NONE;
	/* clear irq */
	cast_writel(0, host, CAST_INT_STATUS);

	complete(&host->complete);

	return IRQ_HANDLED;
}

/**
 * Wait until a register flag reaches a target value.
 *
 * @param host		cast_nand_host structure pointer
 * @param reg           Register to poll
 * @param mask          Mask to apply to register value
 * @param target        Set to 0 or 1 to indicate target value (0 or mask)
 * @param timeo		Timeout value in jiffies
 *
 * @return              0 if successful, -1 upon timeout
 */
static int wait_for_flag(struct cast_nand_host *host,
			 uint32_t reg, uint32_t mask,
			 uint32_t target, unsigned long timeo)
{
	uint32_t busy = 0;
	timeo += jiffies;

	target = target ? mask : 0;

	busy = ((cast_readl(host, reg) & mask) != target);
	while (busy) {
		busy = ((cast_readl(host, reg) & mask) != target);
		if (time_is_before_jiffies(timeo)) {
			break;
		}
		cpu_relax();
	}

	return busy ? -1 : 0;
}

/**
 * Wait target ready
 *
 * @param host		cast_nand_host structure pointer
 * @param timeo		timeout in jiffies
 *
 * @return		0 if successful; -1 upon timeout
 */
static int wait_until_target_rdy(struct cast_nand_host *host,
				 unsigned long timeo)
{
	uint32_t busy;

	/* wait for controller */
	busy = wait_for_flag(host, CAST_STATUS, CAST_STAT_CTRL_STAT,
			     0, timeo);

	/* wait for device */
	if (!busy)
		wait_for_flag(host, CAST_STATUS,
			      CAST_STAT_MEMX_ST(host->chip_nr), 1, timeo);

	return busy;
}

/* Wait for the nand flash RNB line to get high */
static int cast_wait_ready(struct mtd_info *mtd, int irq)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct cast_nand_host *host = nand_chip->priv;
	unsigned long timeo;
	int res;
	int err = 0;

	if (nand_chip->state == FL_ERASING) {
		timeo = msecs_to_jiffies(4000);
	} else {
		timeo = msecs_to_jiffies(2000);
	}

	if (irq) {
		res = wait_for_completion_timeout(&host->complete, timeo);
		if (!res) {
			dev_err(mtd->dev.parent, "Timeout in irq\n");
			err = -1;
		}
		/* Disable all interrupts */
		cast_writel(0, host, CAST_INT_MASK);
	} else {
		err = wait_until_target_rdy(host, timeo);
	}
	return err;
}

#define NAND_CMD_RESET_SYN	0xfc
static void nand_reset(struct mtd_info *mtd)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct cast_nand_host *host = nand_chip->priv;
	u32 reg;

	if (cast_readl(host, CAST_CONTROL) & CAST_CTRL_SYNC_MODE)
		reg = (NAND_CMD_RESET_SYN << SHIFT_CMD_0) | CAST_CMD_SEQ_0;
	else
		reg = (NAND_CMD_RESET << SHIFT_CMD_0) | CAST_CMD_SEQ_0;
	cast_writel(reg, host, CAST_COMMAND);
	cast_wait_ready(mtd, 0);
}

static void nand_read_id(struct mtd_info *mtd, int column)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct cast_nand_host *host = nand_chip->priv;
	u32 reg;

	wait_until_target_rdy(host, msecs_to_jiffies(200));

	/* Reset fifo */
	cast_writel(CAST_FIFO_INIT_FLASH, host, CAST_FIFO_INIT);

	/* readid has only two probobal addresses */
	if (column != 0x20)
		column = 0x00;

	/* Transfer customized size of data */
	reg = cast_readl(host, CAST_CONTROL);
	reg |= CAST_CTRL_CUSTOM_SIZE_EN;
	/* ECC module can be turned on only when CUSTOM_SIZE_EN = 0 */
	reg &= ~CAST_CTRL_ECC_EN;
	cast_writel(reg, host, CAST_CONTROL);

	/* 5 byte sould be enough for id or onfi signature */
	cast_writel(5, host, CAST_DATA_SIZE);

	/* 1 cycle address: 0x00 */
	cast_writel(column, host, CAST_ADDR0_0);
	cast_writel(0x00, host, CAST_ADDR0_1);

	/* Read ID command */
	reg = (NAND_CMD_READID << SHIFT_CMD_0) |
	       CAST_CMD_ADDR_SEL0	       |
	       CAST_CMD_INPUT_SEL_SIU 	       |
	       CAST_CMD_SEQ_1;
	cast_writel(reg, host, CAST_COMMAND);

	cast_wait_ready(mtd, 0);

	/* clean internal buffer */
	host->data_flag = 0;
	host->byte_idx = 0;
}

static void nand_read_status(struct mtd_info *mtd)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct cast_nand_host *host = nand_chip->priv;
	u32 reg;

	/* wait for controller */
	wait_for_flag(host, CAST_STATUS, CAST_STAT_CTRL_STAT, 0,
		      msecs_to_jiffies(200));

	/* READ STATUS command */
	reg = (NAND_CMD_STATUS << SHIFT_CMD_0) | CAST_CMD_SEQ_4;
	cast_writel(reg, host, CAST_COMMAND);

	/* Wait for read_status register ready */
	wait_for_flag(host, CAST_STATUS,
		      CAST_STAT_MEMX_ST(host->chip_nr), 1,
		      msecs_to_jiffies(200));
}

/* @column is the offset within the oob area */
static int cast_read_oob(struct mtd_info *mtd, int column, int page)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct cast_nand_host *host = nand_chip->priv;
	u32 reg;
	u32 block, page_shift, page_addr;
	int i;

	wait_until_target_rdy(host, msecs_to_jiffies(200));

	/* Reset fifo */
	cast_writel(CAST_FIFO_INIT_FLASH, host, CAST_FIFO_INIT);

	reg = cast_readl(host, CAST_CONTROL);
	reg &= ~CAST_CTRL_ECC_EN;
	reg &= ~CAST_CTRL_SPARE_EN;
	reg |= CAST_CTRL_CUSTOM_SIZE_EN;
	cast_writel(reg, host, CAST_CONTROL);

	cast_writel(mtd->oobsize, host, CAST_DATA_SIZE);

	block = page / host->ppb;
	page -= block * host->ppb;
	page_shift = fls(host->ppb - 1);
	page_addr = (block << page_shift) | page;

	cast_writel((mtd->writesize & 0xFFFF) | (page_addr << 16),
			host, CAST_ADDR0_0);
	cast_writel(page_addr >> 16, host, CAST_ADDR0_1);

	reg = (NAND_CMD_READSTART << SHIFT_CMD_1) |
	      (NAND_CMD_READ0 << SHIFT_CMD_0)	  |
	      CAST_CMD_ADDR_SEL0		  |
	      CAST_CMD_INPUT_SEL_SIU		  |
	      CAST_CMD_SEQ_10;

	cast_writel(reg, host, CAST_COMMAND);

	/* wait for controller */
	wait_for_flag(host, CAST_STATUS, CAST_STAT_CTRL_STAT, 0,
		      msecs_to_jiffies(200));

	/*
	 * Spare size read can only be started from
	 * the begining of the spare area.
	 * Remove surplus data from the fifo.
	 */
	for (i = 0; i < column; i += 4) {
		wait_for_flag(host, CAST_FIFO_STATE, CAST_FIFO_STATE_STAT, 1,
			      msecs_to_jiffies(200));
		*(uint32_t *)(host->data_buf) = cast_readl(host,
							   CAST_FIFO_DATA);
	}
	if (i == column) {
		host->data_flag = 0;
		host->byte_idx = 0;
	} else {
		host->data_flag = 1;
		host->byte_idx = column % 4;
	}
	return 0;
}

static int cast_onfi_param(struct mtd_info *mtd, int column, int size)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct cast_nand_host *host = nand_chip->priv;
	u32 reg;

	wait_until_target_rdy(host, msecs_to_jiffies(200));

	/* Reset fifo */
	cast_writel(CAST_FIFO_INIT_FLASH, host, CAST_FIFO_INIT);

	/* Transfer customized size of data */
	reg = cast_readl(host, CAST_CONTROL);
	reg |= CAST_CTRL_CUSTOM_SIZE_EN;
	reg &= ~CAST_CTRL_ECC_EN;
	cast_writel(reg, host, CAST_CONTROL);

	cast_writel(0, host, CAST_DATA_SIZE);

	/* 1 cycle address: 0x00 */
	cast_writel(0x00, host, CAST_ADDR0_0);
	cast_writel(0x00, host, CAST_ADDR0_1);

	/* read parameter page command */
	reg = (NAND_CMD_PARAM << SHIFT_CMD_0) |
	      CAST_CMD_ADDR_SEL0	      |
	      CAST_CMD_INPUT_SEL_SIU	      |
	      CAST_CMD_SEQ_2;
	cast_writel(reg, host, CAST_COMMAND);
	cast_wait_ready(mtd, 0);

	cast_writel(size, host, CAST_DATA_SIZE);
	cast_writel(column, host, CAST_ADDR0_0);

	reg = (0x05 << SHIFT_CMD_0) |
	      (0xe0 << SHIFT_CMD_1) |
	      CAST_CMD_ADDR_SEL0    |
	      CAST_CMD_SEQ_6;
	cast_writel(reg, host, CAST_COMMAND);
	cast_wait_ready(mtd, 0);

	/* clean internal buffer */
	host->data_flag = 0;
	host->byte_idx = 0;

	return 0;
}

static void cast_cmdfunc(struct mtd_info *mtd, unsigned command,
		int column, int page_addr)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct cast_nand_host *host = nand_chip->priv;
	switch (command) {
	case NAND_CMD_RESET:
		nand_reset(mtd);
		break;
	case NAND_CMD_READID:
		nand_read_id(mtd, column);
		break;
	case NAND_CMD_STATUS:
		nand_read_status(mtd);
		host->cmd_precd = NAND_CMD_STATUS;
		break;
	case NAND_CMD_READOOB:
		command = NAND_CMD_READ0;
		cast_read_oob(mtd, column, page_addr);
		break;
	case NAND_CMD_PARAM:
		cast_onfi_param(mtd, 0,
				sizeof(struct nand_onfi_params)*3);
		break;
	case NAND_CMD_READ0:
		// XXX
		break;
	default:
		dev_err(mtd->dev.parent, "unsupported cmd %d\n", command);
		dump_stack();
		break;
	}
}

/* Access to ready/busy line */
static int cast_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct cast_nand_host *host = nand_chip->priv;
	int stat = (cast_readl(host, CAST_STATUS)
		    & CAST_STAT_MEMX_ST(host->chip_nr)) == 0 ? 0 : 1;
	return stat;
}

static void cast_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct cast_nand_host *host = nand_chip->priv;
	u32 reg;

	if (chip == -1) {
		/* Just ignore if deselecting chip */
		return;
	} else if ((chip >= 8) || (chip < 0)) {
		dev_err(mtd->dev.parent, "Chip %d exceeds maximum"
					 " supported flash\n", chip);
		return;
	} else {
		host->chip_nr = chip;
		reg = cast_readl(host, CAST_MEM_CTRL);
		reg &= ~CAST_MEM_CTRL_CE_MASK;
		reg |= chip;
		cast_writel(reg, host, CAST_MEM_CTRL);
	}
}

static u8 cast_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct cast_nand_host *host = nand_chip->priv;
	u8 one_byte;

	/*
	 * Byte returned from read status cmd is stored in READ_STATUS reg
	 * not in the FIFO_DATA reg.
	 */
	if (host->cmd_precd == NAND_CMD_STATUS) {
		*(u32 *)(host->data_buf) = cast_readl(host, CAST_READ_STATUS);
		host->cmd_precd = -1;
		return host->data_buf[0];
	}

	if (host->data_flag) {
		one_byte = host->data_buf[host->byte_idx];
		host->byte_idx++;
		if (host->byte_idx == 4) {
			host->byte_idx = 0;
			host->data_flag = 0;
		}
	} else {
		wait_for_flag(host, CAST_FIFO_STATE, CAST_FIFO_STATE_STAT,
				1, (HZ * 200) / 1000);
		*(u32 *)(host->data_buf) = cast_readl(host, CAST_FIFO_DATA);
		one_byte = host->data_buf[0];
		host->data_flag = 1;
		host->byte_idx = 1;
	}
	return one_byte;
}

static void cast_read_buf(struct mtd_info *mtd, u8 *buf, int len)
{
	while (len--) {
		*buf++ = cast_read_byte(mtd);
	}
}

/**
 * Read @size bytes directly from NAND controller
 *
 * @param host		cast nand host
 * @param buf           data buffer
 * @param size          bytes to be read
 *
 * @return              0 - success, -1 - timeout
 */
static int nand_read_buf(struct cast_nand_host *host,
					uint8_t *buf, unsigned int size)
{
	int err = 0;
	unsigned int i;
	uint32_t *data;
	struct mtd_info *mtd = &(host->mtd);

	WARN_ON((unsigned long)buf & 3);
	WARN_ON(size & 3);
	/* transfer 32-bit words */
	data = (uint32_t *)((unsigned long)buf & ~3UL);
	size >>= 2;

	for (i = 0; i < size; i++) {
		err = wait_for_flag(host, CAST_FIFO_STATE,
				    CAST_FIFO_STATE_STAT, 1,
				    msecs_to_jiffies(200));
		if (err) {
			dev_dbg(mtd->dev.parent,
				"%s: transfer timeout\n", __func__);
			break;
		}
		*data++ = cast_readl(host, CAST_FIFO_DATA);
	}
	return err;
}

/**
 * Write @size bytes to NAND controller
 *
 * @param host		cast nand host
 * @param buf           data buffer
 * @param size          bytes to write
 *
 * @return              0 - success, -1 - timeout
 */
static int nand_write_buf(struct cast_nand_host *host,
			  const u8 *buf, unsigned int size)
{
        int err = 0;
        unsigned int i;
        u32 *data;
	struct mtd_info *mtd = &(host->mtd);

        /* transfer 32-bit words */
        data = (u32 *)((unsigned long)buf & ~3UL);
        size >>= 2;

        err = wait_for_flag(host, CAST_FIFO_STATE,
			    CAST_FIFO_STATE_STAT, 1,
			    msecs_to_jiffies(200));
        if (!err) {
                for (i = 0; i < size; i++)
			cast_writel(*data++, host, CAST_FIFO_DATA);
        } else {
		dev_dbg(mtd->dev.parent, "%s: transfer timeout\n", __func__);
        }
        return err;
}

/*
 * Setting CAST controller timing registers to
 * desired speed mode and data interface
 */
static void cast_init_timings(struct mtd_info *mtd, int mode, int syn, u16 tccs)
{
	u32 period;
	u32 pll;
	int interface;
	struct nand_chip *nand_chip = mtd->priv;
	struct cast_nand_host *host = nand_chip->priv;
	struct cast_plat_data const* pdata;
	int trhw;
	int tref;

	int i;
	int temp;

	union {
		struct cast_timings timings;
		u16 values[sizeof(struct cast_timings)/sizeof(u16)];
	} ucycles;

	struct cast_timings * const cycles = &ucycles.timings;

	/* get device cast-nand.0 platform data */
	pdata = dev_get_platdata(mtd->dev.parent);

	/* Interruption cleaning up */
	cast_writel(0, host, CAST_INT_STATUS);
	cast_writel(0, host, CAST_INT_MASK);

	interface = syn ? T_SYN : T_ASYN;

	if (nand_chip->onfi_version && (nand_chip->onfi_params.t_ccs)
	    && !tccs) {
		tccs = nand_chip->onfi_params.t_ccs;
	} else if (!tccs) {
		/* no onfi or not yet detected */
		tccs = 500;
	}

	dev_dbg(mtd->dev.parent, "chip_rev: %d\n", pdata->chip_rev);
	if (syn)
		pll = cast_modes[T_SYN][mode].pll;
	else if (pdata->chip_rev == 0) {
		/*
		 * This is for correcting mpw1 limit:
		 * Trhw is fixed at 6 clock cycles instead of
		 * being reconfigurable by register TIME_SEQ_0.
		 */
		trhw = cast_modes[interface][mode].trhw;
		pll = (1000000000/DIV_ROUND_UP(trhw, 6)) << 2;
	} else {
		/*
		 * The tccs/tadl/trhw/twhr fields of cast controller
		 * contains the biggest value which set the lower limit
		 * for clock period.
		 */
		tref = max(max(tccs, cast_modes[interface][mode].trhw),
			   max(cast_modes[interface][mode].twhr,
			       cast_modes[interface][mode].tadl));
		pll = ((1000000000/tref)<<5) << 2;

		if (mode == 5 && pll > 400000000)
			pll = 400000000;
		else if (mode == 4 && pll > 360000000)
			pll = 360000000;
		else if (pll > 400000000)
			pll = 400000000;
	}

	clk_disable(host->clk_phy);
	clk_set_rate(host->clk_phy, pll);
	clk_enable(host->clk_phy);

	temp = pll;
	pll = clk_get_rate(host->clk_phy);
	dev_info(mtd->dev.parent, "pll_nand: %dHz (wanted=%dHz mode=%d)\n",
			pll, temp, mode);
	if (pll < 1000) {
		dev_err(mtd->dev.parent, "pll_nand is too low\n");
		return;
	}

	period = (4000000000UL / (pll / 1000));
	dev_dbg(mtd->dev.parent, "period %d\n", period);

	/* convert timings from ns to cycles */
	memcpy(cycles, &cast_modes[interface][mode], sizeof(*cycles));
	cycles->tccs = tccs;
	for (i = 0; i < ARRAY_SIZE(ucycles.values); i++)
		ucycles.values[i] = DIV_ROUND_UP(ucycles.values[i]*1000, period);

	if (cycles->tccs > 32)
		cycles->tccs = 32;

	temp = cycles->twc - cycles->twh - cycles->twp;
	if (temp > 0)
		cycles->twp += temp;

	/* we increase here trp because it should be > tREA + board_REA */
	temp = cycles->trc - cycles->treh - cycles->trp;
	if (temp > 0)
		cycles->trp += temp;

	/* real cycle is (reg_value + 1) */
	for (i = 0; i < ARRAY_SIZE(ucycles.values); i++)
		if (ucycles.values[i] >= 1)
			ucycles.values[i] -= 1;

	WARN_ON(cycles->treh < cycles->twh);
	WARN_ON(cycles->trp < cycles->twp);
	/*
	 * TIMINGS_ASYN
	 * TRWH[7:4], TRWP[3:0]
	 */
	cast_tim_asyn_rd = ((cycles->treh & 0xf) << 4)| (cycles->trp & 0xf);
	cast_tim_asyn_wr = ((cycles->twh & 0xf) << 4)| (cycles->twp & 0xf);

	cast_writel(cast_tim_asyn_rd, host, CAST_TIMINGS_ASYN);
	dev_info(mtd->dev.parent, "CAST_TIMINGS_ASYN rd=0x%x wr=0x%x\n",
				cast_tim_asyn_rd, cast_tim_asyn_wr);

	/*
	 * TIMINGS_SYN
	 * TCAD[3:0]
	 */
	cast_writel(cycles->tcad & 0xf, host, CAST_TIMINGS_SYN);

	/*
	 * TIME_SEQ_0
	 * TWHR[28:24], TRHW[20:16], TADL[12:8], TCCS[4:0]
	 */
	cast_writel(((cycles->twhr & 0x1f) << 24) |
		    ((cycles->trhw & 0x1f) << 16) |
		    ((cycles->tadl & 0x1f) << 8)  |
		    (cycles->tccs & 0x1f), host, CAST_TIME_SEQ_0);
	dev_dbg(mtd->dev.parent, "CAST_TIME_SEQ_0 0x%x\n", ((cycles->twhr & 0x1f) << 24) |
			((cycles->trhw & 0x1f) << 16) |
			((cycles->tadl & 0x1f) << 8)  |
			(cycles->tccs & 0x1f));

	/*
	 * TIME_SEQ_1
	 * TRR[12:9], TWB[4:0]
	 */
	cast_writel(((cycles->trr & 0xf) << 9) |
		    (cycles->twb & 0x1f), host, CAST_TIME_SEQ_1);
	dev_dbg(mtd->dev.parent, "CAST_TIME_SEQ_1 0x%x\n", ((cycles->trr & 0xf) << 9) |
			(cycles->twb & 0x1f));

}

/* structure of equivalent onfi speed for non-onfi nand flash */
struct nonfi_speed_t {
	u16 id; /* maf_id and dev_id */
	u16 onfi_speed;
	u32 tccs;
} nonfi_speed[] = {
	/* TC58NVG3S0FBAID (FC7100) */
	{
		.id = 0x98 | (0xd3 << 8),
		.onfi_speed = 4,
		.tccs = 60,
	},
	/* TC58NVG3S0FBAID (FC7100) */
	{
		.id = 0x98 | (0xa3 << 8),
		.onfi_speed = 4,
		.tccs = 60,
	},
	/* TC58NVG3S0FBAID (FC7100) 4Gb */
	{
		.id = 0x98 | (0xac << 8),
		.onfi_speed = 4,
		.tccs = 60,
	},
	/* TC58BYG0S3HBAI6 (MYKONOS3) */
	{
		.id = 0x98 | (0xa1 << 8),
		.onfi_speed = 4,
		.tccs = 60,
	},
};

static void onfi_compatible(struct mtd_info *mtd)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct cast_nand_host *host = nand_chip->priv;
	u32 reg;
	int i;
	int err = 0;
	const int buf_size = 20;
	u16 tccs;
	u8 buf[buf_size];
	u16 nand_id = host->id[0] | (host->id[1] << 8);

	host->par_page.syn_mode = -1;
	host->par_page.asyn_mode = 0;

	if (nand_chip->onfi_version) {
		for (i = 0; i < 6; i++)
			if (nand_chip->onfi_params.async_timing_mode
			    & (1 << i))
				host->par_page.asyn_mode = i;

		for (i = 0; i < 6; i++)
			if (nand_chip->onfi_params.src_sync_timing_mode
			    & (1 << i))
				host->par_page.syn_mode = i;
	} else { /* use equivalent onfi speed mode for non-onfi nand */
		tccs = 0;
		for (i = 0; i < ARRAY_SIZE(nonfi_speed); i++) {
			if (nonfi_speed[i].id == nand_id) {
				host->par_page.asyn_mode
					= nonfi_speed[i].onfi_speed;
				tccs = nonfi_speed[i].tccs;
				break;
			}
		}
		cast_init_timings(mtd, host->par_page.asyn_mode, 0, tccs);

		goto finish;
	}

	/* switch to supported timing mode */

	/* Reset fifo */
	cast_writel(CAST_FIFO_INIT_FLASH, host, CAST_FIFO_INIT);

	cast_writel(4, host, CAST_DATA_SIZE);
	cast_writel(0x01, host, CAST_ADDR0_0);
	cast_writel(0, host, CAST_ADDR0_1);
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	reg = (0xef << SHIFT_CMD_0) |
	      CAST_CMD_ADDR_SEL0    |
	      CAST_CMD_INPUT_SEL_SIU|
	      CAST_CMD_SEQ_3;

	if (synmode && (host->par_page.syn_mode != -1)) {
		cast_writel(reg, host, CAST_COMMAND);
		buf[0] = (0x1 << 4) | (host->par_page.syn_mode & 0xf);
		nand_write_buf(host, buf, 4);
		err = cast_wait_ready(mtd, 0);
		if (err)
			goto finish;
		cast_init_timings(mtd, host->par_page.syn_mode, 1, 0);
		reg = cast_readl(host, CAST_CONTROL);
		reg |= CAST_CTRL_SYNC_MODE | CAST_CTRL_IO_WIDTH_16;
		cast_writel(reg, host, CAST_CONTROL);
	} else {
		cast_writel(reg, host, CAST_COMMAND);
		buf[0] = host->par_page.asyn_mode & 0xf;

		nand_write_buf(host, buf, 4);
		err = cast_wait_ready(mtd, 0);
		if (err)
			goto finish;
		cast_init_timings(mtd, host->par_page.asyn_mode, 0, 0);
	}

	dev_dbg(mtd->dev.parent, "onfi: %d\n", nand_chip->onfi_version);
	dev_dbg(mtd->dev.parent, "syn_mode: %d\n", host->par_page.syn_mode);
	dev_dbg(mtd->dev.parent, "asyn_mode: %d\n", host->par_page.asyn_mode);
	dev_dbg(mtd->dev.parent, "tccs: %d\n", nand_chip->onfi_params.t_ccs);
finish:
	return;
}

/* CAST controller register configuration called from probe function */
static void cast_config_controller(struct mtd_info *mtd)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct cast_nand_host *host = nand_chip->priv;
	u32 reg;

	switch (mtd->writesize) {
	case 256:
		reg = CAST_CTRL_PAGE_256;
		break;
	case 512:
		reg = CAST_CTRL_PAGE_512;
		break;
	case 1024:
		reg = CAST_CTRL_PAGE_1K;
		break;
	case 2048:
		reg = CAST_CTRL_PAGE_2K;
		break;
	case 4096:
		reg = CAST_CTRL_PAGE_4K;
		break;
	case 8192:
		reg = CAST_CTRL_PAGE_8K;
		break;
	case 16384:
		reg = CAST_CTRL_PAGE_16K;
		break;
	default:
		reg = CAST_CTRL_PAGE_0;
	}

	switch(mtd->erasesize >> (ffs(mtd->writesize) - 1)) {
	case 32:
		reg |= CAST_CTRL_BLOCK_32;
		break;
	case 64:
		reg |= CAST_CTRL_BLOCK_64;
		break;
	case 128:
		reg |= CAST_CTRL_BLOCK_128;
		break;
	case 256:
		reg |= CAST_CTRL_BLOCK_256;
		break;
	default:
		dev_err(mtd->dev.parent, "Non supported erase size for "
					 "%d\n", mtd->erasesize);

		BUG();
	}

	/* nb of cycles is read cycle */
	reg = reg | CAST_CTRL_ADDR_CYCLE0(5) | CAST_CTRL_ADDR_CYCLE1(5);

	if (nand_chip->options & NAND_BUSWIDTH_16) {
	        reg |= CAST_CTRL_IO_WIDTH_16;
	} else {
	        reg |= CAST_CTRL_IO_WIDTH_8;
	}

	/* Disable all interrupts */
	cast_writel(0, host, CAST_INT_MASK);
	/* Clear interrupt status */
	cast_writel(0, host, CAST_INT_STATUS);

	/* Global interrupt enable */
	reg |= CAST_CTRL_INT_EN;
	cast_writel(reg, host, CAST_CONTROL);

	switch (nand_chip->ecc.size) {
	case 256:
		reg = CAST_ECC_CTRL_BLOCK_256;
		break;
	case 512:
		reg = CAST_ECC_CTRL_BLOCK_512;
		break;
	case 1024:
		reg = CAST_ECC_CTRL_BLOCK_1K;
		break;
	default:
		dev_err(mtd->dev.parent, "Non supported ecc size for "
					 "%d\n", nand_chip->ecc.size);
		BUG();
	}
	reg |= CAST_ECC_CTRL_CAP(host->ecc_bits);

	reg |= (host->ecc_threshold - 1) << CAST_ECC_CTRL_ERR_THRLD;

	cast_writel(reg, host, CAST_ECC_CTRL);

	cast_writel(mtd->writesize + nand_chip->ecc.layout->eccpos[0],
		    host, CAST_ECC_OFFSET);
}

static int cast_init_clock(struct cast_nand_host *host,
			   struct platform_device *pdev)
{
	int ret;
	struct clk *cast_clk;
	struct clk *phy_clk;

	cast_clk = clk_get(&pdev->dev, "ctrl");
	if (IS_ERR(cast_clk)) {
		ret = PTR_ERR(cast_clk);
		goto out;
	}
	ret = clk_prepare_enable(cast_clk);
	if (unlikely(ret)) {
		goto put_ctrl_clk;
	}
	host->clk_ctrl = cast_clk;

	phy_clk = clk_get(&pdev->dev, "phy");
	if (IS_ERR(phy_clk)) {
		ret = PTR_ERR(phy_clk);
		goto out_phy_clk;
	}
	ret = clk_prepare_enable(phy_clk);
	if (unlikely(ret)) {
		goto put_phy_clk;
	}
	host->clk_phy = phy_clk;

	return 0;

put_phy_clk:
	clk_put(phy_clk);
out_phy_clk:
	clk_disable_unprepare(cast_clk);
put_ctrl_clk:
	clk_put(cast_clk);
out:
	return ret;
}

/**
 * Prepare CAST controller for DMA transfer (before command sent)
 *
 * @host		cast_nand_host structure
 * @size		nb of bytes to be transfered
 * @direction		operation direction (write/read)
 *
 * @return		0 - success; -1 - upon failure
 */
static int cast_dma_prepare_transfer(struct cast_nand_host *host,
				     int size, int direction,
				     dma_addr_t dmaaddr)
{
	u32	reg;

	cast_writel(dmaaddr, host, CAST_DMA_ADDR);

	/* DMA SFR mode */
	cast_writel(0, host, CAST_DMA_ADDR_OFFSET);
	cast_writel(size, host, CAST_DMA_CNT);
	reg = CAST_DMA_CTRL_DMA_MODE_SFR;

	if (direction == NF_TRANSFER_READ) {
		reg |= CAST_DMA_CTRL_DMA_DIR_READ;
	} else {
		reg |= CAST_DMA_CTRL_DMA_DIR_WRITE;
	}

	reg |= DMA_BURST_UNSPEC_LENGTH;

	reg |= CAST_DMA_CTRL_DMA_START;
	cast_writel(reg, host, CAST_DMA_CTRL);

	return 0;
}

/**
 * Wait for CAST controller finish DMA transfer (after command sent)
 *
 * @host		cast_nand_host structure
 * @irq			choose irq or polling
 *
 * @return		0 - success; -1 - upon failure
 */
static int cast_wait_dma_finish_transfer(struct cast_nand_host *host, int irq)
{
	int err = 0;
	int res;
	u32 reg;
	unsigned long timeo = msecs_to_jiffies(2000);
	struct mtd_info *mtd = &(host->mtd);

	if (irq) {
		res = wait_for_completion_timeout(&host->complete, timeo);
		if (!res) {
			dev_dbg(mtd->dev.parent,
					"%s: Timeout in irq\n", __func__);
			err = -1;
		}
		/* Disable all interrupts */
		cast_writel(0, host, CAST_INT_MASK);
	} else {
		err = wait_for_flag(host, CAST_DMA_CTRL,
				    CAST_DMA_CTRL_DMA_READY, 1, timeo);
	}

	if (err == -1) {
		dev_err(mtd->dev.parent,
			"%s: Wait for DMA transfer timeout\n", __func__);
		goto fail;
	}

	reg = cast_readl(host, CAST_DMA_CTRL);

	if (reg & CAST_DMA_CTRL_DMA_ERR_FLAG) {
		dev_dbg(mtd->dev.parent, "%s: DMA transfer\n", __func__);
		err = -1;
		goto fail;
	}

	return 0;
fail:
	return err;
}

static int cast_read_oob_std(struct mtd_info *mtd, struct nand_chip *chip,
				int page, int sndcmd)
{
	struct cast_nand_host *host = chip->priv;
	u8 *buf = chip->oob_poi;
	u32 reg;
	u32 block, page_shift, page_addr;

	wait_until_target_rdy(host, msecs_to_jiffies(200));

	if (sndcmd) {
		int err;

		/* Reset fifo */
		cast_writel(CAST_FIFO_INIT_FLASH, host, CAST_FIFO_INIT);

		reg = cast_readl(host, CAST_CONTROL);
		reg &= ~CAST_CTRL_ECC_EN;
		reg &= ~CAST_CTRL_SPARE_EN;
		reg |= CAST_CTRL_CUSTOM_SIZE_EN;
		cast_writel(reg, host, CAST_CONTROL);

		cast_writel(mtd->oobsize, host, CAST_DATA_SIZE);

		block = page / host->ppb;
		page -= block * host->ppb;
		page_shift = fls(host->ppb - 1);
		page_addr = (block << page_shift) | page;

		cast_writel((mtd->writesize & 0xFFFF) | (page_addr << 16),
				host, CAST_ADDR0_0);
		cast_writel(page_addr >> 16, host, CAST_ADDR0_1);

		reg = (NAND_CMD_READSTART << SHIFT_CMD_1) |
		      (NAND_CMD_READ0 << SHIFT_CMD_0)	  |
		      CAST_CMD_ADDR_SEL0 		  |
		      CAST_CMD_SEQ_10;

		if (usedma) {
			cast_dma_prepare_transfer(host, mtd->oobsize,
						  NF_TRANSFER_READ,
						  host->dmaaddr);
			reg |= CAST_CMD_INPUT_SEL_DMA;

			/* enable interrupt for dma transfer */
			cast_writel(CAST_INT_MASK_DMA_READY_EN, host, CAST_INT_MASK);
		} else {
			reg |= CAST_CMD_INPUT_SEL_SIU;
		}

		cast_writel(reg, host, CAST_COMMAND);

		if (usedma)
			err = cast_wait_dma_finish_transfer(host, 1);
		else
			err = wait_for_flag(host, CAST_STATUS, CAST_STAT_CTRL_STAT, 0,
						msecs_to_jiffies(200));

		if (err)
			return err;

		sndcmd = 0;
	}

	if (usedma)
		memcpy(buf, host->dmabuf, mtd->oobsize);
	else
		nand_read_buf(host, buf, mtd->oobsize);

	return sndcmd;
}

static int cast_read_page_data(struct mtd_info *mtd, int page,
			       u8 *buf, int raw,
			       int spare_user_size, u8 *spare_buf)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct cast_nand_host *host = nand_chip->priv;
	u32 reg;
	u32 block, page_shift, page_addr;
	int err = 0;
	int transfer_size;

	wait_until_target_rdy(host, msecs_to_jiffies(200));

	reg = cast_readl(host, CAST_CONTROL);
	reg &= ~CAST_CTRL_CUSTOM_SIZE_EN;
	if (spare_user_size && spare_buf)
		reg |= CAST_CTRL_SPARE_EN;
	else
		reg &= ~CAST_CTRL_SPARE_EN;
	if (raw) {
		reg &= ~CAST_CTRL_ECC_EN;
	} else {
		reg |= CAST_CTRL_ECC_EN;
	}
	cast_writel(reg, host, CAST_CONTROL);

	cast_writel(spare_user_size, host, CAST_SPARE_SIZE);

	transfer_size = mtd->writesize
		+ ((spare_user_size && spare_buf) ? spare_user_size : 0);

	block = page / host->ppb;
	page -= block * host->ppb;
	page_shift = fls(host->ppb - 1);
	page_addr = (block << page_shift) | page;
	cast_writel((page_addr << 16), host, CAST_ADDR0_0);
	cast_writel(page_addr >> 16, host, CAST_ADDR0_1);

	reg = (NAND_CMD_READSTART << SHIFT_CMD_1) |
	      (NAND_CMD_READ0 << SHIFT_CMD_0)	  |
	      CAST_CMD_ADDR_SEL0		  |
	      CAST_CMD_SEQ_10;

	if (usedma) {
		cast_dma_prepare_transfer(host,
					  transfer_size,
					  NF_TRANSFER_READ,
					  host->dmaaddr);

		reg |= CAST_CMD_INPUT_SEL_DMA;
		/* enable interrupt for dma transfer */
		cast_writel(CAST_INT_MASK_DMA_READY_EN, host, CAST_INT_MASK);
	} else {
		reg |= CAST_CMD_INPUT_SEL_SIU;
	}

	/* Reset fifo */
	cast_writel(CAST_FIFO_INIT_FLASH, host, CAST_FIFO_INIT);

	cast_writel(reg, host, CAST_COMMAND);

	if (usedma) {
		err = cast_wait_dma_finish_transfer(host, 1);
		if (!err) {
			memcpy(buf, (u8 *)host->dmabuf, mtd->writesize);
			if (spare_user_size && spare_buf)
				memcpy(spare_buf,
				       (u8 *)host->dmabuf + mtd->writesize,
				       spare_user_size);
		}
	} else {
		/* wait for controller */
		err = wait_for_flag(host, CAST_STATUS, CAST_STAT_CTRL_STAT, 0,
				    msecs_to_jiffies(200));
		if (!err) {
			err = nand_read_buf(host, buf, mtd->writesize);
		}

		if (!err && spare_user_size && spare_buf)
			err = nand_read_buf(host, spare_buf, spare_user_size);
	}

	return err;
}

int page_is_programmed(u8 *marker_buf, int marker_len)
{
	int hw, i;

	hw = 0;
	for (i = 0; i < marker_len; i++)
		hw += hweight8(marker_buf[i]);

	/*
	 * threshold: 1/2 of (marker_len*8) bits
	 * hamming weight < threshold ==> programmed page
	 */
	return ((hw < (marker_len << 2)) ? 1 : 0);
}

/*
 * Test whether a non-marked page (containing ff with possible bit-flips)
 * is correctable.
 *
 * @param buf		nand page buffer
 * @param length	page size
 * @param ecc_size	ecc block size (512/1024)
 * @param ecc_bits	ecc correction capability
 *
 * @return		n (>=0): number of ecc bits corrected
 *			-1: uncorrectable error
 */
int correctable_empty_page(u8 *buf, int length, int ecc_size, int ecc_bits)
{
	int ecc_block;
	int i;
	int bit_flip;
	u32 *ptr;
	int ret = 0;

	for (ecc_block = 0; ecc_block < length; ecc_block += ecc_size) {
		bit_flip = 0;
		ptr = (u32 *)(buf + ecc_block);
		for (i = 0; i < (ecc_size >> 2); i++) {
			bit_flip += hweight32(~ptr[i]);
			ptr[i] = 0xffffffff; /* correct the page */
		}

		ret += bit_flip;

		if (bit_flip > ecc_bits) {
			ret = -1;
			break;
		}
	}

	return ret;
}

static int cast_read_page_ecc(struct mtd_info *mtd, struct nand_chip *chip,
			      u8 *buf, int page)
{
	int ret;
	struct cast_nand_host *host = chip->priv;
	u32 reg_ecc_ctrl;
	u8 status;
	u8 spare_buf[NAND_MAX_OOBSIZE];

	ret = cast_read_page_data(mtd, page, buf, 0,
			    host->ppm + BBM_BYTES, spare_buf);
	if (ret)
		return ret;

	reg_ecc_ctrl = cast_readl(host, CAST_ECC_CTRL);

	chip->ecc.read_oob(mtd, chip, page, 1);

	cast_cmdfunc(mtd, NAND_CMD_STATUS, 0, 0);
	status = cast_read_byte(mtd);

	if (!(status & NAND_STATUS_READY))
		return -EIO;

	if (host->benand) {
		if (status & NAND_STATUS_FAIL) {
			mtd->ecc_stats.failed++;
			dev_info(mtd->dev.parent, "benand fail 0x%x page %d\n", status, page);
			return 0;
		}
		else if (status & 8) {
			/* nand with on die ecc :
			   toshiba benand
			 */
			mtd->ecc_stats.corrected += host->ecc_threshold;
			dev_info(mtd->dev.parent, "benand corrected 0x%x page %d\n", status, page);
		}

		/* we should have no ecc correction (corrected by flash) */
		if (reg_ecc_ctrl & CAST_ECC_CTRL_ERR_UNCORRECT) {
			if (page_is_programmed(spare_buf + BBM_BYTES,
						host->ppm))
				dev_err(mtd->dev.parent, "cast ecc uncorr on benand status %x %x\n",
						status, reg_ecc_ctrl);
		}
		else if (reg_ecc_ctrl & CAST_ECC_CTRL_ERR_CORRECT) {
			dev_err(mtd->dev.parent, "cast ecc correction on benand status %x %x\n",
					status, reg_ecc_ctrl);
		}
		return 0;
	}
	if (reg_ecc_ctrl & CAST_ECC_CTRL_ERR_UNCORRECT) {
		if (page_is_programmed(spare_buf + BBM_BYTES,
				       host->ppm)) {
			mtd->ecc_stats.failed++;
		} else {
			//XXX this look wrong as we don't check bitflip
			//in spare aera.
			//Also this should take lot's of time to count
			//bitflip on the whole page
			ret = correctable_empty_page(buf, mtd->writesize,
						     chip->ecc.size,
						     host->ecc_bits);

			/* report as corrected page if within our capacity */
			if (ret >= 0)
				mtd->ecc_stats.corrected += ret;
			else
				mtd->ecc_stats.failed++;
		}
	} else if ((reg_ecc_ctrl & CAST_ECC_CTRL_ERR_CORRECT) &&
		   (reg_ecc_ctrl & CAST_ECC_CTRL_ERR_OVER)) {
			mtd->ecc_stats.corrected += host->ecc_threshold;
	}

	return 0;
}

static int cast_read_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
			      u8 *buf, int page)
{
	struct cast_nand_host *host = chip->priv;
	u32 reg;
	u32 block, page_shift, page_addr;
	int err = 0;

	wait_until_target_rdy(host, msecs_to_jiffies(200));

	/* Reset fifo */
	cast_writel(CAST_FIFO_INIT_FLASH, host, CAST_FIFO_INIT);

	reg = cast_readl(host, CAST_CONTROL);
	reg &= ~CAST_CTRL_ECC_EN;
	reg &= ~CAST_CTRL_SPARE_EN;
	reg |= CAST_CTRL_CUSTOM_SIZE_EN;
	cast_writel(reg, host, CAST_CONTROL);

	cast_writel(mtd->writesize + mtd->oobsize, host, CAST_DATA_SIZE);

	block = page / host->ppb;
	page -= block * host->ppb;
	page_shift = fls(host->ppb - 1);
	page_addr = (block << page_shift) | page;
	cast_writel((page_addr << 16), host, CAST_ADDR0_0);
	cast_writel(page_addr >> 16, host, CAST_ADDR0_1);

	reg = (NAND_CMD_READSTART << SHIFT_CMD_1) |
	      (NAND_CMD_READ0 << SHIFT_CMD_0)	  |
	      CAST_CMD_ADDR_SEL0		  |
	      CAST_CMD_SEQ_10;

	if (usedma) {
		cast_dma_prepare_transfer(host,
					  mtd->writesize + mtd->oobsize,
					  NF_TRANSFER_READ,
					  host->dmaaddr);
		reg |= CAST_CMD_INPUT_SEL_DMA;

		/* enable interrupt for dma transfer */
		cast_writel(CAST_INT_MASK_DMA_READY_EN, host, CAST_INT_MASK);
	} else {
		reg |= CAST_CMD_INPUT_SEL_SIU;
	}

	cast_writel(reg, host, CAST_COMMAND);

	if (usedma) {
		err = cast_wait_dma_finish_transfer(host, 1);
		if (!err) {
			memcpy(buf, (u8 *)host->dmabuf, mtd->writesize);
			memcpy(chip->oob_poi,
			       (u8 *)host->dmabuf + mtd->writesize,
			       mtd->oobsize);
		}
	} else {
		/* wait for controller */
		err = wait_for_flag(host, CAST_STATUS, CAST_STAT_CTRL_STAT, 0,
				    msecs_to_jiffies(200));

		if (!err) {
			nand_read_buf(host, buf, mtd->writesize);
			nand_read_buf(host, chip->oob_poi, mtd->oobsize);
		}
	}
	return err;
}

static int nand_write_oob_std(struct mtd_info *mtd, struct nand_chip *chip,
				int page)
{
	u8 status;
	const u8 *buf = chip->oob_poi;
	struct cast_nand_host *host = chip->priv;
	u32 reg;
	u32 block, page_shift, page_addr;
	int err = 0;

	wait_until_target_rdy(host, msecs_to_jiffies(200));

	reg = cast_readl(host, CAST_CONTROL);
	reg |= CAST_CTRL_CUSTOM_SIZE_EN;
	reg &= ~CAST_CTRL_ECC_EN;
	reg &= ~CAST_CTRL_SPARE_EN;
	cast_writel(reg, host, CAST_CONTROL);

	cast_writel(0, host, CAST_SPARE_SIZE);
	cast_writel(mtd->oobsize, host, CAST_DATA_SIZE);

	block = page / host->ppb;
	page -= block * host->ppb;
	page_shift = fls(host->ppb - 1);
	page_addr = (block << page_shift) | page;
	cast_writel((page_addr << 16) | (mtd->writesize & 0xFFFF),
		    host, CAST_ADDR0_0);
	cast_writel(page_addr >> 16, host, CAST_ADDR0_1);

	reg = (NAND_CMD_PAGEPROG << SHIFT_CMD_1) |
	      (NAND_CMD_SEQIN << SHIFT_CMD_0)	 |
	      CAST_CMD_ADDR_SEL0 		 |
	      CAST_CMD_SEQ_12;

	if (usedma) {
		memcpy(host->dmabuf, buf, mtd->oobsize);
		cast_dma_prepare_transfer(host, mtd->oobsize,
					  NF_TRANSFER_WRITE,
					  host->dmaaddr);
		reg |= CAST_CMD_INPUT_SEL_DMA;

	} else {
		reg |= CAST_CMD_INPUT_SEL_SIU;
	}

	cast_writel(cast_tim_asyn_wr, host, CAST_TIMINGS_ASYN);
	/* enable interrupt */
	cast_writel(CAST_INT_MASK_MEMx_RDY_INT_EN(host->chip_nr),
		    host, CAST_INT_MASK);
	cast_writel(reg, host, CAST_COMMAND);

	if (!usedma) {
		err = nand_write_buf(host, buf, mtd->oobsize);
	}
	if (err)
		goto fail;

	err = cast_wait_ready(mtd, 1);
	if (err)
		goto fail;

	/* Check dma error if using dma */
	if (usedma) {
		reg = cast_readl(host, CAST_DMA_CTRL);
		if (reg & CAST_DMA_CTRL_DMA_ERR_FLAG) {
			dev_dbg(mtd->dev.parent, "%s: DMA Transfer error\n"
						 , __func__);
			err = -1;
			goto fail;
		}
	}

	cast_writel(cast_tim_asyn_rd, host, CAST_TIMINGS_ASYN);

	cast_cmdfunc(mtd, NAND_CMD_STATUS, 0, 0);
	status = cast_read_byte(mtd);

	WARN_ON(!(status & NAND_STATUS_READY));
	return status & NAND_STATUS_FAIL ? -EIO : 0;
fail:
	cast_writel(cast_tim_asyn_rd, host, CAST_TIMINGS_ASYN);
	return err;
}

static void dummy_ecc_write_page(struct mtd_info *mtd, struct nand_chip *chip,
				 const uint8_t *buf)
{
	dev_err(mtd->dev.parent, "Should not enter dummy function\n");
	BUG();
}

static int cast_write_page(struct mtd_info *mtd, struct nand_chip *chip,
			   const u8 *buf, int page, int cached, int raw)
{
	u8 status;
	struct cast_nand_host *host = chip->priv;
	u32 reg;
	u32 block, page_shift, page_addr;
	int err = 0;
	u8 spare_buf[NAND_MAX_OOBSIZE] = { 0xff, 0xff, 0 };

	wait_until_target_rdy(host, msecs_to_jiffies(200));

	/* Reset fifo */
	cast_writel(CAST_FIFO_INIT_FLASH, host, CAST_FIFO_INIT);

	reg = cast_readl(host, CAST_CONTROL);
	reg &= ~CAST_CTRL_CUSTOM_SIZE_EN;
	reg |= CAST_CTRL_SPARE_EN;
	if (raw) {
		reg &= ~CAST_CTRL_ECC_EN;
	} else {
		reg |= CAST_CTRL_ECC_EN;
	}
	cast_writel(reg, host, CAST_CONTROL);

	/* spare size */
	cast_writel(mtd->oobsize, host, CAST_SPARE_SIZE);

	block = page / host->ppb;
	page -= block * host->ppb;
	page_shift = fls(host->ppb - 1);
	page_addr = (block << page_shift) | page;
	cast_writel((page_addr << 16), host, CAST_ADDR0_0);
	cast_writel(page_addr >> 16, host, CAST_ADDR0_1);

	reg = (NAND_CMD_PAGEPROG << SHIFT_CMD_1) |
	      (NAND_CMD_SEQIN << SHIFT_CMD_0)	 |
	      CAST_CMD_ADDR_SEL0 		 |
	      CAST_CMD_SEQ_12;

	if (usedma) {
		memcpy(host->dmabuf, buf, mtd->writesize);
		memcpy(host->dmabuf + mtd->writesize, chip->oob_poi,
		       mtd->oobsize);
		/* spare area marker */
		memcpy(host->dmabuf + mtd->writesize, spare_buf,
		       host->ppm + BBM_BYTES);
		cast_dma_prepare_transfer(
			host,
			mtd->writesize + mtd->oobsize,
			NF_TRANSFER_WRITE,
			host->dmaaddr);
		reg |= CAST_CMD_INPUT_SEL_DMA;
	} else {
		reg |= CAST_CMD_INPUT_SEL_SIU;
	}

	cast_writel(cast_tim_asyn_wr, host, CAST_TIMINGS_ASYN);
	/* enable interrupt */
	cast_writel(CAST_INT_MASK_MEMx_RDY_INT_EN(host->chip_nr),
		    host, CAST_INT_MASK);
	cast_writel(reg, host, CAST_COMMAND);

	if (!usedma) {
		err = nand_write_buf(host, buf, mtd->writesize);
		/* transfer spare area marker in fifo mode */
		err |= nand_write_buf(host, spare_buf, host->ppm + BBM_BYTES);
		err |= nand_write_buf(host, chip->oob_poi + host->ppm + BBM_BYTES, mtd->oobsize - (host->ppm + BBM_BYTES));
	}
	if (err)
		goto fail;

	err = cast_wait_ready(mtd, 1);
	if (err)
		goto fail;

	/* Check dma error if using dma */
	if (usedma) {
		reg = cast_readl(host, CAST_DMA_CTRL);
		if (reg & CAST_DMA_CTRL_DMA_ERR_FLAG) {
			dev_dbg(mtd->dev.parent, "%s: DMA Transfer error\n"
						 , __func__);
			err = -1;
			goto fail;
		}
	}

	cast_writel(cast_tim_asyn_rd, host, CAST_TIMINGS_ASYN);

	cast_cmdfunc(mtd, NAND_CMD_STATUS, 0, 0);
	status = cast_read_byte(mtd);

	WARN_ON(!(status & NAND_STATUS_READY));

	// XXX
	if ((status & NAND_STATUS_FAIL) && (chip->errstat)) {
		status = chip->errstat(mtd, chip, FL_WRITING, status, page);
	}

	if (status & NAND_STATUS_FAIL)
		return -EIO;

	return 0;
fail:
	cast_writel(cast_tim_asyn_rd, host, CAST_TIMINGS_ASYN);
	return err;
}

static void nand_erase_cmd(struct mtd_info *mtd, int page)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct cast_nand_host *host = nand_chip->priv;
	u32 block, page_shift, page_addr;
	u32 reg;

	block = page / host->ppb;
	page -= block * host->ppb;
	page_shift = fls(host->ppb - 1);
	page_addr = (block << page_shift) | page;

	/*
	 * clear eventual set of custom size bit
	 * of cast ctrl register by other functions
	 */
	reg = cast_readl(host, CAST_CONTROL);
	reg &= ~CAST_CTRL_CUSTOM_SIZE_EN;
	cast_writel(reg, host, CAST_CONTROL);

	cast_writel(page_addr << 16, host, CAST_ADDR0_0);
	cast_writel(page_addr >> 16, host, CAST_ADDR0_1);

	reg = (NAND_CMD_ERASE2 << SHIFT_CMD_1) |
	      (NAND_CMD_ERASE1 << SHIFT_CMD_0) |
	      CAST_CMD_ADDR_SEL0 	       |
	      CAST_CMD_SEQ_14;

	/* enable interrupt */
	cast_writel(CAST_INT_MASK_MEMx_RDY_INT_EN(host->chip_nr),
		    host, CAST_INT_MASK);

	cast_writel(reg, host, CAST_COMMAND);

	cast_wait_ready(mtd, 1);
}

/* print current feature of 'address' */
static void print_feature(struct mtd_info *mtd, u32 address)
{
        struct nand_chip *nand_chip = mtd->priv;
        struct cast_nand_host *host = nand_chip->priv;
        u32 reg;
        u8 buf[4];
        int err = 0;

        /* Reset fifo */
        cast_writel(CAST_FIFO_INIT_FLASH, host, CAST_FIFO_INIT);

        reg = cast_readl(host, CAST_CONTROL);
        reg &= ~CAST_CTRL_ECC_EN;
        reg &= ~CAST_CTRL_SPARE_EN;
        reg |= CAST_CTRL_CUSTOM_SIZE_EN;
        cast_writel(reg, host, CAST_CONTROL);

        cast_writel(4, host, CAST_DATA_SIZE);
        cast_writel(address, host, CAST_ADDR0_0);
        cast_writel(0, host, CAST_ADDR0_1);
        reg = (0xee << SHIFT_CMD_0) |
              CAST_CMD_ADDR_SEL0    |
              CAST_CMD_INPUT_SEL_SIU|
              CAST_CMD_SEQ_2;

	/* enable interrupt */
	cast_writel(CAST_INT_MASK_MEMx_RDY_INT_EN(host->chip_nr),
		    host, CAST_INT_MASK);
        cast_writel(reg, host, CAST_COMMAND);

        err = cast_wait_ready(mtd, 1);
        if (err) {
		dev_err(mtd->dev.parent, "Get feature error\n");
                goto finish;
        }

        nand_read_buf(host, buf, 4);
	dev_info(mtd->dev.parent, "Features: %x %x %x %x\n",
				  buf[0], buf[1], buf[2], buf[3]);

finish:
        return;
}

/*
 * Probe for the NAND device.
 */
static int __devinit cast_nand_probe(struct platform_device *pdev)
{
	struct cast_nand_host *host;
	struct mtd_info *mtd;
	struct nand_chip *nand_chip;
	struct resource *res;
	/*struct cast_plat_data const* const pdata = dev_get_platdata(&pdev->dev);*/
	int err = 0;
	int i;
	int ecc_steps, ecc_size, ecc_bytes;
	int spare_user_size;

	/* Allocate memory for the device structure (and zero it) */
	host = kzalloc(sizeof(struct cast_nand_host), GFP_KERNEL);
	if (!host) {
		dev_err(&pdev->dev, "failed to allocate device\n");
		return -ENOMEM;
	}
	host->mode_curr = T_ASYN;

	if (usedma) {
		host->dmabuf = dma_alloc_coherent(&pdev->dev,
					NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE,
					&host->dmaaddr, GFP_KERNEL);
		if (!host->dmabuf) {
			dev_err(&pdev->dev, "failed to allocate dma buffer\n");
			err = -ENOMEM;
			goto no_dma;
		}

		BUG_ON(! (IS_ALIGNED(host->dmaaddr, sizeof(u32))));
		dev_info(&pdev->dev, "DMA mode enabled\n");
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_info(&pdev->dev, "invalid I/O resource\n");
		err = -ENXIO;
		goto no_res;
	}

	host->area = request_mem_region(res->start,
					resource_size(res),
					dev_name(&pdev->dev));

	if (host->area == NULL) {
		err = -EBUSY;
		goto no_res;
	}

	host->regs = ioremap(res->start, resource_size(res));
	if (host->regs == NULL) {
		err = -EIO;
		goto no_ioremap;
	}

	/* Init cast_nand_host fields */
	host->cmd_precd = -1;
	host->data_flag = 0;
	host->byte_idx = 0;

	mtd = &host->mtd;
	nand_chip = &host->nand_chip;

	mtd->owner = THIS_MODULE;
	mtd->dev.parent = &pdev->dev;
	mtd->name = "nand0";
	mtd->priv = &host->nand_chip;
	nand_chip->priv = host;

	nand_chip->IO_ADDR_R = nand_chip->IO_ADDR_W = 0;
	nand_chip->cmdfunc = cast_cmdfunc;
	nand_chip->dev_ready = cast_dev_ready;
	nand_chip->select_chip = cast_select_chip;
	nand_chip->read_byte = cast_read_byte;
	nand_chip->read_buf = cast_read_buf;

	if (cast_init_clock(host, pdev)) {
		err = -EIO;
		goto no_clk_init;
	}

	/* initialize cast with speed mode 0 and aynchronous */
	cast_init_timings(mtd, 0, 0, 0);

	/* No write protect */
	cast_writel(0xFF00, host, CAST_MEM_CTRL);

	/* Reset fifo */
	cast_writel(CAST_FIFO_INIT_FLASH, host, CAST_FIFO_INIT);

	init_completion(&host->complete);

	host->pctl = pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(host->pctl)) {
		err = PTR_ERR(host->pctl);
		goto no_pin;
	}

	err = request_irq(P7_NAND_IRQ, &cast_irq, 0,
			  dev_name(&pdev->dev), host);
	if (err) {
		err = -ENOENT;
		goto no_irq;
	}

	/*
	 * get the device id information for timing matching
	 */
	nand_read_id(mtd, 0x00);
	nand_read_buf(host, (uint8_t *)host->id, 8);
	cast_writel(CAST_FIFO_INIT_FLASH, host, CAST_FIFO_INIT);

	dev_dbg(&pdev->dev, "nand id %x %x %x %x %x %x %x %x\n",
			host->id[0], host->id[1], host->id[2], host->id[3],
			host->id[4], host->id[5], host->id[6], host->id[7]);

	if (host->id[0] == 0x98 && (host->id[4] & 0x80)) {
		host->benand = 1;
		dev_info(&pdev->dev, "nand is benand\n");
	}

	/* first part of the scan to get chip information */
	if (nand_scan_ident(mtd, FLASH_COUNT, NULL)) {
		err = -ENXIO;
		goto no_flash;
	}

	/* Test bus width support conflict */
	if ((!SUPPORT_BUSWIDTH_16)&&(nand_chip->options&NAND_BUSWIDTH_16)) {
		err = -ENXIO;
		goto no_flash;
	}

	host->ppb = mtd->erasesize / mtd->writesize;

	/* ECC Module related settings */
	nand_chip->ecc.mode = NAND_ECC_HW;

	/*
	 * Compared to 1024, 512 deals better with
	 * unbalanced bit-flip distribution
	 */
	nand_chip->ecc.size = 512;

	/*
	 * cast: user data size in spare area should be 8 bytes aligned,
	 * therefore at least 8 bytes need to be reserved (2 bbm + 6 ppm).
	 */
	host->ppm = 6;
	spare_user_size = BBM_BYTES + host->ppm;

	/*
	 * For each bit of correction capability, we need
	 * 14 bits (BCH ecc) + 1 bit (programming marker)
	 */
	ecc_size = mtd->oobsize - spare_user_size;
	ecc_steps = mtd->writesize/nand_chip->ecc.size;
	host->ecc_bits = ((ecc_size / ecc_steps) * 8) / (14 + 1);
	host->ecc_bits &= ~1U;
	nand_chip->ecc.bytes = (14 * host->ecc_bits + 7) >> 3;
	ecc_bytes = nand_chip->ecc.bytes * ecc_steps;

	/* use as many marker bits as host->ecc_bits for each ecc block */
	host->ppm = (host->ecc_bits * ecc_steps + 7) >> 3;
	/* programmed page marker should be at least 6 bytes */
	if (host->ppm < 6)
		host->ppm = 6;

	spare_user_size = host->ppm + BBM_BYTES;
	/* adjust number of marker bytes after rounding */
	if ((spare_user_size + ecc_bytes) > mtd->oobsize)
		spare_user_size = mtd->oobsize - ecc_bytes;

	/* spare_user_size should be 8-byte aligned */
	spare_user_size &= ~7U;
	host->ppm = spare_user_size - BBM_BYTES;

	/* contruct from template layout structure */
	cast_ecc_oob_layout.eccbytes = ecc_bytes;

	for (i = 0; i < cast_ecc_oob_layout.eccbytes; i++)
		cast_ecc_oob_layout.eccpos[i] = spare_user_size + i;

	cast_ecc_oob_layout.oobfree[0].offset =
		spare_user_size + cast_ecc_oob_layout.eccbytes;
	cast_ecc_oob_layout.oobfree[0].length =
		mtd->oobsize - cast_ecc_oob_layout.oobfree[0].offset;
	nand_chip->ecc.layout = &cast_ecc_oob_layout;

	nand_chip->ecc.total = (nand_chip->ecc.layout)->eccbytes;

	/*
	 * Threshold is set half of the correction capability.
	 * The cast controller often corrects more bits than required by
	 * the manufacture. It is safer to report the error earlier
	 * to avoid a sudden degradation.
	 */
	host->ecc_threshold = (host->ecc_bits >> 1) & 0x3f;
	if (!host->ecc_threshold)
		host->ecc_threshold = 1;

	cast_config_controller(mtd);

	nand_chip->ecc.read_page = cast_read_page_ecc;
	nand_chip->ecc.read_page_raw = cast_read_page_raw;
	nand_chip->write_page = cast_write_page;
	nand_chip->ecc.write_page = dummy_ecc_write_page;
	nand_chip->ecc.write_oob = nand_write_oob_std;
	nand_chip->erase_cmd = nand_erase_cmd;
	nand_chip->ecc.read_oob = cast_read_oob_std;

	/*
	 * CAST constroller supports ecc only when the whole page is read.
	 */
	nand_chip->options |= NAND_NO_SUBPAGE_WRITE;

	err = nand_scan_tail(mtd);
	if (err)
		goto no_flash;

	/* Do not use default partitions anymore,
	 * only take into account mtdparts= from cmdline */
	err = mtd_device_parse_register(mtd,
	                                NULL,
	                                NULL,
	                                /*pdata ? pdata->parts : */NULL,
	                                /*pdata ? pdata->parts_nr : */0);
	if (err)
		goto no_part;
	onfi_compatible(mtd);
	if (nand_chip->onfi_version)
		print_feature(mtd, 0x01);

	dev_dbg(&pdev->dev, "reg_ctrl: 0x%x\n", cast_readl(host, CAST_CONTROL));
	dev_dbg(&pdev->dev, "ecc_ctrl: 0x%x\n", cast_readl(host, CAST_ECC_CTRL));
	dev_dbg(&pdev->dev, "ecc_off: 0x%x\n", cast_readl(host, CAST_ECC_OFFSET));

	platform_set_drvdata(pdev, host);

	/* Register sysfs files */
	device_create_file(&pdev->dev, &dev_attr_rom_ctrl);
	device_create_file(&pdev->dev, &dev_attr_rom_ecc_ctrl);
	device_create_file(&pdev->dev, &dev_attr_rom_ecc_offset);
	device_create_file(&pdev->dev, &dev_attr_rom_gen_seq_ctrl);
	device_create_file(&pdev->dev, &dev_attr_rom_time_seq_0);
	device_create_file(&pdev->dev, &dev_attr_rom_time_seq_1);
	device_create_file(&pdev->dev, &dev_attr_rom_time_asyn);

	return 0;

no_part:
	nand_release(mtd);
no_flash:
	free_irq(P7_NAND_IRQ, host);
no_irq:
	pinctrl_put(host->pctl);
no_pin:
	clk_disable_unprepare(host->clk_ctrl);
	clk_put(host->clk_ctrl);
	clk_disable_unprepare(host->clk_phy);
	clk_put(host->clk_phy);
no_clk_init:
	iounmap(host->regs);
no_ioremap:
	release_mem_region(host->area->start, resource_size(host->area));
no_res:
	if (host->dmabuf) {
		dma_free_coherent(&pdev->dev,
				NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE,
				host->dmabuf, host->dmaaddr);
	}
no_dma:
	kfree(host);
	return err;
}

/*
 * Remove a NAND device.
 */
static int __devexit cast_nand_remove(struct platform_device *pdev)
{
	struct cast_nand_host *host = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, NULL);

	/* Remove sysfs files */
	device_remove_file(&pdev->dev, &dev_attr_rom_ctrl);
	device_remove_file(&pdev->dev, &dev_attr_rom_ecc_ctrl);
	device_remove_file(&pdev->dev, &dev_attr_rom_ecc_offset);
	device_remove_file(&pdev->dev, &dev_attr_rom_gen_seq_ctrl);
	device_remove_file(&pdev->dev, &dev_attr_rom_time_seq_0);
	device_remove_file(&pdev->dev, &dev_attr_rom_time_seq_1);
	device_remove_file(&pdev->dev, &dev_attr_rom_time_asyn);

	nand_release(&host->mtd);
	free_irq(platform_get_irq(pdev, 0), host);
	pinctrl_put(host->pctl);
	iounmap(host->regs);
	clk_disable_unprepare(host->clk_ctrl);
	clk_put(host->clk_ctrl);
	clk_disable_unprepare(host->clk_phy);
	clk_put(host->clk_phy);
	release_mem_region(host->area->start, resource_size(host->area));
	if (host->dmabuf) {
		dma_free_coherent(&pdev->dev,
				NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE,
				host->dmabuf, host->dmaaddr);
	}
	kfree(host);
	return 0;
}

static int cast_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct cast_nand_host *host = platform_get_drvdata(pdev);
	dev_info(&pdev->dev, "suspend\n");
	clk_disable_unprepare(host->clk_ctrl);
	clk_put(host->clk_ctrl);
	clk_disable_unprepare(host->clk_phy);
	clk_put(host->clk_phy);

	return 0;
}

static int cast_resume(struct platform_device *pdev)
{
	struct cast_nand_host *host = platform_get_drvdata(pdev);
	struct mtd_info *mtd = &host->mtd;
	int nand_id;

	dev_info(&pdev->dev, "resume\n");

	cast_init_clock(host, pdev);

	/* initialize cast with speed mode 0 and aynchronous */
	cast_init_timings(mtd, 0, 0, 0);

	/* No write protect */
	cast_writel(0xFF00, host, CAST_MEM_CTRL);

	/* Reset fifo */
	cast_writel(CAST_FIFO_INIT_FLASH, host, CAST_FIFO_INIT);

	nand_reset(mtd);
	nand_read_id(mtd, 0x00);
	nand_read_buf(host, (uint8_t *)&nand_id, 4);
	/* Reset fifo */
	cast_writel(CAST_FIFO_INIT_FLASH, host, CAST_FIFO_INIT);

	dev_dbg(&pdev->dev, "nand_id: 0x%x\n", nand_id);

	cast_config_controller(mtd);

	onfi_compatible(mtd);

	dev_dbg(&pdev->dev, "reg_ctrl: 0x%x\n", cast_readl(host, CAST_CONTROL));
	dev_dbg(&pdev->dev, "ecc_ctrl: 0x%x\n", cast_readl(host, CAST_ECC_CTRL));
	dev_dbg(&pdev->dev, "ecc_off: 0x%x\n", cast_readl(host, CAST_ECC_OFFSET));

	return 0;
}

static struct platform_driver cast_nand_driver = {
	.driver     = {
		.name   = CAST_DRV_NAME,
		.owner  = THIS_MODULE,
	},
	.probe      = cast_nand_probe,
	.suspend	= cast_suspend,
	.resume		= cast_resume,
	.remove     = __devexit_p(cast_nand_remove)
};

module_platform_driver(cast_nand_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Guangye Tian guangye.tian@parrot.com");
MODULE_DESCRIPTION("NAND driver for CAST Controller");
MODULE_ALIAS("platform:cast_nand");
