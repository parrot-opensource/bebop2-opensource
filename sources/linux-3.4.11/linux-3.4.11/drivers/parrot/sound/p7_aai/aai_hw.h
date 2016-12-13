
#ifndef _AAI_HW_H_
#define _AAI_HW_H_

#include "reg_aai_gbc.h"
#include "reg_aai.h"
#include "aai_regs.h"

/* Nominal Rate */
/* XXX per board? */
#define AAI_NOMINAL_RATE    48000

int aai_hw_init(struct card_data_t *aai);
int aai_hw_init_card(struct card_data_t *aai, int dev_id);
int aai_hw_remove(struct card_data_t *aai);
int aai_hw_pcm_init(struct card_data_t *aai, struct aai_device_t *chan);
int aai_hw_start_channel(struct aai_device_t *chan);
int aai_hw_stop_channel(struct aai_device_t *chan);
int aai_hw_dma_prepare(struct aai_device_t *chan);
void aai_hw_dma_free(struct snd_pcm_substream *substream);
int aai_hw_dma_alloc(struct snd_pcm_substream *substream, size_t size);

irqreturn_t aai_irq_p7(int irq, void *dev);

int aai_pcms_probe(struct card_data_t *aai);
int aai_pcms_init(struct card_data_t *aai);

int aai_ioctl_hwdep_new(struct card_data_t *aai);

/* AAI pcm ops declaration */
extern struct snd_pcm_ops aai_pcm_ops;

/**
 * @brief Read register wrapper.
 *        Read register value and test offset value
 *
 * @param aai private driver data
 * @param offset register offset
 *
 * @return register value
 */
static unsigned int aai_readreg(struct card_data_t *aai, uint32_t offset)
{
	uint32_t val = 0;

	/* TODO: check input value */

	if (!aai_reg_canread(offset)) {
		dev_warn(aai->dev, "%s (0x%x) is write only register\n",
			 aai_reg_name(offset), aai_reg_addr(offset));
		return 0;
	}

	val = readl(aai->iobase+offset);

	return val;
}

/**
 * @brief Write register wrapper.
 *        Write register value and print debug info
 *
 * @param aai    private driver data
 * @param val    value to write
 * @param offset register offset
 *
 * @return none
 */
static void aai_writereg(struct card_data_t *aai,
			 uint32_t val,
			 uint32_t offset,
			 int disp)
{
	uint32_t reg;

	if (!aai_reg_canwrite(offset)) {
		dev_warn(aai->dev, "%s (0x%x) is read only register\n",
			 aai_reg_name(offset), aai_reg_addr(offset));
		return;
	}

	if (disp) {
		dev_warn(aai->dev, "writing %s @ 0x%x: 0x%x\n",
			 aai_reg_name(offset), aai_reg_addr(offset), val);
	}

	writel(val, aai->iobase + offset);

	/* read back */
	if (aai_reg_canreadback(offset)) {
		reg = aai_readreg(aai, offset);
		if (reg != val)
			dev_warn(aai->dev, "failed writing 0x%x to reg %s "
				 "(0x%x), contains: 0x%x\n",
				 val, aai_reg_name(offset),
				 aai_reg_addr(offset), reg);
	}
}

/**
 * @brief Set several consecutive bits.
 *
 * @param aai private driver data
 * @param offset register offset
 * @param mask bits mask
 * @param shift first bit shift
 * @param val value
 *
 * @return none
 */
static inline void aai_set_nbit(struct card_data_t *aai,
				unsigned int       offset,
				unsigned int       mask,
				unsigned int       shift,
				unsigned int       val)
{
	uint32_t reg;

	reg = aai_readreg(aai, offset);
	reg &= ~mask;               /* set all concerned bits to 0 */
	reg |= (val << shift);      /* set needed bits to 1        */
	aai_writereg(aai, reg, offset, 0);
}

static inline int aai_get_nbit(struct card_data_t *aai,
			       unsigned int       offset,
			       unsigned int       mask,
			       unsigned int       shift)
{
	uint32_t reg = aai_readreg(aai, offset);
	reg &= mask;
	return reg >> shift; /* set needed bits to 1 */
}

/**
 * @brief Set bit value.
 *
 * @param aai private driver data
 * @param offset register offset
 * @param mask bits mask
 * @param val value
 *
 * @return none
 */
static inline void aai_setbit(struct card_data_t *aai,
			      unsigned int       offset,
			      unsigned int       mask,
			      unsigned int       val)
{
	uint32_t reg;

	reg = aai_readreg(aai, offset);
	/*
	 * if the bit has to be modified
	 */
	if (!!(reg & mask) != (!!val)) {
		reg &= ~(mask);           /* reset concerned bit to 0 */
		if (val)
			reg |= mask;          /* set it if needed */
		aai_writereg(aai, reg, offset, 0);
	}
}

/**
 * @brief Get bit value
 *
 * @param aai  AAI card instance
 * @param offset register offset
 * @param mask bits mask
 *
 * @return requested bit value
 */
static inline unsigned int aai_getbit(struct card_data_t *aai,
				      unsigned int       offset,
				      unsigned int       mask)
{
	uint32_t reg = aai_readreg(aai, offset);
	return !!(reg & mask);
}

static inline void aai_it_en(struct card_data_t *aai,
			     struct aai_device_t *device)
{
	uint32_t reg;

	/*
	 * Activate RX/TX interrupt in main interrupt register
	 */
	reg = aai_readreg(aai, AAI_MAIN_ITEN);
	if (device->direction & AAI_RX) {
		reg |= 0x1;
		aai_writereg(aai, reg, AAI_MAIN_ITEN, 0);
	} else if (device->direction & AAI_TX) {
		reg |= 0x100;
		aai_writereg(aai, reg, AAI_MAIN_ITEN, 0);
	}

	/*
	 * Activate specific fifo interrupt, in DMA mode only
	 */
	if (device->direction & AAI_RX) {
		if (device->group == -1) {
			reg = aai_readreg(aai, AAI_MAIN_FIFO_EN_RX);
			reg |= 1 << device->fifo.fifo_id;
			aai_writereg(aai, reg, AAI_MAIN_FIFO_EN_RX, 0);
		}
	} else if (device->direction & AAI_TX) {
		if (device->group == -1) {
			reg = aai_readreg(aai, AAI_MAIN_FIFO_EN_TX);
			reg |= 1 << device->fifo.fifo_id;
			aai_writereg(aai, reg, AAI_MAIN_FIFO_EN_TX, 0);
		}
	} else {
		dev_err(aai->dev, "wrong channel direction\n");
	}
}

static inline void aai_it_dis(struct card_data_t *aai,
			      struct aai_device_t *device)
{
	uint32_t reg;

	/*
	 * Deactivate specific fifo interrupt, in DMA mode only
	 */
	if (device->direction & AAI_RX) {
		reg = aai_readreg(aai, AAI_MAIN_FIFO_EN_RX);
		reg &= ~(1 << device->fifo.fifo_id);
		aai_writereg(aai, reg, AAI_MAIN_FIFO_EN_RX, 0);
	} else if (device->direction & AAI_TX) {
		reg = aai_readreg(aai, AAI_MAIN_FIFO_EN_TX);
		reg &= ~(1 << device->fifo.fifo_id);
		aai_writereg(aai, reg, AAI_MAIN_FIFO_EN_TX, 0);
	} else {
		dev_err(aai->dev, "wrong channel direction\n");
	}

	/*
	 * When all corresponding FIFO are disabled,
	 * deactivate RX/TX interrupt in main interrupt register.
	 */
	reg = aai_readreg(aai, AAI_MAIN_ITEN);
	if ((device->direction & AAI_RX) &&
	    !aai_readreg(aai, AAI_MAIN_FIFO_EN_RX)) {
		reg &= ~(0x7);
		aai_writereg(aai, reg, AAI_MAIN_ITEN, 0);
	} else if ((device->direction & AAI_TX) &&
		 !aai_readreg(aai, AAI_MAIN_FIFO_EN_TX)) {
		reg &= ~(0x700);
		aai_writereg(aai, reg, AAI_MAIN_ITEN, 0);
	}
}

static inline void aai_group_it_en(struct card_data_t *aai, int group)
{
	uint32_t reg;

	reg = aai_readreg(aai, AAI_MAIN_ITEN);
	reg |= 1 << (16 + group);
	aai_writereg(aai, reg, AAI_MAIN_ITEN, 0);
}

static inline void aai_group_it_dis(struct card_data_t *aai, int group)
{
	uint32_t reg;

	reg = aai_readreg(aai, AAI_MAIN_ITEN);
	reg &= ~(1 << (16+group));
	aai_writereg(aai, reg, AAI_MAIN_ITEN, 0);
}

#endif

