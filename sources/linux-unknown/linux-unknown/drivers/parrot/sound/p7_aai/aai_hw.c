
#include <linux/dma-mapping.h>
#include <asm/mach-types.h>

#include "aai.h"
#include "aai_hw.h"
#include "aai_group.h"

#include "aai_clk.h"

#include "aai_hw_chan.h"

static int aai_hw_set_conf(struct card_data_t *aai)
{
	int reg, err = 0;
	struct aai_conf_set *conf = aai->pdata->aai_conf;

	if (!conf) {
		err = -ENOENT;
		goto exit;
	}

	while (conf->addr != -1) {
		reg  = aai_readreg(aai, conf->addr);
		reg &= ~(conf->mask  << conf->shift);
		reg |= (conf->value & conf->mask) << conf->shift;
		aai_writereg(aai, reg, conf->addr, 0);
		conf++;
	}
exit:
	return err;
}

static int aai_hw_assign_pad(struct card_data_t *aai, enum aai_signal_t signal,
			     int padindex, int direction)
{
	int err = 0;
	if (aai->pdata->chiprev < P7_CHIPREV_R3) {
		aai_writereg(aai, padindex, AAI_GBC_PADIN(signal), 0);
		if (direction & PAD_OUT)
			aai_writereg(aai, signal, AAI_GBC_PADOUT(padindex), 0);
	} else {
		aai_writereg(aai, padindex, AAI_LBC_PADIN(signal), 0);
		if (direction & PAD_OUT)
			aai_writereg(aai, signal, AAI_LBC_PADOUT(padindex), 0);
	}

	return err;
}

static int aai_hw_init_pads(struct card_data_t *aai)
{
	struct aai_pad_t *pad = aai->pdata->pad;
	if (!pad)
		return 0;

	while (pad->index != -1) {
		aai_hw_assign_pad(aai, pad->signal, pad->index, pad->direction);
		pad++;
	}

	return 0;
}

static int aai_hw_init_cfg(struct card_data_t *aai)
{
	uint32_t cfg;
	int err = 0;
	int freq;
	int pcmdiv;

	/*
	 * Configure music channels
	 * - 32 bit clock cycles per frame
	 * - MSB *not* synchronous with frame edge
	 * - left data transmitted when sync = 0
	 * - MSB sent first
	 * - left justification
	 * - music data 24 bits wide (bit 11, otherwise: 16 bits wide)
	 * - synchonous to DAC
	 * - I2S mode
	 * - biphase & binary mode unused, spdif mode only
	 */
	cfg = 32 | (1 << 11);
	aai_writereg(aai, cfg, AAI_AAICFG_MUS(0), 0);
	aai_writereg(aai, cfg, AAI_AAICFG_MUS(1), 0);
	aai_writereg(aai, cfg, AAI_AAICFG_MUS(2), 0);
	aai_writereg(aai, cfg, AAI_AAICFG_MUS(3), 0);

	/*
	 * Configure TDM channels
	 * - 32 bit clock cycles per frame
	 * - left data transmitted when sync = 0
	 * - TDM mode disable
	 */
	cfg = 32;
	aai_writereg(aai, cfg, AAI_AAICFG_TDM_IN, 0);
	aai_writereg(aai, cfg, AAI_AAICFG_TDM_OUT, 0);

	/*
	 * ADC & DAC configuration
	 * - set cycles to 32
	 */
	cfg = 32;
	aai_writereg(aai, cfg, AAI_AAICFG_ADC, 0);
	aai_writereg(aai, cfg, AAI_AAICFG_DAC, 0);

	/*
	 * AUX configuration
	 * - 32 bit clock cycles per frame
	 * - MSB *not* synchonous with frame edge
	 * - left data transmitted when sync = 0
	 * - half rate *not* configured
	 */
	cfg = 32;
	aai_writereg(aai, cfg, AAI_AAICFG_AUX, 0);

	/*
	 * MIC muxing configuration
	 * - do not use loop-back mode by default
	 * - high res mode
	 */
	cfg = 0;
	aai_writereg(aai, cfg, AAI_AAICFG_MIC_MUX, 0);

	/*
	 * Voice muxing for DAC 0, 1, 2 & 3
	 * - use standard output channels without left/right inversion
	 *   (0x9: non-inverted ; 0x6: inverted)
	 */
	cfg = 0x9 | (0x9 << 8) | (0x9 << 16) | (0x9 << 24);
	aai_writereg(aai, cfg, AAI_AAICFG_VOI_MUX, 0);

	/*
	 * PCM configuration
	 * - no second PCM frame
	 * - 32 clock cycles in 16kHz mode
	 * - 32 clock cycles in 8kHz mode
	 */
	cfg = (32 << 8) | (32 << 16) | (1 << 25);
	aai_writereg(aai, cfg, AAI_AAICFG_PCM(0), 0);
	aai_writereg(aai, cfg, AAI_AAICFG_PCM(1), 0);

	/*
	 * PCM CLK configuration
	 * - output PCM clk 2Mhz
	 * - frame high is 1-1 =0
	 * - frame low  is 124-1 in 16kHz or 249-1 in 8kHz
	 */
	freq = clk_get_rate(aai->clk[i2s_clk]);
	pcmdiv = freq / 256000 - 2;
	if (aai->pdata->chiprev < P7_CHIPREV_R3) {
		aai_writereg(aai, pcmdiv, AAI_GBC_P1CLK, 0);
		aai_writereg(aai, 0x1e, AAI_GBC_P1FRAME, 0);
		aai_writereg(aai, pcmdiv, AAI_GBC_P2CLK, 0);
		aai_writereg(aai, 0x1e, AAI_GBC_P2FRAME, 0);
	} else {
		aai_writereg(aai, pcmdiv, AAI_LBC_P1CLK, 0);
		aai_writereg(aai, 0x1e, AAI_LBC_P1FRAME, 0);
		aai_writereg(aai, pcmdiv, AAI_LBC_P2CLK, 0);
		aai_writereg(aai, 0x1e, AAI_LBC_P2FRAME, 0);
	}

	/*
	 * SPDIF RX
	 * - place status on LSB part
	 * - accept all data
	 */
	cfg = 0;
	aai_writereg(aai, cfg, AAI_AAICFG_SPDIFRX, 0);

	/*
	 * SPDIF TX
	 * - parity taken from data[31]
	 * - start mode: wait two subframes, to ensure synchro
	 * - disable transfer from fifo
	 */
	cfg = 6 << 1;
	aai_writereg(aai, cfg, AAI_AAICFG_SPDIFTX, 0);

	/* XXX spdif clk div */
	cfg = 0xff;
	if (aai->pdata->chiprev < P7_CHIPREV_R3)
		aai_writereg(aai, cfg, AAI_GBC_SPDIFCLK, 0);
	else
		aai_writereg(aai, cfg, AAI_GBC_SPDIFCLK_R3, 0);

	/*
	 * Voice configuration
	 * - volume is 0dB
	 * - channel enable
	 * - no usage of down-sample channel
	 * - no down-sampling factor set
	 * - same configuration for left and right part of channel
	 */
	cfg = 0x10 | (0x10 << 8);
	aai_writereg(aai, cfg, AAI_AAICFG_VOICE(0), 0);
	aai_writereg(aai, cfg, AAI_AAICFG_VOICE(1), 0);
	aai_writereg(aai, cfg, AAI_AAICFG_VOICE(2), 0);
	aai_writereg(aai, cfg, AAI_AAICFG_VOICE(3), 0);

	/*
	 * Voice speed
	 * - 16kHz (and 8kHz for half speed mode)
	 * - 16kHz: 0x00810e35 ; 22.05kHz: 0x00b1dac7
	 * - see documentation to get the formula
	 */
	cfg = AAI_VOICE_SPEED_16_8;
	aai_writereg(aai, cfg, AAI_AAICFG_VOICE_SPEED, 0);

	/*
	 * Voice 8kHz
	 * - Speaker working @ 8kHz
	 * - PCM working @ 8 kHz
	 */
	cfg = 3;
	aai_writereg(aai, cfg, AAI_AAICFG_VOICE_8K, 0);

	/*
	 * Music filter5
	 * - high freq
	 */
	cfg = 0;
	aai_writereg(aai, cfg, AAI_AAICFG_MUS_FILTER5, 0);

	/*
	 * RAM2RAM SRC filter5
	 * - high freq for both filters
	 */
	cfg = 0;
	aai_writereg(aai, cfg, AAI_AAICFG_SRC_FILTER5, 0);

	/*
	 * RAM2RAM SRC mode
	 * - disable for now
	 * - work on stereo streams
	 * - independant RAM2RAM, no octo configuration
	 */
	cfg = 0;
	aai_writereg(aai, cfg, AAI_AAICFG_SRC_MODE, 0);

	/*
	 * RAM2RAM SRC speed
	 * - input = output for now
	 */
	cfg = 1 << 28;
	aai_writereg(aai, cfg, AAI_AAICFG_SRC_SPEED0, 0);
	aai_writereg(aai, cfg, AAI_AAICFG_SRC_SPEED1, 0);

	return err;
}

int aai_hw_pcm_init(struct card_data_t *aai, struct aai_device_t *chan)
{
	int err = 0;
	int i;
	struct aai_hw_channel_t *fifo = &chan->fifo;
	uint32_t cfg;
	struct aai_audio_chan_t *internal_chan;
	uint32_t reg_fifo16;
	unsigned long flags = 0;

	spin_lock_irqsave(&aai->hwlock, flags);

	if (aai->fifo_address + chan->fifo_size >= 4096) {
		dev_err(aai->dev, "not enough size in aai internal "
				  "memory for fifo allocation\n");
		err = -ENOMEM;
		goto exit;
	}

	/* Set fifo id and registers */
	if (chan->direction & AAI_TX) {
		fifo->fifo_id = aai->fifo_tx++;
		if (fifo->fifo_id > 10) {
			dev_err(aai->dev, "not enough hardware fifo TX "
					  "available for configuration\n");
			err = -EINVAL;
			goto exit;
		}
		fifo->fifo_conf = AAI_FIFOCFG_CFG_TX(fifo->fifo_id);
		fifo->dmaca = AAI_DMACFG_CA_TX(fifo->fifo_id);
		fifo->dmana = AAI_DMACFG_NA_TX(fifo->fifo_id);
		if (aai->pdata->chiprev > P7_CHIPREV_R1) {
			fifo->dmacv = AAI_DMACFG_CV_TX(fifo->fifo_id);
			fifo->dmanv = AAI_DMACFG_NV_TX(fifo->fifo_id);
		}
		reg_fifo16 = AAI_FIFOCFG_TX_16BIT;
	} else if (chan->direction & AAI_RX) {
		fifo->fifo_id = aai->fifo_rx++;
		if (fifo->fifo_id > 17) {
			dev_err(aai->dev, "not enough hardware fifo RX "
					  "available for configuration\n");
			err = -EINVAL;
			goto exit;
		}
		fifo->fifo_conf = AAI_FIFOCFG_CFG_RX(fifo->fifo_id);
		fifo->dmaca = AAI_DMACFG_CA_RX(fifo->fifo_id);
		fifo->dmana = AAI_DMACFG_NA_RX(fifo->fifo_id);
		if (aai->pdata->chiprev > P7_CHIPREV_R1) {
			fifo->dmacv = AAI_DMACFG_CV_RX(fifo->fifo_id);
			fifo->dmanv = AAI_DMACFG_NV_RX(fifo->fifo_id);
		}
		reg_fifo16 = AAI_FIFOCFG_RX_16BIT;
	} else {
		dev_err(aai->dev, "wrong channel direction\n");
		err = -EINVAL;
		goto exit;
	}

	/*
	 * Configure related fifo register
	 * calculate FIFO_DEPTH based on fifo_size
	 */
	if ((chan->fifo_size >> 2) > 256) {
		dev_err(aai->dev, "max FIFO depth is 256 words\n");
		err = -EINVAL;
		goto exit;
	}
	cfg = aai->fifo_address		     |
	      ((chan->fifo_size >> 2) << 16) |
	      ((fifo->chan_nb - 1) << 24);
	aai_writereg(aai, cfg, fifo->fifo_conf, 0);

	/* Configure 16bits or 32bits mode */
	if (aai->pdata->chiprev > P7_CHIPREV_R1 && fifo->mode == FIFO_MODE_16)
		dev_warn(aai->dev,
			 "%s: fifo mode 16 not supported in this revision\n",
			 __func__);

	cfg = aai_readreg(aai, reg_fifo16);
	if (fifo->mode == FIFO_MODE_16) {
		cfg |= (1 << fifo->fifo_id);
	} else if (fifo->mode == FIFO_MODE_32) {
		cfg &= ~(1 << fifo->fifo_id);
	} else {
		dev_err(aai->dev, "wrong fifo mode\n");
		err = -EINVAL;
		goto exit;
	}
	aai_writereg(aai, cfg, reg_fifo16, 0);

	/* increment fifo address for next internal memory allocation */
	aai->fifo_address += chan->fifo_size;

	/* Configure associated aai internal channels */
	for (i = 0; i < fifo->chan_nb; i++) {
		internal_chan = fifo->chan_list[i];
		if (internal_chan->used) {
			dev_err(aai->dev,
				"%s: audio channel \"%s\" already used\n",
				__func__, internal_chan->name);
			err = -EINVAL;
			goto exit;
		}
		cfg = fifo->fifo_id | (i << 8);
		aai_writereg(aai, cfg, internal_chan->fifo_sel_reg, 0);
		internal_chan->used = 1;
	}

	/* Set group to -1 (no group) */
	chan->group = -1;

exit:
	spin_unlock_irqrestore(&aai->hwlock, flags);
	return err;
}

int aai_hw_init(struct card_data_t *aai)
{
	uint32_t cfg;
	int err = 0;

	/* Enable AAI clocks */
	if (aai->pdata->chiprev < P7_CHIPREV_R3) {
		cfg  = aai_readreg(aai, AAI_GBC_CLKEN);
		cfg |= (1 << GBC_AAI0_MFRAME);
		cfg |= (1 << GBC_AAI0_HFRAME);
		aai_writereg(aai, cfg, AAI_GBC_CLKEN, 0);
	} else {
		cfg  = aai_readreg(aai, AAI_LBC_CLKEN);
		cfg |= (1 << GBC_AAI0_MFRAME);
		cfg |= (1 << GBC_AAI0_HFRAME);
		cfg |= (1 << GBC_AAI0_MCLK);
		aai_writereg(aai, cfg, AAI_LBC_CLKEN, 0);
	}

	aai_set_clk(aai, i2s_clk, AAI_NOMINAL_RATE);

	/* Reset all uninitialized AAI registers */
	aai_reset_regs(aai);

	/* Set default AAI CFG regs */
	err = aai_hw_init_cfg(aai);
	if (err)
		goto exit;

	/* Init pads */
	err = aai_hw_init_pads(aai);
	if (err)
		goto exit;

	err = aai_hw_set_conf(aai);
	if (err)
		goto exit;

exit:
	return err;
}

static struct aai_device_t *aai_find_pcm(const char *aai_device)
{
	int itr;
	struct aai_device_t *ret = NULL;
	for (itr = 0; itr < ARRAY_SIZE(aai_pcm_devices); itr++) {
		if (!strcmp(aai_device, aai_pcm_devices[itr].name)) {
			ret = &aai_pcm_devices[itr];
			break;
		}
	}
	return ret;
}

int aai_hw_init_card(struct card_data_t *aai, int dev_id)
{
	int err = 0, itr = 0;
	struct snd_card *card = aai->card;

	/* Init lock */
	spin_lock_init(&aai->hwlock);
	if (!aai->pdata->device_list)
		return -ENODEV;

	while (aai->pdata->device_list[++itr] != NULL)
		;
	aai->chans_nb = itr;

	aai->chans = kmalloc(itr * sizeof(struct aai_device_t), GFP_KERNEL);
	if (!aai->chans)
		return -ENOMEM;

	for (itr = 0; itr < aai->chans_nb; itr++) {
		const char *name = aai->pdata->device_list[itr];
		struct aai_device_t *ret = aai_find_pcm(name);
		if (!ret) {
			dev_err(aai->dev, "channel %s does not exist\n", name);
			kfree(aai->chans);
			return -ENODEV;
		}
		/* XXX need to do a copy? */
		aai->chans[itr] = *ret;
	}

	/* Set irq handler */
	aai->irq_handler = aai_irq_p7;

	/* Register ioctl */
	aai_ioctl_hwdep_new(aai);

	/* Set Sound Card datas */
	strcpy(card->driver, "AAI P7");
	strcpy(card->shortname, "AAI");
	sprintf(card->longname, "P7 Advanced Audio Interface %i", dev_id + 1);

	return err;
}

int aai_hw_remove(struct card_data_t *aai)
{

	/* TODO: reset all registers */

	/* Release clock */
	aai_release_clk(aai, ctrl_clk);
	kfree(aai->chans);

	return 0;
}

int aai_hw_start_channel(struct aai_device_t *chan)
{
	int err = 0;
	struct card_data_t *aai = chan->driver_data;
	unsigned long flags = 0;

	if (chan->ipcm >= aai->chans_nb) {
		dev_warn(chan2dev(chan), "wrong channel number %d\n",
			 chan->ipcm);
		err = -EINVAL;
		goto exit;
	}

	spin_lock_irqsave(&aai->hwlock, flags);

	/* update group ref count */
	if (chan->group != -1)
		aai->groups_ref_count[chan->group]++;

	/* start channel individually */
	chan->bytes_count = 0;
	chan->nbirq = 0;
	chan->nbperiod = 0;
	aai_it_en(aai, chan);

	spin_unlock_irqrestore(&aai->hwlock, flags);

exit:
	return err;
}

int aai_hw_stop_channel(struct aai_device_t *chan)
{
	int err = 0;
	struct card_data_t *aai = chan->driver_data;
	unsigned long flags = 0;

	if (chan->ipcm >= aai->chans_nb) {
		dev_warn(chan2dev(chan), "wrong channel number %d\n",
			 chan->ipcm);
		err = -EINVAL;
		goto exit;
	}

	spin_lock_irqsave(&aai->hwlock, flags);
	if (aai->pdata->chiprev > P7_CHIPREV_R1)
		aai_writereg(aai, 0, (uint32_t)chan->fifo.dmanv, 0);
	aai_it_dis(aai, chan);

	/* unset group if channel was synched */
	aai_group_unset(aai, chan);
	spin_unlock_irqrestore(&aai->hwlock, flags);

exit:
	return err;
}

void aai_hw_dma_free(struct snd_pcm_substream *substream)
{
	int err;
	struct aai_device_t *chan = substream2chan(substream);

	err = snd_pcm_lib_free_pages(substream);
	if (err)
		dev_warn(chan2dev(chan), "%s (%d) - unable to free dma area\n",
			 chan->name, chan->ipcm);

	return;
}

int aai_hw_dma_alloc(struct snd_pcm_substream *substream, size_t size)
{
	int err = 0;
	struct aai_device_t *chan = substream2chan(substream);

	err = snd_pcm_lib_malloc_pages(substream, size);
	if (err < 0)
		dev_warn(chan2dev(chan),
			 "%s(%d) - failed to allocate %d bytes\n",
			 chan->name, chan->ipcm, size);

	return err;
}

int aai_hw_dma_prepare(struct aai_device_t *chan)
{
	int err = 0;
	struct card_data_t *aai = chan->driver_data;
	struct aai_hw_channel_t *fifo = &chan->fifo;
	unsigned long flags = 0;
	uint32_t reg;

	spin_lock_irqsave(&aai->hwlock, flags);

	if (chan->direction & AAI_TX) {
		reg = aai_readreg(aai, AAI_MAIN_DMA_EN_TX);
		reg |= 1 << chan->fifo.fifo_id;
		aai_writereg(aai, reg, AAI_MAIN_DMA_EN_TX, 0);
	} else {
		reg = aai_readreg(aai, AAI_MAIN_DMA_EN_RX);
		reg |= 1 << chan->fifo.fifo_id;
		aai_writereg(aai, reg, AAI_MAIN_DMA_EN_RX, 0);
	}

	/* Set DMA buffer registers */
	chan->dma_burst_nb = chan->period_bytes / (chan->fifo_size >> 1);
	if (chan->dma_burst_nb > 0xff) {
		dev_dbg(aai->dev,
			"%s: '%s' period_bytes is larger than 256 fifo_size\n",
			__func__, chan->name);
		while (chan->dma_burst_nb > 0xff)
			chan->dma_burst_nb >>= 1;
	}
	chan->dma_xfer_size = chan->dma_burst_nb * (chan->fifo_size >> 1);

	/* Set DMA counter size according to dma transfer size */
	if (chan->direction & AAI_TX) {
		aai_writereg(aai, chan->dma_burst_nb - 1,
			     AAI_DMACFG_NC_TX(fifo->fifo_id), 0);
		aai_writereg(aai, chan->dma_burst_nb - 1,
			     AAI_DMACFG_CC_TX(fifo->fifo_id), 0);
	} else if (chan->direction & AAI_RX) {
		aai_writereg(aai, chan->dma_burst_nb - 1,
			     AAI_DMACFG_CC_RX(fifo->fifo_id), 0);
		aai_writereg(aai, chan->dma_burst_nb - 1,
			     AAI_DMACFG_NC_RX(fifo->fifo_id), 0);
	} else {
		dev_err(aai->dev, "%s: wrong channel direction\n", __func__);
		err = -EINVAL;
		goto exit;
	}

	chan->fifo.pfollow = chan->fifo.pcurrent + chan->dma_xfer_size;
	aai_writereg(aai, (uint32_t)chan->fifo.pcurrent,
		     (uint32_t)chan->fifo.dmaca, 0);
	aai_writereg(aai, (uint32_t)chan->fifo.pfollow,
		     (uint32_t)chan->fifo.dmana, 0);
	if (aai->pdata->chiprev > P7_CHIPREV_R1) {
		aai_writereg(aai, 1, (uint32_t)chan->fifo.dmacv, 0);
		aai_writereg(aai, 1, (uint32_t)chan->fifo.dmanv, 0);
	}

exit:
	spin_unlock_irqrestore(&aai->hwlock, flags);
	return err;
}

