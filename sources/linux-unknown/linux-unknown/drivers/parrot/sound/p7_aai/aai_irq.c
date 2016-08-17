
#include <linux/ratelimit.h>

#include "aai.h"
#include "aai_hw.h"

/* Period elaped */
static inline void aai_period_elapsed(struct card_data_t *aai,
				      struct aai_device_t *chan)
{
	struct aai_hw_channel_t *fifo = &chan->fifo;
	chan->bytes_count += chan->dma_xfer_size;

	/* Update pointers to DMA area */
	fifo->pcurrent = fifo->pfollow;
	fifo->pfollow += chan->dma_xfer_size;
	if (fifo->pfollow >= fifo->pend)
		fifo->pfollow = fifo->pstart;

	aai_writereg(aai, (uint32_t)chan->fifo.pfollow,
		     (uint32_t)chan->fifo.dmana, 0);
	if (aai->pdata->chiprev > P7_CHIPREV_R1)
		aai_writereg(aai, 1, (uint32_t)chan->fifo.dmanv, 0);

	/* Call snd_period_elapsed */
	if (chan->bytes_count >= chan->period_bytes) {
		if (chan->direction == AAI_RX)
			snd_pcm_period_elapsed(chan2captsubstream(aai,
								  chan->ipcm));
		else
			snd_pcm_period_elapsed(chan2pbacksubstream(aai,
								   chan->ipcm));
		chan->nbperiod++;
		chan->bytes_count -= chan->period_bytes;
	}
}

/* Second stage IRQ functions */
static inline void aai_irq_fifo_err(struct card_data_t *aai,
				    enum aai_dir_t direction)
{
	uint32_t itreg;
	uint32_t ackreg;
	uint32_t itsrc;
	uint32_t dmaen;
	struct aai_device_t *chan;
	int ipcm;

	if (direction == AAI_RX) {
		itreg = AAI_MAIN_RX_ERR_IT;
		ackreg = AAI_MAIN_RX_ERR_ITACK;
		dmaen = AAI_MAIN_DMA_EN_RX;
	} else {
		itreg = AAI_MAIN_TX_ERR_IT;
		ackreg = AAI_MAIN_TX_ERR_ITACK;
		dmaen = AAI_MAIN_DMA_EN_TX;
	}

	itsrc = aai_readreg(aai, itreg);
	for (ipcm = 0; ipcm < aai->chans_nb; ipcm++) {
		chan = &aai->chans[ipcm];
		if ((chan->direction & direction) &&
		    ((1 << chan->fifo.fifo_id) & itsrc)) {
			pr_err_ratelimited("%s (%d) - %s: fifo %d xrun\n",
					   chan->name, chan->ipcm, __func__, chan->fifo.fifo_id);
		}
	}

	/* Acknowledge all interrupts */
	aai_writereg(aai, itsrc, ackreg, 0);
}

static inline void aai_irq_dma_err(struct card_data_t *aai,
				   enum aai_dir_t direction)
{
	uint32_t itreg;
	uint32_t ackreg;
	uint32_t itsrc;
	uint32_t dmaen;
	struct aai_device_t *chan;
	int ipcm;

	if (direction == AAI_RX) {
		itreg = AAI_MAIN_RX_DMA_IT;
		ackreg = AAI_MAIN_RX_DMA_ITACK;
		dmaen = AAI_MAIN_DMA_EN_RX;
	} else {
		itreg = AAI_MAIN_TX_DMA_IT;
		ackreg = AAI_MAIN_TX_DMA_ITACK;
		dmaen = AAI_MAIN_DMA_EN_TX;
	}

	itsrc = aai_readreg(aai, itreg);
	for (ipcm = 0; ipcm < aai->chans_nb; ipcm++) {
		chan = &aai->chans[ipcm];
		if ((chan->direction & direction)
		    && ((1 << chan->fifo.fifo_id) & itsrc)) {
			pr_err_ratelimited("%s (%d) - %s: dma error\n",
					   chan->name, chan->ipcm, __func__);
		}
	}

	/* Acknowledge all interrupts */
	aai_writereg(aai, itsrc, ackreg, 0);
}

static inline void aai_irq_ok(struct card_data_t *aai,
			      enum aai_dir_t direction)
{
	uint32_t itsrc;
	uint32_t reg;
	uint32_t itreg;
	uint32_t ackreg;
	uint32_t dmaen;
	struct aai_device_t *chan;
	int ipcm;

	if (direction == AAI_RX) {
		itreg = AAI_MAIN_RX_OK_IT;
		ackreg = AAI_MAIN_RX_OK_ITACK;
		dmaen = AAI_MAIN_DMA_EN_RX;
	} else {
		itreg = AAI_MAIN_TX_OK_IT;
		ackreg = AAI_MAIN_TX_OK_ITACK;
		dmaen = AAI_MAIN_DMA_EN_TX;
	}

	itsrc  = aai_readreg(aai, itreg);
	itsrc &= aai_readreg(aai, dmaen);

	for (ipcm = 0; ipcm < aai->chans_nb; ipcm++) {
		chan = &aai->chans[ipcm];
		if ((chan->direction & direction)
		    && ((1 << chan->fifo.fifo_id) & itsrc)) {
			chan->nbirq++;
			aai_period_elapsed(aai, chan);

			/* Acknowledge interrupt */
			reg = 1 << chan->fifo.fifo_id;
			aai_writereg(aai, reg, ackreg, 0);
		}
	}
}

/**
 * @brief Main P7 AAI IRQ handler
 *
 * @param irq  number of irq
 * @param dev  device targeted
 *
 * @return IRQ_HANDLED
 **/
irqreturn_t aai_irq_p7(int irq, void *dev)
{
	const uint32_t itgroup_msk = 0x7fff0000;
	struct card_data_t *aai = dev;
	uint32_t itsrc;
	uint32_t itsrc_masked;

	itsrc = aai_readreg(aai, AAI_MAIN_IT);

	/*
	 * mask itsrc with expected interrupts
	 * might be some rements of previous channels
	 */
	itsrc_masked = itsrc & aai_readreg(aai, AAI_MAIN_ITEN);

	/* First stage of aai irq */
	if (unlikely(itsrc & (1 << 1)))
		aai_irq_fifo_err(aai, AAI_RX);

	if (unlikely(itsrc & (1 << 2)))
		aai_irq_dma_err(aai, AAI_RX);

	if (unlikely(itsrc & (1 << 9)))
		aai_irq_fifo_err(aai, AAI_TX);

	if (unlikely(itsrc & (1 << 10)))
		aai_irq_dma_err(aai, AAI_TX);

	if (itsrc_masked & (1 << 0))
		aai_irq_ok(aai, AAI_RX);

	if (itsrc_masked & (1 << 8))
		aai_irq_ok(aai, AAI_TX);

	/*
	 * Acknowlegde group interrupt if necessary.
	 * Individual interrupts have been handled in stage 2 functions.
	 */
	if (itgroup_msk & itsrc_masked)
		aai_writereg(aai, itgroup_msk & itsrc_masked,
			     AAI_MAIN_ITACK, 0);

	aai_writereg(aai, itsrc, AAI_MAIN_ITACK, 0);

	return IRQ_HANDLED;
}

