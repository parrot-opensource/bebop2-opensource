/*-======================================================================-
 * Parrot S.A. Confidential Property
 *
 * Project      : Parrot 7
 * File name    : reg_aai.h
 * Author       : Marc Tamisier
 *
 * ----------------------------------------------------------------------
 * Purpose : AAI registers definition
 *
 */
#ifndef __REG_AAI_H__
#define __REG_AAI_H__

/*
 * AAI sub-module OFFSET definition
 */
#define AAI_MAIN_OFFSET         0x0000
#define AAI_AAICFG_OFFSET       0x1000
#define AAI_FIFOCFG_OFFSET      0x2000
#define AAI_DMACFG_OFFSET       0x3000
#define AAI_FIFORX_OFFSET       0x6000
#define AAI_FIFOTX_OFFSET       0x7000
#define AAI_LBC_OFFSET          0xF000

/*
 * AAI registers address definition
 */

/*
 * AAI MAIN sub-module registers address definition
 */

/* IP name: AAI ; IP sub-block name: MAIN ; register name: IT */
#define AAI_MAIN_IT			(AAI_MAIN_OFFSET + 0x000)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: ITEN */
#define AAI_MAIN_ITEN			(AAI_MAIN_OFFSET + 0x004)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: ITACK */
#define AAI_MAIN_ITACK			(AAI_MAIN_OFFSET + 0x008)

/* IP name: AAI ; IP sub-block name: MAIN ; register name: RX_OK_IT */
#define AAI_MAIN_RX_OK_IT		(AAI_MAIN_OFFSET + 0x010)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: RX_OK_ITACK */
#define AAI_MAIN_RX_OK_ITACK		(AAI_MAIN_OFFSET + 0x014)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: TX_OK_IT */
#define AAI_MAIN_TX_OK_IT		(AAI_MAIN_OFFSET + 0x018)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: TX_OK_ITACK */
#define AAI_MAIN_TX_OK_ITACK		(AAI_MAIN_OFFSET + 0x01C)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: RX_ERR_IT */
#define AAI_MAIN_RX_ERR_IT		(AAI_MAIN_OFFSET + 0x020)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: RX_ERR_ITACK */
#define AAI_MAIN_RX_ERR_ITACK		(AAI_MAIN_OFFSET + 0x024)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: TX_ERR_IT */
#define AAI_MAIN_TX_ERR_IT		(AAI_MAIN_OFFSET + 0x028)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: TX_ERR_ITACK */
#define AAI_MAIN_TX_ERR_ITACK		(AAI_MAIN_OFFSET + 0x02C)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: RX_DMA_IT */
#define AAI_MAIN_RX_DMA_IT		(AAI_MAIN_OFFSET + 0x030)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: RX_DMA_ITACK */
#define AAI_MAIN_RX_DMA_ITACK		(AAI_MAIN_OFFSET + 0x034)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: TX_DMA_IT */
#define AAI_MAIN_TX_DMA_IT		(AAI_MAIN_OFFSET + 0x038)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: TX_DMA_ITACK */
#define AAI_MAIN_TX_DMA_ITACK		(AAI_MAIN_OFFSET + 0x03C)

/* IP name: AAI ; IP sub-block name: MAIN ; register name: IT_GROUPS_Rx (0-2) */
#define AAI_MAIN_IT_GROUPS_RX(_id_) \
	(AAI_MAIN_OFFSET + (0x040 + 0x4*(_id_)))
/* IP name: AAI ; IP sub-block name: MAIN ; register name: IT_GROUPS_Tx (0-1) */
#define AAI_MAIN_IT_GROUPS_TX(_id_) \
	(AAI_MAIN_OFFSET + (0x050 + 0x4*(_id_)))
/* IP name: AAI ; IP sub-block name: MAIN ; register name: IT_GROUPS_START */
#define AAI_MAIN_IT_GROUPS_START	(AAI_MAIN_OFFSET + 0x060)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: IT_GROUPS_STOP */
#define AAI_MAIN_IT_GROUPS_STOP		(AAI_MAIN_OFFSET + 0x064)

/* IP name: AAI ; IP sub-block name: MAIN ; register name: DMA_EN_RX */
#define AAI_MAIN_DMA_EN_RX		(AAI_MAIN_OFFSET + 0x080)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: DMA_EN_TX */
#define AAI_MAIN_DMA_EN_TX		(AAI_MAIN_OFFSET + 0x084)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: FIFO_EN_RX */
#define AAI_MAIN_FIFO_EN_RX		(AAI_MAIN_OFFSET + 0x088)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: FIFO_EN_TX */
#define AAI_MAIN_FIFO_EN_TX		(AAI_MAIN_OFFSET + 0x08C)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: DMA_BUSY_RX */
#define AAI_MAIN_DMA_BUSY_RX		(AAI_MAIN_OFFSET + 0x090)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: DMA_BUSY_TX */
#define AAI_MAIN_DMA_BUSY_TX		(AAI_MAIN_OFFSET + 0x094)

/* IP name: AAI ; IP sub-block name: MAIN ; register name: TIMER_IT */
#define AAI_MAIN_TIMER_IT		(AAI_MAIN_OFFSET + 0x100)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: TIMER_ITACK */
#define AAI_MAIN_TIMER_ITACK		(AAI_MAIN_OFFSET + 0x104)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: TIMER_EN */
#define AAI_MAIN_TIMER_EN		(AAI_MAIN_OFFSET + 0x108)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: TIMER_MODE */
#define AAI_MAIN_TIMER_MODE		(AAI_MAIN_OFFSET + 0x10C)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: TIMER_SCALE */
#define AAI_MAIN_TIMER_SCALE		(AAI_MAIN_OFFSET + 0x110)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: TIMER_LOAD */
#define AAI_MAIN_TIMER_LOAD		(AAI_MAIN_OFFSET + 0x120)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: TIMER_RESET */
#define AAI_MAIN_TIMER_RESET		(AAI_MAIN_OFFSET + 0x124)
/* IP name: AAI ; IP sub-block name: MAIN ; register name: TIMER_SELECT */
#define AAI_MAIN_TIMER_SELECT(timer_id) \
	(AAI_MAIN_OFFSET + (0x140 + 0x4*(timer_id)))
/* IP name: AAI ; IP sub-block name: MAIN ; register name: TIMER_SETVAL */
#define AAI_MAIN_TIMER_SETVAL(timer_id) \
	(AAI_MAIN_OFFSET + (0x180 + 0x4*(timer_id)))
/* IP name: AAI ; IP sub-block name: MAIN ; register name: TIMER_GETVAL */
#define AAI_MAIN_TIMER_GETVAL(timer_id) \
	(AAI_MAIN_OFFSET + (0x1C0 + 0x4*(timer_id)))


/*
 *  AAI Audio Channel Configuration sub-module registers address definition
 */
/* IP name: AAI ; IP sub-block name: AAICFG ; register name: MUS       */
#define AAI_AAICFG_MUS(mus_id) \
	(AAI_AAICFG_OFFSET + (0x000 + 0x4*(mus_id)))
/* IP name: AAI ; IP sub-block name: AAICFG ; register name: TDM_IN        */
#define AAI_AAICFG_TDM_IN		(AAI_AAICFG_OFFSET + 0x020)
/* IP name: AAI ; IP sub-block name: AAICFG ; register name: TDM_OUT       */
#define AAI_AAICFG_TDM_OUT		(AAI_AAICFG_OFFSET + 0x024)
/* IP name: AAI ; IP sub-block name: AAICFG ; register name: ADC           */
#define AAI_AAICFG_ADC			(AAI_AAICFG_OFFSET + 0x040)
/* IP name: AAI ; IP sub-block name: AAICFG ; register name: DAC           */
#define AAI_AAICFG_DAC			(AAI_AAICFG_OFFSET + 0x044)
/* IP name: AAI ; IP sub-block name: AAICFG ; register name: AUX           */
#define AAI_AAICFG_AUX			(AAI_AAICFG_OFFSET + 0x048)
/* IP name: AAI ; IP sub-block name: AAICFG ; register name: MIC_MUX       */
#define AAI_AAICFG_MIC_MUX		(AAI_AAICFG_OFFSET + 0x04C)
/* IP name: AAI ; IP sub-block name: AAICFG ; register name: VOI_MUX       */
#define AAI_AAICFG_VOI_MUX		(AAI_AAICFG_OFFSET + 0x050)
/* IP name: AAI ; IP sub-block name: AAICFG ; register name: PCM         */
#define AAI_AAICFG_PCM(pcm_id) \
	(AAI_AAICFG_OFFSET + (0x080 + 0x4 * (pcm_id)))
/* IP name: AAI ; IP sub-block name: AAICFG ; register name: SPDIFRX       */
#define AAI_AAICFG_SPDIFRX		(AAI_AAICFG_OFFSET + 0x0C0)
/* IP name: AAI ; IP sub-block name: AAICFG ; register name: SPDIFTX       */
#define AAI_AAICFG_SPDIFTX		(AAI_AAICFG_OFFSET + 0x0C4)
/* IP name: AAI ; IP sub-block name: AAICFG ; register name: VOICE */
#define AAI_AAICFG_VOICE(voice_id) \
	(AAI_AAICFG_OFFSET + (0x100 + 0x4 * (voice_id)))
/* IP name: AAI ; IP sub-block name: AAICFG ; register name: VOICE_SPEED   */
#define AAI_AAICFG_VOICE_SPEED		(AAI_AAICFG_OFFSET + 0x120)
/* IP name: AAI ; IP sub-block name: AAICFG ; register name: VOICE_8K      */
#define AAI_AAICFG_VOICE_8K		(AAI_AAICFG_OFFSET + 0x124)
/* IP name: AAI ; IP sub-block name: AAICFG ; register name: MUS_FILTER5   */
#define AAI_AAICFG_MUS_FILTER5		(AAI_AAICFG_OFFSET + 0x140)
/* IP name: AAI ; IP sub-block name: AAICFG ; register name: SRC_FILTER5   */
#define AAI_AAICFG_SRC_FILTER5		(AAI_AAICFG_OFFSET + 0x180)
/* IP name: AAI ; IP sub-block name: AAICFG ; register name: SRC_MODE      */
#define AAI_AAICFG_SRC_MODE		(AAI_AAICFG_OFFSET + 0x184)
/* IP name: AAI ; IP sub-block name: AAICFG ; register name: SRC_SPEED0    */
#define AAI_AAICFG_SRC_SPEED0		(AAI_AAICFG_OFFSET + 0x188)
/* IP name: AAI ; IP sub-block name: AAICFG ; register name: SRC_SPEED1    */
#define AAI_AAICFG_SRC_SPEED1		(AAI_AAICFG_OFFSET + 0x18C)

/*
 *  AAI DMA Configuration sub-module registers address definition
 */
/* IP name: AAI ; IP sub-block name: DMACFG ; register name: CA_RX */
#define AAI_DMACFG_CA_RX(rxfifo_id) \
	(AAI_DMACFG_OFFSET + (0x000 + 0x4 * (rxfifo_id)))
/* IP name: AAI ; IP sub-block name: DMACFG ; register name: CA_TX */
#define AAI_DMACFG_CA_TX(txfifo_id) \
	(AAI_DMACFG_OFFSET + (0x100 + 0x4 * (txfifo_id)))
/* IP name: AAI ; IP sub-block name: DMACFG ; register name: NA_RX */
#define AAI_DMACFG_NA_RX(rxfifo_id) \
	(AAI_DMACFG_OFFSET + (0x200 + 0x4 * (rxfifo_id)))
/* IP name: AAI ; IP sub-block name: DMACFG ; register name: NA_TX */
#define AAI_DMACFG_NA_TX(txfifo_id) \
	(AAI_DMACFG_OFFSET + (0x300 + 0x4 * (txfifo_id)))
/* IP name: AAI ; IP sub-block name: DMACFG ; register name: CC_RX */
#define AAI_DMACFG_CC_RX(rxfifo_id) \
	(AAI_DMACFG_OFFSET + (0x400 + 0x4 * (rxfifo_id)))
/* IP name: AAI ; IP sub-block name: DMACFG ; register name: CC_TX */
#define AAI_DMACFG_CC_TX(txfifo_id) \
	(AAI_DMACFG_OFFSET + (0x500 + 0x4 * (txfifo_id)))
/* IP name: AAI ; IP sub-block name: DMACFG ; register name: NC_RX */
#define AAI_DMACFG_NC_RX(rxfifo_id) \
	(AAI_DMACFG_OFFSET + (0x600 + 0x4 * (rxfifo_id)))
/* IP name: AAI ; IP sub-block name: DMACFG ; register name: NC_TX */
#define AAI_DMACFG_NC_TX(txfifo_id) \
	(AAI_DMACFG_OFFSET + (0x700 + 0x4 * (txfifo_id)))
/* IP name: AAI ; IP sub-block name: DMACFG ; register name: CV_RX */
#define AAI_DMACFG_CV_RX(rxfifo_id) \
	(AAI_DMACFG_OFFSET + (0x800 + 0x4 * (rxfifo_id)))
/* IP name: AAI ; IP sub-block name: DMACFG ; register name: CV_TX */
#define AAI_DMACFG_CV_TX(txfifo_id) \
	(AAI_DMACFG_OFFSET + (0x900 + 0x4 * (txfifo_id)))
/* IP name: AAI ; IP sub-block name: DMACFG ; register name: NV_RX */
#define AAI_DMACFG_NV_RX(rxfifo_id) \
	(AAI_DMACFG_OFFSET + (0xa00 + 0x4 * (rxfifo_id)))
/* IP name: AAI ; IP sub-block name: DMACFG ; register name: NV_TX */
#define AAI_DMACFG_NV_TX(txfifo_id) \
	(AAI_DMACFG_OFFSET + (0xb00 + 0x4 * (txfifo_id)))


/*
 *  AAI FIFO Configuration sub-module registers address definition
 */
/* IP name: AAI ; IP sub-block name: FIFOCFG ; register name: CFG_RX    */
#define AAI_FIFOCFG_CFG_RX(rxfifo_id) \
	(AAI_FIFOCFG_OFFSET + (0x000 + 0x4 * (rxfifo_id)))
/* IP name: AAI ; IP sub-block name: FIFOCFG ; register name: CFG_TX    */
#define AAI_FIFOCFG_CFG_TX(txfifo_id) \
	(AAI_FIFOCFG_OFFSET + (0x100 + 0x4 * (txfifo_id)))
/* IP name: AAI ; IP sub-block name: FIFOCFG ; register name: SEL_RX    */
#define AAI_FIFOCFG_SEL_RX(rxchan_id) \
	(AAI_FIFOCFG_OFFSET + (0x200 + 0x4 * (rxchan_id)))
/* IP name: AAI ; IP sub-block name: FIFOCFG ; register name: SEL_TX    */
#define AAI_FIFOCFG_SEL_TX(txchan_id) \
	(AAI_FIFOCFG_OFFSET + (0x300 + 0x4 * (txchan_id)))
/* IP name: AAI ; IP sub-block name: FIFOCFG ; register name: RX_16BIT   */
#define AAI_FIFOCFG_RX_16BIT		(AAI_FIFOCFG_OFFSET + 0x400)
/* IP name: AAI ; IP sub-block name: FIFOCFG ; register name: TX_16BIT   */
#define AAI_FIFOCFG_TX_16BIT		(AAI_FIFOCFG_OFFSET + 0x404)
/* IP name: AAI ; IP sub-block name: FIFOCFG ; register name: RX_ENDIAN  */
#define AAI_FIFOCFG_RX_ENDIAN		(AAI_FIFOCFG_OFFSET + 0x408)
/* IP name: AAI ; IP sub-block name: FIFOCFG ; register name: TX_ENDIAN  */
#define AAI_FIFOCFG_TX_ENDIAN		(AAI_FIFOCFG_OFFSET + 0x40C)


/*
 *  AAI FIFO RX sub-module registers address definition
 */
/* IP name: AAI ; IP sub-block name: - ; register name: FIFORX */
#define AAI_FIFORX(rxfifo_id) \
	(AAI_FIFORX_OFFSET + (0x000 + 0x40 * (rxfifo_id)))

/*
 *  AAI FIFO TX sub-module registers address definition
 */
/* IP name: AAI ; IP sub-block name: - ; register name: FIFOTX */
#define AAI_FIFOTX(txfifo_id) \
	(AAI_FIFOTX_OFFSET + (0x000 + 0x40 * (txfifo_id)))

/*
 *  AAI registers bit offset definition
 */

/*
 *  AAI MAIN sub-module registers bit offset definition
 */
/* IP name: AAI ; register name: IT ; bits field name: RX_DONE */
#define AAI_IT_RX_DONE_SHIFT		0
#define AAI_IT_RX_DONE_WIDTH		1
#define AAI_IT_RX_DONE_MSK \
	((uint32_t)((1 << AAI_IT_RX_DONE_WIDTH) - 1) << AAI_IT_RX_DONE_SHIFT)
/* IP name: AAI ; register name: IT ; bits field name: RX_FIFO_ERR */
#define AAI_IT_RX_FIFO_ERR_SHIFT	1
#define AAI_IT_RX_FIFO_ERR_WIDTH	1
#define AAI_IT_RX_FIFO_ERR_MSK \
	((uint32_t)((1 << AAI_IT_RX_FIFO_ERR_WIDTH) - 1) << \
	 AAI_IT_RX_FIFO_ERR_SHIFT)
/* IP name: AAI ; register name: IT ; bits field name: RX_DMA_ERR */
#define AAI_IT_RX_DMA_ERR_SHIFT		2
#define AAI_IT_RX_DMA_ERR_WIDTH		1
#define AAI_IT_RX_DMA_ERR_MSK \
	((uint32_t)((1 << AAI_IT_RX_DMA_ERR_WIDTH) - 1) << \
	 AAI_IT_RX_DMA_ERR_SHIFT)
/* IP name: AAI ; register name: IT ; bits field name: TX_DONE */
#define AAI_IT_TX_DONE_SHIFT		8
#define AAI_IT_TX_DONE_WIDTH		1
#define AAI_IT_TX_DONE_MSK \
	((uint32_t)((1 << AAI_IT_TX_DONE_WIDTH) - 1) << AAI_IT_TX_DONE_SHIFT)
/* IP name: AAI ; register name: IT ; bits field name: TX_FIFO_ERR */
#define AAI_IT_TX_FIFO_ERR_SHIFT	9
#define AAI_IT_TX_FIFO_ERR_WIDTH	1
#define AAI_IT_TX_FIFO_ERR_MSK \
	((uint32_t)((1 << AAI_IT_TX_FIFO_ERR_WIDTH) - 1) << \
	 AAI_IT_TX_FIFO_ERR_SHIFT)
/* IP name: AAI ; register name: IT ; bits field name: TX_DMA_ERR */
#define AAI_IT_TX_DMA_ERR_SHIFT		10
#define AAI_IT_TX_DMA_ERR_WIDTH		1
#define AAI_IT_TX_DMA_ERR_MSK \
	((uint32_t)((1 << AAI_IT_TX_DMA_ERR_WIDTH) - 1) << \
	 AAI_IT_TX_DMA_ERR_SHIFT)
/* IP name: AAI ; register name: IT ; bits field name: TIMER_IT */
#define AAI_IT_TIMER_IT_SHIFT		15
#define AAI_IT_TIMER_IT_WIDTH		1
#define AAI_IT_TIMER_IT_MSK \
	((uint32_t)((1 << AAI_IT_TIMER_IT_WIDTH) - 1) << AAI_IT_TIMER_IT_SHIFT)
/* IP name: AAI ; register name: IT ; bits field name: GROUP_ITEN */
#define AAI_IT_GROUP_ITEN_SHIFT		16
#define AAI_IT_GROUP_ITEN_WIDTH		15
#define AAI_IT_GROUP_ITEN_MSK \
	((uint32_t)((1 << AAI_IT_GROUP_ITEN_WIDTH) - 1) << \
	 AAI_IT_GROUP_ITEN_SHIFT)
/* IP name: AAI ; register name: IT ; bits field name: ONGOING_END_IT */
#define AAI_IT_ONGOING_END_IT_SHIFT	31
#define AAI_IT_ONGOING_END_IT_WIDTH	1
#define AAI_IT_ONGOING_END_IT_MSK \
	((uint32_t)((1 << AAI_IT_ONGOING_END_IT_WIDTH) - 1) << \
	 AAI_IT_ONGOING_END_IT_SHIFT)

/* IP name: AAI ; register name: ITEN ; bits field name: RX_DONE */
#define AAI_ITEN_RX_DONE_SHIFT		0
#define AAI_ITEN_RX_DONE_WIDTH		1
#define AAI_ITEN_RX_DONE_MSK \
	((uint32_t)((1 << AAI_ITEN_RX_DONE_WIDTH) - 1) << \
	 AAI_ITEN_RX_DONE_SHIFT)
/* IP name: AAI ; register name: ITEN ; bits field name: RX_FIFO_ERR */
#define AAI_ITEN_RX_FIFO_ERR_SHIFT	1
#define AAI_ITEN_RX_FIFO_ERR_WIDTH	1
#define AAI_ITEN_RX_FIFO_ERR_MSK \
	((uint32_t)((1 << AAI_ITEN_RX_FIFO_ERR_WIDTH) - 1) << \
	 AAI_ITEN_RX_FIFO_ERR_SHIFT)
/* IP name: AAI ; register name: ITEN ; bits field name: RX_DMA_ERR */
#define AAI_ITEN_RX_DMA_ERR_SHIFT	2
#define AAI_ITEN_RX_DMA_ERR_WIDTH	1
#define AAI_ITEN_RX_DMA_ERR_MSK \
	((uint32_t)((1 << AAI_ITEN_RX_DMA_ERR_WIDTH) - 1) << \
	 AAI_ITEN_RX_DMA_ERR_SHIFT)
/* IP name: AAI ; register name: ITEN ; bits field name: TX_DONE */
#define AAI_ITEN_TX_DONE_SHIFT		8
#define AAI_ITEN_TX_DONE_WIDTH		1
#define AAI_ITEN_TX_DONE_MSK \
	((uint32_t)((1 << AAI_ITEN_TX_DONE_WIDTH) - 1) << \
	 AAI_ITEN_TX_DONE_SHIFT)
/* IP name: AAI ; register name: ITEN ; bits field name: TX_FIFO_ERR */
#define AAI_ITEN_TX_FIFO_ERR_SHIFT	9
#define AAI_ITEN_TX_FIFO_ERR_WIDTH	1
#define AAI_ITEN_TX_FIFO_ERR_MSK \
	((uint32_t)((1 << AAI_ITEN_TX_FIFO_ERR_WIDTH) - 1) << \
	 AAI_ITEN_TX_FIFO_ERR_SHIFT)
/* IP name: AAI ; register name: ITEN ; bits field name: TX_DMA_ERR */
#define AAI_ITEN_TX_DMA_ERR_SHIFT	10
#define AAI_ITEN_TX_DMA_ERR_WIDTH	1
#define AAI_ITEN_TX_DMA_ERR_MSK \
	((uint32_t)((1 << AAI_ITEN_TX_DMA_ERR_WIDTH) - 1) << \
	 AAI_ITEN_TX_DMA_ERR_SHIFT)
/* IP name: AAI ; register name: ITEN ; bits field name: TIMER_IT */
#define AAI_ITEN_TIMER_IT_SHIFT		15
#define AAI_ITEN_TIMER_IT_WIDTH		1
#define AAI_ITEN_TIMER_IT_MSK \
	((uint32_t)((1 << AAI_ITEN_TIMER_IT_WIDTH) - 1) << \
	 AAI_ITEN_TIMER_IT_SHIFT)
/* IP name: AAI ; register name: ITEN ; bits field name: GROUP_ITEN */
#define AAI_ITEN_GROUP_ITEN_SHIFT	16
#define AAI_ITEN_GROUP_ITEN_WIDTH	15
#define AAI_ITEN_GROUP_ITEN_MSK \
	((uint32_t)((1 << AAI_ITEN_GROUP_ITEN_WIDTH) - 1) << \
	 AAI_ITEN_GROUP_ITEN_SHIFT)
/* IP name: AAI ; register name: ITEN ; bits field name: ONGOING_END_IT */
#define AAI_ITEN_ONGOING_END_IT_SHIFT	31
#define AAI_ITEN_ONGOING_END_IT_WIDTH	1
#define AAI_ITEN_ONGOING_END_IT_MSK \
	((uint32_t)((1 << AAI_ITEN_ONGOING_END_IT_WIDTH) - 1) << \
	 AAI_ITEN_ONGOING_END_IT_SHIFT)

/*
 *  AAI Audio Channel Configuration sub-module registers bit offset definition
 */
/* IP name: AAI ; register name: MUSx ; bits field name: MUSx_CYCLES_NUM */
#define AAI_MUSx_CYCLES_NUM_SHIFT	0
#define AAI_MUSx_CYCLES_NUM_WIDTH	7
#define AAI_MUSx_CYCLES_NUM_MSK \
	((uint32_t)((1 << AAI_MUSx_CYCLES_NUM_WIDTH) - 1) << \
	 AAI_MUSx_CYCLES_NUM_SHIFT)
/* IP name: AAI ; register name: MUSx ; bits field name: MATSUSHITA */
#define AAI_MUSx_MATSUSHITA_SHIFT	7
#define AAI_MUSx_MATSUSHITA_WIDTH	1
#define AAI_MUSx_MATSUSHITA_MSK \
	((uint32_t)((1 << AAI_MUSx_MATSUSHITA_WIDTH) - 1) << \
	 AAI_MUSx_MATSUSHITA_SHIFT)
/* IP name: AAI ; register name: MUSx ; bits field name: LEFT_FRAME */
#define AAI_MUSx_LEFT_FRAME_SHIFT	8
#define AAI_MUSx_LEFT_FRAME_WIDTH	1
#define AAI_MUSx_LEFT_FRAME_MSK \
	((uint32_t)((1 << AAI_MUSx_LEFT_FRAME_WIDTH) - 1) << \
	 AAI_MUSx_LEFT_FRAME_SHIFT)
/* IP name: AAI ; register name: MUSx ; bits field name: LSB_FIRST */
#define AAI_MUSx_LSB_FIRST_SHIFT	9
#define AAI_MUSx_LSB_FIRST_WIDTH	1
#define AAI_MUSx_LSB_FIRST_MSK \
	((uint32_t)((1 << AAI_MUSx_LSB_FIRST_WIDTH) - 1) << \
	 AAI_MUSx_LSB_FIRST_SHIFT)
/* IP name: AAI ; register name: MUSx ; bits field name: RIGHT_JUST */
#define AAI_MUSx_RIGHT_JUST_SHIFT	10
#define AAI_MUSx_RIGHT_JUST_WIDTH	1
#define AAI_MUSx_RIGHT_JUST_MSK \
	((uint32_t)((1 << AAI_MUSx_RIGHT_JUST_WIDTH) - 1) << \
	 AAI_MUSx_RIGHT_JUST_SHIFT)
/* IP name: AAI ; register name: MUSx ; bits field name: 24BITS */
#define AAI_MUSx_24BITS_SHIFT		11
#define AAI_MUSx_24BITS_WIDTH		1
#define AAI_MUSx_24BITS_MSK \
	((uint32_t)((1 << AAI_MUSx_24BITS_WIDTH) - 1) << AAI_MUSx_24BITS_SHIFT)
/* IP name: AAI ; register name: MUSx ; bits field name: SYNCHRO_TYPE */
#define AAI_MUSx_SYNCHRO_TYPE_SHIFT	12
#define AAI_MUSx_SYNCHRO_TYPE_WIDTH	3
#define AAI_MUSx_SYNCHRO_TYPE_MSK \
	((uint32_t)((1 << AAI_MUSx_SYNCHRO_TYPE_WIDTH) - 1) << \
	 AAI_MUSx_SYNCHRO_TYPE_SHIFT)
/* IP name: AAI ; register name: MUSx ; bits field name: USE_SPDIF */
#define AAI_MUSx_USE_SPDIF_SHIFT	16
#define AAI_MUSx_USE_SPDIF_WIDTH	1
#define AAI_MUSx_USE_SPDIF_MSK \
	((uint32_t)((1 << AAI_MUSx_USE_SPDIF_WIDTH) - 1) << \
	 AAI_MUSx_USE_SPDIF_SHIFT)
/* IP name: AAI ; register name: MUSx ; bits field name: BIPHASE_SOURCE */
#define AAI_MUSx_BIPHASE_SOURCE_SHIFT	17
#define AAI_MUSx_BIPHASE_SOURCE_WIDTH	1
#define AAI_MUSx_BIPHASE_SOURCE_MSK \
	((uint32_t)((1 << AAI_MUSx_BIPHASE_SOURCE_WIDTH) - 1) << \
	 AAI_MUSx_BIPHASE_SOURCE_SHIFT)
/* IP name: AAI ; register name: MUSx ; bits field name: BINARY_ACCEPT */
#define AAI_MUSx_BINARY_ACCEPT_SHIFT	18
#define AAI_MUSx_BINARY_ACCEPT_WIDTH	1
#define AAI_MUSx_BINARY_ACCEPT_MSK \
	((uint32_t)((1 << AAI_MUSx_BINARY_ACCEPT_WIDTH) - 1) << \
	 AAI_MUSx_BINARY_ACCEPT_SHIFT)

#define AAI_MUSx_SYNCHRO_TYPE_DAC	0
#define AAI_MUSx_SYNCHRO_TYPE_ADC	1
#define AAI_MUSx_SYNCHRO_TYPE_AUX	2
#define AAI_MUSx_SYNCHRO_TYPE_ASYNCH	3
#define AAI_MUSx_SYNCHRO_TYPE_TDM0	4
#define AAI_MUSx_SYNCHRO_TYPE_TDM1	5
#define AAI_MUSx_SYNCHRO_TYPE_TDM2	6
#define AAI_MUSx_SYNCHRO_TYPE_TDM3	7

/* IP name: AAI ; register name:TDM_IN; bits field name: TDMxx_CYCLES_NUMS */
#define AAI_TDM_IN_CYCLES_NUM_SHIFT	0
#define AAI_TDM_IN_CYCLES_NUM_WIDTH	7
#define AAI_TDM_IN_CYCLES_NUM_MSK \
	((uint32_t)((1 << AAI_TDM_IN_CYCLES_NUM_WIDTH) - 1) << \
	 AAI_TDM_IN_CYCLES_NUM_SHIFT)
/* IP name: AAI ; register name:TDM_IN; bits field name: TDMxx_LEFT_FRAME */
#define AAI_TDM_IN_LEFT_FRAME_SHIFT	8
#define AAI_TDM_IN_LEFT_FRAME_WIDTH	1
#define AAI_TDM_IN_LEFT_FRAME_MSK \
	((uint32_t)((1 << AAI_TDM_IN_LEFT_FRAME_WIDTH) - 1) << \
	 AAI_TDM_IN_LEFT_FRAME_SHIFT)
/* IP name: AAI ; register name:TDM_IN; bits field name: TDMxx_EN */
#define AAI_TDM_IN_EN_SHIFT		31
#define AAI_TDM_IN_EN_WIDTH		1
#define AAI_TDM_IN_EN_MSK \
	((uint32_t)((1 << AAI_TDM_IN_EN_WIDTH) - 1) << AAI_TDM_IN_EN_SHIFT)

/* IP name: AAI ; register name:TDM_OUT; bits field name: TDMxx_CYCLES_NUMS */
#define AAI_TDM_OUT_CYCLES_NUM_SHIFT	0
#define AAI_TDM_OUT_CYCLES_NUM_WIDTH	7
#define AAI_TDM_OUT_CYCLES_NUM_MSK \
	((uint32_t)((1 << AAI_TDM_OUT_CYCLES_NUM_WIDTH) - 1) << \
	 AAI_TDM_OUT_CYCLES_NUM_SHIFT)
/* IP name: AAI ; register name:TDM_OUT; bits field name: TDMxx_LEFT_FRAME */
#define AAI_TDM_OUT_LEFT_FRAME_SHIFT	8
#define AAI_TDM_OUT_LEFT_FRAME_WIDTH	1
#define AAI_TDM_OUT_LEFT_FRAME_MSK \
	((uint32_t)((1 << AAI_TDM_OUT_LEFT_FRAME_WIDTH) - 1) << \
	 AAI_TDM_OUT_LEFT_FRAME_SHIFT)
/* IP name: AAI ; register name:TDM_OUT; bits field name: TDMxx_EN */
#define AAI_TDM_OUT_EN_SHIFT		31
#define AAI_TDM_OUT_EN_WIDTH		1
#define AAI_TDM_OUT_EN_MSK \
	((uint32_t)((1 << AAI_TDM_OUT_EN_WIDTH) - 1) << AAI_TDM_OUT_EN_SHIFT)

/* IP name: AAI ; register name:ADC; bits field name: ADC_CYCLES_NUMS */
#define AAI_ADC_CYCLES_NUM_SHIFT	0
#define AAI_ADC_CYCLES_NUM_WIDTH	7
#define AAI_ADC_CYCLES_NUM_MSK \
	((uint32_t)((1 << AAI_ADC_CYCLES_NUM_WIDTH) - 1) << \
	 AAI_ADC_CYCLES_NUM_SHIFT)
/* IP name: AAI ; register name:ADC; bits field name: ADC_MATSUSHITA */
#define AAI_ADC_MATSUSHITA_SHIFT	7
#define AAI_ADC_MATSUSHITA_WIDTH	1
#define AAI_ADC_MATSUSHITA_MSK \
	((uint32_t)((1 << AAI_ADC_MATSUSHITA_WIDTH) - 1) << \
	 AAI_ADC_MATSUSHITA_SHIFT)
/* IP name: AAI ; register name:ADC; bits field name: ADC_LEFT_FRAME */
#define AAI_ADC_LEFT_FRAME_SHIFT	8
#define AAI_ADC_LEFT_FRAME_WIDTH	1
#define AAI_ADC_LEFT_FRAME_MSK \
	((uint32_t)((1 << AAI_ADC_LEFT_FRAME_WIDTH) - 1) << \
	 AAI_ADC_LEFT_FRAME_SHIFT)

/* IP name: AAI ; register name:DAC; bits field name: DAC_CYCLES_NUMS */
#define AAI_DAC_CYCLES_NUM_SHIFT	0
#define AAI_DAC_CYCLES_NUM_WIDTH	7
#define AAI_DAC_CYCLES_NUM_MSK \
	((uint32_t)((1 << AAI_DAC_CYCLES_NUM_WIDTH) - 1) << \
	 AAI_DAC_CYCLES_NUM_SHIFT)
/* IP name: AAI ; register name:DAC; bits field name: DAC_MATSUSHITA */
#define AAI_DAC_MATSUSHITA_SHIFT	7
#define AAI_DAC_MATSUSHITA_WIDTH	1
#define AAI_DAC_MATSUSHITA_MSK \
	((uint32_t)((1 << AAI_DAC_MATSUSHITA_WIDTH) - 1) << \
	 AAI_DAC_MATSUSHITA_SHIFT)
/* IP name: AAI ; register name:DAC; bits field name: DAC_LEFT_FRAME */
#define AAI_DAC_LEFT_FRAME_SHIFT	8
#define AAI_DAC_LEFT_FRAME_WIDTH	1
#define AAI_DAC_LEFT_FRAME_MSK \
	((uint32_t)((1 << AAI_DAC_LEFT_FRAME_WIDTH) - 1) << \
	 AAI_DAC_LEFT_FRAME_SHIFT)

/* IP name: AAI ; register name:AUX; bits field name: AUX_CYCLES_NUMS */
#define AAI_AUX_CYCLES_NUM_SHIFT	0
#define AAI_AUX_CYCLES_NUM_WIDTH	7
#define AAI_AUX_CYCLES_NUM_MSK \
	((uint32_t)((1 << AAI_AUX_CYCLES_NUM_WIDTH) - 1) << \
	 AAI_AUX_CYCLES_NUM_SHIFT)
/* IP name: AAI ; register name:AUX; bits field name: AUX_HALF_FRAME */
#define AAI_AUX_HALF_FRAME_SHIFT	24
#define AAI_AUX_HALF_FRAME_WIDTH	1
#define AAI_AUX_HALF_FRAME_MSK \
	((uint32_t)((1 << AAI_AUX_HALF_FRAME_WIDTH) - 1) << \
	 AAI_AUX_HALF_FRAME_SHIFT)

/* IP name: AAI ; register name: PCM ; bits field name: START_TWO */
#define AAI_PCM_START_TWO_SHIFT		0
#define AAI_PCM_START_TWO_WIDTH		8
#define AAI_PCM_START_TWO_MSK \
	((uint32_t)((1 << AAI_PCM_START_TWO_WIDTH) - 1) << \
	 AAI_PCM_START_TWO_SHIFT)
/* IP name: AAI ; register name: PCM ; bits field name: CYCLES_HIGH */
#define AAI_PCM_CYCLES_HIGH_SHIFT	8
#define AAI_PCM_CYCLES_HIGH_WIDTH	8
#define AAI_PCM_CYCLES_HIGH_MSK \
	((uint32_t)((1 << AAI_PCM_CYCLES_HIGH_WIDTH) - 1) << \
	 AAI_PCM_CYCLES_HIGH_SHIFT)
/* IP name: AAI ; register name: PCM ; bits field name: CYCLES_LOW */
#define AAI_PCM_CYCLES_LOW_SHIFT	16
#define AAI_PCM_CYCLES_LOW_WIDTH	9
#define AAI_PCM_CYCLES_LOW_MSK \
	((uint32_t)((1 << AAI_PCM_CYCLES_LOW_WIDTH) - 1) << \
	 AAI_PCM_CYCLES_LOW_SHIFT)
/* IP name: AAI ; register name: PCM ; bits field name: ON */
#define AAI_PCM_ON_SHIFT		25
#define AAI_PCM_ON_WIDTH		1
#define AAI_PCM_ON_MSK \
	((uint32_t)((1 << AAI_PCM_ON_WIDTH) - 1) << AAI_PCM_ON_SHIFT)
/* IP name: AAI ; register name: PCM ; bits field name: OKI */
#define AAI_PCM_OKI_SHIFT		26
#define AAI_PCM_OKI_WIDTH		1
#define AAI_PCM_OKI_MSK \
	((uint32_t)((1 << AAI_PCM_OKI_WIDTH) - 1) << AAI_PCM_OKI_SHIFT)
/* IP name: AAI ; register name: PCM ; bits field name: RUN_DUAL */
#define AAI_PCM_RUN_DUAL_SHIFT		27
#define AAI_PCM_RUN_DUAL_WIDTH		1
#define AAI_PCM_RUN_DUAL_MSK \
	((uint32_t)((1 << AAI_PCM_RUN_DUAL_WIDTH) - 1) << \
	 AAI_PCM_RUN_DUAL_SHIFT)
/* Values */
#define AAI_PCM_ON			(1 << AAI_PCM_ON_SHIFT)
#define AAI_PCM_OKI			(1 << AAI_PCM_OKI_SHIFT)
#define AAI_PCM_RUN_DUAL		(1 << AAI_PCM_RUN_DUAL_SHIFT)

/* IP name: AAI ; register name: VOICE_8K ; bits field name: PHONE */
#define AAI_VOICE_8K_PHONE_SHIFT	0
#define AAI_VOICE_8K_PHONE_WIDTH	1
#define AAI_VOICE_8K_PHONE_MSK \
	((uint32_t)((1 << AAI_VOICE_8K_PHONE_WIDTH) - 1) << \
	 AAI_VOICE_8K_PHONE_SHIFT)
/* IP name: AAI ; register name: VOICE_8K ; bits field name: SPEAKER */
#define AAI_VOICE_8K_SPEAKER_SHIFT	1
#define AAI_VOICE_8K_SPEAKER_WIDTH	1
#define AAI_VOICE_8K_SPEAKER_MSK \
	((uint32_t)((1 << AAI_VOICE_8K_SPEAKER_WIDTH) - 1) << \
	 AAI_VOICE_8K_SPEAKER_SHIFT)
/* Values */
#define AAI_VOICE_8K_PHONE		(1 << AAI_VOICE_8K_PHONE_SHIFT)
#define AAI_VOICE_8K_SPEAKER		(1 << AAI_VOICE_8K_SPEAKER_SHIFT)

#define AAI_MUS_FILTER5_LOW_FREQ	(1 << 0)


#define AAI_VOICE_SPEED_16_8		0x00810E35
#define AAI_VOICE_SPEED_22_11		0x00B1DAC7


#define AAI_SRC_FILTER5_0		(0 << 0)
#define AAI_SRC_FILTER5_1		(1 << 1)

#define AAI_SRC_MODE_QUADRI0		(1 << 0)
#define AAI_SRC_MODE_QUADRI1		(1 << 1)
#define AAI_SRC_MODE_OCTOPHONY		(1 << 2)
#define AAI_SRC_MODE_RUN0		(1 << 8)
#define AAI_SRC_MODE_RUN1		(1 << 9)

/*
 *  AAI FIFO Configuration sub-module registers address definition
 */
/* IP name: AAI ; register name: CFG_RX ; bits field name: START */
#define AAI_FIFOCFG_CFG_RX_START_SHIFT	0
#define AAI_FIFOCFG_CFG_RX_START_WIDTH	12
#define AAI_FIFOCFG_CFG_RX_START_MSK \
	((uint32_t)((1 << AAI_FIFOCFG_CFG_RX_START_WIDTH) - 1) << \
	 AAI_FIFOCFG_CFG_RX_START_SHIFT)
/* IP name: AAI ; register name: CFG_RX ; bits field name: DEPTH */
#define AAI_FIFOCFG_CFG_RX_DEPTH_SHIFT	16
#define AAI_FIFOCFG_CFG_RX_DEPTH_WIDTH	8
#define AAI_FIFOCFG_CFG_RX_DEPTH_MSK \
	((uint32_t)((1 << AAI_FIFOCFG_CFG_RX_DEPTH_WIDTH) - 1) << \
	 AAI_FIFOCFG_CFG_RX_DEPTH_SHIFT)
/* IP name: AAI ; register name: CFG_RX ; bits field name: CHAN_NUM */
#define AAI_FIFOCFG_CFG_RX_CHAN_NUM_SHIFT	24
#define AAI_FIFOCFG_CFG_RX_CHAN_NUM_WIDTH	6
#define AAI_FIFOCFG_CFG_RX_CHAN_NUM_MSK  \
	((uint32_t)((1 << AAI_FIFOCFG_CFG_RX_CHAN_NUM_WIDTH) - 1) << \
	 AAI_FIFOCFG_CFG_RX_CHAN_NUM_SHIFT)

/* IP name: AAI ; register name: CFG_TX ; bits field name: START */
#define AAI_FIFOCFG_CFG_TX_START_SHIFT	0
#define AAI_FIFOCFG_CFG_TX_START_WIDTH	12
#define AAI_FIFOCFG_CFG_TX_START_MSK \
	((uint32_t)((1 << AAI_FIFOCFG_CFG_TX_START_WIDTH) - 1) << \
	 AAI_FIFOCFG_CFG_TX_START_SHIFT)
/* IP name: AAI ; register name: CFG_TX ; bits field name: DEPTH */
#define AAI_FIFOCFG_CFG_TX_DEPTH_SHIFT	16
#define AAI_FIFOCFG_CFG_TX_DEPTH_WIDTH	8
#define AAI_FIFOCFG_CFG_TX_DEPTH_MSK \
	((uint32_t)((1 << AAI_FIFOCFG_CFG_TX_DEPTH_WIDTH) - 1) << \
	 AAI_FIFOCFG_CFG_TX_DEPTH_SHIFT)
/* IP name: AAI ; register name: CFG_TX ; bits field name: CHAN_NUM */
#define AAI_FIFOCFG_CFG_TX_CHAN_NUM_SHIFT	24
#define AAI_FIFOCFG_CFG_TX_CHAN_NUM_WIDTH	5
#define AAI_FIFOCFG_CFG_TX_CHAN_NUM_MSK \
	((uint32_t)((1 << AAI_FIFOCFG_CFG_TX_CHAN_NUM_WIDTH) - 1) << \
	 AAI_FIFOCFG_CFG_TX_CHAN_NUM_SHIFT)


/* IP name: AAI ; register name: SEL_RX ; bits field name: CHAN_ROUTE */
#define AAI_FIFOCFG_SEL_RX_CHAN_ROUTE_SHIFT	0
#define AAI_FIFOCFG_SEL_RX_CHAN_ROUTE_WIDTH	5
#define AAI_FIFOCFG_SEL_RX_CHAN_ROUTE_MSK \
	((uint32_t)((1 << AAI_FIFOCFG_SEL_RX_CHAN_ROUTE_WIDTH) - 1) << \
	 AAI_FIFOCFG_SEL_RX_CHAN_ROUTE_SHIFT)
/* IP name: AAI ; register name: SEL_RX ; bits field name: CHAN_ORDER */
#define AAI_FIFOCFG_SEL_RX_CHAN_ORDER_SHIFT	8
#define AAI_FIFOCFG_SEL_RX_CHAN_ORDER_WIDTH	6
#define AAI_FIFOCFG_SEL_RX_CHAN_ORDER_MSK \
	((uint32_t)((1 << AAI_FIFOCFG_SEL_RX_CHAN_ORDER_WIDTH) - 1) << \
	 AAI_FIFOCFG_SEL_RX_CHAN_ORDER_SHIFT)

/* IP name: AAI ; register name: SEL_TX ; bits field name: CHAN_ROUTE */
#define AAI_FIFOCFG_SEL_TX_CHAN_ROUTE_SHIFT	0
#define AAI_FIFOCFG_SEL_TX_CHAN_ROUTE_WIDTH	4
#define AAI_FIFOCFG_SEL_TX_CHAN_ROUTE_MSK \
	((uint32_t)((1 << AAI_FIFOCFG_SEL_TX_CHAN_ROUTE_WIDTH) - 1) << \
	 AAI_FIFOCFG_SEL_TX_CHAN_ROUTE_SHIFT)
/* IP name: AAI ; register name: SEL_TX ; bits field name: CHAN_ORDER */
#define AAI_FIFOCFG_SEL_TX_CHAN_ORDER_SHIFT	8
#define AAI_FIFOCFG_SEL_TX_CHAN_ORDER_WIDTH	5
#define AAI_FIFOCFG_SEL_TX_CHAN_ORDER_MSK \
	((uint32_t)((1 << AAI_FIFOCFG_SEL_TX_CHAN_ORDER_WIDTH) - 1) << \
	 AAI_FIFOCFG_SEL_TX_CHAN_ORDER_SHIFT)

/*
 *  AAI LBC Configuration sub-module registers address definition
 */
/* IP name: AAI ; register name: CKEN ; bits field name: MFRAME_MODE */
#define AAI_LBC_CKEN_MFRAME_SHIFT	17
#define AAI_LBC_CKEN_MFRAME_WIDTH	1
#define AAI_LBC_CKEN_MFRAME_MSK \
	((uint32_t)((1 << AAI_LBC_CKEN_MFRAME_WIDTH) - 1) << \
	 AAI_LBC_CKEN_MFRAME_SHIFT)
/* IP name: AAI ; register name: CKEN ; bits field name: HFRAME_MODE */
#define AAI_LBC_CKEN_HFRAME_SHIFT	18
#define AAI_LBC_CKEN_HFRAME_WIDTH	1
#define AAI_LBC_CKEN_HFRAME_MSK \
	((uint32_t)((1 << AAI_LBC_CKEN_HFRAME_WIDTH) - 1) << \
	 AAI_LBC_CKEN_HFRAME_SHIFT)
/* IP name: AAI ; register name: MFRAME ; bits field name: MFRAME_DIV */
#define AAI_LBC_MFRAME_MFRAME_DIV_LOW_SHIFT	0
#define AAI_LBC_MFRAME_MFRAME_DIV_LOW_WIDTH	16
#define AAI_LBC_MFRAME_MFRAME_DIV_LOW_MSK \
	((uint32_t)((1 << AAI_LBC_MFRAME_MFRAME_DIV_LOW_WIDTH) - 1) << \
	 AAI_LBC_MFRAME_MFRAME_DIV_LOW_SHIFT)
/* IP name: AAI ; register name: MFRAME ; bits field name: MFRAME_DIV */
#define AAI_LBC_MFRAME_MFRAME_DIV_HIGH_SHIFT	16
#define AAI_LBC_MFRAME_MFRAME_DIV_HIGH_WIDTH	16
#define AAI_LBC_MFRAME_MFRAME_DIV_HIGH_MSK \
	((uint32_t)((1 << AAI_LBC_MFRAME_MFRAME_DIV_HIGH_WIDTH) - 1) << \
	 AAI_LBC_MFRAME_MFRAME_DIV_HIGH_SHIFT)
/* IP name: AAI ; register name: HFRAME ; bits field name: HFRAME_DIV */
#define AAI_LBC_HFRAME_HFRAME_DIV_LOW_SHIFT	0
#define AAI_LBC_HFRAME_HFRAME_DIV_LOW_WIDTH	16
#define AAI_LBC_HFRAME_HFRAME_DIV_LOW_MSK \
	((uint32_t)((1 << AAI_LBC_HFRAME_HFRAME_DIV_LOW_WIDTH) - 1) << \
	 AAI_LBC_HFRAME_HFRAME_DIV_LOW_SHIFT)
/* IP name: AAI ; register name: HFRAME ; bits field name: HFRAME_DIV */
#define AAI_LBC_HFRAME_HFRAME_DIV_HIGH_SHIFT	16
#define AAI_LBC_HFRAME_HFRAME_DIV_HIGH_WIDTH	16
#define AAI_LBC_HFRAME_HFRAME_DIV_HIGH_MSK \
	((uint32_t)((1 << AAI_LBC_HFRAME_HFRAME_DIV_HIGH_WIDTH) - 1) << \
	 AAI_LBC_HFRAME_HFRAME_DIV_HIGH_SHIFT)

#endif

