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
#ifndef __CAST_REGS_H
#define __CAST_REGS_H

/* Special Function Registers of NANDFLASH-CTRL IP */
#define CAST_COMMAND		0x00 	/* Controller commands register */
#define CAST_CONTROL		0x04 	/* The main configurations register */
#define CAST_STATUS		0x08 	/* The controller status register */
#define CAST_INT_MASK		0x0C 	/* Interrupts mask register */
#define CAST_INT_STATUS		0x10 	/* Interrupts status register */
#define CAST_ECC_CTRL		0x14 	/* ECC module status register */
#define CAST_ECC_OFFSET		0x18 	/* ECC offset in the spare area */
#define CAST_ADDR0_1		0x24	/* The most significant part
					 * of address register 0
					 */
#define CAST_ADDR0_0		0x1C 	/*
					 * The least significant part
	        			 * of address register 0
					 */
#define CAST_ADDR1_1		0x28
#define CAST_ADDR1_0		0x20
#define CAST_SPARE_SIZE		0x30	/*
					 * Actual value of the user
	        			 * data size in spare area
					 */
#define CAST_PROTECT		0x38	/* write/erase protect */
#define CAST_LOOKUP_EN		0x40	/*
					 * Enables Look-up registers during
	        			 * NAND Flash memory addressing
					 */
#define CAST_LOOKUP0		0x44	/* LUTs are used for address remapping
	        			 * Essential for Bad-Block management
					 */
#define CAST_LOOKUP1		0x48
#define CAST_LOOKUP2		0x4C
#define CAST_LOOKUP3		0x50
#define CAST_LOOKUP4		0x54
#define CAST_LOOKUP5		0x58
#define CAST_LOOKUP6		0x5C
#define CAST_LOOKUP7		0x60
#define CAST_DMA_ADDR		0x64	/* DMA module base address */
#define CAST_DMA_CNT		0x68	/* DMA module counter initial value */
#define CAST_DMA_CTRL		0x6C	/* DMA module control register */
#define CAST_MEM_CTRL		0x80	/* Memory device control register */
#define CAST_DATA_SIZE		0x84	/* Custom data size value */
#define CAST_READ_STATUS	0x88	/* READ_STATUS cmd output value */
#define CAST_TIME_SEQ_0		0x8C	/* Timing configuration */
#define CAST_TIME_SEQ_1		0xBC
#define CAST_TIMINGS_ASYN	0x90
#define CAST_TIMINGS_SYN	0x94
#define CAST_FIFO_DATA		0x98	/* FIFO module interface */
#define CAST_DMA_ADDR_OFFSET	0xA0	/* DMA module address offset */
#define	CAST_FIFO_INIT		0xB0	/* FIFO module control register */
#define CAST_GENERIC_SEQ_CTRL	0xB4	/* generic sequence register */
#define CAST_FIFO_STATE		0xB8	/* FIFO module status */
#define CAST_MLUN		0xC0	/* MLUN register */

/* Control register bit coding */
/* Address cycle_0, CTRL_REG[2:0] */
#define CAST_CTRL_ADDR_CYCLE0(_n)	(_n)
#define CAST_CTRL_ADDR_CYCLE0_MASK	7
/* Address cycle_1, CTRL_REG[20:18] */
#define CAST_CTRL_ADDR_CYCLE1(_n)	((_n) << 18)
#define CAST_CTRL_ADDR_CYCLE1_MASK	(7 << 18)
#define CAST_CTRL_SPARE_EN	(1 << 3)
#define CAST_CTRL_INT_EN	(1 << 4)
#define CAST_CTRL_ECC_EN	(1 << 5)
#define CAST_CTRL_BLOCK_32	(0 << 6) /* Pages/Block, CTRL_REG[7:6] */
#define CAST_CTRL_BLOCK_64	(1 << 6)
#define CAST_CTRL_BLOCK_128	(2 << 6)
#define CAST_CTRL_BLOCK_256	(3 << 6)
#define CAST_CTRL_PAGE_256	(0 << 8) /* Bytes/Page, CTRL_REG[10:8] */
#define CAST_CTRL_PAGE_512	(1 << 8)
#define CAST_CTRL_PAGE_1K	(2 << 8)
#define CAST_CTRL_PAGE_2K	(3 << 8)
#define CAST_CTRL_PAGE_4K	(4 << 8)
#define CAST_CTRL_PAGE_8K	(5 << 8)
#define CAST_CTRL_PAGE_16K	(6 << 8)
#define CAST_CTRL_PAGE_0	(7 << 8)
#define CAST_CTRL_CUSTOM_SIZE_EN	(1 << 11)
#define CAST_CTRL_IO_WIDTH_8	(0 << 12)
#define CAST_CTRL_IO_WIDTH_16	(1 << 12)
#define CAST_CTRL_SYNC_MODE     (1 << 15)

/* Command register bit coding */
#define SHIFT_CMD_2		24
#define SHIFT_CMD_1		16
#define SHIFT_CMD_0		8
#define CAST_CMD_ADDR_SEL0	(0 << 7)	/* ADDR0 */
#define CAST_CMD_ADDR_SEL1	(1 << 7)	/* ADDR1 */
#define CAST_CMD_INPUT_SEL_SIU	(0 << 6)
#define CAST_CMD_INPUT_SEL_DMA	(1 << 6)
#define CAST_CMD_SEQ_0		0b000000
#define CAST_CMD_SEQ_1		0b100001
#define CAST_CMD_SEQ_2		0b100010
#define CAST_CMD_SEQ_3		0b000011
#define CAST_CMD_SEQ_4		0b100100
#define CAST_CMD_SEQ_5		0b100101
#define CAST_CMD_SEQ_6		0b100110
#define CAST_CMD_SEQ_7		0b100111
#define CAST_CMD_SEQ_8		0b001000
#define CAST_CMD_SEQ_9		0b101001
#define CAST_CMD_SEQ_10		0b101010
#define CAST_CMD_SEQ_11		0b101011
#define CAST_CMD_SEQ_12		0b001100
#define CAST_CMD_SEQ_13		0b001101
#define CAST_CMD_SEQ_14		0b001110
#define CAST_CMD_SEQ_15		0b101111
#define CAST_CMD_SEQ_16		0b110000
#define CAST_CMD_SEQ_17		0b010101
#define CAST_CMD_SEQ_18		0b110010
#define CAST_CMD_SEQ_19		0b010011

/* ECC_CTRL register bit coding */
/* BCH32 ECC */
#define CAST_ECC_CTRL_ERR_THRLD (8)	/* ecc error threshold ECC_CTRL[13:8] */
#define CAST_ECC_CTRL_CAP_2	(0 << 4)	/* ECC capacity ECC_CTRL[7:4] */
#define CAST_ECC_CTRL_CAP_4	(1 << 4)
#define CAST_ECC_CTRL_CAP_6	(2 << 4)
#define CAST_ECC_CTRL_CAP_8	(3 << 4)
#define CAST_ECC_CTRL_CAP_10	(4 << 4)
#define CAST_ECC_CTRL_CAP_12	(5 << 4)
#define CAST_ECC_CTRL_CAP_14	(6 << 4)
#define CAST_ECC_CTRL_CAP_16	(7 << 4)
#define CAST_ECC_CTRL_CAP_18	(8 << 4)
#define CAST_ECC_CTRL_CAP_20	(9 << 4)
#define CAST_ECC_CTRL_CAP_22	(10 << 4)
#define CAST_ECC_CTRL_CAP_24	(11 << 4)
#define CAST_ECC_CTRL_CAP_26	(12 << 4)
#define CAST_ECC_CTRL_CAP_28	(13 << 4)
#define CAST_ECC_CTRL_CAP_30	(14 << 4)
#define CAST_ECC_CTRL_CAP_32	(15 << 4)
#define CAST_ECC_CTRL_CAP(_n)	((((_n)>>1)-1)<<4)

#define CAST_ECC_CTRL_BLOCK_256	(0 << 14)
#define CAST_ECC_CTRL_BLOCK_512	(1 << 14)
#define CAST_ECC_CTRL_BLOCK_1K	(2 << 14)

/* Set when err_bits>ERR_THRESHOLD */
#define CAST_ECC_CTRL_ERR_OVER	(1 << 2)
/* Set when uncorrectable err occurs */
#define CAST_ECC_CTRL_ERR_UNCORRECT (1 << 1)
/* Set when correctable err occurs */
#define CAST_ECC_CTRL_ERR_CORRECT (1 << 0)

/* MEM_CTRL Register bit coding */
/* MEM_CE[2:0], selected mem */
#define CAST_MEM_CTRL_CE_MASK	7
/* device(_n) WP line state */
#define CAST_MEM_CTRL_WP(_n)	(8 << (_n))

/* STATUS Register bit coding */
/* Controller 0: ready; 1: busy */
#define CAST_STAT_CTRL_STAT	(1 << 8)
/* Device 0: busy; 1: ready */
#define CAST_STAT_MEMX_ST(_n)	(1 << (_n))

/* READ STATUS Register bit coding */
#define CAST_READ_STAT_MASK	0xFF		/* Status bit in last 8 bits */

/* FIFO INIT Register bit coding */
/* Set this bit flashes the fifo */
#define CAST_FIFO_INIT_FLASH	(1 << 0)

/* FIFO STATE Register bit coding */
#define CAST_FIFO_STATE_STAT	(1 << 0)

/* Interrupt registers */
#define CAST_INT_MASK_DMA_READY_EN		(1 << 13)
#define CAST_INT_MASK_FIFO_ERROR_EN		(1 << 12)
#define CAST_INT_MASK_MEMx_RDY_INT_EN(_n)	(16 << (_n))
#define CAST_INT_MASK_ECC_TRSH_ERR_EN		(1 << 3)
#define CAST_INT_MASK_ECC_FATAL_ERR_EN		(1 << 2)
#define CAST_INT_MASK_CMD_END_INT_EN		(1 << 1)
#define CAST_INT_MASK_PROT_INT_EN		(1 << 0)

#define CAST_INT_STATUS_DMA_READY_FL		(1 << 13)
#define CAST_INT_STATUS_FIFO_ERROR_FL		(1 << 12)
#define CAST_INT_STATUS_MEMx_RDY_INT_FL(_n)	(16 << (_n))
#define CAST_INT_STATUS_ECC_TRSH_ERR_FL		(1 << 3)
#define CAST_INT_STATUS_ECC_FATAL_ERR_FL	(1 << 2)
#define CAST_INT_STATUS_CMD_END_INT_FL		(1 << 1)
#define CAST_INT_STATUS_PROT_INT_FL		(1 << 0)

/* DMA CTRL register mapping */
#define CAST_DMA_CTRL_DMA_START			(1 << 7)
#define CAST_DMA_CTRL_DMA_DIR_READ		(1 << 6) /* CNTL's view */
#define CAST_DMA_CTRL_DMA_DIR_WRITE		(0 << 6) /* CNTL's view */
#define CAST_DMA_CTRL_DMA_MODE_SFR		(0 << 5)
#define CAST_DMA_CTRL_DMA_MODE_S_G		(1 << 5)
#define CAST_DMA_CTRL_DMA_BURST_MASK		(7 << 2)
#define CAST_DMA_CTRL_DMA_ERR_FLAG		(1 << 1)
#define CAST_DMA_CTRL_DMA_READY			(1 << 0)

/* incremental burst (burst length is four) */
#define DMA_BURST_INC_BURST_LENGTH_4      	(0 << 2)
/* Stream burst (address const) */
#define DMA_BURST_STREAM_BURST            	(1 << 2)
/* single transfer (address increment) */
#define DMA_BURST_SINGLE_TRANSFER         	(2 << 2)
/* burst of unspecified length (address increment) */
#define DMA_BURST_UNSPEC_LENGTH                	(3 << 2)
/* incremental burst (burst length eight) */
#define DMA_BURST_INC_BURST_LENGTH_8      	(4 << 2)
/* incremental burst (burst length sixteen) */
#define DMA_BURST_INC_BURST_LENGTH_16     	(5 << 2)

#endif /* __CAST_REGS_H */
