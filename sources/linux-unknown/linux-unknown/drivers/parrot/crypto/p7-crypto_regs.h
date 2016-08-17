/**
********************************************************************************
* @file p7-crypto_regs.h
* @brief P7 Analogic to Digital Converter driver
*
* Copyright (C) 2013 Parrot S.A.
*
* @author Karl Leplat <karl.leplat@parrot.com>
* @date 2013-09-19
********************************************************************************
*/
#ifndef __REG_P7CA_H__
#define __REG_P7CA_H__

#define P7CA_REG_CTRL_STATUS			0x00
#define P7CA_REG_IRQ_STATUS			0x04
#define P7CA_REG_FIFO_DATA_OUTPUT_FILL_LEVEL	0x08
#define P7CA_REG_FIFO_DATA_INPUT_FILL_LEVEL	0x0c
#define P7CA_REG_DMA_WRITE_COUNT_DOWN		0x10
#define P7CA_REG_DMA_READ_COUNT_DOWN		0x14

#define P7CA_REG_DESC_CTRL(_id_)		(0x100 + 16*(_id_))
#define P7CA_REG_DESC_SIZE(_id_)		(0x104 + 16*(_id_))
#define P7CA_REG_DESC_SOURCE(_id_)		(0x108 + 16*(_id_))
#define P7CA_REG_DESC_DESTINATION(_id_)		(0x10c + 16*(_id_))

#define P7CA_REG_KEY(_id_)			(0x200 + 32*(_id_))
#define P7CA_REG_IV(_id_)			(0x300 + 16*(_id_))

#define P7CA_REG_IRQ_MEM			0x400
#define P7CA_REG_IRQ_EN				0x404

#define P7CA_IRQ_EN_ENABLE			1
#define P7CA_IRQ_MEM_CLEAR			1

#define P7CA_CTRL_STATUS_CURRENT_DESC		(1 << 0)
#define P7CA_CTRL_STATUS_NB_OF_ACTIVE_DESC	(1 << 8)
#define P7CA_CTRL_STATUS_ACTIVE_DESC_INT_THRES	(1 << 16)

#define P7CA_IRQ_STATUS_EN(_id_)		(1 << (_id_))
#define P7CA_IRQ_STATUS_THRESH			(1 << 16)

#define P7CA_DESC_CTRL_CIPH_ALG_IDX		1
#define P7CA_DESC_CTRL_CIPH_MODE_IDX		4
#define P7CA_DESC_CTRL_DECODE			(1 << 0)
#define P7CA_DESC_CTRL_NEW_KEY_SET		(1 << 7)
#define P7CA_DESC_CTRL_INT_ENABLE		(1 << 11)
#define P7CA_DESC_CTRL_DESC_VALID		(1 << 12)
#define P7CA_DESC_CTRL_DESC_DEST_ENDIAN		(1 << 13)
#define P7CA_DESC_CTRL_DESC_SRC_ENDIAN		(1 << 14)

#define P7CA_DESC_CTRL_CIPH_ALG_DES		(0x00 << P7CA_DESC_CTRL_CIPH_ALG_IDX)
#define P7CA_DESC_CTRL_CIPH_ALG_TDEA		(0x01 << P7CA_DESC_CTRL_CIPH_ALG_IDX)
#define P7CA_DESC_CTRL_CIPH_ALG_AES_128		(0x02 << P7CA_DESC_CTRL_CIPH_ALG_IDX)
#define P7CA_DESC_CTRL_CIPH_ALG_AES_192		(0x03 << P7CA_DESC_CTRL_CIPH_ALG_IDX)
#define P7CA_DESC_CTRL_CIPH_ALG_AES_256		(0x04 << P7CA_DESC_CTRL_CIPH_ALG_IDX)

#define P7CA_DESC_CTRL_CIPH_MODE_ECB		(0x00 << P7CA_DESC_CTRL_CIPH_MODE_IDX)
#define P7CA_DESC_CTRL_CIPH_MODE_CBC		(0x01 << P7CA_DESC_CTRL_CIPH_MODE_IDX)
#define P7CA_DESC_CTRL_CIPH_MODE_CTR		(0x02 << P7CA_DESC_CTRL_CIPH_MODE_IDX)
#define P7CA_DESC_CTRL_CIPH_MODE_OFB		(0x03 << P7CA_DESC_CTRL_CIPH_MODE_IDX)
#define P7CA_DESC_CTRL_CIPH_MODE_F8		(0x04 << P7CA_DESC_CTRL_CIPH_MODE_IDX)

#define P7CA_DESC_CTRL_KEY_INDEX(x) ((x) << 8)

#define CRYPTO_CTRL_AND_STATUS_NB_OF_ACTIVE_DESC_WIDTH            5
#define CRYPTO_CTRL_AND_STATUS_NB_OF_ACTIVE_DESC_SHIFT            8
#define CRYPTO_CTRL_AND_STATUS_NB_OF_ACTIVE_DESC_MASK             ((uint32_t) ((1<<CRYPTO_CTRL_AND_STATUS_NB_OF_ACTIVE_DESC_WIDTH) - 1) << CRYPTO_CTRL_AND_STATUS_NB_OF_ACTIVE_DESC_SHIFT)
#endif /* __REG_P7CA_H__ */

