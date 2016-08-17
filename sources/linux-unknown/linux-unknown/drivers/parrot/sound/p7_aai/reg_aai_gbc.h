/*-======================================================================-
 * Parrot S.A. Confidential Property
 *
 * Project      : Parrot 7
 * File name    : reg_aai_gbc.h
 * Author       : Marc Tamisier
 *
 * ----------------------------------------------------------------------
 * Purpose : AAI registers definition
 */
#ifndef __REG_AAI_GBC_H__
#define __REG_AAI_GBC_H__

#include "reg_aai.h"

#define AAI_OFFSET 0xfc020000

/*
 * General GBC definition
 * block_id
 */
#define GBC_AAI                     0xFFFC

/* module_id */
#define GBC_AAI0                    0

/* clock_id */
#define GBC_AAI0_MCLK               0
#define GBC_AAI0_MFRAME             1
#define GBC_AAI0_HFRAME             2
#define GBC_AAI0_P1CLK              8
#define GBC_AAI0_P2CLK              9
#define GBC_AAI0_CLK                16

/* reset_id */
#define GBC_AAI0_RESET              0
#define GBC_AAI0_RESET_SPDIF        1


/*
 * Specific GBC definition
 */
#define AAI_GBC_OFFSET              0xF000
#define AAI_GBC_OFFSET_R3              0xFF000

/*
 * AAI clocks enables and resets
 * prefer to use GBC function (using block_id, module_id, clock_id)
 */
#define AAI_GBC_CLKEN               (AAI_GBC_OFFSET + 0xFFC)
#define AAI_GBC_CLKEN_R3            (AAI_GBC_OFFSET_R3 + 0xFFC)
/*#define AAI_GBC_RESET               (AAI_GBC_OFFSET + 0xFF8)*/

/* i2s clocks configurations */
#define AAI_GBC_MCLK                (AAI_GBC_OFFSET + 0x7FC)
#define AAI_GBC_MFRAME              (AAI_GBC_OFFSET + 0x7F8)

/* pcm clocks configurations */
#define AAI_GBC_P1CLK               (AAI_GBC_OFFSET + 0x7EC)
#define AAI_GBC_P1FRAME             (AAI_GBC_OFFSET + 0x7E8)
#define AAI_GBC_P2CLK               (AAI_GBC_OFFSET + 0x7E4)
#define AAI_GBC_P2FRAME             (AAI_GBC_OFFSET + 0x7E0)

/* spdif clock configurations */
#define AAI_GBC_SPDIFCLK            (AAI_GBC_OFFSET + 0x7DC)
#define AAI_GBC_SPDIFCLK_R3         (AAI_GBC_OFFSET_R3 + 0xDFC)

/* Pads configurations */
#define AAI_GBC_PADIN(pad_id)       (AAI_GBC_OFFSET + (0x000 + 0x4*(pad_id)))
#define AAI_GBC_PADOUT(pad_id)      (AAI_GBC_OFFSET + (0x100 + 0x4*(pad_id)))


/*
 * Specific GBC definition
 */
#define AAI_LBC_OFFSET              0xF000

/*
 * AAI clocks enables and resets
 * prefer to use GBC function (using block_id, module_id, clock_id)
 */
#define AAI_LBC_CLKEN               (AAI_LBC_OFFSET + 0xFFC)
/*#define AAI_GBC_RESET               (AAI_GBC_OFFSET + 0xFF8)*/

/* i2s clocks configurations */
#define AAI_LBC_MCLK                (AAI_LBC_OFFSET + 0x7FC)
#define AAI_LBC_HFRAME              (AAI_LBC_OFFSET + 0x7F4)
#define AAI_LBC_MFRAME              (AAI_LBC_OFFSET + 0x7F8)

/* pcm clocks configurations */
#define AAI_LBC_P1CLK               (AAI_LBC_OFFSET + 0x7EC)
#define AAI_LBC_P1FRAME             (AAI_LBC_OFFSET + 0x7E8)
#define AAI_LBC_P2CLK               (AAI_LBC_OFFSET + 0x7E4)
#define AAI_LBC_P2FRAME             (AAI_LBC_OFFSET + 0x7E0)


/* Pads configurations */
#define AAI_LBC_PADIN(pad_id)       (AAI_LBC_OFFSET + (0x000 + 0x4*(pad_id)))
#define AAI_LBC_PADOUT(pad_id)      (AAI_LBC_OFFSET + (0x100 + 0x4*(pad_id)))


/*
 * Specific GBC Value
 */
#define PxCLK_DIV_2MHZ              (0x0184 << 0)
#define PxCLK_DIV_1MHZ              (0x030A << 0)
#define PxCLK_DIV_512KHZ            (0x05F1 << 0)
#define PxCLK_DIV_256KHZ            (0x0BE5 << 0)

#endif

