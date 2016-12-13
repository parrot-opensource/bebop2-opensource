/**
 * @file linux/include/asm-arm/arch-parrot/regs-uart.h
 * @brief Device I/O - Description of Parrot5/5+/6 serial hardware
 *
 * Copyright (C) 2005,2006,2007 Parrot S.A.
 *
 * @author     yves.lemoine@parrot.com
 * @author     ivan.djelic@parrot.com
 * @date       2005-09-28
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
 *
 */
#ifndef __ARCH_ARM_PARROT5_UART_REGS_H
#define __ARCH_ARM_PARROT5_UART_REGS_H

#define PXUART_NR   8

#define _UART_TRX                   0x00         /* UART RX/TX Buffers */
#define _UART_IRX                   0x04         /* Rx IRQ Enable Register */
#define _UART_ITX                   0x08         /* Tx IRQ Enable Register */
#define _UART_RSTBRK                0x0C         /* Reset Break Register */
#define _UART_SETBRK                0x10         /* Set Break Register */
#define _UART_LCR                   0x14         /* Line Command Register */
#define _UART_SPD                   0x18         /* Speed Register */
#define _UART_STATUS                0x1C         /* Status Register */

/* Registers bitwise definitions */

/* Receive register */
#define UART_TRX_INVALID            (1 << 8)     /* Byte invalidity flag */
#define UART_TRX_OVERRUN            (1 << 9)     /* Over-run bit */
#define UART_TRX_PARITY_ERROR       (1 << 10)    /* Parity error bit */
#define UART_TRX_FRAME_ERROR        (1 << 11)    /* Frame error bit */
#define UART_TRX_RXBREAK            (1 << 12)    /* Break detection bit */
#define UART_TRX_TXCOLLISION        (1 << 13)    /* TX flag collision */

/* Error mask - ignore invalid and collision flags */
#define UART_TRX_ANY_ERROR      \
	(UART_TRX_OVERRUN     | \
	 UART_TRX_PARITY_ERROR| \
	 UART_TRX_FRAME_ERROR | \
	 UART_TRX_RXBREAK)

/* Receive Interrupt Enable Register */
#define UART_IRX_RIM                1            /* RX Interrupt mask bit */

/* Transmit Interrupt Enable Register */
#define UART_ITX_TIM                1            /* TX Interrupt mask bit */

/* Line Command Register */

#define UART_LCR_LONG_BREAK_2       (0 << 12)
#define UART_LCR_LONG_BREAK_4       (1 << 12)
#define UART_LCR_LONG_BREAK_8       (2 << 12)
#define UART_LCR_LONG_BREAK_16      (3 << 12)
#define UART_LCR_LONG_BREAK_32      (4 << 12)
#define UART_LCR_LONG_BREAK_64      (5 << 12)
#define UART_LCR_LONG_BREAK_128     (6 << 12)
#define UART_LCR_LONG_BREAK_256     (7 << 12)
#define UART_LCR_TSTZERO            (1 << 11)    /* Collision test on level 0*/
#define UART_LCR_SINGLEWIRE         (1 << 10)    /* Single Wire mode. */
#define UART_LCR_TXFIFO             (1 << 9)     /* TX FIFO utilization */
#define UART_LCR_RXFIFO             (1 << 8)     /* RX FIFO utilization */
#define UART_LCR_FIFO_THRESHOLD_1   (0 << 6)     /* RX FIFO threshold */
#define UART_LCR_FIFO_THRESHOLD_8   (1 << 6)
#define UART_LCR_FIFO_THRESHOLD_16  (2 << 6)
#define UART_LCR_FIFO_THRESHOLD_24  (3 << 6)
#define UART_LCR_CTSMODE            (1 << 5)     /* CTS validation bit */
#define UART_LCR_STICKY             (1 << 4)     /* Fixed Parity bit */
#define UART_LCR_EVEN               (1 << 3)     /* Even Parity bit */
#define UART_LCR_PARITY             (1 << 2)     /* Parity enable bit */
#define UART_LCR_TWOSTOP            (1 << 1)     /* 2 stop bits enable bit */
#define UART_LCR_EIGHTBITS          1            /* 8 bits mode enable bit */

/* Parrot5+ LCR extensions */
#define UART_LCR_RTS_CTRL              (1 << 15) /* auto(0)/manual(1) nRTS */
#define UART_LCR_FIFO_RX_THRESHOLD_1   (0 << 6)  /* Receive FIFO threshold */
#define UART_LCR_FIFO_RX_THRESHOLD_16  (1 << 6)
#define UART_LCR_FIFO_RX_THRESHOLD_32  (2 << 6)
#define UART_LCR_FIFO_RX_THRESHOLD_48  (3 << 6)
#define UART_LCR_FIFO_TX_THRESHOLD_0   (0 << 16) /* Transmit FIFO threshold */
#define UART_LCR_FIFO_TX_THRESHOLD_4   (1 << 16)
#define UART_LCR_FIFO_TX_THRESHOLD_8   (2 << 16)
#define UART_LCR_FIFO_TX_THRESHOLD_16  (3 << 16)

/* Status Register */
#define UART_STATUS_COLLISION       (1 << 10)  /* Collision flag bit */
#define UART_STATUS_STOP_BREAK      (1 << 9)   /* Stop break flag bit */
#define UART_STATUS_TX_BREAK        (1 << 8)   /* Transmit Break flag bit */
#define UART_STATUS_TXFILLED        (1 << 7)   /* FIFO TX Filled flag bit */
#define UART_STATUS_TXEMPTY         (1 << 6)   /* TX empty flag bit */
#define UART_STATUS_ITTX            (1 << 5)   /* TX Interrupt flag bit */
#define UART_STATUS_CTSN            (1 << 4)   /* CTSN input read-back value */
#define UART_STATUS_LONG_BREAK      (1 << 3)   /* Long brk detection flag bit*/
#define UART_STATUS_RTSN            (1 << 2)   /* RSTN output read-back value*/
#define UART_STATUS_RXFILLED        (1 << 1)   /* FIFO RX Filled flag bit */
#define UART_STATUS_ITRX            1          /* Receive Interrupt flag bit */

#define BAUD_921600                 (UART_CLOCK/921600)
#define BAUD_460800                 (UART_CLOCK/460800)
#define BAUD_115200                 (UART_CLOCK/115200)
#define BAUD_38400                  (UART_CLOCK/ 38400)
#define BAUD_19200                  (UART_CLOCK/ 19200)
#define BAUD_9600                   (UART_CLOCK/  9600)

/* define alternative values for CPUCLK = 156 MHz mode */
#define BAUD_921600_156MHZ          ((3*UART_CLOCK/4)/921600)
#define BAUD_460800_156MHZ          ((3*UART_CLOCK/4)/460800)
#define BAUD_115200_156MHZ          ((3*UART_CLOCK/4)/115200)
#define BAUD_38400_156MHZ           ((3*UART_CLOCK/4)/ 38400)
#define BAUD_19200_156MHZ           ((3*UART_CLOCK/4)/ 19200)
#define BAUD_9600_156MHZ            ((3*UART_CLOCK/4)/  9600)

/* FIFO sizes */
#define UART_TX_FIFO_SIZE_P5        (16)
#define UART_RX_FIFO_SIZE_P5        (32)

/* Parrot5+ fifos are twice deeper */
#define UART_TX_FIFO_SIZE_P5P       (32)
#define UART_RX_FIFO_SIZE_P5P       (64)

#endif /* __ARCH_ARM_PARROT5_UART_REGS_H */
