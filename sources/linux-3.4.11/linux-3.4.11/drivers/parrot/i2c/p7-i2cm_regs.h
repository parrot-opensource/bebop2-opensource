/**
 * @file linux/include/asm-arm/arch-parrot/i2c.h
 *
 * @brief Parrot I2C registers
 *
 * Copyright (C) 2008 Parrot S.A.
 *
 * @author     david.guilloteau@parrot.com
 * @date       2008-08-28
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
 */

#ifndef __ASM_ARCH_PARROT_REGS_I2C_P5_H
#define __ASM_ARCH_PARROT_REGS_I2C_P5_H

/* Parrot I2C registers */

#define I2CM_TRANSMIT       0x00      /* Transmit register */
#define I2CM_RECEIVE        0x04      /* Receive register */
#define I2CM_ITEN           0x08      /* Interrupt Enable register */
#define I2CM_ITACK          0x0C      /* Interrupt Ack register */
#define I2CM_STATUS         0x10      /* Status register */
#define I2CM_COMMAND        0x14      /* Command register */
#define I2CM_PRESCALE       0x18      /* Prescale register */
#define I2CM_HIGH_PRESCALE  0x1c      /* High speed prescale register */
#define I2CM_WFIFO          0x20      /* Command and write FIFO register */
#define I2CM_RFIFO          0x24      /* Read FIFO register */

#ifdef CONFIG_ARCH_VEXPRESS
#define I2CM_VEXPRESS_SELECT 0x28     /* Vexpress I2CM selection */
#endif

/* FIFO depths */
#define I2CM_WFIFO_DEPTH    8
#define I2CM_RFIFO_DEPTH    128

/* Registers bitwise definitions */

/* I2CM Status Register */
#define I2CM_STATUS_IF          (1 << 0)  /* Interrupt Flag bit */
#define I2CM_STATUS_TIP         (1 << 1)  /* Transfer In Progress bit */
#define I2CM_STATUS_AL          (1 << 2)  /* Arbitration Lost bit */
#define I2CM_STATUS_BUSY        (1 << 3)  /* I2C Busy bit */
#define I2CM_STATUS_RXACK       (1 << 4)  /* Receive Acknowledge from slave */
#define I2CM_STATUS_BAD_CMD     (1 << 8)  /* Bad command */
#define I2CM_STATUS_BAD_WR      (1 << 9)  /* Bad write command */
#define I2CM_STATUS_BAD_RD      (1 << 10) /* Bad read command */
#define I2CM_STATUS_BAD_STOP    (1 << 11) /* Bad stop command */
#define I2CM_STATUS_BAD_START   (1 << 12) /* Bad start command */
#define I2CM_STATUS_BAD_SCL     (1 << 13) /* ? */
#define I2CM_STATUS_BAD_ACK     (1 << 14) /* Received ACK is not the expected one */
#define I2CM_STATUS_CMD_EMPTY   (1 << 16) /* Command FIFO empty interrupt flag */
#define I2CM_STATUS_CMD_FULL    (1 << 17) /* Command FIFO full flag */
#define I2CM_STATUS_FIFO_FULL   (1 << 18) /* Read FIFO full interrupt flag */
#define I2CM_STATUS_FIFO_FILLED (1 << 19) /* Read fifo not empty flag */

#define I2CM_STATUS_ERROR_MASK  0xff00

/* I2CM Command Register */
#define I2CM_COMMAND_ACK        (0<< 0)     // Acknowledge bit
#define I2CM_COMMAND_NACK       (1<< 0)     // Acknowledge bit
#define I2CM_COMMAND_WR         (1<< 1)     // Write bit command
#define I2CM_COMMAND_RD         (1<< 2)     // Read  bit command
#define I2CM_COMMAND_STOP       (1<< 3)     // Stop  bit command
#define I2CM_COMMAND_START      (1<< 4)     // Start bit command
#define I2CM_COMMAND_HIGH_SPEED (1<< 5)     // Transfer data at high speed
#define I2CM_COMMAND_MASK_SCL   (1<< 6)     // Mask the SCL input
#define I2CM_COMMAND_MASK_SDA   (1<< 7)     // Mask the SDA input
#define I2CM_COMMAND_BUS_CLEAR  (1<< 8)     // Try to clear the I2C bus

/* I2CM Interrupt Enable register */

/* The normal interrupt is set at the end of each individual Command, at the end
 * of the last FIFO Command, at the end of a repeated Read Command, or when the
 * Read FIFO is full. */
#define I2CM_ITEN_ITEN      (1 << 0)  /* Interrupt enabled */
#define I2CM_ITEN_FIFO_ITEN (1 << 1)  /* Interrupt generated when repeat read
                                       * has been completed */
#define I2CM_ITEN_DEBUG_EN  (1 << 2)  /* Debug interrupt enabled for individual
                                       * command */
#define I2CM_ITEN_DBG_FIFO  (1 << 3)  /* Debug interrupt enabled for FIFO
                                       * command, the corresponding interrupt
                                       * clears the command FIFO */

/* I2CM Interrupt Acknowledge register */
#define I2CM_ITACK_ACK      (1 << 0)  /* clear pending interrupt not caused by
                                       * FIFO full read*/

/* I2CM Transmit register */
#define I2CM_TRANSMIT_WRITE (0 << 0)  /* for addresses: RW bit : writing to
                                       * slave */
#define I2CM_TRANSMIT_READ  (1 << 0)  /* for addresses: RW bit : reading from
                                       * slave */

// I2CM Command and Write FIFO
#define I2CM_WFIFO_ACK          (0<< 8)     // Acknowledge bit: 0: ACK - 1: NACK
#define I2CM_WFIFO_NACK         (1<< 8)     // Acknowledge bit: 0: ACK - 1: NACK
#define I2CM_WFIFO_WR           (1<< 9)     // Write  bit command
#define I2CM_WFIFO_RD           (1<<10)     // Read bit command
#define I2CM_WFIFO_STOP         (1<<11)     // Stop  bit command
#define I2CM_WFIFO_START        (1<<12)     // Start bit command
#define I2CM_WFIFO_HIGH_SPEED   (1<<13)     // Transfer data at high speed
#define I2CM_WFIFO_LAST_NACK    (1<<14)     // Last acknowledge bit
#define I2CM_WFIFO_LAST_STOP    (1<<15)     // Stop command at the last read
#define I2CM_WFIFO_RX_CLEAR     (1<<16)     // Clear the Read FIFO before executing the rest of the command

// I2CM Read FIFO
#define I2CM_RFIFO_EMPTY        (1<< 8)     // Empty bit: 0: RX_BYTE is valid - 1: RX_BYTE is not valid (FIFO is empty)

#endif /* __ASM_ARCH_PARROT_I2C_H */
