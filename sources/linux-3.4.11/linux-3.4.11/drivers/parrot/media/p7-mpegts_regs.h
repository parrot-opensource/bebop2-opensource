#ifndef __P7_MPEGTS_REGS_H
#define __P7_MPEGTS_REGS_H

#define P7MPG_CORE_OFFSET(mpegts_id)	(0x4000+(mpegts_id)*0x8000)

#define P7MPG_CTRL                  0x000       // MPEGTS Control                          Register
#define P7MPG_SYNC_VAL              0x004       // MPEGTS SYNC byte value                  Register
#define P7MPG_SYNC_NUM              0x008       // MPEGTS Number of SYNC waited            Register
#define P7MPG_PKT_SIZE              0x00C       // MPEGTS Packet size                      Register
#define P7MPG_DATA_PIPE             0x010       // MPEGTS Timing clock delay               Register
#define P7MPG_STAT                  0x014       // MPEGTS Status                           Register
#define P7MPG_ITEN                  0x018       // MPEGTS Interrupt enable                 Register
#define P7MPG_FLUSH                 0x01C       // MPEGTS FIFO & DMA flush                 Register
#define P7MPG_ITACK                 0x020       // MPEGTS Interrupt Acknowledge            Register
#define P7MPG_RXFIFO_TH             0x024       // MPEGTS RX FIFO threshold                Register
#define P7MPG_RXCNT_TH              0x028       // MPEGTS RX Counter threshold             Register
#define P7MPG_RXCNT                 0x02C       // MPEGTS RX Counter Value                 Register
#define P7MPG_FIFO_RXLVL            0x030       // MPEGTS RX FIFO Level                    Register
#define P7MPG_DATA                  0x800       // MPEGTS Data                             Registers

// Control register
#define P7MPG_CTRL_ENABLE           (1 << 0)    // MPEGTS Enabled
#define P7MPG_CTRL_SYNC             (1 << 1)    // MPEGTS Synchronization signal enabled
#define P7MPG_CTRL_VALID            (1 << 2)    // MPEGTS Validation signal enabled
#define P7MPG_CTRL_LSB_FIRST        (1 << 3)    // MPEGTS LSB first
#define P7MPG_CTRL_FALL_SAMPLING    (1 << 4)    // MPEGTS Clock polarity selection
#define P7MPG_CTRL_DMAC_ENABLE      (1 << 5)    // MPEGTS DMA interface enabled

// Status register
#define P7MPG_STATUS_RX_EMPTY          (1 <<  0)   // MPEGTS Rx FIFO is empty
#define P7MPG_STATUS_RX_FULL           (1 <<  1)   // MPEGTS Rx FIFO is full
#define P7MPG_STATUS_RXCNT_TH_REACHED  (1 <<  2)   // MPEGTS FIFO level is high threshold
#define P7MPG_STATUS_RXFIFO_TH_REACHED (1 <<  3)   // MPEGTS FIFO byte counter if higher than threshold
#define P7MPG_STATUS_RXACCESS_ERR      (1 <<  4)   // MPEGTS Rx access error

/*
 * Input pad registers
 * _core: mpegts core id (0 - 1)
 */
#define P7MPG_SWB_IPAD_CLK(_core)	(0x260UL + (16UL * (_core)))
#define P7MPG_SWB_IPAD_DATA(_core)	(0x264UL + (16UL * (_core)))
#define P7MPG_SWB_IPAD_VALID(_core)	(0x268UL + (16UL * (_core)))
#define P7MPG_SWB_IPAD_SYNC(_core)	(0x26CUL + (16UL * (_core)))

#endif
