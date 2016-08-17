#ifndef __P7_SPIREGS_H
#define __P7_SPIREGS_H

#define P7SPI_SPEED_MAXDIV_R1   (0x03ffU)   /* 10 bits on R1        */
#define P7SPI_SPEED_MAXDIV_R23  (0xffffU)   /* 16 bits on R2 and R3 */

#define P7SPI_TSETUP_SHIFT		(0)
#define P7SPI_THOLD_SHIFT		(4)
#define P7SPI_TOFFCLK_SHIFT		(16)
#define P7SPI_TOFFSPI_SHIFT		(20)
#define P7SPI_TCAPTURE_DELAY_SHIFT	(24)
#define P7SPI_TCAPTURE_DELAY_MIN	0x3U

#define P7SPI_CORE_OFFSET(spi_id)	(0x1000*(spi_id))
#define P7SPI_COMMON_OFFSET		0xF000
#define P7SPI_COMMON_LEN		(0x25C + 1)

#define P7SPI_CTRL                0x000       // SPI Control                          Register
#define P7SPI_SPEED               0x004       // SPI Speed                            Register
#define P7SPI_TIMING              0x008       // SPI Timing                           Register
#define P7SPI_STATUS              0x00C       // SPI Status                           Register 
#define P7SPI_ITEN                0x010       // SPI Interrupt Enable                 Register 
#define P7SPI_ITACK               0x014       // SPI Interrupt Acknowledge            Register 
#define P7SPI_TH_RX               0x018       // SPI FIFO RX Threshold                Register 
#define P7SPI_TH_TX               0x01C       // SPI FIFO TX threshold                Register 
#define P7SPI_TH_RXCNT            0x020       // SPI FIFO RX Threshold Bytes Counter  Register
#define P7SPI_TH_TXCNT            0x024       // SPI FIFO TX Threshold Bytes Counter  Register
#define P7SPI_RXCNT               0x028       // SPI FIFO RX Threshold Bytes Counter  Register
#define P7SPI_TXCNT               0x02C       // SPI FIFO TX Threshold Bytes Counter  Register
#define P7SPI_RX_VALID_BYTES      0x030       // Number of valid bytes in a word read from rx fifo
#define P7SPI_FIFO_FLUSH          0x034       // Write-only register for rx/tx fifo flush
#define P7SPI_FIFO_RXLVL          0x038
#define P7SPI_FIFO_TXLVL          0x03c
#define P7SPI_INSTR               0x600       // SPI Instruction                      Registers (first address) 0x030 - 0x5FC
#define P7SPI_DATA                0x800       // SPI Data                             Registers (first address) 0x610 - 0x7FC

#define P7SPI_TH_RX_MAX           0xFF
#define P7SPI_CNT_MAX             (0xFFFFU)

/* FIFO Flush register SPI_FIFO_FLUSH */
#define P7SPI_FIFORX_FLUSH	(1 << 0)
#define P7SPI_FIFOTX_FLUSH	(1 << 1)
#define P7SPI_DMA_FLUSH         (1 << 2)
#define P7SPI_FIFO_FLUSH_MASK	0x7

// Status register
#define P7SPI_STATUS_END_TRANSM        (1 <<  0)   // End of transmission
#define P7SPI_STATUS_INSTR_FULL        (1 <<  1)   // 
#define P7SPI_STATUS_SPI_BUSY          (1 <<  4)   // SPI is busy
#define P7SPI_STATUS_RXEMPTY           (1 <<  8)   // Rx FIFO is empty
#define P7SPI_STATUS_RXFULL            (1 <<  9)   // Rx FIFO is full
#define P7SPI_STATUS_RX_TH_REACHED     (1 << 10)   // Rx FIFO level is high threshold
#define P7SPI_STATUS_RXCNT_TH_REACHED  (1 << 11)   // Rx FIFO byte counter if higher than treshold
#define P7SPI_STATUS_RXACCESS_ERROR    (1 << 12)   // Rx access error
#define P7SPI_STATUS_TXEMPTY           (1 << 16)   // Tx FIFO is empty
#define P7SPI_STATUS_TXFULL            (1 << 17)   // Tx FIFO is full
#define P7SPI_STATUS_TX_TH_REACHED     (1 << 18)   // Tx FIFO level is high threshold
#define P7SPI_STATUS_TXCNT_TH_REACHED  (1 << 19)   // Tx FIFO byte counter if higher than treshold
#define P7SPI_STATUS_TXACCESS_ERROR    (1 << 20)    // Tx access error

#define P7SPI_ITACK_RXCNT_ACK          (1 << 11)
#define P7SPI_ITACK_TXCNT_ACK          (1 << 19)

#define P7SPI_INSTR_SS_END_INSTR_HIGH  (1 << 21)
#define P7SPI_INSTR_SS_END_INSTR_LOW   (0 << 21)
#define P7SPI_INSTR_SPI_TMODE_SHIFT    22
#define P7SPI_INSTR_SPI_TMODE_SINGLE   (0 << P7SPI_INSTR_SPI_TMODE_SHIFT)
#define P7SPI_INSTR_SPI_TMODE_DUAL     (1 << P7SPI_INSTR_SPI_TMODE_SHIFT)
#define P7SPI_INSTR_SPI_TMODE_QUAD     (3 << P7SPI_INSTR_SPI_TMODE_SHIFT)
#define P7SPI_INSTR_SPI_TMODE_MASK     (3 << P7SPI_INSTR_SPI_TMODE_SHIFT)
#define P7SPI_INSTR_DMAE_ENABLE        (1 << 24)
#define P7SPI_INSTR_DMAE_DISABLE       (0 << 24)
#define P7SPI_INSTR_SS_END_BYTE_HIGH   (1 << 29)
#define P7SPI_INSTR_SS_END_BYTE_LOW    (0 << 29)
#define P7SPI_INSTR_RW_CLK_ONLY        (0 << 30)
#define P7SPI_INSTR_RW_WRITE           (1 << 30)
#define P7SPI_INSTR_RW_READ            (2 << 30)
#define P7SPI_INSTR_RW_FULL_DUPLEX     (3 << 30)
#define P7SPI_INSTR_LEN_MASK           (0xFFFFU)

// Control register
#define P7SPI_CTRL_ENABLE         (1 << 0)    // SPI Enabled
#define P7SPI_CTRL_DISABLE        (0 << 0)    // SPI Disabled
#define P7SPI_CTRL_CPHA           (1 << 1)    // Clock phase selection
#define P7SPI_CTRL_CPHAn          (0 << 1)    // Clock phase selection
#define P7SPI_CTRL_CPOL           (1 << 2)    // Clock polarity selection
#define P7SPI_CTRL_CPOLn          (0 << 2)    // Clock polarity selection
#define P7SPI_CTRL_MSTR           (1 << 3)    // Master / Slave selection
#define P7SPI_CTRL_SLAVE          (0 << 3)    // Master / Slave selection
#define P7SPI_CTRL_LSB            (1 << 4)    // Enable LSB first TX/RX data
#define P7SPI_CTRL_MSB            (0 << 4)    // Enable MSB first TX/RX data
#define P7SPI_CTRL_BITMSB         (1 << 5)    // LSB goes to MSB data pin (dual/quad modes only)
#define P7SPI_CTRL_BITLSB         (0 << 5)    // LSB goes to LSB data pin (dual/quad modes only)
#define P7SPI_CTRL_RWS_NOP        (0 << 8) 
#define P7SPI_CTRL_RWS_WO         (1 << 8) 
#define P7SPI_CTRL_RWS_RO         (2 << 8) 
#define P7SPI_CTRL_RWS_RW         (3 << 8) 
#define P7SPI_CTRL_SMODE_SHIFT    10
#define P7SPI_CTRL_SMODE_SINGLE   (0 << 10) 
#define P7SPI_CTRL_SMODE_DUAL     (1 << 10) 
#define P7SPI_CTRL_SMODE_QUAD     (3 << 10) 
#define P7SPI_CTRL_SSIGNS_IGNORED (1 << 12)
#define P7SPI_CTRL_SSIGNS_NORMAL  (0 << 12)
#define P7SPI_CTRL_DMAES_ENABLED  (1 << 13)
#define P7SPI_CTRL_RX_FREE_RUNS   (1 << 14)
#define P7SPI_CTRL_TX_FREE_RUNS   (1 << 15)
#define P7SPI_CTRL_DMAES_DISABLED (0 << 13)
#define P7SPI_CTRL_MISOSEL_DATA0  (0 << 16) 
#define P7SPI_CTRL_MISOSEL_DATA1  (1 << 16) 
#define P7SPI_CTRL_MISOSEL_DATA2  (2 << 16) 
#define P7SPI_CTRL_MISOSEL_DATA3  (3 << 16) 
#define P7SPI_CTRL_MOSISEL_DATA0  (0 << 18) 
#define P7SPI_CTRL_MOSISEL_DATA1  (1 << 18) 
#define P7SPI_CTRL_MOSISEL_DATA2  (2 << 18) 
#define P7SPI_CTRL_MOSISEL_DATA3  (3 << 18) 

#define P7SPI_ITEN_END_TRANSM          (1 <<  0)    // Enable bit for IT: End of transmission
#define P7SPI_ITEN_RX_FULL             (1 <<  9)    // Enable bit for IT: End of transmission
#define P7SPI_ITEN_RX_TH_REACHED       (1 << 10)    // Enable bit for IT: Rx FIFO level is high threshold
#define P7SPI_ITEN_RXCNT_TH_REACHED    (1 << 11)    // Enable bit for IT: Rx FIFO byte counter if higher than treshold
#define P7SPI_ITEN_RXACCESS_ERROR      (1 << 12)    // Enable bit for IT: Rx access error
#define P7SPI_ITEN_TXEMPTY             (1 << 16)    // Enable bit for IT: Tx FIFO is empty
#define P7SPI_ITEN_TX_TH_REACHED       (1 << 18)    // Enable bit for IT: Tx FIFO level is high threshold
#define P7SPI_ITEN_TXCNT_TH_REACHED    (1 << 19)    // Enable bit for IT: Tx FIFO byte counter if higher than treshold
#define P7SPI_ITEN_TXACCESS_ERROR      (1 << 20)    // Enable bit for IT: Tx access error

/*************************************************************
 * Internal multiplexing handling: switchbox and pads control
 *************************************************************/

/*
 * Output pad registers
 * _pad: output pad number (0 - 15)
 */
#define P7SPI_SWB_OPAD(_pad)	(0x100UL + (4UL * (_pad)))

/*
 * Output pad functions
 * _core: core id (0 - 3)
 */
#define P7SPI_SWB_OPAD_CLK(_core)   (((_core)*6) + 0)
#define P7SPI_SWB_OPAD_SS(_core)	(((_core)*6) + 1)
#define P7SPI_SWB_OPAD_DATA0(_core)	(((_core)*6) + 2)
#define P7SPI_SWB_OPAD_DATA1(_core)	(((_core)*6) + 3)
#define P7SPI_SWB_OPAD_DATA2(_core)	(((_core)*6) + 4)
#define P7SPI_SWB_OPAD_DATA3(_core)	(((_core)*6) + 5)
#define P7SPI_SWB_OPAD_LOW			0x18
#define P7SPI_SWB_OPAD_HIGH			0x19
#define P7SPI_SWB_OPAD_Z			0x1a

#define P7SPI_SWB_IPAD_BASE             (0x200UL)

/*
 * Input pad registers
 * _core: spi core id (0 - 3)
 */
#define P7SPI_SWB_IPAD_CLK(_core)	(0x200UL + (24UL * (_core)))
#define P7SPI_SWB_IPAD_SS(_core)	(0x204UL + (24UL * (_core)))
#define P7SPI_SWB_IPAD_DATA0(_core)	(0x208UL + (24UL * (_core)))
#define P7SPI_SWB_IPAD_DATA1(_core)	(0x20CUL + (24UL * (_core)))
#define P7SPI_SWB_IPAD_DATA2(_core)	(0x210UL + (24UL * (_core)))
#define P7SPI_SWB_IPAD_DATA3(_core)	(0x214UL + (24UL * (_core)))

/*
 * Input pad functions
 * _pad: input pad number (0 - 15)
 */
#define P7SPI_SWB_IPAD(_pad)    (_pad)
#define P7SPI_SWB_IPAD_LOW		0x10
#define P7SPI_SWB_IPAD_HIGH		0x11

/***************************
 * Internal FIFOs handling.
 ***************************/

#define P7SPI_MEM           (0x000UL)

#define P7SPI_MEM_0W	    (0U)
#define P7SPI_MEM_16W	    (1U)
#define P7SPI_MEM_32W	    (2U)
#define P7SPI_MEM_64W	    (3U)
#define P7SPI_MEM_MASK      (0x3U)
#define P7SPI_MEM_BITLEN    (2U)
#define P7SPI_MEM_WCNT_MAX_R1  (0xffU)
#define P7SPI_MEM_WCNT_MAX_R23 (0xfffU)
#define P7SPI_MEM_MAX_WORDS 128

#endif
