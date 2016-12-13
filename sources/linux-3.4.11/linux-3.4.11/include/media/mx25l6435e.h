/**
 * Copyright (C) 2013 Parrot S.A.
 * @file mx25l6435e.h
 * @author n.laclau
 * @date 26 June 2013
 */


#ifndef MX25L6435E_H_
#define MX25L6435E_H_

#define MX25L6435E_READ     0x03 /* normal read */
#define MX25L6435E_FREAD    0x0B /* fast read data */
#define MX25L6435E_RDSFDP   0x5A /* Read SFDP */
#define MX25L6435E_2READ    0xBB /* 2 x I/O read command */
#define MX25L6435E_DREAD    0x3B /* 1I/2O read command */
#define MX25L6435E_W4READ   0xE7 /* ... */
#define MX25L6435E_4READ    0xEB /* 4 x I/O read command */
#define MX25L6435E_QREAD    0x6B /* ... */
#define MX25L6435E_WREN     0x06 /* write enable */
#define MX25L6435E_WRDI     0x04 /* write disable */
#define MX25L6435E_RDSR     0x05 /* read status register */
#define MX25L6435E_RDCR     0x15 /* read configuration register */
#define MX25L6435E_4PP      0x38 /* write status / configuration register */
#define MX25L6435E_SE       0x20 /* Sector erase */
#define MX25L6435E_BE32K    0x52 /* Block erase: 32Ko */
#define MX25L6435E_BE64K    0xD8 /* Block erase: 64Ko */
#define MX25L6435E_CE_1     0x60 /* Chip erase, or */
#define MX25L6435E_CE_2     0xC7 /* Chip erase */
#define MX25L6435E_PP       0x02 /* Page program */
#define MX25L6435E_CP       0xAD /* Continuous program */
#define MX25L6435E_DP       0xB9 /* Deep power down */
#define MX25L6435E_RDP      0xAB /* release from deep power down */
#define MX25L6435E_RDID     0x9F /* Read identification */
#define MX25L6435E_RES      0xAB /* Read electronic ID */
#define MX25L6435E_REMS     0x90 /* Read electronic manufacturer & device ID */
#define MX25L6435E_REMS2    0xEF /* Read electronic manufacturer & device ID */
#define MX25L6435E_REMS4    0xDF /* Read electronic manufacturer & device ID */
#define MX25L6435E_ENSO     0xB1 /* Enter secured OTP */
#define MX25L6435E_EXSO     0xC1 /* Exit secured OTP */
#define MX25L6435E_RDSCUR   0x2B /* Read security register */
#define MX25L6435E_WRSCUR   0x2F /* Write security register */
#define MX25L6435E_SBLK     0x36 /* Single block lock */
#define MX25L6435E_SBULK    0x39 /* Single block unlock */
#define MX25L6435E_RDBLOCK  0x3C /* Block protect read */
#define MX25L6435E_GBLK     0x7E /* Gang block lock */
#define MX25L6435E_GBULK    0x98 /* Gang block unlock */
#define MX25L6435E_NOP      0x00 /* No operation */
#define MX25L6435E_RSTEN    0x66 /* Reset enable */
#define MX25L6435E_RST      0x99 /* Reset memory */
#define MX25L6435E_WPSEL    0x68 /* Write protect selection */
#define MX25L6435E_ESRY     0x70 /* Enable SO to output RY/BY# */
#define MX25L6435E_DSRY     0x80 /* Disable SO to output RY/BY# */
#define MX25L6435E_RLZ_RENH 0xFF /* Release Read Enhanced */

#endif /* MX25L6435E_H_ */
