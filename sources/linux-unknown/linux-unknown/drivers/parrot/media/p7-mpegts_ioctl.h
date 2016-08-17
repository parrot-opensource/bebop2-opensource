/*
 * linux/drivers/parrot/media/p7-mpegts_ioctl.h
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * @author: Damien Riegel <damien.riegel.ext@parrot.com>
 * @author: Jeremie Samuel <jeremie.samuel.ext@parrot.com>
 *
 * This file is released under the GPL
 */

#ifndef P7_MPEGTS_IOCTL_H_
#define P7_MPEGTS_IOCTL_H_

#include <linux/ioctl.h>

struct p7mpg_settings {
	unsigned char               fall_sampling:1;
	unsigned char               lsb:1;
	unsigned char               valid_en:1;
	unsigned char               sync_en:1;
	unsigned char               sync_val:8;
	unsigned int                sync_num:10;
	unsigned int                pkt_size:16;
	unsigned char               timing_dl:3;
	unsigned int                pkt_by_blk;
	unsigned char               blk_by_rb;
};

enum p7mpg_block_state {
	P7MPG_BLOCK_INVALID = 0,
	P7MPG_BLOCK_BUSY    = 1,
	P7MPG_BLOCK_FREE    = 2,
	P7MPG_BLOCK_VALID   = 3,
};

struct p7mpg_metadata {
	size_t                  blk_size;
	size_t                  pkt_size;
	unsigned char           blk_by_rb;
	unsigned long           blks_states;
};

#define P7MPG_IOC_MAGIC      0xBB

#define P7MPG_IOC_SET_CONFIG        _IOW(P7MPG_IOC_MAGIC,  1, struct p7mpg_settings *)
#define P7MPG_IOC_GET_CONFIG        _IOR(P7MPG_IOC_MAGIC,  2, struct p7mpg_settings *)
#define P7MPG_IOC_START             _IO(P7MPG_IOC_MAGIC,   3)
#define P7MPG_IOC_STOP              _IO(P7MPG_IOC_MAGIC,   4)

#define P7MPG_IOC_MAXNR		4

#endif /* P7_MPEGTS_IOCTL_H_ */
