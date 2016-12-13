/**
 * linux/arch/arm/mach-parrot7/board.h - Parrot7 boards platform
 *                                       interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    30-Oct-2012
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_BOARD_H
#define _ARCH_PARROT7_BOARD_H

struct p7_board;
struct p7_board_conn;

#ifdef CONFIG_ARCH_PARROT7_BOARD

#include <linux/init.h>

typedef void (p7_init_brd_fn)(struct p7_board const*);

struct p7_board {
       char const*                 name;
       unsigned int                rev;
       struct p7_board_conn*       conn;
       p7_init_brd_fn*             probe;
       p7_init_brd_fn*             rsv_mem;
};

#define P7_INIT_BOARD(_name, _conn, _probe, _rsv_mem)   \
	{                                                   \
		.name       = _name,                            \
		.conn       = _conn,                            \
		.probe      = _probe,                           \
		.rsv_mem    = _rsv_mem                          \
	}

#define P7_DEFINE_BOARD(_name, _idstr, _conn, _probe, _rsv_mem) \
	static struct p7_board _name __initdata =                   \
		P7_INIT_BOARD(_idstr, _conn, _probe, _rsv_mem)

static inline void p7_define_board(struct p7_board *board,
				   char const* const name,
				   struct p7_board_conn* const conn,
				   p7_init_brd_fn* const probe,
				   p7_init_brd_fn* const rsv_mem)
{
	if (name)    board->name = name;
	if (conn)    board->conn = conn;
	if (probe)   board->probe = probe;
	if (rsv_mem) board->rsv_mem = rsv_mem;
}

struct p7_board_conn {
	unsigned int const      id;
	struct p7_board const*  plugged;
};

#define P7_INIT_BOARD_CONN(_id) \
	{                           \
		.id         = _id,      \
		.plugged    = NULL      \
	}

#define P7_DEFINE_BOARD_CONN(_name, _id)                                    \
	static struct p7_board_conn _name __initdata __attribute__((unused)) =  \
		P7_INIT_BOARD_CONN(_id)

#define P7_BOARD_OPTS(_boards, _opt)                            \
	static int __init __setup_ ## _boards(char* options)        \
	{                                                           \
		p7_setup_brd(_boards, ARRAY_SIZE(_boards), options);    \
		return 0;                                               \
	}                                                           \
	early_param(_opt, __setup_ ## _boards)

extern void p7_setup_brd(struct p7_board* const*, size_t, char*) __init;
extern void p7_reserve_brdmem(struct p7_board* const*, size_t) __init;
extern void p7_probe_brd(struct p7_board* const*, size_t) __init;

static inline bool p7_brd_there(struct p7_board const* board)
{
	if (board->conn)
		return board->conn->plugged == board;
	else
		return false;
}

#ifdef CONFIG_ARCH_PARROT7_P7DEV_SND_AAI
extern p7_init_brd_fn aaidb_probe __init;
extern struct pinctrl_map   *p7dev_aai_pin_ptr __initdata;
extern unsigned long        p7_aai_pin_sz;
#endif

#ifdef CONFIG_ARCH_PARROT7_P7DEV_NOR
extern p7_init_brd_fn nordb_probe __init;
#endif

#ifdef CONFIG_ARCH_PARROT7_P7DEV_QUAD_NOR
extern p7_init_brd_fn spiquadb_probe __init;
#endif

#ifdef CONFIG_ARCH_PARROT7_P7DEV_FPGA
extern p7_init_brd_fn fpgadb_probe __init;
extern p7_init_brd_fn fpgadb_rsvmem __init;
#endif

#ifdef CONFIG_ARCH_PARROT7_P7DEV_RNB6
extern p7_init_brd_fn rnb6db_probe __init;
extern p7_init_brd_fn rnb6db_rsvmem __init;
#endif

#ifdef CONFIG_ARCH_PARROT7_P7DEV_HDMI
extern p7_init_brd_fn hdmidb_probe __init;
extern p7_init_brd_fn hdmidb_rsvmem __init;
#endif

#ifdef CONFIG_ARCH_PARROT7_P7DEV_SPI
extern p7_init_brd_fn spidb_probe __init;
extern p7_init_brd_fn spimdb_probe __init;
extern p7_init_brd_fn spisdb_probe __init;
#endif

#ifdef CONFIG_ARCH_PARROT7_P7DEV_CAMERA
extern p7_init_brd_fn cameradb_probe __init;
#endif

#ifdef CONFIG_ARCH_PARROT7_P7DEV_GALILEO2
extern p7_init_brd_fn galileo2db_probe __init;
extern p7_init_brd_fn galileo2db_rsvmem __init;
#endif

#ifdef CONFIG_ARCH_PARROT7_P7DEV_SICILIA
extern p7_init_brd_fn siciliadb_probe __init;
#endif

#ifdef CONFIG_ARCH_PARROT7_P7DEV_TUNER
extern p7_init_brd_fn tunerdb_probe __init;
extern p7_init_brd_fn tunerdb_rsvmem __init;
#endif

#else   /* ! CONFIG_ARCH_PARROT7_BOARD */

#define P7_INIT_BOARD(_name, _conn, _probe)
#define P7_DEFINE_BOARD(_name, _idstr, _conn, _probe)
#define P7_INIT_BOARD_CONN(_id)
#define P7_DEFINE_BOARD_CONN(_name, _id)
#define P7_BOARD_OPTS(_name, _opt)

static inline void p7_setup_brd(struct p7_board * const *b, size_t s, char *n)
{
	return;
}

static inline void p7_reserve_brdmem(struct p7_board* const*b, size_t s)
{
	return;
}

static inline void p7_probe_brd(struct p7_board* const*b , size_t s)
{
	return;
}

static inline bool p7_brd_there(struct p7_board const* board)
{
	return false;
}

#endif  /* CONFIG_ARCH_PARROT7_BOARD */

#endif  /* _ARCH_PARROT7_BOARD_H */
