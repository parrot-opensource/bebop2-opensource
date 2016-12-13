/**
 * linux/arch/arm/mach-parrot7/board.c - Parrot7 boards platform
 *                                       implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    30-Oct-2012
 *
 * This file is released under the GPL
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/string.h>
#include "board.h"

#define P7_BOARD_OPTS_SZ    128
static char p7_board_opts[P7_BOARD_OPTS_SZ] __initdata;

void __init p7_reserve_brdmem(struct p7_board* const* boards,
                              size_t count)
{
	unsigned int    b;

	for (b = 0; b < count; b++) {
		struct p7_board const* const    brd = boards[b];

		if (brd->rsv_mem && p7_brd_there(brd)) {
			pr_debug("P7: reserving memory for daughterboard %s\n",
				 brd->name);
			brd->rsv_mem(brd);
		}
	}
}

void __init p7_probe_brd(struct p7_board* const* boards,
                         size_t count)
{
	unsigned int    b;

	for (b = 0; b < count; b++) {
		struct p7_board const* const    brd = boards[b];

		if (p7_brd_there(brd)) {
			pr_debug("P7: probing daughterboard %s\n", brd->name);
			brd->probe(brd);
		}
	}
}

void __init p7_setup_brd(struct p7_board* const* boards,
                         size_t count,
                         char* options)
{
	char*   opt;

	/*
	 * Duplicate options to prevent from corrupting command line and just to be
	 * safe.
	 */
	if (! strlcpy(p7_board_opts, options, ARRAY_SIZE(p7_board_opts)))
		return;

	/* Remove leading and trailing white spaces. */
	opt = strim(p7_board_opts);

	/* Now parse each comma separated token and init boards accordingly. */
	while (opt && *opt != '\0') {
		char*           o = strchr(opt, ',');
		unsigned long   rev = 0;
		char*           r;
		unsigned int    b;

		if (o)
			*(o++) = '\0';

		/* Get revision number if any... */
		r = strchr(opt, '-');
		if (r) {
			*(r++) = '\0';
			if (strict_strtoul(r, 0, &rev)) {
				rev = 0;
				pr_warn("p7: board \"%s\" specified with invalid "
				        "revision number \"%s\"\n",
				        opt,
				        r);
			}
		}

		/* Match with known boards. */
		for (b = 0; b < count; b++) {
			struct p7_board* const  brd = boards[b];

			if (! strcmp(brd->name, opt)) {
				/* This is the board we are interested in. */
				struct p7_board_conn* const conn = brd->conn;

				if (conn->plugged)
					/* Connector already reserved by another board. */
					break;

				/* Probe succeeded: reserve assigned connector. */
				pr_info("p7: found %s board v%lu on connector J%u\n",
				        brd->name,
				        rev,
				        conn->id);

				brd->rev = rev;
				conn->plugged = brd;
				break;
			}
		}

		if (b == count)
			pr_warn("p7: unknown board \"%s\" specified\n", opt);

		/* Jump to next board token. */
		opt = o;
	}
}
