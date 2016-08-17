/**
 * linux/arch/arm/mach-parrot7/board-p7dev7aai.c - P7Dev aai daughter board
 *							implementation
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author: Samir Ammenouche <samir.ammenouche@parrot.com>
 * date:    04-Apr-2013
 *
 * This file is released under the GPL
 */

#include "board.h"
#include "aai.h"
#include "pinctrl.h"

static unsigned long p7dev_aai_conf[] = {
	P7CTL_DRV_CFG(4),
};

static struct pinctrl_map board_p7dev_aai_pins[] __initdata = {
	P7_INIT_PINMAP(P7_AAI_00),
	P7_INIT_PINCFG(P7_AAI_00, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_01),
	P7_INIT_PINCFG(P7_AAI_01, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_02),
	P7_INIT_PINCFG(P7_AAI_02, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_03),
	P7_INIT_PINCFG(P7_AAI_03, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_04),
	P7_INIT_PINCFG(P7_AAI_04, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_05),
	P7_INIT_PINCFG(P7_AAI_05, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_06),
	P7_INIT_PINCFG(P7_AAI_06, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_07),
	P7_INIT_PINCFG(P7_AAI_07, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_08),
	P7_INIT_PINCFG(P7_AAI_08, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_09),
	P7_INIT_PINCFG(P7_AAI_09, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_10),
	P7_INIT_PINCFG(P7_AAI_10, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_11),
	P7_INIT_PINCFG(P7_AAI_11, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_12),
	P7_INIT_PINCFG(P7_AAI_12, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_13),
	P7_INIT_PINCFG(P7_AAI_13, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_14),
	P7_INIT_PINCFG(P7_AAI_14, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_15),
	P7_INIT_PINCFG(P7_AAI_15, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_16),
	P7_INIT_PINCFG(P7_AAI_16, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_17),
	P7_INIT_PINCFG(P7_AAI_17, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_18),
	P7_INIT_PINCFG(P7_AAI_18, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_19),
	P7_INIT_PINCFG(P7_AAI_19, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_20),
	P7_INIT_PINCFG(P7_AAI_20, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_21),
	P7_INIT_PINCFG(P7_AAI_21, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_22),
	P7_INIT_PINCFG(P7_AAI_22, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_23),
	P7_INIT_PINCFG(P7_AAI_23, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_24),
	P7_INIT_PINCFG(P7_AAI_24, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_25),
	P7_INIT_PINCFG(P7_AAI_25, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_26),
	P7_INIT_PINCFG(P7_AAI_26, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_27),
	P7_INIT_PINCFG(P7_AAI_27, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_28),
	P7_INIT_PINCFG(P7_AAI_28, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_29),
	P7_INIT_PINCFG(P7_AAI_29, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_30),
	P7_INIT_PINCFG(P7_AAI_30, p7dev_aai_conf),
};

void __init aaidb_probe(struct p7_board const *board)
{
	p7dev_aai_pin_ptr = &(board_p7dev_aai_pins[0]);
	p7_aai_pin_sz = ARRAY_SIZE(board_p7dev_aai_pins);
}
