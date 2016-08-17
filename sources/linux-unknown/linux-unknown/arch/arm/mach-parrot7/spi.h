/**
 * linux/arch/arm/mach-parrot7/spi.h - Parrot7 SPI controller platform
 *                                     interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    28-Jun-2012
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_SPI_H
#define _ARCH_PARROT7_SPI_H

struct p7spi_plat_data;
struct p7spi_swb;
struct spi_board_info;
struct pinctrl_map;

#if defined(CONFIG_SPI_PARROT7) || \
    defined(CONFIG_SPI_PARROT7_MODULE)

#include <linux/init.h>

extern void p7_init_spi(void) __init;

#else  /* defined(CONFIG_SPI_PARROT7) || \
          defined(CONFIG_SPI_PARROT7_MODULE) */

#define p7_init_spi()

#endif /* defined(CONFIG_SPI_PARROT7) || \
          defined(CONFIG_SPI_PARROT7_MODULE) */

#if defined(CONFIG_SPI_MASTER_PARROT7) || \
    defined(CONFIG_SPI_MASTER_PARROT7_MODULE)

extern void p7_init_spim(int, struct pinctrl_map*, size_t,
                         struct p7spi_swb const * const) __init;

/**
 * P7_INIT_SPIM_SLAVE() - Init a SPI slave device (connected to master
 *                        controller).
 *
 * @_mod:   name of module drive this slave
 * @_pdata: platform data assigned to this slave
 * @_cdata: master controller platform data to use with this slave
 * @_hz:    maximum frequency this slave may work at
 * @_mode:  clock phase / polarity this slave may work with (one of
 *          %SPI_MODE_0, %SPI_MODE_1, %SPI_MODE_2, %SPI_MODE_3, or a
 *          combination of %SPI_CPHA and / or %SPI_CPOL flags)
 */
#define P7_INIT_SPIM_SLAVE(_mod, _pdata, _cdata, _hz, _mode)\
	{                                                       \
		.modalias           = _mod,                         \
		.platform_data      = _pdata,                       \
		.controller_data    = _cdata,                       \
		.irq                = -1,                           \
		.max_speed_hz       = _hz,                          \
		.chip_select        = 0,                            \
		.mode               = _mode                         \
	}

/**
 * P7_DECLARE_SPIM_SLAVE() - Declare a SPI slave device (connected to master
 *                           controller).
 *
 * @_name:  slave variable name
 * @_mod:   name of module drive this slave
 * @_pdata: platform data assigned to this slave
 * @_cdata: master controller platform data to use with this slave
 * @_hz:    maximum frequency this slave may work at
 * @_mode:  clock phase / polarity this slave may work with (one of
 *          %SPI_MODE_0, %SPI_MODE_1, %SPI_MODE_2, %SPI_MODE_3, or a
 *          combination of %SPI_CPHA and / or %SPI_CPOL flags)
 */
#define P7_DECLARE_SPIM_SLAVE(_name, _mod, _pdata, _cdata, _hz, _mode)  \
	struct spi_board_info _name __initdata = P7_INIT_SPIM_SLAVE(_mod,   \
	                                                            _pdata, \
	                                                            _cdata, \
	                                                            _hz,    \
	                                                            _mode)

extern int p7_init_spim_slave(int, struct spi_board_info*) __init;

#else  /* defined(CONFIG_SPI_MASTER_PARROT7) || \
          defined(CONFIG_SPI_MASTER_PARROT7_MODULE) */

static inline void p7_init_spim(int				 i,
                                struct pinctrl_map		*m,
                                size_t				 s,
                                struct p7spi_swb const * const	 p)
{
	return;
}

static inline int p7_init_spim_slave(int i, struct spi_board_info *s)
{
	return -ENOSYS;
}

#define P7_DECLARE_SPIM_SLAVE(_name, _mod, _pdata, _cdata, _hz, _mode)  \
	struct spi_board_info _name __initdata = P7_INIT_SPIM_SLAVE(_mod,   \
	                                                            _pdata, \
	                                                            _cdata, \
	                                                            _hz,    \
	                                                            _mode)
/**
 * P7_INIT_SPIM_SLAVE() - Init a SPI slave device (connected to master
 *                        controller).
 *
 * @_mod:   name of module drive this slave
 * @_pdata: platform data assigned to this slave
 * @_cdata: master controller platform data to use with this slave
 * @_hz:    maximum frequency this slave may work at
 * @_mode:  clock phase / polarity this slave may work with (one of
 *          %SPI_MODE_0, %SPI_MODE_1, %SPI_MODE_2, %SPI_MODE_3, or a
 *          combination of %SPI_CPHA and / or %SPI_CPOL flags)
 */
#define P7_INIT_SPIM_SLAVE(_mod, _pdata, _cdata, _hz, _mode)\
	{                                                       \
		.modalias           = _mod,                         \
		.platform_data      = _pdata,                       \
		.controller_data    = _cdata,                       \
		.irq                = -1,                           \
		.max_speed_hz       = _hz,                          \
		.chip_select        = 0,                            \
		.mode               = _mode                         \
	}

#endif /* defined(CONFIG_SPI_MASTER_PARROT7) || \
          defined(CONFIG_SPI_MASTER_PARROT7_MODULE) */

#define P7_DECLARE_SPIS_MASTER(_name, _mod, _pdata, _cdata, _hz, _mode) \
	struct spi_board_info _name __initdata = {                          \
		.modalias           = _mod,                                     \
		.platform_data      = _pdata,                                   \
		.controller_data    = _cdata,                                   \
		.irq                = -1,                                       \
		.max_speed_hz       = _hz,                                      \
		.chip_select        = 0,                                        \
		.mode               = _mode                                     \
	}

#if defined(CONFIG_SPI_SLAVE_PARROT7) || \
    defined(CONFIG_SPI_SLAVE_PARROT7_MODULE)

extern void p7_init_spis(int, struct pinctrl_map*, size_t,
                         struct p7spi_swb const * const) __init;

/**
 * P7_DECLARE_SPIS_MASTER() - Declare a SPI master device (connected to slave
 *                            controller).
 *
 * @_name:  master variable name
 * @_mod:   name of module drive this master
 * @_pdata: platform data assigned to this master
 * @_cdata: slave controller platform data to use with this master
 * @_hz:    maximum frequency this master may work at
 * @_mode:  clock phase / polarity this master may work with (one of
 *          %SPI_MODE_0, %SPI_MODE_1, %SPI_MODE_2, %SPI_MODE_3, or a
 *          combination of %SPI_CPHA and / or %SPI_CPOL flags)
 */

extern int  p7_init_spis_master(int, struct spi_board_info*) __init;

#else  /* defined(CONFIG_SPI_SLAVE_PARROT7) || \
          defined(CONFIG_SPI_SLAVE_PARROT7_MODULE) */

static inline void p7_init_spis(int				 i,
                                struct pinctrl_map		*m,
                                size_t				 s,
                                struct p7spi_swb const * const	 p)
{
	return;
}

static inline int p7_init_spis_master(int i, struct spi_board_info *s)
{
	return -ENOSYS;
}

#endif /* defined(CONFIG_SPI_SLAVE_PARROT7) || \
          defined(CONFIG_SPI_SLAVE_PARROT7_MODULE) */

#endif
