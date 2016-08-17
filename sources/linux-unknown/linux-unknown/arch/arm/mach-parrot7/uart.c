/**
 * linux/arch/arm/mach-parrot7/uart.c - Parrot7 serial port platform
 *                                      implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    13-Jun-2012
 *
 * This file is released under the GPL
 */

#include <linux/platform_device.h>
#include <linux/clkdev.h>
#include <serial/px-uart.h>
#include <mach/irqs.h>
#include <mach/p7.h>
#include "common.h"
#include "clock.h"

static struct resource p7_uart0_res[] = {
	[0] = {
		.start = P7_UART0,
		.end   = P7_UART0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM
	},
	[1] = {
		.start = P7_UART0_IRQ,
		.end   = P7_UART0_IRQ,
		.flags = IORESOURCE_IRQ
	}
};

static struct resource p7_uart1_res[] = {
	[0] = {
		.start = P7_UART1,
		.end   = P7_UART1 + SZ_4K - 1,
		.flags = IORESOURCE_MEM
	},
	[1] = {
		.start = P7_UART1_IRQ,
		.end   = P7_UART1_IRQ,
		.flags = IORESOURCE_IRQ
	}
};

static struct resource p7_uart2_res[] = {
	[0] = {
		.start = P7_UART2,
		.end   = P7_UART2 + SZ_4K - 1,
		.flags = IORESOURCE_MEM
	},
	[1] = {
		.start = P7_UART2_IRQ,
		.end   = P7_UART2_IRQ,
		.flags = IORESOURCE_IRQ
	}
};

static struct resource p7_uart3_res[] = {
	[0] = {
		.start = P7_UART3,
		.end   = P7_UART3 + SZ_4K - 1,
		.flags = IORESOURCE_MEM
	},
	[1] = {
		.start = P7_UART3_IRQ,
		.end   = P7_UART3_IRQ,
		.flags = IORESOURCE_IRQ
	}
};

static struct resource p7_uart4_res[] = {
	[0] = {
		.start = P7_UART4,
		.end   = P7_UART4 + SZ_4K - 1,
		.flags = IORESOURCE_MEM
	},
	[1] = {
		.start = P7_UART4_IRQ,
		.end   = P7_UART4_IRQ,
		.flags = IORESOURCE_IRQ
	}
};

static struct resource p7_uart5_res[] = {
	[0] = {
		.start = P7_UART5,
		.end   = P7_UART5 + SZ_4K - 1,
		.flags = IORESOURCE_MEM
	},
	[1] = {
		.start = P7_UART5_IRQ,
		.end   = P7_UART5_IRQ,
		.flags = IORESOURCE_IRQ
	}
};

static struct resource p7_uart6_res[] = {
	[0] = {
		.start = P7_UART6,
		.end   = P7_UART6 + SZ_4K - 1,
		.flags = IORESOURCE_MEM
	},
	[1] = {
		.start = P7_UART6_IRQ,
		.end   = P7_UART6_IRQ,
		.flags = IORESOURCE_IRQ
	}
};

static struct resource p7_uart7_res[] = {
	[0] = {
		.start = P7_UART7,
		.end   = P7_UART7 + SZ_4K - 1,
		.flags = IORESOURCE_MEM
	},
	[1] = {
		.start = P7_UART7_IRQ,
		.end   = P7_UART7_IRQ,
		.flags = IORESOURCE_IRQ
	}
};

static struct platform_device p7_uart_devs[] = {
	{
		.name           = PXUART_DRV_NAME,
		.id             = 0,
		.resource       = p7_uart0_res,
		.num_resources  = ARRAY_SIZE(p7_uart0_res)
	},
	{
		.name           = PXUART_DRV_NAME,
		.id             = 1,
		.resource       = p7_uart1_res,
		.num_resources  = ARRAY_SIZE(p7_uart1_res)
	},
	{
		.name           = PXUART_DRV_NAME,
		.id             = 2,
		.resource       = p7_uart2_res,
		.num_resources  = ARRAY_SIZE(p7_uart2_res)
	},
	{
		.name           = PXUART_DRV_NAME,
		.id             = 3,
		.resource       = p7_uart3_res,
		.num_resources  = ARRAY_SIZE(p7_uart3_res)
	},
	{
		.name           = PXUART_DRV_NAME,
		.id             = 4,
		.resource       = p7_uart4_res,
		.num_resources  = ARRAY_SIZE(p7_uart4_res)
	},
	{
		.name           = PXUART_DRV_NAME,
		.id             = 5,
		.resource       = p7_uart5_res,
		.num_resources  = ARRAY_SIZE(p7_uart5_res)
	},
	{
		.name           = PXUART_DRV_NAME,
		.id             = 6,
		.resource       = p7_uart6_res,
		.num_resources  = ARRAY_SIZE(p7_uart6_res)
	},
	{
		.name           = PXUART_DRV_NAME,
		.id             = 7,
		.resource       = p7_uart7_res,
		.num_resources  = ARRAY_SIZE(p7_uart7_res)
	},
};

/**
 * p7_init_uart() - Instantiate I2C master controller identified by @bus
 *                  for further driver usage.
 *
 * @port:       UART port identifier
 * @pins:       array of pin functions and settings
 * @pin_cnt:    number of element of @pins array
 */
void __init p7_init_uart(int port,
                         struct pinctrl_map* pins,
                         size_t pin_cnt)
{
	int err;

#ifdef DEBUG
	BUG_ON(port >= ARRAY_SIZE(p7_uart_devs));
	BUG_ON(! pins);
	BUG_ON(! pin_cnt);
#endif
	if (port >= ARRAY_SIZE(p7_uart_devs)) {
		printk(KERN_ERR "p7 Uart%d do not exist\n", port);
		return;
	}

	err = p7_init_dev(&p7_uart_devs[port], NULL, pins, pin_cnt);
	if (err)
		panic("p7: failed to init UART %d (%d)\n", port, err);
}
