/*
 *  linux/arch/arm/mach-parrot7/include/mach/uncompress.h
 *
 *  Copyright (C) 2010 Parrot S.A.
 *
 * @author  Gregor Boirie <gregor.boirie@parrot.com>
 * @date  28-Oct-2010
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

#include <mach/p7.h>
#include <mach/regs-uart.h>

#if defined(CONFIG_PARROT7_DEBUG_LL_UART0)
#define UART_TX		(*(volatile unsigned char *)(P7_UART0 + _UART_TRX))
#define UART_STAT	(*(volatile unsigned char *)(P7_UART0 + _UART_STATUS))
#elif defined(CONFIG_PARROT7_DEBUG_LL_UART1)
#define UART_TX		(*(volatile unsigned char *)(P7_UART1 + _UART_TRX))
#define UART_STAT	(*(volatile unsigned char *)(P7_UART1 + _UART_STATUS))
#elif defined(CONFIG_PARROT7_DEBUG_LL_UART2)
#define UART_TX		(*(volatile unsigned char *)(P7_UART2 + _UART_TRX))
#define UART_STAT	(*(volatile unsigned char *)(P7_UART2 + _UART_STATUS))
#elif defined(CONFIG_PARROT7_DEBUG_LL_UART3)
#define UART_TX		(*(volatile unsigned char *)(P7_UART3 + _UART_TRX))
#define UART_STAT	(*(volatile unsigned char *)(P7_UART3 + _UART_STATUS))
#endif


#if defined(CONFIG_PARROT7_DEBUG_LL_UART0) \
	|| defined(CONFIG_PARROT7_DEBUG_LL_UART1) \
	|| defined(CONFIG_PARROT7_DEBUG_LL_UART2) \
	|| defined(CONFIG_PARROT7_DEBUG_LL_UART3)

/*
 * This does not append a newline
 */
static inline void putc(int c)
{
	while (UART_STAT & UART_STATUS_TXFILLED)
		barrier();

	UART_TX = c;
}

static inline void flush(void)
{
	while ((UART_STAT & UART_STATUS_TXEMPTY) == 0)
		barrier();
}
#else
static inline void putc(int c)
{
}

static inline void flush(void)
{
}

#endif

/*
 * nothing to do
 */
#define arch_decomp_setup()
#define arch_decomp_wdog()

/* vim:set ts=4:sw=4:noet:ft=c: */
