/*
 *	linux/arch/arm/mach-parrot7/include/mach/debug-macro.S
 *
 *	Copyright (C) 2010 Parrot S.A.
 *
 * @author	Gregor Boirie <gregor.boirie@parrot.com>
 * @author	Lionel Flandrin <lionel.flandrin@parrot.com>
 * @date  15-Nov-2010
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
#include <asm/memory.h>

#if defined(CONFIG_PARROT7_DEBUG_LL_ZPRINT)
	.macro  addruart,rp,rv,tmp
		ldr \rp, =P7_ZPRINT             @ physical
		ldr \rv, =__MMIO_P2V(P7_ZPRINT) @ virtual
	.endm

	.macro  senduart,rd,rx
		strb    \rd, [\rx, #0]
		dsb
	.endm

	.macro  waituart,rd,rx
	.endm

	.macro  busyuart,rd,rx
	.endm

#else /* ZPRINT */

	.macro  addruart,rp,rv,tmp
	/* The virtual and physical addresses are stored contiguously in
	p7_early_uart declared in march-parrot7/devices.c */
	ldr \tmp, =p7_early_uart

	/* There's a trick here, we can't access =p7_early_uart directly because
	it might not be valid if we're not yet remapped into virtual
	addresses. To make sure this code works in both real and virtual modes I
	use a trick from mach-omap2 where we use the relative offset between the
	current PC and a known location (1999f in the code below) to get a
	correct address each time. */
	adr \rp, 1999f      @ current address of word at label 1999
	ldr \rv, [\rp]      @ absolute (virtual) address of word at label 1999
	sub \rv, \rv, \rp   @ offset between the two. That should be 0 in
	                    @ virtual mode
	sub \tmp, \tmp, \rv @ "patch" the address of p7_early_uart with the
	                    @ computed offset

	/* At this point \tmp contains the proper current address of p7_early_uart */

	ldr \rp, [\tmp]      @ p7_early_uart[0] (physical addr)
	ldr \rv, [\tmp, #4]  @ p7_early_uart[1] (virtual addr)

	/* If it's non-0 we can use these values directly */
	cmp   \rp, #0
	cmpne \rv, #0
	bne   1002f

	/* Otherwise we have to initialize the value of p7_early_uart with the
	address of the UART specified in the P7 BOOT_MODE */
	mrc p15, 0, \rv, c1, c0
	tst \rv, #1           @ is MMU active?

	/* If the MMU is active we can't be sure the SYS block has been properly
	remapped yet, so we bail out rather than risking a DATA_ABORT */
	bne 1002f

	ldr \rv, =P7_BOOT_MODE
	ldr \rp, [\rv]    @ Read the BOOT_MODE register value
	and \rp, \rp, #P7_BOOT_MODE_DIAG_MASK

	cmp   \rp, #P7_BOOT_MODE_DIAG_UART0
	ldreq \rp, =P7_UART0
	ldreq \rv, =__MMIO_P2V(P7_UART0)
	beq   1001f

	cmp   \rp, #P7_BOOT_MODE_DIAG_UART1
	ldreq \rp, =P7_UART1
	ldreq \rv, =__MMIO_P2V(P7_UART1)
	beq   1001f

	cmp   \rp, #P7_BOOT_MODE_DIAG_UART2
	ldreq \rp, =P7_UART2
	ldreq \rv, =__MMIO_P2V(P7_UART2)
	beq   1001f

	cmp   \rp, #P7_BOOT_MODE_DIAG_UART3
	ldreq \rp, =P7_UART3
	ldreq \rv, =__MMIO_P2V(P7_UART3)
	beq   1001f

	/* Fallthrough: the BOOT_MODE has an unknown or unsupported value */
	mov \rp, #0
	mov \rv, #0
1001:
	/* Now we want to store the \rp and \rv values back in p7_early_uart so
	that they are found the next time this function is called. */
	str \rp, [\tmp]      @ p7_early_uart[0] (physical addr)
	str \rv, [\tmp, #4]  @ p7_early_uart[1] (virtual addr)

	b 1002f

1999:
	/* This is just for calculating the offset of p7_early_uart above */
	.word	.
	.ltorg
1002:
	/* At this point \rp and \rv contain either correct physical and virtual
	addresses or 0 if there was a problem and the early printk is unusable. */
	.endm


	.macro  senduart,rd,rx
	/* NOP when rx is 0 (see addruart) */
	cmp \rx, #0
	beq 1003f
	strb \rd, [\rx, #_UART_TRX]
1003:
	.endm


	.macro  waituart,rd,rx
	/* NOP when rx is 0 (see addruart) */
	cmp \rx, #0
	beq 1005f
1004:
	ldr \rd, [\rx, #_UART_STATUS]
	tst \rd, #UART_STATUS_TXFILLED
	bne 1004b
1005:
	.endm


	.macro  busyuart,rd,rx
	/* NOP when rx is 0 (see addruart) */
	cmp \rx, #0
	beq 1007f
1006:
	ldr \rd, [\rx, #_UART_STATUS]
	tst \rd, #UART_STATUS_TXEMPTY
	beq 1006b
1007:
	.endm

#endif /* ZPRINT */
