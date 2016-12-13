/*
 *  linux/arch/arm/mach-parrot7/include/mach/entry-macro.S
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

#include <asm/ptrace.h>
#include <asm/hardware/gic.h>
#include <mach/p7.h>
#include <mach/io.h>
#include <mach/irqs.h>

/* Uncomment this if running on old P7 releases where FIQs are always on... */
#if 0
#define P7_FIQ_WORKAROUND
#endif

	.macro	disable_fiq
#ifdef P7_FIQ_WORKAROUND
	mrs	r8, spsr
    orr r8, r8, #PSR_F_BIT
	msr	spsr, r8
#endif
	.endm

	.macro	get_irqnr_preamble, base, tmp
	ldr	\base, =__MMIO_P2V(P7_CPU_ICC)
	.endm

	.macro	arch_ret_to_user, tmp1, tmp2
	.endm

	/*
	 * The interrupt numbering scheme is defined in the
	 * interrupt controller spec.  To wit:
	 *
	 * Interrupts 0-15 are IPI
	 * 16-28 are reserved
	 * 29-31 are local.  We allow 30 to be used for the watchdog.
	 * 32-1020 are global
	 * 1021-1022 are reserved
	 * 1023 is "spurious" (no interrupt)
	 *
	 * For now, we ignore all local interrupts so only return an interrupt if it's
	 * between 30 and 1020.  The test_for_ipi routine below will pick up on IPIs.
	 *
	 * A simple read from the controller will tell us the number of the highest
	 * priority enabled interrupt.  We then just need to check whether it is in the
	 * valid range for an IRQ (30-1020 inclusive).
	 */

	.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
	ldr     \irqstat, [\base, #GIC_CPU_INTACK] /* bits 12-10 = src CPU, 9-0 = int # */
	ldr     \tmp, =1021
	bic     \irqnr, \irqstat, #0x1c00
	cmp     \irqnr, #P7_LOCALTIMER_IRQ
	cmpcc	\irqnr, \irqnr
	cmpne	\irqnr, \tmp
	cmpcs	\irqnr, \irqnr
	.endm

	/* We assume that irqstat (the raw value of the IRQ acknowledge
	 * register) is preserved from the macro above.
	 * If there is an IPI, we immediately signal end of interrupt on the
	 * controller, since this requires the original irqstat value which
	 * we won't easily be able to recreate later.
	 */

	.macro test_for_ipi, irqnr, irqstat, base, tmp
	bic	\irqnr, \irqstat, #0x1c00
	cmp	\irqnr, #16
	strcc	\irqstat, [\base, #GIC_CPU_EOI]
	cmpcs	\irqnr, \irqnr
	.endm

	/* As above, this assumes that irqstat and base are preserved.. */

	.macro test_for_ltirq, irqnr, irqstat, base, tmp
	bic	\irqnr, \irqstat, #0x1c00
	mov 	\tmp, #0
	cmp	\irqnr, #P7_LOCALTIMER_IRQ
	moveq	\tmp, #1
	streq	\irqstat, [\base, #GIC_CPU_EOI]
	cmp	\tmp, #0
	.endm

/* vim:set ts=4:sw=4:noet:ft=asm: */
