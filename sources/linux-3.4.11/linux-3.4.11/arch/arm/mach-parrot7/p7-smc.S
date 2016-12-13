/*
 * Parrot7 secure API file
 */

#include <linux/linkage.h>

/*
 * CPU0 only, enable or disable L2 cache
 * r0 (smc_id) : SMC_L2CACHE (2) 
 * r1 (arg0): PL310 Auxiliary Control Reg value
 * r2 (arg1): (enable << 31) | (data_ram_ctrl << 16) | tag_ram_ctrl
 *
 */
ENTRY(smc_l2x0_enable)
	stmfd	sp!, {r2-r12, lr}
	mov	r2, r1
	mov	r1, r0
	mov	r0, #0x2
	dsb
	.arch_extension sec
	smc	#0
	ldmfd	sp!, {r2-r12, pc}
ENDPROC(smc_l2x0_enable)

ENTRY(smc_l2x0_disable)
	stmfd	sp!, {r1-r12, lr}
	mov	r2, #0x0
	mov	r1, #0x0
	mov	r0, #0x2
	dsb
	.arch_extension sec
	smc	#0
	ldmfd	sp!, {r1-r12, pc}
ENDPROC(smc_l2x0_disable)

/*
 * Write Auxiliary control register
 * r0 (smc_id): SMC_ACTLR (3)
 * r1 (arg0): value to be written
 */
ENTRY(smc_actlr)
	stmfd	sp!, {r1-r12, lr}
	mov	r1, r0
	mov	r0, #0x3
	dsb
	.arch_extension sec
	smc	#0
	ldmfd	sp!, {r1-r12, pc}
ENDPROC(smc_actlr)

/*
 * CPU0 only, Write PL301 remap register
 * r0 (smc_id): SMC_PL301_REMAP (4)
 * r1 (arg0): value to be written
 */
ENTRY(smc_pl301_remap)
	/*stmfd	sp!, {r2-r12, lr}*/
	mov	r1, r0
	mov	r0, #0x4
	dsb
	.arch_extension sec
	smc	#0
	/*ldmfd	sp!, {r2-r12, pc}*/
	mov pc, lr
ENDPROC(smc_pl301_remap)

/*
 * ARM_ERRATA_751472: An interrupted ICIALLUIS operation may 
 * prevent the completion of a following broadcasted operation
 * Present in all r0, r1 and r2 revisions, fixed in r3r0
 * This service is implemented in the library extension
 * r0 (smc_id): EXT_DISABLE_IR_MNT (6)
 */
ENTRY(smc_disable_ir_mnt)
	stmfd	sp!, {r1-r12, lr}
	mov	r0, #0x6
	dsb
	.arch_extension sec
	smc	#0
	ldmfd	sp!, {r1-r12, pc}
ENDPROC(smc_disable_ir_mnt)

/*
 * ARM_ERRATA_743622: Faulty logic in the store buffer may lead
 * to data corruption
 * Present in all r2 revisions, fixed in r3p0
 * This service is implemented in the library extension
 * r0 (smc_id): EXT_DISABLE_FLU_OPT (7)
 */
ENTRY(smc_disable_flu_opt)
	stmfd	sp!, {r1-r12, lr}
	ldr	r0, =0x7
	dsb
	.arch_extension sec
	smc	#0
	ldmfd	sp!, {r1-r12, pc}
ENDPROC(smc_disable_flu_opt)


/*
 * Enable debug in non-secure modei
 * This service is implemented in the library extension
 * r0 (smc_id): EXT_ENABLE_DEBUG (8)
 */
ENTRY(smc_enable_debug)
	stmfd	sp!, {r1-r12, lr}
	ldr	r0, =0x8
	dsb
	.arch_extension sec
	smc	#0
	ldmfd	sp!, {r1-r12, pc}
ENDPROC(smc_enable_debug)

/*
 * Write access to L2X0 ctrl reg
 * This service is implemented in the library extension
 * r0 (smc_id): EXT_WR_L2_CTRL (9)
 */
ENTRY(smc_l2x0_ctrl)
	stmfd	sp!, {r1-r12, lr}
	mov	r1, r0
	ldr	r0, =0x9
	dsb
	.arch_extension sec
	smc	#0
	ldmfd	sp!, {r1-r12, pc}
ENDPROC(smc_l2x0_ctrl)

/*
 * Write access to L2X0 auxiliary ctrl reg
 * This service is implemented in the library extension
 * r0 (smc_id): EXT_WR_L2_AUX (10)
 */
ENTRY(smc_l2x0_aux)
	stmfd	sp!, {r1-r12, lr}
	mov	r1, r0
	ldr	r0, =0xa
	dsb
	.arch_extension sec
	smc	#0
	ldmfd	sp!, {r1-r12, pc}
ENDPROC(smc_l2x0_aux)

/*
 * Write access to L2X0 tag RAM latency ctrl reg
 * This service is implemented in the library extension
 * r0 (smc_id): EXT_WR_L2_TAG_LAT (11)
 */
ENTRY(smc_l2x0_tag_lat)
	stmfd	sp!, {r1-r12, lr}
	mov	r1, r0
	ldr	r0, =0xb
	dsb
	.arch_extension sec
	smc	#0
	ldmfd	sp!, {r1-r12, pc}
ENDPROC(smc_l2x0_tag_lat)

/*
 * Write access to L2X0 data RAM latency ctrl reg
 * This service is implemented in the library extension
 * r0 (smc_id): EXT_WR_L2_DAT_LAT (12)
 */
ENTRY(smc_l2x0_dat_lat)
	stmfd	sp!, {r1-r12, lr}
	mov	r1, r0
	ldr	r0, =0xc
	dsb
	.arch_extension sec
	smc	#0
	ldmfd	sp!, {r1-r12, pc}
ENDPROC(smc_l2x0_dat_lat)

/*
 * Write access to L2X0 debug ctrl reg
 * This service is implemented in the library extension
 * r0 (smc_id): EXT_WR_L2_DBG_CTRL (13)
 */
ENTRY(smc_l2x0_dbg_ctrl)
	stmfd	sp!, {r1-r12, lr}
	mov	r1, r0
	ldr	r0, =0xd
	dsb
	.arch_extension sec
	smc	#0
	ldmfd	sp!, {r1-r12, pc}
ENDPROC(smc_l2x0_dbg_ctrl)

/*
 * Write access to L2X0 prefetch ctrl reg
 * This service is implemented in the library extension
 * r0 (smc_id): EXT_WR_L2_PFT_CTRL (14)
 */
ENTRY(smc_l2x0_pft_ctrl)
	stmfd	sp!, {r1-r12, lr}
	mov	r1, r0
	ldr	r0, =0xe
	dsb
	.arch_extension sec
	smc	#0
	ldmfd	sp!, {r1-r12, pc}
ENDPROC(smc_l2x0_pft_ctrl)

/*
 * Write access to L2X0 power ctrl reg
 * This service is implemented in the library extension
 * r0 (smc_id): EXT_WR_L2_PWR_CTRL (15)
 */
ENTRY(smc_l2x0_pwr_ctrl)
	stmfd	sp!, {r1-r12, lr}
	mov	r1, r0
	ldr	r0, =0xf
	dsb
	.arch_extension sec
	smc	#0
	ldmfd	sp!, {r1-r12, pc}
ENDPROC(smc_l2x0_pwr_ctrl)

/*
 * Write access to L2X0 addr filtering start reg
 * This service is implemented in the library extension
 * r0 (smc_id): EXT_WR_L2_ADR_FIL_START (16)
 */
ENTRY(smc_l2x0_adr_fil_start)
	stmfd	sp!, {r1-r12, lr}
	mov	r1, r0
	ldr	r0, =0x10
	dsb
	.arch_extension sec
	smc	#0
	ldmfd	sp!, {r1-r12, pc}
ENDPROC(smc_l2x0_adr_fil_start)

/*
 * Write access to L2X0 addr filtering end reg
 * This service is implemented in the library extension
 * r0 (smc_id): EXT_WR_L2_ADR_FIL_END (17)
 */
ENTRY(smc_l2x0_adr_fil_end)
	stmfd	sp!, {r1-r12, lr}
	mov	r1, r0
	ldr	r0, =0x11
	dsb
	.arch_extension sec
	smc	#0
	ldmfd	sp!, {r1-r12, pc}
ENDPROC(smc_l2x0_adr_fil_end)

/*
 * Write access to SCU non-secure access ctrl reg
 * This service is implemented in the library extension
 * r0 (smc_id): EXT_WR_NSACR (18)
 */
ENTRY(smc_scu_nsacr)
	stmfd	sp!, {r1-r12, lr}
	ldr	r0, =0x12
	dsb
	.arch_extension sec
	smc	#0
	ldmfd	sp!, {r1-r12, pc}
ENDPROC(smc_scu_nsacr)

/*
 * Write access to L2X0 invalidate way reg
 * This service is implemented in the library extension
 * r0 (smc_id): EXT_WR_L2_INV_WAY (19)
 */
ENTRY(smc_l2x0_inv_way)
	stmfd	sp!, {r1-r12, lr}
	mov	r1, r0
	ldr	r0, =0x13
	dsb
	.arch_extension sec
	smc	#0
	ldmfd	sp!, {r1-r12, pc}
ENDPROC(smc_l2x0_inv_way)

/*
 * Write access to CPACR CP10/CP11 access ctrl reg
 * This service is implemented in the library extension
 * r0 (smc_id): EXT_WR_CPACR (20)
 */
ENTRY(smc_scu_cpacr)
	stmfd	sp!, {r1-r12, lr}
	mov	r1, r0 
	ldr	r0, =0x14
	dsb
	.arch_extension sec
	smc	#0
	ldmfd	sp!, {r1-r12, pc}
ENDPROC(smc_scu_cpacr)
/*
 * Test if CPU is in non-secure state
 */
ENTRY(p7_is_nonsecure_)
	stmfd	sp!, {r1-r12, lr}
	mrs	r1, cpsr	/*save local irq flags */
	mrc	p15, 0, r2, c1, c0, 1	/* read ACTLR */
	mov	r3, #0x80	/* ACTLR.EXCL mask*/
	eor	r4, r3, r2	/* attempt to toggle ACTLR.EXCL*/
	mcr	p15, 0, r4, c1, c0, 1
	mrc	p15, 0, r5, c1, c0, 1	/* read back ACTLR */
	mcr	p15, 0, r2, c1, c0, 1 	/* write back ACTLR */
	msr	cpsr_c, r1	/*resotre local irq flags */
	eors	r2, r5
	beq	1f
	mov	r0, #0
	b	2f
1:	mov	r0, #1
2:	ldmfd	sp!, {r1-r12, pc}
ENDPROC(p7_is_nonsecure_)
