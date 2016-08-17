
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/p7.h>
#include "gbc.h"

#ifdef CONFIG_ARCH_PARROT7_DEBUG_S2RAM
#define P7_DDR_CRC_ALL 1
#else
#define P7_DDR_CRC_ALL 0
#endif

#define I2CM_TIP                (1 << 1)
#define I2CM_COMMAND_WR         (1 << 1)
#define I2CM_COMMAND_STOP       (1 << 3)
#define I2CM_COMMAND_START      (1 << 4)
#define I2CM_TX_REG             (0x00)
#define I2CM_STATUS_REG         (0x10)
#define I2CM_COMMAND_REG        (0x14)
#define I2CM_PRESCALE           (0x18)

#define P7MU_ADDR               (0x31)
#define P7MU_COMMAND_REG        (0x107)
#define P7MU_SUSPEND_CMD        (1 << 1)
#define P7MU_APP0               (0x900)
#define P7MU_APP1               (0x901)
#define P7MU_APP3               (0x903)
#define P7MU_APP4               (0x904)
#define P7MU_APP5               (0x905)
#define P7MU_APP6               (0x906)

.macro p7mu_write, reg16
	@ input r0 (data)
	@ local use r0: cmd byte, r1: byte to transmit, r2
	mov		r2, r0
	mov     r0, #I2CM_COMMAND_START
	mov     r1, #(P7MU_ADDR << 1)
	bl      send_byte
	mov     r0, #0
	mov     r1, #((\reg16 >> 8))
	bl      send_byte
	mov     r1, #(\reg16 & 0xff)
	bl      send_byte
	mov		r1, r2,	lsr #8
	bl      send_byte
	mov     r0, #I2CM_COMMAND_STOP
	and		r1, r2, #255
	bl      send_byte
.endm

	.text

ENTRY(p7_finish_suspend)
	@set vector at 0. It helps debug in case of abort
	mrc	p15, 0, r0, c1, c0, 0		@ read control register
	bic	r0, r0, #(1 << 13)
	mcr	p15, 0, r0, c1, c0, 0		@ write control reg

#if P7_DDR_CRC_ALL
	ldr r0, p7_s2r_param_ptr
	ldr r10, [r0]
	adr     r0, rom_bases
	ldmia	r0, {r4 - r9}
	@we store the mmu table after the stack. We need 32K align
	mov sp, #(P7_INTRAM+32*1024)
	stmdb sp!, {r0,r1,r2,r4,r5,r7,r9,r11}
	blx r4 @cache invalidate
	ldmia sp!, {r0,r1,r2,r4,r5,r7,r9,r11}
	blx r5 @mmu_off
	mov r0, sp
	mov r1,	r10, lsr #20
	blx r6 @mmu_on
	blx r7 @dcache
	blx r8 @icache


	@crc of all ddr
	mov r0, #(0x80000000)
	mov r1, r10
	blx r9 @posix_crc
	mov r10, r0
#else
	mov r10, #0
#endif

#if 0
	@crc of resume code
	ldr r0, p7_s2r_param_ptr
	ldr r0, [r0, #4]
	mov r1, #4096
	blx r9 @posix_crc
	mov r9, r0
#else
	mov r9, #0
#endif
	
	@ r4: mpmc self refresh register
	@ r5: mpmc gate training register
	@ r6: i2cm base address
	@ r7: i2cm clock register
	@ r8: watchdog base address
	adr     r0, reg_bases
	ldmia	r0, {r4 - r8}

	@ enable watchdog
	mov     r0, #0xff000 @load register. the timeout is ((load + 1) * (prescaler + 1)) /  (cpu_clk/2)
						 @for 780Mhz cpu it should give us 680ms.
	str     r0, [r8, #0x20]
	movw    r0, #0x0000FF09 @ctrl reg : prescaler = 0xff, WD mode, enable
	str     r0, [r8, #0x28]

	@ disable gate training
	mov     r0, #0
	str     r0, [r5]

	@ enable self-refresh
	mov     r0, #1
	str     r0, [r4]
	@ MPMC is now in self-refresh

	@ enable i2c clock and dereset IP
	mov     r0, #1
	str     r0, [r7]
	mov     r0, #0
	str     r0, [r7, #(LSP_GBC_I2CM0_RESET - LSP_GBC_I2CM0_CLOCK)]

	@ init i2c IP
	mov     r0, #0
	str     r0, [r6, #I2CM_PRESCALE]

	mov     r0, #0xd8 @100KHZ
	str     r0, [r6, #I2CM_PRESCALE]

	@crc of ddr
	mov r0, r10, lsr #16
	p7mu_write P7MU_APP3
	mov r0, r10
	p7mu_write P7MU_APP4

	@ Send the suspend command to the P7MU via I2C
	mov r0, #P7MU_SUSPEND_CMD
	p7mu_write P7MU_COMMAND_REG

	@ Wait here for P7MU to shutdown the power domain that
	@ powers the CPU
	b       .

	@ very simple send byte function for I2CM
	@ write command and byte to xfer, wait for xfer to be done, return
	@ there is no error handling
	@ r6: i2cm base address
	@ r0: cmd
	@ r1: byte to write
	@ ip: tmp
send_byte: 
	orr     r0, r0, #I2CM_COMMAND_WR
	str     r1, [r6, #I2CM_TX_REG]
	str     r0, [r6, #I2CM_COMMAND_REG]
1:	ldr     ip, [r6, #I2CM_STATUS_REG]
	ands    ip, ip, #I2CM_TIP
	bne     1b
	bx      lr

reg_bases:
	.long   P7_MPMC_GBC + MPMC_GBC_SELF_REFRESH
	.long   P7_MPMC + P7_MPMC_TRAIN_EN
	.long   P7_I2CM0
	.long   P7_LSP_GBC + LSP_GBC_I2CM0_CLOCK
	.long   P7_CPU_LOCALTIMER

rom_bases:
	.long	0x00000244 @invalidate_caches_and_tlbs
	.long	0x000144ac @mmu_off
	.long   0x0001430c @mmu_on
	.long   0x000016a0 @dcache_on
	.long   0x00001670 @icache_on
	.long	0x00001250 @posix_crc

p7_s2r_param_ptr:
	.word p7_s2r_param

ENTRY(p7_finish_suspend_sz)
	.word . - p7_finish_suspend
