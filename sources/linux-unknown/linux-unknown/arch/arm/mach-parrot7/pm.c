/*
 * Copyright 2013 Parrot S.A.
 * Author: Alvaro Moran <alvaro.moran@parrot.com>
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/suspend.h>
#include <asm/cacheflush.h>
#include <asm/proc-fns.h>
#include <asm/suspend.h>
#include <asm/system_misc.h>
#include <mfd/p7mu.h>
#include <linux/delay.h>
#include <asm/smp_scu.h>
#include <linux/mman.h>
#include <mach/p7.h>
#include "common.h"
#include "system.h"
#include "clock.h"
#include "dma.h"
#include "core.h"

#include <mfd/p7mu.h>
#include <asm/setup.h>
#include <asm/smp_twd.h>

/* 
 * TODO: Consider that the beginning of INTRAM might be used by the DMA, so we 
 * just want to be sure nobody else is using it.
 */

int p7_resume_cnt;
static u32 suspend_vaddr;
extern void p7_finish_suspend(void);
extern int p7_finish_suspend_sz;
struct {
	int p7_ram_size;
} p7_s2r_param;

static int __maybe_unused p7_last_jump(unsigned long arg)
{
	/* assume only one memory bank : we have one CS */
	struct membank *bank = &meminfo.bank[0];
	p7_s2r_param.p7_ram_size = bank->size;

	outer_flush_all();

	soft_restart(P7_INTRAM);

	/* Should never get here. */
	BUG();
}

/* 
 * Will copy the last suspend instructions to intram.
 */
static int p7_prepare_suspend(void)
{

	if (p7_finish_suspend_sz > 8 * 1024) {
		printk("Error sleep code is too big %d\n", p7_finish_suspend_sz);
		return -1;
	}
	/* 
	 * map physical addresses of INTRAM controller to copy the sleep code 
	 * and jump there later.
	 */
	suspend_vaddr = (u32)__arm_ioremap_exec(P7_INTRAM,
	                                        p7_finish_suspend_sz, false);
	if (!suspend_vaddr) {
		printk("Error trying to map 0x%lx\n", P7_INTRAM);
		return -1;
	}

	return 0;
}

static void p7_unprepare_suspend(void)
{
	__iounmap((void*)suspend_vaddr);
}

static int p7_enter_pm(suspend_state_t state)
{
	int err;
	u32 scu_ctrl __maybe_unused;

	switch (state) {
	case PM_SUSPEND_STANDBY:
		cpu_do_idle();
		return 0;

#ifdef CONFIG_PM_SLEEP
	case PM_SUSPEND_MEM:
#ifdef CONFIG_ARCH_PARROT7_DEBUG_S2RAM
		/* debug use rom code and we use p7r3 symbols */
		if (p7_chiprev() != P7_CHIPREV_R3) {
			return -EINVAL;
		}
		/* keep 1K for stack */
		if (p7_is_nonsecure()) {
			return -EINVAL;
		}
#endif

		/* warning intram is also used by PL330 ucode */
		memcpy_toio((void*)suspend_vaddr,
		            p7_finish_suspend, p7_finish_suspend_sz);
		err = cpu_suspend(0, p7_last_jump);

		if (err == 0) {
			p7_resume_cnt++;
		}
		printk("Starting resume #%d... (%d)\n", p7_resume_cnt, err);

		/* stop watchdog set by bootloader on cpu0 */
		writel(0x12345678, __MMIO_P2V(P7_CPU_LOCALTIMER) + TWD_WDOG_DISABLE);
		writel(0x87654321, __MMIO_P2V(P7_CPU_LOCALTIMER) + TWD_WDOG_DISABLE);
		writel(0x0, __MMIO_P2V(P7_CPU_LOCALTIMER) + TWD_WDOG_CONTROL);

		/* De-reset NEON unit. */
		p7_enable_neonclk(0);
#ifdef CONFIG_SMP
		platform_smp_prepare_cpus(num_possible_cpus());
#endif
#ifdef CONFIG_CACHE_L2X0
		outer_resume();
#endif
#ifdef CONFIG_DMA_PARROT7
		/*
		 * PL330 knows that a channel is free when the first
		 * instruction in channel's microcode is DMAEND (0x0).
		 * To simplify, make all memory used for DMA ucode to 0.
		 */
		memset((void*)__MMIO_P2V(p7_dma_ucode_addr()),
		       0, p7_dma_ucode_sz());
#endif
		break;
#endif
	default:
		err = -EINVAL;
	}

	return err;
}

static int p7_begin_pm(suspend_state_t state)
{
	u16 reg;
	int ret;

	disable_hlt();

	p7mu_read16(0xe18, &reg); /* OTP reset value */
	reg = (reg >> 8) ^ 'P'; /* xor with 'P' */
	p7mu_write16(0xe83, reg);

	ret = p7mu_save_cpu_jump(cpu_resume);
	if (ret)
		return ret;
	return p7_prepare_suspend();
}

static void p7_end_pm(void)
{
	p7_unprepare_suspend();
	enable_hlt();
	return;
}

static struct platform_suspend_ops p7_pm_ops = {
	.begin		= p7_begin_pm,
	.end		= p7_end_pm,
	.enter		= p7_enter_pm,
	.valid		= suspend_valid_only_mem,
};

void __init p7_pm_init(void)
{
	suspend_set_ops(&p7_pm_ops);
}

