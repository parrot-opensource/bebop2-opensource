/**
 *
 *       @file  power.c
 *
 *      @brief  
 *
 *     @author  Gregor Boirie <gregor.boirie@parrot.com>
 *       @date  30-Apr-2012
 *
 *        $Id:$
 *
 *       Copyright (C) 2008 Parrot S.A.
 */

#include <linux/init.h>
#include <linux/delay.h>
#include <asm/system_misc.h>
#include <asm/io.h>
#include <asm/smp_scu.h>
#include <asm/hardware/cache-l2x0.h>
#include <linux/clk.h>
#include "system.h"
#include "common.h"
#include "clock.h"
#include "core.h"
#include "gbc.h"


/* only reset cpu1 not powerdown it */
//#define CPU1_NO_PWDN

extern void smc_l2x0_enable(unsigned int aux, unsigned int val);
extern void smc_l2x0_disable(void);
extern void smc_actlr(unsigned int val);
extern void smc_l2x0_ctrl(unsigned int val);
extern void smc_l2x0_aux(unsigned int val);
extern void smc_l2x0_tag_lat(unsigned int val);
extern void smc_l2x0_dat_lat(unsigned int val);
extern void smc_l2x0_dbg_ctrl(unsigned int val);
extern void smc_l2x0_pft_ctrl(unsigned int val);
extern void smc_l2x0_pwr_ctrl(unsigned int val);
extern void smc_l2x0_adr_fil_start(unsigned int val);
extern void smc_l2x0_adr_fil_end(unsigned int val);
extern void smc_l2x0_inv_way(unsigned int val);
extern int p7_is_nonsecure_(void);

int p7_is_nonsecure(void)
{
	static int secure_state = -1;

	if (secure_state == -1)
		secure_state = p7_is_nonsecure_();

	return secure_state;
}


static inline u32 read_actlr(void)
{
	u32 actlr;

	__asm__("mrc p15, 0, %0, c1, c0, 1" : "=r" (actlr));

	return actlr;
}

static inline void write_actlr(u32 actlr)
{
	if (p7_is_nonsecure()) {
		smc_actlr(actlr);
	} else {
		__asm__("mcr p15, 0, %0, c1, c0, 1" : : "r" (actlr));
	}
}

static inline u32 read_sctlr(void)
{
	u32 sctlr;

	__asm__("mrc p15, 0, %0, c1, c0, 0" : "=r" (sctlr));

	return sctlr;
}

static inline void write_sctlr(u32 sctlr)
{
	__asm__("mcr p15, 0, %0, c1, c0, 0" : : "r" (sctlr));
}
/**********************
 * L2 cache controller
 **********************/

#ifdef CONFIG_CACHE_L2X0
/*
 * Tag and Data L2 cache latencies have been estimated in terms of ratios
 * to L2X0_REFERENCE_CLK_RATE. Latencies are:
 *	L2X0_RATE_LOW  if tag_clk <= L2X0_REFERENCE_CLK_RATE
 *	L2X0_RATE_HIGH if tag_clk > L2X0_REFERENCE_CLK_RATE
 * We calculate p7_l2c_data and p7_l2c_tag the data/tag latencies by reading
 * the CPU_GBC_SCU_CLOCK divider.
 * Note that the 'SCU' in CPU_GBC_SCU_CLOCK term is not appropriate, it
 * configures the RAM clocks of L2Cache Controller. It's should be
 * CPU_GBC_L2C_CLOCK
 */
static void __init p7_check_l2c_lat(void)
{
	u32	dataclk_div, tagclk_div;
	struct clk* p7_cpu_clk = clk_get_sys("cpu_clk", NULL);
	const u32 cpu_rate = clk_get_rate(p7_cpu_clk);
	const u32	scu_clock_reg = __raw_readl(__MMIO_P2V(P7_CPU_GBC + CPU_GBC_SCU_CLOCK));

	clk_put(p7_cpu_clk);

	switch (p7_chiprev()) {
	case  P7_CHIPREV_R1:
		dataclk_div = (scu_clock_reg >> CPU_GBC_DATACLK_SHIFT_R1)
			& CPU_GBC_CLK_MASK_R1;
		tagclk_div = (scu_clock_reg >> CPU_GBC_TAGCLK_SHIFT_R1)
			& CPU_GBC_CLK_MASK_R1;
		break;
	case  P7_CHIPREV_R2:
	case  P7_CHIPREV_R3:
	default:
		dataclk_div = (scu_clock_reg >> CPU_GBC_DATACLK_SHIFT_R2)
			& CPU_GBC_CLK_MASK_R2;
		tagclk_div = (scu_clock_reg >> CPU_GBC_TAGCLK_SHIFT_R2)
			& CPU_GBC_CLK_MASK_R2;
		break;
	}

	if (cpu_rate > 390000000) {
		if (tagclk_div == 0)
			panic("wrong tagclk\n");
		if (dataclk_div == 0)
			panic("wrong tagclk\n");
	}
}

static void p7_pl310_set_debug(unsigned long val)
{
	smc_l2x0_dbg_ctrl(val);
}

static void p7_l2x0_disable(void)
{
	smc_l2x0_disable();
}

#ifdef CONFIG_PM_SLEEP
static void p7_l2x0_unlock(u32 cache_id)
{
	void __iomem*   l2x0_base = MMIO_P2V(P7_L2C);
	int lockregs;
	int i;

	if (cache_id == L2X0_CACHE_ID_PART_L310)
		lockregs = 8;
	else
		/* L210 and unknown types */
		lockregs = 1;

	for (i = 0; i < lockregs; i++) {
		writel_relaxed(0x0, l2x0_base + L2X0_LOCKDOWN_WAY_D_BASE +
			       i * L2X0_LOCKDOWN_STRIDE);
		writel_relaxed(0x0, l2x0_base + L2X0_LOCKDOWN_WAY_I_BASE +
			       i * L2X0_LOCKDOWN_STRIDE);
	}
}
static void p7_l2x0_resume(void)
{
	void __iomem*   l2x0_base = MMIO_P2V(P7_L2C);
	u32	l2x0_ways, l2x0_way_mask;

	if (!(readl_relaxed(l2x0_base + L2X0_CTRL) & 1)) {
		/* restore aux ctrl and enable l2 */
		p7_l2x0_unlock(readl_relaxed(l2x0_base + L2X0_CACHE_ID));
		smc_l2x0_aux(l2x0_saved_regs.aux_ctrl);
		if (l2x0_saved_regs.aux_ctrl & (1 << 16))
			l2x0_ways = 16;
		else
			l2x0_ways = 8;
		l2x0_way_mask = (1 << l2x0_ways) - 1;
		smc_l2x0_inv_way(l2x0_way_mask);
		smc_l2x0_ctrl(1);
	}
}

static void p7_pl310_resume(void)
{
	void __iomem*   l2x0_base = MMIO_P2V(P7_L2C);
	u32 l2x0_revision;

	if (!(readl_relaxed(l2x0_base + L2X0_CTRL) & 1)) {
		/* restore pl310 setup */
                smc_l2x0_tag_lat(l2x0_saved_regs.tag_latency);
                smc_l2x0_dat_lat(l2x0_saved_regs.data_latency);
                smc_l2x0_adr_fil_end(l2x0_saved_regs.filter_end);
                smc_l2x0_adr_fil_start(l2x0_saved_regs.filter_start);

                l2x0_revision = readl_relaxed(l2x0_base + L2X0_CACHE_ID) &
                        L2X0_CACHE_ID_RTL_MASK;

                if (l2x0_revision >= L2X0_CACHE_ID_RTL_R2P0) {
                        smc_l2x0_pft_ctrl(l2x0_saved_regs.prefetch_ctrl);
                        if (l2x0_revision >= L2X0_CACHE_ID_RTL_R3P0)
                                smc_l2x0_pwr_ctrl(l2x0_saved_regs.pwr_ctrl);
                }
        }

	p7_l2x0_resume();
}
#endif

/*
 * This must be called with L2 cache controller disabled ! Therefore the boot
 * monitor / boot loader must leave it disabled which is the default state at
 * reset time.
 */
static void __init p7_init_caches(void)
{
	void __iomem*   l2c = MMIO_P2V(P7_L2C);
	u32             word;
	u32		tag_val;
	/*
	   Pour les RAMs de DATA:
	   les signaux Osprey->RAM ont un délai de 2 cycles @780MHz =>
                     cela définit SETUP_LAT et WRITE_LAT
	   les signaux RAM->Osprey ont un délai de 4 cycles @780MHz =>
                     la somme des 2 délais définit READ_LAT

	   Pour les RAMs de TAG:
	   les signaux Osprey->RAM ont un délai de 2 cycles @780MHz =>
                     cela définit SETUP_LAT et WRITE_LAT
	   les signaux RAM->Osprey ont un délai de 3 cycles @780MHz =>
                     la somme des 2 délais définit READ_LAT

	   La config optimale lorsque cpu_clk=780MHz:
	   tag_clk=data_clk=390MHz avec les mêmes latences pour data et tag :
	   SETUP_LAT = 0 car 1 cycle @390MHz = 2 cycles @780MHz
	   WRITE_LAT = 0 car 1 cycle @390MHz = 2 cycles @780MHz
	   READ_LAT = 2 car 3 cycles @390MHz >= 5 ou 6 cycles @780MHz
	 */
	/* we do not care if the config is not optimal when cpu or l2c clk
	   are lower
	 */
	u32	p7_l2c_tag = 0x020;
	u32	p7_l2c_data = 0x020;

	p7_check_l2c_lat();
	pr_debug("p7: setting L2C latencies to tag=%08x data=%08x.\n",
	         p7_l2c_tag,
	         p7_l2c_data);
	if (p7_is_nonsecure()) {
		tag_val = (1U << 31);
		tag_val |= (p7_l2c_data << 16);
		tag_val |= p7_l2c_tag;
	} else {
		__raw_writel(p7_l2c_tag, l2c + L2X0_TAG_LATENCY_CTRL);
		__raw_writel(p7_l2c_data, l2c + L2X0_DATA_LATENCY_CTRL);
	}
	__iowmb();

	/*
	 * L2C auxiliary register ; differences with default reset state listed
	 * below:
	 *  - early BRESP enabled (30)
	 *  - instruction prefetch enabled (29)
	 *  - data prefetch enabled (28)
	 *  - full Line of zero Enabled (0)
	 * Cache & ways sizes & numbers autodetected...
	 *
	 * Note note!
	 * ----------
	 * Clearing bit 22 in the PL310 Auxiliary Control register (shared
	 * attribute override enable) has the side effect of transforming Normal
	 * Shared Non-cacheable reads into Cacheable no-allocate reads.
	 * Coherent DMA buffers in Linux always have a Cacheable alias via the
	 * kernel linear mapping and the processor can speculatively load cache
	 * lines into the PL310 controller. With bit 22 cleared, Non-cacheable
	 * reads would unexpectedly hit such cache lines leading to buffer
	 * corruption.
	 * Set bit 22, waiting patch to be merged into up-stream releases...
	 *
	 * Other patches related to PL310 implementation are hanging around on
	 * linux-arm mailing-list (see google search with "Improve the L2 cache
	 * performance when PL310 is used"): keep watching...
	 */
	word = 1UL << L2X0_AUX_CTRL_SHARE_OVERRIDE_SHIFT;

#ifdef CONFIG_ARCH_PARROT7_PL310_BRESP
	word |=  1UL << L2X0_AUX_CTRL_EARLY_BRESP_SHIFT;
#endif
#ifdef CONFIG_ARCH_PARROT7_PL310_IPREF
	word |=  1UL << L2X0_AUX_CTRL_INSTR_PREFETCH_SHIFT;
#endif
#ifdef CONFIG_ARCH_PARROT7_PL310_DPREF
	word |=  1UL << L2X0_AUX_CTRL_DATA_PREFETCH_SHIFT;
#endif
#ifdef CONFIG_ARCH_PARROT7_PL310_ZEROS
	word |=  1UL << L2X0_AUX_CTRL_WR_FULL_LINE_OF_ZERO_SHIFT;
#endif
#ifdef CONFIG_ARCH_PARROT7_PL310_STORE_BUFFER_DEVICE_LIMITATION
	word |= 1UL << L2X0_AUX_CTRL_STORE_BUF_DEV_LIMIT_SHIFT;
#endif

	if (p7_is_nonsecure()) {
		u32	aux;
		aux = readl_relaxed(l2c + L2X0_AUX_CTRL);
		aux |= word;
		l2x0_saved_regs.aux_ctrl = aux;
		smc_l2x0_enable(aux, tag_val);
	}
	
	l2x0_init(l2c, word, 0xffffffff);

	if (p7_is_nonsecure()) {
		outer_cache.set_debug = p7_pl310_set_debug;
		outer_cache.disable = p7_l2x0_disable;
#ifdef CONFIG_PM_SLEEP
		outer_cache.resume = p7_pl310_resume;
#endif

	}
}
#else
#define p7_init_caches()
#endif  /* defined(CONFIG_CACHE_L2X0) */

#if defined(CONFIG_ARCH_PARROT7_L1PREF) || \
    defined(CONFIG_ARCH_PARROT7_PL310_IPREF) || \
    defined(CONFIG_ARCH_PARROT7_PL310_DPREF) || \
    defined(CONFIG_ARCH_PARROT7_PL310_ZEROS)

/* CP15 Auxiliary control register shifts */
#define CP15_AUX_CTLR_L2_PREFETCH_HINT_SHIFT       1
#define CP15_AUX_CTLR_L1_PREFETCH_HINT_SHIFT       2
#define CP15_AUX_CTLR_WR_FULL_LINE_OF_ZERO_SHIFT   3

/*
 * Enable L2 cache prefetch hints & write full line of zero features at core
 * level. This must be done for both cores and after L2C setup.
 */
static void __cpuinit p7_enable_caches(void)
{
    u32 word = 0;

#if defined(CONFIG_ARCH_PARROT7_L1PREF)
    word |=  1UL << CP15_AUX_CTLR_L1_PREFETCH_HINT_SHIFT;
#endif
#if defined(CONFIG_ARCH_PARROT7_PL310_IPREF) || \
    defined(CONFIG_ARCH_PARROT7_PL310_DPREF)
    word |=  1UL << CP15_AUX_CTLR_L2_PREFETCH_HINT_SHIFT;
#endif
#ifdef CONFIG_ARCH_PARROT7_PL310_ZEROS
    word |=  1UL << CP15_AUX_CTLR_WR_FULL_LINE_OF_ZERO_SHIFT;
#endif

    /* TLB ops broadcasting */
    if (p7_is_nonsecure()) {
	word |= 1UL << 0;
    }

    write_actlr(read_actlr() | word);
}
#else
#define p7_enable_caches()
#endif

/****************************
 * CPU GBC power management.
 ****************************/

#define GBC_CPU_RST_REQ                     (1U)
#define GBC_CPU_RST_RDY                     (1U << 1)

#define GBC_CPU_PWR_REQ                     (1U)
#define GBC_CPU_PWR_ON                      (1U << 1)
#define GBC_CPU_PWR_OFF                     (1U << 2)

#define GBC_CPU_STATUS_SHUTDOWN_PWRMODE_0   (3U)
#define GBC_CPU_STATUS_SHUTDOWN_PWRMODE_1   (3U << 2)

#define GBC_CPU_STATUS_DBGNOPWRDWN_0        (1U << 8)
#define GBC_CPU_STATUS_DBGNOPWRDWN_1        (1U << 9)

/* return 1 : ok, 0 failure */
static int p7_wait_regmsk(unsigned long vaddr, unsigned long mask)
{
	int timeout = 20000;

	do {
		unsigned long const val = readl(vaddr);
        if ((val & mask) == mask)
			return 1;
        udelay(1);
	} while (timeout--);

	return 0;
}

static int __maybe_unused p7_cpublk_is_on(unsigned long offset)
{
	unsigned long const reg_addr = __MMIO_P2V(P7_CPU_GBC) + offset;
	/* Powered on and in reset state ? */
	return readl(reg_addr) & GBC_CPU_PWR_ON;
}

static int  p7_cpublk_is_off(unsigned long offset)
{
	unsigned long const reg_addr = __MMIO_P2V(P7_CPU_GBC) + offset;
	/* Powered off ? */
	return readl(reg_addr) & GBC_CPU_PWR_OFF;
}

static int p7_cpublk_in_debug(void)
{
#ifdef CPU1_NO_PWDN
	return 1;
#endif
	return readl(__MMIO_P2V(P7_CPU_GBC) + CPU_GBC_CORE_STATUS) & (GBC_CPU_STATUS_DBGNOPWRDWN_0|GBC_CPU_STATUS_DBGNOPWRDWN_1);
}

static void p7_pwroff_cpublk(unsigned long offset)
{
	unsigned long const reg_addr = __MMIO_P2V(P7_CPU_GBC) + offset;
	/* Instruct CPU GBC to assert block's reset input pin. */
	if (offset != CPU_GBC_DBGVSOC_POWER)
		__raw_writel(GBC_CPU_RST_REQ, reg_addr + sizeof(u32));
	/* Instruct CPU GBC to switch block off. */
	__raw_writel(0, reg_addr);

	if (!p7_wait_regmsk(reg_addr, GBC_CPU_PWR_OFF)) {
		printk("power off 0x%lx failed\n", offset);
	}
}

static void __maybe_unused p7_pwron_cpublk(unsigned long offset)
{
	unsigned long const reg_addr = __MMIO_P2V(P7_CPU_GBC) + offset;
	/* Instruct CPU GBC to assert block's reset input pin. */
	if (offset != CPU_GBC_DBGVSOC_POWER)
		__raw_writel(GBC_CPU_RST_REQ, reg_addr + sizeof(u32));
	/* Request power on. */
	writel(GBC_CPU_PWR_REQ, reg_addr);
	/*
	 * Then wait untill GBC tells us block is switched on: when powering on,
	 * consider operation cannot fail.
	 */
	if (!p7_wait_regmsk(reg_addr, GBC_CPU_PWR_ON))
		printk("power on 0x%lx failed\n", offset);
}

/*
 * DBGVSOC power handling will disable bypassing the cdbgpwrupack output
 * to prevent from freezing the design when APB debug accesses are performed
 * through external Coresight block CSSYS (see "Osprey MiD clamp issues"
 * errata).
 */

static void p7_pwron_dbgvsoc(void)
{
	if (p7_cpublk_is_on(CPU_GBC_DBGVSOC_POWER))
		return;

	pr_debug("p7: powering debug on\n");

	p7_pwron_cpublk(CPU_GBC_DBGVSOC_POWER);
}

static void p7_pwroff_mbist(void)
{
	if (p7_cpublk_is_off(CPU_GBC_MBIST_POWER))
		return;

	pr_debug("p7: powering built-in self test off\n");
	p7_pwroff_cpublk(CPU_GBC_MBIST_POWER);
}

#ifdef CONFIG_HOTPLUG_CPU
#include <asm/cacheflush.h>

static void p7_pwron_dbg(void)
{
	if (p7_cpublk_is_on(CPU_GBC_DEBUG_POWER))
		return;

	pr_debug("p7: powering traces on\n");
	p7_pwron_cpublk(CPU_GBC_DEBUG_POWER);
}

static void p7_pwron_neon(int cpu)
{
	unsigned long const offset = cpu ?
	                                CPU_GBC_NEON1_POWER :
	                                CPU_GBC_NEON0_POWER;

	BUG_ON(cpu != 0 && cpu != 1);
	if (p7_cpublk_is_on(offset))
		return;

	pr_debug("p7: powering neon%d on\n", cpu);
	p7_pwron_cpublk(offset);
}

static void p7_pwroff_dbg(void)
{
	if (p7_cpublk_is_off(CPU_GBC_DEBUG_POWER))
		return;

	pr_debug("p7: powering traces off\n");
	/*
	 * FIXME: Osprey ATB bridge needs to be flushed before powering down.
	 * Otherwise, ATB transactions might still be in the process, preventing the
	 * debug block from powering down. This is normaly handled in hardware, but
	 * necessary signals are not connected on MPW1 (fixed in MPW2).
	 * see bench_top/osprey/INT_osprey_11/INT_osprey_11.c for sample code.
	 */
	p7_pwroff_cpublk(CPU_GBC_DEBUG_POWER);
}

static void p7_pwroff_dbgvsoc(void)
{
	if (p7_cpublk_is_off(CPU_GBC_DBGVSOC_POWER))
		return;

	pr_debug("p7: powering debug off\n");

	p7_pwroff_cpublk(CPU_GBC_DBGVSOC_POWER);
}

static void p7_pwroff_neon(int cpu)
{
	unsigned long const offset = cpu ?
	                                CPU_GBC_NEON1_POWER :
	                                CPU_GBC_NEON0_POWER;

	BUG_ON(cpu != 0 && cpu != 1);
	if (p7_cpublk_is_off(offset))
		return;

	pr_debug("p7: powering neon%d off\n", cpu);
	p7_pwroff_cpublk(offset);
}

/*
 * Called by primary cpu from 3 pathes:
 * - __cpu_die: system wants to unplug cpu passed in argument (likely to save
 *   power)
 * - smp_kill_cpus: system is rebooting and primary cpu has just requested
 *   secondary cpus to stop (using an IPI_CPU_STOP inter processor interrupt)
 * - smp_kill_cpus: system is under panic context and both cpus may get killed.
 */
int platform_cpu_kill(unsigned int cpu)
{
	if (cpu != 1)
		/* We are CPU1 and we entered panic, requesting CPU0 to stop. */
		return 0;

	if (p7_cpublk_is_off(CPU_GBC_CPU1_POWER))
		/* CPU1 is already off. */
		return 0;

	/*
	 * Wait till core / SCU request power off: PWRCTLOn bus signals CPU
	 * GBC when it is time to switch off.
	 */
	if (! p7_wait_regmsk(__MMIO_P2V(P7_CPU_GBC) + CPU_GBC_CORE_STATUS,
	                     GBC_CPU_STATUS_SHUTDOWN_PWRMODE_1))
		/*
		 * CPU1 did not shut down, meaning we are in the process of
		 * (re)booting * / halting / powering down the whole system...
		 */
		return 0;

	if (p7_cpublk_in_debug()) {
		pr_debug("p7: powering core1 rst\n");
		/* Instruct CPU GBC to assert block's reset input pin. */
		__raw_writel(GBC_CPU_RST_REQ, __MMIO_P2V(P7_CPU_GBC) + CPU_GBC_CPU1_RESET);
		return 1;
	}

	pr_debug("p7: powering core1 off\n");
    p7_pwroff_cpublk(CPU_GBC_CPU1_POWER);

	return 1;
}

void platform_cpu_die(unsigned int cpu)
{
	BUG_ON(cpu != 1);
	pr_debug("p7: powering CPU1 off\n");

	/* Switch off all required Osprey units.
	   See Osprey integration 3-22 */
	if (!p7_cpublk_in_debug()) {
		p7_pwroff_dbg();
		p7_pwroff_dbgvsoc();
		p7_pwroff_neon(1);
	}

	/* Clean and invalidate all cache hierarchy. */
	flush_cache_all();
#if 0
#warning remove me ?
	/* Get out of coherency array. */
    write_actlr(read_actlr() & ~0x41);
    /* Disable data cache. */
    write_sctlr(read_sctlr() & ~CR_C);
#endif
	/*
	 * Tell SCU we are about to cut the core's power down. This will also get
	 * the core out of coherency array implicitly.
	 */
	scu_power_mode(MMIO_P2V(P7_CPU_SCU), SCU_PM_POWEROFF);
	/* Wait for CPU0 to shut us down. */
	cpu_do_idle();

	/* We should never return from idle */
	panic("p7: CPU%d unexpectedly exit from shutdown\n", cpu);
}

int platform_cpu_disable(unsigned int cpu)
{
	/*
	 * we don't allow CPU 0 to be shutdown (it is still too special
	 * e.g. clock tick interrupts)
	 */
	return cpu == 0 ? -EPERM : 0;
}
#endif  /* CONFIG_HOTPLUG_CPU */

#ifdef CONFIG_SMP
#include <linux/smp.h>
#include <asm/localtimer.h>
#include <asm/hardware/gic.h>
#include "clock.h"

extern void secondary_startup(void);

void __cpuinit platform_secondary_init(unsigned int cpu)
{
	BUG_ON(cpu != 1);

	/*
	 * if any interrupts are already enabled for the primary
	 * core (e.g. timer irq), then they will not have been enabled
	 * for us: do so
	 */
	gic_secondary_init(0);

    __raw_writel(1, MMIO_P2V(P7_SYS_SCRATCH_REG1));

    p7_enable_caches();

    /* Enable Neon floating point unit. */
    p7_enable_neonclk(1);
}

int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	void* __iomem const scu = MMIO_P2V(P7_CPU_SCU);
	unsigned long const cpu_addr = __MMIO_P2V(P7_CPU_GBC) + CPU_GBC_CPU1_POWER;

	BUG_ON(cpu != 1);

	if (!p7_cpublk_is_on(CPU_GBC_CPU1_POWER)) {
		pr_debug("p7: powering core1 on\n");
		p7_pwron_cpublk(CPU_GBC_CPU1_POWER);

		p7_pwron_neon(1);
		p7_pwron_dbgvsoc();
		p7_pwron_dbg();
	}

	/* cpu should be in reset state */
	if (!(readl(cpu_addr + sizeof(u32)) & 1)) {
		return -ENOSYS;
	}

		{
			/*
			 * Tell SCU we are about power up: this will implicitly get the
			 * core back into coherency array.
			 */
			unsigned long const addr = (unsigned long) scu +
			                           0x08 +           /* SCU_CPU_STATUS */
			                           1;               /* CPU no */
			writeb_relaxed(readb_relaxed(addr) & ~0x03, /* SCU_PM_NORMAL */
			               addr);
			isb();
		}

	/* Now setup CPU1 bootstrap address. */
	__raw_writel(virt_to_phys(secondary_startup),
		     MMIO_P2V(P7_SYS_SCRATCH_REG1));
	/* De-reset core. */
	__raw_writel(0, cpu_addr + sizeof(u32));
	if (!p7_wait_regmsk(cpu_addr + sizeof(u32), GBC_CPU_RST_RDY)) {
		pr_debug("can't de-reset core1\n");
	}
    dsb_sev();

	if (p7_wait_regmsk(__MMIO_P2V(P7_SYS_SCRATCH_REG1), 1U))
		/* Success. */
		return 0;

    return -ENOSYS;
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
void __init smp_init_cpus(void)
{
	unsigned int        c;
	unsigned int const  ncores = min_t(unsigned int,
	                                   NR_CPUS,
	                                   scu_get_core_count(MMIO_P2V(P7_CPU_SCU)));

	for (c = 0; c < ncores; c++)
		set_cpu_possible(c, true);

	set_smp_cross_call(gic_raise_softirq);
}

void platform_smp_prepare_cpus(unsigned int max_cpus)
{
	if (max_cpus > 1) {
#if defined(CONFIG_ARCH_PARROT7_PL310_SPECULATIVE_READ)
		u32 scu_ctrl = __raw_readl(MMIO_P2V(P7_CPU_SCU)) | 1 << 3;
		if (!(scu_ctrl & 1))
			__raw_writel(scu_ctrl, MMIO_P2V(P7_CPU_SCU));
#endif
		scu_enable(MMIO_P2V(P7_CPU_SCU));
	}
}
#endif  /* CONFIG_SMP */

void p7_restart(char mode, char const* cmd)
{
	/* make sure REBOOT_P7MU pin is set */
#define P7_SYS_PADCTRL_IO_94 (P7_SYS_PADCTRL_IO + UL(4 * 94))
	u32 tmp = readl(__MMIO_P2V(P7_SYS_PADCTRL_IO_94));
	if (tmp & 3)
		writel(tmp & (~3), __MMIO_P2V(P7_SYS_PADCTRL_IO_94));

	/* Try chip reset. */
	__raw_writel(1U, __MMIO_P2V(P7_SYS_WATCHDOG));
	mdelay(200);

	pr_warning("p7: hard reset failed, halting...\n");
	while(1);
}

static int __init p7_init_cores(void)
{
	/* Power on core units. */
	p7_pwron_neon(0);
	p7_pwron_cpublk(CPU_GBC_CPU1_POWER);
	p7_pwron_neon(1);

	p7_pwron_dbgvsoc();
	p7_pwron_dbg();

	/* Except for Memory Built-In Self Test. */
	p7_pwroff_mbist();

	/* Enable Neon floating point unit for CPU 0. */
	p7_enable_neonclk(0);

	p7_init_caches();
	p7_enable_caches();

	return 0;
}
early_initcall(p7_init_cores);
