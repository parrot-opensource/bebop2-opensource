#include <asm/errno.h>
#include <asm/io.h>

#include <linux/kernel.h>
#include <linux/bug.h>
#include <linux/syscore_ops.h>

#include <mach/p7.h>

#include "mpmc.h"

int p7_mpmc_set_prio(enum p7_mpmc_port port, unsigned prio)
{
	int off = port * 4;
	u32 r;

	if (prio > 7)
		return -ERANGE;

	r   = __raw_readl(__MMIO_P2V(P7_MPMC_GBC_PRI));
	r  &= ~(7UL << off);
	r  |= prio << off;

	__raw_writel(r, __MMIO_P2V(P7_MPMC_GBC_PRI));

	return 0;
}

unsigned p7_mpmc_get_prio(enum p7_mpmc_port port)
{
	int off = port * 4;
	u32 r;

	r   = __raw_readl(__MMIO_P2V(P7_MPMC_GBC_PRI));
	r >>= off;
	r  &= 7UL;

	return r;
}


#ifdef CONFIG_PM

static unsigned p7_mpmc_config[P7_MPMC_NR_PORT];

/* Save PRIO values on suspend */
static int p7_mpmc_suspend(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(p7_mpmc_config); i++)
		p7_mpmc_config[i] = p7_mpmc_get_prio(i);

	return 0;
}

/* Restore PRIO values on resume */
static void p7_mpmc_resume(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(p7_mpmc_config); i++)
		p7_mpmc_set_prio(i, p7_mpmc_config[i]);
}

struct syscore_ops p7_mpmc_syscore_ops = {
	.suspend = p7_mpmc_suspend,
	.resume  = p7_mpmc_resume,
};


#endif /* CONFIG_PM */

void __init p7_init_mpmc(void)
{
	/* Default MPMC configuration. We set all ports to priority 1 to disable
	 * QoS. */
	p7_mpmc_set_prio(P7_MPMC_CPU_DMA_PORT,   1);
	p7_mpmc_set_prio(P7_MPMC_AVI_PORT,       1);
	p7_mpmc_set_prio(P7_MPMC_VDEC_VENC_PORT, 1);
	p7_mpmc_set_prio(P7_MPMC_HSP_AAI_PORT,   1);
	p7_mpmc_set_prio(P7_MPMC_GPU_VENC_PORT,  1);

#ifdef CONFIG_PM_SLEEP
	register_syscore_ops(&p7_mpmc_syscore_ops);
#endif
}
