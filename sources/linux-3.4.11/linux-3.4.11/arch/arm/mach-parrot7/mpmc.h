#ifndef _MPMC_H_
#define _MPMC_H_

enum p7_mpmc_port
{
	/* The value of the enum are in register order (from LSB to MSB) */
	P7_MPMC_CPU_DMA_PORT   = 0,
	P7_MPMC_AVI_PORT       = 1,
	P7_MPMC_VDEC_VENC_PORT = 2,
	P7_MPMC_HSP_AAI_PORT   = 3,
	P7_MPMC_GPU_VENC_PORT  = 4,
	P7_MPMC_NR_PORT,
};

/* Prio is in the range [0, 7] */
extern int	p7_mpmc_set_prio(enum p7_mpmc_port port, unsigned prio);
extern unsigned p7_mpmc_get_prio(enum p7_mpmc_port port);

void __init p7_init_mpmc(void);

#endif /* _MPMC_H_ */
