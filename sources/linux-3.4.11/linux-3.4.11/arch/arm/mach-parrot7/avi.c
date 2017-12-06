/**
 * linux/arch/arm/mach-parrot7/avi.c - Parrot7 AVI platform implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Lionel Flandrin <lionel.flandrin@parrot.com>
 * date:    18-Sep-2012
 *
 * This file is released under the GPL
 */

#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/dma-mapping.h>
#include <video/avi.h>
#include <mach/p7.h>
#include <mach/irqs.h>
#include "pinctrl.h"
#include "common.h"
#include "avi.h"

static struct resource p7_avi_res = {
	.start  = P7_AVI,
	.end    = P7_AVI + (96 * SZ_4K) - 1,
	.flags  = IORESOURCE_MEM
};

static struct platform_device p7_avi_dev = {
	.name           = "avi",
	.id             = 0,
	.resource       = &p7_avi_res,
	.num_resources  = 1,
};

static struct avi_platform_data p7_avi_pdata;

int __init p7_init_avi(void)
{
	static int init_done = 0;
	if (init_done)
		return 0;
	init_done = 1;
	switch(p7_chiprev())
	{
		case P7_CHIPREV_R1:
			p7_avi_pdata.revision = AVI_REVISION_1;
			break;
		case P7_CHIPREV_R2:
			p7_avi_pdata.revision = AVI_REVISION_2;
			break;
		case P7_CHIPREV_R3:
			p7_avi_pdata.revision = AVI_REVISION_3;
			break;
		default:
			BUG();
	}

	/* The first interrupt in the AVI is the CFG */
	p7_avi_pdata.irq_base = P7_CFG_IRQ;

	return p7_init_dev(&p7_avi_dev, &p7_avi_pdata, NULL, 0);
}

#define INIT_PARAM_ZERO {0,0,0,0,0,0,0,0}
#define INIT_PARAM_MINUSONE {-1,-1,-1,-1,-1,-1,-1,-1}

static int __init p7_avi_parse_param(unsigned params[][AVIFB_MAX_OVERLAYS],
				     char *options,
				     const int max_instances,
				     const int max_sub_instances)
{
	/* The format is as follows: [I[.O]:]M
	 *
	 * M is the value of the parameter, I is an optional driver
	 * instance and O an optional sub-instance index.
	 *
	 * If the driver instance or the sub-instance are not provided they default
	 * to all instance and all sub-instances respectively.
	 */
	char	*param;
	char	*filter;
	char	*instance;
	char	*sub_instance;
	char	*last;
	int	 in = -1;
	int	 ov = -1;
	int	 ma;
	int	 i, o;

	filter = strsep(&options, ":");
	param = strsep(&options, ":");
	if (!filter)
		return -EINVAL;

	if (param == NULL) {
		/* no instance specifier */
		param = filter;
	} else {
		instance      = strsep(&filter, ".");
		sub_instance  = strsep(&filter, ".");

		if (instance) {
			in = simple_strtoul(instance, &last, 0);
			if (*last || in >= max_instances)
				return -EINVAL;
		}

		if (sub_instance) {
			ov = simple_strtoul(sub_instance, &last, 0);
			if (*last || ov >= max_sub_instances)
				return -EINVAL;
		}
	}

	ma = simple_strtoul(param, &last, 0);
	if (*last)
		return -EINVAL;

	for (i = ((in < 0) ? 0             : in);
	     i < ((in < 0) ? max_instances : in + 1);
	     i++)
		for (o = ((ov < 0) ? 0                  : ov);
		     o < ((ov < 0) ? max_sub_instances : ov + 1);
		     o++)
			params[i][o] = ma;

	return 0;
}

#if defined(CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE)

static int __init p7_avifb_parse_param(unsigned params[][AVIFB_MAX_OVERLAYS],
                                       char *options)
{
	/* The format is as follows: [I[.O]:]M
	 *
	 * M is the value of the parameter in pixels, I is an optional driver
	 * instance and O an optional overlay index.
	 *
	 * If the driver instance or the overlay are not provided they default
	 * to all instance and all overlays respectively.
	 */
	return p7_avi_parse_param(params,
				  options,
				  AVIFB_MAX_INSTANCES,
				  AVIFB_MAX_OVERLAYS);
}
#define INIT_FB_PARAM_ZERO {INIT_PARAM_ZERO,INIT_PARAM_ZERO}
#define INIT_FB_PARAM_MINUSONE {INIT_PARAM_MINUSONE,INIT_PARAM_MINUSONE}

#define P7_AVI_MAKE_FB_PARAM(_param, _init)				\
	unsigned p7_avifb_##_param[AVIFB_MAX_INSTANCES]			\
	                          [AVIFB_MAX_OVERLAYS]			\
	                                 = _init;			\
									\
	static int __init p7_avifb_parse_##_param(char *options)	\
	{								\
		return p7_avifb_parse_param(p7_avifb_##_param, 		\
					     options);			\
        }								\
									\
        early_param("p7_avifb_" #_param, p7_avifb_parse_##_param);

P7_AVI_MAKE_FB_PARAM(width, INIT_FB_PARAM_ZERO)
P7_AVI_MAKE_FB_PARAM(height, INIT_FB_PARAM_ZERO)
P7_AVI_MAKE_FB_PARAM(position_x, INIT_FB_PARAM_ZERO)
P7_AVI_MAKE_FB_PARAM(position_y, INIT_FB_PARAM_ZERO)
P7_AVI_MAKE_FB_PARAM(zorder, INIT_FB_PARAM_MINUSONE)

int __init p7_init_avifb(struct platform_device		*pdev,
			 struct avifb_platform_data	*pdata,
			 struct pinctrl_map		*pins,
			 size_t				 pin_cnt)
{
	int id = pdev->id;
	int i;

	BUG_ON(id                >= AVIFB_MAX_INSTANCES);
	BUG_ON(pdata->overlay_nr >= AVIFB_MAX_OVERLAYS);

	p7_init_avi();

	for (i = 0; i < pdata->overlay_nr; i++) {
		unsigned width  = p7_avifb_width     [id][i];
		unsigned height = p7_avifb_height    [id][i];
		unsigned pos_x  = p7_avifb_position_x[id][i];
		unsigned pos_y  = p7_avifb_position_y[id][i];
		int      zorder = (int)p7_avifb_zorder[id][i];

		if (width)
			pdata->overlays[i].layout.width  = width;
		if (height)
			pdata->overlays[i].layout.height = height;

		if (zorder>=0)
			pdata->overlays[i].zorder        = zorder;

		/* XXX: there is actually a limitation here: if the BSP
		 * specifies one of these values as being != 0 and the user
		 * wants to reset them to 0 using the command line that won't
		 * work since the setting will be ignored. Maybe I should set a
		 * flag when the variable is present instead of checking if it's
		 * non-0. */
		if (pos_x)
			pdata->overlays[i].layout.x	 = pos_x;
		if (pos_y)
			pdata->overlays[i].layout.y	 = pos_y;
	}

	return p7_init_dev(pdev, pdata, pins, pin_cnt);
}


void __init p7_reserve_avifbmem(struct platform_device *pdev,
                                struct avifb_overlay *overlays,
                                int n)
{
	int				 i;

	for (i = 0; i < n; i++) {
		p7_reserve_devmem(pdev,
		                  &overlays[i].dma_memory.start,
		                  &overlays[i].dma_memory.end);
		overlays[i].dma_memory.end += overlays[i].dma_memory.start - 1;
	}
}

#endif /* defined(CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE) */

#if defined(CONFIG_CAM_AVI) || defined(CONFIG_CAM_AVI_MODULE)

int __init p7_init_avicam(struct platform_device	*pdev,
                          struct avicam_platform_data	*pdata,
                          struct pinctrl_map		*pins,
                          size_t			 pin_cnt)
{
	p7_init_avi();
	return p7_init_dev(pdev, pdata, pins, pin_cnt);
}

/**
 * p7_reserve_avicammem() - Reserve DMA memory region for avicam usage.
 *
 * @size: size of DMA region in bytes
 */
void __init p7_reserve_avicammem(struct platform_device *pdev, size_t size)
{
	/* Reserve memory in the global DMA contiguous pool */
	p7_reserve_devmem(pdev, NULL, &size);
}

#endif /* defined(CONFIG_CAM_AVI) || defined(CONFIG_CAM_AVI_MODULE) */

#if defined(CONFIG_VIDOUT_AVI) ||  defined(CONFIG_VIDOUT_AVI_MODULE)

static u64 p7_avi_voc_dma_mask = DMA_BIT_MASK(32);

static struct platform_device p7_avi_voc_dev[] = {
	{
		.name           = "avi-voc",
		.id             = 0,
		.dev            = {
			/* Todo: try to set a narrower mask */
			.dma_mask           = &p7_avi_voc_dma_mask,
			.coherent_dma_mask  = DMA_BIT_MASK(32)
		}
	},
	{
		.name           = "avi-voc",
		.id             = 1,
		.dev            = {
			/* Todo: try to set a narrower mask */
			.dma_mask           = &p7_avi_voc_dma_mask,
			.coherent_dma_mask  = DMA_BIT_MASK(32)
		}
	},
	{
		.name           = "avi-voc",
		.id             = 2,
		.dev            = {
			/* Todo: try to set a narrower mask */
			.dma_mask           = &p7_avi_voc_dma_mask,
			.coherent_dma_mask  = DMA_BIT_MASK(32)
		}
	}
};

struct avi_voc_dma {
	dma_addr_t      addr;
	size_t          size;
};

static struct avi_voc_dma       avi_voc_mem[ARRAY_SIZE(p7_avi_voc_dev)] __initdata;


/**
 * p7_reserve_avi_voc_mem() - Reserve DMA memory region for video encoder usage.
 *
 * @size: size of DMA region in bytes
 */
void __init p7_reserve_avi_voc_mem(int id, size_t size)
{
	BUG_ON(id < 0);
	BUG_ON(id >= ARRAY_SIZE(p7_avi_voc_dev));

	avi_voc_mem[id].size = size;
	p7_reserve_devmem(&p7_avi_voc_dev[id],
			  NULL,
			  &avi_voc_mem[id].size);
}


/**
 * p7_init_avi_voc() - Instantiate Video encoder for further driver usage.
 */
int __init p7_init_avi_voc(int id, struct avi_voc_plat_data *pdata)
{
	int err;

	BUG_ON(id >= ARRAY_SIZE(p7_avi_voc_dev));

	p7_init_avi();

	err = p7_init_dev(&p7_avi_voc_dev[id], pdata, NULL, 0);
	if (err)
		panic("p7: failed to init avi_voc (%d)\n", err);

	return 0;
}

#endif /* defined(CONFIG_VIDOUT_AVI) ||  defined(CONFIG_VIDOUT_AVI_MODULE) */

#if defined(CONFIG_R2R_AVI) || defined(CONFIG_R2R_AVI_MODULE)

static u64 p7_avi_r2r_dma_mask = DMA_BIT_MASK(32);
static struct platform_device p7_avi_r2r_dev = {
	.name           = AVI_R2R_DRIVER_NAME,
	.id             = 0,
	.dev            = {
		.dma_mask          =
			&p7_avi_r2r_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32)
	}
};

/**
 * p7_init_avi_r2r() - Instantiate AVI R2R further driver usage.
 */
void __init p7_init_avi_r2r(void* pdata)
{
	int err;

	p7_init_avi();
	err = p7_init_dev(&p7_avi_r2r_dev, pdata, NULL, 0);
	if (err)
		panic("p7: failed to init %s (%d)\n",
		      dev_name(&p7_avi_r2r_dev.dev),
		      err);
}

#endif /* defined(CONFIG_R2R_AVI) || defined(CONFIG_R2R_AVI_MODULE) */

#if defined(CONFIG_R2R_AVI_TEST_MULTI) || defined(CONFIG_R2R_AVI_TEST_MULTI_MODULE)

#if CONFIG_R2R_AVI_TEST_MULTI_NUMBER > 0
static struct resource p7_avi_r2r_test_multi_resource[6] = {
	{/* 0 */
		.flags = IORESOURCE_MEM,
		.start = 0,
		.end   = CONFIG_R2R_AVI_TEST_MULTI_RESMEM * 1024 * 1024
	},
	{/* 1 */
		.flags = IORESOURCE_MEM,
		.start = 0,
		.end   = CONFIG_R2R_AVI_TEST_MULTI_RESMEM * 1024 * 1024
	},
	{/* 2 */
		.flags = IORESOURCE_MEM,
		.start = 0,
		.end   = CONFIG_R2R_AVI_TEST_MULTI_RESMEM * 1024 * 1024
	},
	{/* 3 */
		.flags = IORESOURCE_MEM,
		.start = 0,
		.end   = CONFIG_R2R_AVI_TEST_MULTI_RESMEM * 1024 * 1024
	},
	{/* 4 */
		.flags = IORESOURCE_MEM,
		.start = 0,
		.end   = CONFIG_R2R_AVI_TEST_MULTI_RESMEM * 1024 * 1024
	},
	{/* 5 */
		.flags = IORESOURCE_MEM,
		.start = 0,
		.end   = CONFIG_R2R_AVI_TEST_MULTI_RESMEM * 1024 * 1024
	}
};

static u64 p7_avi_r2r_test_multi_dma_mask[6] = {
		DMA_BIT_MASK(32),
		DMA_BIT_MASK(32),
		DMA_BIT_MASK(32),
		DMA_BIT_MASK(32),
		DMA_BIT_MASK(32),
		DMA_BIT_MASK(32)
};

#define AVI_R2R_TEST_MULTI_DEVICE(_id) 				\
	{							\
		.name           = "avi_r2r_test_multi",		\
		.id             = _id,				\
		.resource       =				\
			&p7_avi_r2r_test_multi_resource[_id],	\
		.num_resources  = 1,				\
		.dev            = {				\
			.dma_mask =				\
			&p7_avi_r2r_test_multi_dma_mask[_id],	\
			.coherent_dma_mask  = DMA_BIT_MASK(32)	\
		}						\
	}
static struct platform_device p7_avi_r2r_test_multi_dev[6] = {
		AVI_R2R_TEST_MULTI_DEVICE(0),
		AVI_R2R_TEST_MULTI_DEVICE(1),
		AVI_R2R_TEST_MULTI_DEVICE(2),
		AVI_R2R_TEST_MULTI_DEVICE(3),
		AVI_R2R_TEST_MULTI_DEVICE(4),
		AVI_R2R_TEST_MULTI_DEVICE(5)
};

void __init p7_reserve_avi_r2r_mem_test_multi(void)
{
	int i;
	for (i = 0; i < CONFIG_R2R_AVI_TEST_MULTI_NUMBER; i++) {
		p7_reserve_devmem(&p7_avi_r2r_test_multi_dev[i],
				  &p7_avi_r2r_test_multi_resource[i].start,
				  &p7_avi_r2r_test_multi_resource[i].end);
		p7_avi_r2r_test_multi_resource[i].end +=
				p7_avi_r2r_test_multi_resource[i].start -1 ;
	}
}
void __init p7_init_avi_r2r_test_multi(void)
{
	int i, err;
	for (i = 0; i < CONFIG_R2R_AVI_TEST_MULTI_NUMBER; i++) {
		err = p7_init_dev(&p7_avi_r2r_test_multi_dev[i], 0, 0, 0);
		if (err)
			panic("p7: failed to init %s (%d)\n",
			      dev_name(&p7_avi_r2r_test_multi_dev[i].dev),
			      err);
	}
}
#else
#warning "Number of clients is 0 for test module of AVI R2R"
void __init p7_reserve_avi_r2r_mem_test_multi(void)
{
	pr_warn("p7: Number of clients is 0 for test module of AVI R2R");
}
void __init p7_init_avi_r2r_test_multi(void)
{
	pr_warn("p7: Number of clients is 0 for test module of AVI R2R");
}
#endif

#endif /* defined(CONFIG_R2R_AVI_TEST_MULTI) ||
          defined(CONFIG_R2R_AVI_TEST_MULTI_MODULE) */

#if defined(CONFIG_M2M_AVI) || defined(CONFIG_M2M_AVI_MODULE)

static u64 avi_m2m_dma_mask = DMA_BIT_MASK(32);
static struct platform_device p7_avi_m2m_pdev = {
	.name = "avi_m2m",
	.id   = 0,
	.dev  = {
		.dma_mask	   = &avi_m2m_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32)
	}
};

/**
 * p7_init_avi_m2m() - Initialise device AVI M2M
 *
 */
int __init p7_init_avi_m2m(struct avi_m2m_platform_data *pdata)
{
	struct platform_device *pdev;
	int err = 0;
	int i;

	/* Init AVI */
	p7_init_avi();

	/* Register each devices */
	for (i = 0; pdata[i].caps != 0; i++) {
		/* Allocate a new platform device */
		pdev = kzalloc(sizeof(*pdev), GFP_KERNEL);
		*pdev = p7_avi_m2m_pdev;

		/* Register device */
		err = p7_init_dev(pdev, &pdata[i], NULL, 0);
		if (err) {
			panic("p7: failed to init %s (%d)\n",
			      dev_name(&p7_avi_m2m_pdev.dev),
			      err);
		}
		p7_avi_m2m_pdev.id++;
	}

	return err;
}

/**
 * p7_reserve_avi_m2m_mem() - Reserve DMA memory region for AVI M2M.
 *
 * @size: size of DMA region in bytes
 *
 */
void __init p7_reserve_avi_m2m_mem(size_t size)
{
	p7_reserve_devmem(&p7_avi_m2m_pdev, NULL, &size);
}

#endif /* defined(CONFIG_M2M_AVI) || defined(CONFIG_M2M_AVI_MODULE) */

