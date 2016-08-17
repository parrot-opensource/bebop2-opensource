/**
 * linux/arch/arm/mach-parrot7/avi.h - Parrot7 AVI platform implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Lionel Flandrin <lionel.flandrin@parrot.com>
 * date:    18-Sep-2012
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_AVI_H
#define _ARCH_PARROT7_AVI_H

#include <video/avi.h>
#include <video/avi_segment.h>
#include <media/video/avicam.h>
#include <media/video/avi_multicapture.h>
#include <parrot/avicam_dummy_dev.h>
#include <media/video/avi_voc.h>
#include <media/video/avi_r2r.h>
#include <media/video/avi_m2m.h>
#include <video/avifb.h>

#if defined(CONFIG_AVI) || defined(CONFIG_AVI_MODULE)

extern int p7_init_avi(void) __init;
#else

static inline int p7_init_avi(void)
{
	return -ENOSYS;
}

#endif


#if defined(CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE)

extern int  __init p7_init_avifb(struct platform_device*,
	                         struct avifb_platform_data*,
	                         struct pinctrl_map*,
			         size_t);
extern void __init p7_reserve_avifbmem(struct platform_device *,
                                       struct avifb_overlay *,
                                       int);

/* Since the params are parsed globally for all driver instances I have to
 * reserve enough room to store all the param configs */
#define AVIFB_MAX_INSTANCES 2
/* Should be 7 if we need to handle two chained blenders */
#define AVIFB_MAX_OVERLAYS  8

extern unsigned p7_avifb_width     [AVIFB_MAX_INSTANCES][AVIFB_MAX_OVERLAYS];
extern unsigned p7_avifb_height    [AVIFB_MAX_INSTANCES][AVIFB_MAX_OVERLAYS];
extern unsigned p7_avifb_position_x[AVIFB_MAX_INSTANCES][AVIFB_MAX_OVERLAYS];
extern unsigned p7_avifb_position_y[AVIFB_MAX_INSTANCES][AVIFB_MAX_OVERLAYS];

#else

static inline int p7_init_avifb(struct platform_device *pdev,
                                struct avifb_platform_data *pdata,
                                struct pinctrl_map *pins,
                                size_t npins)
{
	return -ENOSYS;
}

static inline void p7_reserve_avifbmem(struct platform_device *pdev,
                                       struct avifb_overlay *o,
                                       int no)
{
	return;
}

#endif

/* segments ID: to be used with avi-voc display parameter */
#define AVIFB_ID(x) "lcd." #x
#define AVIFB_LCD_0_ID AVIFB_ID(0)
#define AVIFB_LCD_1_ID AVIFB_ID(1)

#if defined(CONFIG_CAM_AVI) || defined(CONFIG_CAM_AVI_MODULE)

extern int p7_init_avicam(struct platform_device*,
                          struct avicam_platform_data*,
                          struct pinctrl_map*,
                          size_t) __init;

extern void __init p7_reserve_avicammem(struct platform_device *, size_t);

#else

static inline int p7_init_avicam(struct platform_device *pdev,
	                         struct avicam_platform_data *pdata,
	                         struct pinctrl_map *pins,
	                         size_t npins)

{
	return -ENOSYS;
}

static inline void p7_reserve_avicammem(struct platform_device *pdev,
	                                size_t s)
{
	return;
}

#endif

/* segments ID */
#define AVICAM_ID(x) "cam." #x

#if defined(CONFIG_VIDOUT_AVI) ||  defined(CONFIG_VIDOUT_AVI_MODULE)

extern void __init p7_reserve_avi_voc_mem(int id, size_t size);
extern int __init p7_init_avi_voc(int id, struct avi_voc_plat_data *pdata);

#else

static inline int p7_init_avi_voc(int id, struct avi_voc_plat_data *pdata)
{
	return -ENOSYS;
}

static inline void p7_reserve_avi_voc_mem(int id, size_t size)
{
	return;
}

#endif
/* segments ID for VOC */
#define AVIVOC_ID(x) "voc." #x

#if defined(CONFIG_R2R_AVI) || defined(CONFIG_R2R_AVI_MODULE)

extern void p7_init_avi_r2r(void* pdata) __init;

#else

static inline void p7_init_avi_r2r(void *pdata)
{
	return;
}

#endif

#if defined(CONFIG_R2R_AVI_TEST_MULTI) || \
	defined(CONFIG_R2R_AVI_TEST_MULTI_MODULE)

extern void p7_init_avi_r2r_test_multi(void) __init;
extern void p7_reserve_avi_r2r_mem_test_multi(void) __init;

#else

static inline void p7_init_avi_r2r_test_multi(void)
{
	return;
}

static inline void p7_reserve_avi_r2r_mem_test_multi(void)
{
	return;
}

#endif

/* segments ID for R2R: to be used with avi-voc display parameter */
#define AVIR2R_ID(x) "r2r." #x

/* segments ID for M2M: to be used with avi-voc display parameter */
#define AVIM2M_ID(x) "m2m." #x

#if defined(CONFIG_M2M_AVI) || defined(CONFIG_M2M_AVI_MODULE)
extern struct platform_device p7_avi_m2m_dev;
extern int p7_init_avi_m2m(struct avi_m2m_platform_data *pdata) __init;
extern void p7_reserve_avi_m2m_mem(size_t size) __init;

#else

static inline int p7_init_avi_m2m(struct avi_m2m_platform_data *pdata)
{
	return -ENOSYS;
}

static inline void p7_reserve_avi_m2m_mem(size_t size)
{
	return;
}

#endif

#endif /* _ARCH_PARROT7_AVI_H */
