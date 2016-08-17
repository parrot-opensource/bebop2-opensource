/**
 *
 *	@file  p7.c
 *
 *	@brief  Versatile P7 BSP adaptation implementation
 *
 *	@author  Lionel Flandrin <lionel.flandrin@parrot.com>
 *	@date  10-Oct-2011
 *
 *	@author  Didier Leymarie <didier.leymarie.ext@parrot.com>
 *	@date  10-Oct-2013
 *
 *	$Id:$
 *
 *	Copyright (C) 2008 Parrot S.A.
 *	Copyright (C) 2011 Parrot S.A.
 *	Copyright (C) 2013 Parrot S.A.
 */

#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <asm/setup.h>
#include <asm/mach/map.h>
#include <asm/pgtable.h>
#include <mach/irqs.h>
#include <mach/clkdev.h>
#include <asm/delay.h>
#include <linux/delay.h>
#include <linux/pinctrl/pinctrl-state.h>
#include "core.h"

/* Warning: IRQ 68 is shared between all fifos. */
#define P7_VDEC_IRQ 71
#define P7_I2CS_IRQ 71
#define P7_AVI_FIFO_IRQ 68 /* All AVI FIFO's IRQs are OR'd */
#define P7_AVI_LCD_IRQ  69
#define P7_AVI_CAM_IRQ 70
#define P7_I2CM_IRQ 70

#define P7_CHIPREV_R1 0
#define P7_CHIPREV_R2 1
#define P7_CHIPREV_R3 2

int p7_chiprev(void)
{
	return P7_CHIPREV_R3;
}

#define SPI_REVISION    SPI_REVISION_3
#define I2CM_REVISION   I2CM_REVISION_3

#if defined (CONFIG_AVI) || defined(CONFIG_AVI_MODULE)
#include <video/avi.h>
#include <video/avi_segment.h>
#include "camera-sensor-common.h"
#include "lcd-monspecs.h"
#include "board-p7dev-rnb6-config.h"

#define AVI_REVISION		AVI_REVISION_3
#define P7_AVI_FIFO00_IRQ	P7_AVI_FIFO_IRQ
#define P7_AVI_FIFO01_IRQ	P7_AVI_FIFO_IRQ
#define P7_AVI_FIFO02_IRQ	P7_AVI_FIFO_IRQ
#define P7_AVI_FIFO03_IRQ	P7_AVI_FIFO_IRQ
#define P7_AVI_FIFO04_IRQ	P7_AVI_FIFO_IRQ
#define P7_AVI_FIFO05_IRQ	P7_AVI_FIFO_IRQ
#define P7_AVI_FIFO06_IRQ	P7_AVI_FIFO_IRQ
#define P7_AVI_FIFO07_IRQ	P7_AVI_FIFO_IRQ
#define P7_AVI_FIFO08_IRQ	P7_AVI_FIFO_IRQ
#define P7_AVI_FIFO09_IRQ	P7_AVI_FIFO_IRQ
#define P7_AVI_FIFO10_IRQ	P7_AVI_FIFO_IRQ
#define P7_AVI_FIFO11_IRQ	P7_AVI_FIFO_IRQ

#define P7_AVI_LCD0_IRQ		P7_AVI_LCD_IRQ
#define P7_AVI_LCD1_IRQ		P7_AVI_LCD_IRQ

#define P7_AVI_CAM0_IRQ		P7_AVI_CAM_IRQ
#define P7_AVI_CAM1_IRQ		P7_AVI_CAM_IRQ
#define P7_AVI_CAM2_IRQ		P7_AVI_CAM_IRQ
#define P7_AVI_CAM3_IRQ		P7_AVI_CAM_IRQ
#define P7_AVI_CAM4_IRQ		P7_AVI_CAM_IRQ
#define P7_AVI_CAM5_IRQ		P7_AVI_CAM_IRQ

#endif /* defined (CONFIG_AVI) || defined(CONFIG_AVI_MODULE) */


//#undef CONFIG_AVI
//#undef CONFIG_AVI_MODULE
//#undef CONFIG_FB_AVI
//#undef CONFIG_FB_AVI_MODULE
//#undef CONFIG_CAM_AVI
//#undef CONFIG_CAM_AVI_MODULE

#if defined (CONFIG_I2CM_PARROT7) || defined(CONFIG_I2CM_PARROT7_MODULE) \
	|| defined (CONFIG_I2CM_PARROT7_LEGACY) \
	|| defined(CONFIG_I2CM_PARROT7_LEGACY_MODULE)

#include <i2c/p7-i2cm.h>
#include <i2c/p7-i2cs.h>

#define I2CM0_BASE  0xfc010000

#define I2CM_RESOURCE  {						\
		[0] = {									\
			.start = I2CM0_BASE,				\
			.end   = I2CM0_BASE + SZ_64K - 1,	\
			.flags = IORESOURCE_MEM,			\
		},										\
		[1] = {							\
			.start = P7_I2CM_IRQ,		\
			.end   = P7_I2CM_IRQ,		\
			.flags = IORESOURCE_IRQ,	\
		}								\
	}

static struct p7i2cm_plat_data p7_i2cm_platform = {
  .bus_freq      = 100,   /* I2C clock in kHz */
  .high_speed    = false, /* Disable high speed mode */
  .master_id     = 1,     /* ID of the master on the bus */
  .bus_high_freq = 150,   /* I2C high-speed clock in kHz*/
  .revision      = I2CM_REVISION /* revision of IP */
};

static struct resource p7_i2cm0_resource[] = I2CM_RESOURCE;

struct platform_device p7_i2cm0_dev = {
	.name           = "p7-i2cm",
	.id             = 0,
	.resource       = p7_i2cm0_resource,
	.num_resources  = ARRAY_SIZE(p7_i2cm0_resource),
		.dev            = {
		.platform_data  = &p7_i2cm_platform,
	},
};

static struct resource p7_i2cm1_resource[] = I2CM_RESOURCE;
struct platform_device p7_i2cm1_dev = {
	.name           = "p7-i2cm",
	.id             = 1,
	.resource       = p7_i2cm1_resource,
	.num_resources  = ARRAY_SIZE(p7_i2cm1_resource),
		.dev            = {
		.platform_data  = &p7_i2cm_platform,
	},
};

static struct resource p7_i2cm2_resource[] = I2CM_RESOURCE;
struct platform_device p7_i2cm2_dev = {
	.name           = "p7-i2cm",
	.id             = 2,
	.resource       = p7_i2cm2_resource,
	.num_resources  = ARRAY_SIZE(p7_i2cm2_resource),
		.dev            = {
		.platform_data  = &p7_i2cm_platform,
	},
};

static struct resource p7_i2cm3_resource[] = I2CM_RESOURCE;
struct platform_device p7_i2cm3_dev = {
	.name           = "p7-i2cm",
	.id             = 3,
	.resource       = p7_i2cm3_resource,
	.num_resources  = ARRAY_SIZE(p7_i2cm3_resource),
		.dev            = {
		.platform_data  = &p7_i2cm_platform,
	},
};

/* Dummy clock. The rate has to match the clock configured on the Vexpress
 * binary. */
static struct clk p7_i2cm_clk = { .rate = 40000000 };

#endif  /* defined (CONFIG_I2CM_PARROT7) || defined(CONFIG_I2CM_PARROT7_MODULE) */

#if defined (CONFIG_PLDS_I2CS) || defined(CONFIG_PLDS_I2CS_MODULE)

#define I2CS0_BASE  0xf0010000

static struct p7i2cm_plat_data p7_i2cs_platform = {
	.address = 0x11,
	.size    = SZ_256K,
};

static struct resource p7_i2cs0_resource[] = {
	[0] = {
		.start = I2CS0_BASE,
		.end   = I2CS0_BASE + SZ_64K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_I2CS_IRQ,
		.end   = P7_I2CS_IRQ,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device p7_i2cs0_dev = {
	.name           = "p7-i2cs",
	.id             = 0,
	.resource       = p7_i2cs0_resource,
	.num_resources  = ARRAY_SIZE(p7_i2cs0_resource),
		.dev            = {
		.platform_data  = &p7_i2cs_platform,
	},
};

#endif  /* defined (CONFIG_PLDS_I2CS) || defined(CONFIG_PLDS_I2CS_MODULE) */

static long avi_osc_round(struct clk *clk, unsigned long rate)
{
	return rate;
}

static int avi_osc_set(struct clk *clk, unsigned long rate)
{
	return 0;
}

static const struct clk_ops avi_clk_ops = {
	.round	= avi_osc_round,
	.set	= avi_osc_set,
};

#if defined (CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE)
#include <video/avifb.h>

#undef CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD0_ENABLE
#ifdef CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD0_ENABLE

#define P7_AVIFB0_BUFFER_SIZE PAGE_ALIGN(1280*720*4*2)
static struct avifb_overlay rnb6_avi_lcd0_overlays[] = {
	{
		.layout = {
			.alpha = AVI_ALPHA(100),
			.enabled = 1,
		},
		.id		 = "PANEL-main_fb",
		.dma_memory.end = P7_AVIFB0_BUFFER_SIZE,
		.dma_memory.flags = IORESOURCE_MEM,
		.caps		 = 0,
		.zorder		 = -1,
	},
};

/* Reserve enough memory for a single dual-buffered 32bpp full XGA screen */

static struct avifb_platform_data rnb6_avifb0_pdata = {
	.lcd_format_control    = AVI_FORMAT_CONTROL_RGB888_1X24, /* RGB 24 bits */
	.lcd_default_videomode = &versatile_video_mode,
	.lcd_videomodes        = p7_all_video_modes,
	.lcd_interface={{
		.free_run      = 1,
		.itu656        = 0,
	}},
	.id		       = "PANEL-LCD0_out",
	.caps		       = AVI_CAP_LCD_0,
	.overlays	       = rnb6_avi_lcd0_overlays,
	.overlay_nr	       = ARRAY_SIZE(rnb6_avi_lcd0_overlays),
};

static u64 rnb6_avifb0_dma_mask = DMA_BIT_MASK(32);
static struct platform_device rnb6_avifb0_dev = {
	.name   = "avifb",
	.id     = 0,
	.dev    = {
		.platform_data      = &rnb6_avifb0_pdata,
		/* Todo: try to set a narrower mask */
		.dma_mask           = &rnb6_avifb0_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32),
	}
};

/* Dummy clock, have a look at p7_enable_clkgbc to actually enable GBC
 * clocks. This is possible since the plat-versatile clk_enable/disable do
 * nothing. */
static struct clk rnb6_lcd0_clk =
{
	.rate = 52175000,
	.ops	= &avi_clk_ops
};

#endif /* CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD0_ENABLE */

//#define CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD1_ENABLE y
#ifdef CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD1_ENABLE


/*#define P7_AVIFB1_BUFFER_SIZE PAGE_ALIGN(1920*1080*4*2)*/ /* Full HD 16/9 */
/*#define P7_AVIFB1_BUFFER_SIZE PAGE_ALIGN(1280*720*4*2)*/ /* HD 16/9 */
/*#define P7_AVIFB1_BUFFER_SIZE PAGE_ALIGN(1024*768*4*2)*/ /* 1024x768 4/3 */
/*#define P7_AVIFB1_BUFFER_SIZE PAGE_ALIGN(1440*1080*4*2)*/ /* Full HD 4/3 */
/*#define P7_AVIFB1_BUFFER_SIZE PAGE_ALIGN(960*720*4*2)*/  /* HD 4/3 */
/*#define P7_AVIFB1_BUFFER_SIZE PAGE_ALIGN(720*576*4*2)*/ /* SD (PAL) 4/3 */
#define P7_AVIFB1_BUFFER_SIZE PAGE_ALIGN(1024*1024*4*2)  /* Max available with ZBT 8 Mbytes */
static struct avifb_overlay rnb6_avi_lcd1_overlays[] = {
	{
		.layout		 = {
			.alpha	 = AVI_ALPHA(100),
			.enabled = 1,
		},
		.id		 = "HDMI-main_fb",
		.dma_memory.end	 = P7_AVIFB1_BUFFER_SIZE,
		.dma_memory.flags = IORESOURCE_MEM,
		.caps		 = 0, //AVI_CAP_SCAL,
		.zorder		 = -1,
	},
};

static struct avifb_platform_data rnb6_avifb1_pdata = {
	.lcd_format_control    = AVI_FORMAT_CONTROL_UYVY_2X8,
	.output_cspace         = AVI_BT709_CSPACE,
	.lcd_default_videomode =  &hdmi_720x576i50_video_mode,
	.lcd_videomodes        = p7_all_video_modes,
	.lcd_interface={{
		.ipc           = 1,
		.free_run      = 1,
		.itu656        = 0,
	}},
	.id		       = "HDMI-LCD1_out",
	.caps		       = AVI_CAP_LCD_1 | AVI_CAP_CONV,
	.overlays	       = rnb6_avi_lcd1_overlays,
	.overlay_nr	       = ARRAY_SIZE(rnb6_avi_lcd1_overlays),
};

static u64 rnb6_avifb1_dma_mask = DMA_BIT_MASK(32);
static struct platform_device rnb6_avifb1_dev = {
	.name   = "avifb",
	.id     = 1,
	.dev    = {
		.platform_data      = &rnb6_avifb1_pdata,
		/* Todo: try to set a narrower mask */
		.dma_mask           = &rnb6_avifb1_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32),
	}
};

/* Dummy clock, have a look at p7_enable_clkgbc to actually enable GBC
 * clocks. This is possible since the plat-versatile clk_enable/disable do
 * nothing. */
static struct clk rnb6_lcd1_clk =
{
	.rate = 52175000,
	.ops	= &avi_clk_ops
};
#endif /* CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD1_ENABLE */

#endif /* defined (CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE) */

#if defined (CONFIG_CAM_AVI) || defined(CONFIG_CAM_AVI_MODULE)
#include <media/video/avicam.h>
#include <media/soc_camera_platform.h>

#define CAMERA_CAM_IF 1
#define CAMERA_AVI_CAM_NODE AVI_CAM1_NODE
#define CAMERA_AVI_CAM_IRQ P7_AVI_CAM1_IRQ
#define CAMERA_AVI_FIFO_NODE AVI_FIFO02_NODE
#define CAMERA_AVI_FIFO_IRQ P7_AVI_FIFO02_IRQ
#define CAMERA_I2C_BUS 2

static unsigned long rnb6_cam1_query_bus_param(struct soc_camera_link *icl)
{
	/* We put here the allowed bus widths (number of usable
	 * wires/pads). Note that the P7 capture interface and the sensor may
	 * not support all formats. This needs to be an exhaustive list: for
	 * instance if the bus can support 16 bit interfaces or smaller you
	 * should set at least SOCAM_DATAWIDTH_8 | SOCAM_DATAWIDTH_16, unless
	 * you want to force a 16 bit interface. */
	return SOCAM_DATAWIDTH_8|SOCAM_DATAWIDTH_16;
}

static unsigned int const rnb6_cam1_avi_nodes[] = {
	CAMERA_AVI_CAM_NODE,
#if (CAM1_CONV_NODE_NUMBER>=0)
	AVI_CONV_NODE(CAM1_CONV_NODE_NUMBER),
#endif
	CAMERA_AVI_FIFO_NODE,
};

#if (CAM1_SYNCHRO_TYPE==CAM_SYNCHRO_ITU_BT656) && \
	(CAM1_TYPE!=SOC_CAM_TYPE_MT9F002)
#define CAM1_MBUS_TYPE V4L2_MBUS_BT656
#else
#define CAM1_MBUS_TYPE V4L2_MBUS_PARALLEL
#endif

#if (CAM1_TYPE==SOC_CAM_TYPE_MT9V117)
#define CAMERA_AVI_RAM_SIZE mt9v117_camera_avi_ram_size
#define CAMERA_NAME MT9V117_CAMERA_NAME
SOC_CAMERA_PLATFORM_LINK_MT9V117(rnb6_cam1,CAMERA_CAM_IF,CAMERA_I2C_BUS,rnb6_cam1_query_bus_param);
#endif /* (CAM1_TYPE==SOC_CAM_TYPE_MT9V117) */

#if (CAM1_TYPE==SOC_CAM_TYPE_MT9F002)
#define CAMERA_AVI_RAM_SIZE mt9f002_camera_avi_ram_size
#define CAMERA_NAME MT9F002_CAMERA_NAME
SOC_CAMERA_PLATFORM_LINK_MT9F002(rnb6_cam1,CAMERA_CAM_IF,CAMERA_I2C_BUS,rnb6_cam1_query_bus_param);
#endif /* (CAM1_TYPE==SOC_CAM_TYPE_MT9F002) */

#if (CAM1_TYPE==SOC_CAM_TYPE_AS0260)
#define CAMERA_AVI_RAM_SIZE as0260_camera_avi_ram_size
#define CAMERA_NAME AS0260_CAMERA_NAME
SOC_CAMERA_LINK_AS0260(rnb6_cam1,CAMERA_CAM_IF,CAMERA_I2C_BUS,rnb6_cam1_query_bus_param,CAM1_MBUS_TYPE);
#endif /* (CAM1_TYPE==SOC_CAM_TYPE_AS0260) */

#if (CAM1_TYPE==SOC_CAM_TYPE_MT9M114)
#define CAMERA_AVI_RAM_SIZE mt9m114_camera_avi_ram_size
#define CAMERA_NAME MT9M114_CAMERA_NAME
SOC_CAMERA_LINK_MT9M114(rnb6_cam1,CAMERA_CAM_IF,CAMERA_I2C_BUS,rnb6_cam1_query_bus_param,CAM1_MBUS_TYPE);
#endif /* (CAM1_TYPE==SOC_CAM_TYPE_MT9M114) */

AVICAM_PLATFORM_DEVICE(rnb6_cam1,
			CAMERA_CAM_IF,
			CAMERA_AVI_CAM_IRQ,
			CAMERA_AVI_FIFO_IRQ,
			CAM1_MBUS_TYPE,
			0,
//#if defined(CAM1_OVERLAY_FB)
//			&rnb6_avi_chans_lcd1[3]);
//#else
			0);
//#endif


#ifdef Omnivision_OV9740_sensor
/*
 * Omnivision OV9740 sensor
 */
static struct i2c_board_info p7_avi_i2c_cameras[] = {
	{
		I2C_BOARD_INFO("ov9740", 0x10),
		.platform_data = NULL,
	},
};
static struct soc_camera_link camera_iclink = {
	.bus_id		= 0, /* id of the camif platform device */
	.board_info	= p7_avi_i2c_cameras,
	.i2c_adapter_id	= 0,
	.module_name	= "ov9740",
};
#endif /* Omnivision_OV9740_sensor */


#endif /* defined (CONFIG_CAM_AVI) || defined(CONFIG_CAM_AVI_MODULE) */

#if defined(CONFIG_R2R_AVI) || defined(CONFIG_R2R_AVI_MODULE)
#include <media/video/avi_r2r.h>

static struct avi_r2r_platform_data rnb6_avi_r2r_pdata = { {
		[0]={.caps=AVI_CAP_PLANAR|AVI_CAP_SCAL|AVI_CAP_CONV,
		     .id="avi_r2r.0"},
		[1]={.caps=AVI_CAP_CONV|AVI_CAP_GAM,
		     .id="avi_r2r.1"},
		[2]={.caps=0,
		     .id="avi_r2r.2"},
		[3]={.caps=0,
		     .id=NULL}
} };

static u64 p7_avi_r2r_dma_mask = DMA_BIT_MASK(32);
static struct platform_device p7_avi_r2r_dev = {
	.name           = "avi_r2r",
	.id             = 0,
	.dev            = {
		.platform_data     = &rnb6_avi_r2r_pdata,
		.dma_mask          = &p7_avi_r2r_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32)
	}
};

#if defined(CONFIG_R2R_AVI_TEST) || defined(CONFIG_R2R_AVI_TEST_MODULE)
static struct platform_device p7_avi_r2r_test_dev = {
		.name           = "avi_r2r_test",
		.id             = 0,
		.dev            = { 0 }
};
#endif
#if defined(CONFIG_R2R_AVI_TEST_MULTI) || \
	defined(CONFIG_R2R_AVI_TEST_MULTI_MODULE)
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
#else
#warning "Number of clients is 0 for test module of AVI R2R"
#endif
#endif
#endif /* defined(CONFIG_R2R_AVI) || defined(CONFIG_CAM_R2R_MODULE) */

#if defined (CONFIG_AVI) || defined(CONFIG_AVI_MODULE)
/* base address of the AVI on the FPGAs */
#define P7_AVI          0xfe600000
#define P7_AVI_GBC      (P7_AVI + UL(0xff000))
#define AVI_GBC_RESET   UL(0xff8)
#define AVI_GBC_CLOCK   UL(0xffc)

static struct resource p7_avi_res[] = {
	{
	.start  = P7_AVI,
	.end    = P7_AVI + PAGE_ALIGN(0x60000) - 1,
	.flags  = IORESOURCE_MEM,
	},
	{
	.start = P7_AVI_FIFO_IRQ,
	.end   = P7_AVI_FIFO_IRQ,
	.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE,
	},
	{
	.start = P7_AVI_LCD_IRQ,
	.end   = P7_AVI_LCD_IRQ,
	.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE,
	},
	{
	.start = P7_AVI_CAM_IRQ,
	.end   = P7_AVI_CAM_IRQ,
	.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE,
	}
};

/*
static void p7_enable_clkgbc(u32 base, u32 clk_off, u32 reset_off, u32 clk_msk);

static int p7_avi_enable_clk(struct clk* clk)
{
	p7_enable_clkgbc(P7_AVI_GBC, AVI_GBC_CLOCK, AVI_GBC_RESET, 1);
}
*/
static const struct clk_ops avi_core_clk_ops = {
	.round	= avi_osc_round,
	.set	= avi_osc_set,
};

/* Dummy clock, have a look at p7_enable_clkgbc to actually enable GBC
 * clocks. This is possible since the plat-versatile clk_enable/disable do
 * nothing. */
static struct clk rnb6_avi_clk =
{
	.rate = 52175000,
	.ops  = &avi_core_clk_ops
};

/* Table of AVI channels groups. */
static struct avi_group* rnb6_avi_groups[] = {
#if defined (CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE)
#ifdef CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD0_ENABLE
	//&rnb6_avi_group_lcd0,
#endif /* CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD0_ENABLE */

#ifdef CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD1_ENABLE
	//&rnb6_avi_group_lcd1,
#endif /* CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD1_ENABLE */

#endif
#if defined (CONFIG_CAM_AVI) || defined (CONFIG_CAM_AVI_MODULE)
	//&rnb6_cam1_avi_group,
#endif
  0					  /* End of table marker */
};

static struct avi_plat_data rnb6_avi_data = {
	.groups = rnb6_avi_groups,
	.revision = AVI_REVISION,
	.irq_base = 0
};

static struct platform_device rnb6_avi_dev = {
	.name   = "avi",
	.id     = 0,
	.dev = {
		.platform_data  = &rnb6_avi_data,
	},
	.resource       = p7_avi_res,
	.num_resources  = ARRAY_SIZE(p7_avi_res)
};
#endif /* defined (CONFIG_AVI) || defined(CONFIG_AVI_MODULE) */

#if defined(CONFIG_VDEC_HX270) || defined(CONFIG_VDEC_HX270_MODULE)

#define P7_HX270_RAM_SIZE   (96 << 20)
#define P7_VDEC             UL(0xf0600000)
#define P7_VDEC_GBC         (P7_VDEC + UL(0xff000))
#define VDEC_GBC_RESET      UL(0xff8)
#define VDEC_GBC_CLOCK      UL(0xffc)

/********************************
 * Hantro / On2 G1 video decoder
 ********************************/
static struct resource p7_vdec_res[] = {
	[0] = {
		.start = P7_VDEC,
		.end   = P7_VDEC + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_VDEC_IRQ,
		.end   = P7_VDEC_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.end   = P7_HX270_RAM_SIZE - 1,
		.flags = IORESOURCE_MEM | IORESOURCE_DMA_MASTER,
	},
};

static u64 p7_vdec_dma_mask = DMA_BIT_MASK(32);
static struct platform_device p7_vdec_dev = {
	.name   = "hx270-vdec",
	.dev    = {
		.dma_mask           = &p7_vdec_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	},
	.id             = 0,
	.resource       = p7_vdec_res,
	.num_resources  = 3,
};

static struct clk p7_vdec_clk = {};

#endif /* defined(CONFIG_VDEC_HX270) || defined(CONFIG_VDEC_HX270_MODULE) */

#include <media/p7-mpegts.h>

#if defined(CONFIG_SPI_MASTER_PARROT7) || \
    defined(CONFIG_SPI_MASTER_PARROT7_MODULE)

/***************
 * P7 SPI controller
 ***************/

#define P7_SPI_BASE		0xfd000000
#define P7_SPI0         (P7_SPI_BASE + UL(0x100000))
#define P7_SPI          (P7_SPI_BASE + UL(0x10f000))

#include <spi/p7-spi.h>
#include <spi/p7-spim.h>
#include <linux/spi/flash.h>

static enum p7swb_function p7swb_available_funcs_r23[] = {
	P7_SWB_SPI_CLK,
	P7_SWB_SPI_SS,
	P7_SWB_SPI_DATA0,
	P7_SWB_SPI_DATA1,
	P7_SWB_SPI_DATA2,
	P7_SWB_SPI_DATA3,

	P7_SWB_MPEG_CLK,
	P7_SWB_MPEG_DATA,
	P7_SWB_MPEG_VALID,
	P7_SWB_MPEG_SYNC,

	P7_SWB_LOW,
	P7_SWB_HIGH,
	P7_SWB_Z,
};

/* SPI Kernel resource */
static struct resource p7_spi_res = {
	.start = P7_SPI,
	.end   = P7_SPI + SZ_4K - 1,
	.flags = IORESOURCE_MEM,
};

/* SPI Kernel platform data */
static struct p7spi_plat_data p7_spi_pdata = {
	.wcnt = P7SPI_MEM_WCNT(0, 32) | P7MPEGTS_MEM_WCNT(0, 16),
	.max_master_hz = 65000000,
	.rev = SPI_REVISION,
	.max_slave_hz = 10000000,
	.min_hz = 186000,
	.num_pads = 20,
	.available_funcs = p7swb_available_funcs_r23,
	.available_funcs_sz = ARRAY_SIZE(p7swb_available_funcs_r23),
};

/* SPI Kernel device */
static struct platform_device p7_spi_dev = {
	.name           = P7SPI_DRV_NAME,
	.id             = 0,
	.resource       = &p7_spi_res,
	.num_resources  = 1,
	.dev            = {
		.platform_data		= &p7_spi_pdata,
	}
};

#define P7_SPI_IRQ	68
static struct clk p7_spi_clk =  { .rate = 80000000 };
static struct resource p7_spi0_res[] = {
	[0] = {
		.start = P7_SPI0,
		.end   = P7_SPI0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_SPI_IRQ,
		.end   = P7_SPI_IRQ,
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE,
	},
	[2] = { /* DMA channel 0 */
		.start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_DMA,
	}
};

/*
 * Single SPI half-duplex mode SPI controller internal multiplexing setup:
 * the same line (DATA0) is used alternativly for MISO and MOSI functions.
 */
static struct p7spi_swb const p7_spi0_swb[] = {
	P7SPI_INIT_SWB(0, P7_SWB_DIR_OUT,   P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB(1, P7_SWB_DIR_OUT,   P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB(2, P7_SWB_DIR_IN,    P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB(3, P7_SWB_DIR_OUT,   P7_SWB_SPI_SS),
	P7SPI_SWB_LAST,
};

#if 1
static struct p7spi_ctrl_data p7_spinor_cdata = {
	.half_duplex        = true,
	.read               = true,
	.write              = true,
	.xfer_mode          = P7SPI_SINGLE_XFER,
	.thres_wcnt         = 24,
	.tsetup_ss_ns       = 8,    /* Tces */
	.thold_ss_ns	    = 8,    /* Tceh */
	.toffclk_ns	        = 0,
	.toffspi_ns         = 25,   /* Tcph */
	.tcapture_delay_ns  = P7SPI_TCAPTURE_DELAY_MIN
};
#else
static struct p7spi_ctrl_data p7_spinor_cdata = {
	.half_duplex        = true,
	.read               = true,
	.write              = true,
	.xfer_mode          = P7SPI_SINGLE_XFER,
	.thres_wcnt         = 48,
	.tsetup_ss_ns       = 10,
	.thold_ss_ns	    = 10,
	.toffclk_ns	        = 10,
	.toffspi_ns         = 0,
	.tcapture_delay_ns  = 40
};
#endif

static struct spi_board_info p7_spi_board_info[] __initconst = {
	[0] = {
		.modalias           = "spidev",
		.controller_data    = &p7_spinor_cdata,
		.irq                = -1,
		.max_speed_hz       = 40000000,
		.bus_num            = 0,
		.chip_select        = 0,
		.mode               = SPI_MODE_0
	}
};

static u64 p7_spi0_dma_mask = DMA_BIT_MASK(32);
static struct platform_device p7_spi_devs[] = {
	[0] = {
		.name   	= "p7-spim",
		.id     	= 0,
		.resource       = p7_spi0_res,
		.num_resources  = ARRAY_SIZE(p7_spi0_res),
		.dev            = {
			.dma_mask		= &p7_spi0_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
			.platform_data          = p7_spi0_swb,
		}
	},
};


#if 0
//#define P7_SPI_FLASH
#define P7_SPI_SPIDEV


#define P7_SPI_IRQ

#ifdef P7_SPI_IRQ
#undef P7_SPI_IRQ
#endif /* P7_SPI_IRQ */

// comment or uncomment the following line
#define P7_SPI_IRQ	68

static struct clk p7_spi_clk =  { .rate = 80000000 };

static struct resource p7_spi0_res[] = {
	[0] = {
		.start  = P7_SPI + P7_SPI_CORE_OFFSET(0),
		.end    = P7_SPI + P7_SPI_CORE_OFFSET(1) - 1,
		.flags  = IORESOURCE_MEM
	},
#ifdef P7_SPI_IRQ
	[1] = {
		.start  = P7_SPI_IRQ,
		.end    = P7_SPI_IRQ,
		.flags  = IORESOURCE_IRQ
	},
#endif /* P7_SPI_IRQ */
};

static p7_spi_swb_reg p7_spi_swb[] = {
	P7_SPI_SWB(P7_SPI_SWB_PAD(10), P7_SPI_SWB_PAD_CLK_OUT(0)),
	P7_SPI_SWB(P7_SPI_SWB_PAD(11), P7_SPI_SWB_PAD_DATA0_OUT(0)),
	P7_SPI_SWB(P7_SPI_SWB_CTRL_DATA0_IN(0), P7_SPI_SWB_CTRL_PAD(12)),
	P7_SPI_SWB(P7_SPI_SWB_PAD(13), P7_SPI_SWB_PAD_SS_OUT(0)),

#if defined(CONFIG_P7_SPIS) || defined(CONFIG_P7_SPIS_MODULE)
	P7_SPI_SWB(P7_SPI_SWB_CTRL_CLK_IN(1), P7_SPI_SWB_CTRL_PAD(10)),
	P7_SPI_SWB(P7_SPI_SWB_CTRL_DATA0_IN(1), P7_SPI_SWB_CTRL_PAD(11)),
	P7_SPI_SWB(P7_SPI_SWB_PAD(12), P7_SPI_SWB_PAD_DATA0_OUT(1)),
	P7_SPI_SWB(P7_SPI_SWB_CTRL_SS_IN(1), P7_SPI_SWB_CTRL_PAD(13)),
#endif /* defined(CONFIG_P7_SPIS) || defined(CONFIG_P7_SPIS_MODULE) */
	P7_SPI_SWB_LAST,
};

static struct p7_spi_common_config p7_controller_common_conf  = {
	.swb = p7_spi_swb,
#if defined(CONFIG_P7_SPIS) || defined(CONFIG_P7_SPIS_MODULE)
	.fifo_sz[0] = P7_SPI_FIFO_SZ_32W,
	.fifo_sz[1] = P7_SPI_FIFO_SZ_32W,
#else
	.fifo_sz[0] = P7_SPI_FIFO_SZ_64W,
#endif /* defined(CONFIG_P7_SPIS) || defined(CONFIG_P7_SPIS_MODULE) */
	.phys  = P7_SPI + P7_SPI_COMMON_OFFSET,
};

static struct p7_spi_config p7_controller_data = {
	.common_conf = &p7_controller_common_conf,

	.tsetup_ss_ns	= 10,
	.thold_ss_ns	= 10,
	.toffclk_ns	= 10,
	.tcapture_delay_ns = 40,

	.spi_tmode = P7_SPI_INSTR_SPI_TMODE_SINGLE,
};

static u64 p7_spi_dma_mask = DMA_BIT_MASK(32);
static struct platform_device p7_spi_dev[] = {
	[0] = {
		.name   	= "p7_spi",
		.id     	= 0,
		.resource       = p7_spi0_res,
		.num_resources  = ARRAY_SIZE(p7_spi0_res),
		.dev            = {
			.dma_mask		= &p7_spi_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
			.platform_data		= &p7_controller_data,
		}
	},
};

#ifdef P7_SPI_FLASH
static struct flash_platform_data p7_spi_flash = {
	.type = "sst25wf010",
};
#endif /* P7_SPI_FLASH */

static struct spi_board_info p7_spi_board_info[] = {
	{
#ifdef P7_SPI_FLASH
		.modalias = "m25p80",
		.platform_data = &p7_spi_flash,
#endif /* P7_SPI_FLASH */
#ifdef P7_SPI_SPIDEV
		.modalias = "spidev",
		.platform_data = NULL,
#endif /* P7_SPI_SPIDEV */
		.max_speed_hz = 1000000,
		.bus_num = 0,
 		.chip_select = 0,
		.mode = SPI_MODE_0,
	},
};

#endif
#endif /* defined(CONFIG_SPI_MASTER_PARROT7) || defined(CONFIG_SPI_MASTER_PARROT7_MODULE) */

#if defined(CONFIG_P7_SPIS) || defined(CONFIG_P7_SPIS_MODULE)
static struct resource p7_spi2_res[] = {
	[0] = {
		.start  = P7_SPI + P7_SPI_CORE_OFFSET(1),
		.end    = P7_SPI + P7_SPI_CORE_OFFSET(2) - 1,
		.flags  = IORESOURCE_MEM,
	},
#ifdef P7_SPI_IRQ
	[1] = {
		.start  = P7_SPI_IRQ,
		.end    = P7_SPI_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
#endif /* P7_SPI_IRQ */
};

static struct p7_spis_config p7_spis_conf = {
	.common_conf = &p7_controller_common_conf,
	.stmode = P7_SPI_CTRL_SMODE_SINGLE,
};

static u64 p7_spis_dma_mask = DMA_BIT_MASK(32);
static struct platform_device p7_spis_dev = {
	.name   	= "p7_spis",
	.id     	= 1,
	.resource       = p7_spi2_res,
	.num_resources  = ARRAY_SIZE(p7_spi2_res),
	.dev            = {
		.dma_mask		= &p7_spis_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data		= &p7_spis_conf,
	}
};
#endif /* defined(CONFIG_P7_SPIS) || defined(CONFIG_P7_SPIS_MODULE) */

#if defined(CONFIG_MPEGTS_PARROT7) || defined(CONFIG_MPEGTS_PARROT7_MODULE)

#define P7_SPI_BASE	0xfd000000
#define P7_MPEGTS0      (P7_SPI_BASE + UL(0x104000))
#define P7_SPI          (P7_SPI_BASE + UL(0x10f000))
#define P7_MPEGTS0_DMA  5

static struct p7spi_swb const p7_mpegts0_swb[] = {
	P7SPI_INIT_SWB(16, P7_SWB_DIR_IN,    P7_SWB_MPEG_CLK),
	P7SPI_INIT_SWB(17, P7_SWB_DIR_IN,    P7_SWB_MPEG_DATA),
	P7SPI_SWB_LAST,
};

static struct p7mpegts_plat_data p7_mpegts0_pdata = {
	.swb        = p7_mpegts0_swb,
	.thres_wcnt = 8,
};

static struct resource p7_mpegts0_res[] = {
	[0] = {
		.start = P7_MPEGTS0,
		.end   = P7_MPEGTS0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
	/*	.start = P7_MPEGTS0_IRQ,
		.end   = P7_MPEGTS0_IRQ,
		.flags = IORESOURCE_IRQ*/
	},
	[2] = {
		.start	= P7_MPEGTS0_DMA,
		.end	= P7_MPEGTS0_DMA,
		.flags	= IORESOURCE_DMA,
	},
	[3] = {
		.flags = IORESOURCE_MEM,
	},
};

static u64 p7_mpegts0_dma_mask = DMA_BIT_MASK(32);

static struct platform_device p7_mpegts_dev[] = {
	{
		.name           = P7MPEGTS_DRV_NAME,
		.id             = 0,
		.resource       = p7_mpegts0_res,
		.num_resources  = ARRAY_SIZE(p7_mpegts0_res),
		.dev            = {
			.dma_mask           = &p7_mpegts0_dma_mask,
			.coherent_dma_mask  = DMA_BIT_MASK(32),
			.platform_data      = &p7_mpegts0_pdata,
		},
	},
};
#endif

#if defined(CONFIG_PL330_DMA) ||  defined(CONFIG_PL330_DMA_MODULE)

/***************
 * DMAC PL330 device
 ***************/
#include <linux/amba/bus.h>
#include <linux/amba/pl330.h>

#define P7_DMA		0xfc100000
#define P7_DMA_IRQ	69
#define P7_DMA_GBC         (P7_DMA + UL(0xff000))
#define P7_DMA_GBC_RESET      UL(0xff8)
#define P7_DMA_GBC_CLOCK      UL(0xffc)

/* FIXME: initialize using do_device_register to prevent conflicts with oether
 * devices */
#define P7_DMA_UCODE		0xe0000000

static u8 p7_dma_peri[] = {
	0,  /* SPI0 */
	1,  /* SPI1 */
	2,  /* SPI2 */
	3,  /* SPI3 */
	4,  /* PARINT */
	5,  /* RAM */
};

static struct dma_pl330_platdata p7_dma_pdata = {
	.flushp         = true,
	.nr_valid_peri  = ARRAY_SIZE(p7_dma_peri),
	.peri_id        = p7_dma_peri,
	.mcbuf_sz       = 512 /* 512 bytes ucode / channel */
};

static struct amba_device p7_dma_dev = {
	.dev        = __AMBA_DEV("pl330-dma.0",
	                         &p7_dma_pdata,
	                         DMA_BIT_MASK(32)),
	.res        = DEFINE_RES_MEM(P7_DMA, SZ_4K),
	.dma_mask   = DMA_BIT_MASK(32),
	.irq        = { P7_DMA_IRQ, P7_DMA_IRQ + 6 },
	.periphid   = 0x00041330
};

static struct clk p7_dma_clk = {};

#if 0
#define P7_DMA_UCODE_LEN	PAGE_SIZE*2
#include <dma/p7-pl330.h>

static struct clk p7_dma_clk = {};

static u64 p7_dma_dmamask = DMA_BIT_MASK(32);

static u8 p7_dma_peri[] = {
	DMACH_SPI0,
	DMACH_SPI1,
	DMACH_SPI2,
	DMACH_SPI3,
	DMACH_PARINT,
	DMACH_M2M,
};

static struct dma_pl330_platdata p7_dma_pdata = {
	.nr_valid_peri = ARRAY_SIZE(p7_dma_peri),
	.peri_id = p7_dma_peri,
};

static struct amba_device p7_amba_device_pdma = {
	.dev = {
		.init_name = "dma-pl330.0",
		.dma_mask = &p7_dma_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &p7_dma_pdata,
		},
	.res = {
		.start = P7_DMA,
		.end = P7_DMA + SZ_4K,
		.flags = IORESOURCE_MEM,
		},
	.irq = {
		P7_DMA_IRQ,	/* first */
		P7_DMA_IRQ,	/* last */
		},
	.periphid = 0x00041330,
};

static struct resource p7_dma_ucode_res[] = {
	[0] = {
		.start  = P7_DMA_UCODE,
		.end    = P7_DMA_UCODE + P7_DMA_UCODE_LEN,
		.flags  = IORESOURCE_MEM
	},
};

static struct platform_device p7_pdma_dev = {
	.name			= "p7-pl330",
	.id				= 0,
	.resource       = p7_dma_ucode_res,
	.num_resources  = ARRAY_SIZE(p7_dma_ucode_res),
	.dev		= {
		.platform_data = &p7_amba_device_pdma,
	},
};
#endif
#endif

#if defined(CONFIG_GPU_M200) || defined(CONFIG_GPU_M200_MODULE)

#define P7_GPU	0xf0600000
#define P7_GPU_GBC         (P7_GPU + UL(0xff000))
#define P7_GPU_GBC_CLOCK      UL(0xfec)
#define P7_GPU_GBC_RESET      UL(0xfe8)

static struct clk p7_gpu_busclk = {};
static struct clk p7_gpu_coreclk = {};

/***************
 * MALI 200 GPU
 ***************/
static struct platform_device p7_gpu_dev = {
	.name   = "m200-gpu",
	.id     = 0
};

#endif /* defined(CONFIG_GPU_M200) || defined(CONFIG_GPU_M200_MODULE) */

#include <linux/debugfs.h>
#include <linux/seq_file.h>

static int rnb6_show(struct seq_file *s, void *unused)
{
#if defined(CONFIG_CAM_AVI) || defined(CONFIG_CAM_AVI_MODULE)

//#ifdef CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_CAMERA_ENABLE
	seq_printf(s, "cam1_cfg=\"%s\"\n", CAMERA_NAME);
	seq_printf(s, "cam1_use_itu_bt656=%d\n",CAM1_MBUS_TYPE);
	seq_printf(s, "cam1_conv=%d\n", CAM1_CONV_NODE_NUMBER);
#ifdef	CAM1_SYNCHRO_USE_HS_VS
	seq_printf(s, "cam1_hs_vs_wired=1\n");
#else
	seq_printf(s, "cam1_hs_vs_wired=0\n");
#endif
//#endif /* CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_CAMERA_ENABLE */

#ifdef CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_HDMI_INPUT_ENABLE
	seq_printf(s, "cam4_cfg=\"%s\"\n", P7_AVI_CAM4_NAME);
	seq_printf(s, "cam4_use_itu_bt656=%d\n",CAM4_MBUS_TYPE);
	seq_printf(s, "cam4_conv=%d\n", CAM4_CONV_NODE_NUMBER);
#ifdef CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_CAM_4_HS_VS
	seq_printf(s, "cam4_hs_vs_wired=1\n");
#else
	seq_printf(s, "cam4_hs_vs_wired=0\n");
#endif
#endif /* CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_HDMI_INPUT_ENABLE */

#ifdef CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_ANALOG_VIDEO_INPUT_ENABLE
	seq_printf(s, "cam0_cfg=\"%s\"\n", P7_AVI_CAM0_NAME);
	seq_printf(s, "cam0_use_itu_bt656=%d\n",CAM0_MBUS_TYPE);
	seq_printf(s, "cam0_conv=%d\n", CAM0_CONV_NODE_NUMBER);
#endif /* CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_ANALOG_VIDEO_INPUT_ENABLE */

#endif /* defined(CONFIG_CAM_AVI) || defined(CONFIG_CAM_AVI_MODULE) */

#if defined(CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE)
#ifdef CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD0_ENABLE
	seq_printf(s, "lcd0=\"LCD\"\n");
#endif /* CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD0_ENABLE */
#ifdef CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD1_ENABLE
	seq_printf(s, "lcd1=\"HDMI\"\n");
#endif /* CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD1_ENABLE */
#endif /* defined(CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE) */

	return 0;
}

static int rnb6_debugfs_open(struct inode *inode, struct file *file)
{
	/* inode->i_private contains the "show" callback */
	return single_open(file, inode->i_private, NULL);
}

static const struct file_operations rnb6_debugfs_fops = {
	.open		= rnb6_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


#define FPGA_RAM_BASE  0xE0000000
#define FPGA_RAM_SIZE  SZ_256M

static unsigned fpga_ddr_base = FPGA_RAM_BASE;

#ifdef	CONFIG_ARCH_VEXPRESS_ZBT_ENABLE
#define ZBT_RAM_BASE  0xfd000000
#define ZBT_RAM_SIZE  SZ_16M

static unsigned zbt_ddr_base = ZBT_RAM_BASE;
#endif

static int do_alloc_resource_mem(struct platform_device *pdev,
		                                struct resource *res,
		                                unsigned *ddr_base,
		                                const unsigned ram_base,
		                                const unsigned ram_size)
{
	if ((res->flags & IORESOURCE_MEM) && res->start == 0)
	{
		/* If the start address is not specified, we "dynamically"
		 * allocate a new chunk of the FPGA/ZBT memory */
		res->start = roundup(*ddr_base, SECTION_SIZE);
		res->end = res->start + roundup(res->end + 1, SECTION_SIZE) - 1;

		if (res->end >= ram_base + ram_size) {
				printk(KERN_ERR "RAM %08X exhausted! No room for %s.%d (%d bytes)\n",
					   ram_base,
					   pdev->name,pdev->id,res->end-res->start);
				return -ENOMEM;
		}

		printk("	%08x:%08x: %s.%d (RES)\n",
			   res->start,
			   res->end,
			   pdev->name,
			   pdev->id);

		*ddr_base = res->end;
	}
	return 0;
}

static int __init do_alloc_resource(struct platform_device *pdev,
		                            struct resource *res)
{
#ifdef	CONFIG_ARCH_VEXPRESS_ZBT_ENABLE
	return do_alloc_resource_mem(pdev,
								 res,
								 &zbt_ddr_base,
								 ZBT_RAM_BASE,
								 ZBT_RAM_SIZE);
#else
	return do_alloc_resource_mem(pdev,
						 res,
						 &fpga_ddr_base,
						 FPGA_RAM_BASE,
						 FPGA_RAM_SIZE);
#endif
}

static int __init do_device_register(struct platform_device *pdev)
{
	int i;

	printk(KERN_INFO ". VExpress Register device %s.%d\n",pdev->name,pdev->id);

	for (i = 0; i < pdev->num_resources; i++) {
		struct resource *res = &pdev->resource[i];

		if ((res->flags & IORESOURCE_MEM) && res->start == 0)
		{
			do_alloc_resource_mem(pdev,
					 res,
					 &fpga_ddr_base,
					 FPGA_RAM_BASE,
					 FPGA_RAM_SIZE);
		}
		else if (res->flags & IORESOURCE_MEM)
		{
			printk("	%08x:%08x: %s.%d (IO)\n",
			       res->start,
			       res->end,
			       pdev->name,
			       pdev->id);
		}
		else if (res->flags & IORESOURCE_IRQ)
		{
			printk("	IRQ%03d:IRQ%03d: %s.%d\n",
			       res->start,
			       res->end,
			       pdev->name,
			       pdev->id);
		}
	}

	return platform_device_register(pdev);
}

#define DMC_BASE 0xF0020000

/* redefine registers accesses for debugging */
#define __my_writel(val,mem) \
({ \
	__raw_writel((val),(mem)); \
	printk(KERN_INFO ". VExpress W [%08lx] : 0x%08lx\n",(mem),(long unsigned int)(val)); \
})

#define __my_readl(mem) \
({ \
	unsigned long ret; \
	ret=__raw_readl(mem);  \
	printk(KERN_INFO ". VExpress R [%08lx] : 0x%08lx\n",(mem),ret); \
	ret; \
})

/* GBC clock enable */
static void p7_enable_clkgbc(u32 base, u32 clk_off, u32 reset_off, u32 clk_msk)
{
	unsigned long b;

	b = (unsigned long)ioremap(base, SZ_4K);

	BUG_ON(!b);

	printk( KERN_INFO
			". VExpress Enable CLK GBC %08X, clk=%08X, reset=%08X, mask=%X\n",
			base,clk_off,reset_off,clk_msk);

	/* De-reset GBC child block. */
	__my_writel(0, b + reset_off);
	/* Enable clock. */
	__my_writel(clk_msk | __my_readl(b + clk_off), b + clk_off);

	mdelay(20);

	/* Reset GBC child block. */
	__my_writel(1, b + reset_off);
	/* Disable clock. */
	__my_writel(__my_readl(b + clk_off) & ~clk_msk, b + clk_off);

	mdelay(20);

	/* De-reset GBC child block. */
	__my_writel(0, b + reset_off);
	/* Enable clock. */
	__my_writel(clk_msk | __my_readl(b + clk_off), b + clk_off);

#if 0
	/* Enable clock, */
	__my_writel(clk_msk | __my_readl(b + clk_off), b + clk_off);

	/* then request reset if necessary, */
	if (! reset_off)
		goto unmap;

	/* De-reset GBC child block. */
	__my_writel(0, b + reset_off);
#endif
#if 0
	if (reset_off)
	{
		if ((__my_readl(b + clk_off) & clk_msk) && (__my_readl(b + reset_off) == 0))
			/* if the clock is already enabled and the IP is not
			 * reset we don't have anything left to do. */
		{
			printk(KERN_INFO
					". VExpress Enable CLK GBC %08X: clock already enabled and not in reset\n",
					base);
			goto unmap;
		}
		printk(KERN_INFO
				". VExpress Enable CLK GBC %08X set up\n",
				base);
		/* We have to disable the clock when we (un)assert the reset
		 * (see section 3.2.5: Clock and reset sequence of the user
		 * manual). */
		__my_writel(__my_readl(b + clk_off) & ~clk_msk, b + clk_off);

		/* Force reset of the GBC child block */
		__my_writel(1, b + reset_off);

		/* Enable clock to make the synchronous reset take effect */
		__my_writel(__my_readl(b + clk_off) | clk_msk, b + clk_off);
		mdelay(20);
		/* Then disable once again to de-reset the block */
		__my_writel(__my_readl(b + clk_off) & ~clk_msk, b + clk_off);
		__my_writel(0, b + reset_off);

		/* At this point we're sure the block is correctly reset and the
		 * clock is disabled */
	}
	/* Enable clock */
	__my_writel(__my_readl(b + clk_off) | clk_msk, b + reset_off);

unmap:
#endif

	iounmap((void *)b);
}

/* FPGA Memory Controler Init */
static void __init init_DMC(void)
{
	unsigned long base;

	printk(KERN_INFO ". VExpress FPGA Memory Controler Initialisation...\n");

	base = (unsigned long)ioremap(DMC_BASE, SZ_4K);

	BUG_ON(!base);

	*((volatile unsigned int *) (base + 0x4)) = 0x00000004;
	*((volatile unsigned int *) (base + 0x10)) = 0x00000900;
	*((volatile unsigned int *) (base + 0x14)) = 0x00000006;
	*((volatile unsigned int *) (base + 0x18)) = 0x00000002;
	*((volatile unsigned int *) (base + 0x1C)) = 0x00000002;
	*((volatile unsigned int *) (base + 0x20)) = 0x00000010;
	*((volatile unsigned int *) (base + 0x24)) = 0x00000010;
	*((volatile unsigned int *) (base + 0x28)) = 0x00000003;
	*((volatile unsigned int *) (base + 0x2C)) = 0x00004425;
	*((volatile unsigned int *) (base + 0x30)) = 0x00000003;
	*((volatile unsigned int *) (base + 0x34)) = 0x00000002;
	*((volatile unsigned int *) (base + 0x38)) = 0x00000003;
	*((volatile unsigned int *) (base + 0x3C)) = 0x00000003;
	*((volatile unsigned int *) (base + 0x40)) = 0x00000003;
	*((volatile unsigned int *) (base + 0x44)) = 0x000000C8;
	*((volatile unsigned int *) (base + 0x48)) = 0x000000C8;
	*((volatile unsigned int *) (base + 0x54)) = 0x00000407;
	*((volatile unsigned int *) (base + 0xC)) = 0x00210022;
	*((volatile unsigned int *) (base + 0x4C)) = 0x000000B8;
	*((volatile unsigned int *) (base + 0x200)) = 0x0001A080;
	*((volatile unsigned int *) (base + 0x204)) = 0x000100F8;
	*((volatile unsigned int *) (base + 0x8)) = 0x000C0000;
	*((volatile unsigned int *) (base + 0x8)) = 0x001C0000;
	udelay(1000);
	*((volatile unsigned int *) (base + 0x8)) = 0x00000000;
	*((volatile unsigned int *) (base + 0x8)) = 0x00100000;
	udelay(1000);
	*((volatile unsigned int *) (base + 0x8)) = 0x000A0000;
	*((volatile unsigned int *) (base + 0x8)) = 0x001A0000;
	*((volatile unsigned int *) (base + 0x8)) = 0x000B0000;
	*((volatile unsigned int *) (base + 0x8)) = 0x001B0000;
	*((volatile unsigned int *) (base + 0x8)) = 0x00090000;
	*((volatile unsigned int *) (base + 0x8)) = 0x00190000;
	*((volatile unsigned int *) (base + 0x8)) = 0x00080100;
	*((volatile unsigned int *) (base + 0x8)) = 0x00180100;
	udelay(1000);
	*((volatile unsigned int *) (base + 0x8)) = 0x00000000;
	*((volatile unsigned int *) (base + 0x8)) = 0x00100000;
	*((volatile unsigned int *) (base + 0x8)) = 0x00040000;
	*((volatile unsigned int *) (base + 0x8)) = 0x00140000;
	*((volatile unsigned int *) (base + 0x8)) = 0x00040000;
	*((volatile unsigned int *) (base + 0x8)) = 0x00140000;
	*((volatile unsigned int *) (base + 0x8)) = 0x00080432;
	*((volatile unsigned int *) (base + 0x8)) = 0x00180432;
	*((volatile unsigned int *) (base + 0x8)) = 0x000903C4;
	*((volatile unsigned int *) (base + 0x8)) = 0x001903C4;
	*((volatile unsigned int *) (base + 0x8)) = 0x00090044;
	*((volatile unsigned int *) (base + 0x8)) = 0x00190044;
	*((volatile unsigned int *) (base + 0x4)) = 0x00000000;

	iounmap((void *)base);
}

static struct clk_lookup p7_dummy_clocks[] = {
#if defined (CONFIG_I2CM_PARROT7) || defined(CONFIG_I2CM_PARROT7_MODULE) \
		|| defined (CONFIG_I2CM_PARROT7_LEGACY) \
		|| defined(CONFIG_I2CM_PARROT7_LEGACY_MODULE)
	{
		.dev_id = "p7-i2cm.0",
		.clk    = &p7_i2cm_clk,
	},
	{
		.dev_id = "p7-i2cm.1",
		.clk    = &p7_i2cm_clk,
	},
	{
		.dev_id = "p7-i2cm.2",
		.clk    = &p7_i2cm_clk,
	},
	{
		.dev_id = "p7-i2cm.3",
		.clk    = &p7_i2cm_clk,
	},
#endif /* defined (CONFIG_I2CM_PARROT7) || defined(CONFIG_I2CM_PARROT7_MODULE) */
#if defined (CONFIG_AVI) || defined(CONFIG_AVI_MODULE)
	{
		.dev_id = "avi.0",
		.clk    = &rnb6_avi_clk,
	},
#endif /* defined (CONFIG_AVI) || defined(CONFIG_AVI_MODULE) */
#if defined (CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE)
#ifdef CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD0_ENABLE
	{
		.dev_id = "avifb.0",
		.con_id = "lcd",
		.clk    = &rnb6_lcd0_clk,
	},
#endif /* CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD0_ENABLE */
#ifdef CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD1_ENABLE
	{
		.dev_id = "avifb.1",
		.con_id = "lcd",
		.clk    = &rnb6_lcd1_clk,
	},
#endif /* CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD1_ENABLE */
#endif /* defined (CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE) */
#if defined(CONFIG_VDEC_HX270) || defined(CONFIG_VDEC_HX270_MODULE)
	{	/* Video DECoder. */
		.dev_id = "hx270-vdec.0",
		.clk    = (struct clk*) &p7_vdec_clk,
	},
#endif /* defined(CONFIG_VDEC_HX270) || defined(CONFIG_VDEC_HX270_MODULE) */
#if defined(CONFIG_SPI_PARROT7) || defined(CONFIG_SPI_PARROT7_MODULE)
    {   /* SPI clock. */
		.con_id		= "p7-spi",
		.clk		= &p7_spi_clk,
    },
#endif /* defined(CONFIG_SPI_PARROT7) || defined(CONFIG_SPI_PARROT7_MODULE) */
#if defined(CONFIG_PL330_DMA) || defined(CONFIG_PL330_DMA_MODULE)
    {	/* DMA controller. */
		.dev_id		= "pl330-dma.0",
		.clk		= (struct clk*) &p7_dma_clk,
    },
#endif /* defined(CONFIG_PL330_DMA) || defined(CONFIG_PL330_DMA_MODULE) */
#if defined(CONFIG_GPU_M200) || defined(CONFIG_GPU_M200_MODULE)
	{   /* GPU APB. */
		.dev_id = "m200-gpu.0",
		.con_id = "bus",
		.clk    = (struct clk*) &p7_gpu_busclk,
	},
	{   /* GPU core. */
		.dev_id = "m200-gpu.0",
		.con_id = "core",
		.clk    = (struct clk*) &p7_gpu_coreclk,
	},
#endif /* defined(CONFIG_GPU_M200) || defined(CONFIG_GPU_M200_MODULE) */
};

static int __init vexpress_p7_init(void)
{
	struct avifb_platform_data *avifb_pdata;
	int res = 0, i;

	printk(KERN_INFO "VExpress Initialisation\n");

	init_DMC();

	/* Register the dummy clocks */
	clkdev_add_table(p7_dummy_clocks, ARRAY_SIZE(p7_dummy_clocks));

#if defined (CONFIG_I2CM_PARROT7) || defined(CONFIG_I2CM_PARROT7_MODULE) \
		|| defined (CONFIG_I2CM_PARROT7_LEGACY) \
		|| defined(CONFIG_I2CM_PARROT7_LEGACY_MODULE)
	if (/*(res = do_device_register(&p7_i2cm0_dev)) ||
	    (res = do_device_register(&p7_i2cm1_dev)) ||*/
	    (res = do_device_register(&p7_i2cm2_dev)) /*||
	    (res = do_device_register(&p7_i2cm3_dev))*/)
		goto out;
	printk(KERN_INFO ". VExpress I2C master Initialisation\n");
#endif /* defined (CONFIG_I2CM_PARROT7) || defined(CONFIG_I2CM_PARROT7_MODULE) */

#if defined (CONFIG_PLDS_I2CS) || defined(CONFIG_PLDS_I2CS_MODULE)
	if ((res = do_device_register(&p7_i2cs0_dev)))
		goto out;
	printk(KERN_INFO ". VExpress I2C slave Initialisation\n");
#endif /* defined (CONFIG_PLDS_I2CS) || defined(CONFIG_PLDS_I2CS_MODULE) */

#if defined (CONFIG_AVI) || defined(CONFIG_AVI_MODULE)
	if ((res = do_device_register(&rnb6_avi_dev)))
		goto out;
	p7_enable_clkgbc(P7_AVI_GBC, AVI_GBC_CLOCK, AVI_GBC_RESET, 1);
	printk(KERN_INFO ". VExpress AVI Initialisation\n");
#endif /* defined (CONFIG_AVI) || defined(CONFIG_AVI_MODULE) */

#if defined (CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE)

#ifdef CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD0_ENABLE
	if ((res = do_device_register(&rnb6_avifb0_dev)))
		goto out;
	avifb_pdata=(struct avifb_platform_data *)rnb6_avifb0_dev.dev.platform_data;
	for (i = 0; i < avifb_pdata->overlay_nr; i++)
	{
		if ((res = do_alloc_resource(&rnb6_avifb0_dev,
				(struct resource *)&avifb_pdata->overlays[i].dma_memory)))
		goto out;
	}
	printk(KERN_INFO ". VExpress AVI LCD0 Initialisation\n");
#endif /* CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD0_ENABLE */

#ifdef CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD1_ENABLE
	if ((res = do_device_register(&rnb6_avifb1_dev)))
		goto out;
	avifb_pdata=(struct avifb_platform_data *)rnb6_avifb1_dev.dev.platform_data;
	for (i = 0; i < avifb_pdata->overlay_nr; i++)
	{
		if ((res = do_alloc_resource(&rnb6_avifb1_dev,
				(struct resource *)&avifb_pdata->overlays[i].dma_memory)))
		goto out;
	}
	printk(KERN_INFO ". VExpress AVI LCD1 Initialisation\n");
#endif /* CONFIG_ARCH_PARROT7_VEXPRESS_RNB6_LCD1_ENABLE */

#endif /* defined (CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE) */

#if defined (CONFIG_CAM_AVI) || defined(CONFIG_CAM_AVI_MODULE)
	//rnb6_cam1_res[1].start=0;
	//rnb6_cam1_res[1].end=CAMERA_AVI_RAM_SIZE-1;
	if ((res = do_device_register(&rnb6_cam1_dev)) ||
	    (res = do_device_register(&rnb6_cam1_soc_pdev)))
		goto out;

	printk(KERN_INFO ". VExpress AVI CAM1 Initialisation: %s, %d bytes\n",
			CAMERA_NAME,
			CAMERA_AVI_RAM_SIZE);
#endif /* defined (CONFIG_CAM_AVI) || defined(CONFIG_CAM_AVI_MODULE) */

#if defined(CONFIG_R2R_AVI) || defined(CONFIG_R2R_AVI_MODULE)
	if ((res = do_device_register(&p7_avi_r2r_dev)))
		goto out;

	printk(KERN_INFO ". VExpress AVI RAM to RAM Initialisation\n");

#if defined(CONFIG_R2R_AVI_TEST) || defined(CONFIG_R2R_AVI_TEST_MODULE)
	if ((res = do_device_register(&p7_avi_r2r_test_dev)))
		goto out;
	printk(KERN_INFO ". VExpress AVI RAM to RAM Initialisation: Test \n");
#endif /* defined(CONFIG_R2R_AVI_TEST) || defined(CONFIG_R2R_AVI_TEST_MODULE) */

#if defined(CONFIG_R2R_AVI_TEST_MULTI) || defined(CONFIG_R2R_AVI_TEST_MULTI_MODULE)
#if CONFIG_R2R_AVI_TEST_MULTI_NUMBER>=1
	for (i = 0; i < CONFIG_R2R_AVI_TEST_MULTI_NUMBER; i++) {
		if ((res = do_device_register(&p7_avi_r2r_test_multi_dev[i])))
			goto out;
	}
	printk(KERN_INFO ". VExpress AVI RAM to RAM Initialisation: "
			"Test %d Clients\n",
			CONFIG_R2R_AVI_TEST_MULTI_NUMBER);
#else
#warning "Number of clients is 0 for test module of AVI R2R"
#endif
#endif /* defined(CONFIG_R2R_AVI_TEST_MULTI) || defined(CONFIG_R2R_AVI_TEST_MULTI_MODULE) */

#endif /* defined(CONFIG_R2R_AVI) || defined(CONFIG_R2R_AVI_MODULE) */

#if defined(CONFIG_VDEC_HX270) || defined(CONFIG_VDEC_HX270_MODULE)
	if ((res = do_device_register(&p7_vdec_dev)))
		goto out;
	p7_enable_clkgbc(P7_VDEC_GBC, VDEC_GBC_CLOCK, VDEC_GBC_RESET, 1);
	printk(KERN_INFO ". VExpress VDEC HX270 init\n");
#endif /* defined(CONFIG_VDEC_HX270) || defined(CONFIG_VDEC_HX270_MODULE) */

	#if defined(CONFIG_SPI_PARROT7) || defined(CONFIG_SPI_PARROT7_MODULE)
	if ((res = do_device_register(&p7_spi_dev)))
		goto out;
#endif /* defined(CONFIG_SPI_PARROT7) || defined(CONFIG_SPI_PARROT7_MODULE) */

#if defined(CONFIG_SPI_MASTER_PARROT7) || defined(CONFIG_SPI_MASTER_PARROT7_MODULE)
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(p7_spi_devs); i++)
	{
		if ((res = do_device_register(&p7_spi_devs[i])))
		goto out;
	}
	spi_register_board_info(p7_spi_board_info, ARRAY_SIZE(p7_spi_board_info));
	printk(KERN_INFO ". VExpress SPI master Initialisation\n");
}
#endif /* defined(CONFIG_SPI_MASTER_PARROT7) || defined(CONFIG_SPI_MASTER_PARROT7_MODULE) */

#if defined(CONFIG_P7_SPIS) || defined(CONFIG_P7_SPIS_MODULE)
	if ((res = do_device_register(&p7_spis_dev)))
		goto out;
	printk(KERN_INFO ". VExpress SPI slave Initialisation\n");
#endif

#if defined(CONFIG_MPEGTS_PARROT7) || defined(CONFIG_MPEGTS_PARROT7_MODULE)
	if ((res = do_device_register(&p7_mpegts_dev[0])))
		goto out;
	printk(KERN_INFO ". VExpress MPEGTS Initialisation\n");
#endif

#if defined(CONFIG_PL330_DMA) || defined(CONFIG_PL330_DMA_MODULE)
	if (dma_declare_coherent_memory(&p7_dma_dev.dev,
									P7_DMA_UCODE,
									P7_DMA_UCODE,
	                                PAGE_ALIGN(p7_dma_pdata.mcbuf_sz *
	                                           ARRAY_SIZE(p7_dma_peri)),
	                                DMA_MEMORY_MAP |
	                                DMA_MEMORY_EXCLUSIVE) & DMA_MEMORY_MAP) {
		dev_info(&p7_dma_dev.dev,
		         "mapped microcode memory region [%08x:%08x]\n",
				 P7_DMA_UCODE,
				 P7_DMA_UCODE + PAGE_ALIGN(p7_dma_pdata.mcbuf_sz *
	                                       ARRAY_SIZE(p7_dma_peri)) - 1);
		dma_cap_set(DMA_SLAVE, p7_dma_pdata.cap_mask);
		dma_cap_set(DMA_CYCLIC, p7_dma_pdata.cap_mask);
		dma_cap_set(DMA_MEMCPY, p7_dma_pdata.cap_mask);
		res = amba_device_register(&p7_dma_dev, &iomem_resource);
		if (res)
			goto out;
		p7_enable_clkgbc(P7_DMA_GBC, P7_DMA_GBC_CLOCK, P7_DMA_GBC_RESET, 1);
	}
	else {
		res = -ENOMEM;
		goto out;
	}
	printk(KERN_INFO ". VExpress PL330 DMA Initialisation\n");
#endif /* defined(CONFIG_PL330_DMA) || defined(CONFIG_PL330_DMA_MODULE) */

#if defined(CONFIG_GPU_M200) || defined(CONFIG_GPU_M200_MODULE)
	if ((res = do_device_register(&p7_gpu_dev)))
		goto out;
	p7_enable_clkgbc(P7_GPU_GBC, P7_GPU_GBC_CLOCK, P7_GPU_GBC_RESET, 1);
	printk(KERN_INFO ". VExpress GPU M200 Initialisation\n");
#endif /*  defined(CONFIG_GPU_M200) || defined(CONFIG_GPU_M200_MODULE) */

	if (IS_ERR_OR_NULL(debugfs_create_file("vexpress_rnb6", S_IRUGO,
						   NULL,
						   (&rnb6_show),
						   &rnb6_debugfs_fops))) {
		pr_warn("RNB6: failed to initialize debugfs for VExpress RNB6 board\n");
	}

out:
	return res;
}
arch_initcall(vexpress_p7_init);
