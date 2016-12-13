#include "lcd-monspecs.h"

/* The 7" OEM screen */
/* The screen used in the RNB5. 6.2" with capacitive touchscreen */
struct avi_videomode fg0700k_video_mode = {
	.name           = "fg0700k4dsswbgt1",
	.xres           = 800,
	.yres           = 480,
	.pixclock       = 33260,
	.left_margin	= 104,/* Hback  */
	.right_margin	= 24, /* Hfront */
	.upper_margin	= 12, /* Vback  */
	.lower_margin	= 3,  /* Vfront */
	.hsync_len	= 80,
	.vsync_len	= 7,
	.flags          = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
};

/* The 10" screen */
struct avi_videomode lt104ac_video_mode = {
	.name           = "lt104ac",
	.xres           = 1024,
	.yres           = 768,
	.pixclock       = 65000,
	.left_margin	= 160,
	.right_margin	= 24,
	.upper_margin	= 29,
	.lower_margin	= 3,
	.hsync_len	= 136,
	.vsync_len	= 6,
	.flags          = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
};

struct avi_videomode lt104ac_no_den_video_mode = {
	.name           = "lt104ac_no_den",
	.xres           = 1024,
	.yres           = 768,
	.pixclock       = 65000,
	.left_margin	= 296,
	.right_margin	= 24,
	.upper_margin	= 34,
	.lower_margin	= 3,
	.hsync_len	= 136,
	.vsync_len	= 6,
	.flags          = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
};


/* The Antec 9" Screen */
struct avi_videomode chimei_antec_video_mode = {
	.name           = "chimei-antec",
	.xres           = 800,
	.yres           = 480,
	.pixclock       = 33300,
	.flags          = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
	.left_margin	= 46, /* Hback  */
	.right_margin	= 210, /* Hfront */
	.upper_margin	= 23, /* Vback  */
	.lower_margin	= 22, /* Vfront */
	.hsync_len	= 80,
	.vsync_len	= 7,
};

/* The FC6100 workbench 7' Screen */
struct avi_videomode tpo_laj07t001a_video_mode = {
        .name           = "tpo_laj07t001a",
        .xres           = 800,
        .yres           = 480,
        .pixclock       = 33230,
        .flags          = 0,
        .left_margin    = 128, /* Hback  */
        .right_margin   = 49, /* Hfront */
        .upper_margin   = 34, /* Vback  */
        .lower_margin   = 10, /* Vfront */
        .hsync_len      = 19,
        .vsync_len      = 2,
};

/* OEM FC7100 kyocera screen */
struct avi_videomode kyocera_video_mode = {
	.name           = "kyocera",
	.xres           = 1280,
	.yres           = 800,
	.pixclock       = 71100,
	/* The rest of the timings will be calculated using the VESA
	 * generalized timing formula. */
	.flags          = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
	.hsync_len      = 100,
	.left_margin    = 30,
	.right_margin   = 30,
	.vsync_len      = 5,
	.upper_margin   = 16,
	.lower_margin   = 2,
	/* Don't attempt to validate monitor timings */
};

struct avi_videomode porsche_rse_video_mode = {
	.name           = "porsche_rse",
	.xres           = 1280,
	.yres           = 800,
	.pixclock       = 80500,
	/* The rest of the timings will be calculated using the VESA
	 * generalized timing formula. */
	.flags          = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
	.hsync_len      = 200,
	.left_margin    = 10,
	.right_margin   = 10,
	.vsync_len      = 67,
	.upper_margin   = 5,
	.lower_margin   = 5,
};

struct avi_videomode drse_video_mode = {
	.name           = "drse",
	.xres           = 1280,
	.yres           = 800,
	.pixclock       = 71100,
	/* The rest of the timings will be calculated using the VESA
	 * generalized timing formula. */
	.flags          = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
	.hsync_len      = 100,
	.left_margin    = 30,
	.right_margin   = 30,
	.vsync_len      = 5,
	.upper_margin   = 16,
	.lower_margin   = 2,
};

struct avi_videomode rnb6_truly_video_mode = {
	.name	      = "rnb6_truly_video_mode",
	.xres	      = 720,
	.yres	      = 1280,
	.flags	      = 0,
	.hsync_len    = 8,
	.left_margin  = 45,
	.right_margin = 70,
	.vsync_len    = 2,
	.upper_margin = 15,
	.lower_margin = 16,
	.pixclock     = 66412,
};

struct avi_videomode ntsc_720_video_mode = {
	.name	      = "ntsc_720",
	.xres	      = 720,
	.yres	      = 480,
	.flags	      = AVI_VIDEOMODE_INTERLACED,

	.hsync_len    = 62,
	.left_margin  = 57,
	.right_margin = 19,

	.vsync_len    = 5,
	.upper_margin = 15,
	.lower_margin = 2,

	.pixclock     = 13500,
};

struct avi_videomode ntsc_640_video_mode = {
	.name	      = "ntsc_640",
	.xres	      = 640,
	.yres	      = 480,
	.flags	      = AVI_VIDEOMODE_INTERLACED,

	.hsync_len    = 69,
	.left_margin  = 57,
	.right_margin = 14,

	.vsync_len    = 5,
	.upper_margin = 15,
	.lower_margin = 2,

	/* The AVI PLL can not actually generate anything under 12.5MHz, let's
	 * hope this is within tolerance. */
	.pixclock     = 12273,
};

/* OEM FC7100 Vovo truck screen: TRULY SEMICONDUCTORS 2K31715 */
struct avi_videomode tft800480_video_mode = {
	.name           = "2K31715",
	.xres           = 800,
	.yres           = 480,

	.pixclock       = 30000,

	.flags          = 0,

	.hsync_len      = 48,
	.left_margin    = 41,
	.right_margin   = 40,

	.vsync_len      = 3,
	.upper_margin   = 29,
	.lower_margin   = 13,

};

/* OEM FC7100 BORGWARD screen:  */
struct avi_videomode borgward_12_video_mode= {
	.name           = "borgward_12",
	.xres           = 1360,
	.yres           = 540,

	.pixclock       = 50000,

	.flags          = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,

	.hsync_len      = 5,
	.left_margin    = 25,
	.right_margin   = 28,

	.vsync_len      = 2,
	.upper_margin   = 0,
	.lower_margin   = 0,

};

/*************************************************/
/********** HDMI OUT section *********************/
/*************************************************/

/* CEA 861D/TV Formats */

struct avi_videomode hdmi_720x576p50_video_mode = {
	.name	      = "hdmi_720x576p50",
	.xres	      = 720,
	.yres	      = 576,
	.flags	      = 0,
	/* As per CEA-861-D.pdf page 25 720x576p @50 Hz (Formats 17 & 18) */
	.hsync_len    = 64,
	.left_margin  = 68,
	.right_margin = 12,
	.vsync_len    = 5,
	.upper_margin = 39,
	.lower_margin = 5,
	.pixclock     = 27000,
};

struct avi_videomode hdmi_720x576i50_video_mode = {
	.name	      = "hdmi_720x576i50",
	.xres	      = 720,
	.yres	      = 576,
	.flags	      = AVI_VIDEOMODE_INTERLACED,
	/* As per CEA-861-D.pdf page 26 720(1440)x576i @50 Hz (Formats 21 & 22)
	 * Horizontal timings, horizontal resolution and pixel clock
	 * are divided by two from original values of CEA-861
	 */
	.hsync_len    = 63,
	.left_margin  = 69,
	.right_margin = 12,
	.vsync_len    = 3,
	.upper_margin = 19,
	.lower_margin = 2,
	.pixclock     = 13500,
};

struct avi_videomode hdmi_720x480p60_video_mode = {
	.name	      = "hdmi_720x480p60",
	.xres	      = 720,
	.yres	      = 480,
	.flags	      = 0,
	/* As per CEA-861-D.pdf page 21 720x480p @59.94/60 Hz (Formats 2 & 3) */
	.hsync_len    = 62,
	.left_margin  = 60,
	.right_margin = 16,
	.vsync_len    = 6,
	.upper_margin = 30,
	.lower_margin = 9,
	.pixclock     = 27027,
};

struct avi_videomode hdmi_720x480p5994_video_mode = {
	.name	      = "hdmi_720x480p59.94",
	.xres	      = 720,
	.yres	      = 480,
	.flags	      = 0,
	/* As per CEA-861-D.pdf page 21 720x480p @59.94/60 Hz (Formats 2 & 3) */
	.hsync_len    = 62,
	.left_margin  = 60,
	.right_margin = 16,
	.vsync_len    = 6,
	.upper_margin = 30,
	.lower_margin = 9,
	.pixclock     = 27000,
};

struct avi_videomode hdmi_720x480i60_video_mode = {
	.name	      = "hdmi_720x480i60",
	.xres	      = 720,
	.yres	      = 480,
	.flags	      = AVI_VIDEOMODE_INTERLACED,
	/* As per CEA-861-D.pdf page 22  720(1440)x480i @59.94/60 Hz (Formats 6 & 7)
	 * Horizontal timings, horizontal resolution and pixel clock
	 * are divided by two from original values of CEA-861
	 */
	.hsync_len    = 62,
	.left_margin  = 57,
	.right_margin = 19,
	.vsync_len    = 3,
	.upper_margin = 15,
	.lower_margin = 4,
	.pixclock     = 13513,
};

struct avi_videomode hdmi_1440x480p60_video_mode = {
	.name	      = "hdmi_1440x480p60",
	.xres	      = 1440,
	.yres	      = 480,
	.flags	      = 0,
	/* As per CEA-861-D.pdf page 32 1440x480p @59.94/60 Hz (Formats 14 & 15) */
	.hsync_len    = 124,
	.left_margin  = 120,
	.right_margin = 32,
	.vsync_len    = 6,
	.upper_margin = 30,
	.lower_margin = 9,
	.pixclock     = 54054,
};

struct avi_videomode hdmi_1440x576p50_video_mode = {
	.name	      = "hdmi_1440x576p50",
	.xres	      = 1440,
	.yres	      = 576,
	.flags	      = 0,
	/* As per CEA-861-D.pdf 1440x576p @50 Hz (Formats 29 & 30) */
	.hsync_len    = 128,
	.left_margin  = 136,
	.right_margin = 24,
	.vsync_len    = 5,
	.upper_margin = 39,
	.lower_margin = 5,
	.pixclock     = 54000,
};

struct avi_videomode hdmi_1280x720p24_video_mode = {
	.name	      = "hdmi_1280x720p24", /* vic#60 */
	.xres	      = 1280,
	.yres	      = 720,
	.flags	      = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
	/* As per CEA-861-E.pdf page 16/17 1280x720p @24 Hz (Format 60) */
	.hsync_len    = 40,
	.left_margin  = 220,
	.right_margin = 1760,
	.vsync_len    = 5,
	.upper_margin = 20,
	.lower_margin = 5,
	.pixclock     = 59400,
};

struct avi_videomode hdmi_1280x720p25_video_mode = {
	.name	      = "hdmi_1280x720p25", /* vic#61 */
	.xres	      = 1280,
	.yres	      = 720,
	.flags	      = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
	/* As per CEA-861-E.pdf page 16/17 1280x720p @25 Hz (Format 61) */
	.hsync_len    = 40,
	.left_margin  = 220,
	.right_margin = 2420,
	.vsync_len    = 5,
	.upper_margin = 20,
	.lower_margin = 5,
	.pixclock     = 74250,
};

struct avi_videomode hdmi_1280x720p30_video_mode = {
	.name	      = "hdmi_1280x720p30", /* vic#62 */
	.xres	      = 1280,
	.yres	      = 720,
	.flags	      = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
	/* As per CEA-861-E.pdf page 16/17 1280x720p @30 Hz (Format 62) */
	.hsync_len    = 40,
	.left_margin  = 220,
	.right_margin = 1760,
	.vsync_len    = 5,
	.upper_margin = 20,
	.lower_margin = 5,
	.pixclock     = 74250,
};

struct avi_videomode hdmi_1280x720p50_video_mode = {
	.name	      = "hdmi_1280x720p50", /* vic#19 */
	.xres	      = 1280,
	.yres	      = 720,
	.flags	      = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
	/* As per CEA-861-D.pdf page 23 1280x720p @50 Hz (Format 19) */
	.hsync_len    = 40,
	.left_margin  = 220,
	.right_margin = 440,
	.vsync_len    = 5,
	.upper_margin = 20,
	.lower_margin = 5,
	.pixclock     = 74250,
};

struct avi_videomode hdmi_1280x720p60_video_mode = {
	.name	      = "hdmi_1280x720p60", /* vic#4 */
	.xres	      = 1280,
	.yres	      = 720,
	.flags	      = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
	/* As per CEA-861-D.pdf page 19 1280x720p @59.94/60 Hz (Format 4) */
	.hsync_len    = 40,
	.left_margin  = 220,
	.right_margin = 110,
	.vsync_len    = 5,
	.upper_margin = 20,
	.lower_margin = 5,
	.pixclock     = 74250,
};

struct avi_videomode hdmi_1920x1080p30_video_mode = {
	.name	      = "hdmi_1920x1080p30", /* vic#34 */
	.xres	      = 1920,
	.yres	      = 1080,
	.flags	      = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
	/* As per CEA-861-D.pdf 1920x1080p @ 30Hz (Format 34) */
	.hsync_len    = 44,
	.left_margin  = 148,
	.right_margin = 88,
	.vsync_len    = 5,
	.upper_margin = 36,
	.lower_margin = 4,
	.pixclock     = 74250,
};

struct avi_videomode hdmi_1920x1080p24_video_mode = {
	.name	      = "hdmi_1920x1080p24", /* vic#32 */
	.xres	      = 1920,
	.yres	      = 1080,
	.flags	      = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
	/* As per CEA-861-D.pdf 1920x1080p @ 24Hz (Format 32) */
	.hsync_len    = 44,
	.left_margin  = 148,/* Hback */
	.right_margin = 638,/* Hfront */
	.vsync_len    = 5,
	.upper_margin = 36,/* Vback */
	.lower_margin = 4,/* Vfront */
	.pixclock     = 74250,
};

struct avi_videomode hdmi_1920x1080p25_video_mode = {
	.name	      = "hdmi_1920x1080p25", /* vic#33 */
	.xres	      = 1920,
	.yres	      = 1080,
	.flags	      = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
	/* As per CEA-861-D.pdf 1920x1080p @ 25Hz (Format 33) */
	.hsync_len    = 44,
	.left_margin  = 148,/* Hback */
	.right_margin = 528,/* Hfront */
	.vsync_len    = 5,
	.upper_margin = 36,/* Vback */
	.lower_margin = 4,/* Vfront */
	.pixclock     = 74250,
};

struct avi_videomode hdmi_1920x1080i50_video_mode = {
	.name	      = "hdmi_1920x1080i50", /* vic#20 */
	.xres	      = 1920,
	.yres	      = 1080,
	.flags	      = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH
                        | AVI_VIDEOMODE_INTERLACED,
	/* As per CEA-861-D.pdf page 24  1920x1080i @50 Hz (Format 20) */
	.hsync_len    = 44,
	.left_margin  = 148,
	.right_margin = 528,
	.vsync_len    = 5,
	.upper_margin = 15,
	.lower_margin = 2,
	.pixclock     = 74250,
};

struct avi_videomode hdmi_1920x1080p50_video_mode = {
	.name	      = "hdmi_1920x1080p50", /* vic#31 */
	.xres	      = 1920,
	.yres	      = 1080,
	.flags	      = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
	/* As per CEA-861-D.pdf 1920x1080p @ 50Hz (Format 31) */
	.hsync_len    = 44,
	.left_margin  = 148,/* Hback */
	.right_margin = 528,/* Hfront */
	.vsync_len    = 5,
	.upper_margin = 36,/* Vback */
	.lower_margin = 4,/* Vfront */
	.pixclock     = 148500,
};

struct avi_videomode hdmi_1920x1080i60_video_mode = {
	.name	      = "hdmi_1920x1080i60", /* vic#5 */
	.xres	      = 1920,
	.yres	      = 1080,
	.flags	      = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH
                        | AVI_VIDEOMODE_INTERLACED,
	/* As per CEA-861-D.pdf page 20  1920x1080i @59.94/60 Hz (Format 5) */
	.hsync_len    = 44,
	.left_margin  = 148,
	.right_margin = 88,
	.vsync_len    = 5,
	.upper_margin = 15,
	.lower_margin = 2,
	.pixclock     = 74250,
};

struct avi_videomode hdmi_1920x1080i5994_video_mode = {
	.name	      = "hdmi_1920x1080i59.94", /* vic#5 */
	.xres	      = 1920,
	.yres	      = 1080,
	.flags	      = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH
                        | AVI_VIDEOMODE_INTERLACED,
	/* As per CEA-861-D.pdf page 20  1920x1080i @59.94/60 Hz (Format 5) */
	.hsync_len    = 44,
	.left_margin  = 148,
	.right_margin = 88,
	.vsync_len    = 5,
	.upper_margin = 15,
	.lower_margin = 2,
	.pixclock     = 74176,
};

struct avi_videomode hdmi_1920x1080p60_video_mode = {
	.name	      = "hdmi_1920x1080p60", /* vic#16 */
	.xres	      = 1920,
	.yres	      = 1080,
	.flags	      = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
	/* As per CEA-861-D.pdf page 33  1920x1080p @ 59.94/60Hz (Format 16) */
	.hsync_len    = 44,
	.left_margin  = 148,/* Hback */
	.right_margin = 88,/* Hfront */
	.vsync_len    = 5,
	.upper_margin = 36,/* Vback */
	.lower_margin = 4,/* Vfront */
	.pixclock     = 148500,
};

/* VESA Formats */

struct avi_videomode hdmi_1024x768p60_video_mode = {
	.name	      = "hdmi_1024x768p60", /* vesa */
	.xres	      = 1024,
	.yres	      = 768,
	.flags	      = 0,
	.hsync_len    = 136,
	.left_margin  = 160,
	.right_margin = 24,
	.vsync_len    = 6,
	.upper_margin = 29,
	.lower_margin = 3,
	.pixclock     = 65003,
};

struct avi_videomode hdmi_1024x768p75_video_mode = {
	.name	      = "hdmi_1024x768p75", /* vesa */
	.xres	      = 1024,
	.yres	      = 768,
	.flags	      = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
	.hsync_len    = 96,
	.left_margin  = 176,
	.right_margin = 16,
	.vsync_len    = 3,
	.upper_margin = 28,
	.lower_margin = 1,
	.pixclock     = 78802,
};

struct avi_videomode hdmi_1024x768p85_video_mode = {
	.name	      = "hdmi_1024x768p85", /* vesa */
	.xres	      = 1024,
	.yres	      = 768,
	.flags	      = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
	.hsync_len    = 96,
	.left_margin  = 208,
	.right_margin = 48,
	.vsync_len    = 3,
	.upper_margin = 36,
	.lower_margin = 1,
	.pixclock     = 94500,
};

struct avi_videomode hdmi_800x600p75_video_mode = {
	.name	      = "hdmi_800x600p75",  /* vesa */
	.xres	      = 800,
	.yres	      = 600,
	.flags	      = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
	.hsync_len    = 80,
	.left_margin  = 160,
	.right_margin = 16,
	.vsync_len    = 3,
	.upper_margin = 21,
	.lower_margin = 1,
	.pixclock     = 49500,
};

struct avi_videomode hdmi_800x600p85_video_mode = {
	.name	      = "hdmi_800x600p85", /* vesa */
	.xres	      = 800,
	.yres	      = 600,
	.flags	      = AVI_VIDEOMODE_SYNC_ACTIVE_HIGH,
	.hsync_len    = 64,
	.left_margin  = 152,
	.right_margin = 32,
	.vsync_len    = 3,
	.upper_margin = 27,
	.lower_margin = 1,
	.pixclock     = 56250,
};

struct avi_videomode hdmi_640x480p75_video_mode = {
	.name	      = "hdmi_640x480p75", /* vesa */
	.xres	      = 640,
	.yres	      = 480,
	.flags	      = 0,
	.hsync_len    = 64,
	.left_margin  = 120,
	.right_margin = 16,
	.vsync_len    = 3,
	.upper_margin = 16,
	.lower_margin = 1,
	.pixclock     = 31500,
};

struct avi_videomode sfx1_tegra_640x3360p32_video_mode = {
	.name	      = "sfx1_tegra_640x3360p32", /* sfx1 */
	.xres	      = 640,
	.yres	      = 3361, // One additionnal line for metadata
	.flags	      = 0,
	.hsync_len    = 32,
	.left_margin  = 16,
	.right_margin = 16,
	.vsync_len    = 54,
	.upper_margin = 16,
	.lower_margin = 1,
	.pixclock     = 77300,
};

struct avi_videomode sfx1_tegra_640x2401p32_video_mode = {
	.name	      = "sfx1_tegra_640x2401p32", /* sfx1 */
	.xres	      = 640,
	.yres	      = 2401, // One additionnal line for metadata
	.flags	      = 0,
	.hsync_len    = 32,
	.left_margin  = 16,
	.right_margin = 16,
	.vsync_len    = 54,
	.upper_margin = 16,
	.lower_margin = 1,
	.pixclock     = 56700,
};

struct avi_videomode hdmi_640x480p85_video_mode = {
	.name	      = "hdmi_640x480p85", /* vesa */
	.xres	      = 640,
	.yres	      = 480,
	.flags	      = 0,
	.hsync_len    = 48,
	.left_margin  = 112,
	.right_margin = 32,
	.vsync_len    = 3,
	.upper_margin = 25,
	.lower_margin = 1,
	.pixclock     = 36000,
};

struct avi_videomode hdmi_1280x1024p60_video_mode = {
	.name	      = "hdmi_1280x1024p60", /* vesa */
	.xres	      = 1280,
	.yres	      = 1024,
	.flags	      = 0,
	.hsync_len    = 112,
	.left_margin  = 248,
	.right_margin = 48,
	.vsync_len    = 3,
	.upper_margin = 38,
	.lower_margin = 1,
	.pixclock     = 108000,
};

struct avi_videomode hdmi_1280x1024p75_video_mode = {
	.name	      = "hdmi_1280x1024p75", /* vesa */
	.xres	      = 1280,
	.yres	      = 1024,
	.flags	      = 0,
	.hsync_len    = 144,
	.left_margin  = 248,
	.right_margin = 16,
	.vsync_len    = 3,
	.upper_margin = 38,
	.lower_margin = 1,
	.pixclock     = 135000,
};

struct avi_videomode hdmi_1280x1024p85_video_mode = {
	.name	      = "hdmi_1280x1024p85", /* vesa */
	.xres	      = 1280,
	.yres	      = 1024,
	.flags	      = 0,
	.hsync_len    = 160,
	.left_margin  = 240,
	.right_margin = 48,
	.vsync_len    = 3,
	.upper_margin = 44,
	.lower_margin = 1,
	.pixclock     = 157500,
};

const struct avi_videomode *p7_all_video_modes[] = {
	&fg0700k_video_mode,
	&lt104ac_video_mode,
	&lt104ac_no_den_video_mode,
	&kyocera_video_mode,
	&porsche_rse_video_mode,
	&drse_video_mode,
	&chimei_antec_video_mode,
	&tpo_laj07t001a_video_mode,
	&ntsc_720_video_mode,
	&ntsc_640_video_mode,
	&tft800480_video_mode,
	&rnb6_truly_video_mode,
	&hdmi_720x480p60_video_mode,
	&hdmi_720x480p5994_video_mode,
	&hdmi_720x480i60_video_mode,
	&hdmi_1440x480p60_video_mode,
	&hdmi_720x576p50_video_mode,
	&hdmi_720x576i50_video_mode,
	&hdmi_1440x576p50_video_mode,
	&hdmi_1280x720p24_video_mode,
	&hdmi_1280x720p25_video_mode,
	&hdmi_1280x720p50_video_mode,
	&hdmi_1280x720p30_video_mode,
	&hdmi_1280x720p60_video_mode,
	&hdmi_1920x1080p24_video_mode,
	&hdmi_1920x1080p25_video_mode,
	&hdmi_1920x1080i50_video_mode,
	&hdmi_1920x1080p50_video_mode,
	&hdmi_1920x1080p30_video_mode,
	&hdmi_1920x1080i60_video_mode,
	&hdmi_1920x1080i5994_video_mode,
	&hdmi_1920x1080p60_video_mode,
	&hdmi_1280x1024p60_video_mode,
	&hdmi_1280x1024p75_video_mode,
	&hdmi_1280x1024p85_video_mode,
	&hdmi_1024x768p60_video_mode,
	&hdmi_1024x768p75_video_mode,
	&hdmi_1024x768p85_video_mode,
	&hdmi_800x600p75_video_mode,
	&hdmi_800x600p85_video_mode,
	&hdmi_640x480p75_video_mode,
	&hdmi_640x480p85_video_mode,
	NULL /* end of table */
};
