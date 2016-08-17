/**
 * linux/arch/arm/mach-parrot7/lcd-monspecs.h
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Lionel Flandrin <lionel.flandrin@parrot.com>
 * date:    08-Oct-2012
 *
 * This file is released under the GPL
 */

#ifndef _LCD_MONSPECS_H_
#define _LCD_MONSPECS_H_


#include <video/avi.h>

/* The 7" OEM screen */
/* The screen used in the RNB5. 6.2" with capacitive touchscreen */
extern struct avi_videomode fg0700k_video_mode;
/* The 10" screen */
extern struct avi_videomode lt104ac_video_mode;
/* When DEN is not wired */
extern struct avi_videomode lt104ac_no_den_video_mode;
/* OEM kyocera */
extern struct avi_videomode kyocera_video_mode;
/* The Antec 9" Screen */
extern struct avi_videomode chimei_antec_video_mode;
/* PRSE */
extern struct avi_videomode porsche_rse_video_mode;
/* DRSE */
extern struct avi_videomode drse_video_mode;

extern struct avi_videomode ntsc_720_video_mode;
extern struct avi_videomode ntsc_640_video_mode;

/* OEM FC7100 Vovo truck screen: TRULY SEMICONDUCTORS 2K31715 */
extern struct avi_videomode tft800480_video_mode;

/* OEM FC7100 Borgward screen */
extern struct avi_videomode borgward_12_video_mode;

/* RNB6 screen 720x1280 */
extern struct avi_videomode rnb6_truly_video_mode;

/**** HDMI Output ****/
/* CEA 861D/TV Formats */
/* SD TV */
/*  NTSC */
extern struct avi_videomode hdmi_720x480p60_video_mode;
extern struct avi_videomode hdmi_720x480p5994_video_mode;
extern struct avi_videomode hdmi_720x480i60_video_mode;
extern struct avi_videomode hdmi_1440x480p60_video_mode;

/*  PAL */
extern struct avi_videomode hdmi_720x576p50_video_mode;
extern struct avi_videomode hdmi_720x576i50_video_mode;
extern struct avi_videomode hdmi_1440x576p50_video_mode;

/* HD TV HD Ready */
/*  Cinema */
extern struct avi_videomode hdmi_1280x720p24_video_mode;
/*  Europe TV */
extern struct avi_videomode hdmi_1280x720p25_video_mode;
extern struct avi_videomode hdmi_1280x720p50_video_mode;
/*  America/Japan TV */
extern struct avi_videomode hdmi_1280x720p30_video_mode;
extern struct avi_videomode hdmi_1280x720p60_video_mode;

/* HD TV Full HD */
/*  Cinema */
extern struct avi_videomode hdmi_1920x1080p24_video_mode;

/*  Europe TV */
extern struct avi_videomode hdmi_1920x1080p25_video_mode;
extern struct avi_videomode hdmi_1920x1080i50_video_mode;
/* requires 24 bit data bus separate syncs only */
extern struct avi_videomode hdmi_1920x1080p50_video_mode;

/*  America/Japan TV */
extern struct avi_videomode hdmi_1920x1080p30_video_mode;
extern struct avi_videomode hdmi_1920x1080i60_video_mode;
extern struct avi_videomode hdmi_1920x1080i5994_video_mode;
/* requires 24 bit data bus separate syncs only */
extern struct avi_videomode hdmi_1920x1080p60_video_mode;

/* VESA Formats */
/* formats p85 are not supported by HDTV monitor Samsung T22B300EW */
/* requires 24 bit data bus separate syncs only */
extern struct avi_videomode hdmi_1280x1024p60_video_mode;
/* requires 24 bit data bus separate syncs only */
extern struct avi_videomode hdmi_1280x1024p75_video_mode;
/* requires 24 bit data bus separate syncs only */
extern struct avi_videomode hdmi_1280x1024p85_video_mode;
extern struct avi_videomode hdmi_1024x768p60_video_mode;
extern struct avi_videomode hdmi_1024x768p75_video_mode;
/* requires 24 bit data bus separate syncs only */
extern struct avi_videomode hdmi_1024x768p85_video_mode;
/* not supported by HDTV monitor Samsung T22B300EW */
extern struct avi_videomode hdmi_800x600p75_video_mode;
extern struct avi_videomode hdmi_800x600p85_video_mode;
extern struct avi_videomode hdmi_640x480p75_video_mode;
extern struct avi_videomode sfx1_tegra_640x3360p32_video_mode;
extern struct avi_videomode sfx1_tegra_640x2401p32_video_mode;
extern struct avi_videomode hdmi_640x480p85_video_mode;

// Null terminated list of all the above modes
extern const struct avi_videomode *p7_all_video_modes[];


#endif /* _LCD_MONSPECS_H_ */
