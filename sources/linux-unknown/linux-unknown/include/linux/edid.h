/* vim: set noet ts=8 sts=8 sw=8 : */
/*
 * Copyright © 2010 Saleem Abdulrasool <compnerd@compnerd.org>.
 * Copyright © 2010 Genesi USA, Inc. <matt@genesi-usa.com>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LINUX_EDID_H
#define LINUX_EDID_H

/* EDID constants and Structures */
#define EDID_I2C_DDC_DATA_ADDRESS			(0x50)

#define EDID_BLOCK_SIZE					(0x80)
#define EDID_MAX_EXTENSIONS				(0xfe)


static const u8 EDID_HEADER[] = { 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00 };


enum edid_extension_type {
	EDID_EXTENSION_TIMING           = 0x01, // Timing Extension
	EDID_EXTENSION_CEA              = 0x02, // Additional Timing Block Data (CEA EDID Timing Extension)
	EDID_EXTENSION_VTB              = 0x10, // Video Timing Block Extension (VTB-EXT)
	EDID_EXTENSION_EDID_2_0         = 0x20, // EDID 2.0 Extension
	EDID_EXTENSION_DI               = 0x40, // Display Information Extension (DI-EXT)
	EDID_EXTENSION_LS               = 0x50, // Localised String Extension (LS-EXT)
	EDID_EXTENSION_MI               = 0x60, // Microdisplay Interface Extension (MI-EXT)
	EDID_EXTENSION_DTCDB_1          = 0xa7, // Display Transfer Characteristics Data Block (DTCDB)
	EDID_EXTENSION_DTCDB_2          = 0xaf,
	EDID_EXTENSION_DTCDB_3          = 0xbf,
	EDID_EXTENSION_BLOCK_MAP        = 0xf0, // Block Map
	EDID_EXTENSION_DDDB             = 0xff, // Display Device Data Block (DDDB)
};

enum edid_display_type {
	EDID_DISPLAY_TYPE_MONOCHROME,
	EDID_DISPLAY_TYPE_RGB,
	EDID_DISPLAY_TYPE_NON_RGB,
	EDID_DISPLAY_TYPE_UNDEFINED,
};

enum edid_aspect_ratio {
	EDID_ASPECT_RATIO_16_10,
	EDID_ASPECT_RATIO_4_3,
	EDID_ASPECT_RATIO_5_4,
	EDID_ASPECT_RATIO_16_9,
};

enum edid_monitor_descriptor_type {
	EDID_MONITOR_DESCRIPTOR_STANDARD_TIMING_IDENTIFIERS = 0xfa,
	EDID_MONITOR_DESCRIPTOR_COLOR_POINT                 = 0xfb,
	EDID_MONITOR_DESCRIPTOR_MONITOR_NAME                = 0xfc,
	EDID_MONITOR_DESCRIPTOR_MONITOR_RANGE_LIMITS        = 0xfd,
	EDID_MONITOR_DESCRIPTOR_MONITOR_SERIAL_NUMBER       = 0xff,
};


struct __packed edid_detailed_timing_descriptor {
	u16      pixel_clock;                   /* = value * 10000 */

	u8       horizontal_active_lo;
	u8       horizontal_blanking_lo;

	unsigned horizontal_blanking_hi         : 4;
	unsigned horizontal_active_hi           : 4;

	u8       vertical_active_lo;
	u8       vertical_blanking_lo;

	unsigned vertical_blanking_hi           : 4;
	unsigned vertical_active_hi             : 4;

	u8       horizontal_sync_offset_lo;
	u8       horizontal_sync_pulse_width_lo;

	unsigned vertical_sync_pulse_width_lo   : 4;
	unsigned vertical_sync_offset_lo        : 4;

	unsigned vertical_sync_pulse_width_hi   : 2;
	unsigned vertical_sync_offset_hi        : 2;
	unsigned horizontal_sync_pulse_width_hi : 2;
	unsigned horizontal_sync_offset_hi      : 2;

	u8       horizontal_image_size_lo;
	u8       vertical_image_size_lo;

	unsigned vertical_image_size_hi         : 4;
	unsigned horizontal_image_size_hi       : 4;

	u8       horizontal_border;
	u8       vertical_border;

	u8       flags;
};


static inline u16
edid_timing_pixel_clock(const struct edid_detailed_timing_descriptor * const dtb)
{
	return dtb->pixel_clock * 10000;
}

static inline u16
edid_timing_horizontal_blanking(const struct edid_detailed_timing_descriptor * const dtb)
{
	return (dtb->horizontal_blanking_hi << 8) | dtb->horizontal_blanking_lo;
}

static inline u16
edid_timing_horizontal_active(const struct edid_detailed_timing_descriptor * const dtb)
{
	return (dtb->horizontal_active_hi << 8) | dtb->horizontal_active_lo;
}

static inline u16
edid_timing_vertical_blanking(const struct edid_detailed_timing_descriptor * const dtb)
{
	return (dtb->vertical_blanking_hi << 8) | dtb->vertical_blanking_lo;
}

static inline u16
edid_timing_vertical_active(const struct edid_detailed_timing_descriptor * const dtb)
{
	return (dtb->vertical_active_hi << 8) | dtb->vertical_active_lo;
}

static inline u8
edid_timing_vertical_sync_offset(const struct edid_detailed_timing_descriptor * const dtb)
{
	return (dtb->vertical_sync_offset_hi << 4) | dtb->vertical_sync_offset_lo;
}

static inline u8
edid_timing_vertical_sync_pulse_width(const struct edid_detailed_timing_descriptor * const dtb)
{
	return (dtb->vertical_sync_pulse_width_hi << 4) | dtb->vertical_sync_pulse_width_lo;
}

static inline u8
edid_timing_horizontal_sync_offset(const struct edid_detailed_timing_descriptor * const dtb)
{
	return (dtb->horizontal_sync_offset_hi << 4) | dtb->horizontal_sync_offset_lo;
}

static inline u8
edid_timing_horizontal_sync_pulse_width(const struct edid_detailed_timing_descriptor * const dtb)
{
	return (dtb->horizontal_sync_pulse_width_hi << 4) | dtb->horizontal_sync_pulse_width_lo;
}

static inline u16
edid_timing_horizontal_image_size(const struct edid_detailed_timing_descriptor * const dtb)
{
	return (dtb->horizontal_image_size_hi << 8) | dtb->horizontal_image_size_lo;
}

static inline u16
edid_timing_vertical_image_size(const struct edid_detailed_timing_descriptor * const dtb)
{
	return (dtb->vertical_image_size_hi << 8) | dtb->vertical_image_size_lo;
}


struct __packed edid_monitor_descriptor {
	u16 flag0;
	u8  flag1;
	u8  tag;
	u8  flag2;
	u8  data[13];
};

struct __packed edid_standard_timing_descriptor {
	u8       horizontal_active_pixels;    /* = (value + 31) * 8 */

	unsigned refresh_rate       : 6;      /* = value + 60 */
	unsigned image_aspect_ratio : 2;
};

struct __packed edid_block0 {
	/* header information */
	u8 header[8];

	/* vendor/product identification */
	struct __packed {
		unsigned id2  : 5;
		unsigned id1  : 5;
		unsigned id0  : 5;
		unsigned zero : 1;
	} manufacturer;

	u8 product[2];
	u8 serial_number[4];
	u8 manufacture_week;
	u8 manufacture_year;

	/* EDID version */
	u8 version;
	u8 revision;

	/* basic display parameters and features */
	struct __packed {
		unsigned dfp_1x_vsync_serration : 1; /* VESA DFP 1.x */
		unsigned green_video_sync       : 1;
		unsigned composite_sync         : 1;
		unsigned separate_sync          : 1;
		unsigned blank_to_black_setup   : 1;
		unsigned signal_level_standard  : 2;
		unsigned digital                : 1;
	} video_input_definition;

	u8 maximum_horizontal_image_size;       /* cm */
	u8 maximum_vertical_image_size;         /* cm */

	u8 display_transfer_characteristics;    /* gamma = (value + 100) / 100 */

	struct __packed {
		unsigned default_gtf                    : 1; /* generalised timing formula */
		unsigned preferred_timing_mode          : 1;
		unsigned standard_default_color_space   : 1;
		unsigned display_type                   : 2;
		unsigned active_off                     : 1;
		unsigned suspend                        : 1;
		unsigned standby                        : 1;
	} feature_support;

	/* color characteristics block */
	unsigned green_y_low    : 2;
	unsigned green_x_low    : 2;
	unsigned red_y_low      : 2;
	unsigned red_x_low      : 2;

	unsigned white_y_low    : 2;
	unsigned white_x_low    : 2;
	unsigned blue_y_low     : 2;
	unsigned blue_x_low     : 2;

	u8 red_x;
	u8 red_y;
	u8 green_x;
	u8 green_y;
	u8 blue_x;
	u8 blue_y;
	u8 white_x;
	u8 white_y;

	/* established timings */
	struct __packed {
		unsigned timing_800x600_60   : 1;
		unsigned timing_800x600_56   : 1;
		unsigned timing_640x480_75   : 1;
		unsigned timing_640x480_72   : 1;
		unsigned timing_640x480_67   : 1;
		unsigned timing_640x480_60   : 1;
		unsigned timing_720x400_88   : 1;
		unsigned timing_720x400_70   : 1;

		unsigned timing_1280x1024_75 : 1;
		unsigned timing_1024x768_75  : 1;
		unsigned timing_1024x768_70  : 1;
		unsigned timing_1024x768_60  : 1;
		unsigned timing_1024x768_87  : 1;
		unsigned timing_832x624_75   : 1;
		unsigned timing_800x600_75   : 1;
		unsigned timing_800x600_72   : 1;
	} established_timings;

	struct __packed {
		unsigned reserved            : 7;
		unsigned timing_1152x870_75  : 1;
	} manufacturer_timings;

	/* standard timing id */
	struct edid_standard_timing_descriptor standard_timing_id[8];

	/* detailed timing */
	union {
		struct edid_detailed_timing_descriptor detailed_timing[4];
		struct edid_monitor_descriptor         monitor_descriptor[4];
	} detailed_timings;

	u8 extensions;
	u8 checksum;
};

struct __packed edid_color_characteristics_data {
	struct {
		u16 x;
		u16 y;
	} red, green, blue, white;
};


static inline u16 edid_gamma(const struct edid_block0 * const block0)
{
	return (block0->display_transfer_characteristics + 100) / 100;
}

static inline struct edid_color_characteristics_data
edid_color_characteristics(const struct edid_block0 * const block0)
{
	struct edid_color_characteristics_data characteristics = {
		.red = {
			.x = (block0->red_x << 8) | block0->red_x_low,
			.y = (block0->red_y << 8) | block0->red_y_low,
		},
		.green = {
			.x = (block0->green_x << 8) | block0->green_x_low,
			.y = (block0->green_y << 8) | block0->green_y_low,
		},
		.blue = {
			.x = (block0->blue_x << 8) | block0->blue_x_low,
			.y = (block0->blue_y << 8) | block0->blue_y_low,
		},
		.white = {
			.x = (block0->white_x << 8) | block0->white_x_low,
			.y = (block0->white_y << 8) | block0->white_y_low,
		},
	};

	return characteristics;
}

struct __packed edid_block_map {
	u8 tag;
	u8 extension_tag[126];
	u8 checksum;
};

struct __packed edid_extension {
	u8 tag;
	u8 revision;
	u8 extension_data[125];
	u8 checksum;
};

static inline bool edid_verify_checksum(const u8 * const block)
{
	u8 checksum = 0;
	int i;

	for (i = 0; i < EDID_BLOCK_SIZE; i++)
		checksum += block[i];

	return (checksum == 0);
}

#endif

