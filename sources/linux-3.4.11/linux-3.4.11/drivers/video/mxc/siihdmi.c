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

#include <linux/module.h>
#include <linux/fb.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/console.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/edid.h>
#include <linux/cea861.h>
#include <video/avifb.h>
#include <linux/switch.h>
#include <linux/hdmi.h>
#include "../cea861_modedb.h"
#include "siihdmi.h"


/* logging helpers */
#define CONTINUE(fmt, ...)	printk(KERN_CONT    fmt, ## __VA_ARGS__)
#define DEBUG(fmt, ...)		printk(KERN_DEBUG   "SIIHDMI: " fmt, ## __VA_ARGS__)
#define ERROR(fmt, ...)		printk(KERN_ERR     "SIIHDMI: " fmt, ## __VA_ARGS__)
#define WARNING(fmt, ...)	printk(KERN_WARNING "SIIHDMI: " fmt, ## __VA_ARGS__)
#define INFO(fmt, ...)		printk(KERN_INFO    "SIIHDMI: " fmt, ## __VA_ARGS__)

/* DDC segment for 4 block read */
#define EDID_I2C_DDC_DATA_SEGMENT 0x30

/* Default Monspecs when EDID is not available or no format is available in EDID */
static struct fb_monspecs default_monspecs = {
	.modedb = (struct fb_videomode *) &cea_modes[1],
	.modedb_len = 1,
};

/* Aspect ratio extracted from CEA-861-D: needed for AVI Infoframe */
static enum hdmi_picture_aspect cea_ratios[65] = {
	HDMI_PICTURE_ASPECT_NONE, /* No VIC: default mode */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 1: 640x480p @ 59.94Hz/60Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 2: 720x480p @ 59.94Hz/60Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 3: 720x480p @ 59.94Hz/60Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 4: 1280x720p @ 59.94Hz/60Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 5: 1920x1080i @ 59.94Hz/60Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 6: 720(1440)x480i @ 59.94Hz/60Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 7: 720(1440)x480i @ 59.94Hz/60Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 8: 720(1440)x240p @ 59.94Hz/60Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 9: 720(1440)x240p @ 59.94Hz/60Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 10: 2880x480i @ 59.94Hz/60Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 11: 2880x480i @ 59.94Hz/60Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 12: 2880x240p @ 59.94Hz/60Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 13: 2880x240p @ 59.94Hz/60Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 14: 1440x480p @ 59.94Hz/60Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 15: 1440x480p @ 59.94Hz/60Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 16: 1920x1080p @ 59.94Hz/60Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 17: 720x576p @ 50Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 18: 720x576p @ 50Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 19: 1280x720p @ 50Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 20: 1920x1080i @ 50Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 21: 720(1440)x576i @ 50Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 22: 720(1440)x576i @ 50Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 23: 720(1440)x288p @ 50Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 24: 720(1440)x288p @ 50Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 25: 2880x576i @ 50Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 26: 2880x576i @ 50Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 27: 2880x288p @ 50Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 28: 2880x288p @ 50Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 29: 1440x576p @ 50Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 30: 1440x576p @ 50Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 31: 1920x1080p @ 50Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 32: 1920x1080p @ 23.97Hz/24Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 33: 1920x1080p @ 25Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 34: 1920x1080p @ 29.97Hz/30Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 35: 2880x480p @ 59.94Hz/60Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 36: 2880x480p @ 59.94Hz/60Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 37: 2880x576p @ 50Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 38: 2880x576p @ 50Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 39: 1920x1080i (1250 total) @ 50Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 40: 1920x1080i @ 100Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 41: 1280x720p @ 100Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 42: 720x576p @ 100Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 43: 720x576p @ 100Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 44: 720(1440)x576i @ 100Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 45: 720(1440)x576i @ 100Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 46: 1920x1080i @ 119.88/120Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 47: 1280x720p @ 119.88/120Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 48: 720x480p @ 119.88/120Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 49: 720x480p @ 119.88/120Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 50: 720(1440)x480i @ 119.88/120Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 51: 720(1440)x480i @ 119.88/120Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 52: 720x576p @ 200Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 53: 720x576p @ 200Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 54: 720(1440)x576i @ 200Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 55: 720(1440)x576i @ 200Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 56: 720x480p @ 239.76/240Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 57: 720x480p @ 239.76/240Hz */
	HDMI_PICTURE_ASPECT_4_3,  /* VIC 58: 720(1440)x480i @ 239.76/240Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 59: 720(1440)x480i @ 239.76/240Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 60: 1280x720p @ 24Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 61: 1280x720p @ 25Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 62: 1280x720p @ 30Hz */
	HDMI_PICTURE_ASPECT_16_9, /* VIC 63: 1920x1080p @ 120Hz */
	HDMI_PICTURE_ASPECT_16_9  /* VIC 64: 1920x1080p @ 100Hz */
};

/* module parameters */
static unsigned int bus_timeout = 50;
module_param(bus_timeout, uint, 0644);
MODULE_PARM_DESC(bus_timeout, "bus timeout in milliseconds");

static unsigned int seventwenty	= 1;
module_param(seventwenty, uint, 0644);
MODULE_PARM_DESC(seventwenty, "attempt to use 720p mode");

static unsigned int teneighty    = 0;
module_param(teneighty, uint, 0644);
MODULE_PARM_DESC(teneighty, "attempt to use 1080p mode");

static unsigned int useitmodes	= 1;
module_param(useitmodes, uint, 0644);
MODULE_PARM_DESC(useitmodes, "prefer IT modes over CEA modes when sanitizing the modelist");

static unsigned int modevic = 0;
module_param_named(vic, modevic, uint, 0644);
MODULE_PARM_DESC(modevic, "CEA VIC to try and match before autodetection");

static int siihdmi_detect_revision(struct siihdmi_tx *tx)
{
	u8 data;
	unsigned long start;

	start = jiffies;
	do {
		data = i2c_smbus_read_byte_data(tx->client,
						SIIHDMI_TPI_REG_DEVICE_ID);
	} while (data != SIIHDMI_DEVICE_ID_902x &&
		 !time_after(jiffies, start + bus_timeout));

	if (data != SIIHDMI_DEVICE_ID_902x)
		return -ENODEV;

	INFO("Device ID: %#02x", data);

	data = i2c_smbus_read_byte_data(tx->client,
					SIIHDMI_TPI_REG_DEVICE_REVISION);
	if (data)
		CONTINUE(" (rev %01u.%01u)",
			 (data >> 4) & 0xf, (data >> 0) & 0xf);

	data = i2c_smbus_read_byte_data(tx->client,
					SIIHDMI_TPI_REG_TPI_REVISION);
	CONTINUE(" (%s",
		 (data & SIIHDMI_VERSION_FLAG_VIRTUAL) ? "Virtual " : "");
	data &= ~SIIHDMI_VERSION_FLAG_VIRTUAL;
	data = data ? data : SIIHDMI_BASE_TPI_REVISION;
	CONTINUE("TPI revision %01u.%01u)",
		 (data >> 4) & 0xf, (data >> 0) & 0xf);

	data = i2c_smbus_read_byte_data(tx->client,
					SIIHDMI_TPI_REG_HDCP_REVISION);
	if (data)
		CONTINUE(" (HDCP version %01u.%01u)",
			 (data >> 4) & 0xf, (data >> 0) & 0xf);

	CONTINUE("\n");

	return 0;
}

static inline int siihdmi_power_up(struct siihdmi_tx *tx)
{
	int ret;

	DEBUG("Powering up transmitter\n");

	ret = i2c_smbus_write_byte_data(tx->client,
					SIIHDMI_TPI_REG_PWR_STATE,
					SIIHDMI_POWER_STATE_D0);
	if (ret < 0)
		ERROR("unable to power up transmitter\n");

	return ret;
}

static inline int siihdmi_power_down(struct siihdmi_tx *tx)
{
	int ret;
	u8 ctrl;

	DEBUG("Powering down transmitter\n");

#ifdef SIIHDMI_USE_FB
	memset((void *) &tx->sink.current_mode, 0, sizeof(struct fb_videomode));
#endif

	ctrl = SIIHDMI_SYS_CTRL_TMDS_OUTPUT_POWER_DOWN;
	if (tx->sink.type == SINK_TYPE_HDMI)
		ctrl |= SIIHDMI_SYS_CTRL_OUTPUT_MODE_SELECT_HDMI;
	ret = i2c_smbus_write_byte_data(tx->client,
					SIIHDMI_TPI_REG_SYS_CTRL, ctrl);
	if (ret < 0) {
		ERROR("unable to power down transmitter\n");
		return ret;
	}

	ret = i2c_smbus_write_byte_data(tx->client,
					SIIHDMI_TPI_REG_PWR_STATE,
					SIIHDMI_POWER_STATE_D2);
	if (ret < 0) {
		ERROR("unable to set transmitter into D2\n");
		return ret;
	}

	return 0;
}

static int siihdmi_check_sync_mux_mode(struct siihdmi_tx *tx)
{
	int ret;
	u8 value;

	/* Configure only when mux */
	if (tx->platform->input_format != YUV_422_8_MUX)
		return 0;

	/* Check sync mux mode activated */
	value = i2c_smbus_read_byte_data(tx->client,
					    0x60);
	if ((value & 1 << 5) && (value & 1 << 7))
		return 0;

	/* Restore explicitly, if necessary, TPI 0x60[5] to enable YC Mux mode */
	value |= 1 << 5; /* YC mux mode one-to-two data channel demux enable */
	value |= 1 << 7; /* External sync method */
	ret = i2c_smbus_write_byte_data(tx->client,
					0x60,
					value);
	if (ret < 0) {
		WARNING("failed to configure syn mux mode: %d\n", ret);
		return ret;
	}
	return 0;
}

static int siihdmi_check_embsync_extract(struct siihdmi_tx *tx)
{
	int ret;
	u8 value;

	/* Configure only when mux */
	if (tx->platform->input_format != YUV_422_8_MUX)
		return 0;

	siihdmi_check_sync_mux_mode(tx);

	/* Check embedded sync extraction */
	value = i2c_smbus_read_byte_data(tx->client,
					    0x63);
	if (value & 1 << 6)
		return 0;

	value |= 1 << 6;
	ret = i2c_smbus_write_byte_data(tx->client,
					0x63,
					value);
	if (ret < 0) {
		WARNING("failed to activate de generator: %d\n", ret);
		return ret;
	}

	/* Check again .. */
	siihdmi_check_sync_mux_mode(tx);
	return 0;
}

static int siihdmi_initialise(struct siihdmi_tx *tx)
{
	int ret;
	u8 value;

	ret = i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_RQB, 0x00);
	if (ret < 0) {
		WARNING("unable to initialise device to TPI mode\n");
		return ret;
	}

	/* step 2: detect revision */
	if ((ret = siihdmi_detect_revision(tx)) < 0) {
		DEBUG("unable to detect device revision\n");
		return ret;
	}

	/* step 3: power up transmitter */
	if ((ret = siihdmi_power_up(tx)) < 0)
		return ret;

	/* step 3.5: enable source termination
	 * (recommanded for pixclock > 100 MHz)
	 *  1. Set to internal page 0 (0xBC = 0x01)
	 *  2. Set to indexed register 130 (0xBD = 0x82)
	 *  3. Read value of register (0xBE)
	 *  4. Set bit 0 to 1 in order to enable source termination
	 *  5. Write value to register (0xBE)
	 */
	ret = i2c_smbus_write_byte_data(tx->client, 0xBC, 0x01);
	if (ret < 0)
		return ret;
	ret = i2c_smbus_write_byte_data(tx->client, 0xBD, 0x82);
	if (ret < 0)
		return ret;
	value = i2c_smbus_read_byte_data(tx->client, 0xBE);
	value |= 0x01;
	ret = i2c_smbus_write_byte_data(tx->client, 0xBE, value);
	if (ret < 0)
		return ret;

	/* step 4: configure input bus and pixel repetition */
	if (tx->platform->input_format == YUV_422_8_MUX) {
		ret = i2c_smbus_write_byte_data(tx->client,
				     SIIHDMI_TPI_REG_INPUT_BUS_PIXEL_REPETITION,
				     0x30);
		if (ret < 0)
			return ret;
	}

	/* step 5: select YC input mode */

	/* step 6: configure sync methods */

	/* step 7: configure explicit sync DE generation */

	/* step 8: configure embedded sync extraction */

	/* step 8.5: power down trasnmitter since interrupt power up transmitter
	 *           if a display is attached
	 */
	value = i2c_smbus_read_byte_data(tx->client, SIIHDMI_TPI_REG_ISR);
	if (!(value & SIIHDMI_ISR_DISPLAY_ATTACHED))
		siihdmi_power_down(tx);

	/* step 9: setup interrupt service */
	if (tx->hotplug.enabled) {
		ret = i2c_smbus_write_byte_data(tx->client,
						SIIHDMI_TPI_REG_IER,
						SIIHDMI_IER_HOT_PLUG_EVENT);
		if (ret < 0)
			WARNING("unable to setup interrupt request\n");
	}

	return ret;
}

static int siihdmi_read_edid(struct siihdmi_tx *tx, u8 *edid, unsigned int block, size_t size)
{
	u8 offset = block * EDID_BLOCK_SIZE;
	u8 segment = block >> 1;
	u8 xfers = segment ? 3 : 2;
	u8 ctrl;
	int ret;
	unsigned long start;

	struct i2c_msg request[] = {
		{ .addr  = EDID_I2C_DDC_DATA_SEGMENT,
		  .flags = 0,
		  .len   = 1,
		  .buf   = &segment, },
		{ .addr  = EDID_I2C_DDC_DATA_ADDRESS,
		  .flags = 0,
		  .len   = 1,
		  .buf   = &offset, },
		{ .addr  = EDID_I2C_DDC_DATA_ADDRESS,
		  .flags = I2C_M_RD,
		  .len   = size,
		  .buf   = edid, },
	};

	/* step 1: (potentially) disable HDCP */

	/* step 2: request the DDC bus */
	ctrl = i2c_smbus_read_byte_data(tx->client, SIIHDMI_TPI_REG_SYS_CTRL);
	ret = i2c_smbus_write_byte_data(tx->client,
					SIIHDMI_TPI_REG_SYS_CTRL,
					ctrl | SIIHDMI_SYS_CTRL_DDC_BUS_REQUEST);
	if (ret < 0) {
		DEBUG("unable to request DDC bus\n");
		return ret;
	}

	/* step 3: poll for bus grant */
	start = jiffies;
	do {
		ctrl = i2c_smbus_read_byte_data(tx->client,
						SIIHDMI_TPI_REG_SYS_CTRL);
	} while ((~ctrl & SIIHDMI_SYS_CTRL_DDC_BUS_GRANTED) &&
		 !time_after(jiffies, start + bus_timeout));

	if (~ctrl & SIIHDMI_SYS_CTRL_DDC_BUS_GRANTED)
		goto relinquish;

	/* step 4: take ownership of the DDC bus */
	ret = i2c_smbus_write_byte_data(tx->client,
					SIIHDMI_TPI_REG_SYS_CTRL,
					SIIHDMI_SYS_CTRL_DDC_BUS_REQUEST |
					SIIHDMI_SYS_CTRL_DDC_BUS_OWNER_HOST);
	if (ret < 0) {
		DEBUG("unable to take ownership of the DDC bus\n");
		goto relinquish;
	}

	/* step 5: read edid */
	ret = i2c_transfer(tx->client->adapter, &request[3 - xfers], xfers);
	if (ret != xfers)
		DEBUG("unable to read EDID block\n");

relinquish:
	/* step 6: relinquish ownership of the DDC bus */
	start = jiffies;
	do {
		i2c_smbus_write_byte_data(tx->client,
					  SIIHDMI_TPI_REG_SYS_CTRL,
					  0x00);
		ctrl = i2c_smbus_read_byte_data(tx->client,
						SIIHDMI_TPI_REG_SYS_CTRL);
	} while ((ctrl & SIIHDMI_SYS_CTRL_DDC_BUS_GRANTED) &&
		 !time_after(jiffies, start + bus_timeout));

	/* step 7: (potentially) enable HDCP */

	return ret;
}

static inline void _process_cea861_vsdb(struct siihdmi_tx *tx,
					const struct hdmi_vsdb * const vsdb)
{
	unsigned int max_tmds;

	if (memcmp(vsdb->ieee_registration, CEA861_OUI_REGISTRATION_ID_HDMI_LSB,
		   sizeof(vsdb->ieee_registration)))
		return;

	max_tmds = KHZ2PICOS(vsdb->max_tmds_clock * 200);

	/* Sink type is HDMI when VSDB is available in one of extensions */
	tx->sink.type = SINK_TYPE_HDMI;

	DEBUG("HDMI VSDB detected (basic audio %ssupported)\n",
	      tx->audio.available ? "" : "not ");
	INFO("HDMI port configuration: %u.%u.%u.%u\n",
	     vsdb->port_configuration_a, vsdb->port_configuration_b,
	     vsdb->port_configuration_c, vsdb->port_configuration_d);

	if (max_tmds && max_tmds < tx->platform->pixclock) {
		INFO("maximum TMDS clock limited to %u by device\n", max_tmds);
		tx->platform->pixclock = max_tmds;
	}
}

static inline void _process_cea861_video(struct siihdmi_tx *tx,
					 const struct cea861_video_data_block * const video)
{
	const struct cea861_data_block_header * const header =
		(struct cea861_data_block_header *) video;
	u8 i, count;

	for (i = 0, count = 0; i < header->length; i++) {
		const int vic = video->svd[i] & ~CEA861_SVD_NATIVE_FLAG;

		if (vic && vic <= ARRAY_SIZE(cea_modes)) {
#ifdef SIIHDMI_USE_FB
			fb_add_videomode(&cea_modes[vic], &tx->info->modelist);
#endif
			count++;
		}
	}

	DEBUG("%u modes parsed from CEA video data block\n", count);
}

static inline void _process_cea861_extended(struct siihdmi_tx *tx,
					    const struct cea861_data_block_extended *ext)
{
	static const char * const scannings[] = {
		[SCAN_INFORMATION_UNKNOWN]      = "unknown",
		[SCAN_INFORMATION_OVERSCANNED]  = "overscanned",
		[SCAN_INFORMATION_UNDERSCANNED] = "underscanned",
		[SCAN_INFORMATION_RESERVED]     = "reserved",
	};

	switch (ext->extension_tag) {
	case CEA861_DATA_BLOCK_EXTENSION_VIDEO_CAPABILITY: {
		const struct cea861_video_capability_block * const vcb =
			(struct cea861_video_capability_block *) ext;
		INFO("CEA video capability (scanning behaviour):\n"
		     "    Preferred Mode: %s\n"
		     "    VESA/PC Mode: %s\n"
		     "    CEA/TV Mode: %s\n",
		     scannings[vcb->pt_overunder_behavior],
		     scannings[vcb->it_overunder_behavior],
		     scannings[vcb->ce_overunder_behavior]);
		}
		break;
	default:
		break;
	}
}

static void siihdmi_parse_cea861_timing_block(struct siihdmi_tx *tx,
					      const struct edid_extension *ext)
{
	const struct cea861_timing_block * const cea = (struct cea861_timing_block *) ext;
	const u8 size = cea->dtd_offset - offsetof(struct cea861_timing_block, data);
	u8 index;

	BUILD_BUG_ON(sizeof(*cea) != sizeof(*ext));

	tx->audio.available = cea->basic_audio_supported;
	if (cea->underscan_supported)
		tx->sink.scanning = SCANNING_UNDERSCANNED;

	if (cea->dtd_offset == CEA861_NO_DTDS_PRESENT)
		return;

	index = 0;
	while (index < size) {
		const struct cea861_data_block_header * const header =
			(struct cea861_data_block_header *) &cea->data[index];

		switch (header->tag) {
		case CEA861_DATA_BLOCK_TYPE_VENDOR_SPECIFIC:
			_process_cea861_vsdb(tx, (struct hdmi_vsdb *) header);
			break;
#ifdef SIIHDMI_USE_FB
		case CEA861_DATA_BLOCK_TYPE_VIDEO:
			_process_cea861_video(tx, (struct cea861_video_data_block *) header);
			break;
#endif
		case CEA861_DATA_BLOCK_TYPE_EXTENDED:
			_process_cea861_extended(tx, (struct cea861_data_block_extended *) header);
			break;
		}

		index = index + header->length + sizeof(*header);
	}
}

static void siihdmi_set_vmode_registers(struct siihdmi_tx *tx,
					const struct fb_videomode *mode)
{
	enum basic_video_mode_fields {
		PIXEL_CLOCK,
		REFRESH_RATE,
		X_RESOLUTION,
		Y_RESOLUTION,
		FIELDS,
	};

	u16 vmode[FIELDS];
	u32 pixclk, htotal, vtotal, refresh;
	u8 format;
	int ret;
	u16 hsync_len, vsync_len;

	BUILD_BUG_ON(sizeof(vmode) != 8);

	BUG_ON(mode->pixclock == 0);
	pixclk = PICOS2KHZ(mode->pixclock);

	htotal = mode->xres + mode->left_margin + mode->hsync_len + mode->right_margin;
	vtotal = mode->yres + mode->upper_margin + mode->vsync_len + mode->lower_margin;

	/* explicitly use 64-bit division to avoid overflow truncation */
	refresh = (u32) div_u64(pixclk * 100000ull, htotal * vtotal);

	/* basic video mode data */
	vmode[PIXEL_CLOCK]  = (u16) (pixclk / 10);
	if (tx->platform->input_format == YUV_422_8_MUX)
		vmode[PIXEL_CLOCK] *= 2;

	/*
	  Silicon Image example code implies refresh to be 6000 for 60Hz?
	  This may work simply because we only test it on little-endian :(
	*/
	vmode[REFRESH_RATE] = (u16) refresh;
	vmode[X_RESOLUTION] = (u16) htotal;
	vmode[Y_RESOLUTION] = (u16) vtotal;

	ret = i2c_smbus_write_i2c_block_data(tx->client,
					     SIIHDMI_TPI_REG_VIDEO_MODE_DATA_BASE,
					     sizeof(vmode),
					     (u8 *) vmode);
	if (ret < 0)
		DEBUG("unable to write video mode data\n");

	/* input format */
	format = SIIHDMI_INPUT_VIDEO_RANGE_EXPANSION_AUTO
		| SIIHDMI_INPUT_COLOR_DEPTH_8BIT;

	switch (tx->platform->input_format) {
	case YUV_422_8_MUX:
		format |= SIIHDMI_INPUT_COLOR_SPACE_YUV_422;
		break;
	case RGB_24:
	default :
		format |= SIIHDMI_INPUT_COLOR_SPACE_RGB;
		break;
	}

	ret = i2c_smbus_write_byte_data(tx->client,
					SIIHDMI_TPI_REG_AVI_INPUT_FORMAT,
					format);
	if (ret < 0)
		DEBUG("unable to set input format\n");

	/* output format */
	format = SIIHDMI_OUTPUT_VIDEO_RANGE_COMPRESSION_AUTO
	       | SIIHDMI_OUTPUT_COLOR_STANDARD_BT601
	       | SIIHDMI_OUTPUT_COLOR_DEPTH_8BIT;

	if (tx->sink.type == SINK_TYPE_HDMI)
		format |= SIIHDMI_OUTPUT_FORMAT_HDMI_RGB;
	else
		format |= SIIHDMI_OUTPUT_FORMAT_DVI_RGB;

	ret = i2c_smbus_write_byte_data(tx->client,
					SIIHDMI_TPI_REG_AVI_OUTPUT_FORMAT,
					format);
	if (ret < 0)
		DEBUG("unable to set output format\n");

	/* If format is not muxed, terminated */
	if (tx->platform->input_format != YUV_422_8_MUX)
		return;

	/* Default values from hdmi_1024x768p60_video_mode */
	hsync_len = mode->hsync_len;
	vsync_len = mode->vsync_len;

	/* Embedded sync register set */
	siihdmi_check_sync_mux_mode(tx);
	i2c_smbus_write_byte_data(tx->client, 0x62, 0x01);
	i2c_smbus_write_byte_data(tx->client, 0x64, 0x00);
	i2c_smbus_write_byte_data(tx->client, 0x65, 0x02);
	i2c_smbus_write_byte_data(tx->client, 0x66, hsync_len & 0xFF);
	i2c_smbus_write_byte_data(tx->client, 0x67, (hsync_len >> 8) & 0x03);
	i2c_smbus_write_byte_data(tx->client, 0x68, 0x01);
	i2c_smbus_write_byte_data(tx->client, 0x69, vsync_len & 0x3F);
	siihdmi_check_sync_mux_mode(tx);
}

static int siihdmi_clear_avi_info_frame(struct siihdmi_tx *tx)
{
	const u8 buffer[SIIHDMI_TPI_REG_AVI_INFO_FRAME_LENGTH] = {0};
	int ret;

	BUG_ON(tx->sink.type != SINK_TYPE_DVI);

	ret = i2c_smbus_write_i2c_block_data(tx->client,
					     SIIHDMI_TPI_REG_AVI_INFO_FRAME_BASE,
					     sizeof(buffer), buffer);
	if (ret < 0)
		DEBUG("unable to clear avi info frame\n");

	return ret;
}

static int siihdmi_set_avi_info_frame(struct siihdmi_tx *tx)
{
	int ret = 0, i;
	u8 buffer[HDMI_INFOFRAME_HEADER_SIZE + HDMI_AVI_INFOFRAME_SIZE];
	int min_refresh, max_refresh;
	struct hdmi_avi_infoframe infoframe;
	struct edid_block0 *block0;
	struct fb_videomode *m;
	unsigned ratio;
	enum {
		ASPECT_RATIO_4_3 = 1,
		ASPECT_RATIO_16_9 = 2
	} modes = 0;

	hdmi_avi_infoframe_init(&infoframe);

	infoframe.colorspace 	= HDMI_COLORSPACE_RGB;
	infoframe.active_aspect = HDMI_ACTIVE_ASPECT_PICTURE;

	/* Fill Video Code and Aspect ratio */
	if (tx->selected_mode != NULL) {
		/* Vertical refresh is +/- 0.5% in CEA spec */
		min_refresh = tx->selected_mode->refresh * 995 / 1000;
		max_refresh = tx->selected_mode->refresh * 1005 / 1000;

		/* Find correct VIC in CEA
		 * Interlaced video are not supported
		 * Vertical refesh is +/-0.5%
		 * The name is NULL or equal to name in cea_modes (some format
		 * have same timings but not same VIC and aspect ratio)
		 */
		for (i = 1; i <= ARRAY_SIZE(cea_modes); i++) {
			m = (struct fb_videomode *) &cea_modes[i];
			if (!(m->vmode & FB_VMODE_INTERLACED) &&
			    tx->selected_mode->xres == m->xres &&
			    tx->selected_mode->yres == m->yres &&
			    min_refresh <= m->refresh &&
			    max_refresh >= m->refresh &&
			    tx->selected_mode->hsync_len == m->hsync_len &&
			    (tx->selected_mode->name == NULL ||
			     tx->selected_mode->name == m->name)) {
				infoframe.video_code = i;
				break;
		    }
		}

		/* Get aspect ratio from CEA ratio table */
		if (infoframe.video_code > 1 && infoframe.video_code < 64 &&
		    (fb_mode_is_equal(&cea_modes[infoframe.video_code],
				      &cea_modes[infoframe.video_code+1]) ||
		     fb_mode_is_equal(&cea_modes[infoframe.video_code],
				      &cea_modes[infoframe.video_code-1]))) {
			/* Video format has two aspect ratio in CEA table
			 * Get correct VIC from E-EDID (supported CEA modes)
			 * If none or both aspect ratio are available, ratio of
			 * screen is calculated and correct VIC is selected.
			 */
			if (cea_ratios[infoframe.video_code] ==
						       HDMI_PICTURE_ASPECT_16_9)
				infoframe.video_code--;

			/* Find in E-EDID which VIC are available */
			for (i = 0; i < tx->monspecs.modedb_len; i++) {
				if (tx->monspecs.modedb[i].name ==
					   cea_modes[infoframe.video_code].name)
					modes |= ASPECT_RATIO_4_3;
				else if (tx->monspecs.modedb[i].name ==
					 cea_modes[infoframe.video_code+1].name)
					modes |= ASPECT_RATIO_16_9;
			}

			/* Calculate ratio from EDID */
			if ((modes == 0 ||
			     modes == (ASPECT_RATIO_4_3 | ASPECT_RATIO_16_9)) &&
			    tx->edid.data != NULL) {
				block0 = (struct edid_block0 *) tx->edid.data;
				ratio = block0->maximum_horizontal_image_size *
				      100 / block0->maximum_vertical_image_size;
				if (ratio >= 177)
					infoframe.video_code++;
			}
			else if (modes == ASPECT_RATIO_16_9)
				infoframe.video_code++;

			/* Select an active aspect ratio:
			 *  The CEA modes with two aspect ratio (4:3 and 16:9)
			 *  for the same resolution as 480p or 576p, imply that
			 *  video must be streched to 4:3 when aspect ratio is
			 *  16:9 since the screen will rescale the image to fit
			 *  16:9. To bypass the streching process, an active
			 *  aspect ratio can be specified for both formats in
			 *  order to pass HDMI certification: if the active
			 *  aspect ratio field is not "As the coded frame"
			 *  (= 8), the aspect ratio of displayed picture is not
			 *  verified and the 16:9 case doesn't make issue.
			 */
			if (cea_ratios[infoframe.video_code] ==
							HDMI_PICTURE_ASPECT_4_3)
				infoframe.active_aspect =
							HDMI_ACTIVE_ASPECT_16_9;
			else
				infoframe.active_aspect =
						 HDMI_ACTIVE_ASPECT_16_9_CENTER;
		}
		infoframe.picture_aspect = cea_ratios[infoframe.video_code];

		/* Set pixel repetition */
		if (tx->selected_mode->vmode & FB_VMODE_DOUBLE)
			infoframe.pixel_repeat = 1;

		/* Log VIC format used */
		if (infoframe.video_code > 0)
			INFO("Use VIC %d\n", infoframe.video_code);
	}

	switch (tx->sink.scanning) {
	case SCANNING_UNDERSCANNED:
		infoframe.scan_mode = HDMI_SCAN_MODE_UNDERSCAN;
		break;
	case SCANNING_OVERSCANNED:
		infoframe.scan_mode = HDMI_SCAN_MODE_OVERSCAN;
		break;
	default:
		infoframe.scan_mode = HDMI_SCAN_MODE_NONE;
	}

	hdmi_avi_infoframe_pack(&infoframe,buffer,sizeof(buffer));

	ret = i2c_smbus_write_i2c_block_data(tx->client,
					     SIIHDMI_TPI_REG_AVI_INFO_FRAME_BASE,
					     sizeof(buffer) - SIIHDMI_AVI_INFO_FRAME_OFFSET,
					     (buffer) + SIIHDMI_AVI_INFO_FRAME_OFFSET);

	if (ret < 0)
		DEBUG("unable to write avi info frame\n");

	return ret;
}

static int siihdmi_set_audio_info_frame(struct siihdmi_tx *tx)
{
	int ret;
	struct siihdmi_audio_info_frame packet = {
		.header = {
			.info_frame = SIIHDMI_INFO_FRAME_AUDIO,
			.repeat     = false,
			.enable     = tx->audio.available,
		},

		.info_frame = {
			.header = {
				.type    = INFO_FRAME_TYPE_AUDIO,
				.version = CEA861_AUDIO_INFO_FRAME_VERSION,
				.length  = sizeof(packet.info_frame) - sizeof(packet.info_frame.header),
			},

			.channel_count         = CHANNEL_COUNT_REFER_STREAM_HEADER,
			.channel_allocation    = CHANNEL_ALLOCATION_STEREO,

			.format_code_extension = CODING_TYPE_REFER_STREAM_HEADER,

			/* required to Refer to Stream Header by CEA861-D */
			.coding_type           = CODING_TYPE_REFER_STREAM_HEADER,

			.sample_size           = SAMPLE_SIZE_REFER_STREAM_HEADER,
			.sample_frequency      = FREQUENCY_REFER_STREAM_HEADER,
		},
	};

	BUG_ON(tx->sink.type != SINK_TYPE_HDMI);

	cea861_checksum_hdmi_info_frame((u8 *) &packet.info_frame);

	BUILD_BUG_ON(sizeof(packet) != SIIHDMI_TPI_REG_AUDIO_INFO_FRAME_LENGTH);
	ret = i2c_smbus_write_i2c_block_data(tx->client,
					     SIIHDMI_TPI_REG_MISC_INFO_FRAME_BASE,
					     sizeof(packet),
					     (u8 *) &packet);
	if (ret < 0)
		DEBUG("unable to write audio info frame\n");

	return ret;
}

static int siihdmi_set_spd_info_frame(struct siihdmi_tx *tx)
{
	int ret;
	struct siihdmi_spd_info_frame packet = {
		.header = {
			.info_frame = SIIHDMI_INFO_FRAME_SPD_ACP,
			.repeat     = false,
			.enable     = true,
		},

		.info_frame = {
			.header = {
				.type    = INFO_FRAME_TYPE_SOURCE_PRODUCT_DESCRIPTION,
				.version = CEA861_SPD_INFO_FRAME_VERSION,
				.length  = sizeof(packet.info_frame) - sizeof(packet.info_frame.header),
			},

			.source_device_info = SPD_SOURCE_PC_GENERAL,
		},
	};

	BUG_ON(tx->sink.type != SINK_TYPE_HDMI);

	strncpy(packet.info_frame.vendor, tx->platform->vendor,
		sizeof(packet.info_frame.vendor));

	strncpy(packet.info_frame.description, tx->platform->description,
		sizeof(packet.info_frame.description));

	cea861_checksum_hdmi_info_frame((u8 *) &packet.info_frame);

	BUILD_BUG_ON(sizeof(packet) != SIIHDMI_TPI_REG_MISC_INFO_FRAME_LENGTH);
	ret = i2c_smbus_write_i2c_block_data(tx->client,
					     SIIHDMI_TPI_REG_MISC_INFO_FRAME_BASE,
					     sizeof(packet),
					     (u8 *) &packet);
	if (ret < 0)
		DEBUG("unable to write SPD info frame\n");

	return ret;
}

static inline void siihdmi_audio_mute(struct siihdmi_tx *tx)
{
	u8 data;

	data = i2c_smbus_read_byte_data(tx->client,
					SIIHDMI_TPI_REG_I2S_AUDIO_PACKET_LAYOUT_CTRL);

	i2c_smbus_write_byte_data(tx->client,
				  SIIHDMI_TPI_REG_I2S_AUDIO_PACKET_LAYOUT_CTRL,
				  data | SIIHDMI_AUDIO_MUTE);
}

static inline void siihdmi_audio_unmute(struct siihdmi_tx *tx)
{
	u8 data;

	data = i2c_smbus_read_byte_data(tx->client,
					SIIHDMI_TPI_REG_I2S_AUDIO_PACKET_LAYOUT_CTRL);

	i2c_smbus_write_byte_data(tx->client,
				  SIIHDMI_TPI_REG_I2S_AUDIO_PACKET_LAYOUT_CTRL,
				  data & ~SIIHDMI_AUDIO_MUTE);
}

static inline void siihdmi_configure_audio(struct siihdmi_tx *tx)
{
	siihdmi_audio_mute(tx);

	i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_I2S_AUDIO_PACKET_LAYOUT_CTRL, SIIHDMI_AUDIO_SPDIF_ENABLE | SIIHDMI_AUDIO_MUTE);
	i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_I2S_AUDIO_SAMPLING_HBR, 0);
	i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_I2S_ORIGINAL_FREQ_SAMPLE_LENGTH, SIIHDMI_AUDIO_HANDLING_DOWN_SAMPLE);

	siihdmi_audio_unmute(tx);
	siihdmi_set_audio_info_frame(tx);
}

static void siihdmi_print_modeline(const struct siihdmi_tx *tx,
				   const struct fb_videomode *mode,
				   const char * const message)
{
	const bool interlaced = (mode->vmode & FB_VMODE_INTERLACED);
	const bool double_scan = (mode->vmode & FB_VMODE_DOUBLE);

	u32 pixclk = mode->pixclock ? PICOS2KHZ(mode->pixclock) : 0;
	char flag = ' ';

	pixclk >>= (double_scan ? 1 : 0);

#ifdef SIIHDMI_USE_FB
	if (fb_mode_is_equal(&tx->sink.preferred_mode, mode))
		flag = '*';
#endif

#ifdef SIIHDMI_USE_FB
	if (mode->flag & FB_MODE_IS_CEA)
		flag = 'C';
#endif
	INFO("  %c \"%dx%d@%d%s\" %lu.%.2lu %u %u %u %u %u %u %u %u %chsync %cvsync",
	     /* CEA or preferred status of modeline */
	     flag,

	     /* mode name */
	     mode->xres, mode->yres,
	     mode->refresh << (interlaced ? 1 : 0),
	     interlaced ? "i" : (double_scan ? "d" : ""),

	     /* dot clock frequency (MHz) */
	     pixclk / 1000ul,
	     pixclk % 1000ul,

	     /* horizontal timings */
	     mode->xres,
	     mode->xres + mode->right_margin,
	     mode->xres + mode->right_margin + mode->hsync_len,
	     mode->xres + mode->right_margin + mode->hsync_len + mode->left_margin,

	     /* vertical timings */
	     mode->yres,
	     mode->yres + mode->lower_margin,
	     mode->yres + mode->lower_margin + mode->vsync_len,
	     mode->yres + mode->lower_margin + mode->vsync_len + mode->upper_margin,

	     /* sync direction */
	     (mode->sync & FB_SYNC_HOR_HIGH_ACT) ? '+' : '-',
	     (mode->sync & FB_SYNC_VERT_HIGH_ACT) ? '+' : '-');

	if (message)
		CONTINUE(" (%s)", message);

	CONTINUE("\n");
}

static int siihdmi_set_resolution(struct siihdmi_tx *tx,
				  const struct fb_videomode *mode)
{
	u8 ctrl;
	int ret;

#ifdef SIIHDMI_USE_FB
	/* don't care if config differs from FB */
	if (0 == memcmp((void *) &tx->sink.current_mode, (void *) mode, sizeof(struct fb_videomode)))
	{
		return 0;
	}
#endif

	INFO("selected configuration: \n");
	siihdmi_print_modeline(tx, mode, NULL);

	ctrl = i2c_smbus_read_byte_data(tx->client, SIIHDMI_TPI_REG_SYS_CTRL);

	/* setup the sink type */
	if (tx->sink.type == SINK_TYPE_DVI)
		ctrl &= ~SIIHDMI_SYS_CTRL_OUTPUT_MODE_SELECT_HDMI;
	else
		ctrl |= SIIHDMI_SYS_CTRL_OUTPUT_MODE_SELECT_HDMI;

	/* step 1: (potentially) disable HDCP */

	/* step 2: (optionally) blank the display */
	/*
	 * Note that if we set the AV Mute, switching to DVI could result in a
	 * permanently muted display until a hardware reset.  Thus only do this
	 * if the sink is a HDMI connection
	 */
	if (tx->sink.type == SINK_TYPE_HDMI)
		ctrl |= SIIHDMI_SYS_CTRL_AV_MUTE_HDMI;
	/* optimisation: merge the write into the next one */

	/* step 3: prepare for resolution change */
	ctrl |= SIIHDMI_SYS_CTRL_TMDS_OUTPUT_POWER_DOWN;
	ret = i2c_smbus_write_byte_data(tx->client,
					SIIHDMI_TPI_REG_SYS_CTRL,
					ctrl);
	if (ret < 0)
		DEBUG("unable to prepare for resolution change\n");

	msleep(SIIHDMI_CTRL_INFO_FRAME_DRAIN_TIME);

	/* step 4: change video resolution */

	/* step 5: set the vmode registers */
	siihdmi_set_vmode_registers(tx, mode);
	siihdmi_check_embsync_extract(tx);

	/*
	 * step 6:
	 *      [DVI]  clear AVI InfoFrame
	 *      [HDMI] set AVI InfoFrame
	 */
	if (tx->sink.type == SINK_TYPE_HDMI)
		siihdmi_set_avi_info_frame(tx);
	else
		siihdmi_clear_avi_info_frame(tx);
	siihdmi_check_embsync_extract(tx);

	/* step 7: [HDMI] set new audio information */
	if (tx->sink.type == SINK_TYPE_HDMI) {
		if (tx->audio.available)
			siihdmi_configure_audio(tx);
		siihdmi_set_spd_info_frame(tx);
	}

	/* step 8: enable display */
	ctrl &= ~SIIHDMI_SYS_CTRL_TMDS_OUTPUT_POWER_DOWN;
	/* optimisation: merge the write into the next one */

	/* step 9: (optionally) un-blank the display */
	ctrl &= ~SIIHDMI_SYS_CTRL_AV_MUTE_HDMI;
	ret = i2c_smbus_write_byte_data(tx->client,
					SIIHDMI_TPI_REG_SYS_CTRL,
					ctrl);
	if (ret < 0)
		DEBUG("unable to enable the display\n");

	/* step 10: (potentially) enable HDCP */

#ifdef SIIHDMI_USE_FB
	memcpy((void *) &tx->sink.current_mode, mode, sizeof(struct fb_videomode));
#endif

	return ret;
}

#ifdef SIIHDMI_USE_FB
static void siihdmi_dump_modelines(struct siihdmi_tx *tx)
{
	const struct fb_modelist *entry;
	const struct list_head * const modelines = &tx->info->modelist;

	INFO("supported modelines:\n");
	list_for_each_entry(entry, modelines, list)
		siihdmi_print_modeline(tx, &entry->mode, NULL);
}
#endif

static inline void siihdmi_process_extensions(struct siihdmi_tx *tx)
{
	const struct edid_block0 * const block0 =
		(struct edid_block0 *) tx->edid.data;
	const struct edid_extension * const extensions =
		(struct edid_extension *) (tx->edid.data + sizeof(*block0));
	u8 i;

	for (i = 0; i < block0->extensions; i++) {
		const struct edid_extension * const extension = &extensions[i];

		if (!edid_verify_checksum((u8 *) extension))
			WARNING("EDID block %u CRC mismatch\n", i);

		switch (extension->tag) {
		case EDID_EXTENSION_CEA:
			siihdmi_parse_cea861_timing_block(tx, extension);
			break;
		default:
			break;
		}
	}
}

#ifdef SIIHDMI_USE_FB

static const struct fb_videomode *
_find_similar_mode(const struct fb_videomode * const mode, struct list_head *head)
{
	const struct fb_modelist *entry, *next;

	list_for_each_entry_safe(entry, next, head, list) {
		if (fb_res_is_equal(mode, &entry->mode) && (mode != &entry->mode))
			return &entry->mode;
	}

	return NULL;
}

static void siihdmi_sanitize_modelist(struct siihdmi_tx * const tx)
{
	struct list_head *modelist = &tx->info->modelist;
	const struct fb_modelist *entry, *next;
	const struct fb_videomode *mode;
	int num_removed = 0;

	if ((mode = fb_find_best_display(&tx->info->monspecs, modelist)))
		tx->sink.preferred_mode = *mode;

	list_for_each_entry_safe(entry, next, modelist, list) {
		const char *reason = NULL;
		mode = &entry->mode;

		if (mode->vmode & FB_VMODE_INTERLACED) {
			reason = "interlaced";
		} else if (mode->vmode & FB_VMODE_DOUBLE) {
			reason = "doublescan";
		} else if (mode->pixclock < tx->platform->pixclock) {
			reason = "pixel clock exceeded";
		} else if ((tx->sink.type == SINK_TYPE_HDMI) && mode->lower_margin < 2) {
			/*
			 * HDMI spec (§ 5.1.2) stipulates ≥2 lines of vsync
			 *
			 * We do not care so much on DVI, although it may be that the SII9022 cannot
			 * actually display this mode. Requires testing!!
			 */
			reason = "insufficient margin";
		} else {
			const struct fb_videomode *match = _find_similar_mode(mode, modelist);

			if (match) {
				/*
				 * Prefer detailed timings found in EDID.  Certain sinks support slight
				 * variations of VESA/CEA timings, and using those allows us to support
				 * a wider variety of monitors.
				 */
				if ((~(mode->flag) & FB_MODE_IS_DETAILED) &&
					(match->flag & FB_MODE_IS_DETAILED)) {
					reason = "detailed match present";
				} else if ((~(mode->flag) & FB_MODE_IS_CEA) &&
						   (match->flag & FB_MODE_IS_CEA)) {
					if ((tx->sink.type == SINK_TYPE_HDMI) && !useitmodes) {
						/*
						 * for HDMI connections we want to remove any detailed timings
						 * and leave in CEA mode timings. This is on the basis that you
						 * would expect HDMI monitors to do better with CEA (TV) modes
						 * than you would PC modes. No data is truly lost: these modes
						 * are duplicated in terms of size and refresh but may have
						 * subtle differences insofaras more compatible timings.
						 *
						 * That is, unless we want to prefer IT modes, since most TVs
						 * will overscan CEA modes (720p, 1080p) by default, but display
						 * IT (PC) modes to the edge of the screen.
						 */
						reason = "CEA match present";
					} else {
						/*
						 * DVI connections are the opposite to the above; remove CEA
						 * modes which duplicate normal modes, on the basis that a
						 * DVI sink will better display a standard EDID mode but may
						 * not be fully compatible with CEA timings. This is the
						 * behavior on HDMI sinks if we want to prefer IT modes.
						 *
						 * All we do is copy the matched mode into the mode value
						 * such that we remove the correct mode below.
						 */
						mode = match;
						reason = "IT match present";
					}
				}
			}
		}

		if (reason) {
			struct fb_modelist *modelist =
				container_of(mode, struct fb_modelist, mode);

			if (num_removed == 0) { // first time only
				INFO("Unsupported modelines:\n");
			}

			siihdmi_print_modeline(tx, mode, reason);

			list_del(&modelist->list);
			kfree(&modelist->list);
			num_removed++;
		}
	}

	if (num_removed > 0) {
		INFO("discarded %u incompatible modes\n", num_removed);
	}
}

static inline const struct fb_videomode *_match(const struct fb_videomode * const mode,
						struct list_head *modelist)
{
	const struct fb_videomode *match;

	if ((match = fb_find_best_mode_at_most(mode, modelist)))
		return match;

	return fb_find_nearest_mode(mode, modelist);
}
#endif

/* This function iterates through the videomode and removes those we don't
 * support */
static void siihdmi_filter_monspecs(struct fb_monspecs *monspecs) {
	int i, j;

	for (i = 0, j = 0; i < monspecs->modedb_len; i++) {
		struct fb_videomode *m = &monspecs->modedb[i];

		if (m->vmode & FB_VMODE_INTERLACED) {
			// SIIHDMI doesn't handle interlaced video
			continue;
		}

		// We can use this mode
		memmove(&monspecs->modedb[j++], m, sizeof(*m));
	}

	monspecs->modedb_len = j;
}

static int siihdmi_setup_display(struct siihdmi_tx *tx)
{
#ifdef SIIHDMI_USE_FB
	struct fb_var_screeninfo	var    = {0};
#endif
	struct edid_block0		*block0;
	unsigned			width;
	unsigned			height;
	int				ret;
	u8				isr;
	u8				*new;
	int				i;

	BUILD_BUG_ON(sizeof(struct edid_block0)    != EDID_BLOCK_SIZE);
	BUILD_BUG_ON(sizeof(struct edid_extension) != EDID_BLOCK_SIZE);

	/* defaults */
	tx->sink.scanning   = SCANNING_EXACT;
	tx->sink.type       = SINK_TYPE_DVI;
	tx->audio.available = false;

	isr = i2c_smbus_read_byte_data(tx->client, SIIHDMI_TPI_REG_ISR);
	DEBUG("hotplug: display %s, powered %s\n",
	      (isr & SIIHDMI_ISR_DISPLAY_ATTACHED) ? "attached" : "detached",
	      (isr & SIIHDMI_ISR_RECEIVER_SENSE) ? "on" : "off");

	if (~isr & (SIIHDMI_ISR_DISPLAY_ATTACHED))
		return siihdmi_power_down(tx);

	/* Free previous video mode database and selected mode */
	if (tx->monspecs.modedb != NULL)
	{
		fb_destroy_modedb(tx->monspecs.modedb);
		tx->selected_mode = NULL;
		tx->monspecs.modedb = NULL;
		tx->monspecs.modedb_len = 0;
	}

	/* Allocate EDID buffer for one block */
	if (tx->edid.data != NULL)
		kfree(tx->edid.data);
	tx->edid.data = kzalloc(EDID_BLOCK_SIZE, GFP_KERNEL);
	if (!tx->edid.data)
		return -ENOMEM;

	/* use EDID to detect sink characteristics */
	ret = siihdmi_read_edid(tx, tx->edid.data, 0, EDID_BLOCK_SIZE);
	block0 = (struct edid_block0 *) tx->edid.data;

	if (ret < 0 || !edid_verify_checksum((u8 *) block0)) {
		WARNING("couldn't read EDID, selecting default vmode");
		/* Set display to 640x480p @ 60Hz */
		tx->selected_mode = avifb_select_mode(
						   tx->platform->lcd_id,
						   &default_monspecs);
		return siihdmi_set_resolution(tx, tx->selected_mode);
	}

	/* Reallocate space for block 0 as well as the extensions */
	tx->edid.length = (block0->extensions + 1) * EDID_BLOCK_SIZE;
	new = krealloc(tx->edid.data, tx->edid.length, GFP_KERNEL);
	if (!new)
		return -ENOMEM;
	tx->edid.data = new;
	block0 = (struct edid_block0 *) new;

	/* create monspecs from EDID for the basic stuff */
	fb_edid_to_monspecs(tx->edid.data, &tx->monspecs);

	/* Read all E-EDID */
	for (i = 1; i <= block0->extensions; i++)
	{
		/* Read an E-EDID block */
		new = tx->edid.data + (i * EDID_BLOCK_SIZE);
		ret = siihdmi_read_edid(tx, new, i, EDID_BLOCK_SIZE);
		if (ret >= 0)
		{
			/* Add monspecs from E-EDID */
			fb_edid_add_monspecs(new, &tx->monspecs);
		}
	}

	if (block0->extensions)
		siihdmi_process_extensions(tx);

	/* Filter unsupported monspecs */
	siihdmi_filter_monspecs(&tx->monspecs);

	tx->selected_mode = avifb_select_mode(tx->platform->lcd_id,
					   tx->monspecs.modedb_len > 0 ?
						     &tx->monspecs :
						     &default_monspecs);

	if (IS_ERR_OR_NULL(tx->selected_mode))
		return tx->selected_mode ? PTR_ERR(tx->selected_mode) : -ENODEV;

	/* Width and height in the EDID block are in cm */
	width = block0->maximum_horizontal_image_size * 10;
	height = block0->maximum_vertical_image_size * 10;

	if (avifb_set_screen_size(tx->platform->lcd_id,
				  width,
				  height) < 0)
		return -ENODEV;

	/* Set monitor specs in avifb */
	avifb_set_monspecs(tx->platform->lcd_id, &tx->monspecs);

	ret = siihdmi_set_resolution(tx, tx->selected_mode);
	if (ret < 0)
		return ret;

#ifdef SIIHDMI_USE_FB
	/* activate the framebuffer */
	fb_videomode_to_var(&var, mode);
	var.activate = FB_ACTIVATE_ALL;

	console_lock();
	tx->info->flags |= FBINFO_MISC_USEREVENT;
	fb_set_var(tx->info, &var);
	tx->info->flags &= ~FBINFO_MISC_USEREVENT;
	console_unlock();
#endif

	return 0;
}

#ifdef SIIHDMI_USE_FB

static int siihdmi_blank(struct siihdmi_tx *tx)
{
	u8 data;

	data = i2c_smbus_read_byte_data(tx->client, SIIHDMI_TPI_REG_RQB);

	return i2c_smbus_write_byte_data(tx->client,
					 SIIHDMI_TPI_REG_RQB,
					 data | SIIHDMI_RQB_FORCE_VIDEO_BLANK);
}

static int siihdmi_unblank(struct siihdmi_tx *tx)
{
	u8 data;

	data = i2c_smbus_read_byte_data(tx->client, SIIHDMI_TPI_REG_RQB);

	return i2c_smbus_write_byte_data(tx->client,
					 SIIHDMI_TPI_REG_RQB,
					 data & ~SIIHDMI_RQB_FORCE_VIDEO_BLANK);
}

static int siihdmi_fb_event_handler(struct notifier_block *nb,
				    unsigned long val,
				    void *v)
{
	const struct fb_event * const event = v;
	struct siihdmi_tx * const tx = container_of(nb, struct siihdmi_tx, nb);

	if (strcmp(event->info->fix.id, tx->platform->framebuffer))
		return 0;

	switch (val) {
	case FB_EVENT_FB_REGISTERED:
	case FB_EVENT_FB_UNREGISTERED:
		return siihdmi_setup_display(tx);

	case FB_EVENT_MODE_CHANGE:
		if (event->info->mode)
			return siihdmi_set_resolution(tx, event->info->mode);
	case FB_EVENT_MODE_CHANGE_ALL:
		/* is handled above, removes a "unhandled event" warning in dmesg */
		break;
	case FB_EVENT_BLANK:
		switch (*((int *) event->data)) {
			case FB_BLANK_POWERDOWN:
				/* do NOT siihdmi_power_down() here */
			case FB_BLANK_VSYNC_SUSPEND:
			case FB_BLANK_HSYNC_SUSPEND:
			case FB_BLANK_NORMAL:
				return siihdmi_blank(tx);
			case FB_BLANK_UNBLANK:
				return siihdmi_unblank(tx);
		}
		break;
	default:
		DEBUG("unhandled fb event 0x%lx", val);
		break;
	}

	return 0;
}
#endif

static irqreturn_t siihdmi_hotplug_event(int irq, void *dev_id)
{
	struct siihdmi_tx *tx = dev_id;
	u8 isr;

	isr = i2c_smbus_read_byte_data(tx->client, SIIHDMI_TPI_REG_ISR);
	if (~isr & SIIHDMI_ISR_HOT_PLUG_EVENT)
		goto complete;

	DEBUG("hotplug: display %s, powered %s\n",
	      (isr & SIIHDMI_ISR_DISPLAY_ATTACHED) ? "attached" : "detached",
	      (isr & SIIHDMI_ISR_RECEIVER_SENSE) ? "on" : "off");

	if (isr & SIIHDMI_ISR_DISPLAY_ATTACHED) {
		siihdmi_power_up(tx);
		siihdmi_setup_display(tx);
	} else {
		siihdmi_power_down(tx);
	}

	if (isr & SIIHDMI_ISR_HOT_PLUG_EVENT) {
		switch_set_state(&tx->sdev,!!(isr & SIIHDMI_ISR_DISPLAY_ATTACHED));
		if (tx->platform->on_hotplug)
			tx->platform->on_hotplug(!!(isr & SIIHDMI_ISR_DISPLAY_ATTACHED));
	}

complete:
	/* clear the interrupt */
	i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_ISR, isr);

	return IRQ_HANDLED;
}

#if defined(CONFIG_SYSFS)
static ssize_t siihdmi_sysfs_read_edid(struct file *file,
				       struct kobject *kobj,
				       struct bin_attribute *bin_attr,
				       char *buf, loff_t offset, size_t count)
{
	const struct siihdmi_tx * const tx =
		container_of(bin_attr, struct siihdmi_tx, edid.attributes);

	return memory_read_from_buffer(buf, count, &offset,
				       tx->edid.data, tx->edid.length);
}

static ssize_t siihdmi_sysfs_read_audio(struct file *file,
					struct kobject *kobj,
					struct bin_attribute *bin_attr,
					char *buf, loff_t off, size_t count)
{
	static const char * const sources[] = { "none\n", "hdmi\n" };
	const struct siihdmi_tx * const tx =
		container_of(bin_attr, struct siihdmi_tx, audio.attributes);

	return memory_read_from_buffer(buf, count, &off,
				       sources[tx->audio.available],
				       strlen(sources[tx->audio.available]));
}
#endif

static int __devinit siihdmi_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct siihdmi_tx *tx;
	int ret;

	tx = kzalloc(sizeof(struct siihdmi_tx), GFP_KERNEL);
	if (!tx)
		return -ENOMEM;

	tx->client = client;
	tx->platform = client->dev.platform_data;

	i2c_set_clientdata(client, tx);

	/* Check if device is present (chip must be reset before checking I2C) */
	if (tx->platform->reset)
		tx->platform->reset();
	ret = i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_RQB, 0x00);
	if (ret < 0) {
		return  -ENODEV;
	}

	tx->sdev.name = "hdmi";
	ret = switch_dev_register(&tx->sdev);
	if (ret)
		goto switch_dev_register_failed;

	/* i2c_client irq is set in parrot_init_i2c_slave_shared_gpio */
	tx->platform->hotplug.start = client->irq;
	ret = request_threaded_irq(tx->platform->hotplug.start, NULL, siihdmi_hotplug_event,
				   tx->platform->hotplug.flags,
				   tx->platform->hotplug.name, tx);
	if (ret < 0)
		WARNING("failed to setup hotplug interrupt: %d\n", ret);
	else
		tx->hotplug.enabled = true;

	/* reinitialise the device to configure irq */
	/* step 1: reset and initialise */
	if (tx->platform->reset)
		tx->platform->reset();
	/* Do not use spinlock here, the siihdmi_initialise
	 * will call scheduler (need to wait the chip) */
	ret = siihdmi_initialise(tx);
	if (ret < 0)
		goto irq_initialize_failed;
	/* Do nothing on the chip configuration because an
	 * interrupt can be raised at anytime.
	 * This will totally break the chip */

#if defined(CONFIG_SYSFS)
	/* /sys/<>/edid */
	tx->edid.attributes.attr.name  = "edid";
	tx->edid.attributes.attr.mode  = 0444;

	/* maximum size of EDID, not necessarily the size of our data */
	tx->edid.attributes.size       = SZ_32K;
	tx->edid.attributes.read       = siihdmi_sysfs_read_edid;

	sysfs_attr_init(&tx->edid.attributes.attr);
	if (sysfs_create_bin_file(&tx->client->dev.kobj, &tx->edid.attributes) < 0)
		WARNING("unable to construct attribute for sysfs exported EDID\n");

	/* /sys/<>/audio */
	tx->audio.attributes.attr.name  = "audio";
	tx->audio.attributes.attr.mode  = 0444;

	/* we only want to return the value "hdmi" or "none" */
	tx->audio.attributes.size       = 5;
	tx->audio.attributes.read       = siihdmi_sysfs_read_audio;

	sysfs_attr_init(&tx->audio.attributes.attr);
	if (sysfs_create_bin_file(&tx->client->dev.kobj, &tx->audio.attributes) < 0)
		WARNING("unable to construct attribute for sysfs exported audio\n");
#endif

	return 0;

irq_initialize_failed:
	if (tx->platform->hotplug.start)
		free_irq(tx->platform->hotplug.start, tx);
	switch_dev_unregister(&tx->sdev);
switch_dev_register_failed:
	i2c_set_clientdata(client, NULL);
	kfree(tx);
	return ret;
}

static int __devexit siihdmi_remove(struct i2c_client *client)
{
	struct siihdmi_tx *tx;

	tx = i2c_get_clientdata(client);

	if (!tx)
		return -ENODEV;

	siihdmi_power_down(tx);

	if (tx->platform->hotplug.start)
		free_irq(tx->platform->hotplug.start, tx);

#if defined(CONFIG_SYSFS)
	sysfs_remove_bin_file(&tx->client->dev.kobj, &tx->edid.attributes);
	sysfs_remove_bin_file(&tx->client->dev.kobj, &tx->audio.attributes);
#endif

	if (tx->edid.data)
		kfree(tx->edid.data);

	switch_dev_unregister(&tx->sdev);
	i2c_set_clientdata(client, NULL);
	kfree(tx);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int siihdmi_suspend(struct i2c_client *client, pm_message_t state)
{
	DEBUG("suspend\n");
	return 0;
}

static int siihdmi_resume(struct i2c_client *client)
{
	int ret;
	struct siihdmi_tx *tx;
	DEBUG("resume\n");
	tx = i2c_get_clientdata(client);
	if (tx == NULL) {
		ERROR("Unconfigured siihdmi\n");
		return 1;
	}

#if 0
	if ((ret = siihdmi_initialise(tx)) < 0)
		return ret;
	ret = siihdmi_setup_display(tx);
#else
	/* Try to initialize faster */
	ret = i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_RQB, 0x00);
	if (ret < 0) {
		WARNING("unable to initialise device to TPI mode\n");
		return ret;
	}

	/*  power up transmitter */
	if ((ret = siihdmi_power_up(tx)) < 0)
		return ret;

	if (tx->hotplug.enabled) {
		/* Re-enable interrupt service */
		ret = i2c_smbus_write_byte_data(tx->client,
						SIIHDMI_TPI_REG_IER,
						SIIHDMI_IER_HOT_PLUG_EVENT);
		if (ret < 0)
			WARNING("unable to setup interrupt request\n");
	}

	ret = i2c_smbus_write_byte_data(tx->client,
					SIIHDMI_TPI_REG_SYS_CTRL,
					SIIHDMI_SYS_CTRL_OUTPUT_MODE_SELECT_HDMI);
#endif
	return 0;
}
#else
#define siihdmi_suspend NULL
#define siihdmi_resume  NULL
#endif

static const struct i2c_device_id siihdmi_device_table[] = {
	{ "siihdmi", 0 },
	{ },
};

static struct i2c_driver siihdmi_driver = {
	.driver   = {
		.owner = THIS_MODULE,
		.name = "siihdmi",
	},
	.suspend  = siihdmi_suspend,
	.resume	  = siihdmi_resume,
	.probe    = siihdmi_probe,
	.remove   = __devexit_p(siihdmi_remove),
	.id_table = siihdmi_device_table,
};

#ifdef MODULE
module_i2c_driver(siihdmi_driver);
#else
static int __init siihdmi_init(void)
{
       return i2c_add_driver(&siihdmi_driver);
}

late_initcall(siihdmi_init);
#endif

/* Module Information */
MODULE_AUTHOR("Saleem Abdulrasool <compnerd@compnerd.org>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Silicon Image SiI9xxx TMDS Driver");
MODULE_DEVICE_TABLE(i2c, siihdmi_device_table);
