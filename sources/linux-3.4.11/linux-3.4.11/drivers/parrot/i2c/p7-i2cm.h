#ifndef _P7_I2CM_H
#define _P7_I2CM_H

#if defined(CONFIG_I2CM_PARROT7) || \
    defined(CONFIG_I2CM_PARROT7_MODULE)

#define P7I2CM_DRV_NAME "p7-i2cm"

enum p7i2cm_revision {
        I2CM_REVISION_1 = 1,
        I2CM_REVISION_2,
        I2CM_REVISION_3,
        I2CM_REVISION_NR
};

struct p7i2cm_plat_data {
	/* Bus frequency in kHz. Should be:
	 *
	 *  <= 100kHz for standard-mode
	 *  <= 400kHz for fast-mode,
	 *  <= 1MHz for fast-mode+
	 */
	unsigned int            bus_freq;
	/* High speed mode (enabled globaly for all devices on the bus) */
	bool                    high_speed;
	/* 3bit master ID, used in high speed mode arbitration. 0 shouldn't be
	 * used ("reserved for test and diagnostic purposes") */
	unsigned                master_id;
	/* bus frequency for high speed mode (in kHz). Up to 3.4MHz. */
	unsigned int            bus_high_freq;
	/* if true the controller is behind a mux and has no direct control on
	 * its output pins. */
	unsigned                muxed;
	/* Chip revision */
	enum p7i2cm_revision    revision;
};

#endif /* defined(CONFIG_I2CM_PARROT7) || \
          defined(CONFIG_I2CM_PARROT7_MODULE) */

#endif
