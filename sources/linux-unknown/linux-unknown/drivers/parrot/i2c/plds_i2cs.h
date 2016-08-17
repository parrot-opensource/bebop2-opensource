#ifndef _P7_I2CS_H
#define _P7_I2CS_H

#define PLDS_I2CS_NAME "plds"

/* IOCTLS */
#define PLDS_FLUSH_ALL  _IO(0x08,1)
#define PLDS_DUMP_DEBUG _IO(0x08,2)
#define PLDS_RESET	_IO(0x08,3)

enum p7i2cs_revision {
        I2CS_REVISION_1 = 1,
        I2CS_REVISION_2,
        I2CS_REVISION_3,
        I2CS_REVISION_NR
};

struct plds_i2cs_pdata {
	int                     gpio_request;
	int			gpio_reset;
	int                     i2c_bus;
	enum p7i2cs_revision    revision;
};

#endif /* _P7_I2CS_H */
