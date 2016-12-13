#ifndef _IIO_MYKONOS3_H_
#define _IIO_MYKONOS3_H_

#include <linux/iio/iio.h>

enum iio_mykonos3_devtype {
	IIO_MYKONOS3_MOTORS,
	IIO_MYKONOS3_MAGNETO,
	IIO_MYKONOS3_BAROMETER,
	IIO_MYKONOS3_TEMP,

	IIO_MYKONOS3_DEVTYPE_NB,
};

typedef int (*iio_mykonos3_request_data)(struct iio_dev *);
typedef int (*iio_mykonos3_fetch_data)(struct iio_dev *);

struct iio_mykonos3_ops {
	iio_mykonos3_request_data	request_data;
	iio_mykonos3_fetch_data		fetch_data;
};

void iio_mykonos3_register(enum iio_mykonos3_devtype type,
			   struct iio_dev *indio_dev,
			   struct iio_mykonos3_ops *ops);
void iio_mykonos3_unregister(enum iio_mykonos3_devtype type,
			     struct iio_dev *indio_dev);

#endif
