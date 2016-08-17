#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include <iio/platform_data/mykonos3.h>

struct iio_mykonos3_data {
	unsigned int		count;
	/* child devices */
	struct device_driver	*ak8975;
	struct device_driver	*ms5607;
	struct device_driver	*bldc;
};

struct iio_mykonos3_device {
	struct iio_dev		*indio_dev;
	struct iio_mykonos3_ops	*ops;
	int			status;
	int			retry;
};

static struct iio_mykonos3_device iio_mykonos3_devtab[IIO_MYKONOS3_DEVTYPE_NB];

static const struct iio_info iio_mykonos3_info = {
	.driver_module = THIS_MODULE,
};

static const struct iio_chan_spec iio_mykonos3_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(0),
};

static irqreturn_t iio_mykonos3_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct iio_mykonos3_data *data = iio_priv(indio_dev);
	struct iio_mykonos3_device *dev;

	/* bldc */
	dev = &iio_mykonos3_devtab[IIO_MYKONOS3_MOTORS];
	dev->status = dev->ops->fetch_data(dev->indio_dev);

	if (!(data->count % 4)) {
		/* magnetometer */
		dev = &iio_mykonos3_devtab[IIO_MYKONOS3_MAGNETO];
		if (dev->ops && dev->ops->request_data)
			dev->status = dev->ops->request_data(dev->indio_dev);
	} else if (!((data->count + 3) % 4)) {
		/* magnetometer */
		dev = &iio_mykonos3_devtab[IIO_MYKONOS3_MAGNETO];
		if (dev->ops && dev->ops->fetch_data)
			dev->status = dev->ops->fetch_data(dev->indio_dev);
	} else if (!((data->count + 2) % 4)) {
		/* temperature */
		dev = &iio_mykonos3_devtab[IIO_MYKONOS3_TEMP];
		if (dev->ops && dev->ops->fetch_data)
			dev->status = dev->ops->fetch_data(dev->indio_dev);
	} else {
		/* pressure */
		dev = &iio_mykonos3_devtab[IIO_MYKONOS3_BAROMETER];
		if (dev->ops && dev->ops->fetch_data)
			dev->status = dev->ops->fetch_data(dev->indio_dev);
	}
	data->count++;

	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

void iio_mykonos3_register(enum iio_mykonos3_devtype type,
			   struct iio_dev *indio_dev,
			   struct iio_mykonos3_ops *ops)
{
	struct iio_mykonos3_device *dev = NULL;

	if (type >= IIO_MYKONOS3_DEVTYPE_NB) {
		pr_err("Error registering device: Unknown device type: %d\n",
		       type);
		goto exit;
	}

	dev = &iio_mykonos3_devtab[type];

	if (dev->indio_dev || dev->ops) {
		pr_err("Error registering device: type %d already registered\n",
		       type);
		goto exit;
	}

	dev->indio_dev = indio_dev;
	dev->ops = ops;
exit:
	return;
}
EXPORT_SYMBOL(iio_mykonos3_register);

void iio_mykonos3_unregister(enum iio_mykonos3_devtype type,
			     struct iio_dev *indio_dev)
{
	struct iio_mykonos3_device *dev = NULL;

	if (type >= IIO_MYKONOS3_DEVTYPE_NB) {
		pr_err("Error unregistering device: Unknown device type: %d\n",
		       type);
		goto exit;
	}

	dev = &iio_mykonos3_devtab[type];
	if (!dev || dev->indio_dev != indio_dev) {
		pr_err("Error registering device: type %d not owned by '%s'\n",
		       type, indio_dev->name);
		goto exit;
	}

	dev->indio_dev = NULL;
	dev->ops = NULL;
exit:
	return;
}
EXPORT_SYMBOL(iio_mykonos3_unregister);

static int iio_mykonos3_probe(struct platform_device *dev)
{
	struct iio_mykonos3_data *data = NULL;
	struct iio_dev*indio_dev = NULL;
	int ret = 0;

	/* Register with IIO */
	indio_dev = devm_iio_device_alloc(&dev->dev, sizeof(*data));
	if (!indio_dev) {
		ret = -ENOMEM;
		goto exit;
	}

	data = iio_priv(indio_dev);

	indio_dev->dev.parent   = &dev->dev;
	indio_dev->channels     = iio_mykonos3_channels;
	indio_dev->num_channels = ARRAY_SIZE(iio_mykonos3_channels);
	indio_dev->info         = &iio_mykonos3_info;
	indio_dev->modes        = INDIO_DIRECT_MODE;
	indio_dev->name         = dev->name;

	ret = iio_triggered_buffer_setup(indio_dev, NULL,
					 iio_mykonos3_trigger_handler, NULL);
	if (ret < 0)
		goto exit;

	ret = devm_iio_device_register(&dev->dev, indio_dev);
	if (ret < 0)
		goto exit;

	platform_set_drvdata(dev, indio_dev);

	return 0;
exit:
	iio_triggered_buffer_cleanup(indio_dev);
	return ret;
}

static int iio_mykonos3_remove(struct platform_device *dev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(dev);

	devm_iio_device_unregister(&dev->dev, indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);

	return 0;
}

static struct platform_driver iio_mykonos3_driver = {
	.probe		= iio_mykonos3_probe,
	.remove		= __devexit_p(iio_mykonos3_remove),
	.driver		= {
		.owner  = THIS_MODULE,
		.name	= "iio_mykonos3",
	},
};
module_platform_driver(iio_mykonos3_driver);
