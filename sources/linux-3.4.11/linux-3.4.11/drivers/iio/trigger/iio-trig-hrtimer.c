/*
 * iio-trig-hrtimer.c - Standalone IIO trigger based on hrtimer
 *
 * Copyright (c) 2015 Parrot <didier.leymarie.ext@parrot.com>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 */
// #define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/hrtimer.h>

#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>

struct iio_hrtimer_trig {
  struct iio_trigger	*trig;
  struct hrtimer	timer;
  struct mutex		lock;
  int			id;
  unsigned long		period_ns;
  unsigned long 	frequency;
  ktime_t		ktime_period_ns;
  bool			enable;
  struct list_head	l;
};

static LIST_HEAD(iio_hrtimer_trig_list);
static DEFINE_MUTEX(iio_hrtimer_trig_list_mut);

static int iio_hrtimer_trigger_probe(const int id);
static ssize_t iio_hrtimer_trig_add(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t len)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 10, &input);
	if (ret)
		return ret;
	ret = iio_hrtimer_trigger_probe(input);
	if (ret)
		return ret;
	return len;
}
static DEVICE_ATTR(add_trigger, S_IWUSR, NULL, &iio_hrtimer_trig_add);

static int iio_hrtimer_trigger_remove(const int id);
static ssize_t iio_hrtimer_trig_remove(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t len)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 10, &input);
	if (ret)
		return ret;
	ret = iio_hrtimer_trigger_remove(input);
	if (ret)
		return ret;
	return len;
}

static DEVICE_ATTR(remove_trigger, S_IWUSR, NULL, &iio_hrtimer_trig_remove);

static struct attribute *iio_hrtimer_trig_attrs[] = {
	&dev_attr_add_trigger.attr,
	&dev_attr_remove_trigger.attr,
	NULL,
};

static const struct attribute_group iio_hrtimer_trig_group = {
	.attrs = iio_hrtimer_trig_attrs,
};

static const struct attribute_group *iio_hrtimer_trig_groups[] = {
	&iio_hrtimer_trig_group,
	NULL
};

/* Nothing to actually do upon release */
static void iio_trigger_hrtimer_release(struct device *dev)
{
}

static struct device iio_hrtimer_trig_dev = {
	.bus = &iio_bus_type,
	.groups = iio_hrtimer_trig_groups,
	.release = &iio_trigger_hrtimer_release,
};

/* Timer callback: called at the end of period. */
static enum hrtimer_restart iio_hrtimer_periodic_callback(struct hrtimer *timer)
{
	struct iio_hrtimer_trig		*data;
	ktime_t				kt_now;
	int				overrun;
	enum hrtimer_restart		ret = HRTIMER_RESTART;

	data = container_of(timer, struct iio_hrtimer_trig, timer);

	kt_now = hrtimer_cb_get_time(&data->timer);

	overrun = hrtimer_forward(&data->timer,
				  kt_now,
				  data->ktime_period_ns);

	iio_trigger_poll(data->trig);

	dev_dbg(&data->trig->dev, "hrtimer #%d restarted\n", data->id);

	return ret;
}

static int iio_hrtimer_start(struct hrtimer *timer)
{
	struct iio_hrtimer_trig		*hrt_trig;
	hrt_trig = container_of(timer, struct iio_hrtimer_trig, timer);

	if (!hrtimer_active(&hrt_trig->timer)) {
		hrt_trig->ktime_period_ns = ktime_set(0, hrt_trig->period_ns);
		hrt_trig->timer.function = &iio_hrtimer_periodic_callback;
		hrtimer_start(&hrt_trig->timer,
			      hrt_trig->ktime_period_ns,
			      HRTIMER_MODE_REL);
	}
	return 1;
}

static int iio_hrtimer_stop(struct hrtimer *timer)
{
	struct iio_hrtimer_trig	*hrt_trig;
	int			ret = 1;

	hrt_trig = container_of(timer, struct iio_hrtimer_trig, timer);

	if (hrtimer_callback_running(timer)) {
		dev_dbg(&hrt_trig->trig->dev,
			"hrtimer #%d callback is running\n",
			hrt_trig->id);
	}

	if (hrtimer_active(timer) != 0) {
		ret = hrtimer_cancel(timer);
		dev_dbg(&hrt_trig->trig->dev,
			"active hrtimer #%d cancelled: %d\n",
			hrt_trig->id, ret);
	}

	if (hrtimer_is_queued(timer) != 0) {
		ret = hrtimer_cancel(timer);
		dev_dbg(&hrt_trig->trig->dev,
			"queued hrtimer #%d cancelled: %d\n",
			hrt_trig->id, ret);
	}

	return ret;
}

static ssize_t iio_hrtimer_enable_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct iio_trigger	*trig 	  = to_iio_trigger(dev);
	struct iio_hrtimer_trig *hrt_trig = iio_trigger_get_drvdata(trig);
	bool value;
	int ret;

	ret = strtobool(buf, &value);
	if (ret < 0)
		return ret;

	if (value == hrt_trig->enable)
		return count;

	mutex_lock(&hrt_trig->lock);
	switch (value)
	{
	  case false:
		  /* disable timer */
		  ret = iio_hrtimer_stop(&hrt_trig->timer);

		  dev_info(&hrt_trig->trig->dev,
			   "Trigger HR Timer '%s' #%d stopped\n",
			   hrt_trig->trig->name,
			   hrt_trig->id);

		  hrt_trig->enable = false;

		  break;

	  case true:
		  /* enable timer */
		  ret = iio_hrtimer_start(&hrt_trig->timer);
		  hrt_trig->enable = true;
		  dev_info(&hrt_trig->trig->dev,
			    "Trigger HR Timer '%s' #%d started\n",
			    hrt_trig->trig->name,
			    hrt_trig->id);
		  break;
	}
	mutex_unlock(&hrt_trig->lock);
	return count;
}

static ssize_t iio_hrtimer_enable_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct iio_trigger *trig = to_iio_trigger(dev);
	struct iio_hrtimer_trig *hrt_trig = iio_trigger_get_drvdata(trig);

	return sprintf(buf, "%d\n", hrt_trig->enable);
}

/* period_ns <==> frequency Hz */
#define FREQUENCY_1_GHz (1000000000L)

/* period minimum 10 ns ==> frequency max 100 MHz */
#define PERIOD_NS_MIN	(10L)
#define FREQUENCY_MAX	(FREQUENCY_1_GHz/PERIOD_NS_MIN)


static ssize_t iio_hrtimer_period_ns_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct iio_trigger		*trig = to_iio_trigger(dev);
	struct iio_hrtimer_trig	*hrt_trig = iio_trigger_get_drvdata(trig);
	unsigned long			value;
	unsigned long 			frequency;
	int				ret;

	ret = kstrtoul(buf, 10, &value);
	if (ret < 0)
	  return ret;

	if (value == hrt_trig->period_ns)
		return count;

	if ( value < PERIOD_NS_MIN )
		return -EINVAL;

	if (hrtimer_active(&hrt_trig->timer))
		return -EBUSY;

	frequency = FREQUENCY_1_GHz/value;

	mutex_lock(&hrt_trig->lock);
	hrt_trig->period_ns = value;
	hrt_trig->frequency = frequency;
	mutex_unlock(&hrt_trig->lock);

	return count;
}

static ssize_t iio_hrtimer_period_ns_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
  struct iio_trigger *trig = to_iio_trigger(dev);
  struct iio_hrtimer_trig *hrt_trig = iio_trigger_get_drvdata(trig);

  return sprintf(buf, "%lu\n", hrt_trig->period_ns);
}

static ssize_t iio_hrtimer_frequency_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
    struct iio_trigger		*trig = to_iio_trigger(dev);
    struct iio_hrtimer_trig	*hrt_trig = iio_trigger_get_drvdata(trig);
    unsigned long			value;
    unsigned long 			period_ns;
    int				ret;

    ret = kstrtoul(buf, 10, &value);
    if (ret < 0)
	return ret;

    if (value == hrt_trig->frequency)
	return count;

    if ( value == 0 || value > FREQUENCY_MAX )
	return -EINVAL;

    if (hrtimer_active(&hrt_trig->timer))
	return -EBUSY;

    period_ns = FREQUENCY_1_GHz/value;

    mutex_lock(&hrt_trig->lock);
    hrt_trig->frequency = value;
    hrt_trig->period_ns = period_ns;
    mutex_unlock(&hrt_trig->lock);

    return count;
}

static ssize_t iio_hrtimer_frequency_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
    struct iio_trigger *trig = to_iio_trigger(dev);
    struct iio_hrtimer_trig *hrt_trig = iio_trigger_get_drvdata(trig);

    return sprintf(buf, "%lu\n", hrt_trig->frequency);
}

static DEVICE_ATTR(enable,    S_IRUGO|S_IWUSR, iio_hrtimer_enable_show,    iio_hrtimer_enable_store);
static DEVICE_ATTR(period_ns, S_IRUGO|S_IWUSR, iio_hrtimer_period_ns_show, iio_hrtimer_period_ns_store);
static DEVICE_ATTR(frequency, S_IRUGO|S_IWUSR, iio_hrtimer_frequency_show, iio_hrtimer_frequency_store);

static struct attribute *iio_hrtimer_trigger_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_period_ns.attr,
	&dev_attr_frequency.attr,
	NULL,
};

static const struct attribute_group iio_hrtimer_trigger_attr_group = {
	.attrs = iio_hrtimer_trigger_attrs,
};

static const struct attribute_group *iio_hrtimer_trigger_attr_groups[] = {
	&iio_hrtimer_trigger_attr_group,
	NULL
};

static const struct iio_trigger_ops iio_hrtimer_trigger_ops = {
	.owner = THIS_MODULE,
};

static int iio_hrtimer_trigger_probe(const int id)
{
	struct iio_hrtimer_trig *t;
	int ret;
	bool foundit = false;
	mutex_lock(&iio_hrtimer_trig_list_mut);
	list_for_each_entry(t, &iio_hrtimer_trig_list, l)
	if (id == t->id) {
		foundit = true;
		break;
	}
	if (foundit) {
		ret = -EINVAL;
		goto out1;
	}
	t = kmalloc(sizeof(*t), GFP_KERNEL);
	if (t == NULL) {
		ret = -ENOMEM;
		goto out1;
	}
	t->id = id;
	t->trig = devm_iio_trigger_alloc(&iio_hrtimer_trig_dev,
					 "hrtimertrig%d",
					 t->id);
	if (!t->trig) {
		ret = -ENOMEM;
		dev_err(&iio_hrtimer_trig_dev,
			"devm_iio_trigger_alloc(hrtimertrig%d) failed ENOMEM\n",
			t->id);
		goto free_t;
	}

	t->trig->dev.groups = iio_hrtimer_trigger_attr_groups;
	t->trig->ops = &iio_hrtimer_trigger_ops;
	t->trig->dev.parent = &iio_hrtimer_trig_dev;
	iio_trigger_set_drvdata(t->trig, t);

	mutex_init(&t->lock);
	hrtimer_init(&t->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	t->period_ns 	= 1000000000LL/HZ;
	t->frequency	= HZ;
	t->enable 	= false;

	ret = iio_trigger_register(t->trig);
	if (ret)
		goto out2;
	list_add(&t->l, &iio_hrtimer_trig_list);
	__module_get(THIS_MODULE);
	mutex_unlock(&iio_hrtimer_trig_list_mut);
	dev_info(&t->trig->dev, "Trigger HR Timer '%s' #%d created\n",
		 t->trig->name,
		 t->id);
	return 0;

out2:
	iio_trigger_put(t->trig);
free_t:
	kfree(t);
out1:
	mutex_unlock(&iio_hrtimer_trig_list_mut);
	return ret;
}

static int iio_hrtimer_trigger_remove(const int id)
{
	bool foundit = false;
	struct iio_hrtimer_trig *t;
	mutex_lock(&iio_hrtimer_trig_list_mut);
	list_for_each_entry(t, &iio_hrtimer_trig_list, l)
	if (id == t->id) {
		foundit = true;
		break;
	}
	if (!foundit) {
		mutex_unlock(&iio_hrtimer_trig_list_mut);
		return -EINVAL;
	}

	iio_hrtimer_stop(&t->timer);
	iio_trigger_unregister(t->trig);
	devm_iio_trigger_free(&iio_hrtimer_trig_dev, t->trig);

	list_del(&t->l);
	kfree(t);
	module_put(THIS_MODULE);
	mutex_unlock(&iio_hrtimer_trig_list_mut);
	return 0;
}


static int __init iio_hrtimer_trig_init(void)
{
	device_initialize(&iio_hrtimer_trig_dev);
	dev_set_name(&iio_hrtimer_trig_dev, "iio_hrtimer_trigger");
	return device_add(&iio_hrtimer_trig_dev);
}
module_init(iio_hrtimer_trig_init);

static void __exit iio_hrtimer_trig_exit(void)
{
	device_unregister(&iio_hrtimer_trig_dev);
}
module_exit(iio_hrtimer_trig_exit);

MODULE_AUTHOR("Didier Leymarie <didier.leymarie.ext@parrot.com>");
MODULE_DESCRIPTION("HRtimer trigger for the IIO subsystem");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:iio-trig-hrtimer");
