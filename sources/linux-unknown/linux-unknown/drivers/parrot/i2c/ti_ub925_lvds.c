#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include "ti_ub925_lvds.h"

#define LVDS_MAXIRQ 2
struct ti_lvds_data {
	struct i2c_client *client;
	const struct ti_lvds_platform_data *pdata;
};


/* data */
static struct {
	irq_handler_t handler;
	void *dev_id;
} lvds_irq [LVDS_MAXIRQ];
static int ti_lvds_irq;

static DEFINE_SPINLOCK(ldvs_lock);

/* func */
static int _ti_lvds_read_reg(struct i2c_client *client, int addr,
			       u8 reg)
{
	struct i2c_msg xfer[2];
	u8 buf[1];

	buf[0] = reg & 0xff;

	/* Write register */
	xfer[0].addr = addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = 1;
	xfer[1].buf = buf;

	if (i2c_transfer(client->adapter, xfer, 2) != 2) {
		dev_err(&client->dev, "%s: i2c transfer failed for (%x,%x)\n",
				__func__, addr, reg);
		return -EIO;
	}

	return buf[0];
}
static int ti_lvds_read_reg(struct i2c_client *client,
			       u8 reg)
{
	return _ti_lvds_read_reg(client, client->addr, reg);
}

static int ti_lvds_read_write(struct i2c_client *client,
			       u8 reg, u8 val)
{
	char buff[2] = {reg, val};
	if (i2c_master_send(client, buff, 2) != 2) {
		dev_err(&client->dev, "%s: i2c transfer failed for (%x,%x)\n",
				__func__, client->addr, reg);
		return -EIO;
	}
	return 0;
}

static int _ti_lvds_read_write(struct i2c_client *client, int addr,
			       u8 reg, u8 val)
{
	char buff[2] = {reg, val};
	struct i2c_msg msg;
	int ret;

	msg.addr = addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = 2;
	msg.buf = (char *)buff;

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret != 1) {
		dev_err(&client->dev, "%s: i2c transfer failed\n", __func__);
		return -EIO;
	}
	return 0;
}

static ssize_t i2c_pass_all_show(struct device *device,
					struct device_attribute *attr,
					char *buf)
{
	struct i2c_client *client = to_i2c_client(device);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			ti_lvds_read_reg(client, 0x17) & 0x80);
}

static ssize_t i2c_pass_all_store(struct device *device,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(device);
	unsigned long val = simple_strtoull(buf, NULL, 10);

	ti_lvds_read_write(client, 0x17, 0x5e | (!!val)<<7 );

	return count;
}


static struct device_attribute ti_lvds_sysfs_attrs =
        __ATTR(i2c_pass_all, S_IRUGO | S_IWUSR, i2c_pass_all_show, i2c_pass_all_store);

static int ti_lvds_initialize(struct ti_lvds_data *data)
{
	struct i2c_client *client = data->client;
	const struct ti_lvds_platform_data *settings = NULL;
	int remote;

	if (ti_lvds_read_reg(client, 0xF0) != '_')
		return -ENODEV;
	if (ti_lvds_read_reg(client, 0xF1) != 'U')
		return -ENODEV;
	if (ti_lvds_read_reg(client, 0xF2) != 'B')
		return -ENODEV;
	if (ti_lvds_read_reg(client, 0xF3) != '9')
		return -ENODEV;
	if (ti_lvds_read_reg(client, 0xF4) != '2')
		return -ENODEV;
	if (ti_lvds_read_reg(client, 0xF5) != '5')
		return -ENODEV;

	remote = ti_lvds_read_reg(client, 0x6);
	remote >>= 1;
	if (!remote) {
		dev_err(&client->dev,"lvds there is no remote");
		return -ENODEV;
	}
	dev_info(&client->dev, "TI UB925 LVDS deser addr %x\n", remote);
	if (data->pdata->deser_callback)
		data->pdata->deser_callback(remote, &settings);

	if(!settings){
		settings = data->pdata;
	}

	/* i2c pass thu : access to remote deser and other slave on local i2c bus */
	ti_lvds_read_write(client, 0x3, 0xda |
				(0x1 & settings->clock_rising));

	if (settings->nb_i2c_slave != 1)
		/* i2c pass all : all i2c transfert forwarded to remote i2c bus (default is
		remote + slave_alias) */
		ti_lvds_read_write(client, 0x17, 0xde);

	/* irq ena */
	ti_lvds_read_write(client, 0xc6, 0x21);

	/* i2c address to remap */
	if(settings->premap.slave_id && settings->premap.slave_alias){
		ti_lvds_read_write(client, 0x7, settings->premap.slave_id << 1); /* i2c addr of slave */
		ti_lvds_read_write(client, 0x8, settings->premap.slave_alias << 1); /* i2c addr seem by the master */
	}

	/* increase remote i2c speed */
	_ti_lvds_read_write(client, remote, 0x26, 0x25);
	_ti_lvds_read_write(client, remote, 0x27, 0x25);

	/* deser : Pixel Clock Edge Select rising
		(compliant with DS9OUB926Q & DS9OUB928Q) */
	if(settings->clock_rising)
		_ti_lvds_read_write(client, remote, 0x3, 0xf1);

	if(settings->cmd){
		int i = 0;

		while(settings->cmd[i].reg && settings->cmd[i].data){
			_ti_lvds_read_write(client, remote,
						settings->cmd[i].reg,
						settings->cmd[i].data);

			i++;
		}
	}
	return 0;
}

#if 0
{
	static struct irq_chip	twl4030_irq_chip;
	int irq_base = TWL_IRQ_END;
	int irq_end = irq_base + 4;
	int i;

	/* install an irq handler for each of the SIH modules;
	 * clone dummy irq_chip since PIH can't *do* anything
	 */
	twl4030_irq_chip = dummy_irq_chip;
	twl4030_irq_chip.name = "lvds";

	for (i = irq_base; i < irq_end; i++) {
		set_irq_chip_and_handler(i, &twl4030_irq_chip,
				handle_simple_irq);
		activate_irq(i);
	}

}
#endif

static irqreturn_t lvds_interrupt(int irq, void *dev_id)
{
	struct ti_lvds_data *data = dev_id;
	struct i2c_client *client = data->client;
	int count = 0;

	ti_lvds_read_reg(client, 0xc7);
#if 0
	if (!status)
		printk(KERN_DEBUG "%s spurious irq ???\n", __func__);
#endif
	do {
		int r = 0;
		int i;

		for (i = 0; i < LVDS_MAXIRQ; i++) {
			if (!lvds_irq[i].handler)
				continue;
			r = lvds_irq[i].handler(irq, lvds_irq[i].dev_id);
			if (r)
				break;
		}
		/* no irq handle for all handler */
		if (!r)
			break;

		if (count++ == 10)
			printk(KERN_DEBUG "ti ldvs irq storm. May be driver don't return IRQ_NONE !\n");

		/* ack lvds */
		ti_lvds_read_reg(client, 0xc7);
	} while (1);

	return IRQ_HANDLED;
}

static int __devinit ti_lvds_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	const struct ti_lvds_platform_data *pdata = client->dev.platform_data;
	struct ti_lvds_data *data;
	int error;

	data = kzalloc(sizeof(*data), GFP_KERNEL);

	data->client = client;


	i2c_set_clientdata(client, data);

	data->pdata = pdata;

	error = ti_lvds_initialize(data);
	if (error)
		goto err_free_object;

	error = request_threaded_irq(client->irq, NULL, lvds_interrupt,
			IRQF_TRIGGER_FALLING, client->dev.driver->name, data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_object;
	}
	ti_lvds_irq = client->irq;
	ti_lvds_read_reg(client, 0xc7);

	error = device_create_file(&client->dev, &ti_lvds_sysfs_attrs);
	if (error) {
		goto err_free_object;
	}

	dev_info(&client->dev, "TI UB925 LVDS serializer initialized.\n");

	return 0;

err_free_object:
	kfree(data);

	return error;
}

static int __devexit ti_lvds_remove(struct i2c_client *client)
{
	struct ti_lvds_data *data = i2c_get_clientdata(client);

	free_irq(client->irq, data);
	ti_lvds_irq = 0;
	device_remove_file(&client->dev, &ti_lvds_sysfs_attrs);
	kfree(data);

	return 0;
}

static int ti_lvds_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}
static int ti_lvds_resume(struct i2c_client *client)
{
	struct ti_lvds_data *data = i2c_get_clientdata(client);
	int error;
	/* on workbench, after resume the chip comme out of reset (gpio
	   expander reinit) and we need to wait to address it on i2c
	 */
	int timeout = 10;
	while (timeout-- && ti_lvds_read_reg(client, 0xF0) < 0);
	error = ti_lvds_initialize(data);
	if (!error)
		ti_lvds_read_reg(client, 0xc7);
	return 0;
}

static const struct i2c_device_id ti_lvds_id[] = {
	{ "lvds", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ti_lvds_id);

static struct i2c_driver ti_lvds_driver = {
	.driver = {
		.name	= "lvds",
		.owner	= THIS_MODULE,
	},
	.probe		= ti_lvds_probe,
	.remove		= __devexit_p(ti_lvds_remove),
	.suspend	= ti_lvds_suspend,
	.resume		= ti_lvds_resume,
	.id_table	= ti_lvds_id,
};

static int __init ti_lvds_init(void)
{
	return i2c_add_driver(&ti_lvds_driver);
}

static void __exit ti_lvds_exit(void)
{
	i2c_del_driver(&ti_lvds_driver);
}

/* need to be started before other i2c device to init bus */
subsys_initcall(ti_lvds_init);
module_exit(ti_lvds_exit);


int
lvds_request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags1,
	    const char *name, void *dev)
{
	unsigned long flags;
	int i;
	spin_lock_irqsave(&ldvs_lock, flags);
	for (i = 0; i < LVDS_MAXIRQ; i++) {
		if (!lvds_irq[i].handler) {
			lvds_irq[i].dev_id = dev;
			lvds_irq[i].handler = handler;
			break;
		}
	}
	spin_unlock_irqrestore(&ldvs_lock, flags);
	if (i == LVDS_MAXIRQ)
		return -EINVAL;

	/* Call one time handler, to prevent from unprocessed irq blocking */
	handler(irq, dev);

	return 0;
}

void lvds_free_irq(unsigned int irq, void *dev_id)
{
	unsigned long flags;
	int i;
	spin_lock_irqsave(&ldvs_lock, flags);
	for (i = 0; i < LVDS_MAXIRQ; i++) {
		if (lvds_irq[i].dev_id == dev_id ) {
			lvds_irq[i].handler = NULL;
			lvds_irq[i].dev_id = NULL;
			break;
		}
	}
	spin_unlock_irqrestore(&ldvs_lock, flags);
}

void lvds_enable_irq(unsigned int irq)
{
	enable_irq(ti_lvds_irq);
}

void lvds_disable_irq(unsigned int irq)
{
	disable_irq(ti_lvds_irq);
}

EXPORT_SYMBOL(lvds_request_irq);
EXPORT_SYMBOL(lvds_free_irq);
EXPORT_SYMBOL(lvds_enable_irq);
EXPORT_SYMBOL(lvds_disable_irq);

MODULE_LICENSE("GPL");
