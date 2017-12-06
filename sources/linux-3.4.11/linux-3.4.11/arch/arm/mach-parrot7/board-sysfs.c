#include <linux/string.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <asm/io.h>

#include <mach/p7.h>

#include "common.h"
#include "board-sysfs.h"
#include "mpmc.h"
#include "avi.h"

struct hw_info_i2c {
	int            bus;
	unsigned short addr;
};
struct hw_info_uart {
	int            bus;
	int            cts_rts;
};
struct hw_info {
	char           *type;
	char           *name;
	char           *version;
	union {
		struct hw_info_i2c i2c;
		struct hw_info_uart uart;
	} data;
};

static int match_device(struct device *dev, void *data)
{
	struct hw_info *infos = dev_get_drvdata(dev);
	return !strcmp(infos->name, data);
}
static ssize_t hw_show_info(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct hw_info *infos = dev_get_drvdata(dev);
	if (infos == NULL)
		return -EINVAL;
	if (attr == NULL)
		return -EINVAL;
	if (!strcmp(attr->attr.name, "type"))
		return sprintf(buf, "%s\n", infos->type);
	if (!strcmp(attr->attr.name, "name"))
		return sprintf(buf, "%s\n", infos->name);
	if (!strcmp(attr->attr.name, "version"))
		return sprintf(buf, "%s\n", infos->version);
	if (!strcmp(attr->attr.name, "i2c_bus"))
		return sprintf(buf, "%d\n", infos->data.i2c.bus);
	if (!strcmp(attr->attr.name, "i2c_addr"))
		return sprintf(buf, "0x%02X\n", infos->data.i2c.addr);
	if (!strcmp(attr->attr.name, "uart_bus"))
		return sprintf(buf, "%d\n", infos->data.uart.bus);
	if (!strcmp(attr->attr.name, "uart_cts_rts"))
		return sprintf(buf, "%d\n", infos->data.uart.cts_rts);
	printk(KERN_ERR "Hardware infos : Invalid property %s\n",
	       attr->attr.name);
	return -EINVAL;
}

static DEVICE_ATTR(type, S_IRUGO, hw_show_info, NULL);
static DEVICE_ATTR(name, S_IRUGO, hw_show_info, NULL);
static DEVICE_ATTR(version, S_IRUGO, hw_show_info, NULL);
static DEVICE_ATTR(i2c_bus, S_IRUGO, hw_show_info, NULL);
static DEVICE_ATTR(i2c_addr, S_IRUGO, hw_show_info, NULL);
static DEVICE_ATTR(uart_bus, S_IRUGO, hw_show_info, NULL);
static DEVICE_ATTR(uart_cts_rts, S_IRUGO, hw_show_info, NULL);

struct hw_info *get_hw_info_new_device(const char *name, char *type)
{
	static struct class *hw_class;
	dev_t hw_info_dev;
	struct device *dev;
	struct hw_info *dev_hw_infos;
	const char *device_name;
	char unique_name[32];
	int ret;
	int item_nb = 0;

	/* If class doe not exists, create it */
	if (hw_class == NULL) {
		hw_class = class_create(NULL, "hwdb");
		if (IS_ERR(hw_class))
			return NULL;
	}

	/* If necessary, use default name */
	if (name == NULL || strlen(name) == 0)
		device_name = "unknown";
	else
		device_name = name;

	/* Check if 'name' device exists, try from 'name'1 to 'name'16 */
	strncpy(unique_name, device_name, sizeof(unique_name));
	unique_name[sizeof(unique_name)-2] = '\0';
	while(class_find_device(hw_class, NULL, unique_name, match_device)) {
		item_nb++;
		if (item_nb > 16)
			return NULL;
		snprintf(unique_name, sizeof(unique_name), "%s%d",
			 device_name, item_nb);
	}

	/* Create new device for this element */
	alloc_chrdev_region(&hw_info_dev, 0, 1, unique_name);
	dev = device_create(hw_class, NULL, hw_info_dev, NULL, unique_name);

	/* Export needed file entries */
	device_create_file(dev, &dev_attr_type);
	device_create_file(dev, &dev_attr_name);
	device_create_file(dev, &dev_attr_version);
	if (!strcmp(type, "uart")) {
		device_create_file(dev, &dev_attr_uart_cts_rts);
		device_create_file(dev, &dev_attr_uart_bus);
	}
	else if (!strcmp(type, "i2c")) {
		device_create_file(dev, &dev_attr_i2c_bus);
		device_create_file(dev, &dev_attr_i2c_addr);
	}

	/* Register device infos in device */
	dev_hw_infos = kmalloc(sizeof(struct hw_info), GFP_KERNEL);
	if (dev_hw_infos == NULL)
		return NULL;
	ret = dev_set_drvdata(dev, dev_hw_infos);
	if (ret)
		return NULL;

	/* Give back device infos to allow completion */
	return dev_hw_infos;
}

/**
 * p7brd_export_i2c_hw_infos - Create sysfs entries
 *                             for i2c user hw information.
 */
void __init p7brd_export_i2c_hw_infos(int i2c_bus, unsigned short addr,
				      char *version, const char *name)
{
	struct hw_info *new_i2c_entry;
	printk(KERN_INFO "hw_info : i2c-%d device 0x%02X : %s (%s)\n",
	       i2c_bus, addr, name, version);
	new_i2c_entry = get_hw_info_new_device(name, "i2c");
	if (new_i2c_entry == NULL)
		return;
	new_i2c_entry->type = "i2c";
	new_i2c_entry->name = kstrdup(name, GFP_KERNEL);
	new_i2c_entry->version = kstrdup(version, GFP_KERNEL);
	new_i2c_entry->data.i2c.bus = i2c_bus;
	new_i2c_entry->data.i2c.addr = addr;
}


/**
 * p7brd_export_uart_hw_infos - Create sysfs entries
 *                             for uart user hw information.
 */
void __init p7brd_export_uart_hw_infos(int uart, int rts_cts, char *version)
{
	struct hw_info *new_uart_entry;
	char unid[32];
	printk(KERN_INFO "hw_info : uart %d rts/cts %d\n",
	       uart, rts_cts);
	snprintf(unid, sizeof(unid), "uart-%d", uart);
	new_uart_entry = get_hw_info_new_device(unid, "uart");
	if (new_uart_entry == NULL)
		return;
	new_uart_entry->type = "uart";
	new_uart_entry->name = kstrdup(unid, GFP_KERNEL);
	new_uart_entry->version = kstrdup(version, GFP_KERNEL);
	new_uart_entry->data.uart.bus = uart;
	new_uart_entry->data.uart.cts_rts = rts_cts;
}

static enum p7_mpmc_port p7brd_mpmpc_find_port(const char *name)
{
	static const struct {
		const char		*name;
		enum p7_mpmc_port	 port;
	} ports[] = {
		{ .name = "cpu_dma",   .port = P7_MPMC_CPU_DMA_PORT   },
		{ .name = "avi",       .port = P7_MPMC_AVI_PORT       },
		{ .name = "vdec_venc", .port = P7_MPMC_VDEC_VENC_PORT },
		{ .name = "hsp_aai",   .port = P7_MPMC_HSP_AAI_PORT   },
		{ .name = "gpu_venc",  .port = P7_MPMC_GPU_VENC_PORT  },
	};

	int i;

	for (i = 0; i < ARRAY_SIZE(ports); i++) {
		if (!strcmp(name, ports[i].name))
			return ports[i].port;
	}

	BUG();
}


static ssize_t p7brd_mpmc_show_prio(struct device		*device,
				    struct device_attribute	*attr,
				    char			*buf)
{
	enum p7_mpmc_port	port = p7brd_mpmpc_find_port(attr->attr.name);
	unsigned		prio = p7_mpmc_get_prio(port);

	return snprintf(buf, PAGE_SIZE, "%u\n", prio);
}

static ssize_t p7brd_mpmc_store_prio(struct device		*device,
				     struct device_attribute	*attr,
				     const char			*buf,
				     size_t			 count)
{
	char			*last;
	unsigned long		 prio = simple_strtoul(buf, &last, 0);
	enum p7_mpmc_port	 port = p7brd_mpmpc_find_port(attr->attr.name);
	int			 ret;

	if (*last != '\0' && *last != '\n')
		return -EINVAL;

	ret = p7_mpmc_set_prio(port, prio);
	if (ret)
		return ret;

	return count;
}

static struct device_attribute p7brd_mpmc_prio_attrs[] = {
#define MAKE_PRIO(_n) __ATTR(_n, S_IRUGO | S_IWUSR, \
			     p7brd_mpmc_show_prio, p7brd_mpmc_store_prio)
	MAKE_PRIO(cpu_dma),
	MAKE_PRIO(avi),
	MAKE_PRIO(vdec_venc),
	MAKE_PRIO(hsp_aai),
	MAKE_PRIO(gpu_venc),
};

int __init p7brd_export_mpmc_priorities(void)
{
	struct class	*class;
	struct device   *dev;
	int		 i;
	int		 ret;

	class = class_create(NULL, "mpmc");
	if (IS_ERR(class)) {
		ret = PTR_ERR(class);
		goto class_create_failed;
	}

	dev = device_create(class, NULL, MKDEV(0, 0), NULL, "prio");
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		goto device_create_failed;
	}

	for (i = 0; i < ARRAY_SIZE(p7brd_mpmc_prio_attrs); i++) {
		ret = device_create_file(dev, &p7brd_mpmc_prio_attrs[i]);
		if (ret) {
			while (i--)
				device_remove_file(dev,
						   &p7brd_mpmc_prio_attrs[i]);
			goto sysfs_failed;
		}
	}

	return 0;

 sysfs_failed:
	device_destroy(class, MKDEV(0, 0));
 device_create_failed:
	class_destroy(class);
 class_create_failed:
	return ret;
}

#if defined(CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE)

static struct device_attribute p7brd_fb_pos_attrs[AVIFB_MAX_INSTANCES *
						  AVIFB_MAX_OVERLAYS];

#define FB_POS_SIZE 15

static char p7brd_fb_pos_name[AVIFB_MAX_INSTANCES * AVIFB_MAX_OVERLAYS][FB_POS_SIZE];

static ssize_t p7brd_fb_show_position(struct device		*device,
				      struct device_attribute	*attr,
				      char			*buf)
{
	unsigned	lcd;
	unsigned	overlay;
	unsigned	off;

	/* We need to know which lcd/overlay we need to display. We use the
	 * offset of the attr in the global array to recover that. */
	off = attr - p7brd_fb_pos_attrs;

	lcd     = off / AVIFB_MAX_OVERLAYS;
	overlay = off % AVIFB_MAX_OVERLAYS;

	return snprintf(buf, PAGE_SIZE, "%ux%u @ %ux%u\n",
			p7_avifb_width     [lcd][overlay],
			p7_avifb_height    [lcd][overlay],
			p7_avifb_position_x[lcd][overlay],
			p7_avifb_position_y[lcd][overlay]);
}

int __init p7brd_export_fb_positions(void)
{
	struct class	*class;
	struct device   *dev;
	int		 ret;
	unsigned	 lcd;
	unsigned	 overlay;

	class = class_create(NULL, "lcdfb");
	if (IS_ERR(class)) {
		ret = PTR_ERR(class);
		goto class_create_failed;
	}

	dev = device_create(class, NULL, MKDEV(0, 0), NULL, "position");
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		goto device_create_failed;
	}

	for (lcd = 0; lcd < AVIFB_MAX_INSTANCES; lcd++) {
		for (overlay = 0; overlay < AVIFB_MAX_OVERLAYS; overlay++) {
			char			*namestr;
			struct device_attribute	*devattr;
			unsigned		 off;

			off = lcd * AVIFB_MAX_OVERLAYS + overlay;

			namestr =  p7brd_fb_pos_name[off];
			devattr = &p7brd_fb_pos_attrs[off];

			snprintf(namestr, FB_POS_SIZE, "avifb.%u.%u",
				 lcd, overlay);

			devattr->attr.name = namestr;
			devattr->attr.mode = S_IRUGO;

			devattr->show  = &p7brd_fb_show_position;
			devattr->store = NULL;

			ret = device_create_file(dev, devattr);
			if (ret)
				/* XXX cleanup sysfs entries */
				goto sysfs_failed;
		}
	}

	return 0;

 sysfs_failed:
	device_destroy(class, MKDEV(0, 0));
 device_create_failed:
	class_destroy(class);
 class_create_failed:
	return ret;

}

#endif /* if defined(CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE) */
