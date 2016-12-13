/**
 * linux/driver/parrot/gpu/ion/p7-ion.c - Parrot7 ION allocator implementation
 *
 * Copyright (C) 2014 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    16-Jan-2014
 *
 * This file is released under the GPL.
 *
 * TODO:
 *  - review me !!
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/ion.h>
#include "ion_priv.h"
#include <asm/uaccess.h>
#include "p7-ion.h"

static struct ion_device*   p7ion_dev;
static unsigned int         p7ion_heaps_nr;
static struct ion_heap**    p7ion_heaps;

static long p7ion_custom_ioctl(struct ion_client *client,
                                unsigned int cmd,
                                unsigned long arg)
{
	switch (cmd) {
		case P7ION_GET_BUFFER_INFO:
		{
			struct p7ion_buffer_info data;
			struct ion_handle *handle;
			if (copy_from_user(&data, (void __user *)arg, sizeof(struct p7ion_buffer_info)))
				return -EFAULT;

			handle = ion_handle_get_by_id(client,(int)data.handle);
			ion_phys(client,handle,&data.phys_address,&data.len);
			ion_handle_put(handle);

			if(copy_to_user ((void __user *)arg, &data, sizeof(struct p7ion_buffer_info)))
				return -EFAULT;
		break;
		}
		default:
		return -ENOTTY;
	}
	return 0;
}

static int __devinit p7ion_probe(struct platform_device* pdev)
{
	struct ion_platform_data* const pdata = dev_get_platdata(&pdev->dev);
	int                             err;
	unsigned int                    h;

	BUG_ON(! pdata);
	BUG_ON(! pdata->nr);

	p7ion_heaps_nr = pdata->nr;
	p7ion_heaps = kcalloc(p7ion_heaps_nr, sizeof(p7ion_heaps[0]), GFP_KERNEL);
	if (! p7ion_heaps)
		return -ENOMEM;

	p7ion_dev = ion_device_create(p7ion_custom_ioctl);
	if (IS_ERR_OR_NULL(p7ion_dev)) {
		err = PTR_ERR(p7ion_dev);
		goto err;
	}

	for (h = 0; h < p7ion_heaps_nr; h++) {
		p7ion_heaps[h] = ion_heap_create(&pdata->heaps[h]);
		if (IS_ERR_OR_NULL(p7ion_heaps[h])) {
			err = PTR_ERR(p7ion_heaps[h]);
			goto free;
		}

		ion_device_add_heap(p7ion_dev, p7ion_heaps[h]);
	}

	return 0;

free:
	while (h--)
		ion_heap_destroy(p7ion_heaps[h]);
err:
	kfree(p7ion_heaps);
	return err;
}

static int __devexit p7ion_remove(struct platform_device* pdev)
{
	int h;

	for (h = 0; h < p7ion_heaps_nr; h++)
		ion_heap_destroy(p7ion_heaps[h]);

	ion_device_destroy(p7ion_dev);
	kfree(p7ion_heaps);

	return 0;
}

static struct platform_driver p7ion_driver = {
	.probe  = p7ion_probe,
	.remove = __devexit_p(p7ion_remove),
	.driver = {
		.name   = "p7-ion",
		.owner  = THIS_MODULE,
	}
};
module_platform_driver(p7ion_driver);
