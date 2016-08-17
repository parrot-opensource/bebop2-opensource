/**
 * linux/driver/parrot/block/phy-brd.c - Physical Block RamDisk implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Guangye Tian <guangye.tian@parrot.com>
 * date:    26-Jan-2011
 *
 * This file is released under the GPL
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/major.h>
#include <linux/blkdev.h>
#include <linux/bio.h>
#include <linux/highmem.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <mach/memory.h>
#include "phy-brd.h"

#define KERNEL_SECTOR_SIZE	512

static int brd_phy_major = 0;
static int hardsect_size = 512;

/* Reserve last 128MB of system memory as ram disk */
static unsigned long brd_ram_start;
static unsigned int brd_ram_size;	/* ram block size in byte */

struct brd_phy_device {
	int 			size;		/* Device size in bytes */
	void __iomem		*data;
	struct resource		*res;
	struct request_queue	*queue;
	struct gendisk		*gd;
	spinlock_t		lock;
};

static struct brd_phy_device *brd_phy_dev = NULL;

/*
 * Handle an I/O request
 */
static void brd_phy_transfer(struct brd_phy_device *dev, unsigned long sector,
			unsigned long nsect, char *buffer, int write)
{
	unsigned long offset = sector * KERNEL_SECTOR_SIZE;
	unsigned long nbytes = nsect * KERNEL_SECTOR_SIZE;

	if ((offset + nbytes) > dev->size) {
		printk(KERN_NOTICE "Beyond-end write (%ld %ld)\n", offset, nbytes);
		return;
	}

    if (write)
        memcpy(dev->data + offset, buffer, nbytes);
    else
        memcpy(buffer, dev->data + offset, nbytes);
}

/*
 * Transfer a single BIO
 */
static int brd_phy_xfer_bio(struct brd_phy_device *dev, struct bio *bio)
{
	int i;
	struct bio_vec *bvec;
	sector_t sector = bio->bi_sector;

	bio_for_each_segment(bvec, bio, i) {
		char *buffer = __bio_kmap_atomic(bio, i, KM_USER0);
		brd_phy_transfer(dev, sector, bio_cur_bytes(bio) >> 9,
				buffer, bio_data_dir(bio) == WRITE);
		sector += bio_cur_bytes(bio) >> 9;
		__bio_kunmap_atomic(bio, KM_USER0);
	}
	return 0;
}

static void brd_phy_make_request(struct request_queue *q, struct bio *bio)
{
	struct brd_phy_device *dev = q->queuedata;
	int status;

	status = brd_phy_xfer_bio(dev, bio);

	bio_endio(bio, status);
}

int (*getgeo)(struct block_device *, struct hd_geometry *);

int brd_phy_getgeo(struct block_device *dev, struct hd_geometry *geo)
{
	return 0;
}

static const struct block_device_operations brd_phy_fops = {
	.owner =	THIS_MODULE,
	.getgeo =	brd_phy_getgeo
};

static int __devinit brd_phy_probe(struct platform_device *pdev)
{
	int err = 0;
	struct resource* const res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL)
		return -ENOENT;

	brd_phy_major = register_blkdev(brd_phy_major, PHYBRD_DRV_NAME);
	if (brd_phy_major < 0)
		return brd_phy_major;

	brd_phy_dev = kzalloc(sizeof(struct brd_phy_device), GFP_KERNEL);
	if (brd_phy_dev == NULL) {
		err = -ENOMEM;
		goto out_unregister;
	}

	brd_ram_start = res->start;
	brd_ram_size = resource_size(res);
	brd_phy_dev->res = request_mem_region(brd_ram_start, brd_ram_size, PHYBRD_DRV_NAME);
	if (brd_phy_dev->res == NULL) {
		err = -EBUSY;
		goto no_mem_region;
	}

	brd_phy_dev->data = ioremap_wc(brd_ram_start, brd_ram_size);
	if (brd_phy_dev->data == NULL) {
		err = -EIO;
		goto no_ioremap;
	}

	brd_phy_dev->size = brd_ram_size;

	spin_lock_init(&brd_phy_dev->lock);

	brd_phy_dev->queue = blk_alloc_queue(GFP_KERNEL);
	if (brd_phy_dev->queue == NULL) {
		err = -ENOMEM;
		goto no_request_queue;
	}
	blk_queue_make_request(brd_phy_dev->queue, brd_phy_make_request);

	blk_queue_physical_block_size(brd_phy_dev->queue, hardsect_size);

	brd_phy_dev->queue->queuedata = brd_phy_dev;

	brd_phy_dev->gd = alloc_disk(1);
	if (brd_phy_dev->gd == NULL) {
		err = -ENOMEM;
		goto no_request_queue;
	}

	brd_phy_dev->gd->major = brd_phy_major;
	brd_phy_dev->gd->first_minor = 0;
	brd_phy_dev->gd->fops = &brd_phy_fops;
	brd_phy_dev->gd->queue = brd_phy_dev->queue;
	brd_phy_dev->gd->private_data = brd_phy_dev;
	snprintf(brd_phy_dev->gd->disk_name, sizeof("phyram"), "phyram");
	set_capacity(brd_phy_dev->gd, brd_ram_size/KERNEL_SECTOR_SIZE);
	add_disk(brd_phy_dev->gd);

	dev_info(&pdev->dev, "loaded\n");
	return 0;

no_request_queue:
	iounmap(brd_phy_dev->data);
no_ioremap:
	release_mem_region(brd_phy_dev->res->start, resource_size(brd_phy_dev->res));
no_mem_region:
	kfree(brd_phy_dev);
out_unregister:
	unregister_blkdev(brd_phy_major, PHYBRD_DRV_NAME);
	return err;
}

static int __devexit brd_phy_remove(struct platform_device *pdev)
{
	if (brd_phy_dev->gd) {
		del_gendisk(brd_phy_dev->gd);
	}

	if (brd_phy_dev->queue) {
		blk_cleanup_queue(brd_phy_dev->queue);
	}

	iounmap(brd_phy_dev->data);
	release_resource(brd_phy_dev->res);
	unregister_blkdev(brd_phy_major, PHYBRD_DRV_NAME);
	kfree(brd_phy_dev);
	return 0;
}

static struct platform_driver brd_phy_driver = {
	.probe		= brd_phy_probe,
	.remove		= __devexit_p(brd_phy_remove),
	.driver		= {
		.name	= PHYBRD_DRV_NAME,
		.owner  = THIS_MODULE,
	},
};
module_platform_driver(brd_phy_driver);

MODULE_AUTHOR("Guangye Tian <guangye.tian@parrot.com>");
MODULE_DESCRIPTION("Physical block ramdisk driver");
MODULE_LICENSE("GPL");
