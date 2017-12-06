/**
 * linux/driver/parrot/gpuo/p7ump.c - Parrot 7 Mali UMP API
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Alvaro Moran <alvaro.moran@parrot.com>
 * date:    2013-12-04
 *
 * This file is released under the GPL
 */


#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/semaphore.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <asm/signal.h>
#include "ump_kernel_interface_ref_drv.h"
#include "p7ump.h"

#define DRV_NAME	"p7ump"

/* This is to keep track of all mapped regions */
struct p7ump_map {
	struct list_head	list;
	u32			addr;
	size_t			size;
	atomic_t		cnt;
};

struct p7ump_record {
	struct list_head	list;
	u32			id;
	u32			addr;
	size_t			size;
	ump_dd_handle 		ump_h;
};

static LIST_HEAD( p7ump_maps );
static DEFINE_MUTEX(p7ump_lock);

int p7ump_add_map(u32 addr, size_t size)
{
	struct list_head *pos, *q;
	struct p7ump_map* map;

	if ((0 == addr) || (0 == size))
		return -EINVAL;

	/* Check if address is already present */
	mutex_lock(&p7ump_lock);
	list_for_each_safe(pos, q, &p7ump_maps) {
		map = list_entry(pos, struct p7ump_map, list);
		if ((map->addr == addr) && (map->size == size)) {
			atomic_inc(&map->cnt);
			mutex_unlock(&p7ump_lock);
			return 0;
		}
	}
	mutex_unlock(&p7ump_lock);

	/* Add new map */
	map = kmalloc(sizeof(*map), GFP_KERNEL);
	if (unlikely(! map)) {
		return -ENOMEM;
	}
	map->addr = addr;
	map->size = size;
	atomic_set(&map->cnt, 1);
	//printk(KERN_DEBUG "P7UMP: Adding map 0x%08x sz %d\n", addr, size);
	mutex_lock(&p7ump_lock);
	list_add(&map->list, &p7ump_maps);
	mutex_unlock(&p7ump_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(p7ump_add_map);

int p7ump_remove_map(u32 addr, size_t size)
{
	struct p7ump_map* map;
	struct list_head *pos, *q;
	int ret = -EFAULT;

	if (addr == 0 || size == 0)
		return -EINVAL;

	mutex_lock(&p7ump_lock);
	list_for_each_safe(pos, q, &p7ump_maps) {
		map = list_entry(pos, struct p7ump_map, list);
		if ((map->addr == addr) && (map->size == size) &&
		    atomic_dec_and_test(&map->cnt)) {
			//printk(KERN_DEBUG "P7UMP: Removing map 0x%08x sz %d\n",
			//	addr, size);
			list_del(pos);
			kfree(map);
			ret =  0;
			goto unlock_exit;
		}
	}

unlock_exit:
	mutex_unlock(&p7ump_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(p7ump_remove_map);

static struct p7ump_token* p7ump_record_get(struct list_head* rlist, 
		struct p7ump_token* token)
{
	struct p7ump_map *map;
	struct p7ump_record* record;
	ump_dd_physical_block ump_memory_description;
	ump_dd_handle ump_h;
	bool valid_map = false;
	u32 size = PAGE_ALIGN(token->size);
	u32 addr = token->bus;
	u32 end = token->bus + size;

	token->id = 0;

#ifdef CONFIG_UMP_PARROT7_AUTOLOAD
	if (p7ump_add_map(addr, size) < 0)
		return token;
#endif
	
	/* Look for a valid registered mapping */
	mutex_lock(&p7ump_lock);
	list_for_each_entry(map, &p7ump_maps, list) {
		if ((addr >= map->addr) && (end <= (map->addr + map->size))) {
			valid_map = true;
			break;
		}
	}
	mutex_unlock(&p7ump_lock);
	if (!valid_map) 
		goto failed;

	ump_memory_description.addr = token->bus;
	ump_memory_description.size = size;
	ump_h = 
		ump_dd_handle_create_from_phys_blocks(&ump_memory_description, 1);
	if (UMP_DD_HANDLE_INVALID == ump_h)
		goto failed;
	token->id = ump_dd_secure_id_get(ump_h);
	token->size = size;

	/* 
	 * If everything went fine then save token in a new record and 
	 * append to the list 
	 */
	record = kmalloc(sizeof(*record), GFP_KERNEL);
	if (unlikely(! record)) {
		goto failed;
	}	
	record->ump_h = ump_h;
	record->id = token->id;
	record->addr = addr;
	record->size = size;

	mutex_lock(&p7ump_lock);
	list_add(&record->list, rlist);
	mutex_unlock(&p7ump_lock);

	/* Success */
	return token;

/* Failed */
failed:
#ifdef CONFIG_UMP_PARROT7_AUTOLOAD
	p7ump_remove_map(addr, size);
#endif
	return token;
}

static int p7ump_record_put(struct list_head* rlist, int secure_id)
{
	int ret = 0;
	struct p7ump_record* record;
	struct list_head *pos, *q;
	ump_dd_handle ump_h;
	size_t size = 0;
	u32 addr = 0;

	ump_h = ump_dd_handle_create_from_secure_id(secure_id);
	if (ump_h == UMP_DD_HANDLE_INVALID) {
		/* secure id not found so likely a bad value */
		return -EINVAL;
	}

	/* now that we have the handle, we can decrease the reference again */
	ump_dd_reference_release(ump_h);

	mutex_lock(&p7ump_lock);
	list_for_each_safe(pos, q, rlist) {
		record = list_entry(pos, struct p7ump_record, list);
		if (record->ump_h == ump_h)
		{
			addr = record->addr;
			size = record->size;
			list_del(pos);
			ump_dd_reference_release(record->ump_h);
			kfree(record);
			break;
		}
	}
	mutex_unlock(&p7ump_lock);

#ifdef CONFIG_UMP_PARROT7_AUTOLOAD
	ret = p7ump_remove_map(addr, size);
#endif

	return ret;
}

static long p7ump_ioctl(struct file* filep, unsigned int cmd, unsigned long arg)
{
	struct list_head* rlist = filep->private_data;
	struct p7ump_token __user * ptoken;
	int __user * pid;
	struct p7ump_token token;
	int id;

	switch (cmd) {
	case P7UMP_GET_ID:
		ptoken = (struct p7ump_token __user *) arg;
		if (copy_from_user(&token, ptoken, sizeof(token)))
			return -EFAULT;
		p7ump_record_get(rlist, &token);
		if (0 == token.id)
			return -EINVAL;
		return copy_to_user( ptoken, &token, sizeof(token));
	case P7UMP_REL_ID:
		pid = (int __user *) arg;
		if (get_user(id, pid))
			return -EFAULT;
		return p7ump_record_put(rlist, id);
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int p7ump_release(struct inode* inode, struct file* filep)
{
	struct list_head* rlist = filep->private_data;
	struct p7ump_record* record;
	struct list_head *pos, *q;
#ifdef CONFIG_UMP_PARROT7_AUTOLOAD
	struct p7ump_map* map;
	struct list_head *mpos, *k;
#endif

	/* cleanup records and list */
	mutex_lock(&p7ump_lock);
	list_for_each_safe(pos, q, rlist) {
		record = list_entry(pos, struct p7ump_record, list);
		list_del(pos);

#ifdef CONFIG_UMP_PARROT7_AUTOLOAD
		list_for_each_safe(mpos, k, &p7ump_maps) {
			map = list_entry(mpos, struct p7ump_map, list);
			if ((map->addr == record->addr) &&
			    (map->size == record->size) &&
			     atomic_dec_and_test(&map->cnt)) {
				list_del(mpos);
				kfree(map);
				break;
			}
		}
#endif

		ump_dd_reference_release(record->ump_h);
		kfree(record);
	}
	mutex_unlock(&p7ump_lock);

	kfree(rlist);
	return 0;
}

static int p7ump_open(struct inode* inode, struct file* filep)
{
	static struct list_head* rlist;

	rlist = kmalloc(sizeof(*rlist), GFP_KERNEL);
	if (unlikely(! rlist)) {
		return -ENOMEM;
	}
	/* Init record list head */
	INIT_LIST_HEAD(rlist);

	filep->private_data = rlist;
	return 0;
}

static struct file_operations const p7ump_fops = {
	.owner			= THIS_MODULE,
	.open			= p7ump_open,
	.release		= p7ump_release,
	.unlocked_ioctl 	= p7ump_ioctl,
};

static struct miscdevice p7ump_dev = 
{
	MISC_DYNAMIC_MINOR,
	DRV_NAME,
	&p7ump_fops
};

static int __init p7ump_init(void)
{
	return misc_register(&p7ump_dev);
}

static void __exit p7ump_exit(void)
{
	misc_deregister(&p7ump_dev);
}

module_init(p7ump_init);
module_exit(p7ump_exit);


MODULE_AUTHOR("Alvaro Moran");
MODULE_LICENSE("GPL");

