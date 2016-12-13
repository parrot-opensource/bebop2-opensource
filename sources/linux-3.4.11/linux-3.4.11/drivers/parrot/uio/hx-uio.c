/**
 * linux/driver/parrot/uio/hx-uio.c - On2 / Hantro user I/O
 *                                    implementation
 *
 * Copyright (C) 2011 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    30-Aug-2011
 *
 * This file is released under the GPL
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/ratelimit.h>
#include <linux/poll.h>
#include <asm/signal.h>
#include <linux/semaphore.h>
#include <asm/uaccess.h>
#include "hx-uio.h"
#if defined(CONFIG_UMP_PARROT7) || defined(CONFIG_UMP_PARROT7_MODULE)
#include "gpu/p7ump.h"
#endif

#define DRV_NAME    "hxuio"
#define HX_DEVS_NR  2

struct hx_file {
	struct hx_device*   hxdev;
	struct hx_unit*		notifier;
	unsigned int        irqcnt;
};

struct hx_vmdma {
	void*           virt;
	dma_addr_t      bus;
	atomic_t        cnt;
};

#define HX_DMA_PGNO_MAGIC PFN_DOWN(~(0UL))



static int                  hx_major;
static struct class*        hx_class;
static struct hx_device*    hx_devices[HX_DEVS_NR];
static DEFINE_MUTEX(        hx_lock);
static int dma_memory_used;


static void hx_dma_mem_stat_add(struct hx_device* hxdev, int size)
{
	int pid = current->pid;
	struct device* const    parent = hxdev->device.parent;

	dma_memory_used += size;
	dev_dbg(parent, "dma allocated %d pid %d (%s) total %d\n", size, pid,
			current->comm, dma_memory_used);
}

static void hx_dma_mem_stat_free_lock(struct hx_device* hxdev, int size)
{
	struct device* const    parent = hxdev->device.parent;
	struct mutex* const     lck = &hxdev->vm_lock;

	mutex_lock(lck);
	dma_memory_used -= size;
	mutex_unlock(lck);
	dev_dbg(parent, "dma free %d total %d\n", size, dma_memory_used);
}

static void hx_open_vmdma(struct vm_area_struct* vma)
{
	struct hx_vmdma* const  dma = (struct hx_vmdma*) vma->vm_private_data;
	struct device* const    parent = ((struct hx_file*) vma->vm_file->private_data)->hxdev->device.parent;

	dev_dbg(parent,
	        "open vmdma %p [vma=%08lx-%08lx,dmavirt=%08lx, dmabus=%08lx] cnt=%d\n",
	         vma,
	         vma->vm_start,
	         vma->vm_end,
	         (unsigned long) dma->virt,
	         (unsigned long) dma->bus,
	         atomic_read(&dma->cnt));

	atomic_inc(&dma->cnt);
}

static void hx_close_vmdma(struct vm_area_struct* vma)
{
	struct hx_vmdma* const  dma = (struct hx_vmdma*) vma->vm_private_data;
	struct device* const    parent = ((struct hx_file*) vma->vm_file->private_data)->hxdev->device.parent;
	struct hx_device* const     hxdev = ((struct hx_file*) vma->vm_file->private_data)->hxdev;

	dev_dbg(parent,
	        "close vmdma %p [vma=%08lx-%08lx,dmavirt=%08lx, dmabus=%08lx] cnt=%d\n",
	        vma,
	        vma->vm_start,
	        vma->vm_end,
	        (unsigned long) dma->virt,
	        (unsigned long) dma->bus,
	        atomic_read(&dma->cnt));

	if (atomic_dec_and_test(&dma->cnt)) {
#if defined(CONFIG_UMP_PARROT7) || defined(CONFIG_UMP_PARROT7_MODULE)
		p7ump_remove_map(dma->bus,
				 PAGE_ALIGN(vma->vm_end - vma->vm_start));
#endif
		dev_dbg(parent,"Freeing hx-uio memory !\n");
		hx_dma_mem_stat_free_lock(hxdev, PAGE_ALIGN(vma->vm_end - vma->vm_start));
		dma_free_coherent(parent,
		                  PAGE_ALIGN(vma->vm_end - vma->vm_start),
		                  dma->virt,
		                  dma->bus);

		kfree(dma);
	}
}

static int hx_fault_vmdma(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	/* Disable mremap. */
	return VM_FAULT_SIGBUS;
}

static struct vm_operations_struct const hx_vm_ops = {
	.open   = &hx_open_vmdma,
	.close  = &hx_close_vmdma,
	.fault  = &hx_fault_vmdma
};

static int hx_mmap_dma(struct vm_area_struct* vma,
                       struct hx_device* hxdev,
                       size_t size)
{
	struct device* const    parent = hxdev->device.parent;
	struct mutex* const     lck = &hxdev->vm_lock;
	struct hx_vmdma        *dma = (struct hx_vmdma*) vma->vm_private_data;
	int                     err = -ENOMEM;

	if (! (vma->vm_flags & VM_WRITE))
		return -EPERM;

	mutex_lock(lck);

	if (!dma) {
		dma = kmalloc(sizeof(*dma), GFP_KERNEL);
		if (! dma) {
			mutex_unlock(lck);
			return err;
		}
		dev_dbg(parent,"Allocating hx-uio memory !\n");

		dma->virt = dma_alloc_coherent(parent,
					       PAGE_ALIGN(size),
					       &dma->bus,
					       GFP_KERNEL);

		if (! dma->virt) {
			dev_warn(parent, "can't allocate dma memory %d\n", PAGE_ALIGN(size));
			goto free;
		}

		vma->vm_flags |= VM_DONTCOPY;
		vma->vm_private_data = dma;

		atomic_set(&dma->cnt, 0);
	}

	err = dma_mmap_coherent(parent, vma, dma->virt, dma->bus, size);
	if (err)
		goto dma;

	atomic_inc(&dma->cnt);

	vma->vm_ops = &hx_vm_ops;

	*((dma_addr_t*) (dma->virt)) = dma->bus;

#if defined(CONFIG_UMP_PARROT7) || defined(CONFIG_UMP_PARROT7_MODULE)
	/* Allow GPU to access this memory through UMP */
	p7ump_add_map(dma->bus, size);
#endif

	hx_dma_mem_stat_add(hxdev, PAGE_ALIGN(size));


	mutex_unlock(lck);
	return 0;

dma:
	dma_free_coherent(parent,
	                  PAGE_ALIGN(size),
	                  dma->virt,
	                  dma->bus);
free:
	mutex_unlock(lck);
	kfree(dma);
	return err;
}

static int hx_mmap(struct file* filep, struct vm_area_struct* vma)
{
	struct hx_device* const     hxdev = ((struct hx_file*) filep->private_data)->hxdev;
	unsigned long const         pageno = vma->vm_pgoff;
	size_t const                sz = vma->vm_end - vma->vm_start;
	int                         err;

	if (! (vma->vm_flags & VM_SHARED))
		return -EINVAL;

	if (vma->vm_end <= vma->vm_start) {
		dev_dbg(hxdev->device.parent,
		        "invalid vma region: 0x%08lx-0x%08lx.\n",
		        vma->vm_start,
		        vma->vm_end);
		return -EINVAL;
	}

	if (pageno && pageno != HX_DMA_PGNO_MAGIC) {
		dev_dbg(hxdev->device.parent,
		        "invalid vma offset page %lu.\n",
		        pageno);
		return -EINVAL;
	}

	vma->vm_flags &= ~(VM_MAYEXEC | VM_EXEC);
	vma->vm_flags |= VM_SPECIAL;

	if (unlikely(! pageno)) {
		struct resource const* const res = hxdev->regs_res;

		if (sz > resource_size(res)) {
			dev_dbg(hxdev->device.parent,
					"invalid vma size %u (limited to %u).\n",
					(unsigned int) sz,
					(unsigned int) resource_size(res));
			return -EINVAL;
		}

		vma->vm_page_prot = pgprot_noncached(vm_get_page_prot(vma->vm_flags));
		err = remap_pfn_range(vma,
		                      vma->vm_start,
		                      PFN_DOWN(res->start),
		                      sz,
		                      vma->vm_page_prot);
	}
	else
		err = hx_mmap_dma(vma, hxdev, sz);

	if (err)
		dev_dbg(hxdev->device.parent,
		        "failed to remap range with error %d.\n", err);
	return err;
}

static struct hx_device* hx_get_dev(int minor)
{
	int                 d;
	struct hx_device*   hxdev = 0;

	mutex_lock(&hx_lock);

	for (d = 0; d < ARRAY_SIZE(hx_devices); d++) {
		struct hx_device* const dev = hx_devices[d];

		if (dev &&
			MINOR(dev->device.devt) == minor) {
			hxdev = dev;
			break;
		}
	}
	if (! hxdev)
		goto unlock;

	__module_get(hxdev->owner);

unlock:
	mutex_unlock(&hx_lock);
	return hxdev;
}

static void hx_put_dev(struct hx_device const* hxdev)
{
	module_put(hxdev->owner);
}

static int hx_register_dev(struct hx_device* hxdev, char const* name, int id)
{
	int err;
	int d;

	mutex_lock(&hx_lock);

	for (d = 0; d < ARRAY_SIZE(hx_devices); d++) {
		if (! hx_devices[d])
			break;
	}
	if (d == ARRAY_SIZE(hx_devices)) {
		err = -ENXIO;
		goto unlock;
	}

	hxdev->device.devt = MKDEV(hx_major, d);
	err = dev_set_name(&hxdev->device, "%s%c", name, id + 'a');
	if (err)
		goto unlock;

	err = device_register(&hxdev->device);
	if (err)
		goto free;

	hx_devices[d] = hxdev;

	mutex_unlock(&hx_lock);
	return 0;

free:
	kfree(dev_name(&hxdev->device));
unlock:
	mutex_unlock(&hx_lock);
	return err;
}

static int hx_unregister_dev(struct hx_device* hxdev)
{
	int d;
	int err = 0;

	mutex_lock(&hx_lock);

	for (d = 0; d < ARRAY_SIZE(hx_devices); d++) {
		if (hxdev == hx_devices[d])
			break;
	}
	if (d == ARRAY_SIZE(hx_devices)) {
		err = -ENXIO;
		goto unlock;
	}

	device_unregister(&hx_devices[d]->device);
	hx_devices[d] = 0;

unlock:
	mutex_unlock(&hx_lock);
	return err;
}

static int hx_init_maps(struct hx_device* hxdev,
                        struct platform_device* pdev)
{
	struct device* const            parent = &pdev->dev;
	struct resource const* const    regs_res = platform_get_resource(pdev,
	                                                                 IORESOURCE_MEM,
	                                                                 0);
	char const*                     msg;
	int                             err = -EBUSY;

	hxdev->regs_res = NULL;
	if (!regs_res) {
		msg = "no registers";
		goto err;
	}

	if (! request_mem_region(regs_res->start,
	                         resource_size(regs_res),
	                         dev_name(parent))) {
		msg = "failed to reserve registers";
		goto err;
	}

	hxdev->regs_virt = (unsigned long) ioremap(regs_res->start,
	                                           resource_size(regs_res));
	if (hxdev->regs_virt) {
		hxdev->regs_res = regs_res;
		return 0;
	}

	msg = "failed to remap registers";
	release_mem_region(regs_res->start, resource_size(regs_res));

err:
	dev_dbg(parent, "%s region.\n", msg);
	dev_err(parent, "failed to setup memory mappings (%d).\n", err);
	return err;
}

static void hx_exit_maps(struct hx_device const* hxdev)
{
	if (! hxdev->regs_res)
		return;

	iounmap((void*) hxdev->regs_virt);
	release_mem_region(hxdev->regs_res->start,
					   resource_size(hxdev->regs_res));
}

static irqreturn_t hx_handle_irq(int irq, void* dev_id)
{
	struct hx_device const* const   hxdev = (struct hx_device*) dev_id;
	int                             irqs = 0;
	int                             n;

	for (n = 0; n < hxdev->unit_cnt; n++) {
		struct hx_unit* const	unit = &hxdev->units[n];
		int const               happened = unit->handle_irq(hxdev);

		if (! happened)
			continue;

		irqs += happened;
		if (! unit->refcnt)
			continue;

		unit->irqcnt++;
		wake_up_interruptible(&unit->waitq);
		kill_fasync(&unit->asyncq, SIGIO, POLL_IN);
	}

	if (likely(irqs))
		return IRQ_HANDLED;

	return IRQ_NONE;
}

static void hx_release_dev(struct device* device)
{
}

void hx_init_unit(struct hx_unit* unit,
                  int (*handle_irq)(struct hx_device const*),
                  void (*setup_irq)(struct hx_device const*, int))
{
	unit->refcnt = 0;
	unit->irqcnt = 0;
	unit->handle_irq = handle_irq;
	sema_init(&unit->sem, 1);
	unit->owner = NULL;
	init_waitqueue_head(&unit->waitq);
	unit->asyncq = 0;
	unit->setup_irq = setup_irq;
}
EXPORT_SYMBOL(hx_init_unit);

int _hx_probe(struct platform_device* pdev,
              struct module* owner,
              char const* name,
              struct hx_unit* units,
              size_t unit_count,
              int (*init_hw)(struct hx_device const*))
{
	struct device* const    parent = &pdev->dev;
	struct clk* const       clk = clk_get(&pdev->dev, NULL);
	struct hx_device*       hxdev;
	int                     err;

	if (IS_ERR(clk)) {
		dev_dbg(parent, "failed to get clock.\n");
		err = PTR_ERR(clk);
		goto err;
	}

	hxdev = kzalloc(sizeof(*hxdev), GFP_KERNEL);
	if (! hxdev) {
		dev_dbg(parent, "failed to allocate device.\n");
		err = -ENOMEM;
		goto put;
	}

	hxdev->irq = platform_get_irq(pdev, 0);
	BUG_ON(hxdev->irq < 0);
	err = hx_init_maps(hxdev, pdev);
	if (err)
		goto free;

	err = clk_prepare_enable(clk);
	if (err) {
		dev_dbg(parent, "failed to enable clock.\n");
		goto exit;
	}

	spin_lock_init(&hxdev->unit_lock);
	hxdev->unit_cnt = unit_count;
	hxdev->units = units;
	mutex_init(&hxdev->vm_lock);
	hxdev->device.class = hx_class;
	hxdev->device.parent = parent;
	hxdev->device.release = &hx_release_dev;
	hxdev->owner = owner;

	/* Give top-level driver a chance to perform specific initializations. */
	if (init_hw) {
		err = (*init_hw)(hxdev);
		if (err)
			goto exit;
	}

	err = request_irq(hxdev->irq,
	                  &hx_handle_irq,
	                  0,
	                  dev_name(parent),
	                  hxdev);
	if (err) {
		dev_dbg(parent, "failed to reserve irq %d.\n", hxdev->irq);
		goto clk;
	}

	err = hx_register_dev(hxdev, name, pdev->id);
	if (! err) {
		clk_put(clk);
		parent->platform_data = hxdev;
		return 0;
	}

clk:
	clk_disable_unprepare(clk);
exit:
	hx_exit_maps(hxdev);
free:
	kfree(hxdev);
put:
	clk_put(clk);
err:
	return err;
}
EXPORT_SYMBOL(_hx_probe);

int hx_remove(struct platform_device* pdev)
{
	struct device* const    parent = &pdev->dev;
	struct hx_device* const hxdev = (struct hx_device*) dev_get_platdata(parent);
	struct clk* const       clk = clk_get(parent, NULL);
	int                     err = 0;

	if (IS_ERR(clk)) {
		dev_dbg(parent, "failed to get clock.\n");
		return PTR_ERR(clk);
	}

	err = hx_unregister_dev(hxdev);
	if (err) {
		dev_dbg(parent, "failed to unregister device.\n");
		goto put;
	}

	clk_disable_unprepare(clk);
	free_irq(hxdev->irq, hxdev);
	hx_exit_maps(hxdev);
	kfree(hxdev);

put:
	clk_put(clk);
	if (err)
		dev_err(parent, "failed to remove device (%d).\n", err);
	return err;
}
EXPORT_SYMBOL(hx_remove);

int hx_suspend(struct platform_device* pdev)
{
	struct device* const    parent = &pdev->dev;
	struct clk* const       clk = clk_get(parent, NULL);

	clk_disable_unprepare(clk);
	return 0;
}
EXPORT_SYMBOL(hx_suspend);

int hx_resume(struct platform_device* pdev,
		int (*init_hw)(struct hx_device const*))
{
	struct device* const    parent = &pdev->dev;
	struct hx_device* const hxdev = (struct hx_device*) dev_get_platdata(parent);
	struct clk* const       clk = clk_get(parent, NULL);
	int ret;

	ret = clk_prepare_enable(clk);
	if (!ret && init_hw) {
		ret = (*init_hw)(hxdev);
	}
	return ret;
}
EXPORT_SYMBOL(hx_resume);

static unsigned int hx_enable_notif(struct hx_unit* notifier,
                                    struct hx_device const* hxdev)
{
	if (! notifier->refcnt)
		notifier->setup_irq(hxdev, 1);
	notifier->refcnt++;

	return notifier->irqcnt;
}


static void hx_disable_notif(struct hx_unit* notifier,
                             struct hx_device const* hxdev)
{
	if (! (--notifier->refcnt))
		notifier->setup_irq(hxdev, 0);
}

static long hx_ioctl(struct file* filep, unsigned int cmd, unsigned long arg)
{
	int                     err;
	struct hx_unit*			unit = NULL;
	struct hx_file*    const priv = (struct hx_file*) filep->private_data;
	struct hx_device* const hxdev = priv->hxdev;

	if (unlikely(arg >= hxdev->unit_cnt)) {
		/* Fix your userland driver !! */
		static DEFINE_RATELIMIT_STATE(rate,
		                              DEFAULT_RATELIMIT_INTERVAL,
		                              DEFAULT_RATELIMIT_BURST);

		if (__ratelimit(&rate))
			dev_warn(hxdev->device.parent,
			         "unknown (un)lock command %lu request from %16s[%5d].\n",
			         arg,
			         current->comm,
			         task_pid_nr(current));

		return -EINVAL;
	}

	unit = &hxdev->units[arg];

	switch (cmd) {
	case HX_LOCK :
		if (unit->owner == filep->private_data)
			return -EPERM;

		err = down_interruptible(&unit->sem);
		if (err == -EINTR)
			return -ERESTARTSYS;

		unit->owner = filep->private_data;

		/* Userspace library can reserve both decoder and post-processor
		 * in pipelined mode. Since we only enable 'notification' for
		 * one unit (decoder or pp), we shall only update private file
		 * irqcnt field if unit we try to lock match the one notified.
		 */
		if (priv->notifier == unit)
			priv->irqcnt = unit->irqcnt;
		break;

	case HX_UNLOCK :
		if (unit->owner != filep->private_data)
			return -EPERM;

		unit->owner = NULL;
		up(&unit->sem);
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static ssize_t hx_write(struct file* filep,
                        char const __user* buf,
                        size_t count,
                        loff_t* ppos)
{
	int                         err;
	unsigned int                command;
	struct hx_file* const       priv = filep->private_data;
	struct hx_device* const     hxdev = priv->hxdev;
	struct hx_unit*				notif = NULL;

	if (count != sizeof(command))
		return -EINVAL;

	err = get_user(command, (unsigned int __user*) buf);
	if (err)
		return err;

	if (unlikely(command > hxdev->unit_cnt)) {
		/* Fix your userland driver !! */
		static DEFINE_RATELIMIT_STATE(rate,
		                              DEFAULT_RATELIMIT_INTERVAL,
		                              DEFAULT_RATELIMIT_BURST);

		if (__ratelimit(&rate))
			dev_warn(hxdev->device.parent,
			         "unknown interrupt setup command %d request "
			         "from %16s[%5d].\n",
			         command,
			         current->comm,
			         task_pid_nr(current));

		return -EIO;
	}

	if (command) {
		notif = &hxdev->units[command - 1];
		if (priv->notifier != notif) {
			spin_lock_irq(&hxdev->unit_lock);
			if (priv->notifier)
				hx_disable_notif(priv->notifier, hxdev);
			priv->irqcnt = hx_enable_notif(notif, hxdev);
			spin_unlock_irq(&hxdev->unit_lock);
		}
	}
	else {
		if (priv->notifier) {
			spin_lock_irq(&hxdev->unit_lock);
			hx_disable_notif(priv->notifier, hxdev);
			spin_unlock_irq(&hxdev->unit_lock);
		}
	}

	priv->notifier = notif;
	return sizeof(command);
}

static inline int hx_saw_irqs(unsigned int priv_count,
                              unsigned int volatile const* notif_count,
                              unsigned int* irq_count,
                              spinlock_t* lock)

{
	unsigned int	seen;

	spin_lock_irq(lock);
	*irq_count = *notif_count;
	seen = (*irq_count != priv_count);
	spin_unlock_irq(lock);

	return seen;
}

static ssize_t hx_read(struct file* filep,
                       char __user* buf,
                       size_t count,
                       loff_t* ppos)
{
	struct hx_file* const   priv = filep->private_data;
	struct hx_unit* const	notif = priv->notifier;
	unsigned int            irqs;
	int                     err;

	if (count != sizeof(irqs))
		return -EINVAL;

	if (! notif)
		return -EIO;

	if (! (filep->f_flags & O_NONBLOCK)) {
		switch (wait_event_interruptible_timeout(notif->waitq,
		                                         hx_saw_irqs(priv->irqcnt,
		                                                     &notif->irqcnt,
		                                                     &irqs,
		                                                     &priv->hxdev->unit_lock),
		                                         HZ / 2)) {
		case -ERESTARTSYS:
			return -ERESTARTSYS;
		case 0:
			return -ETIMEDOUT;
		}
	}
	else {
		if (! hx_saw_irqs(priv->irqcnt,
		                  &notif->irqcnt,
		                  &irqs,
		                  &priv->hxdev->unit_lock))
			return -EAGAIN;
	}

	err = put_user(irqs, (unsigned int __user*) buf);
	if (err)
		return err;

	priv->irqcnt = irqs;
	return sizeof(irqs);
}

static int hx_release(struct inode* inode, struct file* filep)
{
	struct hx_file* const   priv = filep->private_data;
	struct hx_device* const hxdev = priv->hxdev;
	unsigned int			no;

	for (no = 0; no < hxdev->unit_cnt; no++) {
		struct hx_unit* const	unit = &hxdev->units[no];

		if (unit->owner == priv) {
			/* We own the lock: release it. */
			unit->owner = NULL;
			up(&unit->sem);
		}
	}

	if (priv->notifier) {
		spin_lock_irq(&hxdev->unit_lock);
		hx_disable_notif(priv->notifier, hxdev);
		spin_unlock_irq(&hxdev->unit_lock);
	}

	hx_put_dev(hxdev);
	kfree(priv);
	dev_dbg(hxdev->device.parent, "device closed.\n");
	return 0;
}

static int hx_open(struct inode* inode, struct file* filep)
{
	struct hx_device* const hxdev = hx_get_dev(iminor(inode));
	struct hx_file*         priv;

	if (! hxdev)
		return -ENODEV;

	priv = kmalloc(sizeof(*priv), GFP_KERNEL);
	if (unlikely(! priv)) {
		hx_put_dev(hxdev);
		return -ENOMEM;
	}

	priv->hxdev = hxdev;
	priv->notifier = NULL;
	priv->irqcnt = 0;
	filep->private_data = priv;

	dev_dbg(hxdev->device.parent, "device opened.\n");
	return 0;
}

static unsigned int hx_poll(struct file* filep, poll_table* wait)
{
	struct hx_file* const   priv = filep->private_data;
	spinlock_t* const       lck = &priv->hxdev->unit_lock;
	struct hx_unit* const	notif = priv->notifier;
	unsigned int            irqcnt;

	if (! notif)
		return POLLERR;

	poll_wait(filep, &notif->waitq, wait);

	spin_lock_irq(lck);
	irqcnt = notif->irqcnt;
	spin_unlock_irq(lck);

	if (priv->irqcnt != irqcnt)
		return POLLIN | POLLRDNORM;

	return 0;
}

static int hx_fasync(int fd, struct file* filep, int on)
{
	struct hx_unit* const	notif = ((struct hx_file*)
									 filep->private_data)->notifier;

	if (! notif)
		return -EIO;

	return fasync_helper(fd, filep, on, &notif->asyncq);
}

static struct file_operations const hx_fops = {
	.owner			= THIS_MODULE,
	.open			= hx_open,
	.release		= hx_release,
	.read			= hx_read,
	.write			= hx_write,
	.mmap			= hx_mmap,
	.poll			= hx_poll,
	.fasync			= hx_fasync,
	.unlocked_ioctl = hx_ioctl,
};

static int __init hx_init(void)
{
	hx_major = register_chrdev(0, DRV_NAME, &hx_fops);
	if (hx_major < 0)
		return hx_major;

	hx_class = class_create(THIS_MODULE, DRV_NAME);
	if (! IS_ERR(hx_class))
		return 0;

	unregister_chrdev(hx_major, DRV_NAME);
	return PTR_ERR(hx_class);
}

static void __exit hx_exit(void)
{
	class_destroy(hx_class);
	unregister_chrdev(hx_major, DRV_NAME);
}

module_init(hx_init);
module_exit(hx_exit);

MODULE_AUTHOR("Gregor Boirie");
MODULE_DESCRIPTION("On2 / Hantro video userspace I/O platform driver");
MODULE_LICENSE("GPL");
