/**
 * linux/driver/parrot/uio/hx-uio.h - On2 / Hantro user I/O
 *                                    interface
 *
 * Copyright (C) 2011 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    30-Aug-2011
 *
 * This file is released under the GPL
 */

#ifndef _HX_UIO_H
#define _HX_UIO_H

#include <linux/irqreturn.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/ioctl.h>
#include <linux/semaphore.h>

#define HX_IO_MAGIC 'k'

#define HX_LOCK _IO(HX_IO_MAGIC, 1)
#define HX_UNLOCK _IO(HX_IO_MAGIC, 2)

struct hx_device;
struct platform_device;

struct hx_unit {
	unsigned int            refcnt;
	unsigned int            irqcnt;
	int                     (*handle_irq)(struct hx_device const*);
	struct semaphore        sem;
	void const*				owner;
	wait_queue_head_t       waitq;
	struct fasync_struct*   asyncq;
	void                    (*setup_irq)(struct hx_device const*, int);
};

struct hx_device {
	spinlock_t                  unit_lock;
	size_t                      unit_cnt;
	struct hx_unit*				units;
	struct mutex                vm_lock;
	unsigned long               regs_virt;
	struct resource const*      regs_res;
	struct device               device;
	struct module*              owner;
	int                         irq;
};

extern void hx_init_unit(struct hx_unit*,
						 int (*)(struct hx_device const*),
						 void (*)(struct hx_device const*, int));

extern int _hx_probe(struct platform_device*,
					 struct module*,
					 char const* name,
					 struct hx_unit*,
					 size_t,
					 int (*)(struct hx_device const*));

#define hx_probe(_pdev, _name, _unit, _cnt, _init) \
    _hx_probe(_pdev, THIS_MODULE, _name, _unit, _cnt, _init)

int hx_remove(struct platform_device*);

int hx_suspend(struct platform_device* pdev);
int hx_resume(struct platform_device* pdev,
		int (*init_hw)(struct hx_device const*));

#endif
