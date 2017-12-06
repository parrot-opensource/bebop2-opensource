/*
 * Copyright (C) 2010-2014 ARM Limited. All rights reserved.
 * 
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 * 
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/**
 * @file mali_osk_wq.c
 * Implementation of the OS abstraction layer for the kernel device driver
 */

#include <linux/slab.h> /* For memory allocation */
#include <linux/workqueue.h>
#include <linux/version.h>
#include <linux/sched.h>
#include <linux/kthread.h>

#include "mali_osk.h"
#include "mali_kernel_common.h"
#include "mali_kernel_license.h"
#include "mali_kernel_linux.h"

#ifndef MALI_LICENSE_IS_GPL
#undef CONFIG_MALI400_RT_WORKER
#endif

typedef struct _mali_osk_wq_work_s {
	_mali_osk_wq_work_handler_t handler;
	void *data;
	mali_bool high_pri;
#ifdef CONFIG_MALI400_RT_WORKER
	struct kthread_work kwork;
#else
	struct work_struct work_handle;
#endif
} mali_osk_wq_work_object_t;

typedef struct _mali_osk_wq_delayed_work_s {
	_mali_osk_wq_work_handler_t handler;
	void *data;
	struct delayed_work work;
} mali_osk_wq_delayed_work_object_t;

#if MALI_LICENSE_IS_GPL
static struct workqueue_struct *mali_wq_normal = NULL;
static struct workqueue_struct *mali_wq_high = NULL;

#ifdef CONFIG_MALI400_RT_WORKER
struct task_struct *mali_rt_worker_task = NULL;
struct kthread_worker mali_rt_worker;
#endif
#endif

#ifdef CONFIG_MALI400_RT_WORKER
static void mali_rt_worker_callback(struct kthread_work *kwork);
#else
static void _mali_osk_wq_work_func(struct work_struct *work);
#endif

_mali_osk_errcode_t _mali_osk_wq_init(void)
{
#if MALI_LICENSE_IS_GPL
	MALI_DEBUG_ASSERT(NULL == mali_wq_normal);
	MALI_DEBUG_ASSERT(NULL == mali_wq_high);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
	mali_wq_normal = alloc_workqueue("mali", WQ_UNBOUND, 0);
	mali_wq_high = alloc_workqueue("mali_high_pri", WQ_HIGHPRI | WQ_UNBOUND, 0);
#else
	mali_wq_normal = create_workqueue("mali");
	mali_wq_high = create_workqueue("mali_high_pri");
#endif
	if (NULL == mali_wq_normal || NULL == mali_wq_high) {
		MALI_PRINT_ERROR(("Unable to create Mali workqueues\n"));

		if (mali_wq_normal) destroy_workqueue(mali_wq_normal);
		if (mali_wq_high)   destroy_workqueue(mali_wq_high);

		mali_wq_normal = NULL;
		mali_wq_high   = NULL;

		return _MALI_OSK_ERR_FAULT;
	}

#ifdef CONFIG_MALI400_RT_WORKER
	MALI_DEBUG_ASSERT(NULL == mali_rt_worker_task);

	init_kthread_worker(&mali_rt_worker);
	mali_rt_worker_task = kthread_run(kthread_worker_fn, &mali_rt_worker, "mali_rt");
	if (NULL == mali_rt_worker_task) {
		MALI_PRINT_ERROR(("Unable to create Mali worker thread\n"));

		if (mali_rt_worker_task) {
			flush_kthread_worker(&mali_rt_worker);
			kthread_stop(mali_rt_worker_task);
		}
		return _MALI_OSK_ERR_FAULT;
	} else {
		struct sched_param param = {
		.sched_priority = CONFIG_MALI400_RT_WORKER_PRIORITY,
		};

		/* Use FIFO scheduler for realtime purpose */
		sched_setscheduler(mali_rt_worker_task, SCHED_FIFO, &param);
	}
#endif
#endif /* MALI_LICENSE_IS_GPL */

	return _MALI_OSK_ERR_OK;
}

void _mali_osk_wq_flush(void)
{
#if MALI_LICENSE_IS_GPL
	flush_workqueue(mali_wq_high);
	flush_workqueue(mali_wq_normal);
#ifdef CONFIG_MALI400_RT_WORKER
	flush_kthread_worker(&mali_rt_worker);
#endif
#else
	flush_scheduled_work();
#endif
}

void _mali_osk_wq_term(void)
{
#if MALI_LICENSE_IS_GPL
	MALI_DEBUG_ASSERT(NULL != mali_wq_normal);
	MALI_DEBUG_ASSERT(NULL != mali_wq_high);

	flush_workqueue(mali_wq_normal);
	destroy_workqueue(mali_wq_normal);

	flush_workqueue(mali_wq_high);
	destroy_workqueue(mali_wq_high);

	mali_wq_normal = NULL;
	mali_wq_high   = NULL;
#ifdef CONFIG_MALI400_RT_WORKER
	MALI_DEBUG_ASSERT(NULL != mali_rt_worker_task);

	flush_kthread_worker(&mali_rt_worker);
	kthread_stop(mali_rt_worker_task);
	mali_rt_worker_task = NULL;
#endif
#else
	flush_scheduled_work();
#endif
}

_mali_osk_wq_work_t *_mali_osk_wq_create_work(_mali_osk_wq_work_handler_t handler, void *data)
{
	mali_osk_wq_work_object_t *work = kmalloc(sizeof(mali_osk_wq_work_object_t), GFP_KERNEL);

	if (NULL == work) return NULL;

	work->handler = handler;
	work->data = data;
	work->high_pri = MALI_FALSE;

#ifdef CONFIG_MALI400_RT_WORKER
	init_kthread_work(&work->kwork, mali_rt_worker_callback);
#else
	INIT_WORK(&work->work_handle, _mali_osk_wq_work_func);
#endif

	return work;
}

_mali_osk_wq_work_t *_mali_osk_wq_create_work_high_pri(_mali_osk_wq_work_handler_t handler, void *data)
{
	mali_osk_wq_work_object_t *work = kmalloc(sizeof(mali_osk_wq_work_object_t), GFP_KERNEL);

	if (NULL == work) return NULL;

	work->handler = handler;
	work->data = data;
	work->high_pri = MALI_TRUE;

#ifdef CONFIG_MALI400_RT_WORKER
	init_kthread_work(&work->kwork, mali_rt_worker_callback);
#else
	INIT_WORK(&work->work_handle, _mali_osk_wq_work_func);
#endif

	return work;
}

void _mali_osk_wq_delete_work(_mali_osk_wq_work_t *work)
{
	mali_osk_wq_work_object_t *work_object = (mali_osk_wq_work_object_t *)work;
	_mali_osk_wq_flush();
	kfree(work_object);
}

void _mali_osk_wq_delete_work_nonflush(_mali_osk_wq_work_t *work)
{
	mali_osk_wq_work_object_t *work_object = (mali_osk_wq_work_object_t *)work;
	kfree(work_object);
}

void _mali_osk_wq_schedule_work(_mali_osk_wq_work_t *work)
{
	mali_osk_wq_work_object_t *work_object = (mali_osk_wq_work_object_t *)work;
#if MALI_LICENSE_IS_GPL
#ifdef CONFIG_MALI400_RT_WORKER
	queue_kthread_work(&mali_rt_worker, &work_object->kwork);
#else
	queue_work(mali_wq_normal, &work_object->work_handle);
#endif
#else
	schedule_work(&work_object->work_handle);
#endif
}

void _mali_osk_wq_schedule_work_high_pri(_mali_osk_wq_work_t *work)
{
	mali_osk_wq_work_object_t *work_object = (mali_osk_wq_work_object_t *)work;
#if MALI_LICENSE_IS_GPL
#ifdef CONFIG_MALI400_RT_WORKER
	queue_kthread_work(&mali_rt_worker, &work_object->kwork);
#else
	queue_work(mali_wq_high, &work_object->work_handle);
#endif
#else
	schedule_work(&work_object->work_handle);
#endif
}

#ifdef CONFIG_MALI400_RT_WORKER
static void mali_rt_worker_callback(struct kthread_work *kwork)
{
	mali_osk_wq_work_object_t *work_object = container_of(kwork, mali_osk_wq_work_object_t, kwork);
	work_object->handler(work_object->data);
}
#else
static void _mali_osk_wq_work_func(struct work_struct *work)
{
	mali_osk_wq_work_object_t *work_object;

	work_object = _MALI_OSK_CONTAINER_OF(work, mali_osk_wq_work_object_t, work_handle);

#if MALI_LICENSE_IS_GPL
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
	/* We want highest Dynamic priority of the thread so that the Jobs depending
	** on this thread could be scheduled in time. Without this, this thread might
	** sometimes need to wait for some threads in user mode to finish its round-robin
	** time, causing *bubble* in the Mali pipeline. Thanks to the new implementation
	** of high-priority workqueue in new kernel, this only happens in older kernel.
	*/
	if (MALI_TRUE == work_object->high_pri) {
		set_user_nice(current, -19);
	}
#endif
#endif /* MALI_LICENSE_IS_GPL */

	work_object->handler(work_object->data);
}
#endif

static void _mali_osk_wq_delayed_work_func(struct work_struct *work)
{
	mali_osk_wq_delayed_work_object_t *work_object;

	work_object = _MALI_OSK_CONTAINER_OF(work, mali_osk_wq_delayed_work_object_t, work.work);
	work_object->handler(work_object->data);
}

mali_osk_wq_delayed_work_object_t *_mali_osk_wq_delayed_create_work(_mali_osk_wq_work_handler_t handler, void *data)
{
	mali_osk_wq_delayed_work_object_t *work = kmalloc(sizeof(mali_osk_wq_delayed_work_object_t), GFP_KERNEL);

	if (NULL == work) return NULL;

	work->handler = handler;
	work->data = data;

	INIT_DELAYED_WORK(&work->work, _mali_osk_wq_delayed_work_func);

	return work;
}

void _mali_osk_wq_delayed_delete_work_nonflush(_mali_osk_wq_delayed_work_t *work)
{
	mali_osk_wq_delayed_work_object_t *work_object = (mali_osk_wq_delayed_work_object_t *)work;
	kfree(work_object);
}

void _mali_osk_wq_delayed_cancel_work_async(_mali_osk_wq_delayed_work_t *work)
{
	mali_osk_wq_delayed_work_object_t *work_object = (mali_osk_wq_delayed_work_object_t *)work;
	cancel_delayed_work(&work_object->work);
}

void _mali_osk_wq_delayed_cancel_work_sync(_mali_osk_wq_delayed_work_t *work)
{
	mali_osk_wq_delayed_work_object_t *work_object = (mali_osk_wq_delayed_work_object_t *)work;
	cancel_delayed_work_sync(&work_object->work);
}

void _mali_osk_wq_delayed_schedule_work(_mali_osk_wq_delayed_work_t *work, u32 delay)
{
	mali_osk_wq_delayed_work_object_t *work_object = (mali_osk_wq_delayed_work_object_t *)work;

#if MALI_LICENSE_IS_GPL
	queue_delayed_work(mali_wq_normal, &work_object->work, delay);
#else
	schedule_delayed_work(&work_object->work, delay);
#endif

}
