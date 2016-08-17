/* Allocate memory blocks
 *
 * Copyright (C) 2009  On2 Technologies Finland Oy.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
--------------------------------------------------------------------------------
--
--  Version control information, please leave untouched.
--
--  $RCSfile: memalloc.c,v $
--  $Date: 2011/03/10 14:03:45 $
--  $Revision: 1.1 $
--
------------------------------------------------------------------------------*/

#include <linux/kernel.h>
#include <linux/module.h>
/* needed for __init,__exit directives */
#include <linux/init.h>
/* needed for remap_page_range */
#include <linux/mm.h>
/* obviously, for kmalloc */
#include <linux/slab.h>
/* for struct file_operations, register_chrdev() */
#include <linux/fs.h>
/* standard error codes */
#include <linux/errno.h>
/* this header files wraps some common module-space operations ...
   here we use mem_map_reserve() macro */
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/ioport.h>
#include <linux/list.h>
/* for current pid */
#include <linux/sched.h>
/* Our header */
#include "hx280enc.h"
#include "memalloc.h"
/*platform device*/
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
/* module description */
MODULE_LICENSE ("GPL");
MODULE_AUTHOR ("Hantro Products Oy");
MODULE_DESCRIPTION ("RAM allocation");

#define MAXDEV 2	//ENCODE and DECODER 

#define MEMALLOC_SW_MINOR 6
#define MEMALLOC_SW_MAJOR 0
#define MEMALLOC_SW_BUILD ((MEMALLOC_SW_MAJOR * 1000) + MEMALLOC_SW_MINOR)



static void __exit
memalloc_cleanup (void)
{
  printk (KERN_INFO "memalloc: module removed\n");
  return;
}

/*We must keep a list of the allocated chunk*/
int
memalloc_AllocMemory (struct device *dev, hlinc *chunks,unsigned *busaddr, unsigned int size)
{
  int err;
  void *rslt;
  unsigned int asked_page;
  hlinc *elem;
  if (!size)
    {
      err = -1;
      goto out;
    }
  asked_page = PAGE_ALIGN (size);
  rslt =
    (unsigned *) dma_alloc_coherent (dev, asked_page,
				     (dma_addr_t *) busaddr, GFP_DMA);
  if (!rslt)
    {
      PDEBUG ("MEMALLOC FAILED: size = %d\n", size);
      goto out;
    }
  PDEBUG (KERN_WARNING
	      "MEMALLOC OK: size: %d, size reserved: %d at @ 0x%x\n", size,
	      asked_page, *busaddr);
  elem = kmalloc (sizeof *chunks, GFP_KERNEL);
  PDEBUG (KERN_WARNING "alloc  %d\n", asked_page);
  elem->bus_address = *busaddr;
  elem->size = asked_page;
  elem->ba = rslt;
  list_add (&(elem->list), &(chunks->list));
  return 0;
out:
  return err;
}
EXPORT_SYMBOL (memalloc_AllocMemory);

void
memalloc_FreeMemory (struct device *dev, hlinc *chunks,unsigned long busaddr)
{
   hlinc *res, *resn;
  list_for_each_entry_safe (res, resn, &(chunks->list), list)
  {
    PDEBUG (KERN_INFO "elem 0x%x, curr 0x%x\n", res->bus_address,
	    (unsigned int) busaddr);
    if (res->bus_address == busaddr)
      {
	PDEBUG (KERN_WARNING "alloc - 1 %d\n", res->size);
	dma_free_coherent (dev, res->size, (void *) res->ba,
			   res->bus_address);
	list_del (&res->list);
	kfree (res);
	break;
      }
  }
}
EXPORT_SYMBOL (memalloc_FreeMemory);

void
memalloc_ResetMems (struct device *dev, hlinc *chunks)
{
   hlinc *res, *resn;
  list_for_each_entry_safe (res, resn, &(chunks->list), list)
  {
    dma_free_coherent (dev, res->size, (void *) res->ba, res->bus_address);
    list_del (&res->list);
    kfree (res);
  }
}

EXPORT_SYMBOL (memalloc_ResetMems);

int
memalloc_register(struct device *dev, hlinc *chunks, dma_addr_t bus_addr, unsigned int size)
{
  int err;
  INIT_LIST_HEAD (&(chunks->list));
  err =
    dma_declare_coherent_memory (dev, bus_addr, bus_addr,
				 size, DMA_MEMORY_MAP | DMA_MEMORY_EXCLUSIVE);
  if (!err)
    {
      return -ENOMEM;
    }
  return 0;
}

EXPORT_SYMBOL (memalloc_register);

int
memalloc_unregister (struct device *dev)
{
  dma_release_declared_memory (dev);
  return 0;
}

EXPORT_SYMBOL (memalloc_unregister);

int __init
memalloc_init (void)
{
  return 0;
}

module_init (memalloc_init);
module_exit (memalloc_cleanup);
