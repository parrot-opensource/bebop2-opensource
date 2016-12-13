/*
*
*  Copyright (C) 2010 Parrot S.A.
*
* @author  Samir Ammenouche <samir.ammenouche@parrot.com>
* @date  1-Jun-2011
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/



#include <linux/kernel.h>
#include <linux/module.h>
/* needed for __init,__exit directives */
#include <linux/init.h>
/* needed for remap_page_range 
	SetPageReserved
	ClearPageReserved
*/
#include <linux/mm.h>
/* obviously, for kmalloc */
#include <linux/slab.h>
/* for struct file_operations, register_chrdev() */
#include <linux/fs.h>
/* standard error codes */
#include <linux/errno.h>

#include <linux/moduleparam.h>
/* request_irq(), free_irq() */
#include <linux/interrupt.h>

/* needed for virt_to_phys() */
#include <asm/io.h>
#include <linux/pci.h>
#include <asm/uaccess.h>
#include <linux/ioport.h>

#include <asm/irq.h>

#include <linux/version.h>
//#include <linux/types.h>
#include<asm/signal.h>
#include<asm-generic/siginfo.h>

#include<linux/spinlock.h>
#include"aximon.h"

/* module description */
MODULE_LICENSE ("GPL");
MODULE_AUTHOR ("Parrot");
MODULE_DESCRIPTION ("Axi monitor driver");


static spinlock_t axi_lock = SPIN_LOCK_UNLOCKED;
static volatile u8 *vmap;

static unsigned int aximon_major;
const unsigned int irq = 106;

static axi_mon_reg axi_mon;


irqreturn_t aximon_it (int, void *);



static long
aximon_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
  unsigned long rslt;
  switch (cmd)
    {
    case AXIMON_IOCGHWSET:
      PDEBUG ("AXIMON_IOCGHWSET\n");
      spin_lock (&axi_lock);
      rslt =
	__copy_from_user (&axi_mon, (const void *) arg, sizeof (axi_mon));
      spin_unlock (&axi_lock);
      if (rslt)
	return -EFAULT;
      writel (axi_mon.iten, vmap + MON_ITEN);
      writel (axi_mon.timer_set, vmap + MON_TIMER_SET);
      writel (axi_mon.xtra_id, vmap + MON_XTRA_ID);
      writel (axi_mon.mode_rw, vmap + MON_MODE_RW);
      writel (axi_mon.mode_lat, vmap + MON_MODE_LAT);
      writel (axi_mon.mode_aws, vmap + MON_MODE_AWS);
      break;
    case AXIMON_IOCGHWGET:
      PDEBUG ("AXIMON_IOCGHWGET\n");
      spin_lock (&axi_lock);
      rslt =
	__copy_to_user ((void *) arg, (const void *) &axi_mon,
			sizeof (axi_mon));
      spin_unlock (&axi_lock);
      if (rslt)
	return -EFAULT;
      break;
    case AXIMON_IOCGHWRESET:
      PDEBUG ("AXIMON_IOCGHWRESET\n");
      aximon_reset ();
      break;
    case AXIMON_IOCGHWSTART:
      PDEBUG ("AXIMON_IOCGHWSTART\n");
      aximon_start ();
      break;
    case AXIMON_IOCGHWSTOP:
      PDEBUG ("AXIMON_IOCGHWSTOP\n");
      aximon_stop ();
      break;
    }
  return 0;
}


static int
aximon_open (struct inode *inode, struct file *filp)
{
  axi_mon_reg *dev = &axi_mon;
  filp->private_data = (void *) dev;
  return 0;
};

static int
aximon_release (struct inode *inode, struct file *filp)
{
  return 0;
};

static struct file_operations aximon_fops = {
  .open = aximon_open,
  .release = aximon_release,
  .unlocked_ioctl = aximon_ioctl,
};

int __init
aximon_init (void)
{
  int result;
  printk (KERN_INFO "Axi monitor driver\n");

  vmap = (volatile u8 *) ioremap (AXI_MON_OFFSET, 0x100);
  if (vmap == NULL)
    goto err_map;


/*Set mode RW */
  writel (0x03FF, vmap + MON_MODE_RW);
/*Set max lat mode*/
  writel (0x1000000, vmap + MON_XTRA_ID);
  writel (0x0, vmap + MON_MODE_LAT);
/*Set Timer */
  axi_mon.timer_set=0xFFFFFFFF;
  writel (axi_mon.timer_set, vmap + MON_TIMER_SET);
/*reset all counters */
  writel (0x007F0000, vmap + MON_CTRL);
/*disable all interupts */
  writel (0x0, vmap + MON_ITEN);
/*stop counting if counter overflow */
  writel (0x0000007F, vmap + MON_CTRL);

  /*
   * The bits [8-14] of MON_CTRL countrol the enable axi monitor (set 1 == enable)
   * bit_8 Timer counter
   * bit_9 AddressAccess counter
   * bit_10 AddressWaitStates counter 
   * bit_11 DataAccess counter 
   * bit_12 DataWaitState counter
   * bit_13 CumulativeLatency counter
   * bit_14 global idle counter
   */

  result = register_chrdev (aximon_major, "aximon", &aximon_fops);
  if (result < 0)
    {
      printk (KERN_INFO "aximon: unable to get major <%d>\n", aximon_major);
      goto err_chdev;
    }
  else if (result != 0)		// this is for dynamic major 
    {
      aximon_major = result;
    }


  result = request_irq ((unsigned int) irq, aximon_it, 0, "aximon", NULL);
  if (result == -EINVAL)
    {
      printk (KERN_ERR "aximon: Bad irq number or handler\n");
      goto err_irq;
    }
  else if (result == -EBUSY)
    {
      printk (KERN_ERR "aximon: IRQ <%d> busy, change your config\n", irq);
      goto err_irq;
    }

  printk (KERN_ERR
	  "Axi driver mounted with default config, irq=0x%d,  Major <%d> \n",
	  irq, aximon_major);
  return 0;
err_irq:unregister_chrdev (aximon_major, "aximon");
err_chdev:iounmap (vmap);
err_map:printk (KERN_ERR "Unable to mount axi driver\n");
  return -1;
}


void
aximon_start (void)
{

  writel (0x00007F00, vmap + MON_CTRL);
}
EXPORT_SYMBOL_GPL(aximon_start);

void
aximon_stop (void)
{
  int i,j,k=0;
//  printk("aximon: status register 0x%x",readl(vmap + MON_STATUS ));
  writel (0x00000000, vmap + MON_CTRL);
  for(j = 0; j < 1; j++)
  for(i = 0 ; i < 6; i++)
    {
      axi_mon.Tablecpt[k++] += readl (vmap + MON_AXI_0_Filter + (i<<2) + (j<<5));
//      printk (KERN_ERR"aximon 0x%x\n", readl (vmap + MON_AXI_0_Filter + (i<<2) + (j<<5)) );
    }
}
EXPORT_SYMBOL_GPL(aximon_stop);

static void
aximon_reset_from_irq (void)
{
  writel (0x007F0000, vmap + MON_CTRL);
  writel (axi_mon.iten, vmap + MON_ITEN);
  writel (axi_mon.timer_set, vmap + MON_TIMER_SET);
  writel (axi_mon.xtra_id, vmap + MON_XTRA_ID);
  writel (axi_mon.mode_rw, vmap + MON_MODE_RW);
  writel (axi_mon.mode_lat, vmap + MON_MODE_LAT);
  writel (axi_mon.mode_aws, vmap + MON_MODE_AWS);
}

void
aximon_reset (void)
{ 
  int i;
  for(i = 0 ; i < 6; i++)
      axi_mon.Tablecpt[i]=0;
  writel (0x007F0000, vmap + MON_CTRL);
  writel (0x3070300, vmap + MON_AXI_0_Filter);
}
EXPORT_SYMBOL_GPL(aximon_reset);

void aximon_print(void)
{
  int i;
  for(i = 0 ; i < 6; i++)
      printk (KERN_INFO"aximon %llu\n", axi_mon.Tablecpt[i]);
}
EXPORT_SYMBOL_GPL(aximon_print);


irqreturn_t
aximon_it (int irq, void *dev_id)
{
  aximon_stop();
  aximon_reset_from_irq ();
  aximon_start();
  return IRQ_HANDLED;
}



void __exit
aximon_exit (void)
{ 
  printk (KERN_INFO "Remove AXI_MON driver...");
  unregister_chrdev (aximon_major, "aximon");
  aximon_reset ();
  iounmap (vmap);
  free_irq (irq, NULL);
  printk (KERN_INFO "...Done\n");
}

module_init (aximon_init);
module_exit (aximon_exit);
