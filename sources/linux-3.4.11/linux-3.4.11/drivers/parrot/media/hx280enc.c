#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/pci.h>
#include <asm/uaccess.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <asm/irq.h>
#include <linux/version.h>
#include <linux/wait.h>
#include<linux/sched.h>
#include <linux/clk.h>
#include <linux/list.h>
//#include<asm/signal.h>
//#include<asm-generic/siginfo.h>

/* On2 own stuff */
#include "hx280enc.h"
#include "memalloc.h"

/* module description */
MODULE_LICENSE ("GPL");
MODULE_AUTHOR ("Hantro Products Oy");
MODULE_DESCRIPTION ("Hantro 6280/7280/8270 Encoder driver");

//static int COUNTER=0;
static int SLEEP = 0;
static DECLARE_WAIT_QUEUE_HEAD (wq);

#define ENC_IO_SIZE			(96 * 4)	/* bytes */
#define ENC_HW_ID	              	0x72800000
#define MEMALLOC_SIZE			(96 << 20)

/* here's all the must remember stuff */



struct hx280_venc_t
{
  struct device *dev;
  char *buffer;
  unsigned int buffsize;
  int hx280_venc_major;
  unsigned long iobaseaddr;
  unsigned int iosize;
  volatile u8 *hwregs;
  unsigned int irq;
  struct clk *hx280clk;
  hlinc chunks;
  struct fasync_struct *async_queue;
};

static struct hx280_venc_t *hxvenc;
static spinlock_t mem_lock = SPIN_LOCK_UNLOCKED;

static int hx280_venc_cleanup (struct platform_device *);
static int ReserveIO (void);
static void ReleaseIO (void);
static void ResetAsic (struct hx280_venc_t *);
static void dump_regs (unsigned long);
static int hx280_venc_probe (struct platform_device *);
int memalloc_register (struct device *, hlinc *, dma_addr_t, unsigned int);
int memalloc_unregister (struct device *);
int memalloc_AllocMemory (struct device *, hlinc *, unsigned *, unsigned int);
void memalloc_FreeMemory (struct device *, hlinc*, unsigned long);
void memalloc_ResetMems (struct device *, hlinc *);

/* IRQ handler */
static irqreturn_t hx280_venc_isr (int, void *);
/*clock init  */
static int hx280_venc_init_clock (struct hx280_venc_t *,
				  struct platform_device *);


/*------------*/

static int
hx280_venc_vm_fault (struct vm_area_struct *vma, struct vm_fault *vmf)
{
  PDEBUG ("hx280enc_vm_fault: problem with mem access\n");
  return VM_FAULT_SIGBUS;	// send a SIGBUS 
}

static void
hx280_venc_vm_open (struct vm_area_struct *vma)
{
  PDEBUG ("hx280enc_vm_open:\n");
}

static void
hx280_venc_vm_close (struct vm_area_struct *vma)
{
  PDEBUG ("hx280enc_vm_close:\n");
}

static struct vm_operations_struct hx280enc_vm_ops = {
open:hx280_venc_vm_open,
close:hx280_venc_vm_close,
fault:hx280_venc_vm_fault,
};

static int
hx280_venc_mmap (struct file *filp, struct vm_area_struct *vma)
{
  int result = -EINVAL;
  vma->vm_ops = &hx280enc_vm_ops;
  return result;
}

static long
hx280_venc_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
  int err = 0;
  PDEBUG (KERN_INFO"ioctl cmd %u\n", cmd);

  // extract the type and number bitfields, and don't encode
  // wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()

  if (_IOC_TYPE (cmd) != HX280ENC_IOC_MAGIC)
    {
      return -ENOTTY;
    }
  if (_IOC_NR (cmd) > HX280ENC_IOC_MAXNR)
    {
      return -ENOTTY;
    }

  // the direction is a bitmask, and VERIFY_WRITE catches R/W
  // transfers. `Type' is user-oriented, while
  // access_ok is kernel-oriented, so the concept of "read" and
  // "write" is reversed

  if (_IOC_DIR (cmd) & _IOC_READ)
    err = !access_ok (VERIFY_WRITE, (void *) arg, _IOC_SIZE (cmd));
  else if (_IOC_DIR (cmd) & _IOC_WRITE)
    err = !access_ok (VERIFY_READ, (void *) arg, _IOC_SIZE (cmd));
  if (err)
    return -EFAULT;
  switch (cmd)
    {
    case HX280ENC_IOCGHWOFFSET:
      __put_user (hxvenc->iobaseaddr, (unsigned long *) arg);
      break;

    case HX280ENC_IOCGHWIOSIZE:
      __put_user (hxvenc->iosize, (unsigned int *) arg);
      break;
    case HX280ENC_IOCWAIT:
      err = wait_event_interruptible_timeout (wq, SLEEP, HZ / 10);
      SLEEP = 0;
      if (!err)
	return -ETIMEDOUT;
      break;
    case MEMALLOC_IOCHARDRESET:
      PDEBUG ("HARDRESET\n");
      memalloc_ResetMems (hxvenc->dev, &hxvenc->chunks);
      break;
    case MEMALLOC_IOCXGETBUFFER:
      {
	int result;
	MemallocParams memparams;
	PDEBUG (KERN_INFO"GETBUFFER\n");
	spin_lock (&mem_lock);
	if (__copy_from_user
	    (&memparams, (const void *) arg, sizeof (memparams)))
	      result = -EFAULT;
	else
	  {
	    PDEBUG(KERN_INFO"%u\n",memparams.busAddress);
	    result =
	      memalloc_AllocMemory (hxvenc->dev, &hxvenc->chunks, &memparams.busAddress, memparams.size);
	    PDEBUG(KERN_INFO"%u\n",hxvenc->chunks.bus_address);
	    if (__copy_to_user ((void *) arg, &memparams, sizeof (memparams)))
	      {
		result = -EFAULT;
	      }
	  }
	spin_unlock (&mem_lock);
	return result;
      }
    case MEMALLOC_IOCSFREEBUFFER:
      {
	unsigned long busaddr;
	PDEBUG ("FREEBUFFER\n");
	spin_lock (&mem_lock);
	__get_user (busaddr, (unsigned long *) arg);
	spin_unlock (&mem_lock);
	memalloc_FreeMemory (hxvenc->dev, &hxvenc->chunks, busaddr);
	break;	
      }
    default:
	printk(KERN_ERR"IOCTL cmd not recognized\n");
	return -1;
    }
   
  return 0;
}

static int
hx280_venc_open (struct inode *inode, struct file *filp)
{
  int result = 0;
  struct hx280_venc_t *dev = hxvenc;
  filp->private_data = (void *) dev;
  PDEBUG ("dev opened\n");
  return result;
}

static int
hx280_venc_fasync (int fd, struct file *filp, int mode)
{
  struct hx280_venc_t *dev = (struct hx280_venc_t *) filp->private_data;
  PDEBUG ("fasync called\n");
  return fasync_helper (fd, filp, mode, &dev->async_queue);
}

static int
hx280_venc_release (struct inode *inode, struct file *filp)
{
  struct hx280_venc_t *dev = (struct hx280_venc_t *) filp->private_data;
  dump_regs ((unsigned long) dev);	// dump the regs 
  hx280_venc_fasync (-1, filp, 0);
  PDEBUG ("dev closed\n");
  return 0;
}

/* VFS methods */
static struct file_operations hx280_venc_fops = {
mmap:hx280_venc_mmap,
open:hx280_venc_open,
release:hx280_venc_release,
unlocked_ioctl:hx280_venc_ioctl,
fasync:hx280_venc_fasync,
};


static int
hx280_venc_probe (struct platform_device *pdev)
{
  struct resource *resm, *resi, *resmem;
  int result;


  /*dynamic allocation od data structure*/
  hxvenc = kzalloc (sizeof (struct hx280_venc_t), GFP_KERNEL);
  if (!hxvenc)
    {
      printk (KERN_ERR "HX280enc: Unable to allocate device structure\n");
      result = -EINVAL;
      goto err;
    }
  /*save the device in the struct*/
  hxvenc->dev=&pdev->dev;

  /*register driver */
  hxvenc->hx280_venc_major = 0;
  result =
    register_chrdev (hxvenc->hx280_venc_major, "hx280-venc",
		     &hx280_venc_fops);
  if (result < 0)
    {
      printk (KERN_INFO "HX280enc: unable to get major <%d>\n",
	      hxvenc->hx280_venc_major);
      goto err_alloc;
    }
  else if (result != 0)		// this is for dynamic major 
    {
      hxvenc->hx280_venc_major = result;
    }
  /*get memory ressource from platform device*/
  resm = platform_get_resource (pdev, IORESOURCE_MEM, 0);
  if (!resm)
    {
      printk ("HX280enc: Unable to get ressource\n");
      result = -ENXIO;
      goto err_reg;
    }
  hxvenc->iobaseaddr = resm->start;
  hxvenc->iosize = resm->end - resm->start + 1;
  /*init clk*/
  result = hx280_venc_init_clock (hxvenc, pdev);
  if (result)
    {
      printk (KERN_ERR "clock enable failed");
      goto err_reg;
    }
  /*map the address of the device on kernel space*/
  result = ReserveIO ();
  if (result < 0)
    goto err_clk;

  ResetAsic (hxvenc);		/* reset hardware */
  /*get irq ressource from platform device*/
  resi = platform_get_resource (pdev, IORESOURCE_IRQ, 0);
  if (!resi)
    {
      printk ("HX280enc: Unable to get ressource\n");
      result = -ENXIO;
      goto err_clk;
    }
  hxvenc->irq = resi->start;
  printk (KERN_INFO "HX280enc: irq=%i\n", hxvenc->irq);
  hxvenc->async_queue = NULL;

  /* get the IRQ line */
  if (hxvenc->irq != -1)
    {
      result = request_irq (hxvenc->irq, hx280_venc_isr,
			    IRQF_DISABLED | IRQF_SHARED,
			    "hx280enc", (void *) hxvenc);
      if (result == -EINVAL)
	{
	  printk (KERN_ERR "HX280enc: Bad irq number or handler\n");
	  ReleaseIO ();
	  goto err;
	}
      else if (result == -EBUSY)
	{
	  printk (KERN_ERR "HX280enc: IRQ <%d> busy, change your config\n", hxvenc->irq);
	  ReleaseIO ();
	  goto err;
	}
    }
  else
    {
      printk (KERN_INFO "HX280enc: IRQ not in use!\n");
    }

  /* get memalloc resources from platform device*/
  resmem = platform_get_resource (pdev, IORESOURCE_MEM, 1);
  if (!resmem)
    {
      printk ("HX280enc: Unable to get memalloc ressource\n");
      result = -ENXIO;
      goto err_clk;
    }
  /*call the dma-declare-coherent-memory*/
  memalloc_register (&pdev->dev, &hxvenc->chunks ,resmem->start,
		     (resmem->end - resmem->start + 1));

  printk (KERN_INFO "HX280enc: module inserted. Major <%d>\n",
	  hxvenc->hx280_venc_major);
  return 0;

err_clk:
  clk_disable (hxvenc->hx280clk);
  clk_put (hxvenc->hx280clk);
err_reg:
  unregister_chrdev (hxvenc->hx280_venc_major, "hx280-venc");
err_alloc:
  kfree (hxvenc);
err:
  printk (KERN_INFO "HX280enc: module not inserted\n");
  return result;
}


static int
hx280_venc_cleanup (struct platform_device *pdev)
{
  PDEBUG (KERN_INFO "HX280enc: remove module\n");
  writel (0, hxvenc->hwregs + 0x04);	/* clear enc IRQ */
  PDEBUG (KERN_INFO "HX280enc: reset IRQ\n");
  /* free the encoder IRQ */
  if (hxvenc->irq != -1)
    {
      free_irq (hxvenc->irq, (void *) hxvenc);
    }
  PDEBUG (KERN_INFO "HX280enc: free IRQ\n");
  ReleaseIO ();
  PDEBUG (KERN_INFO "HX280enc: Release IO\n");
  memalloc_unregister (&pdev->dev);
  unregister_chrdev (hxvenc->hx280_venc_major, "hx280-venc");
  clk_disable (hxvenc->hx280clk);
  printk (KERN_INFO "HX280enc: module removed\n");
  kfree (hxvenc);
  return 0;
}

static int
ReserveIO (void)
{
  long int hwid;
  if (!request_mem_region (hxvenc->iobaseaddr, hxvenc->iosize, "hx280-venc"))
    {
      printk (KERN_INFO "HX280enc: failed to reserve HW regs\n");
      return -EBUSY;
    }
  hxvenc->hwregs =
    (volatile u8 *) ioremap (hxvenc->iobaseaddr, hxvenc->iosize);
  if (hxvenc->hwregs == NULL)
    {
      printk (KERN_INFO "HX280enc: failed to ioremap HW regs\n");
      ReleaseIO ();
      return -EBUSY;
    }
  hwid = readl (hxvenc->hwregs);
  /* check for encoder HW ID */
  if ((((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID >> 16) & 0xFFFF)))
    {
      printk (KERN_INFO "HX280enc: HW not found at 0x%08lx\n",
	      hxvenc->iobaseaddr);
      dump_regs ((unsigned long) hxvenc);
      ReleaseIO ();
      return -EBUSY;
    }
  PDEBUG (KERN_INFO
	  "HX280enc: Hardware at bus @ <0x%08lx> with ID <0x%08lx>\n",
	  hxvenc->iobaseaddr, hwid);
  return 0;
}

static void
ReleaseIO (void)
{
  if (hxvenc->hwregs)
    iounmap ((void *) hxvenc->hwregs);
  release_mem_region (hxvenc->iobaseaddr, hxvenc->iosize);
}

irqreturn_t
hx280_venc_isr (int irq, void *dev_id)
{
  struct hx280_venc_t *dev = (struct hx280_venc_t *) dev_id;
  u32 irq_status;
  irq_status = readl (dev->hwregs + 0x04);
  if (irq_status & 0x01)
    {
      writel (irq_status & (~0x01), dev->hwregs + 0x04);	/* clear enc IRQ */
/*      if (dev->async_queue)
	kill_fasync (&dev->async_queue, SIGIO, POLL_IN);
      else
	{
	  printk (KERN_WARNING
		  "hx280enc: IRQ received w/o anybody waiting for it!\n");
	}
*/ SLEEP = 1;
      wake_up_interruptible (&wq);

      PDEBUG ("IRQ handled!\n");
      return IRQ_HANDLED;
    }
  else
    {
      PDEBUG ("IRQ received, but NOT handled!\n");
      return IRQ_NONE;
    }
}

static int
hx280_venc_init_clock (struct hx280_venc_t *host,
		       struct platform_device *pdev)
{
  int ret;
  struct clk *hx280_venc_clk;
  hx280_venc_clk = clk_get (&pdev->dev, NULL);
  if (IS_ERR (hx280_venc_clk))
    {
      ret = PTR_ERR (hx280_venc_clk);
      goto out;
    }
  ret = clk_enable (hx280_venc_clk);
  if (ret)
    {
      goto put_clk;
    }
  host->hx280clk = hx280_venc_clk;
  return 0;

put_clk:
  clk_put (hx280_venc_clk);
out:
  return ret;
}


void
ResetAsic (struct hx280_venc_t *dev)
{
  int i;
  writel (0, dev->hwregs + 0x38);
  for (i = 4; i < ENC_IO_SIZE; i += 4)
    writel (0, dev->hwregs + i);
}

void
dump_regs (unsigned long data)
{
  //struct hx280_venc_t *dev = (struct hx280_venc_t *) data;
  int i;
  PDEBUG ("Reg Dump Start\n");
  for (i = 0; i < ENC_IO_SIZE; i += 4)
    {
      PDEBUG ("\toffset %02X = %08X\n", i, readl (dev->hwregs + i));
    }
  PDEBUG ("Reg Dump End\n");
}

static struct platform_driver hx280_venc_driver = {
  .remove = &hx280_venc_cleanup,
  .probe = &hx280_venc_probe,
  .driver = {
	     .name = "hx280-venc",
	     .owner = THIS_MODULE,
	     },
};

static int __init
hx280_venc_init (void)
{
  return platform_driver_register (&hx280_venc_driver);
}

module_init (hx280_venc_init);

static void __exit
hx280_venc_exit (void)
{
  platform_driver_unregister (&hx280_venc_driver);
}

module_exit (hx280_venc_exit);
