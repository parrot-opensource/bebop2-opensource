#ifndef H_AXIMON_H
#define H_AXIMON_H
#include <linux/ioctl.h>    /* needed for the _IOW etc stuff used later */
#include <linux/types.h>

typedef struct 
{
  unsigned int status;
  unsigned int ctrl;
  unsigned int iten;
  unsigned int timer_set;
  unsigned int timer_get;
  unsigned int idle;
  unsigned int xtra_id;
  unsigned int xtra_lat;
  unsigned int mode_rw;
  unsigned int mode_lat;
  unsigned int mode_aws;
  //unsigned int Tablecpt[25];	// 5 events * 5 cells
  uint64_t Tablecpt[25];	// 5 events * 5 cells
} axi_mon_reg;

void aximon_reset (void);
void aximon_start (void);
void aximon_stop (void);
void aximon_print(void);


#define MON_STATUS 	     0x000
#define MON_CTRL             0x004
#define MON_ITEN             0x008
#define MON_TIMER_SET        0x010
#define MON_TIMER_GET        0x014
#define MON_IDLE             0x020
#define MON_XTRA_ID          0x028
#define MON_XTRA_LAT         0x02c
#define MON_MODE_RW          0x030
#define MON_MODE_LAT         0x034
#define MON_MODE_AWS         0x038
#define MON_AXI_0_Filter     0x040
#define MON_AXI_0_AA         0x044
#define MON_AXI_0_AW         0x048
#define MON_AXI_0_DA         0x04C
#define MON_AXI_0_DW         0x050
#define MON_AXI_0_LAT        0x054

#define MON_AXI1_FILTER        0x060
#define MON_AXI1_AA            0x064
#define MON_AXI1_AW            0x068
#define MON_AXI1_DA            0x06C
#define MON_AXI1_DW            0x070
#define MON_AXI1_LAT           0x074

#define MON_AXI2_FILTER        0x080
#define MON_AXI2_AA            0x084
#define MON_AXI2_AW            0x088
#define MON_AXI2_DA            0x08C
#define MON_AXI2_DW            0x090
#define MON_AXI2_LAT           0x094

#define MON_AXI3_FILTER        0x0A0
#define MON_AXI3_AA            0x0A4
#define MON_AXI3_AW            0x0A8
#define MON_AXI3_DA            0x0AC
#define MON_AXI3_DW            0x0B0
#define MON_AXI3_LAT           0x0B4

#define MON_AXI4_FILTER        0x0C0
#define MON_AXI4_AA            0x0C4
#define MON_AXI4_AW            0x0C8
#define MON_AXI4_DA            0x0CC
#define MON_AXI4_DW            0x0D0
#define MON_AXI4_LAT           0x0D4


#define MPMC_OFFSET 0x00900000
#define AXI_MON_OFFSET (0x00080000 + MPMC_OFFSET)

#define AXIMON_IOC_MAGIC  'x'
#define AXIMON_IOCGHWSET      _IOW(AXIMON_IOC_MAGIC,  10, unsigned int *)
#define AXIMON_IOCGHWGET      _IOR(AXIMON_IOC_MAGIC,  11, unsigned int *)
#define AXIMON_IOCGHWRESET    _IOR(AXIMON_IOC_MAGIC,  12, unsigned int *)
#define AXIMON_IOCGHWSTART    _IOR(AXIMON_IOC_MAGIC,  13, unsigned int *)
#define AXIMON_IOCGHWSTOP     _IOR(AXIMON_IOC_MAGIC,  14, unsigned int *)

#undef PDEBUG   /* undef it, just in case */
#ifdef AXI_MON_DEBUG
#  ifdef __KERNEL__
    /* This one if debugging is on, and kernel space */
#    define PDEBUG(fmt, args...) printk( KERN_INFO "memalloc: " fmt, ## args)
#  else
    /* This one for user space */
#    define PDEBUG(fmt, args...) fprintf(stderr, fmt, ## args)
#  endif
#else
#  define PDEBUG(fmt, args...)  /* not debugging: nothing */
#endif


#endif
