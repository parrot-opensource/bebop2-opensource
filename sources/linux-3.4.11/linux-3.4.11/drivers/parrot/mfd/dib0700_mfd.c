#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/firmware.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/kfifo.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/export.h>
#include <linux/platform_device.h>

#include <parrot/mfd/dib0700_mfd.h>

#define UART_DEV_PENDING
#define MAX_NBR_UARTS	20 //127 /* Maximum number of uart that can be managed by the driver */

#define DRIVER_NAME "dib0700_mfd"
/* 1 pin for SCL can be common all i2c adapter,
   then 10 sda available + the default i2c controller (no bit bangint) */
#define ADAPTER_MAX	10+1

/* DIB0700 internal state machine */
#define STATE_DISCONNECT	0
#define STATE_I2C_REGISTER	1
#define STATE_TTY_REGISTER	2
#define STATE_TTY_INSTALL	3
#define STATE_TTY_ENABLE	4
#define STATE_TTY_TX_START	5

#define kref_to_dib0700_state(d) container_of(d, struct dib0700_state, kref)

static char *i2c_param = NULL;
module_param(i2c_param, charp, 0000);
MODULE_PARM_DESC(i2c_param, "i2c adapter parameter");

static char *firmware = "dvb-usb-dib0700-1.20.fw";
module_param(firmware, charp, 0000);
MODULE_PARM_DESC(firmware, "firmware parameter");


int dvb_usb_dib0700_debug;
module_param_named(debug,dvb_usb_dib0700_debug, int, 0644);

int serial_debug = 0;
module_param_named(serial, serial_debug, int, 0644);

static struct platform_device static_hook = {
	.name          = "static_hook",
	.id            = -1,
	.dev = {
		.platform_data = NULL,
	},
};

struct i2c_adapter_properties {
	/* if master=1, master i2c is used
	   there is not need of sda and scl pins */
	int	sda;
	int	scl;
	int	master;
	int	speed;

	struct i2c_adapter i2c_adap;
	int num_adapter;
};

#define USB_ENDP_CTRL_BUSY 0
struct dib0700_state {
	/* USB */
	struct usb_device *udev;
	struct usb_interface *intf;

	/* Misc */
	u32 fw_version;
	u32 hwversion;
	u32 romversion;
	u32 fwtype;
	spinlock_t lock;
	struct kref kref;
	unsigned long	istate; /* internal state */
	wait_queue_head_t wq_endp0;


	/* i2c */
	struct i2c_adapter_properties props[ADAPTER_MAX];
	int num_adapters;
	struct rt_mutex bus_lock; /* clk is shared */

#ifdef UART_DEV_PENDING
	/* serial */
	struct tty_port		port;
	struct tty_struct	*tty;
	struct work_struct	work;
	unsigned int		index;
	struct async_icount	icount;
	struct device		*ttydev;
	struct timer_list	rx_timer;
	unsigned long		rx_expires;
	struct timer_list	tx_timer;
	unsigned long		tx_expires;

#define NBR_MAX_RD_URBS		2
/* The hook cannot manage request greater than 1408 bytes */
#define RD_URB_SIZE		128 //1408
				/* bytes for the size
					  of FIFO (in the dib0700 hook) */
				/* 8 * 1000 * total urb size / baudrate */
#define RX_TIMER_CONST		(8000 * RD_URB_SIZE)

	unsigned char		*uart_in_buffers[NBR_MAX_RD_URBS];
	struct urb		*read_urbs[NBR_MAX_RD_URBS];
	struct usb_ctrlrequest	rd_ctrlrq;
	unsigned long		read_urbs_free;

#define SRL_WRITE_OFFSET	1
#define NBR_MAX_WR_URBS		3
#define WR_URB_SIZE		(64 - SRL_WRITE_OFFSET) //512
#define TX_TIMER_CONST		(8000 * WR_URB_SIZE * (NBR_MAX_WR_URBS-1))
	unsigned char		*uart_out_buffers[NBR_MAX_WR_URBS];
	struct urb		*write_urbs[NBR_MAX_WR_URBS];
	struct usb_ctrlrequest	wrt_ctrlrq;
	unsigned long		write_urbs_free;
	struct kfifo		write_fifo;
	int			tx_bytes;
	int			tx_urb_cnt;

#define TX_MAX_RETRIES		10
	int			tx_retries;
	unsigned long		tx_retry;
	unsigned long		flags;
#endif //UART_DEV_PENDING

	/* gpio */
	struct gpio_chip        gpio_chip;
};

static struct dib0700_state *chip_state;
#ifdef UART_DEV_PENDING
/* serial */
static struct dib0700_state *dib0700_uart_table[MAX_NBR_UARTS];
static DEFINE_SPINLOCK(dib0700_uart_table_lock);
static struct tty_driver *dib0700_uart_tty_driver;

static void dib0700_write_start(struct dib0700_state *st);
#endif //UART_DEV_PENDING

static void __dib0700_mfd_put(struct dib0700_state *st);
#ifdef DEBUG
#define dib0700_mfd_put(st)	\
		do { pr_err("[%s][%d]\n",__func__, __LINE__); \
		__dib0700_mfd_put(st); }while(0);
#else
#define dib0700_mfd_put(st)	\
	do { __dib0700_mfd_put(st); }while(0);
#endif

/* commonly used firmware download types and function */
struct hexline {
	u8 len;
	u16 /*u32*/ addr;
	u8 type;
	u8 data[255];
	u8 chk;
};


static enum dib07x0_gpios gpio_to_reg[] ={
	GPIO0,
	GPIO1,
	GPIO2,
	GPIO3,
	GPIO4,
	GPIO5,
	GPIO6,
	GPIO7,
	GPIO8,
	GPIO9,
	GPIO10,
};

#define debug_dump_usb_xfer(st,text,data,size) \
	do { if ((dvb_usb_dib0700_debug & 0x08)){	\
		print_hex_dump(KERN_INFO, text,		\
				DUMP_PREFIX_NONE,	\
				16, 1, data,		\
				size, 0); } } while (0)

#ifdef DEBUG

#define FUNC_IN		do{ pr_err("[%s] in\n", __func__); }while(0);
#define FUNC_OUT	do{ pr_err("[%s][%d] out\n", __func__, __LINE__); }while(0);

#else
#define FUNC_IN
#define FUNC_OUT
#endif

static int dib0700_usb_control_msg(struct dib0700_state *st, unsigned int pipe_num, __u8 request,
		    __u8 requesttype, __u16 value, __u16 index, void *data,
		    __u16 size, int timeout)
{
	int ret;
	unsigned int pipe;

	if (test_bit(STATE_DISCONNECT, &st->istate)) {
		return -ENODEV;
	}

	kref_get(&st->kref);

	if (test_and_set_bit_lock(USB_ENDP_CTRL_BUSY, &st->flags)) {
		wait_event_interruptible(st->wq_endp0,
				!test_and_set_bit_lock(USB_ENDP_CTRL_BUSY, &st->flags));
	}

	if (requesttype & USB_DIR_IN)
		pipe = usb_rcvctrlpipe(st->udev, pipe_num);
	else
		pipe = usb_sndctrlpipe(st->udev, pipe_num);
	ret = usb_control_msg(st->udev, pipe, request, requesttype, value, index, data, size, timeout);

	clear_bit_unlock(USB_ENDP_CTRL_BUSY, &st->flags);
	wake_up_interruptible(&st->wq_endp0);

	dib0700_mfd_put(st);

	return ret;
}

#ifdef UART_DEV_PENDING
/***********************
	UART
************************/

/* Misc UART */
static void dib0700_serial_port_softint(struct dib0700_state *st)
{
	schedule_work(&st->work);
}

static void dib0700_port_work(struct work_struct *work)
{
	struct dib0700_state *st =
		container_of(work, struct dib0700_state, work);
	struct tty_struct *tty;

	dev_dbg(&st->intf->dev,
		"%s - port %d", __func__, st->index);

	tty = tty_port_tty_get(&st->port);
	if (!tty)
		return;

	tty_wakeup(tty);
	tty_kref_put(tty);
}

static void kill_urbs(struct dib0700_state *st)
{
	int index;

	for (index = 0; index < ARRAY_SIZE(st->read_urbs); index++)
		usb_kill_urb(st->read_urbs[index]);

	for (index = 0; index < ARRAY_SIZE(st->write_urbs); index++)
		usb_kill_urb(st->write_urbs[index]);
}

static void free_urbs(struct dib0700_state *st)
{
	int index, count = 0;

	for (index = 0; index < NBR_MAX_RD_URBS; index++) {
		if (st->read_urbs[index]) {
			usb_free_urb(st->read_urbs[index]);
			st->read_urbs[index] = NULL;
			count++;
		}

		if (st->uart_in_buffers[index]) {
			kfree(st->uart_in_buffers[index]);
			st->uart_in_buffers[index] = NULL;
			count++;
		}
	}

	count = 0;

	for (index = 0; index < NBR_MAX_WR_URBS; index++) {
		if (st->write_urbs[index]) {
			usb_free_urb(st->write_urbs[index]);
			st->write_urbs[index] = NULL;
			count++;
		}

		if (st->uart_out_buffers[index]) {
			kfree(st->uart_out_buffers[index]);
			st->uart_out_buffers[index] = NULL;
			count++;
		}
	}
}

static void dib0700_read_start(struct dib0700_state *st)
{

	int result, idx;
	struct urb *urb;
	//struct usb_ctrlrequest *setup = NULL;
	unsigned long flags;

	if (!st)
		return;

	if (test_bit(STATE_DISCONNECT, &st->istate))
		return;

	if (test_and_set_bit_lock(USB_ENDP_CTRL_BUSY, &st->flags)) {
		mod_timer(&st->rx_timer, jiffies + msecs_to_jiffies(10));
		return;
	}

	spin_lock_irqsave(&st->lock, flags);
	if (!test_bit(STATE_TTY_ENABLE, &st->istate)
		|| (!st->read_urbs_free)) {
		clear_bit_unlock(USB_ENDP_CTRL_BUSY, &st->flags);
		spin_unlock_irqrestore(&st->lock, flags);
		wake_up_interruptible(&st->wq_endp0);

		if (!st->read_urbs_free) {
			mod_timer(&st->rx_timer,
					jiffies + st->rx_expires);
		}
		return;
	}

	idx = (int)find_first_bit(&st->read_urbs_free,
					ARRAY_SIZE(st->read_urbs));
	spin_unlock_irqrestore(&st->lock, flags);

	clear_bit(idx, &st->read_urbs_free);

	urb = st->read_urbs[idx];

	result = usb_submit_urb(urb, GFP_ATOMIC);
	if (result) {
		pr_err("[%s] - error submitting urb: "
				"%d (urb = %p, idx = %d)\n",
				__func__, result, urb, idx);

		set_bit(idx, &st->read_urbs_free);
		clear_bit_unlock(USB_ENDP_CTRL_BUSY, &st->flags);
		wake_up_interruptible(&st->wq_endp0);

		if (result != -ENODEV) {
			mod_timer(&st->rx_timer,
				jiffies + st->rx_expires);
		}

		return;
	}

	//Debug
	/*if (dvb_usb_dib0700_debug >= 0x06) {
		setup = (struct usb_ctrlrequest*) urb->setup_packet;
		dev_err(&st->intf->dev, "USB control read: %x %x %x %x %x %x %d = %d\n",
			0, setup->bRequest, setup->bRequestType,
			setup->wValue, setup->wIndex,
			setup->wLength, 0, result);
	}*/

	/*if (test_bit(STATE_TTY_ENABLE, &st->istate)) {
		mod_timer(&st->rx_timer,
				jiffies + st->rx_expires);
		dev_dbg(&st->intf->dev,
			"%s - timer restart", __func__);
	}*/
}

static void dib0700_read_ctrl_callback(struct urb *urb)
{
	struct dib0700_state *st = urb->context;
	unsigned char *data = urb->transfer_buffer;
	int status = urb->status, idx, count = 0;
	//struct usb_ctrlrequest *setup
	//	= (struct usb_ctrlrequest*) urb->setup_packet;
	struct tty_struct *tty;
	unsigned long flags;
	char tty_flag = TTY_NORMAL;

	/*if (dvb_usb_dib0700_debug >= 0x06)
		dev_err(&st->intf->dev, "USB control read(clbck): %x %x %x %x %x %x %d = %d\n",
			0, setup->bRequest, setup->bRequestType,
			setup->wValue, setup->wIndex,
			setup->wLength, 0, status);

	debug_dump_usb_xfer(st,"UART RX>",
				urb->transfer_buffer,
				urb->actual_length);*/

	for (idx = 0; idx < ARRAY_SIZE(st->read_urbs); idx++)
		if (urb == st->read_urbs[idx])
			break;

	clear_bit_unlock(USB_ENDP_CTRL_BUSY, &st->flags);
	wake_up_interruptible(&st->wq_endp0);

	//Should never occurs
	if (idx >= ARRAY_SIZE(st->read_urbs) ) {
		pr_err("[%s] Bad urb [%p]",
			__func__, urb);
		return;
	}

	if (status) {
		//TODO manage retry
		/* Free URB */
		set_bit(idx, &st->read_urbs_free);

		pr_err("[%s] - non-zero urb status: %d\n",
			__func__, status);

		mod_timer(&st->rx_timer,
				jiffies + st->rx_expires);
		return;
	}

	tty = tty_port_tty_get(&st->port);

	count = tty_insert_flip_string_fixed_flag(tty,
					data, tty_flag,
					urb->actual_length);

	if ( count < urb->actual_length) {
		/* data are lost, the tty buffer has not been read */
		pr_debug("[%s] - unable to copy all data received (%d<%d)\n",
			__func__, count,
			urb->actual_length);
	}

	if (count)
		tty_flip_buffer_push(tty);

	tty_kref_put(tty);

	set_bit(idx, &st->read_urbs_free);
	spin_lock_irqsave(&st->lock, flags);
	st->icount.rx += count;
	spin_unlock_irqrestore(&st->lock, flags);

	if (urb->actual_length) {
		dib0700_read_start(st);
	} else {
		mod_timer(&st->rx_timer,
			jiffies + st->rx_expires);
	}
}

static void dib0700_write_start(struct dib0700_state *st)
{
	struct urb *urb = NULL;
	int count = 0, result;
	unsigned long flags;
	unsigned char *data;
	struct usb_ctrlrequest *setup = NULL;
	int idx;

	if (test_bit(STATE_DISCONNECT, &st->istate))
		return;

	if (test_and_set_bit_lock(USB_ENDP_CTRL_BUSY, &st->flags)) {
		mod_timer(&st->tx_timer, jiffies +
			msecs_to_jiffies(20));
		return;
	}

	if (st->tx_urb_cnt > NBR_MAX_WR_URBS) {
		/* Sent enough buffer and let the bus free */
		clear_bit_unlock(USB_ENDP_CTRL_BUSY, &st->flags);
		wake_up_interruptible(&st->wq_endp0);

		mod_timer(&st->tx_timer,
			jiffies + st->tx_expires);
		return;
	}

	spin_lock_irqsave(&st->lock, flags);
	if (st->tx_retries && st->tx_retry) {

		//Re-submit packet
		idx = (int)find_first_bit(&st->tx_retry,
				ARRAY_SIZE(st->write_urbs));

		urb = st->write_urbs[idx];

		if (st->tx_retries >= TX_MAX_RETRIES) {
			pr_err("[%s] message dropped after "
				"%d retries\n",
				__func__, TX_MAX_RETRIES);

#if 1	//Debug
			print_hex_dump(KERN_INFO,"UART TX (dropped)>",
					DUMP_PREFIX_NONE,
					16, 1,
					urb->transfer_buffer,
					urb->transfer_buffer_length,
					1);
#endif	//Debug

			st->tx_retries = 0;
			clear_bit(idx, &st->tx_retry);
			st->tx_bytes -= count;
			st->icount.tx -= count;
			set_bit(idx, &st->write_urbs_free);

			spin_unlock_irqrestore(&st->lock, flags);
			clear_bit_unlock(USB_ENDP_CTRL_BUSY, &st->flags);
			wake_up_interruptible(&st->wq_endp0);
			return;
		}
		spin_unlock_irqrestore(&st->lock, flags);
	} else {

		if (!st->write_urbs_free || !kfifo_len(&st->write_fifo)) {

			// Delays processing
			if (kfifo_len(&st->write_fifo)) {
				mod_timer(&st->tx_timer,
					jiffies + st->tx_expires);
			} else {
			//kfifo is empty
				clear_bit(STATE_TTY_TX_START, &st->istate);
			}

			spin_unlock_irqrestore(&st->lock, flags);
			clear_bit_unlock(USB_ENDP_CTRL_BUSY, &st->flags);
			wake_up_interruptible(&st->wq_endp0);
			return;
		}

		idx = (int)find_first_bit(&st->write_urbs_free,
					ARRAY_SIZE(st->write_urbs));

		spin_unlock_irqrestore(&st->lock, flags);

		clear_bit(idx, &st->write_urbs_free);
		urb = st->write_urbs[idx];

		data = (unsigned char *) urb->transfer_buffer;
		data[0] = REQUEST_UART_WRITE;

		count = kfifo_out_locked(&st->write_fifo,
				data + SRL_WRITE_OFFSET,
				(WR_URB_SIZE - SRL_WRITE_OFFSET),
				&st->lock);

		if (!count) {
			/* should not happens, kfifo allready checked */
			set_bit(idx, &st->write_urbs_free);
			clear_bit_unlock(USB_ENDP_CTRL_BUSY, &st->flags);
			wake_up_interruptible(&st->wq_endp0);
			return;
		}

		urb->transfer_buffer_length = count + SRL_WRITE_OFFSET;
		setup = (struct usb_ctrlrequest*) urb->setup_packet;
		setup->wLength = cpu_to_le16(count + SRL_WRITE_OFFSET);

		spin_lock_irqsave(&st->lock, flags);
		st->icount.tx += count;
		st->tx_bytes += count;
		spin_unlock_irqrestore(&st->lock, flags);
	}

	result = usb_submit_urb(urb, GFP_ATOMIC);

	if (result) {

		/*
		ENOMEM:  Out of memory
		ENODEV:  Unplugged device
		EPIPE:   Stalled endpoint
		*/

		if (result != -ENODEV) {
			spin_lock_irqsave(&st->lock, flags);

			if (st->tx_retries < TX_MAX_RETRIES) {
				//Re-submit
				st->tx_retries++;
				set_bit(idx, &st->tx_retry);
			} else {
				st->tx_retries = 0;
				clear_bit(idx, &st->tx_retry);
				st->tx_bytes -= count;
				st->icount.tx -= count;
				set_bit(idx, &st->write_urbs_free);

#if 1	//Debug
				print_hex_dump(KERN_INFO,"UART TX (dropped)>",
						DUMP_PREFIX_NONE,
						16, 1,
						urb->transfer_buffer,
						urb->transfer_buffer_length,
						1);
#endif	//Debug
			}
			spin_unlock_irqrestore(&st->lock, flags);

			if (st->tx_retries) {
				mod_timer(&st->tx_timer,
					jiffies + st->tx_expires);
			} else {
				pr_err("[%s] message dropped after "
					"%d retries\n",
					__func__, TX_MAX_RETRIES);
			}
		}

		clear_bit_unlock(USB_ENDP_CTRL_BUSY, &st->flags);
		wake_up_interruptible(&st->wq_endp0);
		return;
	}


	//Debug
	debug_dump_usb_xfer(st,"UART TX>",
				urb->transfer_buffer,
				urb->actual_length);
}

/* similar to usb_serial_generic_write_bulk_callback */
static void dib0700_write_ctrl_callback(struct urb *urb)
{
	struct dib0700_state *st = urb->context;
	int status = urb->status, idx;
	unsigned long flags;

	for (idx = 0; idx < ARRAY_SIZE(st->write_urbs); idx++)
		if (urb == st->write_urbs[idx])
			break;

	//Should never occurs
	if (idx >= ARRAY_SIZE(st->write_urbs) ) {
		pr_err("[%s] Bad urb [%p]", __func__, urb);
		return;
	}

	if (status) {
		// There is no more space left on the hook
		spin_lock_irqsave(&st->lock, flags);
		if (st->tx_retries < TX_MAX_RETRIES) {
			st->tx_retries++;
			set_bit(idx, &st->tx_retry);
		} else {
			st->tx_retries = 0;
			clear_bit(idx, &st->tx_retry);
			set_bit(idx, &st->write_urbs_free);
			st->tx_bytes -= (urb->transfer_buffer_length
					- SRL_WRITE_OFFSET);
			st->icount.tx -= (urb->transfer_buffer_length
					- SRL_WRITE_OFFSET);

			pr_err("[%s] message dropped after "
				"%d retries\n",
				__func__, TX_MAX_RETRIES);

#if 1	//Debug
		print_hex_dump(KERN_INFO,"UART TX (dropped)>",
				DUMP_PREFIX_NONE,
				16, 1,
				urb->transfer_buffer,
				urb->transfer_buffer_length,
				1);
#endif	//Debug

		}
		spin_unlock_irqrestore(&st->lock, flags);
		clear_bit_unlock(USB_ENDP_CTRL_BUSY, &st->flags);
		wake_up_interruptible(&st->wq_endp0);

		if (st->tx_retries) {
			mod_timer(&st->tx_timer, jiffies + st->tx_expires);
		}

	} else {

#if 0	//Debug
		print_hex_dump(KERN_INFO,"UART TX (cllbck)>",
				DUMP_PREFIX_NONE,
				16, 1,
				urb->transfer_buffer,
				urb->transfer_buffer_length,
				1);
#endif	//Debug

		//Debug
		debug_dump_usb_xfer(st,"UART TX (clbck)>",
					urb->transfer_buffer,
					urb->actual_length);

		spin_lock_irqsave(&st->lock, flags);
		st->tx_urb_cnt++;
		st->tx_retries = 0;
		clear_bit(idx, &st->tx_retry);
		set_bit(idx, &st->write_urbs_free);
		st->tx_bytes -= (urb->transfer_buffer_length
					- SRL_WRITE_OFFSET);
		spin_unlock_irqrestore(&st->lock, flags);
		clear_bit_unlock(USB_ENDP_CTRL_BUSY, &st->flags);
		wake_up_interruptible(&st->wq_endp0);

		if (test_bit(STATE_DISCONNECT, &st->istate))
			goto schedule_work;

		dib0700_write_start(st);
	}

schedule_work:
	dib0700_serial_port_softint(st);
}

static void rx_timer_routine(unsigned long data)
{
	dib0700_read_start((struct dib0700_state *) data);
}

#define TX_FIFO_REQUEST_BUSY 2
static void tx_timer_routine(unsigned long data)
{
	struct dib0700_state *st = (struct dib0700_state *) data;
	st->tx_urb_cnt = 0;
	dib0700_write_start(st);
}

static struct dib0700_state *dib0700_uart_port_get(unsigned index)
{
	struct dib0700_state *st = NULL;

	if (index >= MAX_NBR_UARTS)
		return NULL;

	spin_lock(&dib0700_uart_table_lock);
	st = dib0700_uart_table[index];

	if (test_bit(STATE_DISCONNECT, &st->istate))
		st = NULL;

	if (st)
		kref_get(&st->kref);
	spin_unlock(&dib0700_uart_table_lock);

	return st;
}


static void dib0700_uart_port_remove(struct dib0700_state *st)
{
	struct tty_struct *tty;

	if (!st)
		return;

	FUNC_IN;
	BUG_ON(dib0700_uart_table[st->index] != st);

	spin_lock(&dib0700_uart_table_lock);
	dib0700_uart_table[st->index] = NULL;
	spin_unlock(&dib0700_uart_table_lock);
	tty_unregister_device(dib0700_uart_tty_driver,
				st->index);

	mutex_lock(&st->port.mutex);
	tty = tty_port_tty_get(&st->port);

	if (tty) {
		tty_hangup(tty);
		tty_kref_put(tty);
	}

	mutex_unlock(&st->port.mutex);
	FUNC_OUT;
}

/* Port operations */
static int dib0700_uart_activate(struct tty_port *tport, struct tty_struct *tty)
{
	struct dib0700_state *st = tport->tty->driver_data;
	struct ktermios dummy = {0};

	if (!st)
		return -ENODEV;

	FUNC_IN;

	if (test_bit(STATE_DISCONNECT, &st->istate))
		return -ENODEV;

	/* Activate uart on hook */
	set_bit(STATE_TTY_ENABLE, &st->istate);

	st->tx_bytes = 0;
	st->tx_urb_cnt = 0;

	/* Set the termios */
	if (tport->tty->ops->set_termios)
		tport->tty->ops->set_termios(tport->tty, &dummy);

	/* Activate the timer to trigger the next reading */
	mod_timer(&st->rx_timer, jiffies + st->rx_expires);

	FUNC_OUT;
	return 0;
}

static void dib0700_uart_shutdown(struct tty_port *tport)
{
	struct dib0700_state *st = tport->tty->driver_data;
	struct ktermios dummy = {0};

	if (!st)
		return;

	FUNC_IN;

	/* Deactivate uart on hook */
	clear_bit(STATE_TTY_ENABLE, &st->istate);

	/* Deactivate the timer */
	del_timer_sync(&st->rx_timer);
	del_timer_sync(&st->tx_timer);

	/* kill urbs */
	kill_urbs(st);

	clear_bit(STATE_TTY_TX_START, &st->istate);

	/* Set the termios */
	if (!test_bit(STATE_DISCONNECT, &st->istate))
		if (tport->tty->ops->set_termios)
			tport->tty->ops->set_termios(tport->tty, &dummy);

	FUNC_OUT;
}

static const struct tty_port_operations dib0700_uart_port_ops = {
	.activate = dib0700_uart_activate,
	.shutdown = dib0700_uart_shutdown,
};


/* TTY operations */
static int dib0700_uart_open(struct tty_struct *tty, struct file *filp)
{
	struct dib0700_state *st = tty->driver_data;
	int err;

	if (!st)
		return -ENODEV;

	FUNC_IN;

	if (test_bit(STATE_DISCONNECT, &st->istate)) {
		FUNC_OUT;
		return -ENODEV;
	}

	/* Set a timer that launch a read over serial line */
	err = tty_port_open(&st->port, tty, filp);
	FUNC_OUT;

	return err;
}

static void dib0700_uart_close(struct tty_struct * tty, struct file * filp)
{
	struct dib0700_state *st = tty->driver_data;

	if(!st)
		return;

	FUNC_IN;
	tty_port_close(&st->port, tty, filp);

	/* workaround to force call to dib0700_uart_cleanup
	   when the device is unplug while a user-application
	   still hold a file descriptor */
	if (test_bit(STATE_DISCONNECT, &st->istate))
		tty_kref_put(tty);

	FUNC_OUT;
}

static int dib0700_uart_write (struct tty_struct * tty,
		      const unsigned char *buf, int count)
{
	struct dib0700_state *st = tty->driver_data;

	if (!st)
		return -ENODEV;

	FUNC_IN;

	if (test_bit(STATE_DISCONNECT, &st->istate)) {
		return -ENODEV;
	}

	if (!count)
		return 0;

	//Debug
	/*print_hex_dump(KERN_INFO,"uart_write>",
				DUMP_PREFIX_NONE,
				16, 1,
				buf,count,
				1);*/
	//Debug

	count = kfifo_in_locked(&st->write_fifo, buf, count, &st->lock);

	if (!test_and_set_bit(STATE_TTY_TX_START, &st->istate)) {
		//Force 1st write if necessary
		dib0700_write_start(st);
	}

	FUNC_OUT;
	return count;
}

static int dib0700_uart_write_room(struct tty_struct *tty)
{
	struct dib0700_state *st = tty->driver_data;
	unsigned long flags;
	int room;

	if (!st) /* TODO check */
		return -ENODEV;

	if (test_bit(STATE_DISCONNECT, &st->istate))
		return -ENODEV;

	spin_lock_irqsave(&st->lock, flags);
	room = kfifo_avail(&st->write_fifo);
	spin_unlock_irqrestore(&st->lock, flags);

	dev_dbg(&st->intf->dev,
		"%s - returns %d\n", __func__, room);

	return room;
}

static int dib0700_uart_chars_in_buffer(struct tty_struct *tty)
{
	struct dib0700_state *st = tty->driver_data;
	unsigned long flags;
	int chars;

	if (!st) /* TODO check */
		return -ENODEV;

	if (test_bit(STATE_DISCONNECT, &st->istate))
		return -ENODEV;

	spin_lock_irqsave(&st->lock, flags);
	chars = kfifo_len(&st->write_fifo) + st->tx_bytes;
	spin_unlock_irqrestore(&st->lock, flags);

	dev_dbg(&st->intf->dev,
		"%s - returns (%d)\n", __func__, chars);

	if (chars<0)
		pr_err("[%s] fifo [%d] tx_bytes [%d]\n",
			__func__, kfifo_len(&st->write_fifo),
			st->tx_bytes);
	return chars;
}

static int dib0700_uart_install(struct tty_driver *driver, struct tty_struct *tty)
{
	int idx = tty->index;
	struct dib0700_state *st = dib0700_uart_port_get(idx);
	int ret = -ENODEV;

	if(!st)
		goto no_dev;

	FUNC_IN;
	set_bit(STATE_TTY_INSTALL, &st->istate);

	ret = tty_standard_install(driver, tty);

	if (ret == 0) {
		tty->driver_data = st;
	}
	else
		dev_err(&st->intf->dev, "%s: failed\n", __func__);
no_dev:
	FUNC_OUT;
	return ret;
}

static void dib0700_uart_cleanup(struct tty_struct *tty)
{
	struct dib0700_state *st = tty->driver_data;

	FUNC_IN;
	tty->driver_data = NULL;
	if (test_and_clear_bit(STATE_TTY_INSTALL, &st->istate))
		dib0700_mfd_put(st);
	FUNC_OUT;
}

static void dib0700_uart_set_termios(struct tty_struct *tty,
					struct ktermios *old_termios)
{
	// 31		24		         16	12	8	0
	// ...................................................................
	//  request type      | parity   | tty enable  | baud rate (MSB)
	//  (8 bits)             (4 bits)     (4 bits)       (16 bits)
	//
	//  31                                                               0
	//  ..................................................................
	//                            baud rate (LSB)  |
	//                            (16 bits)

	struct dib0700_state *st = tty->driver_data;
	unsigned long flags;
	unsigned int cflag = tty->termios->c_cflag, expires_ms = 0;
	int result = 0;
	speed_t baud = 0;
	u8 buf[6] = {0};

	FUNC_IN;

	if (!st) /* probably disconnected */
		return;

	if (!tty_termios_hw_change(old_termios,
			tty->termios))
			return; /* no change */

	spin_lock_irqsave(&st->lock, flags);

	if (test_bit(STATE_TTY_ENABLE, &st->istate)) {
		baud = tty_termios_baud_rate(tty->termios);
		if (baud == 0)
			baud = 9600;

		expires_ms = RX_TIMER_CONST/baud;
		st->rx_expires = msecs_to_jiffies(expires_ms);

		expires_ms = TX_TIMER_CONST/baud;
		st->tx_expires = msecs_to_jiffies(expires_ms);
	}
	spin_unlock_irqrestore(&st->lock, flags);

	buf[0] = REQUEST_UART_SET_PARAM;
	buf[1] = (cflag & PARENB) << 5
			| (cflag & PARODD) << 4; /* parity */

	if (serial_debug) {
		/* Activate serial loopback in the firmware */
		buf[1] |= 0x80;
	}

	if (test_bit(STATE_TTY_ENABLE, &st->istate))
		buf[1] |= 0x7; /* UART enabling */

	memcpy(&buf[2], (void*) &baud, 4);	/* baud rate */

	result = dib0700_usb_control_msg(st,
			0,
			REQUEST_UART_SET_PARAM,
			USB_TYPE_VENDOR | USB_DIR_OUT,
			0, 0, buf, ARRAY_SIZE(buf),
			USB_CTRL_GET_TIMEOUT);

	//Debug
	/*print_hex_dump(KERN_INFO, "URB termios: ",
				DUMP_PREFIX_NONE,
				16, 1, buf,
				6, 0);*/

	if (result < 0)
		dev_err(&st->intf->dev, "UART settings failed (status = %d)\n", result);

	//Debug
	if (dvb_usb_dib0700_debug >= 0x06)
		dev_err(&st->intf->dev, "USB control write: %x %x %x %x %x %x %d = %d\n",
				0, REQUEST_UART_SET_PARAM,
				USB_TYPE_VENDOR | USB_DIR_OUT,
				0, 0,
				6, USB_CTRL_GET_TIMEOUT, result);

	FUNC_OUT;
}


static const struct tty_operations dib0700_uart_ops = {
	.open			= dib0700_uart_open,
	.close			= dib0700_uart_close,
	.write			= dib0700_uart_write,
	.write_room		= dib0700_uart_write_room,
	.chars_in_buffer	= dib0700_uart_chars_in_buffer,
	.install		= dib0700_uart_install,
	.cleanup		= dib0700_uart_cleanup,
	.set_termios		= dib0700_uart_set_termios,
};
#endif //UART_DEV_PENDING

/***********************
	Misc
************************/
/*
   Parse module argument
   example : modprobe dib0700_mfd  i2c_param="(3:5),(7:5),(2:5)"
*/

static void dib0700_destroy_ressources(struct kref *kref)
{
	struct dib0700_state *st;

	st = kref_to_dib0700_state(kref);

#ifdef UART_DEV_PENDING
	/* Free URBs */
	free_urbs(st);

	kfifo_free(&st->write_fifo);
#endif //UART_DEV_PENDING

	/* release ref to usb device */
	usb_put_dev(st->udev);
	st->udev = NULL;

	pr_debug("[%s][%d]\n", __func__, __LINE__);
}


static void __dib0700_mfd_put(struct dib0700_state *st)
{
	kref_put(&st->kref, dib0700_destroy_ressources);
}

static int dib0700_parse_i2c_param(struct dib0700_state *st)
{
	struct usb_device *udev = st->udev;
	char *end, *s1, *s2 = i2c_param;
	int val;

	while (s2 != NULL) {

		st->props[st->num_adapters].master = 0;

		s1 = strchr(s2, '(');
		if (s1) {
			val = simple_strtoul(++s1, &end, 0);
			if (*end != ':')
				return -EINVAL;

			//dev_err(&udev->dev, "sda : \"%d\"\n", val);

			if (val < ARRAY_SIZE(gpio_to_reg)) {
				st->props[st->num_adapters].sda =
					gpio_to_reg[val];
			}

			s1 = ++end;

			val = simple_strtoul(s1, &end, 0);
			if (*end != ')')
				return -EINVAL;

			//dev_err(&udev->dev, "scl : \"%d\"\n", val);

			if (val < ARRAY_SIZE(gpio_to_reg)) {
				st->props[st->num_adapters++].scl =
					gpio_to_reg[val];
			}

			s2 = ++end;

		} else {
			return -EINVAL;
		}

		if (!strchr(s2, ',')) {
			return 0;
		}
	}


	dev_err(&udev->dev, "There is no parameter\n");
	return 0;
}

/* expecting rx buffer: request data[0] data[1] ... data[2] */
static int dib0700_ctrl_wr(struct dib0700_state *st, u8 *tx, u8 txlen)
{
	int status;
	status = dib0700_usb_control_msg(st, 0,
		tx[0], USB_TYPE_VENDOR | USB_DIR_OUT, 0, 0, tx, txlen,
		USB_CTRL_GET_TIMEOUT);

	if (status != txlen)
		deb_data("ep 0 write error (status = %d, len: %d)\n",status,txlen);

	return status < 0 ? status : 0;
}

static int dib0700_set_gpio(struct dib0700_state *st, enum dib07x0_gpios gpio, u8 gpio_dir, u8 gpio_val)
{
	int ret;
	u8 buf[3] = {0};


	buf[0] = REQUEST_SET_GPIO;
	buf[1] = gpio;
	buf[2] = ((!!gpio_dir) << 7) | ((!!gpio_val) << 6);

	ret = dib0700_ctrl_wr(st, buf, ARRAY_SIZE(buf));

	return ret;
}

/* Gpiolib interface */
static void dib0700_gpio_set(struct gpio_chip *c, unsigned gpio, int v)
{
	struct dib0700_state *st = container_of(c,
						struct dib0700_state,
						gpio_chip);

	if (gpio >= ARRAY_SIZE(gpio_to_reg)) {
		dev_err(&st->intf->dev, "Error: GPIO %u out of range", gpio);
		return;
	}

	/* We should split value and direction in different functions */
	(void)dib0700_set_gpio(st, gpio_to_reg[gpio], 1, v);
}

static int dib0700_gpio_direction_output(struct gpio_chip *c, unsigned gpio, int v)
{
	struct dib0700_state *st = container_of(c,
						struct dib0700_state,
						gpio_chip);

	if (gpio >= ARRAY_SIZE(gpio_to_reg)) {
		dev_err(&st->intf->dev, "Error: GPIO %u out of range", gpio);
		return -1;
	}

	return dib0700_set_gpio(st, gpio_to_reg[gpio], 1, v);
}

int dib0700_gpio_init(struct dib0700_state *st)
{
	int ret;

	st->gpio_chip.set = dib0700_gpio_set;
	st->gpio_chip.direction_output = dib0700_gpio_direction_output;
	st->gpio_chip.can_sleep = true;
	// Let gpiolib find a base for us
	st->gpio_chip.base = -1;
	st->gpio_chip.label = DRIVER_NAME;
	st->gpio_chip.ngpio = ARRAY_SIZE(gpio_to_reg);
	st->gpio_chip.dev = &static_hook.dev;

	ret = gpiochip_add(&st->gpio_chip);

	if (ret) {
		dev_err(&st->intf->dev,
			"Couldn't register GPIO chip: %d\n",
			ret);
		return ret;
	}

	dev_info(&st->intf->dev, "GPIO chip registered\n");

	return 0;
}

static void dib0700_gpio_release(struct dib0700_state *st)
{
	int i;
	int ret;
	dev_info(&st->intf->dev, "GPIO chip is being removed\n");

	/* XXX this is ugly but we don't have choice : we can't keep gpio
	   driver open while the device is removed because
	 */
	for (i = st->gpio_chip.base; i < st->gpio_chip.base + st->gpio_chip.ngpio;
			i++) {
		gpio_free(i);
	}

	ret = gpiochip_remove(&st->gpio_chip);
	if (ret < 0) {
		dev_info(&st->intf->dev, "GPIO cannot be removed %d\n", ret);
	}
}

/***********************
	I2C stuff
************************/
#define I2C_WR_OFFSET 6

static int dib0700_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msg,
				int num)
{
	/* The new i2c firmware messages are more reliable and in particular
	   properly support i2c read calls not preceded by a write */

	struct dib0700_state *st = i2c_get_adapdata(adap);
	struct i2c_adapter_properties *prop
			= container_of(adap, struct i2c_adapter_properties, i2c_adap);

	uint8_t bus_mode = !prop->master; /* 0=eeprom bus, 1=frontend bus */
	uint8_t gen_mode = !prop->master; /* 0=master i2c, 1=gpio i2c */
	uint8_t en_start = 0;
	uint8_t en_stop = 0;
	int result, i;
	uint8_t buf[256 + I2C_WR_OFFSET] = {0};

	if (test_bit(STATE_DISCONNECT, &st->istate))
		return -ENODEV;

	rt_mutex_lock(&st->bus_lock);
	/* Acces to i2c is protected by a mutex in i2c-core.c */

	for (i = 0; i < num; i++) {
		if (i == 0) {
			/* First message in the transaction */
			en_start = 1;
		} else if (!(msg[i].flags & I2C_M_NOSTART)) {
			/* Device supports repeated-start */
			en_start = 1;
		} else {
			/* Not the first packet and device doesn't support
			   repeated start */
			en_start = 0;
		}
		if (i == (num - 1)) {
			/* Last message in the transaction */
			en_stop = 1;
		}

		if (msg[i].flags & I2C_M_RD) {

			/* Read request */
			u16 index, value;
			uint8_t i2c_dest;

			i2c_dest = (msg[i].addr << 1);
			value = ((en_start << 7) | (en_stop << 6)
				| ((uint8_t)prop->sda)) << 8
				| i2c_dest;

			/* I2C ctrl + FE bus; */
			index = ((gen_mode << 6) & 0xC0) |
				((bus_mode << 4) & 0x30) |
				(uint8_t)prop->scl;


			result = dib0700_usb_control_msg(st,
						 0,
						 REQUEST_NEW_I2C_READ,
						 USB_TYPE_VENDOR | USB_DIR_IN,
						 value, index, msg[i].buf,
						 msg[i].len,
						 USB_CTRL_GET_TIMEOUT);

			if (dvb_usb_dib0700_debug >= 0x06) {
				dev_err(&adap->dev, "USB control read: %x %x %x %x %x %x %d = %d\n",
					0,
					REQUEST_NEW_I2C_READ,
					USB_TYPE_VENDOR | USB_DIR_IN,
					value, index,
					msg[i].len,
					USB_CTRL_GET_TIMEOUT,
					result);
			}

			if (result != msg[i].len) {
				deb_info("i2c read error (status = %d)\n", result);
				result = -EREMOTEIO;
				goto exit_i2c_xfer;
			}

			debug_dump_usb_xfer(st, "I2C read",
					msg[i].buf, msg[i].len);

		} else {
			buf[0] = REQUEST_NEW_I2C_WRITE;
			buf[1] = msg[i].addr << 1;
			buf[2] = (en_start << 7) | (en_stop << 6) |
				(msg[i].len & 0x3F);
			/* I2C ctrl + FE bus; */
			buf[3] = ((gen_mode << 6) & 0xC0) |
				 ((bus_mode << 4) & 0x30);

			buf[4] = ((uint8_t)prop->sda) << 4
					| (uint8_t)prop->scl;

			if ( msg[i].len + I2C_WR_OFFSET >
				ARRAY_SIZE(buf)) {
				dev_err(&adap->dev,
					"msg[i].len = %d\n", msg[i].len);
				dib0700_mfd_put(st);
				return -EINVAL;
			}

			/* The Actual i2c payload */
			memcpy(&buf[I2C_WR_OFFSET], msg[i].buf, msg[i].len);

			result = dib0700_usb_control_msg(st,
						 0,
						 REQUEST_NEW_I2C_WRITE,
						 USB_TYPE_VENDOR | USB_DIR_OUT,
						 0, 0, buf, msg[i].len + I2C_WR_OFFSET,
						 USB_CTRL_GET_TIMEOUT);

			if (dvb_usb_dib0700_debug >= 0x06) {
				dev_err(&adap->dev, "USB control write: %x %x %x %x %x %x %d = %d\n",
					0,
					REQUEST_NEW_I2C_WRITE,
					USB_TYPE_VENDOR | USB_DIR_OUT,
					0, 0, msg[i].len + I2C_WR_OFFSET,
					USB_CTRL_GET_TIMEOUT, result);
			}

			debug_dump_usb_xfer(st, "I2C write ",
					buf, msg[i].len + I2C_WR_OFFSET);

			if (result != msg[i].len + I2C_WR_OFFSET) {
				deb_info("i2c write error (status = %d)\n", result);
				result = -EREMOTEIO;
				goto exit_i2c_xfer;
			}
		}
	}


	result = i;

exit_i2c_xfer:
	rt_mutex_unlock(&st->bus_lock);
	return result;
}

static u32 dib0700_i2c_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static struct i2c_algorithm dib0700_i2c_algo = {
	.master_xfer   = dib0700_i2c_xfer,
	.functionality = dib0700_i2c_func,
};

#if 0
static int dib0700_set_i2c_speed(struct dib0700_state *st, u16 scl_kHz)
{
	u16 divider;
	int ret;
	u8 buf[8];

	if (scl_kHz == 0)
		return -EINVAL;

	buf[0] = REQUEST_SET_I2C_PARAM;
	divider = (u16) (30000 / scl_kHz);
	buf[1] = 0;
	buf[2] = (u8) (divider >> 8);
	buf[3] = (u8) (divider & 0xff);
	divider = (u16) (72000 / scl_kHz);
	buf[4] = (u8) (divider >> 8);
	buf[5] = (u8) (divider & 0xff);
	divider = (u16) (72000 / scl_kHz); /* clock: 72MHz */
	buf[6] = (u8) (divider >> 8);
	buf[7] = (u8) (divider & 0xff);

	ret = dib0700_ctrl_wr(st, buf, ARRAY_SIZE(buf));

	deb_info("setting I2C speed: %04x %04x %04x (%d kHz).",
		(buf[2] << 8) | (buf[3]), (buf[4] << 8) |
		buf[5], (buf[6] << 8) | buf[7], scl_kHz);

	return ret;
}
#endif

/***********************
	Firmware stuff
************************/
static int usb_get_hexline(const struct firmware *fw, struct hexline *hx,
			       int *pos)
{
	u8 *b = (u8 *) &fw->data[*pos];
	int data_offs = 4;
	if (*pos >= fw->size)
		return 0;

	memset(hx,0,sizeof(struct hexline));

	hx->len  = b[0];

	if ((*pos + hx->len + 4) >= fw->size)
		return -EINVAL;

	hx->addr = b[1] | (b[2] << 8);
	hx->type = b[3];

	if (hx->type == 0x04) {
		/* b[4] and b[5] are the Extended linear address record data field */
		hx->addr |= (b[4] << 24) | (b[5] << 16);
/*		hx->len -= 2;
		data_offs += 2; */
	}
	memcpy(hx->data,&b[data_offs],hx->len);
	hx->chk = b[hx->len + data_offs];

	*pos += hx->len + 5;

	return *pos;
}


static int dib0700_jumpram(struct usb_device *udev, u32 address)
{
	int ret = 0, actlen;
	u8 *buf;

	buf = kmalloc(8, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	buf[0] = REQUEST_JUMPRAM;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	buf[4] = (address >> 24) & 0xff;
	buf[5] = (address >> 16) & 0xff;
	buf[6] = (address >> 8)  & 0xff;
	buf[7] =  address        & 0xff;

	if ((ret = usb_bulk_msg(udev, usb_sndbulkpipe(udev, 0x01),
				buf,8,&actlen,USB_CTRL_TMOUT)) < 0) {
		dev_err(&udev->dev, "jumpram to 0x%x failed\n",address);
		goto out;
	}
	if (actlen != 8) {
		dev_err(&udev->dev, "jumpram to 0x%x failed\n",address);
		ret = -EIO;
		goto out;
	}
out:
	kfree(buf);
	return ret;
}


static int dib0700_download_firmware(struct usb_device *udev, const struct firmware *fw)
{
	struct hexline hx;
	int pos = 0, ret, act_len;
	u8 *buf;

	buf = kmalloc(260, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	while ((ret = usb_get_hexline(fw, &hx, &pos)) > 0) {
		//dev_err(&udev->dev, "writing to address 0x%08x (buffer: 0x%02x %02x)\n",
		//		hx.addr, hx.len, hx.chk);
		deb_fwdata("writing to address 0x%08x (buffer: 0x%02x %02x)\n",
				hx.addr, hx.len, hx.chk);

		buf[0] = hx.len;
		buf[1] = (hx.addr >> 8) & 0xff;
		buf[2] =  hx.addr       & 0xff;
		buf[3] = hx.type;
		memcpy(&buf[4],hx.data,hx.len);
		buf[4+hx.len] = hx.chk;

		ret = usb_bulk_msg(udev,
			usb_sndbulkpipe(udev, 0x01),
			buf,
			hx.len + 5,
			&act_len,
			USB_CTRL_TMOUT);

		if (ret < 0) {
			dev_err(&udev->dev, "firmware download failed at %d with %d",pos,ret);
			goto out;
		}

	}

	if (ret == 0) {
		/* start the firmware */
		if ((ret = dib0700_jumpram(udev, 0x70000000)) == 0) {
			dev_info(&udev->dev, "Firmware started successfully\n");
			msleep(200);
		}
	} else
		ret = -EIO;

out:
	kfree(buf);
	return ret;
}

static int dib0700_get_version(struct dib0700_state *st)
{
	int ret;
	unsigned long flags;
	u8 buf[16] = {0};

	ret = usb_control_msg(st->udev, usb_rcvctrlpipe(st->udev, 0),
				  REQUEST_GET_VERSION,
				  USB_TYPE_VENDOR | USB_DIR_IN, 0, 0,
				  buf, 16, USB_CTRL_GET_TIMEOUT);

	spin_lock_irqsave(&st->lock, flags);

	st->hwversion  = (buf[0] << 24)  | (buf[1] << 16)  |
		(buf[2] << 8)  | buf[3];

	st->romversion = (buf[4] << 24)  | (buf[5] << 16)  |
		(buf[6] << 8)  | buf[7];

	st->fw_version = (buf[8] << 24)  | (buf[9] << 16)  |
		(buf[10] << 8) | buf[11];

	st->fwtype     = (buf[12] << 24) | (buf[13] << 16) |
		(buf[14] << 8) | buf[15];

	spin_unlock_irqrestore(&st->lock, flags);
	return ret;
}

/***********************************************/
#ifdef UART_DEV_PENDING

static void dib0700_serial_release(struct dib0700_state *st)
{
	int index;

	FUNC_IN;

	/* Deactivate the timer */
	del_timer_sync(&st->rx_timer);
	del_timer_sync(&st->tx_timer);

	for (index = 0; index < MAX_NBR_UARTS; index++) {
		if (dib0700_uart_table[index]
		&& (dib0700_uart_table[st->index] == st)) {
			dib0700_uart_port_remove(st);
		}
	}

	/* Kill URBs */
	kill_urbs(st);
	cancel_work_sync(&st->work);

	FUNC_OUT;
}

static int dib0700_serial_init(struct dib0700_state *st)
{
	int err = -ENOMEM, index;
	struct usb_host_interface *intf_desc;

	/* Look for available space for tty registration */
	spin_lock(&dib0700_uart_table_lock);
	for (index = 0; index < MAX_NBR_UARTS; index++) {
		if (!dib0700_uart_table[index]) {
			st->index = index;
			dib0700_uart_table[index] = st;
			break;
		}
	}
	spin_unlock(&dib0700_uart_table_lock);

	intf_desc = st->intf->cur_altsetting;
	tty_port_init(&st->port);
	st->port.ops = &dib0700_uart_port_ops;
	spin_lock_init(&st->lock);
	INIT_WORK(&st->work, dib0700_port_work);

	if (index<MAX_NBR_UARTS) {
		st->ttydev = tty_register_device(dib0700_uart_tty_driver,
						st->index, &st->intf->dev);
		if (IS_ERR(st->ttydev)) {
			dib0700_uart_port_remove(st);
			dev_info(&st->intf->dev, "TTY registration failed\n");
			err = PTR_ERR(st->ttydev);
			goto serial_init_failed;
		} else
			dev_info(&st->intf->dev, "TTY registration succeed\n");

		set_bit(STATE_TTY_REGISTER, &st->istate);

		/* Setup timer */
		setup_timer(&st->rx_timer, rx_timer_routine, (unsigned long) st);
		setup_timer(&st->tx_timer, tx_timer_routine, (unsigned long) st);

		/* allocate and initialize URBs */
		st->rd_ctrlrq.bRequest = REQUEST_UART_READ;
		st->rd_ctrlrq.bRequestType = USB_TYPE_VENDOR | USB_DIR_IN;
		st->rd_ctrlrq.wValue = 0;
		st->rd_ctrlrq.wIndex = 0;
		st->rd_ctrlrq.wLength = cpu_to_le16(RD_URB_SIZE);

		st->wrt_ctrlrq.bRequest = REQUEST_UART_WRITE;
		st->wrt_ctrlrq.bRequestType = USB_TYPE_VENDOR
						| USB_DIR_OUT;
		st->wrt_ctrlrq.wValue = 0;
		st->wrt_ctrlrq.wIndex = 0;
		st->wrt_ctrlrq.wLength
			= cpu_to_le16(WR_URB_SIZE + SRL_WRITE_OFFSET);

		for (index = 0; index < NBR_MAX_RD_URBS; index++) {
			/* similar to usb-serial */
			set_bit(index, &st->read_urbs_free);

			st->read_urbs[index] = usb_alloc_urb(0, GFP_KERNEL);
			if (!st->read_urbs[index]) {
				dev_err(&st->intf->dev,
						"No free urbs available\n");
				goto serial_init_failed;
			}
			st->uart_in_buffers[index] = kmalloc(RD_URB_SIZE,
								GFP_KERNEL);
			if (!st->uart_in_buffers[index]) {
				dev_err(&st->intf->dev,
					"Couldn't allocate buffer for URBs\n");
				goto serial_init_failed;
			}

			st->uart_in_buffers[index][0] = REQUEST_UART_READ;

			usb_fill_control_urb(st->read_urbs[index], st->udev,
					usb_rcvctrlpipe(st->udev, 0),
					(unsigned char *)&st->rd_ctrlrq,
					st->uart_in_buffers[index],
					RD_URB_SIZE,
					dib0700_read_ctrl_callback,
					st);
		}

		for (index = 0; index < NBR_MAX_WR_URBS; index++) {
			set_bit(index, &st->write_urbs_free);

			st->write_urbs[index] = usb_alloc_urb(0, GFP_KERNEL);
			if (!st->write_urbs[index]) {
				dev_err(&st->intf->dev,
						"No free urbs available\n");
				goto serial_init_failed;
			}
			st->uart_out_buffers[index] = kmalloc(WR_URB_SIZE,
								GFP_KERNEL);
			if (!st->uart_out_buffers[index]) {
				dev_err(&st->intf->dev,
					"Couldn't allocate buffer for URBs\n");
				goto serial_init_failed;
			}

			usb_fill_control_urb(st->write_urbs[index], st->udev,
					usb_sndctrlpipe(st->udev, 0),
					(unsigned char *)&st->wrt_ctrlrq,
					st->uart_out_buffers[index],
					WR_URB_SIZE + 6,
					dib0700_write_ctrl_callback,
					st);
		}

		/* init write FIFO */
		if (kfifo_alloc(&st->write_fifo, PAGE_SIZE, GFP_KERNEL)) {
			dev_err(&st->intf->dev,
				"FIFO can't be allocated\n");
			goto serial_init_failed;
		} else
			err = 0;
	}

serial_init_failed:
	return err;
}
#endif //UART_DEV_PENDING


static void dib0700_i2c_init(struct dib0700_state *st)
{
	int idx, check = 0, loop;
	int sda = 0, scl = 0;
	struct usb_device *udev = st->udev;

	rt_mutex_init(&st->bus_lock);
	for (idx = 0; idx < st->num_adapters; idx++) {

		if( (!st->props[idx].sda)
			&& (!st->props[idx].scl)
			&& (!st->props[idx].master) )
			return;

		st->props[idx].num_adapter         = idx;
		st->props[idx].i2c_adap.owner      = THIS_MODULE;
		st->props[idx].i2c_adap.algo       = &dib0700_i2c_algo;
		st->props[idx].i2c_adap.algo_data  = NULL;
		//st->props[idx].i2c_adap.dev.parent = &st->udev->dev;
		st->props[idx].i2c_adap.dev.parent = &static_hook.dev;

		if (!st->props[idx].master) {
			for (check = 0; check < idx; check++) {

				if ( (!st->props[check].master)
				&& ((st->props[idx].sda == st->props[check].sda)
				|| (st->props[idx].sda == st->props[check].scl)
				|| (st->props[idx].scl == st->props[check].sda))) {
					dev_err(&udev->dev,
					"Pin used\n[idx=%d] : (%d,%d)\t [check=%d] : (%d,%d)\n",
					idx, st->props[idx].sda, st->props[idx].scl,
					check, st->props[check].sda, st->props[check].scl);

					break;
				}
			}
			if (check!=idx)
				continue;
		}

		/* Set pins as output */
		if (dib0700_set_gpio(st, st->props[idx].sda, 0, 1)
				|| dib0700_set_gpio(st, st->props[idx].scl, 0, 1)) {
			dev_err(&udev->dev, "can't set i2c pin as output\n");
			continue;
		}

		//TODO change the name of adapter
		for (loop = 0;
				loop < ARRAY_SIZE(gpio_to_reg);
				loop++) {

			if (st->props[idx].sda
					== gpio_to_reg[loop]) {
				sda = loop;
			} else if (st->props[idx].scl
					== gpio_to_reg[loop]) {
				scl = loop;
			}
		}

		snprintf(st->props[idx].i2c_adap.name,
				sizeof(st->props[idx].i2c_adap.name),
				DRIVER_NAME ".%d.%d", sda, scl);

		dev_info(&udev->dev, "adapter \"%s\" (%s sda:%d,scl:%d)\n",
				st->props[idx].i2c_adap.name,
				(st->props[idx].master)?
				"master":"bit banging",
				sda, scl);

		i2c_set_adapdata(&st->props[idx].i2c_adap, st);

		if (i2c_add_adapter(&st->props[idx].i2c_adap) < 0) {
			dev_err(&udev->dev, "Failed to add adapter\n");
		}

		set_bit(STATE_I2C_REGISTER, &st->istate);
	}
}


static int dib0700_hook_init(struct dib0700_state *st)
{
	int err = -1;
	const struct firmware *fw = NULL;
	struct usb_device *udev = st->udev;
	//char *firmware = "dvb-usb-dib0700-1.20.fw";

	/* Get firmware revision */
	//if ((err = dib0700_get_version(st)) < 0) {
		/* Download firmware */
		dev_info(&st->intf->dev, "Request firware \"%s\"\n", firmware);
		if ((err = request_firmware(&fw, firmware, &udev->dev)) != 0) {
			dev_err(&st->intf->dev, "Firware \"%s\" not found\n", firmware);
			goto init_failed;
		}

		/* TODO find a better solution for
			struct hexline (see dvb-usb.h) and
			dvb_usb_get_hexline/usb_get_hexline ( exported from dvb-usb-firmware.c )*/
		if ((err = dib0700_download_firmware(st->udev, fw)) !=0) {
			dev_err(&st->intf->dev, "Firware download failed\n");
			goto init_failed;
		}

		dev_info(&st->intf->dev, "Firmware downloaded\n");

		/* Get firmware revision */
		if ((err = dib0700_get_version(st)) < 0) {
			dev_err(&st->intf->dev, "Unable to get firmware version\n");
			goto init_failed;
		}

		/* Force USB descriptor update (from ROM settings, to firmware) */
		/*usb_control_msg(st->udev, usb_rcvctrlpipe(st->udev, 0),
					  REQUEST_DISCONNECT,
					  USB_TYPE_VENDOR | USB_DIR_IN, 0, 0,
					  NULL, 0, USB_CTRL_GET_TIMEOUT);
	} else {
		dev_info(&st->intf->dev, "Device in warm state\n");
	}*/

	err = 0;

init_failed:
	if (fw)
		release_firmware(fw);
	return err;
}

int dib0700_probe(struct usb_interface *intf, int *gpio_base)
{
	int err = -ENXIO;
	struct dib0700_state *st = NULL;
	if (chip_state) {
		st = chip_state;
	}
	else {
		st = kzalloc(sizeof(struct dib0700_state), GFP_KERNEL);
		if (st == NULL) {
			dev_err(&intf->dev, "Out of memory\n");
			err = -ENOMEM;
			goto error;
		}
	}

	st->intf = intf;
	/* get a ref to usb device */
	st->udev = usb_get_dev(interface_to_usbdev(intf));

	kref_init(&st->kref);
	spin_lock_init(&st->lock);
	init_waitqueue_head(&st->wq_endp0);

	usb_set_intfdata(intf, st);

	if ((err = dib0700_hook_init(st)) !=0) {
		dev_err(&intf->dev, "Init failed\n");
		goto free_mem;
	}
	dev_info(&intf->dev, "Firmware version: %x, %d, 0x%x, %d\n",
				st->hwversion, st->romversion,
				st->fw_version, st->fwtype);


	/* Define default i2c adapter */
	st->num_adapters = 0;
	st->props[0].master = 1;
	st->num_adapters++;

	/* Define i2c adapter that use bit banging */
	if (dib0700_parse_i2c_param(st)){
		dev_err(&intf->dev,
			"Bad parameter (bad i2c_param)\n");
	}

#ifdef UART_DEV_PENDING
	/* serial device */
	err = dib0700_serial_init(st);
	if (err<0) {
		dev_err(&intf->dev,
			"Serial init failed\n");
		goto free_mem;
	}
#endif //UART_DEV_PENDING

	if (!chip_state) {
		dib0700_i2c_init(st);

		err = dib0700_gpio_init(st);
		if (err  < 0) {
			dev_err(&intf->dev,
					"GPIO init failed\n");
			goto release_uart;
		}
	}
	*gpio_base = st->gpio_chip.base;

	clear_bit(STATE_DISCONNECT, &st->istate);

	chip_state = st;
	return 0;

release_uart:
#ifdef UART_DEV_PENDING
	if (test_and_clear_bit(STATE_TTY_REGISTER, &st->istate)) {
		/* Remove serial */
		dib0700_serial_release(st);
	}
#endif //UART_DEV_PENDING
free_mem:
	dib0700_mfd_put(st);
error:
	return err;
}
EXPORT_SYMBOL(dib0700_probe);

void dib0700_device_exit(struct usb_interface *intf)
{
	struct dib0700_state *st = usb_get_intfdata(intf);

	usb_set_intfdata(intf, NULL);
	set_bit(STATE_DISCONNECT, &st->istate);

	/* TODO: protect retrieval by mutex */

#ifdef UART_DEV_PENDING
	if (test_and_clear_bit(STATE_TTY_REGISTER, &st->istate)) {
		/* Remove serial */
		dib0700_serial_release(st);
	}
#endif //UART_DEV_PENDING

	dib0700_mfd_put(st);
	dev_info(&intf->dev, "driver exit\n");
}
EXPORT_SYMBOL(dib0700_device_exit);

static int __init dib0700_init(void)
{
	int ret;
	struct tty_driver *tty_drv;

	platform_device_register(&static_hook);

	dib0700_uart_tty_driver = tty_drv = alloc_tty_driver(MAX_NBR_UARTS);
	if (!tty_drv)
		return -ENOMEM;

	tty_drv->driver_name = DRIVER_NAME;
	tty_drv->name =   "ttyDIB";
	tty_drv->major = 0;
	tty_drv->minor_start = 0;
	tty_drv->type = TTY_DRIVER_TYPE_SERIAL;
	tty_drv->subtype = SERIAL_TYPE_NORMAL;
	tty_drv->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	tty_drv->init_termios = tty_std_termios;
	tty_drv->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	tty_drv->init_termios.c_ispeed = 9600;
	tty_drv->init_termios.c_ospeed = 9600;
	tty_set_operations(tty_drv, &dib0700_uart_ops);

	/* register the TTY driver */
	ret = tty_register_driver(tty_drv);
	if (ret < 0) {
		printk(KERN_ERR DRIVER_NAME "%s - tty_register_driver failed\n",
		       __func__);
		goto tty_register_failed;
	}

	return 0;

tty_register_failed:
	put_tty_driver(tty_drv);
	return ret;
}

static void __exit dib0700_exit(void)
{
	tty_unregister_driver(dib0700_uart_tty_driver);
	//TODO check memory leak if unload without being used
	put_tty_driver(dib0700_uart_tty_driver);

	/* i2c and gpio subsystem are not robust to removal while in
	   use, don't remove them */
	if (chip_state) {
		if (test_and_clear_bit(STATE_I2C_REGISTER, &chip_state->istate)) {
			int index;
			for (index = 0; index < chip_state->num_adapters; index++)
				i2c_del_adapter(&chip_state->props[index].i2c_adap);
		}

		/* this is safe to call here if we don't
		   delete st->gpio_chip */
		dib0700_gpio_release(chip_state);

		kfree(chip_state);
		chip_state = NULL;
	}
}

module_init(dib0700_init);
module_exit(dib0700_exit);


MODULE_AUTHOR("Christian ROSALIE <christian.rosalie@parrot.com>");
MODULE_DESCRIPTION("Driver for multi-function devices based on DiBcom DiB0700 - USB bridge");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
