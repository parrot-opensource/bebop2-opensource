/*
 * @file linux/drivers/char/parrot5.c
 * @brief Driver for Parrot5/5+/6 UARTs
 *
 * Copyright (C) 2007 Parrot S.A.
 *
 * @author     ivan.djelic@parrot.com
 * @date       2007-07-11
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

#if defined(CONFIG_SERIAL_PARROTX_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>

#include <linux/io.h>
#include <asm/sizes.h>

#include <mach/regs-uart.h>
#include "px-uart.h"

#define PARROT5_SERIAL_NAME	       "ttyPA"
#define PARROT5_SERIAL_MAJOR	       204
#define PARROT5_SERIAL_MINOR	       64

#define UART_TRX_DUMMY_RX	       (1 << 16)
#define UART_NR                        8

/* module parameter to enable/disable low latency */
static bool low_latency = 0;
module_param(low_latency, bool, S_IRUGO);
MODULE_PARM_DESC(low_latency, "UART low latency enable/disable");

/*
 * We wrap our port structure around the generic uart_port.
 */
struct uart_parrot5_port {
	struct uart_port	    port;
	u32                     spd;
	u32                     lcr;
	u32                     lcr_fifo;
	unsigned int            tx_fifosize;
	unsigned int            tx_threshold;
	struct clk             *uart_clk;
	struct pinctrl         *pctl;
};

/* port locked, interrupts locally disabled */
static void parrot5_serial_start_tx(struct uart_port *port)
{
	__raw_writel(UART_ITX_TIM, port->membase+_UART_ITX);
}

/* port locked, interrupts locally disabled */
static void parrot5_serial_stop_tx(struct uart_port *port)
{
	/* writing 0 to RSTBRK disables TX interrupts */
	__raw_writel(0, port->membase+_UART_RSTBRK);
}

/* port locked, interrupts locally disabled */
static void parrot5_serial_stop_rx(struct uart_port *port)
{
	__raw_writel(0, port->membase+_UART_IRX);
}

/* port locked, interrupts locally disabled */
static void parrot5_serial_enable_ms(struct uart_port *port)
{
	/* nothing to do */
}

static void parrot5_serial_rx_chars(struct uart_port *port, u32 status)
{
	struct tty_struct *tty = port->state->port.tty;
	unsigned int c, flag;

	while (status & UART_STATUS_RXFILLED) {

		c = __raw_readl(port->membase+_UART_TRX) | UART_TRX_DUMMY_RX;
		flag = TTY_NORMAL;
		port->icount.rx++;

		if (unlikely(c & UART_TRX_ANY_ERROR)) {

			if (c & UART_TRX_RXBREAK) {
				port->icount.brk++;
				if (uart_handle_break(port)) {
					goto ignore_char;
				}
			}
			if (c & UART_TRX_PARITY_ERROR) {
				port->icount.parity++;
			}
			if (c & UART_TRX_FRAME_ERROR) {
				port->icount.frame++;
			}
			if (c & UART_TRX_OVERRUN) {
				port->icount.overrun++;
			}

			c &= port->read_status_mask;

			if (c & UART_TRX_RXBREAK) {
				flag = TTY_BREAK;
			}
			else if (c & UART_TRX_PARITY_ERROR) {
				flag = TTY_PARITY;
			}
			else if (c & UART_TRX_FRAME_ERROR) {
				flag = TTY_FRAME;
			}
		}

		if (uart_handle_sysrq_char(port, c & 0xff)) {
			goto ignore_char;
		}
		uart_insert_char(port, c, UART_TRX_OVERRUN, c, flag);

	ignore_char:
		status = __raw_readl(port->membase+_UART_STATUS);
	}

	/* tty_flip_buffer_push doc say we can't call this from interrupt
	 * context, but other drivers do it
	 */
	WARN_ON_ONCE(tty->low_latency == 1);

	spin_unlock(&port->lock);
	tty_flip_buffer_push(tty);
	spin_lock(&port->lock);
}

static void parrot5_serial_tx_chars(struct uart_port *port, u32 status)
{
	int count;
	struct circ_buf *xmit = &port->state->xmit;
	struct uart_parrot5_port *up = (struct uart_parrot5_port *)port;

	if (port->x_char) {
		__raw_writel(port->x_char, port->membase+_UART_TRX);
		port->icount.tx++;
		port->x_char = 0;
		goto out;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		parrot5_serial_stop_tx(port);
		goto out;
	}

	count = up->tx_fifosize;

	/* on Parrot5+ chips TX fifo has a configurable IRQ threshold */
	if (up->tx_threshold && (status & UART_STATUS_TXFILLED)) {
		count -= up->tx_threshold;
	}

	while (!uart_circ_empty(xmit) && (count-- > 0)) {

		__raw_writel(xmit->buf[xmit->tail], port->membase+_UART_TRX);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
		uart_write_wakeup(port);
	}
	if (uart_circ_empty(xmit)) {
		parrot5_serial_stop_tx(port);
	}

out:
	return;
}

static irqreturn_t parrot5_serial_int(int irq, void *dev_id)
{
	u32 status;
	struct uart_port *port = dev_id;

	spin_lock(&port->lock);

	status = __raw_readl(port->membase+_UART_STATUS);

	do {
		if (status & UART_STATUS_ITRX) {
			parrot5_serial_rx_chars(port, status);
		}
		if (status & UART_STATUS_ITTX) {
			parrot5_serial_tx_chars(port, status);
		}
		status = __raw_readl(port->membase+_UART_STATUS);

	} while (status & (UART_STATUS_ITRX|UART_STATUS_ITTX));

	spin_unlock(&port->lock);

	return IRQ_HANDLED;
}

/* port unlocked, interrupts status is called dependant */
static unsigned int parrot5_serial_tx_empty(struct uart_port *port)
{
	u32 status = __raw_readl(port->membase+_UART_STATUS);
	return (status & UART_STATUS_TXFILLED)? 0 : TIOCSER_TEMT;
}

/* port locked, interrupts locally disabled */
static unsigned int parrot5_serial_get_mctrl(struct uart_port *port)
{
	unsigned int result;
	u32 status;

	status = __raw_readl(port->membase+_UART_STATUS);
	result = TIOCM_CAR | TIOCM_DSR;

	/* pretend CTS is always on, otherwise we would have to dynamically
	   report CTS changes, which is useless anyway since everything is done
	   in HW */
	if (1 /* !(status & UART_STATUS_CTSN) */) {
		result |= TIOCM_CTS;
	}
	return result;
}

/* port locked, interrupts locally disabled */
static void parrot5_serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	u32 lcr = __raw_readl(port->membase+_UART_LCR);

	/* allow manual control of nRTS pin */
	if (mctrl & TIOCM_RTS) {
		/* restore automatic nRTS control */
		lcr &= ~UART_LCR_RTS_CTRL;
	}
	else {
		/* force nRTS to 1, i.e. throttle */
		lcr |= UART_LCR_RTS_CTRL;
	}
	__raw_writel(lcr, port->membase+_UART_LCR);
}

/* port unlocked, interrupts status is called dependant */
static void parrot5_serial_break_ctl(struct uart_port *port, int break_state)
{
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	if (break_state) {
		/* special write: any value will do */
		__raw_writel(0, port->membase+_UART_SETBRK);
	}
	else {
		/* special write: any value will do */
		__raw_writel(0, port->membase+_UART_RSTBRK);
	}

	spin_unlock_irqrestore(&port->lock, flags);
}

/* port_sem locked, interrupts globally disabled */
static int parrot5_serial_startup(struct uart_port *port)
{
	int ret;
	struct uart_parrot5_port *up = (struct uart_parrot5_port *)port;

	/* allocate IRQ */
	ret = request_irq(port->irq,
	                  parrot5_serial_int,
	                  0,
	                  PXUART_DRV_NAME,
	                  port);
	if (ret)
		return ret;

	/* stop UART */
	__raw_writel(0, port->membase+_UART_SPD);

	/* write configuration */
	__raw_writel(up->lcr, port->membase+_UART_LCR);

	spin_lock_irq(&port->lock);

	/* start UART */
	__raw_writel(up->spd, port->membase+_UART_SPD);

	/* enable interrupts */
	__raw_writel(UART_IRX_RIM, port->membase+_UART_IRX);
        __raw_writel(UART_ITX_TIM, port->membase+_UART_ITX);

	spin_unlock_irq(&port->lock);

	return 0;
}

/* port_sem locked, interrupts status is caller dependent */
static void parrot5_serial_shutdown(struct uart_port *port)
{
	/* disable all interrupts and reset break */
	spin_lock_irq(&port->lock);
	__raw_writel(0, port->membase+_UART_IRX);
	__raw_writel(0, port->membase+_UART_RSTBRK);
	spin_unlock_irq(&port->lock);

	/* disable the port */
	__raw_writel(0, port->membase+_UART_SPD);

	/* free the interrupt */
	free_irq(port->irq, port);
}

static void parrot5_serial_set_termios(struct uart_port *port,
				       struct ktermios *termios,
				       struct ktermios *old)
{
	u32 lcr;
	unsigned long flags;
	unsigned int baud, csize;
	struct uart_parrot5_port *up = (struct uart_parrot5_port *)port;

	/*
	 * We don't support modem control lines.
	 */
	termios->c_cflag &= ~(HUPCL | CMSPAR);
	termios->c_cflag |= CLOCAL;

	/*
	 * Ask the core to calculate the baudrate for us.
	 */
	baud = uart_get_baud_rate(port, termios, old,
				  port->uartclk >> 20,
				  port->uartclk >> 4);

	if (baud) {
		up->spd = (port->uartclk+(baud/2))/baud;
	}

	/* we only support CS7 and CS8 */
	csize = termios->c_cflag & CSIZE;

	if ((csize != CS7) && (csize != CS8)) {
		/* default to CS8 */
		csize = (old && ((old->c_cflag & CSIZE) == CS7))? CS7 : CS8;
		termios->c_cflag &= ~CSIZE;
		termios->c_cflag |= csize;
	}
	lcr = (csize == CS8)? UART_LCR_EIGHTBITS : 0;

	if (termios->c_cflag & CSTOPB) {
		lcr |= UART_LCR_TWOSTOP;
	}
	if (termios->c_cflag & PARENB) {
		lcr |= UART_LCR_PARITY;
		if (!(termios->c_cflag & PARODD)) {
			lcr |= UART_LCR_EVEN;
		}
	}
	if (termios->c_cflag & CRTSCTS) {
		lcr |= UART_LCR_CTSMODE;
	}
	if (port->fifosize > 1) {
		lcr |= up->lcr_fifo;
	}

	/* change port with interrupts disabled */
	spin_lock_irqsave(&port->lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	port->read_status_mask = UART_TRX_OVERRUN | 0xff;

	if (termios->c_iflag & INPCK) {
		port->read_status_mask |=
			UART_TRX_FRAME_ERROR | UART_TRX_PARITY_ERROR;
	}
	if (termios->c_iflag & (BRKINT | PARMRK)) {
		port->read_status_mask |= UART_TRX_RXBREAK;
	}

	/*
	 * Characters to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR) {
		port->ignore_status_mask |=
			UART_TRX_FRAME_ERROR | UART_TRX_PARITY_ERROR;
	}
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= UART_TRX_RXBREAK;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR) {
			port->ignore_status_mask |= UART_TRX_OVERRUN;
		}
	}

	/*
	 * Ignore all characters if CREAD is not set.
	 */
	if ((termios->c_cflag & CREAD) == 0) {
		port->ignore_status_mask |= UART_TRX_DUMMY_RX;
	}

	up->lcr = lcr;

	/* set baud rate */
	__raw_writel(up->spd, port->membase+_UART_SPD);

	/* configure port */
	__raw_writel(up->lcr, port->membase+_UART_LCR);

	spin_unlock_irqrestore(&port->lock, flags);
}

static void parrot5_pm(struct uart_port *port,
                       unsigned int state,
                       unsigned int oldstate)
{
	struct uart_parrot5_port *up = (struct uart_parrot5_port *)port;

	if (! state)
		clk_prepare_enable(up->uart_clk);
	else
		clk_disable_unprepare(up->uart_clk);
}

static const char *parrot5_serial_type(struct uart_port *port)
{
	return (port->type == PORT_PARROT5)? "PARROT5" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'
 */
static void parrot5_serial_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, SZ_4K);
}

/*
 * Request the memory region(s) being used by 'port'
 */
static int parrot5_serial_request_port(struct uart_port *port)
{
	struct resource *res;

	res = request_mem_region(port->mapbase, SZ_4K, PXUART_DRV_NAME);

	return (res != NULL)? 0 : -EBUSY;
}

/*
 * Configure/autoconfigure the port.
 */
static void parrot5_serial_config_port(struct uart_port *port, int flags)
{
	if ((flags & UART_CONFIG_TYPE) &&
	    (parrot5_serial_request_port(port) == 0)) {
		port->type = PORT_PARROT5;
	}
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int parrot5_serial_verify_port(struct uart_port *port,
				      struct serial_struct *ser)
{
	int ret = 0;

	if ((ser->type != PORT_UNKNOWN) && (ser->type != PORT_PARROT5)) {
		ret = -EINVAL;
	}

	return ret;
}

#ifdef CONFIG_CONSOLE_POLL
static int parrot5_serial_poll_get_char(struct uart_port *port)
{
	if (!(__raw_readl(port->membase+_UART_STATUS) &
		  UART_STATUS_RXFILLED))
		return NO_POLL_CHAR;

	return __raw_readl(port->membase+_UART_TRX) & 0xff;
}

static void parrot5_serial_poll_put_char(struct uart_port *port, unsigned char ch)
{
	while (__raw_readl(port->membase+_UART_STATUS) &
		   UART_STATUS_TXFILLED)
		cpu_relax();

	__raw_writel(ch & 0xff, port->membase+_UART_TRX);
}
#endif

static struct uart_ops parrot5_serial_pops = {
	.tx_empty	= parrot5_serial_tx_empty,
	.set_mctrl	= parrot5_serial_set_mctrl,
	.get_mctrl	= parrot5_serial_get_mctrl,
	.stop_tx	= parrot5_serial_stop_tx,
	.start_tx	= parrot5_serial_start_tx,
	.stop_rx	= parrot5_serial_stop_rx,
	.enable_ms	= parrot5_serial_enable_ms,
	.break_ctl	= parrot5_serial_break_ctl,
	.startup	= parrot5_serial_startup,
	.shutdown	= parrot5_serial_shutdown,
	.set_termios	= parrot5_serial_set_termios,
	.pm         = parrot5_pm,
	.type		= parrot5_serial_type,
	.release_port	= parrot5_serial_release_port,
	.request_port	= parrot5_serial_request_port,
	.config_port	= parrot5_serial_config_port,
	.verify_port	= parrot5_serial_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char  = parrot5_serial_poll_get_char,
	.poll_put_char  = parrot5_serial_poll_put_char,
#endif
};

static struct uart_parrot5_port p5_ports[UART_NR];

/*
 * Initialize a single Parrot5/5+/6 serial port.
 */
static void __init parrot5_serial_init_port(struct uart_parrot5_port *up,
					    int index)
{
	/* Parrot5+ / Parrot6 / Parrot7 */
	up->tx_fifosize = UART_TX_FIFO_SIZE_P5P;
	/*
	 * set default tx threshold to 4 bytes, i.e. a tx
	 * interrupt will be raised when 4 bytes are left in tx fifo
	 */
	up->tx_threshold = 4;
	/*
	 * set rx fifo threshold to 48 bytes, i.e. an rx interrupt
	 * will be raised when 16 empty slots are left in rx fifo
	 */
	up->lcr_fifo =
		UART_LCR_RXFIFO|
		UART_LCR_TXFIFO|
		UART_LCR_FIFO_RX_THRESHOLD_48|
		UART_LCR_FIFO_TX_THRESHOLD_4;

	up->spd = 0;
	up->lcr = 0;
	up->port.iotype = UPIO_MEM;
	up->port.fifosize = up->tx_fifosize-up->tx_threshold;
	up->port.ops = &parrot5_serial_pops;
	up->port.flags = UPF_BOOT_AUTOCONF|(low_latency? UPF_LOW_LATENCY : 0);
	up->port.line = index;
}

#ifdef CONFIG_SERIAL_PARROTX_CONSOLE

static void parrot5_serial_console_putchar(struct uart_port *port, int ch)
{
        while (__raw_readl(port->membase+_UART_STATUS)
               & UART_STATUS_TXFILLED) {
                barrier();
        }
        __raw_writel(ch, port->membase+_UART_TRX);
}

static void
parrot5_serial_console_write(struct console *co, const char *s,
			     unsigned int count)
{
	struct uart_parrot5_port *up = &p5_ports[co->index];

	uart_console_write((struct uart_port*)up,
	                   s,
	                   count,
	                   parrot5_serial_console_putchar);
}

static void __init
parrot5_serial_console_get_options(struct uart_port *port, int *baud,
				   int *parity, int *bits)
{
	u32 lcr, spd = __raw_readl(port->membase+_UART_SPD);

	if (spd) {
		*baud = clk_get_rate(((struct uart_parrot5_port*) port)->uart_clk)/spd;

		lcr = __raw_readl(port->membase+_UART_LCR);

		*parity = 'n';
		if (lcr & UART_LCR_PARITY) {
			*parity = (lcr & UART_LCR_EVEN)? 'e' : 'o';
		}

		*bits = (lcr & UART_LCR_EIGHTBITS)? 8 : 7;
	}
}

static int __init parrot5_serial_console_setup(struct console *co,
					       char *options)
{
	struct uart_parrot5_port *up;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if ((co->index < 0) || (co->index >= UART_NR)) {
		co->index = 0;
	}
	up = &p5_ports[co->index];
	if (up->port.membase == NULL)
		return -ENODEV;

	if (options) {
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	}
	else {
		parrot5_serial_console_get_options(&up->port, &baud, &parity,
						   &bits);
	}

	return uart_set_options(&up->port, co, baud, parity, bits, flow);
}

static struct tty_driver* pxuart_console_device(struct console* co, int* index)
{
	struct uart_driver* p = co->data;

	*index = co->index;
	return p->tty_driver;
}

static struct uart_driver parrot5_reg;
static struct console parrot5_serial_console = {
	.name		= PARROT5_SERIAL_NAME,
	.write		= parrot5_serial_console_write,
	.device		= pxuart_console_device,
	.setup		= parrot5_serial_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &parrot5_reg,
};

/* XXX TODO support earlycon stuff ... */
#define PARROT5_CONSOLE	(&parrot5_serial_console)
#else
#define PARROT5_CONSOLE	NULL
#endif

static struct uart_driver parrot5_reg = {
	.owner			= THIS_MODULE,
	.driver_name	= "Parrot serial",
	.nr			    = PXUART_NR,
	.dev_name		= PARROT5_SERIAL_NAME,
	.major			= PARROT5_SERIAL_MAJOR,
	.minor			= PARROT5_SERIAL_MINOR,
	.cons			= PARROT5_CONSOLE,
};

static int parrot5_serial_suspend(struct device *dev)
{
	struct uart_parrot5_port *up = dev_get_drvdata(dev);

	if (up)
		uart_suspend_port(&parrot5_reg, &up->port);

	return 0;
}

static int parrot5_serial_resume(struct device *dev)
{
	struct uart_parrot5_port *up = dev_get_drvdata(dev);

	if (up)
		uart_resume_port(&parrot5_reg, &up->port);

	return 0;
}

static const struct dev_pm_ops parrot5_pm_ops = {
	.suspend	= parrot5_serial_suspend,
	.resume		= parrot5_serial_resume,
};

static int __devinit parrot5_serial_probe(struct platform_device *dev)
{
	int ret;
	struct resource *res;
	struct uart_parrot5_port *up;
	int irq;

	if ((dev->id < 0) || (dev->id >= UART_NR)) {
		printk(KERN_ERR"invalid dev id %x\n", dev->id);
		ret = -ENOENT;
		goto noregs;
	}
	up = &p5_ports[dev->id];

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		ret = -ENOENT;
		goto noregs;
	}

	up->port.dev = &dev->dev;
	up->port.mapbase = res->start;
	up->port.membase = ioremap(res->start, res->end-res->start+1);

	if (!up->port.membase) {
		ret = -ENOMEM;
		goto nomap;
	}

	irq = platform_get_irq(dev, 0);
	if (irq < 0) {
		ret = -ENOENT;
		goto noirq;
	}
	up->port.irq = irq;

	up->uart_clk = clk_get(&dev->dev, NULL);
	if (IS_ERR(up->uart_clk)) {
		ret = PTR_ERR(up->uart_clk);
		goto noadd;
	}
	up->port.uartclk = clk_get_rate(up->uart_clk);

	up->pctl = pinctrl_get_select_default(&dev->dev);
	if (IS_ERR(up->pctl)) {
		ret = PTR_ERR(up->pctl);
		goto noclk;
	}

	parrot5_serial_init_port(up, dev->id);

	ret = uart_add_one_port(&parrot5_reg, &up->port);
	if (ret) {
		goto put_pin;
	}

	platform_set_drvdata(dev, up);

	return 0;

put_pin:
	pinctrl_put(up->pctl);
noclk:
    clk_put(up->uart_clk);
noadd:
noirq:
	iounmap(up->port.membase);
nomap:
noregs:
	return ret;
}

static int __devexit parrot5_serial_remove(struct platform_device *dev)
{
	struct uart_parrot5_port *up = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

	if (up) {
		uart_remove_one_port(&parrot5_reg, &up->port);
		pinctrl_put(up->pctl);
		clk_put(up->uart_clk);
		iounmap(up->port.membase);
	}

	return 0;
}

static struct platform_driver parrot5_serial_driver = {
	.probe		= parrot5_serial_probe,
	.remove		= __devexit_p(parrot5_serial_remove),
	.driver		= {
		.owner  = THIS_MODULE,
		.name	= PXUART_DRV_NAME,
		.pm     = &parrot5_pm_ops,
	},
};

static int __init parrot5_serial_init(void)
{
	int ret;

	printk(KERN_INFO "Serial: Parrot5/5+ UART driver $Revision: 1.19 $\n");

	ret = uart_register_driver(&parrot5_reg);
	if (ret == 0) {
		ret = platform_driver_register(&parrot5_serial_driver);
		if (ret) {
			uart_unregister_driver(&parrot5_reg);
		}
	}
	return ret;
}

static void __exit parrot5_serial_exit(void)
{
	platform_driver_unregister(&parrot5_serial_driver);
	uart_unregister_driver(&parrot5_reg);
}

module_init(parrot5_serial_init);
module_exit(parrot5_serial_exit);

MODULE_AUTHOR("Parrot S.A. <ivan.djelic@parrot.com>");
MODULE_DESCRIPTION("Parrot5/5+/6 serial port driver");
MODULE_LICENSE("GPL");
