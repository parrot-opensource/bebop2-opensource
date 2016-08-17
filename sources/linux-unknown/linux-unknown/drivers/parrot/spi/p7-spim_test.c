/*
 * @file p7_spi_test.c
 * @brief Driver for Parrot7 SPI controller, master mode (test module)
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * @author     alvaro.moran@parrot.com
 * @date       2012-06-18
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

#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/module.h> 
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>

MODULE_AUTHOR( "Alvaro Moran - Parrot S.A. <alvaro.moran@parrot.com>" );
MODULE_DESCRIPTION( "Parrot7 SPI master test module" );
MODULE_LICENSE( "GPL" );


static unsigned int rx_len = 0;
module_param(rx_len, uint, 0644);

static unsigned int tx_len = 0;
module_param(tx_len, uint, 0644);

static unsigned int verbose = 1;
module_param(verbose, uint, 0644);

static unsigned int seed = 0;
module_param(seed, uint, 0644);

static unsigned int speed = 1 * 1000 * 1000;
module_param(speed, uint, 0644);



#define VPRINT(_format, ...)  if (verbose) printk(_format, ## __VA_ARGS__);


/**  spi_tx_rx - Alternative to spi_write_then_read that allows bigger buffers 
 */
int spi_tx_rx(struct spi_device *spi,
		const u8 *buf, unsigned n_tx,
		unsigned n_rx)
{
	int			status;
	struct spi_message	message;
	struct spi_transfer	x[2];

	if (n_tx && n_rx)
		return -1;

	spi_message_init(&message);
	memset(x, 0, sizeof x);
	if (n_tx) {
		x[0].len = n_tx;
		spi_message_add_tail(&x[0], &message);
	}
	if (n_rx) {
		x[1].len = n_rx;
        x[1].speed_hz = speed;
		spi_message_add_tail(&x[1], &message);
	}

	x[0].tx_buf = buf;
	x[1].rx_buf = (u8*)(buf + n_tx);

	/* do the i/o */
	status = spi_sync(spi, &message);

	return status;
}


static int __devinit p7_spi_test_probe(struct spi_device *spi)
{ 
	u8* tx = NULL;
	u8* rx = NULL;
	int status = 0;
	int ret = 0;
	int i;

    if (spi->master->bus_num != 2)
        return -1;

	if (tx_len) {
		tx = kmalloc(sizeof(u8)*tx_len, GFP_KERNEL);
		if (!tx)
			return -ENOMEM;
		/* Init with a simple pattern */
		for (i = 0; i < tx_len; i++)
			tx[i] = (seed+i) & 0xff;
		status = spi_tx_rx(spi, tx, tx_len, 0);
		if (status < 0) {
			printk("Err in tx: %d\n", status);
			goto exit;
		}
		msleep(10);
	}
	if (rx_len) {
		rx = kzalloc(sizeof(u8)*rx_len, GFP_KERNEL);
		if (!rx)
			return -ENOMEM;
		status = spi_tx_rx(spi, rx, 0, rx_len);
		if (status < 0) {
			printk("Err in rx: %d\n", status);
			goto exit;
		}
	}

	if (tx) {
		VPRINT("Master Transfer OK (%d bytes)\n", tx_len);
	}
	if (rx) {
		VPRINT("Master Received OK (%d bytes): ", rx_len);
		for (i = 0; i < rx_len; i++)
			VPRINT("%02x ", rx[i]);
		VPRINT("\n");
	}

exit:
	if (rx)
		kfree(rx);
	if (tx) 
		kfree(tx);
	return ret;
}

static int __devexit p7_spi_test_remove(struct spi_device *spi)
{
	return 0;
}


static struct spi_driver p7_spi_test_driver = {
	.driver = {
		.name	= "spidev",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= p7_spi_test_probe,
	.remove	= __devexit_p(p7_spi_test_remove),
};

static int __init p7_spi_test_init(void)
{
	return spi_register_driver(&p7_spi_test_driver);
}

static void __exit p7_spi_test_exit( void ) 
{ 
	spi_unregister_driver(&p7_spi_test_driver);
}

module_init( p7_spi_test_init ); 
module_exit( p7_spi_test_exit ); 
