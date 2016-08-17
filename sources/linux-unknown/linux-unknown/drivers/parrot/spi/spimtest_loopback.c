/*
 * @file spis_22_master.c
 * @brief Receive a big buffer by sending multiple times the same messages.
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * @author     damien.riegel.ext@parrot.com
 * @date       Dec 2012
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
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/dma-mapping.h>

struct wrapper_spi_info {
	struct spi_message* mesg;
	struct spi_device*	spi;
	int 				mesg_position;
	int 				nb_send;
	int					target_send;
	unsigned char		offset;
	int					fail_at;
};

static unsigned int nb_messages = 1;
module_param(nb_messages, uint, 0644);

static unsigned int message_size = 256;
module_param(message_size, uint, 0644);

static unsigned int total_len = 256;
module_param(total_len, uint, 0644);

static unsigned int offset = 0x2A;
module_param(offset, uint, 0644);

static unsigned int speed_hz = 26 * 1000 * 1000;
module_param(speed_hz, uint, 0644);

static struct spi_message *mesgs = NULL;
static struct spi_transfer *xfers = NULL;
static struct wrapper_spi_info* infos = NULL;
static unsigned int nb_send = 2;
static int total_messages_received;
static struct completion spimtest_completion;
static unsigned int should_exit = 0;

#define spimtest_print(level, format, arg...) \
	printk(level "[SPIMTEST LOOPBACK][MSTR] " format, ##arg)

#define spimtest_info(format, arg...) \
	spimtest_print(KERN_INFO, format, ##arg)

#define spimtest_err(format, arg...) \
	spimtest_print(KERN_ERR, format, ##arg)


static int spimtest_allocate(void)
{
	int err = -ENOMEM;
	int i;

	infos = kzalloc(nb_messages * sizeof(struct spi_message), GFP_KERNEL);
	if (! infos) {
		goto exit;
	}

	mesgs = kzalloc(nb_messages * sizeof(struct spi_message), GFP_KERNEL);
	if (! mesgs) {
		goto free_infos;
	}

	xfers = kzalloc(nb_messages * sizeof(struct spi_transfer), GFP_KERNEL);
	if (! xfers) {
		goto free_mesgs;
	}

	for(i = 0; i < nb_messages; i++) {
		int j, loc_offset;
		u8 *buf;

		spi_message_init(&mesgs[i]);
		memset(&xfers[i], 0, sizeof(struct spi_transfer));
		xfers[i].len = message_size;
		xfers[i].speed_hz = speed_hz;

		buf = kzalloc(message_size, GFP_KERNEL);
		if (! buf ) {
			goto free_buffers;
		}

		loc_offset = (offset + (i * message_size)) % 256;
		for(j = 0; j < message_size; j++) {
			buf[j] = loc_offset++ % 256;
		}

		xfers[i].tx_buf = buf;

		spi_message_add_tail(&xfers[i], &mesgs[i]);
	}

	return 0;

free_buffers:
	for(i = 0; i < nb_messages; i++) {

		if (xfers[i].tx_buf) {
			kfree(xfers[i].tx_buf);
		}
	}
	kfree(xfers);

free_mesgs:
	kfree(mesgs);
free_infos:
	kfree(infos);
exit:
	return err;
}

static void spimtest_receive(void* context)
{
	struct wrapper_spi_info* info = context;
	struct spi_message* mesg = info->mesg;

	if (mesg->status) {
		spimtest_err("-FAIL- Msg %d (round %d) received with error code %d\n",
				info->mesg_position, info->nb_send, mesg->status);
		should_exit++;
		goto completion;
	}

	info->nb_send++;
	if (info->nb_send < info->target_send) {
		spi_async(info->spi, mesg);
	}

completion:
	total_messages_received++;
	if (total_messages_received == nb_send || should_exit == nb_messages)
		complete(&spimtest_completion);
}

static int __devinit spimtest_probe(struct spi_device *spi) 
{
	int err = 0, i;
	int send_per_message;
	int remaining_send;

	total_messages_received = 0;
	nb_send = total_len / message_size;
	if (total_len % message_size) {
		spimtest_err("total_len (%u) must be a multiple of message_size (%u)\n",
				total_len, message_size);
		return -EINVAL;
	}

	if (((message_size * nb_messages) % 256) != 0) {
		spimtest_err("(message_size * nb_messages) must be a multiple of 256\n");
		return -EINVAL;
	}

	/* some messages are going to be sent "send_per_message" times */
	send_per_message = nb_send / nb_messages;
	/* and some "send_per_message" + 1 times */
	remaining_send = nb_send % nb_messages;

	init_completion(&spimtest_completion);
	err = spimtest_allocate();
	if (err) {
		goto exit;
	}

	/*
	 * Send each message once, the rest is done in completion function
	 */
	for(i = 0; i < nb_messages; i++) {
		infos[i].mesg = &mesgs[i];
		infos[i].spi = spi;
		infos[i].nb_send = 0;
		infos[i].target_send = send_per_message;
		if (remaining_send) {
			infos[i].target_send++;
			remaining_send--;
		}
		infos[i].fail_at = 0;
		infos[i].mesg_position = i;

		mesgs[i].complete = spimtest_receive;
		mesgs[i].context = &infos[i];
		err = spi_async(spi, &mesgs[i]);
		if (err) {
			spimtest_err("-FAIL- Can't send a message (%d)\n",
					err);
		}
	}
	
	wait_for_completion(&spimtest_completion);
	spimtest_info("%s message pool %d msg. %d msgs sent\n",
			should_exit ? "-FAIL-" : "-PASS-",
			nb_messages, nb_send);

exit:
	for(i = 0; i < nb_messages; i++) {
		kfree(xfers[i].tx_buf);
	}

	kfree(mesgs);
	kfree(xfers);
	kfree(infos);
	return err;
}

static int __devexit spimtest_remove(struct spi_device *spi)
{
	return 0;
}


static struct spi_driver spimtest_driver = {
	.driver = {
		.name	= "spidev",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= spimtest_probe,
	.remove	= __devexit_p(spimtest_remove),
};

module_spi_driver(spimtest_driver);
MODULE_DESCRIPTION( "spimtest loopback module" );
MODULE_LICENSE( "GPL" );
