#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/module.h> 
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/fs.h>

struct spistest_core;

struct wrapper_spi_info {
	struct spi_message* mesg;
	struct spi_device*	spi;
	int 				mesg_position;
	int 				nb_send;
	int					target_send;
	unsigned char		offset;
	int					fail_at;

	struct spistest_core* core;
};

struct spistest_core {
	int                     bus_num;
	struct spi_message      *mesgs;
	struct spi_transfer     *xfers;
	struct wrapper_spi_info *infos;
	
	int                     nb_send;
	int                     nb_fail;
	int                     total_messages_received;
	
	struct completion       done;
};

static unsigned int nb_messages = 1;
module_param(nb_messages, uint, 0644);

static unsigned int message_size = 256;
module_param(message_size, uint, 0644);

static unsigned int total_len = 256;
module_param(total_len, uint, 0644);

static unsigned int verbose = 1;
module_param(verbose, uint, 0644);

static unsigned int offset = 0x2A;
module_param(offset, uint, 0644);

static struct spistest_core cores[2];

#define spistest_print(level, core, format, arg...) \
	printk(level "[SPISTEST LOOPBACK][SLV %d] " format, core->bus_num, ##arg)

#define spistest_info(core, format, arg...) \
	spistest_print(KERN_INFO, core, format, ##arg)

#define spistest_err(core, format, arg...) \
	spistest_print(KERN_ERR, core, format, ##arg)

static struct spistest_core* get_spistest_core(int bus_num)
{
	/* that's hackish but... 
	 * In some case, there are two SPI slave receiving the same data (because
	 * they are wired to the same pads, yet another hack), so we want this
	 * driver to support up to two cores. For the moment, slaves are on bus
	 * 1 and 3.
	 */
	switch(bus_num) {
	case 1:
		cores[0].bus_num = 1;
		return &cores[0];
	case 3:
		cores[1].bus_num = 3;
		return &cores[1];
	default:
		return NULL;
	}

}

static int spistest_allocate(struct spistest_core *core)
{
	int err = -ENOMEM;
	int i;

	core->infos = kzalloc(nb_messages * sizeof(struct spi_message), GFP_KERNEL);
	if (! core->infos) {
		goto exit;
	}

	core->mesgs = kzalloc(nb_messages * sizeof(struct spi_message), GFP_KERNEL);
	if (! core->mesgs) {
		goto free_infos;
	}

	core->xfers = kzalloc(nb_messages * sizeof(struct spi_transfer), GFP_KERNEL);
	if (! core->xfers) {
		goto free_mesgs;
	}

	for(i = 0; i < nb_messages; i++) {
		spi_message_init(&core->mesgs[i]);
		memset(&core->xfers[i], 0, sizeof(struct spi_transfer));
		core->xfers[i].len = message_size;

		core->xfers[i].rx_buf = kzalloc(message_size, GFP_KERNEL);
		if (! core->xfers[i].rx_buf ) {
			goto free_buffers;
		}

		spi_message_add_tail(&core->xfers[i], &core->mesgs[i]);
	}

	return 0;

free_buffers:
	for(i = 0; i < nb_messages; i++) {
		if (core->xfers[i].rx_buf)
			kfree(core->xfers[i].rx_buf);
	}
free_mesgs:
	kfree(core->mesgs);
free_infos:
	kfree(core->infos);
exit:
	return err;
}


static int spistest_check(struct spistest_core *core,
		u8* buf, int len, unsigned char offset)
{
	int i, j, err = -1;
	for(i = 0; i < len; i++) {
		if (buf[i] != offset) {
			spistest_err(core, "-FAIL- Unexpected value at offset %d."
					"Read 0x%.2X, expected 0x%.2X\n",
					i, buf[i], offset);

			for (j = 0; j < 10; j++) {
				if (i + j < len)
					spistest_err(core, "-FAIL- buf[%d] = 0x%X\n",
							i + j, buf[i + j]);
			}
			return i;

		}
		offset++;
	}

	return err;
}


static void spistest_receive(void* context)
{
	int err;
	struct wrapper_spi_info* info = context;
	struct spistest_core*   core = info->core;
	struct spi_message* mesg = info->mesg;
	struct spi_transfer* xfer = list_entry(
			mesg->transfers.next,
			struct spi_transfer,
			transfer_list);


	if (mesg->status) {
		spistest_err(core, "-FAIL- Msg %d (round %d) received with error code %d\n",
				info->mesg_position, info->nb_send, mesg->status);
		core->nb_fail++;
		spistest_check(core, xfer->rx_buf, message_size, info->offset);
		goto completion;
	}

	/* check buffer */
	err = spistest_check(core, xfer->rx_buf, message_size, info->offset);

	if (err != -1) {
		spistest_err(core, "-FAIL- Msg %d (round %d) is invalid\n",
				info->mesg_position, info->nb_send);
		info->fail_at = info->nb_send;
		core->nb_fail++;
		goto completion;
	}

	if (verbose)
		spistest_info(core, "-PASS- Msg %d (round %d) validated\n",
				info->mesg_position, info->nb_send);

	if (core->nb_fail)
		/* If this message was successful but a previous one failed,
		 * do not resend this message to end the test asap */
		core->nb_fail++;
	else {
		info->nb_send++;
		if (info->nb_send < info->target_send) {
			spi_async(info->spi, mesg);
		}
	}

completion:

	core->total_messages_received++;

	if (core->total_messages_received == core->nb_send ||
			core->nb_fail == nb_messages)
		complete(&core->done);
}


static int __devinit spistest_probe(struct spi_device *spi) 
{
	int err = 0, i;
	int send_per_message;
	int remaining_send;
	struct spistest_core *core;

	core = get_spistest_core(spi->master->bus_num);
	if ( ! core ) {
		printk(KERN_ERR "SPI bus number must be only 1 or 3\n");
		return -EINVAL;
	}

	core->total_messages_received = 0;
	core->nb_send = total_len / message_size;
	if (total_len % message_size) {
		spistest_err(core, "total_len must be a multiple of message_size\n");
		return -EINVAL;
	}

	send_per_message = core->nb_send / nb_messages;
	remaining_send = core->nb_send % nb_messages;

	init_completion(&core->done);
	err = spistest_allocate(core);
	if (err) {
		goto exit;
	}

	/*
	 * Send each message once, the rest is done in completion function
	 */
	for(i = 0; i < nb_messages; i++) {
		core->infos[i].mesg = &core->mesgs[i];
		core->infos[i].spi = spi;
		core->infos[i].nb_send = 0;
		core->infos[i].target_send = send_per_message;
		if (remaining_send) {
			core->infos[i].target_send++;
			remaining_send--;
		}
		core->infos[i].offset = (offset + (i * message_size)) % 256;
		core->infos[i].fail_at = 0;
		core->infos[i].mesg_position = i;
		core->infos[i].core = core;

		core->mesgs[i].complete = spistest_receive;
		core->mesgs[i].context = &core->infos[i];
		err = spi_async(spi, &core->mesgs[i]);
		if (err) {
			spistest_err(core, "-FAIL- Can't send a message (%d)\n",
					err);
		}
		else {
			spistest_info(core,
					"Sending msg %d [%p], size %d (total %d)\n",
					i, core->xfers[i].rx_buf, message_size, total_len);
		}
	}

exit:
	return err;
}


static int __devexit spistest_remove(struct spi_device *spi)
{
	int i;
	struct spistest_core *core;

	core = get_spistest_core(spi->master->bus_num);
	wait_for_completion(&core->done);
	if (! core->nb_fail) {
		bool success = true;
		for(i = 0; i < nb_messages; i++) {
			if (core->infos[i].fail_at != 0)
				success = false;
		}

		spistest_info(core, "%s message pool %d msg. %d msgs sent\n",
				success ? "-PASS-" : "-FAIL-",
				nb_messages, core->nb_send);
	}
	else {
		spistest_err(core, "-FAIL- Could not send all messages\n");
		for(i = 0; i < nb_messages; i++) {
			spistest_err(core, "\tMsg %d sent %d time(s) successfully\n",
					i, core->infos[i].nb_send);
		}
	}

	for(i = 0; i < nb_messages; i++) {
		kfree(core->xfers[i].rx_buf);
	}
	kfree(core->mesgs);
	kfree(core->xfers);
	kfree(core->infos);
	return 0;
}


static struct spi_driver spistest_driver = {
	.driver = {
		.name	= "spistest",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= spistest_probe,
	.remove	= __devexit_p(spistest_remove),
};

module_spi_driver(spistest_driver);
MODULE_DESCRIPTION( "spistest loopback module" );
MODULE_LICENSE( "GPL" );
