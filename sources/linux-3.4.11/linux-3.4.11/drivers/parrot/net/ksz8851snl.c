/*
 * KSZ8851SNL ethernet driver
 *
 * Copyright (C) 2008 Parrot SA
 * Author: Florent Bayendrian <florent.bayendrian@parrot.com>
 * Inspired by enc28j60.c and the Micrel GPL driver for s3c2412
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * $Id: ksz8851snl.c,v 1.12 2009-08-26 09:11:44 fbayendrian Exp $
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/tcp.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/time.h>
#include <linux/crc32.h>

#include "ksz8851_hw.h"

#define DRV_NAME	"ksz8851snl"
#define DRV_VERSION	"$Revision: 1.12 $"

#define SPI_OPLEN		2
#define SPI_BUFOPLEN		1
#define SPI_DUMMY_READBUF	8 /* 4 real dummy bytes + RXFHSR + RXFHBCR */

#define SPI_OPCODE_IOREAD	(0x0<<6)
#define SPI_OPCODE_IOWRITE	(0x1<<6)
#define SPI_OPCODE_BREAD	(0x2<<6)
#define SPI_OPCODE_BWRITE	(0x3<<6)

#define KSZ8851SNL_MSG_DEFAULT	\
	(NETIF_MSG_PROBE | NETIF_MSG_IFUP | NETIF_MSG_IFDOWN | NETIF_MSG_LINK)

#define	MAX_FRAMELEN		1518
/* Buffer size required for the largest SPI transfer (i.e., reading a
 * frame). */
#define SPI_TRANSFER_BUF_LEN	(SPI_OPCODE_BREAD  + \
				 SPI_DUMMY_READBUF + \
				 MAX_FRAMELEN + 3) // word align access

#define TX_TIMEOUT	(4 * HZ)

#define RX_HIGH_WATERMARK       0x300
#define RX_LOW_WATERMARK        0x500
#define RX_OVERRUN_WATERMARK    0x40

// for the first silicon bug
#define KSZ8851SNL_LOWSPEED	31250000

#define FILTER_LIMIT	32

static int ksz8851snl_setlink(struct net_device *dev);

/*
 * delay rx interrupt from ksz8851 to the host. max value is 0xCFFF
 */
static unsigned int rx_delay_us = 0;
module_param(rx_delay_us, uint, 0444);


/* Driver local data */
struct ksz8851snl_net {
	struct net_device *netdev;
	struct spi_device *spi;
	struct mutex lock;
	struct sk_buff *tx_skb;
	struct work_struct tx_work;
	struct work_struct irq_work;
	struct work_struct setrx_work;
	bool hw_enable;
	u8 rev_id;
	u32 speed_hz;
	u32 autoneg;
	u32 speed;
	u32 duplex;
	u32 msg_enable;
	u8 spi_transfer_buf[SPI_TRANSFER_BUF_LEN];
};

static int
spi_read_byte(struct ksz8851snl_net *priv, u8 addr, u8* data)
{
	struct spi_transfer	t[2];
	struct spi_message	msg;
	u8 cmd[SPI_OPLEN];
	int ret;

	cmd[0] = SPI_OPCODE_IOREAD |
		 ((1 << (addr & 0x3)) << 2) |
		 (addr & 0xC0)>>6;
	cmd[1] = (addr & ~(0x3)) << 2;
	memset(t, 0, sizeof(t));
	t[0].tx_buf = &cmd;
	t[0].len = SPI_OPLEN;
	t[0].speed_hz = priv->speed_hz;
	t[1].rx_buf = data;
	t[1].len = sizeof(u8);
	t[1].speed_hz = priv->speed_hz;
	spi_message_init(&msg);
	spi_message_add_tail(&t[0], &msg);
	spi_message_add_tail(&t[1], &msg);
	ret = spi_sync(priv->spi, &msg);
	if (ret == 0) {
		ret = msg.status;
	}

	if (ret && netif_msg_drv(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() failed: ret = %d\n",
			__FUNCTION__, ret);

	return ret;
}

static int
spi_write_byte(struct ksz8851snl_net *priv, u8 addr, u8 data)
{
	struct spi_transfer	t;
	struct spi_message	msg;
	u8 buf[SPI_OPLEN + sizeof(u8)];
	int ret;

	buf[0] = SPI_OPCODE_IOWRITE |
		 ((1 << (addr & 0x3)) << 2) |
		 (addr & 0xC0)>>6;
	buf[1] = (addr & ~(0x3)) << 2;
	buf[2] = data;
	memset(&t, 0, sizeof(t));
	t.tx_buf = buf;
	t.len = SPI_OPLEN + sizeof(u8);
	t.speed_hz = priv->speed_hz;
	spi_message_init(&msg);
	spi_message_add_tail(&t, &msg);
	ret = spi_sync(priv->spi, &msg);
	if (ret == 0) {
		ret = msg.status;
	}

	if (ret && netif_msg_drv(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() failed: ret = %d\n",
			__FUNCTION__, ret);

	return ret;
}

static int
spi_read_hword(struct ksz8851snl_net *priv, u8 addr, u16 *data)
{
	struct spi_transfer	t[2];
	struct spi_message	msg;
	u8 cmd[SPI_OPLEN];
	int ret;

	addr &= ~0x1; // make addr even
	cmd[0] = (addr & 0x2)? 0xC<<2 : 0x3<<2;
	cmd[0] |= SPI_OPCODE_IOREAD |
		 (addr & 0xC0)>>6;
	cmd[1] = (addr & ~(0x3)) << 2;

	memset(t, 0, sizeof(t));
	t[0].tx_buf = &cmd;
	t[0].len = SPI_OPLEN;
	t[0].speed_hz = priv->speed_hz;
	t[1].rx_buf = data;
	t[1].len = sizeof(u16);
	t[1].speed_hz = priv->speed_hz;
	spi_message_init(&msg);
	spi_message_add_tail(&t[0], &msg);
	spi_message_add_tail(&t[1], &msg);
	ret = spi_sync(priv->spi, &msg);
	if (ret == 0) {
		ret = msg.status;
	}

	if (ret && netif_msg_drv(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() failed: ret = %d\n",
			__FUNCTION__, ret);

	return ret;
}

static int
spi_write_hword(struct ksz8851snl_net *priv, u8 addr, u16 data)
{
	struct spi_transfer	t;
	struct spi_message	msg;
	u8 buf[SPI_OPLEN + sizeof(u16)];
	int ret;

	addr &= ~0x1; // make addr even
	buf[0] = (addr & 0x2)? 0xC<<2 : 0x3<<2;
	buf[0] |= SPI_OPCODE_IOWRITE |
		 (addr & 0xC0)>>6;
	buf[1] = (addr & ~(0x3)) << 2;
	buf[2] = data & 0xFF;
	buf[3] = data >> 8;

	memset(&t, 0, sizeof(t));
	t.tx_buf = buf;
	t.len = SPI_OPLEN + sizeof(u16);
	t.speed_hz = priv->speed_hz;
	spi_message_init(&msg);
	spi_message_add_tail(&t, &msg);
	ret = spi_sync(priv->spi, &msg);
	if (ret == 0) {
		ret = msg.status;
	}

	if (ret && netif_msg_drv(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() failed: ret = %d\n",
			__FUNCTION__, ret);

	return ret;
}

static void
spi_read_buf_enable(struct ksz8851snl_net *priv)
{
	u16 reg;

	spi_write_hword(priv, REG_RX_ADDR_PTR, ADDR_PTR_AUTO_INC);
	spi_read_hword(priv, REG_RXQ_CMD, &reg);
	reg |= RXQ_CMD_CNTL | RXQ_START;
	spi_write_hword(priv, REG_RXQ_CMD, reg);
}

static void spi_read_buf_disable(struct ksz8851snl_net *priv)
{
	spi_write_hword(priv, REG_RXQ_CMD, RXQ_CMD_CNTL);
}


/*
 * SPI read buffer
 */
static int
spi_read_buf(struct ksz8851snl_net *priv, unsigned int len, u8 *data)
{
	struct spi_transfer	t[4];
	struct spi_message	msg;
	u8 cmd[SPI_BUFOPLEN];
	u8 dummy[SPI_DUMMY_READBUF];
	int ret;

	cmd[0] = SPI_OPCODE_BREAD;
	memset(t, 0, sizeof(t));
	t[0].tx_buf = cmd;
	t[0].len = SPI_BUFOPLEN;
	t[0].speed_hz = priv->speed_hz;
	t[1].rx_buf = dummy;
	t[1].len = SPI_DUMMY_READBUF;
	t[1].speed_hz = priv->speed_hz;
	// packet rx
	t[2].rx_buf = data;
	t[2].len = len;
	t[2].speed_hz = priv->speed_hz;
	// padding
	t[3].rx_buf = priv->spi_transfer_buf;
	t[3].len = ((len + 3) & ~0x3) - len;
	t[2].speed_hz = priv->speed_hz;
	spi_message_init(&msg);
	spi_message_add_tail(&t[0], &msg);
	spi_message_add_tail(&t[1], &msg);
	spi_message_add_tail(&t[2], &msg);
	if (t[3].len)
		spi_message_add_tail(&t[3], &msg);

	spi_read_buf_enable(priv);
	ret = spi_sync(priv->spi, &msg);
	spi_read_buf_disable(priv);

	if (ret == 0) {
		ret = msg.status;
	}

	if (ret && netif_msg_drv(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() failed: ret = %d\n",
			__FUNCTION__, ret);

	return ret;
}

static void
spi_write_buf_enable(struct ksz8851snl_net *priv)
{
	spi_write_hword(priv, REG_TX_ADDR_PTR, ADDR_PTR_AUTO_INC);
	spi_write_hword(priv, REG_RXQ_CMD, RXQ_CMD_CNTL | RXQ_START);
}

static void
spi_write_buf_disable(struct ksz8851snl_net *priv)
{
	spi_write_hword(priv, REG_RXQ_CMD, RXQ_CMD_CNTL);
	spi_write_hword(priv, REG_TXQ_CMD, TXQ_ENQUEUE);
}

/*
 * SPI write buffer
 */
static int spi_write_buf(struct ksz8851snl_net *priv, unsigned int len,
			 const u8 *data)
{
	struct spi_transfer	t[3];
	struct spi_message	msg;
	u8 buf[5];
	int ret;

	buf[0] = SPI_OPCODE_BWRITE;
	buf[1] = TX_CTRL_INTERRUPT_ON & 0xff;
	buf[2] = (TX_CTRL_INTERRUPT_ON >> 8) & 0xff;
	buf[3] = len & 0xff;
	buf[4] = (len >> 8) & 0xff;
	memset(t, 0, sizeof(t));
	t[0].tx_buf = buf;
	t[0].len = 5;
	t[0].speed_hz = priv->speed_hz;
	// packet data
	if ((len & 0x3) != 0)
		memset(priv->spi_transfer_buf + len, 0, 3);
	memcpy(priv->spi_transfer_buf, data, len);
	t[1].tx_buf = data;
	t[1].len = len;
	t[1].speed_hz = priv->speed_hz;
	t[2].tx_buf = priv->spi_transfer_buf;
	t[2].len = ((len + 3) & ~0x3) - len;
	t[2].speed_hz = priv->speed_hz;
	spi_message_init(&msg);
	spi_message_add_tail(&t[0], &msg);
	spi_message_add_tail(&t[1], &msg);
	if (t[2].len)
		spi_message_add_tail(&t[2], &msg);
	spi_write_buf_enable(priv);
	ret = spi_sync(priv->spi, &msg);
	spi_write_buf_disable(priv);
	if (ret == 0) {
		ret = msg.status;
	}

	if (ret && netif_msg_drv(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() failed: ret = %d\n",
			__FUNCTION__, ret);

	return ret;
}

static void ksz8851snl_soft_reset(struct net_device *dev)
{
	struct ksz8851snl_net *priv = netdev_priv(dev);

	if (netif_msg_hw(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() enter\n", __FUNCTION__);

	spi_write_hword(priv, REG_RESET_CTRL, GLOBAL_SOFTWARE_RESET);
	msleep(10); // wait for device to reset
	spi_write_hword(priv, REG_RESET_CTRL, 0);
#if 0
	{
		u16 reg;
		/* from Micrel driver:
		 * "workaround link setup issue for KSZ8851"
		 */
		spi_read_hword(priv, 0xe0, &reg);
		spi_write_hword(priv, 0xe0, reg | 0x8000);
		spi_read_hword(priv, 0xe0, &reg);
		mdelay(4000); // "wait until finishing AN"
	}
#endif
}

static int ksz8851snl_read_hw_macaddr(struct net_device *dev)
{
	int ret = 0;
	unsigned int i;
	struct ksz8851snl_net *priv = netdev_priv(dev);

	mutex_lock(&priv->lock);

	for (i = 0; i < ETH_ALEN && !ret; i++) {
		ret = spi_read_byte(priv, REG_MAC_ADDR_0 + i,
				&dev->dev_addr[ETH_ALEN-1+i]);
	}

	mutex_unlock(&priv->lock);

	return ret;
}

/*
 * Program the hardware MAC address from dev->dev_addr.
 */
static int ksz8851snl_set_hw_macaddr(struct net_device *dev)
{
	int ret = 0;
	struct ksz8851snl_net *priv = netdev_priv(dev);
	int i;

	if (!priv->hw_enable) {
		if (netif_msg_drv(priv)) {
			printk(KERN_INFO DRV_NAME
				": %s: Setting MAC address\n",
				dev->name);
		}
		for (i = 0; i < ETH_ALEN && !ret; i++) {
			ret = spi_write_byte(priv, REG_MAC_ADDR_0 + i,
				dev->dev_addr[ETH_ALEN-1-i]);
		}
	} else {
		if (netif_msg_drv(priv))
			printk(KERN_DEBUG DRV_NAME
				": %s() Hardware must be disabled to set "
				"MAC address\n", __FUNCTION__);
		ret = -EBUSY;
	}

	return ret;
}

/*
 * Store the new hardware address in dev->dev_addr, and update the MAC.
 */
static int ksz8851snl_set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *address = addr;
	struct ksz8851snl_net *priv = netdev_priv(dev);
	int ret;

	if (netif_running(dev))
		return -EBUSY;
	if (!is_valid_ether_addr(address->sa_data))
		return -EADDRNOTAVAIL;

	mutex_lock(&priv->lock);
	ret = ksz8851snl_set_hw_macaddr(dev);
	if (!ret)
		memcpy(dev->dev_addr, address->sa_data, dev->addr_len);
	mutex_unlock(&priv->lock);

	return ret;
}

/*
 * Access the PHY to determine link status
 */
static void ksz8851snl_check_link_status(struct net_device *dev)
{
	struct ksz8851snl_net *priv = netdev_priv(dev);
	unsigned int old_carrier, new_carrier;
	u16 link_status;

	old_carrier = netif_carrier_ok(dev) ? 1 : 0;
	spi_read_hword(priv, REG_PORT_STATUS, &link_status);
	new_carrier = (link_status & PORT_STATUS_LINK_GOOD)? 1: 0;

	if (new_carrier != old_carrier) {
		if (new_carrier) {
			netif_carrier_on(dev);
		} else {
			netif_carrier_off(dev);
		}
		if (netif_msg_link(priv)) {
			printk(KERN_INFO "%s: link %s", dev->name,
				new_carrier ? "up": "down");
			if (new_carrier) {
				if (link_status & PORT_STAT_SPEED_100MBIT) {
					printk(", 100Mbps ");
				} else {
					printk(", 10Mbps ");
				}
				if (link_status & PORT_STAT_FULL_DUPLEX) {
					printk("full-duplex");
				} else {
					printk("half-duplex");
				}
			}
			printk("\n");

		}
	}
}

static int ksz8851snl_hw_check(struct net_device *dev)
{
	struct ksz8851snl_net *priv = netdev_priv(dev);
	int ret = 0;
	u16 cider;

	/* read the chip ID, just to be sure that the chip is connected */
	spi_read_hword(priv, REG_CHIP_ID, &cider);

	priv->rev_id = (cider & REVISION_MASK) >> REVISION_SHIFT;
	if ((cider & CHIP_ID_MASK) != CHIP_ID_8851_16) {
		ret = -EINVAL;
		dev_err(&priv->spi->dev, "Invalid chip ID 0x%4.4x\n", cider);
	}

	return ret;
}

static void ksz8851snl_hw_enable(struct net_device *dev)
{
	struct ksz8851snl_net *priv = netdev_priv(dev);

	spi_write_hword(priv, REG_INT_MASK, 0);
	spi_write_hword(priv, REG_TX_ADDR_PTR, ADDR_PTR_AUTO_INC);
	spi_write_hword(priv, REG_RX_ADDR_PTR, ADDR_PTR_AUTO_INC);
	spi_write_hword(priv, REG_RX_FRAME_CNT_THRES,
				1 & RX_FRAME_THRESHOLD_MASK);
	/* Setup Receive High Water Mark to avoid loss packets under flow control */
	spi_write_hword(priv, REG_RX_HIGH_WATERMARK, RX_HIGH_WATERMARK);
	spi_write_hword(priv, REG_RX_LOW_WATERMARK, RX_LOW_WATERMARK);
	spi_write_hword(priv, REG_RX_OVERRUN_WATERMARK, RX_OVERRUN_WATERMARK);
	if (rx_delay_us)
		spi_write_hword(priv, REG_RXQ_CMD, RXQ_CMD_CNTL|RXQ_TIME_INT);
	else
		spi_write_hword(priv, REG_RXQ_CMD, RXQ_CMD_CNTL);

	// interrupt setup
	if (rx_delay_us > RX_TIME_THRESHOLD_MAX)
		rx_delay_us = RX_TIME_THRESHOLD_MAX;
	if (rx_delay_us)
		spi_write_hword(priv, REG_RX_TIME_THRES, rx_delay_us);
	spi_write_hword(priv, REG_INT_MASK, INT_SETUP_MASK | INT_RX_OVERRUN);

	// enable tx
	spi_write_hword(priv, REG_TX_CTRL,
			TX_CTRL_ICMP_CHECKSUM | TX_CTRL_UDP_CHECKSUM |
			TX_CTRL_TCP_CHECKSUM | TX_CTRL_IP_CHECKSUM |
			TX_CTRL_FLOW_ENABLE | TX_CTRL_PAD_ENABLE |
			TX_CTRL_CRC_ENABLE | TX_CTRL_ENABLE);

	// enable rx
	spi_write_hword(priv, REG_RX_CTRL1, RX_CTRL_MODE_PMAP);
	spi_write_hword(priv, REG_RX_CTRL2, RX_CTRL_SINGLE_FRAME_BURST |
					    RX_CTRL_UDP_LITE_CHECKSUM |
					    RX_CTRL_ICMP_CHECKSUM);
	spi_write_hword(priv, REG_INT_STATUS, INT_RX_STOPPED);
	priv->hw_enable = true;

	netif_start_queue(dev);
}

static void ksz8851snl_hw_disable(struct net_device *dev)
{
	struct ksz8851snl_net *priv = netdev_priv(dev);

	/* disable interrupt and packet reception */
	spi_write_hword(priv, REG_INT_MASK, 0);
	spi_write_hword(priv, REG_RX_CTRL1, RX_CTRL_FLUSH_QUEUE);
	spi_write_hword(priv, REG_RX_CTRL1, TX_CTRL_FLUSH_QUEUE);
	priv->hw_enable = false;
}

static void ksz8851snl_free_rx_packet(struct net_device *dev)
{
	struct ksz8851snl_net *priv = netdev_priv(dev);

	spi_write_hword(priv, REG_RXQ_CMD,RXQ_CMD_CNTL | RXQ_CMD_FREE_PACKET);
}

/*
 * Read all rx frames and update stats
 */
static void ksz8851snl_rx_handler(struct net_device *dev)
{
	struct ksz8851snl_net *priv = netdev_priv(dev);
	u16 frame_count;

	spi_read_hword(priv, REG_RX_FRAME_CNT_THRES, &frame_count);
	frame_count >>= 8;
	dev->stats.rx_packets += frame_count;

	while (frame_count--) {
		u16 rx_status, rx_length;

		spi_read_hword(priv, REG_RX_FHR_STATUS, &rx_status);
		spi_read_hword(priv, REG_RX_FHR_BYTE_CNT, &rx_length);
		rx_length &= RX_BYTE_CNT_MASK;
		dev->stats.rx_bytes += rx_length;

		if (rx_status & RX_MULTICAST)
			dev->stats.multicast++;

		if ((rx_status & RX_VALID) && !(rx_status & RX_ERRORS)) {
			struct sk_buff *skb;

			skb = dev_alloc_skb(rx_length + NET_IP_ALIGN);
			if (skb) {
				skb_reserve(skb, NET_IP_ALIGN);
				spi_read_buf(priv, rx_length,
						skb_put(skb, rx_length));
				skb->dev = dev;
				skb->protocol = eth_type_trans(skb, dev);
				skb->ip_summed = CHECKSUM_COMPLETE;
				dev->last_rx = jiffies;
				netif_rx_ni(skb);
			} else {
				dev->stats.rx_dropped++;
				ksz8851snl_free_rx_packet(dev);
				printk(KERN_NOTICE "%s: Low memory, "
					"packet dropped\n",
					dev->name);
			}
		} else {
			dev->stats.rx_errors++;
			if (rx_status & RX_BAD_CRC)
				dev->stats.rx_crc_errors++;
			if (rx_status & RX_TOO_LONG)
				dev->stats.rx_length_errors++;
			if (rx_status & RX_RUNT_ERROR)
				dev->stats.collisions++;
			ksz8851snl_free_rx_packet(dev);
		}
	}
}

static void ksz8851snl_irq_work_handler(struct work_struct *work)
{
	struct ksz8851snl_net *priv =
		container_of(work, struct ksz8851snl_net, irq_work);
	struct net_device *dev = priv->netdev;
	u16 intflags, intmask;

	if (netif_msg_intr(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() enter\n", __FUNCTION__);

	mutex_lock(&priv->lock);

	// temporary disable interrupt
	spi_read_hword(priv, REG_INT_MASK, &intmask);
	spi_write_hword(priv, REG_INT_MASK, 0);

	// ack
	spi_read_hword(priv, REG_INT_STATUS, &intflags);
	spi_write_hword(priv, REG_INT_STATUS, intflags);

	/* link change handler */
	if (intflags & INT_PHY) {
		u16 pmecr;

		if (netif_msg_intr(priv))
			printk(KERN_DEBUG "%s: link changed\n", dev->name);
		ksz8851snl_check_link_status(dev);
		// ack the link change
		spi_read_hword(priv, REG_POWER_CNTL, &pmecr);
		spi_write_hword(priv, REG_POWER_CNTL, pmecr);
	}

	if (intflags & INT_RX) {
		ksz8851snl_rx_handler(dev);
	}

	if (intflags & INT_RX_SPI_ERROR) {
		dev->stats.rx_errors++;
		// protocol or HW error
		printk(KERN_ERR "%s: SPI error\n", dev->name);
	}

	if (intflags & INT_RX_OVERRUN) {
		dev->stats.rx_errors++;
		dev->stats.rx_over_errors++;
		printk(KERN_ERR "%s: RX FIFO overrun\n", dev->name);
	}

	// re-enable interrupt
	spi_write_hword(priv, REG_INT_MASK, intmask);

	mutex_unlock(&priv->lock);

	if (netif_msg_intr(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() exit\n", __FUNCTION__);
}

/*
 * Transmit function.
 * Fill the buffer memory and send the contents of the transmit buffer
 * onto the network
 */
static int ksz8851snl_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct ksz8851snl_net *priv = netdev_priv(dev);

	if (netif_msg_tx_queued(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() enter\n", __FUNCTION__);

	netif_stop_queue(dev);

	/* save the timestamp */
	dev->trans_start = jiffies;
	/* Remember the skb for deferred processing */
	priv->tx_skb = skb;

	schedule_work(&priv->tx_work);

	return 0;
}

static void ksz8851snl_tx_work_handler(struct work_struct *work)
{
	struct ksz8851snl_net *priv =
		container_of(work, struct ksz8851snl_net, tx_work);
	struct net_device *dev = priv->netdev;
	unsigned int write_size;
	u16 mem_avail;

	if (netif_msg_tx_queued(priv))
		printk(KERN_DEBUG DRV_NAME
			": Tx Packet Len:%d\n", priv->tx_skb->len);

	write_size = ((priv->tx_skb->len + 3) & ~0x3) + 5;

	mutex_lock(&priv->lock);

	spi_read_hword(priv, REG_TX_MEM_INFO, &mem_avail);
	while (mem_avail < write_size) {
		cpu_relax();
		spi_read_hword(priv, REG_PHY_CNTL, &mem_avail);
	}

	spi_write_buf(priv, priv->tx_skb->len, priv->tx_skb->data);

	// update transmit statistics
	dev->stats.tx_packets++;
	dev->stats.tx_bytes += priv->tx_skb->len;
	// free tx resources
	dev_kfree_skb(priv->tx_skb);
	priv->tx_skb = NULL;
	netif_wake_queue(priv->netdev);

	mutex_unlock(&priv->lock);
}

static irqreturn_t ksz8851snl_irq(int irq, void *dev_id)
{
	struct ksz8851snl_net *priv = dev_id;

	/*
	 * Can't do anything in interrupt context because we need to
	 * block (spi_sync() is blocking) so fire of the interrupt
	 * handling workqueue.
	 * Remember that we access ksz8851snl registers through SPI bus
	 * via spi_sync() call.
	 */
	schedule_work(&priv->irq_work);

	return IRQ_HANDLED;
}

/*
 * Open/initialize the board. This is called (in the current kernel)
 * sometime after booting when the 'ifconfig' program is run.
 *
 * This routine should set everything up anew at each open, even
 * registers that "should" only need to be set once at boot, so that
 * there is non-reboot way to recover if something goes wrong.
 */
static int ksz8851snl_net_open(struct net_device *dev)
{
	struct ksz8851snl_net *priv = netdev_priv(dev);

	if (netif_msg_drv(priv))
		printk(KERN_DEBUG "%s: %s() enter\n", __FUNCTION__, dev->name);

	if (!is_valid_ether_addr(dev->dev_addr)) {
		if (netif_msg_ifup(priv)) {
			printk(KERN_ERR "%s: invalid MAC address\n",
				dev->name);
		}
		return -EADDRNOTAVAIL;
	}
	/* Reset the hardware here */
	ksz8851snl_soft_reset(dev);
	if (ksz8851snl_hw_check(dev)) {
		if (netif_msg_ifup(priv))
			printk("hw_check() failed\n");
		return -EINVAL;
	}
	/* Update the MAC address (in case user has changed it) */
	mutex_lock(&priv->lock);
	ksz8851snl_set_hw_macaddr(dev);
	ksz8851snl_setlink(dev);
	netif_carrier_off(dev);
	ksz8851snl_hw_enable(dev);
	mutex_unlock(&priv->lock);

	/* We are now ready to accept transmit requests from
	 * the queueing layer of the networking.
	 */
	netif_start_queue(dev);


	return 0;
}

static int ksz8851snl_net_stop(struct net_device *dev)
{
	struct ksz8851snl_net *priv = netdev_priv(dev);

	if (netif_msg_drv(priv))
		printk(KERN_DEBUG "%s: %s() enter\n", dev->name,
						      __FUNCTION__);

	netif_stop_queue(dev);
	netif_carrier_off(dev);

	mutex_lock(&priv->lock);
	ksz8851snl_hw_disable(dev);
	if (priv->tx_skb) {
		dev_kfree_skb(priv->tx_skb);
		priv->tx_skb = NULL;
		dev->stats.tx_errors++;
		dev->stats.tx_aborted_errors++;
	}
	mutex_unlock(&priv->lock);

	return 0;
}

/*
 * Set or clear the multicast filter for this adapter
 */
static void ksz8851snl_set_rx_mode(struct net_device *dev)
{
	struct ksz8851snl_net *priv = netdev_priv(dev);

	schedule_work(&priv->setrx_work);
}

static void ksz8851snl_setrx_work_handler(struct work_struct *work)
{
	struct ksz8851snl_net *priv =
		container_of(work, struct ksz8851snl_net, setrx_work);
	struct net_device *dev = priv->netdev;

	if (dev->flags & IFF_PROMISC) {
		if (netif_msg_drv(priv))
			printk(KERN_DEBUG "%s: promiscuous mode\n",
				dev->name);
		spi_write_hword(priv, REG_RX_CTRL1, RX_CTRL_MODE_PROMISC);
	} else if (dev->flags & IFF_ALLMULTI) {
		if (netif_msg_drv(priv))
			printk(KERN_DEBUG "%s: multicast mode\n", dev->name);
		spi_write_hword(priv, REG_RX_CTRL1, RX_CTRL_MODE_PMAP);
	} else if (dev->flags & IFF_MULTICAST && !netdev_mc_empty(dev)) {
		struct netdev_hw_addr *ha;
		u16 mc_filter[4];
		int i;

		if (netif_msg_drv(priv))
			printk(KERN_DEBUG "%s: normal mode with hash table\n",
				dev->name);

		memset(mc_filter, 0, sizeof(mc_filter));
		netdev_for_each_mc_addr(ha, dev) {
			u32 bit_nr = ether_crc(ETH_ALEN, ha->addr) >> 26;
			mc_filter[bit_nr >> 4] |= 1 << (bit_nr & 31);
		}
		for (i = 0; i < 4; i++)
			spi_write_hword(priv, REG_MAC_HASH_0 + 2*i,
					mc_filter[i]);
		// now enable the hash filter mode
		spi_write_hword(priv, REG_RX_CTRL1, RX_CTRL_MODE_HASH);
	} else {
		// just accept broadcast / unicast
		spi_write_hword(priv, REG_RX_CTRL1, RX_CTRL_MODE_NORMAL);
	}
}

/* ......................... ETHTOOL SUPPORT ........................... */

static void
ksz8851snl_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
}

static int
ksz8851snl_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct ksz8851snl_net *priv = netdev_priv(dev);

	u16 mii_ctrl, link_status;

	spi_read_hword(priv, REG_PHY_CNTL, &mii_ctrl);
	spi_read_hword(priv, REG_PORT_STATUS, &link_status);


	cmd->transceiver = XCVR_INTERNAL;
	cmd->supported	= SUPPORTED_10baseT_Half | SUPPORTED_10baseT_Full |
			  SUPPORTED_100baseT_Half| SUPPORTED_100baseT_Full|
			  SUPPORTED_Autoneg | SUPPORTED_Pause | SUPPORTED_TP;

	if (link_status & PORT_STATUS_LINK_GOOD) {
		cmd->speed = (link_status & PORT_STAT_SPEED_100MBIT) ?
				SPEED_100:
				SPEED_10;
		cmd->duplex = (link_status & PORT_STAT_FULL_DUPLEX) ?
				DUPLEX_FULL:
				DUPLEX_HALF;
	} else {
		cmd->speed = -1;
		cmd->duplex = -1;
	}
	cmd->port	= PORT_TP;
	cmd->autoneg	= (mii_ctrl & PHY_AUTO_NEG_ENABLE) ?
				AUTONEG_DISABLE:
				AUTONEG_DISABLE;

	return 0;
}

static int ksz8851snl_setlink(struct net_device *dev)
{
	struct ksz8851snl_net *priv = netdev_priv(dev);
	u16 mii_ctrl;

	spi_read_hword(priv, REG_PHY_CNTL, &mii_ctrl);

	if ((mii_ctrl & PHY_AUTO_NEG_ENABLE) &&
		priv->autoneg == AUTONEG_ENABLE)
		return 0;

	if (netif_msg_link(priv))
		printk("%s: setting autoneg = %d, speed=%d, duplex=%s\n",
			 dev->name,
			(priv->autoneg == AUTONEG_ENABLE)? 1: 0,
			(priv->speed == SPEED_100)? 100: 10,
			(priv->duplex == DUPLEX_FULL)? "full": "half");

	if (priv->autoneg == AUTONEG_ENABLE) {
		mii_ctrl |= PHY_AUTO_NEG_ENABLE;
	} else {
		mii_ctrl &= ~PHY_AUTO_NEG_ENABLE;
	}

	if (priv->speed == SPEED_100) {
		mii_ctrl |= PHY_SPEED_100MBIT;
	} else {
		mii_ctrl &= ~PHY_SPEED_100MBIT;
	}

	if (priv->duplex == DUPLEX_FULL) {
		mii_ctrl |= PHY_FULL_DUPLEX;
	} else {
		mii_ctrl &= ~PHY_FULL_DUPLEX;
	}

	mii_ctrl |= PHY_AUTO_NEG_RESTART;
	return spi_write_hword(priv, REG_PHY_CNTL, mii_ctrl);
}

static int
ksz8851snl_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct ksz8851snl_net *priv = netdev_priv(dev);
	int ret = 0;

	mutex_lock(&priv->lock);

	if (!priv->hw_enable) {
		if ((cmd->speed == SPEED_100 || cmd->speed == SPEED_10) &&
		    (cmd->duplex == DUPLEX_FULL || cmd->duplex == DUPLEX_HALF)) {
			priv->autoneg = cmd->autoneg;
			priv->speed = cmd->speed;
			priv->duplex = cmd->duplex;
			ksz8851snl_setlink(dev);
		} else {
			if (netif_msg_link(priv))
				printk("%s: unsupported link setting\n",
					dev->name);
			ret = -EOPNOTSUPP;
		}
	} else {
		if (netif_msg_link(priv))
			printk(KERN_DEBUG DRV_NAME "hw must be disabled "
				"to set link mode\n");
		ret = -EBUSY;
	}

	mutex_unlock(&priv->lock);

	return ret;
}

static u32 ksz8851snl_get_msglevel(struct net_device *dev)
{
	struct ksz8851snl_net *priv = netdev_priv(dev);
	return priv->msg_enable;
}

static void ksz8851snl_set_msglevel(struct net_device *dev, u32 val)
{
	struct ksz8851snl_net *priv = netdev_priv(dev);
	priv->msg_enable = val;
}

static const struct ethtool_ops ksz8851snl_ethtool_ops = {
	.get_settings	= ksz8851snl_get_settings,
	.set_settings	= ksz8851snl_set_settings,
	.get_drvinfo	= ksz8851snl_get_drvinfo,
	.get_msglevel	= ksz8851snl_get_msglevel,
	.set_msglevel	= ksz8851snl_set_msglevel,
};

static const struct net_device_ops ks8851snl_netdev_ops = {
	.ndo_open		= ksz8851snl_net_open,
	.ndo_stop		= ksz8851snl_net_stop,
	.ndo_start_xmit		= ksz8851snl_start_xmit,
	.ndo_set_mac_address	= ksz8851snl_set_mac_address,
	.ndo_set_rx_mode	= ksz8851snl_set_rx_mode,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
};

static int __devinit ksz8851snl_probe(struct spi_device *spi)
{
	struct net_device *dev;
	struct ksz8851snl_net *priv;
	int ret = 0;
	void (*jtag_spi_mode) (int enable) = spi->dev.platform_data;

	dev_info(&spi->dev, "ethernet driver loaded\n");

	dev = alloc_etherdev(sizeof(struct ksz8851snl_net));
	if (!dev) {
		dev_err(&spi->dev, "unable to alloc new etherdev\n");
		ret = -ENOMEM;
		goto error_alloc;
	}
	priv = netdev_priv(dev);

	priv->netdev = dev;	/* priv to netdev reference */
	priv->spi = spi;	/* priv to spi reference */
	priv->msg_enable = KSZ8851SNL_MSG_DEFAULT;
	priv->hw_enable = false;
	priv->autoneg = AUTONEG_ENABLE;
	priv->speed = SPEED_100;
	priv->duplex = DUPLEX_FULL;
	priv->speed_hz = KSZ8851SNL_LOWSPEED;
	mutex_init(&priv->lock);
	INIT_WORK(&priv->tx_work, ksz8851snl_tx_work_handler);
	INIT_WORK(&priv->setrx_work, ksz8851snl_setrx_work_handler);
	INIT_WORK(&priv->irq_work, ksz8851snl_irq_work_handler);
	dev_set_drvdata(&spi->dev, priv);	/* spi to priv reference */
	SET_NETDEV_DEV(dev, &spi->dev);

	if (jtag_spi_mode)
		jtag_spi_mode(1);
	/* reset the chip */
	ksz8851snl_soft_reset(dev);

	if (ksz8851snl_hw_check(dev)) {
		ret = -EINVAL;
		goto error_chip;
	}

	dev_info(&spi->dev, "revision %u\n", priv->rev_id);
	if (priv->rev_id >= 1) {
		priv->speed_hz = 0;
	}

	random_ether_addr(dev->dev_addr);

	ksz8851snl_set_hw_macaddr(dev);
	ksz8851snl_read_hw_macaddr(dev);

	ret = request_irq(spi->irq, ksz8851snl_irq, IRQF_TRIGGER_NONE,
			  DRV_NAME, priv);
	if (ret < 0) {
		dev_err(&spi->dev, "request irq %d failed "
				"(ret = %d)\n", spi->irq, ret);
		goto error_irq;
	}

	dev->irq = spi->irq;
	dev->netdev_ops = &ks8851snl_netdev_ops;
	dev->watchdog_timeo = TX_TIMEOUT;
	dev->features = NETIF_F_HW_CSUM;
	SET_ETHTOOL_OPS(dev, &ksz8851snl_ethtool_ops);

	ret = register_netdev(dev);
	if (ret) {
		dev_err(&spi->dev, "register netdev "
				"failed (ret = %d)\n", ret);
		goto error_register;
	}
	dev_info(&spi->dev, "driver registered\n");

	return 0;

error_register:
	free_irq(spi->irq, priv);
error_irq:
error_chip:
	free_netdev(dev);
error_alloc:
	if (jtag_spi_mode)
		jtag_spi_mode(0);
	dev_set_drvdata(&spi->dev, NULL);
	dev_info(&spi->dev, "probe failed\n");
	return ret;
}

static int __devexit ksz8851snl_remove(struct spi_device *spi)
{
	struct ksz8851snl_net *priv = dev_get_drvdata(&spi->dev);
	void (*jtag_spi_mode) (int enable) = spi->dev.platform_data;

	if (priv) {
		if (netif_msg_drv(priv))
			printk(KERN_DEBUG DRV_NAME ": remove\n");
		unregister_netdev(priv->netdev);
		free_irq(spi->irq, priv);
		free_netdev(priv->netdev);
	}
	if (jtag_spi_mode)
		jtag_spi_mode(0);

	return 0;
}

static struct spi_driver ksz8851snl_driver = {
	.driver = {
		   .name = DRV_NAME,
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE,
		   },
	.probe = ksz8851snl_probe,
	.remove = __devexit_p(ksz8851snl_remove),
};

static int __init ksz8851snl_init(void)
{
	printk(KERN_INFO "Parrot6 " DRV_NAME " driver " DRV_VERSION "\n");
	return spi_register_driver(&ksz8851snl_driver);
}

static void __exit ksz8851snl_exit(void)
{
	spi_unregister_driver(&ksz8851snl_driver);
}

module_init(ksz8851snl_init);
module_exit(ksz8851snl_exit);

MODULE_DESCRIPTION(DRV_NAME " ethernet driver");
MODULE_AUTHOR("Florent Bayendrian <florent.bayendrian@parrot.com>");
MODULE_LICENSE("GPL");
