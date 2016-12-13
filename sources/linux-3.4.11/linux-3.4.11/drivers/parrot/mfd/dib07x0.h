#ifndef _DIB07X0_H_
#define _DIB07X0_H_

enum dib07x0_gpios {
	GPIO0  =  0,
	GPIO1  =  2,
	GPIO2  =  3,
	GPIO3  =  4,
	GPIO4  =  5,
	GPIO5  =  6,
	GPIO6  =  8,
	GPIO7  = 10,
	GPIO8  = 11,
	GPIO9  = 14,
	GPIO10 = 15,
};

#define GPIO_IN  0
#define GPIO_OUT 1

extern int dvb_usb_dib0700_debug;
#define dprintk(var,level,args...) \
	    do { if ((var & level)) { printk(args); } } while (0)

#define debug_dump(b,l,func) {\
	int loop_; \
	for (loop_ = 0; loop_ < l; loop_++) func("%02x ", b[loop_]); \
	func("\n");\
}


#define deb_info(args...)   dprintk(dvb_usb_dib0700_debug,0x01,args)
#define deb_fw(args...)     dprintk(dvb_usb_dib0700_debug,0x02,args)
#define deb_fwdata(args...) dprintk(dvb_usb_dib0700_debug,0x04,args)
#define deb_data(args...)   dprintk(dvb_usb_dib0700_debug,0x08,args)

#define REQUEST_SET_USB_XFER_LEN    0x0 /* valid only for firmware version */
					/* higher than 1.21 */
#define REQUEST_I2C_READ            0x2
#define REQUEST_I2C_WRITE           0x3
#define REQUEST_POLL_RC             0x4 /* deprecated in firmware v1.20 */
#define REQUEST_DISCONNECT          0x5 /* force Linux to renews USB descriptors */
#define REQUEST_JUMPRAM             0x8
#define REQUEST_SET_CLOCK           0xB
#define REQUEST_SET_GPIO            0xC
#define REQUEST_ENABLE_VIDEO        0xF
	// 1 Byte: 4MSB(1 = enable streaming, 0 = disable streaming) 4LSB(Video Mode: 0 = MPEG2 188Bytes, 1 = Analog)
	// 2 Byte: MPEG2 mode:  4MSB(1 = Master Mode, 0 = Slave Mode) 4LSB(Channel 1 = bit0, Channel 2 = bit1)
	// 2 Byte: Analog mode: 4MSB(0 = 625 lines, 1 = 525 lines)    4LSB(     "                "           )
#define REQUEST_SET_I2C_PARAM       0x10
#define REQUEST_SET_RC              0x11
#define REQUEST_NEW_I2C_READ        0x12
#define REQUEST_NEW_I2C_WRITE       0x13
#define REQUEST_GET_VERSION         0x15
#define REQUEST_UART_SET_PARAM      (REQUEST_GET_VERSION + 0x6)
#define REQUEST_UART_WRITE          (REQUEST_GET_VERSION + 0x7)
#define REQUEST_UART_READ           (REQUEST_GET_VERSION + 0x8)
#define REQUEST_UART_TX_FIFO_LVL    (REQUEST_GET_VERSION + 0x9)

#define USB_VID_DIBCOM				0x10b8
#define USB_PID_DIBCOM_HOOK_DEFAULT		0x0066

#define USB_CTRL_TMOUT     1000

#endif
