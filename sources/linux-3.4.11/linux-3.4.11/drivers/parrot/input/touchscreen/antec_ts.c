#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>

#include <linux/gpio.h>
#define I2C_DEVICE_ADDRESS 0x0a
#define OMAP3_DEVKIT_TS_GPIO 130
#define I2C_DEVICE_NAME "uc6511"
#define I2C_ADAPTER_NUMBER 2

#define PRINT_XY		0
#define MAX_TRACK_POINT		5
#define	MAX_12BIT			((1<<12)-1)

typedef struct i2c_client	bus_device;

static u16 fingers_x[MAX_TRACK_POINT];
static u16 fingers_y[MAX_TRACK_POINT];

//################################################################################################
// read and check parameter file, begin
#include <linux/fs.h>
#include <linux/file.h>
#include <asm/uaccess.h>

#define Calibrate_Name 	 	"/system/etc/com.TPCalibration/CalibrateData.txt"
static u16 Calibrate_data[10];		//x0,y0,x1,y1,x2,y2,x3,y4,host_width,host_height
static bool Calibrate_run = false;
static bool init_read= false;
static u16 TOUCH_MAX_X = 0;
static u16 TOUCH_MAX_Y = 0;
static u16 TOUCH_MIN_X = 0;
static u16 TOUCH_MIN_Y = 0;
static u16 MUL_X_PARAMETER = 0;
static u16 DIV_X_PARAMETER = 0;            
static u16 MUL_Y_PARAMETER = 0;
static u16 DIV_Y_PARAMETER = 0;     

//*****added for proc used 20110929
#include <linux/proc_fs.h>
#include <linux/sched.h>
#define STRINGLEN 1024
char Touch_Version_buffer[STRINGLEN];
struct proc_dir_entry *touch_file;
static char APK_CHECK;

//*****added for proc used 20110929

mm_segment_t oldfs;
static struct file *openFile(char *path,int flag,int mode)
{
        struct file *fp = NULL;
        fp=filp_open(path, flag, 0);

	//printk(KERN_ALERT "fp===== %x",fp);
        if(IS_ERR(fp))
        {
                //printk(KERN_ALERT "NO FILE !");
                return NULL;
        }
	init_read = true;
        printk(KERN_ALERT " HAVE FILE");
        return fp;
}

static int readFile(struct file *fp,char *buf,int readlen)
{
        if (fp->f_op && fp->f_op->read)
                return fp->f_op->read(fp,buf,readlen, &fp->f_pos);
        else
                return -1;
}

static int closeFile(struct file *fp)
{
        filp_close(fp,NULL);
        return 0;
}
static void initKernelEnv(void)
{
        oldfs = get_fs();
        set_fs(KERNEL_DS);
}

static void readfile(void)
{
        char buf[1024];
        struct file *fp;
	int ret = 0;
	int i = 0;
	int k = 0;
	int last_number = 0;
	int file_data_number = 0;
	bool data_error;
	initKernelEnv();

	fp=openFile(Calibrate_Name,O_RDONLY,0);

	//printk(KERN_ALERT"fp=  %x ----",fp);
       	if (fp!= NULL)
        {
                memset(buf,0,1024);
                if ((ret=readFile(fp,buf,1024))>0)				//get file data
                        printk(KERN_ALERT"buf:%s\n",buf);
                else
                        printk(KERN_ALERT"read file error %d\n",ret);
                closeFile(fp);
		set_fs(oldfs);
        
		for( i = 0 ; i < 1024 ; i++ )				
		{
			if( buf[i]==',' )				//Calibrate_data[10]
			{
                        	for( k = last_number ; k < i ; k++)
                        	{
                                	if( k == last_number )
                                        	Calibrate_data[file_data_number] = buf[k] -48 ;
                                	else
                                        	Calibrate_data[file_data_number] = Calibrate_data[file_data_number]*10 + buf[k] - 48;
                        	}
                        	last_number=i+1;
				file_data_number++;			
			}
		}
		data_error = false;

		for( i = 0 ; i < 10 ; i++)			//確定得到數值不大於 螢幕解析度
		{
			if ( (i % 2) == 0 )
			{
				if( Calibrate_data[i] > Calibrate_data[8])
					data_error = true;	
			}
			else if( (i % 2) == 1 )
			{
		        	if( Calibrate_data[i] > Calibrate_data[9])      
                                      	data_error = true;
			}
			printk(KERN_INFO "%d\n",Calibrate_data[i] );
		} 
		if( data_error == false )				//沒錯誤發生存入校正數值
		{
        		TOUCH_MAX_X = Calibrate_data[4] * (MAX_12BIT + 1) /Calibrate_data[8];
        		TOUCH_MAX_Y = Calibrate_data[7] * (MAX_12BIT + 1) /Calibrate_data[9];
        		TOUCH_MIN_X = Calibrate_data[2] * (MAX_12BIT + 1) /Calibrate_data[8];
        		TOUCH_MIN_Y = Calibrate_data[1] * (MAX_12BIT + 1) /Calibrate_data[9];
        		MUL_X_PARAMETER = MAX_12BIT * 1000 / (TOUCH_MAX_X - TOUCH_MIN_X);
        		DIV_X_PARAMETER = 1000;             
        		MUL_Y_PARAMETER = MAX_12BIT * 1000 / (TOUCH_MAX_Y - TOUCH_MIN_Y); 
        		DIV_Y_PARAMETER = 1000;
			Calibrate_run = false;            
		}
		else
			Calibrate_run = true;
	}
	else
	{
		//printk(KERN_INFO "=no file=");
		// If no use of parrotVTK wich recalibrate, use TPCalibration_mode_0.apk
		// to create /data/data/com.TPCalibration/CalibrateData.txt
		Calibrate_run = true;
	}
}
// read and check parameter file, end
//#################################################################################################

struct uc6511_point {
	char			count;
	char			number;
	u16			x;
	u16			y;
};

struct uc6511 {
	bus_device		*bus;
	struct input_dev	*input;
	struct work_struct	work;
	struct timer_list	timer;

	struct mutex		mutex;
	unsigned		disabled:1;	/* P: mutex */

	char			phys[32];
};

static void uc6511_work(struct work_struct *work)
{
	struct uc6511_point pt;
	char point_count;
	char point_no;
	int x, y;
	int ret;

	struct uc6511 *ts = container_of(work, struct uc6511, work);
	struct input_dev *input_dev = ts->input;

	ret = i2c_master_recv(ts->bus, (char *)&pt, 6);
	if (ret < 6) {
		printk(KERN_WARNING "uc6511 antec touchscreen can't read i2c values\n");
		return;
	}
	point_count = pt.count;
	point_no    = pt.number;

	// calculate x/y sampling values
	//x = MAX_12BIT - swab16(pt.y);
	//y = swab16(pt.x);
	x = MAX_12BIT - swab16(pt.x);
	y = MAX_12BIT - swab16(pt.y);

//#######################################################
//calibration of x,y coordinates, begin
	if (init_read ==false)
	{
		readfile();
		//init_read=true;
	}

// added for proc test 2011092
	if( APK_CHECK == 0x51)
	{
		Calibrate_run=true ;
		APK_CHECK = 0x00;
	//	printk(KERN_INFO " calibrate_run = true\n");
	}
	else if( APK_CHECK == 0x52)
	{
		Calibrate_run= false;
		APK_CHECK = 0x00;
	//	printk(KERN_INFO " calibrate_run = false\n");
                readfile();
	}
// mask for proc test 20110929

	if ( Calibrate_run == false )
	{
	//	printk(KERN_INFO " calibrate_run = false\n");
	int TmpX = 0, TmpY = 0, TmpX2 = 0, TmpY2 = 0; 
        int px = 0, py =0;
 
    	px=x;
     	py=y; 
     	TmpX2 = (px >= TOUCH_MAX_X) ? (TOUCH_MAX_X) : px; 
     	TmpY2 = (py >= TOUCH_MAX_Y) ? (TOUCH_MAX_Y) : py; 

     	TmpX = (TmpX2 < TOUCH_MIN_X) ? (TOUCH_MIN_X) : TmpX2;
     	TmpY = (TmpY2 < TOUCH_MIN_Y) ? (TOUCH_MIN_Y) : TmpY2;
 
     	TmpX -= TOUCH_MIN_X; 
     	TmpY -= TOUCH_MIN_Y; 
 
     	TmpX = (TmpX) ? TmpX : 0; 
     	TmpY = (TmpY) ? TmpY : 0; 
     	px = TmpX * MUL_X_PARAMETER / DIV_X_PARAMETER; 
     	py = TmpY * MUL_Y_PARAMETER / DIV_Y_PARAMETER; 
 
     	if(px>4095) px=4095;   //add by falcon, 2011/1/24, last_y有出現超過4095的數值, 加上這兩行來預防.
     	if(py>4095) py=4095;
     
     	x = px;  //For TopTouch PDC2 TP
     	y = py;  //For TopTouch PDC2 TP 
	}
//calibration of x,y coordinates, end
//#######################################################
	//printk(KERN_ERR "%d %d\n", x, y);

	fingers_x[point_no - 1] = x;
        fingers_y[point_no - 1] = y;

	//printk(KERN_INFO "%s_work %d / %d-------%d ----%d +\n", I2C_DEVICE_NAME, point_count, point_no,x,y);
#if PRINT_XY
	//printk(KERN_INFO "%s_work %d / %d +\n", I2C_DEVICE_NAME, point_count, point_no);
#endif
	//report_count = report_count + 1;
	if (point_count == point_no) {
	
#if PRINT_XY
	//	printk(KERN_INFO "input\n");
	//	printk(KERN_INFO "%d / %d", point_count, point_no);
#endif
		for (point_no = 0; point_no < point_count; point_no ++) {
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 128);
			input_report_abs(input_dev, ABS_MT_POSITION_X, fingers_x[(unsigned int)point_no]);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, fingers_y[(unsigned int)point_no]);
			input_mt_sync(input_dev);
#if PRINT_XY
	//		printk(KERN_INFO "(%d, %d)", fingers_x[point_no], fingers_y[point_no]);
#endif
		}
		input_sync(input_dev);
	}
	else if (point_count == 0x80) {
		//printk(KERN_INFO "report_count= %d", report_count);
		//report_count = 0;
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_mt_sync(input_dev);
		input_sync(input_dev);
	}
}

static irqreturn_t uc6511_irq(int irq, void *handle)
{
	struct uc6511 *ts = handle;

	/* The repeated conversion sequencer controlled by TMR kicked off too fast.
	 * We ignore the last and process the sample sequence currently in the queue.
	 * It can't be older than 9.4ms
	 */

	if (!work_pending(&ts->work))
		schedule_work(&ts->work);

	return IRQ_HANDLED;
}

static void uc6511_disable(struct uc6511 *ts)
{
	mutex_lock(&ts->mutex);

	if (!ts->disabled) {

		ts->disabled = 1;
		disable_irq(ts->bus->irq);

		cancel_work_sync(&ts->work);
	}

	mutex_unlock(&ts->mutex);
}

static void uc6511_enable(struct uc6511 *ts)
{
	mutex_lock(&ts->mutex);

	if (ts->disabled) {
		ts->disabled = 0;
		enable_irq(ts->bus->irq);
	}

	mutex_unlock(&ts->mutex);
}

static ssize_t uc6511_pen_down_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return 0;
}

static DEVICE_ATTR(pen_down, 0644, uc6511_pen_down_show, NULL);

static ssize_t uc6511_disable_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct uc6511 *ts = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ts->disabled);
}

static ssize_t uc6511_disable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct uc6511 *ts = dev_get_drvdata(dev);
	unsigned long val;
	int error;

	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	if (val)
		uc6511_disable(ts);
	else
		uc6511_enable(ts);

	return count;
}
	
static DEVICE_ATTR(disable, 0664, uc6511_disable_show, uc6511_disable_store);

static struct attribute *uc6511_attributes[] = {
	&dev_attr_disable.attr,
	&dev_attr_pen_down.attr,
	NULL
};

static const struct attribute_group uc6511_attr_group = {
	.attrs = uc6511_attributes,
};

static int __devinit uc6511_construct(bus_device *bus, struct uc6511 *ts)
{
	struct input_dev *input_dev;
//-	struct uc6511_platform_data *pdata = bus->dev.platform_data;
	int err;
	u16 revid=0;
	printk(KERN_ERR "uc6511_construct\n");
	printk(KERN_INFO "%s_construct +\n",I2C_DEVICE_NAME);
	if (!bus->irq) {
		dev_err(&bus->dev, "no IRQ?\n");
		return -ENODEV;
	}

//--	if (!pdata) {
//---		dev_err(&bus->dev, "no platform data?\n");
//----		return -ENODEV;
//-----	}

	input_dev = input_allocate_device();
	if (!input_dev)
		return -ENOMEM;

	ts->input = input_dev;

	INIT_WORK(&ts->work, uc6511_work);
	mutex_init(&ts->mutex);

	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(&bus->dev));

	input_dev->name = "UC6511 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->dev.parent = &bus->dev;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
/*	__set_bit(ABS_X, input_dev->absbit);
	__set_bit(ABS_Y, input_dev->absbit);
	__set_bit(ABS_PRESSURE, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);*/

	input_set_abs_params(input_dev, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	/*err = uc6511_write(bus, AD7879_REG_CTRL2, AD7879_RESET);
	if (err < 0) {
		dev_err(&bus->dev, "Failed to write %s\n", input_dev->name);
		goto err_free_mem;
		}*/

	err = request_irq(bus->irq, uc6511_irq,
			  IRQF_TRIGGER_FALLING, bus->dev.driver->name, ts);

	if (err) {
		dev_err(&bus->dev, "irq %d busy?\n", bus->irq);
		goto err_free_mem;
	}

	err = sysfs_create_group(&bus->dev.kobj, &uc6511_attr_group);
	if (err)
		goto err_free_irq;

	err = input_register_device(input_dev);
	if (err)
		goto err_remove_attr;

	dev_info(&bus->dev, "Rev.%d touchscreen, irq %d\n",
		 revid >> 8, bus->irq);

	printk(KERN_INFO "%s_construct -\n",I2C_DEVICE_NAME);
	return 0;

err_remove_attr:
	sysfs_remove_group(&bus->dev.kobj, &uc6511_attr_group);
err_free_irq:
	free_irq(bus->irq, ts);
err_free_mem:
	input_free_device(input_dev);

	printk(KERN_ERR "%s_construct error\n",I2C_DEVICE_NAME);
	return err;
}

static int __devexit uc6511_destroy(bus_device *bus, struct uc6511 *ts)
{
	uc6511_disable(ts);
	sysfs_remove_group(&ts->bus->dev.kobj, &uc6511_attr_group);
	free_irq(ts->bus->irq, ts);
	input_unregister_device(ts->input);
	dev_dbg(&bus->dev, "unregistered touchscreen\n");

	return 0;
}


static int uc6511_suspend(bus_device *bus, pm_message_t message)
{
	struct uc6511 *ts = dev_get_drvdata(&bus->dev);
        unsigned char buf[2];
        printk("i2cTP:write-----------------suspend \n");
        buf[0]=0x07;
        buf[1]=0x00;

        i2c_master_send(ts->bus, buf, 2);
	uc6511_disable(ts);
	return 0;
}

static int uc6511_resume(bus_device *bus)
{
	struct uc6511 *ts = dev_get_drvdata(&bus->dev);
	unsigned char buf[2];
	uc6511_enable(ts);
        printk("i2cTP:write------------------resume \n");
        buf[0]=0x07;
        buf[1]=0x01;

        i2c_master_send(ts->bus, buf, 2);
        i2c_master_send(ts->bus, buf, 2);

	return 0;
}

//#######################################################
//*****added for proc used 20110929
struct i2c_board_info info;
struct i2c_adapter *adapter;
struct i2c_client *client2;

int proc_read_touch(char *page, char **start, off_t off, int count,  int *eof, void *data)
{
 	//struct uc6511 *ts = dev_get_drvdata(&client2->dev);
        char z[]="ULTRACHHIP I2C Version is 3.0 ";
	int len;
        //unsigned char buf[2];
        printk("i2cTP:write------------------ \n");
        //buf[0]=0x07;
        //buf[1]=0x50;
        //i2c_master_send(ts->bus, buf, 2);
        //msleep(15);

        //int ret;
        //unsigned char data2[6];
        //ret = i2c_master_recv(ts->bus, data2, 6);
        //printk("\ndata= %d %d %d %d %d %d\n",data2[0],data2[1],data2[2],data2[3],data2[4],data2[5]);
        printk("\n==%s==222\n",z);
        strcpy(Touch_Version_buffer,z);

	//printk(" read Touch data \n");
        len = sprintf(page, Touch_Version_buffer); //把global_buffer的内容显示给访问者
        return len;
}
#include <asm/uaccess.h>

int proc_write_touch(struct file *file, const char *buffer,  unsigned long count, void *data)
{
        int len;
	int ret;
	char global_buffer[1024];
        if (count >= STRINGLEN)
                len = STRINGLEN-1;
        else
                len = count;
        ret = copy_from_user(global_buffer, buffer, len);
        //copy_from_user函数将数据从用户空间拷贝到内核空间，此处将用户写入的数据存入global_buffer
        global_buffer[len] = '\0';
        
        if (global_buffer[0]=='A' && global_buffer[1]=='A') {
        	APK_CHECK = 0x51;
        }
        else if (global_buffer[0]=='B' && global_buffer[1]=='B') {
        	APK_CHECK = 0x52;
        }
        else {
        	APK_CHECK = 0x00;
        }
	printk("write count : %d",(int)count);
  
      return len;
}
//*****added for proc used 20110929
//#######################################################

/* All registers are word-sized.
 * AD7879 uses a high-byte first convention.
 */

static int __devinit uc6511_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct uc6511 *ts;
	int error;

	ts = kzalloc(sizeof(struct uc6511), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	i2c_set_clientdata(client, ts);
	ts->bus = client;

	error = uc6511_construct(client, ts);
	if (error) {
		i2c_set_clientdata(client, NULL);
		kfree(ts);
	}

//#######################################################	
		//*****added for proc used 20110929
		touch_file = create_proc_entry("TouchData", 0666, NULL);
    touch_file->read_proc = proc_read_touch;
    touch_file->write_proc = proc_write_touch;	
		//*****added for proc used 20110929
//#######################################################		

	return error;
}

static int __devexit uc6511_remove(struct i2c_client *client)
{
	struct uc6511 *ts = dev_get_drvdata(&client->dev);

	uc6511_destroy(client, ts);
	i2c_set_clientdata(client, NULL);
	kfree(ts);
	return 0;
}

static const struct i2c_device_id uc6511_id[] = {
	{ I2C_DEVICE_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, uc6511_id);

static struct i2c_driver uc6511_driver = {
	.driver = {
		.name	= I2C_DEVICE_NAME,
		.owner	= THIS_MODULE,
	},
	.id_table	= uc6511_id,
	.probe		= uc6511_probe,
	.remove		= __devexit_p(uc6511_remove),
	.suspend	= uc6511_suspend,
	.resume		= uc6511_resume,
};

static int __init uc6511_init(void)
{
	printk(KERN_INFO "%s_init +\n",I2C_DEVICE_NAME);
//#######################################################
//Check Parameter file if exist or not, begin
	readfile();
//Check Parameter file if exist or not, end
//#######################################################
/*
        struct i2c_board_info info;
        struct i2c_adapter *adapter;
        struct i2c_client *client;

	printk(KERN_ERR "initialize struct\n");
        memset(&info, 0, sizeof(struct i2c_board_info));

        info.addr = I2C_DEVICE_ADDRESS;				
        info.irq = gpio_to_irq(OMAP3_DEVKIT_TS_GPIO);	
	printk(KERN_ERR "irq = %d, copy type\n", info.irq);
        strlcpy(info.type, I2C_DEVICE_NAME, I2C_NAME_SIZE);

	printk(KERN_ERR "get adapter %d\n", I2C_ADAPTER_NUMBER);
        adapter = i2c_get_adapter(I2C_ADAPTER_NUMBER);			       	
        if (!adapter) {
                printk("*******get_adapter error!\n");
        }
        client = i2c_new_device(adapter, &info);
*/
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	printk(KERN_ERR "add driver\n");

	return i2c_add_driver(&uc6511_driver);
}
module_init(uc6511_init);

static void __exit uc6511_exit(void)
{
	i2c_del_driver(&uc6511_driver);
	printk(KERN_INFO "%s_exit +\n",I2C_DEVICE_NAME);
}
module_exit(uc6511_exit);
 
MODULE_AUTHOR("Larry Yang <larry.yang@ultrachip.com>");
MODULE_DESCRIPTION("uc6511(-1) touchscreen Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:uc6511");
