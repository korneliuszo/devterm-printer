#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/device.h>

#include "mtp02-ioctl.h"

#define MAX_DEV BIT((MINORBITS-1)) /* ... up to 128 */

struct mtp02_device {
	/* device handling related values */
	dev_t			devt;
	dev_t			cups_devt;
	int			minor;
	struct device		*dev;
	struct device		*cups_dev;
	struct cdev		*cdev;
	struct cdev		*cups_cdev;
	struct spi_device	*spi;

	struct pinctrl *pinctrl;

	struct gpio_desc	*latch_gpio;
	struct gpio_desc	*strobe_gpio;

	struct gpio_desc	*pa_gpio;
	struct gpio_desc	*pan_gpio;
	struct gpio_desc	*pb_gpio;
	struct gpio_desc	*pbn_gpio;

	struct gpio_desc	*pwr_gpio;
	struct gpio_desc	*pap_gpio;

	int pa_step;
	int feed_time;
	uint8_t temp[48];
	int byte_in_line;
	int default_close_feed;
	int default_burnatonce;
	struct mtp02_settings settings;
	atomic_t used;

	uint32_t cups_ras_magic;
	uint8_t cups_page_header[1796];
	uint32_t cups_lines_left;
	uint32_t cups_advance;
	enum {
		CUPS_MAGIC,
		CUPS_HEADER,
		CUPS_LINE,
	} cups_state;

};

static dev_t mtp02_dev = 0;
static DEFINE_IDR(mtp02_idr);
static DEFINE_MUTEX(minor_lock); /* Protect idr accesses */

static struct class *mtp02_class = NULL;

static int mtp02_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	add_uevent_var(env, "DEVMODE=%#o", 0666);
	return 0;
}

static const struct {
	bool pa;
	bool pan;
	bool pb;
	bool pbn;
} steps[] =
{
		{0,1,0,1},
		{0,1,1,0},
		{1,0,1,0},
		{1,0,0,1},
		{0,0,0,0},
};

static void mtp02_step_setup(struct mtp02_device * device, int step)
{
	gpiod_set_value(device->pa_gpio,steps[step].pa);
	gpiod_set_value(device->pan_gpio,steps[step].pan);
	gpiod_set_value(device->pb_gpio,steps[step].pb);
	gpiod_set_value(device->pbn_gpio,steps[step].pbn);

}

static bool mtp02_is_paper(struct mtp02_device * device)
{
	return !(gpiod_get_value(device->pap_gpio));
}

static void mtp02_step(struct mtp02_device * device, int steps)
{
	int i;
	if(steps>0)
	{
		for(i=0;i<steps;i++)
		{
			device->pa_step = (device->pa_step +1)%4;
			mtp02_step_setup(device,device->pa_step);
			msleep(device->feed_time);
		}
	}
	else
	{
		for(i=steps;i>0;i--)
		{
			device->pa_step = (device->pa_step -1)%4;
			mtp02_step_setup(device,device->pa_step);
			msleep(device->feed_time);
		}
	}
	mtp02_step_setup(device,4);
}

static int mtp02_open(struct inode *inode, struct file *file)
{
	struct mtp02_device	*device;

	mutex_lock(&minor_lock);
	device = idr_find(&mtp02_idr, iminor(inode));
	mutex_unlock(&minor_lock);
	if (!device) {
		pr_debug("device: minor %d unknown.\n", iminor(inode));
		return -ENODEV;
	}
	if(atomic_cmpxchg(&device->used,0,1))
		return -EBUSY;

	file->private_data = device;

	gpiod_set_value(device->pwr_gpio,1);
	mtp02_step(device,0);
	device->byte_in_line = 0;

	device->settings.close_feed = device->default_close_feed;
	device->settings.line_feed = 2;
	device->settings.burn_time = 250;
	device->settings.burn_count = 10;
	device->settings.bytesatonce = device->default_burnatonce;

	printk("mtp02: Device open\n");

	if(!mtp02_is_paper(device))
	{
		gpiod_set_value(device->pwr_gpio,0);
		atomic_set(&device->used,0);
		return -EBUSY;
	}

	return 0;
}

static int mtp02_cups_open(struct inode *inode, struct file *file)
{
	struct mtp02_device	*device;

	mutex_lock(&minor_lock);
	device = idr_find(&mtp02_idr, iminor(inode)-MAX_DEV);
	mutex_unlock(&minor_lock);
	if (!device) {
		pr_debug("device: minor %d unknown.\n", iminor(inode));
		return -ENODEV;
	}
	if(atomic_cmpxchg(&device->used,0,1))
		return -EBUSY;

	file->private_data = device;

	gpiod_set_value(device->pwr_gpio,1);
	mtp02_step(device,0);
	device->byte_in_line = 0;

	device->settings.close_feed = device->default_close_feed;
	device->settings.line_feed = 2;
	device->settings.burn_time = 250;
	device->settings.burn_count = 10;
	device->settings.bytesatonce = device->default_burnatonce;

	device->cups_state = CUPS_MAGIC;
	device->cups_advance = 0;

	if(!mtp02_is_paper(device))
	{
		gpiod_set_value(device->pwr_gpio,0);
		atomic_set(&device->used,0);
		return -EBUSY;
	}

	return 0;
}

static int mtp02_release(struct inode *inode, struct file *file)
{
	struct mtp02_device * device = file->private_data;

	mtp02_step(device, device->settings.close_feed);

	gpiod_set_value(device->pwr_gpio,0);

	atomic_set(&device->used,0);
	printk("mtp02: Device close\n");
	return 0;
}

static int mtp02_cups_release(struct inode *inode, struct file *file)
{
	struct mtp02_device * device = file->private_data;

	if(device->cups_advance == 2)
		mtp02_step(device, device->settings.close_feed);

	gpiod_set_value(device->pwr_gpio,0);

	atomic_set(&device->used,0);
	return 0;
}

static long mtp02_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct mtp02_device * device = file->private_data;

	if(!mtp02_is_paper(device))
	{
		return -EBUSY;
	}
	switch(cmd){
	case MTP02_FEED:
		mtp02_step(device, (int)arg);
		break;
	case MTP02_GET_SETTINGS:
	{
		int ret = copy_to_user((void __user *)arg,&device->settings, sizeof(device->settings));
		if(ret < 0){
			printk("Error in MTP02_GET_SETTINGS\n");
			return -1;
		}
		break;
	}
	case MTP02_SET_SETTINGS:
	{
		int ret = copy_from_user(&device->settings, (void __user *)arg, sizeof(device->settings));
		if(ret < 0){
			printk("Error in MTP02_SET_SETTINGS\n");
			return -1;
		}
		break;
	}
	default :
		return -ENOTTY;
	}
	return 0;
}

static void mtp02_burn(struct mtp02_device *device,uint8_t *buf)
{
	spi_write(device->spi,buf,48);
	ndelay(25);
	gpiod_set_value(device->latch_gpio,1);
	ndelay(25);
	gpiod_set_value(device->latch_gpio,0);
	ndelay(25);
	{
		int i;
		int burn_count = device->settings.burn_count;
		for(i=0;i<burn_count;i++)
		{
			gpiod_set_value(device->strobe_gpio,1);
			udelay(device->settings.burn_time);
			gpiod_set_value(device->strobe_gpio,0);
			udelay(14);
		}
	}
}

static void mtp02_burn_segmented(struct mtp02_device *device,uint8_t *buf)
{
	uint8_t tmpbuf[48];
	int i;
	for(i=0;i<48;i+=device->settings.bytesatonce)
	{
		int bytes = min(48-i,device->settings.bytesatonce);
		memset(tmpbuf,0,48);
		memcpy(&tmpbuf[i],&buf[i],bytes);
		mtp02_burn(device,tmpbuf);
	}
}

static ssize_t mtp02_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	struct mtp02_device * device = file->private_data;
	int maxbytes; /* maximum bytes that can be read from ppos to BUFFER_SIZE*/
	int bytes_to_write; /* gives the number of bytes to write*/
	int bytes_writen; /* number of bytes actually writen*/

	if(!mtp02_is_paper(device))
	{
		return -EBUSY;
	}

	maxbytes = 48 - device->byte_in_line;
	if (maxbytes > count)
		bytes_to_write = count;
	else
		bytes_to_write = maxbytes;
	bytes_writen = bytes_to_write - copy_from_user(&device->temp[device->byte_in_line], buf, bytes_to_write);
	*offset += bytes_writen;
	device->byte_in_line+=bytes_writen;
	if(device->byte_in_line == 48)
	{
		mtp02_burn_segmented(device,device->temp);
		mtp02_step(device,device->settings.line_feed);
		device->byte_in_line=0;
	}
	return bytes_writen;
}

static ssize_t mtp02_cups_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	struct mtp02_device * device = file->private_data;
	int maxbytes; /* maximum bytes that can be read from ppos to BUFFER_SIZE*/
	int bytes_to_write; /* gives the number of bytes to write*/
	int bytes_writen; /* number of bytes actually writen*/

	int fullline;
	uint8_t *buff;

	if(!mtp02_is_paper(device))
	{
		return -EBUSY;
	}



	switch(device->cups_state)
	{
		case CUPS_MAGIC:
			fullline = 4;
			buff = (uint8_t*)&device->cups_ras_magic;
			break;
		case CUPS_HEADER:
			fullline = 1796;
			buff = device->cups_page_header;
			break;
		case CUPS_LINE:
			fullline = 48;
			buff = device->temp;
			break;
	}
	maxbytes = fullline - device->byte_in_line;
	if (maxbytes > count)
		bytes_to_write = count;
	else
		bytes_to_write = maxbytes;
	bytes_writen = bytes_to_write - copy_from_user(&buff[device->byte_in_line], buf, bytes_to_write);
	*offset += bytes_writen;
	device->byte_in_line+=bytes_writen;
	if(device->byte_in_line == fullline)
	{
		switch(device->cups_state)
		{
		case CUPS_MAGIC:
			if(device->cups_ras_magic == 0x52615333) // RaS3
				device->cups_state = CUPS_HEADER;
			else
				return -EINVAL;
			break;
		case CUPS_HEADER:
			{
				uint32_t lines;
				uint32_t advance_lines;
				uint32_t width;
				memcpy(&lines,&device->cups_page_header[376],4);
				memcpy(&advance_lines,&device->cups_page_header[256],4);
				memcpy(&width,&device->cups_page_header[372],4);
				if(width != 384)
					return -EINVAL;
				memcpy(&device->cups_advance,&device->cups_page_header[260],4);
				device->settings.close_feed = advance_lines*2;
				device->cups_lines_left = lines;
				device->cups_state = CUPS_LINE;
				break;
			}
		case CUPS_LINE:
			{
				mtp02_burn_segmented(device,device->temp);
				mtp02_step(device,device->settings.line_feed);
			if(--device->cups_lines_left == 0)
			{
				device->cups_state = CUPS_HEADER;
				if(device->cups_advance == 4)
					mtp02_step(device, device->settings.close_feed);
			}
			break;
			}
		}
		device->byte_in_line=0;
	}
	return bytes_writen;
}

static int import_gpio_out(struct mtp02_device *device, struct gpio_desc ** gpio, const char* name, enum gpiod_flags flags)
{
	*gpio = devm_gpiod_get(&device->spi->dev, name, flags);
	if (IS_ERR(*gpio))
		return PTR_ERR(*gpio);
	gpiod_unexport(*gpio);
	return gpiod_direction_output(*gpio,0);
}

static int setup_gpio(struct mtp02_device *device)
{
	int retval;
	retval = import_gpio_out(device,&device->latch_gpio,"latch", GPIOD_OUT_LOW);
	if (retval) return retval;
	retval = import_gpio_out(device,&device->strobe_gpio,"strobe", GPIOD_OUT_LOW);
	if (retval) return retval;
	retval = import_gpio_out(device,&device->pa_gpio,"pa", GPIOD_OUT_LOW);
	if (retval) return retval;
	retval = import_gpio_out(device,&device->pan_gpio,"pan", GPIOD_OUT_LOW);
	if (retval) return retval;
	retval = import_gpio_out(device,&device->pb_gpio,"pb", GPIOD_OUT_LOW);
	if (retval) return retval;
	retval = import_gpio_out(device,&device->pbn_gpio,"pbn", GPIOD_OUT_LOW);
	if (retval) return retval;
	retval = import_gpio_out(device,&device->pwr_gpio,"pwr", GPIOD_OUT_LOW);
	if (retval) return retval;
	device->pap_gpio = devm_gpiod_get(&device->spi->dev, "pap", GPIOD_IN);
	if (IS_ERR(device->pap_gpio))
		return PTR_ERR(device->pap_gpio);
	gpiod_unexport(device->pap_gpio);
	retval = gpiod_direction_input(device->pap_gpio);
	return retval;
}

static int mtp02_get_minor(struct mtp02_device *device)
{
	int retval = -ENOMEM;

	mutex_lock(&minor_lock);
	retval = idr_alloc(&mtp02_idr, device, 0, MAX_DEV, GFP_KERNEL);
	if (retval >= 0) {
		device->minor = retval;
		retval = 0;
	} else if (retval == -ENOSPC) {
		dev_err(&device->spi->dev, "too many mtp02 devices\n");
		retval = -EINVAL;
	}
	mutex_unlock(&minor_lock);
	return retval;
}

static void mtp02_free_minor(struct mtp02_device *dev)
{
	mutex_lock(&minor_lock);
	idr_remove(&mtp02_idr, dev->minor);
	mutex_unlock(&minor_lock);
}

static const struct file_operations mtp02_fops = {
		.owner      = THIS_MODULE,
		.open       = mtp02_open,
		.release    = mtp02_release,
		.unlocked_ioctl = mtp02_ioctl,
		.write       = mtp02_write
};

static const struct file_operations mtp02_cups_fops = {
		.owner      = THIS_MODULE,
		.open       = mtp02_cups_open,
		.release    = mtp02_cups_release,
		.write       = mtp02_cups_write
};

static ssize_t int_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct mtp02_device *device = dev_get_drvdata(dev);
	if (strcmp("feed_time",attr->attr.name)==0)
		return scnprintf(buf, PAGE_SIZE, "%d\n", device->feed_time);
	else if (strcmp("burnatonce",attr->attr.name)==0)
		return scnprintf(buf, PAGE_SIZE, "%d\n", device->default_burnatonce);
	else
		return -EINVAL;
}

static ssize_t int_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct mtp02_device *device = dev_get_drvdata(dev);
	unsigned long val;
	char *endp;
	val = simple_strtoul(buf, &endp, 0);
	if (buf == endp)
		return -EINVAL;

	if (strcmp("feed_time",attr->attr.name)==0)
		device->feed_time = val;
	else if (strcmp("burnatonce",attr->attr.name)==0)
		device->default_burnatonce = val;
	else
		return -EINVAL;
	return len;
}

static DEVICE_ATTR(feed_time, 0644, int_show, int_store);
static DEVICE_ATTR(burnatonce, 0644, int_show, int_store);


static int mtp02_probe(struct spi_device *spi)
{
	struct mtp02_device	*device;
	int			retval;

	/* setup spi parameters */
	spi->mode = 0x00;
	spi->bits_per_word = 8;

	retval = spi_setup(spi);
	if (retval) {
		dev_dbg(&spi->dev, "configuration of SPI interface failed!\n");
		return retval;
	}

	dev_dbg(&spi->dev,
			"spi interface setup: mode 0x%2x, %d bits per word, %dhz max speed",
			spi->mode, spi->bits_per_word, spi->max_speed_hz);

	/* Allocate driver data */
	device = kzalloc(sizeof(*device), GFP_KERNEL);
	if (!device)
		return -ENOMEM;

	if(device_create_file(&spi->dev, &dev_attr_feed_time))
		goto create_file_failed;
	if(device_create_file(&spi->dev, &dev_attr_burnatonce))
		goto create_file_failed2;

	/* Initialize the driver data */
	device->spi = spi;
	device->feed_time = 6;
	device->default_burnatonce=6;

	device->pinctrl = devm_pinctrl_get_select_default(&spi->dev);
	if (IS_ERR(device->pinctrl)) {
		dev_dbg(&spi->dev, "setup of PINMUX failed");
		goto GPIO_failed;
	}

	/* setup GPIO */
	retval = setup_gpio(device);
	if (retval) {
		dev_dbg(&spi->dev, "setup of GPIOs failed");
		goto GPIO_failed;
	}

	retval = device_property_read_u32(&spi->dev, "close-feed",
			&device->default_close_feed);
	if (retval) {
		dev_dbg(&spi->dev, "close-feed error");
		goto GPIO_failed;
	}

	/* determ minor number */
	retval = mtp02_get_minor(device);
	if (retval) {
		dev_dbg(&spi->dev, "get of minor number failed");
		goto minor_failed;
	}

	/* create device */
	device->devt = MKDEV(MAJOR(mtp02_dev), device->minor);
	device->dev = device_create(mtp02_class,
			&spi->dev,
			device->devt,
			device,
			"mtp02.%d",
			device->minor);
	if (IS_ERR(device->dev)) {
		pr_err("mtp02: device register failed\n");
		retval = PTR_ERR(device->dev);
		goto device_create_failed;
	} else {
		dev_dbg(device->dev,
				"created device for major %d, minor %d\n",
				MAJOR(mtp02_dev),
				device->minor);
	}

	/* create cdev */
	device->cdev = cdev_alloc();
	if (!device->cdev) {
		dev_dbg(device->dev, "allocation of cdev failed");
		retval = -ENOMEM;
		goto cdev_failed;
	}
	device->cdev->owner = THIS_MODULE;
	cdev_init(device->cdev, &mtp02_fops);
	retval = cdev_add(device->cdev, device->devt, 1);
	if (retval) {
		dev_dbg(device->dev, "register of cdev failed");
		goto del_cdev;
	}

	device->cups_devt = MKDEV(MAJOR(mtp02_dev), device->minor+MAX_DEV);
	device->cups_dev = device_create(mtp02_class,
			&spi->dev,
			device->cups_devt,
			device,
			"mtp02.%d_cups",
			device->minor);
	if (IS_ERR(device->cups_dev)) {
		pr_err("mtp02: device register failed\n");
		retval = PTR_ERR(device->dev);
		goto cups_device_create_failed;
	} else {
		dev_dbg(device->cups_dev,
				"created device for major %d, minor %d\n",
				MAJOR(mtp02_dev),
				device->minor);
	}

	/* create cdev */
	device->cups_cdev = cdev_alloc();
	if (!device->cups_cdev) {
		dev_dbg(device->cups_dev, "allocation of cdev failed");
		retval = -ENOMEM;
		goto cups_cdev_failed;
	}
	device->cups_cdev->owner = THIS_MODULE;
	cdev_init(device->cups_cdev, &mtp02_cups_fops);
	retval = cdev_add(device->cups_cdev, device->cups_devt, 1);
	if (retval) {
		dev_dbg(device->dev, "register of cdev failed");
		goto del_cups_cdev;
	}

	/* spi setup */
	spi_set_drvdata(spi, device);

	return 0;

	del_cups_cdev:
	cdev_del(device->cups_cdev);
	cups_cdev_failed:
	device_destroy(mtp02_class, device->cups_devt);
	cups_device_create_failed:

	del_cdev:
	cdev_del(device->cdev);
	cdev_failed:
	device_destroy(mtp02_class, device->devt);
	device_create_failed:
	mtp02_free_minor(device);
	minor_failed:
	GPIO_failed:
	device_remove_file(&spi->dev, &dev_attr_burnatonce);
	create_file_failed2:
	device_remove_file(&spi->dev, &dev_attr_feed_time);
	create_file_failed:
	kfree(device);

	return retval;
}

static void mtp02_remove(struct spi_device *spi)
{
	struct mtp02_device	*device = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	device->spi = NULL;

	device_destroy(mtp02_class, device->devt);
	device_destroy(mtp02_class, device->cups_devt);

	cdev_del(device->cdev);
	cdev_del(device->cups_cdev);

	mtp02_free_minor(device);

	device_remove_file(&spi->dev, &dev_attr_burnatonce);
	device_remove_file(&spi->dev, &dev_attr_feed_time);

	kfree(device);

	return 0;
}

static const struct of_device_id mtp02_dt_ids[] = {
		{ .compatible = "devterm,printer-mtp02" },
		{},
};

MODULE_DEVICE_TABLE(of, mtp02_dt_ids);

static const struct spi_device_id mtp02_device_id[] = {
		{ "printer-mtp02", 0 },
		{ }
};
MODULE_DEVICE_TABLE(spi, mtp02_device_id);

static struct spi_driver mtp02_spi_driver = {
		.driver = {
				.name =		"mtp02",
				.owner =	THIS_MODULE,
				.of_match_table = of_match_ptr(mtp02_dt_ids),
		},
		.id_table = mtp02_device_id,
		.probe =	mtp02_probe,
		.remove =	mtp02_remove,

		/*
		 * NOTE:  suspend/resume methods are not necessary here.
		 * We don't do anything except pass the requests to/from
		 * the underlying controller.  The refrigerator handles
		 * most issues; the controller driver handles the rest.
		 */
};


static int __init mtp02_init(void)
{
	int status;

	/*
	 * Claim device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * Last, register the driver which manages those device numbers.
	 */
	status = alloc_chrdev_region(&mtp02_dev, 0, MAX_DEV, "mtp02");
	if (status < 0)
		return status;

	mtp02_class = class_create(THIS_MODULE, "mtp02");
	if (IS_ERR(mtp02_class)) {
		unregister_chrdev(MAJOR(mtp02_dev),
				mtp02_spi_driver.driver.name);
		return PTR_ERR(mtp02_class);
	}
	mtp02_class->dev_uevent = mtp02_uevent;

	status = spi_register_driver(&mtp02_spi_driver);
	if (status < 0) {
		class_destroy(mtp02_class);
		unregister_chrdev(MAJOR(mtp02_dev),
				mtp02_spi_driver.driver.name);
	}

	return status;
}

static void __exit mtp02_exit(void)
{
	spi_unregister_driver(&mtp02_spi_driver);
	class_destroy(mtp02_class);
	unregister_chrdev(MAJOR(mtp02_dev), mtp02_spi_driver.driver.name);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Korneliusz Osmenda <korneliuszo@gmail.com>");

module_init(mtp02_init);
module_exit(mtp02_exit);
