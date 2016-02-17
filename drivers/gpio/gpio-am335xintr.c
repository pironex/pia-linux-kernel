/* drivers/gpio/gpio-am335xintr.c
   Interrupting /VFS methods for GPIO pin 61 for AM335x device 
   for pulse counting application.
   Copyright (c) 2015  pradeepkumar soman <pradeep2481@gmail.com>
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/cdev.h>
#include <linux/jiffies.h>
// All the GPIO BANK and registers mapping 
// Defines the GPIO bank 
#define GPIO0_BANK_START_ADDR 0x44E07000
#define GPIO0_BANK_END_ADDR   0x44E07FFF
#define GPIO0_BANK_SIZE GPIO0_BANK_END_ADDR - GPIO0_BANK_START_ADDR
#define GPIO1_BANK_START_ADDR 0x4804C000
#define GPIO1_BANK_END_ADDR   0x4804CFFF
#define GPIO1_BANK_SIZE      GPIO1_BANK_END_ADDR - GPIO1_BANK_START_ADDR
#define GPIO2_BANK_START_ADDR 0x481AC000
#define GPIO2_BANK_END_ADDR   0x481ACFFF
#define GPIO2_BANK_SIZE   GPIO2_BANK_END_ADDR - GPIO2_BANK_START_ADDR
#define GPIO3_BANK_START_ADDR 0x481AE000
#define GPIO3_BANK_END_ADDR   0x481AEFFF
#define GPIO3_BANK_END_SIZE   GPIO2_BANK_END_ADDR - GPIO2_BANK_START_ADDR
// GPIO register offset for each bank
#define GPIO_IRQSTATUS_RAW_0  0x24
#define GPIO_IRQSTATUS_RAW_1  0x28
#define GPIO_IRQSTATUS_0      0x2C
#define GPIO_IRQSTATUS_1      0x30
#define GPIO_IRQSTATUS_SET_0  0x34
#define GPIO_IRQSTATUS_SET_1  0x38
#define GPIO_LEVEL_DETECT_0   0x140
#define GPIO_LEVEL_DETECT_1   0x144
#define GPIO_RISING_DETECT    0x148
#define GPIO_FALLING_DETECT   0x14C
#define GPIO_DEBOUNCENABLE    0x150
#define GPIO_DEBOUNCINGTIME   0x154

// Clock control for the peripherals
#define CM_PER_START_ADDR     0x44E00000
#define CM_PER_END_ADDR     0x44E03FFF
#define CMP_PER_SIZE        0x400
#define CM_PER_GPIO1_CLKCTRL  0xAC
#define CM_PER_GPIO2_CLKCTRL  0xB0
#define CM_PER_GPIO3_CLKCTRL  0xB4 
/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))
// Define the GPIO pin number for which interrupt registerd
#define INTR_GPIO_PIN   GPIO_TO_PIN(1, 29)
#define GPIO_BIT_POSITION 29
#define INTR_GPIO_PIN_FLAG GPIOF_IN
#define INTR_GPIO_PIN_LABEL "GPIO_61"

#define MAX_PINS 1
#define SFT_DEBOUNCINGTIME 30  // milliseconds

MODULE_AUTHOR("psoman");
MODULE_DESCRIPTION("GPIO driver using interrupts");
MODULE_LICENSE("GPL");

static DEFINE_SPINLOCK(event_lock);
static unsigned long pulse_count = 0;
static int major = 0;
extern unsigned long volatile jiffies;

static irqreturn_t gpio_interrupt_handler(int gpio_irq, void *dev_id)
{
	static unsigned long last_jiffies;
	static int last_value;

	unsigned long cur_jiffies = jiffies;
	int value = gpio_get_value(INTR_GPIO_PIN);

	// Invert the value
	value = ~value & 1;

	if (value == last_value)
		return IRQ_HANDLED;

	if ((last_value == 1) && (value == 0) &&
		((cur_jiffies - last_jiffies) > msecs_to_jiffies(SFT_DEBOUNCINGTIME)))
	{
		last_value = value;
		unsigned long flags;
		spin_lock_irqsave(&event_lock, flags);
		// Increment pulse counter
		pulse_count++;
		spin_unlock_irqrestore(&event_lock, flags);
	}

	last_jiffies = cur_jiffies;
	last_value = value;

	return IRQ_HANDLED;
}

static int gpio_irq_open(struct inode *inode, struct file *file)
{
	return 0;
}
static int gpio_irq_release(struct inode *inode, struct file *file)
{
	return 0;
}
static size_t gpio_irq_read(struct file *file, char __user *buffer, size_t count, loff_t *pos)
{
	unsigned long usr_pulse_count;
	unsigned long flags;
	int ret;
	int size;

	spin_lock_irqsave(&event_lock, flags);
	usr_pulse_count = pulse_count;
	spin_unlock_irqrestore(&event_lock, flags);
	// Copy the buffer if condtion is true
	ret = copy_to_user(buffer, &usr_pulse_count, sizeof(usr_pulse_count));
	if (ret == -1)
	{
		return -1;
	}
	size = sizeof(usr_pulse_count);
	return size; 
}
// have a character device interface here
struct file_operations gpio_char_ops = {
owner: THIS_MODULE,
       open:  gpio_irq_open,
       release: gpio_irq_release,
       read: gpio_irq_read,
};
static struct cdev gpio_intr_cdev;
static struct class *gpio_intr_class;

static char *gpio_devnode(struct device *dev, mode_t *mode)
{
	return kasprintf(GFP_KERNEL, "gpiointr/%s", dev_name(dev));
}

static int register_gpio_char_dev(void)
{
	int ret;
	dev_t dev_id;
	ret = alloc_chrdev_region(&dev_id, 0, MAX_PINS, "gpio_interupt");
	if (ret < 0)
	{
		printk(KERN_ERR "Unable to allocate the character device region\n");
		return-1;
	}
	major = MAJOR(dev_id);
	cdev_init(&gpio_intr_cdev, &gpio_char_ops);
	cdev_add(&gpio_intr_cdev, dev_id, MAX_PINS);
	// Create a class for the device
	gpio_intr_class = class_create(THIS_MODULE, "gpio_intr");
	if (IS_ERR(gpio_intr_class)) {
		printk(KERN_ERR "Error creating gpio_intr class.\n");
		cdev_del(&gpio_intr_cdev);
		ret = PTR_ERR(gpio_intr_class);
		goto error_region;
	}
	// update device node here
	gpio_intr_class->devnode = gpio_devnode;
	// Create the device node 
	device_create(gpio_intr_class, NULL, MKDEV(major, 0), NULL, "gpio-intr");
	return 0;
error_region:
	unregister_chrdev_region(dev_id, 0);
	return ret;
}
static void unregister_gpio_char_dev(void)
{
	device_destroy(gpio_intr_class, MKDEV(major, 0));
	class_destroy(gpio_intr_class);
	cdev_del(&gpio_intr_cdev);
	unregister_chrdev_region(MKDEV(major, 0), MAX_PINS);
}
// This is the intialization function for the GPIO interrupt
static int __init gpio_interrupt_init(void)
{
	int gpio_irq, retval, reg;
	void __iomem *clk_reg_mem;
	void __iomem *gpio_reg_mem;

	retval = gpio_request_one(INTR_GPIO_PIN, INTR_GPIO_PIN_FLAG, INTR_GPIO_PIN_LABEL);
	if (retval)
	{
		printk (KERN_ERR "Failed to get GPIO %i for IRQ (error %i)\n", 
				INTR_GPIO_PIN, retval);
		return -1;
	}
	// Get the interrup number using gpio_to_irq()
	gpio_irq = gpio_to_irq(INTR_GPIO_PIN);
	if (gpio_irq < 0)
	{
		printk (KERN_ERR "Failed to get IRQ for GPIO pin#%i (error %i)\n", 
				INTR_GPIO_PIN, gpio_irq);
		return -1;
	}

	// request the interrupt for the GPIO pin request_irq, TRIGER RISING/FALLING
	retval = request_irq(gpio_irq, gpio_interrupt_handler, 0, 
			INTR_GPIO_PIN_LABEL, (void*)NULL);
	if (retval)
	{
		printk (KERN_ERR "Failed to get request IRQ for GPIO pin#%i (error %i)\n", 
				INTR_GPIO_PIN, retval);
		goto request_irq_fail;
	}

	// Trigget on both edges
	retval = irq_set_irq_type(gpio_irq, IRQ_TYPE_EDGE_BOTH);
	if (retval)
	{
		printk (KERN_ERR "Failed to set IRQ type for GPIO pin#%i (error %i)\n", 
				INTR_GPIO_PIN, retval);
		goto request_irq_fail;
	}

	// Get the device in to the memory
	clk_reg_mem = ioremap(CM_PER_START_ADDR, CMP_PER_SIZE);
	if (!clk_reg_mem) {
		printk(KERN_ERR "ioremp failed for Clock register for peripherals\n"); 
		goto request_irq_fail;
	}

	// Enable the clock for the GPIO and debouncing
	iowrite32((0x2 | (0x1 << 18)), clk_reg_mem + CM_PER_GPIO1_CLKCTRL);

	iounmap(clk_reg_mem);
	// Setup the IRQ registers
	gpio_reg_mem = ioremap(GPIO1_BANK_START_ADDR, GPIO1_BANK_SIZE);
	if (!gpio_reg_mem) {
		printk(KERN_ERR "ioremp failed for GPIO1  register for peripherals\n");
		goto clock_fail;
	}
	// Access the IRQ status registers for GPIO1
	// interrupt0
	reg = ioread32(gpio_reg_mem + GPIO_IRQSTATUS_SET_0);
	reg |= 1 << GPIO_BIT_POSITION;
	iowrite32(reg, gpio_reg_mem + GPIO_IRQSTATUS_SET_0);

	// intrrupt1
	reg = ioread32(gpio_reg_mem + GPIO_IRQSTATUS_SET_1);
	reg |= 1 << GPIO_BIT_POSITION;
	iowrite32(reg, gpio_reg_mem + GPIO_IRQSTATUS_SET_1);

	// Set hw debouncimg time to maximum (255 * 31 mcs)
        iowrite32(0xFF, gpio_reg_mem + GPIO_DEBOUNCINGTIME);

        reg = ioread32(gpio_reg_mem + GPIO_DEBOUNCENABLE);
	reg |=  1 << GPIO_BIT_POSITION;
	iowrite32(reg, gpio_reg_mem + GPIO_DEBOUNCENABLE);

	iounmap(gpio_reg_mem);

	retval = register_gpio_char_dev();
	if (retval != -1)
	{
		printk(KERN_INFO "Successfully resgisterd character device for GPIO1_29\n");
		return 0;
	}
	else
	{
		printk(KERN_ERR "Failed to register gpio interupt as character device\n");
	}
clock_fail:
	iounmap(clk_reg_mem);
request_irq_fail:
        free_irq(gpio_to_irq(INTR_GPIO_PIN), (void*)NULL);
	gpio_free(INTR_GPIO_PIN);
	return -1;  
}

static void __exit gpio_interrupt_exit(void)
{
	unregister_gpio_char_dev();
	free_irq(gpio_to_irq(INTR_GPIO_PIN), (void*)NULL);
	gpio_free(INTR_GPIO_PIN);
}
module_init(gpio_interrupt_init);
module_exit(gpio_interrupt_exit);
