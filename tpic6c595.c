/*
 * This driver uses four GPIOs to bitbang the Texas Instruments TPIC595 Power Logic
 * 8-Bit Shift Register and make it accessible as a character-device.
 *
 * A fifth gpio is used as a button to receive input from the user. Its functionality
 * can be configured through ioctl.
 *
 *      Example Device-Tree entry:
 *      ----------------------------
 *          shift_register {
 *              compatible = "texasinstruments,tpic6c595";
 *              status = "okay";
 *              btn-gpios = <&gpio6 17 GPIO_ACTIVE_HIGH>;
 *              ser-in-gpios = <&gpio6 18 GPIO_ACTIVE_HIGH>;
 *              g-gpios = <&gpio7 0 GPIO_ACTIVE_HIGH>;
 *              srck-gpios = <&gpio7 1 GPIO_ACTIVE_HIGH>;
 *              rck-gpios = <&gpio7 7 GPIO_ACTIVE_HIGH>;
 *          };
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "tpic6c595.h"

#define TPIC6C595_DEV_MAJOR             191
#define TPIC6C595_BUFFER_SIZE           1024
#define TPIC6C595_WRITE_UDELAY          10
#define TPIC6C595_FLASH_DURATION        50

static struct class *tpic6c595_class;
static long tpic6c595_flash_duration = TPIC6C595_FLASH_DURATION;

/* represents a single tpic6c595 device */
struct tpic6c595_dev {
    struct device    *dev;
    struct cdev      cdev;
    dev_t            dev_t;
    int              irq_btn;
    u8               mode;
    u8               buf[TPIC6C595_BUFFER_SIZE];
    int              curlen;
    struct gpio_desc *gpio_srck;
    struct gpio_desc *gpio_rck;
    struct gpio_desc *gpio_g;
    struct gpio_desc *gpio_ser_in;
    struct gpio_desc *gpio_btn;
};

/*
 * Forward-declarations
 */
static int tpic6c595_open(struct inode *node, struct file *fd);
static ssize_t tpic6c595_write(struct file* fd, const char __user *buf, size_t len, loff_t *off);
static long tpic6c595_ioctl(struct file *fd, unsigned int req, unsigned long arg);

static ssize_t write_buffer(struct tpic6c595_dev *dev) {
    unsigned int i, j, val;

    udelay(TPIC6C595_WRITE_UDELAY);

    /* set G-pin to disable output */
    gpiod_set_value(dev->gpio_g, 1);
    udelay(TPIC6C595_WRITE_UDELAY);

    /* write every byte in the buffer */
    for (i = 0; i < dev->curlen; i++) {

        /* write every bit in the byte */
        for (j = 0; j < 8; j++) {
            /* set serial-data pin */
            val = ((unsigned char)dev->buf[i] & (1u << j)) > 0;
            gpiod_set_value(dev->gpio_ser_in, val);

            /* set SRCK */
            udelay(TPIC6C595_WRITE_UDELAY);
            gpiod_set_value(dev->gpio_srck, 1);
            udelay(TPIC6C595_WRITE_UDELAY);
            gpiod_set_value(dev->gpio_srck, 0);
        }

        /* set & unset RCK */
        gpiod_set_value(dev->gpio_rck, 1);
        udelay(TPIC6C595_WRITE_UDELAY);
        gpiod_set_value(dev->gpio_rck, 0);
    }

    /* unset G-pin to enable output */
    udelay(TPIC6C595_WRITE_UDELAY);
    gpiod_set_value(dev->gpio_g, 0);

    return dev->curlen;
}

/**
 * Opens the device.
 *
 * @param node contains device-information
 * @param fd to add device-information to
 * @return 0 for success, always succeeds
 */
static int tpic6c595_open(struct inode *node, struct file *fd) {
    struct tpic6c595_dev *dev = container_of(node->i_cdev, struct tpic6c595_dev, cdev);

    fd->private_data = dev;

    return 0;
}

static ssize_t tpic6c595_write(struct file* fd, const char __user *buf, size_t len, loff_t *off) {
    struct tpic6c595_dev *dev = fd->private_data;

    if (len > TPIC6C595_BUFFER_SIZE) {
        dev_err(dev->dev, "length exceeds buffer size");
        return -ENOMEM;
    }

    if (copy_from_user(dev->buf, buf, len) != 0) {
        dev_err(dev->dev, "failed to copy from user");
        return -EBADE;
    }

    dev->curlen = len;

    /* write buffer to the device */
    return write_buffer(dev);
}

static long tpic6c595_ioctl(struct file *fd, unsigned int req, unsigned long arg) {
    long ret = 0;
    struct tpic6c595_dev *dev = fd->private_data;

    switch (req) {
        case TPIC6C595_IOCTL_BTN:
            dev_err(dev->dev, "mode=%ld", arg);
            dev->mode = arg;
            break;
        
        default:
            ret = -EINVAL;
            break;
    }

    return ret;
}

static const struct file_operations tpic6c595_fops = {
        .write          = tpic6c595_write,
        .open           = tpic6c595_open,
        .llseek         = no_llseek,
        .unlocked_ioctl = tpic6c595_ioctl, /* primary interface to access the driver from userspace */
};

static const struct of_device_id tpic_dt_ids[] = {
        { .compatible = "texasinstruments,tpic6c595" },
        { /* sentinel */ },
};

static irqreturn_t tpic6c595_btn_irq(int irq, void *dev_id) {
    struct tpic6c595_dev *dev = dev_id;

    /* btn triggered this irq. ignore the irq if btn is disabled. */
    if (unlikely(dev->mode == TPIC6C595_MODE_BTN_DISABLED))
        return IRQ_NONE;

    /* wake the threaded irq-handler */
    return IRQ_WAKE_THREAD;
}

static irqreturn_t tpic6c595_btn_irq_threaded(int irq, void *dev_id) {
    struct tpic6c595_dev *dev = dev_id;

    switch (dev->mode) {
        case TPIC6C595_MODE_BTN_CLEAR:
            dev_err(dev->dev, "clearing\n");
            dev->buf[0] = 0x00;
            dev->curlen = 1;
            write_buffer(dev);
            break;

        case TPIC6C595_MODE_BTN_FLASH:
            dev_err(dev->dev, "flashing\n");
            dev->curlen = 1;
            dev->buf[0] = 0x00;
            write_buffer(dev);
            msleep(tpic6c595_flash_duration);
            dev->buf[0] = 0xff;
            write_buffer(dev);
            msleep(tpic6c595_flash_duration);
            break;

        default:
            dev_err(dev->dev, "unsupported mode=%d\n", dev->mode);
            break;
    }

    return IRQ_HANDLED;
}

static int tpic6c595_probe(struct platform_device *pdev) {
    int err;
    struct tpic6c595_dev *tpic_dev;

    pr_info("probing for tpic6c595\n");

    /* allocate memory for device */
    tpic_dev = kzalloc(sizeof(struct tpic6c595_dev), GFP_KERNEL);
    if (!tpic_dev)
        return -ENOMEM;

    pr_info("dev: %p", tpic_dev);

    /* set defaults */
    tpic_dev->mode = TPIC6C595_MODE_BTN_DISABLED;

    /* gpio_g */
    tpic_dev->gpio_g = gpiod_get(&pdev->dev, "g", GPIOD_OUT_HIGH);
    if (IS_ERR(tpic_dev->gpio_g)) {
        kfree(tpic_dev);
        pr_err("failed to get gpio_g\n");
        return PTR_ERR(tpic_dev->gpio_g);
    }

    /* gpio_srck */
    tpic_dev->gpio_srck = gpiod_get(&pdev->dev, "srck", GPIOD_OUT_HIGH);
    if (IS_ERR(tpic_dev->gpio_srck)) {
        kfree(tpic_dev);
        pr_err("failed to get gpio_srck\n");
        return PTR_ERR(tpic_dev->gpio_srck);
    }

    /* gpio_rck */
    tpic_dev->gpio_rck = gpiod_get(&pdev->dev, "rck", GPIOD_OUT_HIGH);
    if (IS_ERR(tpic_dev->gpio_rck)) {
        kfree(tpic_dev);
        pr_err("failed to get gpio_rck\n");
        return PTR_ERR(tpic_dev->gpio_rck);
    }

    /* gpio_ser_in */
    tpic_dev->gpio_ser_in = gpiod_get(&pdev->dev, "ser-in", GPIOD_OUT_HIGH);
    if (IS_ERR(tpic_dev->gpio_ser_in)) {
        kfree(tpic_dev);
        pr_err("failed to get gpio_ser_in\n");
        return PTR_ERR(tpic_dev->gpio_ser_in);
    }

    /* gpio_btn */
    tpic_dev->gpio_btn = gpiod_get(&pdev->dev, "btn", GPIOD_IN);
    if (IS_ERR(tpic_dev->gpio_btn)) {
        kfree(tpic_dev);
        pr_err("failed to get gpio_btn\n");
        return PTR_ERR(tpic_dev->gpio_btn);
    }

    /* make major/minor device identifiers and create device */
    tpic_dev->dev_t = MKDEV(TPIC6C595_DEV_MAJOR, 0);
    tpic_dev->dev = device_create(tpic6c595_class, &pdev->dev, tpic_dev->dev_t, tpic_dev, "tpic6c59");
    if (PTR_ERR_OR_ZERO(tpic_dev->dev)) {
        kfree(tpic_dev);
        pr_err("failed to create device\n");
        return PTR_ERR(tpic_dev->dev);
    }

    /* set private driver data for dev */
    dev_set_drvdata(tpic_dev->dev, tpic_dev);

    /* init & add cdev */
    cdev_init(&tpic_dev->cdev, &tpic6c595_fops);
    if ((err = cdev_add(&tpic_dev->cdev, tpic_dev->dev_t, 1)) < 0) {
        pr_err("failed to cdev_add");
        return err;
    }

    /* get irq number for gpio_btn */
    tpic_dev->irq_btn = gpiod_to_irq(tpic_dev->gpio_btn);
    if (tpic_dev->irq_btn < 0) {
        kfree(tpic_dev);
        pr_err("failed to get irq_btn, err=%d", tpic_dev->irq_btn);
        return tpic_dev->irq_btn;
    }

    /* register interrupt-handler for gpio_btn */
    err = request_threaded_irq(tpic_dev->irq_btn, tpic6c595_btn_irq, tpic6c595_btn_irq_threaded,
                               IRQF_TRIGGER_LOW | IRQF_ONESHOT, "tpic6c595", tpic_dev);
    if (err) {
        kfree(tpic_dev);
        pr_err("failed to request irq %d, err=%d\n", tpic_dev->irq_btn, err);
        return err;
    }
    pr_info("registered interrupt-handler for irq_btn=%d\n", tpic_dev->irq_btn);

    pr_err("tpic6c595 probe done\n");

    return 0;
}

static int tpic6c595_remove(struct platform_device *pdev) {
    struct tpic6c595_dev *dev = platform_get_drvdata(pdev);

    if (!dev) {
        dev_err(&pdev->dev, "failed to get driver-data\n");
        return -ENODEV;
    }

    /* unregister irq-handler */
    if (dev->irq_btn >= 0) {
        disable_irq(dev->irq_btn);
        free_irq(dev->irq_btn, dev);
        dev_info(dev->dev, "unregistered irq-handler=%d\n", dev->irq_btn);
    }

    /* free gpios */
    if (dev->gpio_g)
        gpiod_put(dev->gpio_g);
    if (dev->gpio_rck)
        gpiod_put(dev->gpio_rck);
    if (dev->gpio_ser_in)
        gpiod_put(dev->gpio_ser_in);
    if (dev->gpio_srck)
        gpiod_put(dev->gpio_srck);
    if (dev->gpio_btn)
        gpiod_put(dev->gpio_btn);

    pr_info("removed tpic6c595\n");

    return 0;
}

static struct platform_driver tpic6c595_driver = {
        .driver = {
                .name           = "tpic6c595",
                .of_match_table = of_match_ptr(tpic_dt_ids),
                .owner          = THIS_MODULE,
        },
        .probe  = tpic6c595_probe,
        .remove = tpic6c595_remove,
};

static int __init tpic6c595_init(void) {
    int status;

    /* create class */
    tpic6c595_class = class_create(THIS_MODULE, tpic6c595_driver.driver.name);
    if (IS_ERR(tpic6c595_class)) {
        pr_err("failed to create class\n");
        return PTR_ERR(tpic6c595_class);
    }

    /* register character-device for access from userspace */
    status = register_chrdev(TPIC6C595_DEV_MAJOR, tpic6c595_driver.driver.name, &tpic6c595_fops);
    if (status < 0) {
        class_destroy(tpic6c595_class);
        pr_err("failed to register character-device\n");
        return status;
    }

    /* register platform driver, this will result in probing */
    status = platform_driver_register(&tpic6c595_driver);
    if (status < 0) {
        pr_err("failed to register platform driver\n");
        return status;
    }

    pr_info("initialized tpic6c595\n");

    return 0;
}
module_init(tpic6c595_init);

static void __exit tpic6c595_exit(void) {
    /* unregister platform-driver */
    platform_driver_unregister(&tpic6c595_driver);

    /* unregister character-device */
    unregister_chrdev(TPIC6C595_DEV_MAJOR, tpic6c595_driver.driver.name);

    /* unregister class */
    class_destroy(tpic6c595_class);

    pr_info("exit tpic6c595\n");
}
module_exit(tpic6c595_exit);

MODULE_DESCRIPTION("Driver for the Texas Instruments TPIC6C595 8-bit shift register");
MODULE_AUTHOR("Joscha Behrmann, <behrmann@volke-muc.de>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");