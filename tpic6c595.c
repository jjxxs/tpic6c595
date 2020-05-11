/**
 * This driver uses four GPIOs to bitbang the Texas Instruments TPIC595 Power Logic
 * 8-Bit Shift Register. This drivers makes the shift-register accessible through
 * a character-device. A button can be used to trigger an interrupt which can
 * be used to clear the shift-register or provide some other functionality which
 * can be configured through ioctl. See tpic6c595.h for supported ioctl-requests.
 *
 * Example Device-Tree entry:
 * ----------------------------
 *    shift_register {
 *       compatible = "texasinstruments,tpic6c595";
 *       status = "okay";
 *       btn-gpios = <&gpio6 17 GPIO_ACTIVE_HIGH>; // button is optional
 *       ser-in-gpios = <&gpio6 18 GPIO_ACTIVE_HIGH>;
 *       g-gpios = <&gpio7 0 GPIO_ACTIVE_HIGH>;
 *       srck-gpios = <&gpio7 1 GPIO_ACTIVE_HIGH>;
 *       rck-gpios = <&gpio7 7 GPIO_ACTIVE_HIGH>;
 *    };
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
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

#define CREATE_TRACE_POINTS
#include "tpic6c595-trace.h"

/* define/undefine if functionality for the button should be included or not */
#define TPIC6C595_USE_BUTTON

#ifdef TPIC6C595_USE_BUTTON
    /* time to wait in ms between flashes when using the button to flash */
    #define TPIC6C595_EFFECT_DURATION 50
#endif

#define TPIC6C595_DEV_MAJOR 191     /* major-number of this device */
#define TPIC6C595_DEV_MINORS 32     /* allow up to 32 devices */
#define TPIC6C595_BITBANG_UDELAY 10 /* time to wait in µs between gpio-state switches */
#define TPIC6C595_BUFFER_SIZE 512   /* size of the write-buffer per device */

static struct class *tpic6c595_class;  /* drivers class, initialized in the __init routine */
static LIST_HEAD(device_list);         /* list of all devices managed by this driver */
static DEFINE_MUTEX(device_list_lock); /* synchronizes access to device_list */
static DECLARE_BITMAP(minors, TPIC6C595_DEV_MINORS); /* use bitmap to manage minor-numbers of the device */

/* structure to represent a tpic6c595 device */
struct tpic6c595_dev {
    struct list_head device_entry;  /* entry of this device in the device-list */

    struct device    *dev;
    struct cdev      cdev;
    dev_t            dev_t;

    struct mutex     mutex;         /* used to synchronize write-access */
    int              users;         /* count of users, e.g. open file-handles */
    u8               *buf;          /* buffer used for write-operations */
    u8               *curval;       /* current value of the device, e.g. last written value */
    long             curval_ptr;    /* index of the start of curval */
    long             dev_size;      /* size of the device in bytes */
    long             bitbang_delay; /* time to wait between gpio-state changes */

    struct gpio_desc *gpio_srck;
    struct gpio_desc *gpio_rck;
    struct gpio_desc *gpio_g;
    struct gpio_desc *gpio_ser_in;

#ifdef TPIC6C595_USE_BUTTON
    struct gpio_desc *gpio_btn;
    u8               mode;          /* current mode of the button */
    int              irq_btn;       /* irq of the button */
    long             effect_duration;
    u8               chase_curdir;
    u64              chase_curval;
#endif
};

/*
 * forward-declarations
 */
static int tpic6c595_open(struct inode *node, struct file *fd);
static ssize_t tpic6c595_write(struct file* fd, const char __user *buf, size_t len, loff_t *off);
static long tpic6c595_ioctl(struct file *fd, unsigned int req, unsigned long arg);

/**
 * Writes the devices current buffer to the device. This function
 * does the actual gpio-bitbanging to communicate with the hardware.
 *
 * @param dev the dev to write its buffer to
 * @return count of bytes written
 */
static ssize_t write_buffer(struct tpic6c595_dev *dev, size_t len) {
    int i, j, val;

    mutex_lock(&dev->mutex);

    /* set G-pin to disable output */
    gpiod_set_value(dev->gpio_g, 1);
    udelay(dev->bitbang_delay);

    /* write byte for byte in the buffer */
    for (i = len - 1; i >= 0; i--) {

        /* write every bit in the byte, lsb */
        for (j = 7; j >= 0; j--) {

            /* set serial-data pin */
            val = ((unsigned char)dev->buf[i] & (1u << j)) > 0;
            gpiod_set_value(dev->gpio_ser_in, val);

            /* set SRCK */
            udelay(dev->bitbang_delay);
            gpiod_set_value(dev->gpio_srck, 1);
            udelay(dev->bitbang_delay);
            gpiod_set_value(dev->gpio_srck, 0);
        }

        /* set & unset RCK */
        gpiod_set_value(dev->gpio_rck, 1);
        udelay(dev->bitbang_delay);
        gpiod_set_value(dev->gpio_rck, 0);

        /* add to curval */
        // TODO: fix
        //dev->curval_ptr = dev->curval_ptr % dev->dev_size;
        //dev->curval[dev->curval_ptr++] = dev->buf[i];
    }

    /* unset G-pin to enable output */
    udelay(dev->bitbang_delay);
    gpiod_set_value(dev->gpio_g, 0);

    mutex_unlock(&dev->mutex);

    return len;
}

/**
 * Opens the device for use from userspace with the specified
 * file handle.
 *
 * @param node the node that contains the maj/min
 * @param fd   the file to add device-information to
 * @return 0 for success,
 *         -ENXIO on failure
 */
static int tpic6c595_open(struct inode *node, struct file *fd) {
    struct tpic6c595_dev *dev;
    int err = -ENXIO;

    mutex_lock(&device_list_lock);

    /* find device-entry for the device that is being opened */
    list_for_each_entry(dev, &device_list, device_entry) {
        if (dev->dev_t == node->i_rdev) {
            err = 0;
            break;
        }
    }

    /* associate file with device and increment users */
    if (err == 0) {
        fd->private_data = dev;
        dev->users++;
    }
    else
        pr_err("failed to open unassigned minor\n");

    mutex_unlock(&device_list_lock);

    return err;
}

/**
 * Closes the device to be used from userspace with
 * the specified file-handle.
 *
 * @param node the node that contains the maj/min
 * @param fd   the file to close
 * @return 0 on success,
 *         -ENXIO on failure
 */
static int tpic6c595_release(struct inode *node, struct file *fd) {
    struct tpic6c595_dev *dev = fd->private_data;

    if (!dev)
        return -ENXIO;

    /* file was closed, decrement users */
    mutex_lock(&device_list_lock);
    fd->private_data = NULL;
    dev->users--;
    mutex_unlock(&device_list_lock);

    return 0;
}

/**
 * Writes to the device.
 *
 * @param fd  the file-handle to write to
 * @param buf the buffer to write
 * @param len the length of the buffer
 * @param off the offset within the buffer, not supported
 * @return count of bytes written on success,
 *         -ENXIO or -EBADE on failure
 */
static ssize_t tpic6c595_write(struct file* fd, const char __user *buf, size_t len, loff_t *off) {
    struct tpic6c595_dev *dev = fd->private_data;
    size_t total, cur, tx = 0;

    if (!dev)
        return -ENXIO;

    for (total = 0; total < len; total += TPIC6C595_BUFFER_SIZE) {

        /* copy buffer_size at most per transfer */
        cur = len - total > TPIC6C595_BUFFER_SIZE ? TPIC6C595_BUFFER_SIZE : len - total;
        if (copy_from_user(dev->buf, &buf[total], cur) != 0) {
            dev_err(dev->dev, "failed to copy from user\n");
            return -EBADE;
        }

        /* write the current buffer */
        tx += write_buffer(dev, len);
    }

    return tx;
}

/**
 * Reads the current value of the device. This doesn't
 * perform any actual hardware-read. It returns the bytes
 * that were last written to the device.
 *
 * @param fd  the file-handle to read from
 * @param buf the buffer to read into
 * @return count of bytes read on success,
 *         -ENXIO or -EBADE on failure
 */
static ssize_t tpic6c595_read(struct file *fd, char __user *buf, size_t len, loff_t *f_pos) {
    ssize_t res;
    struct tpic6c595_dev *dev = fd->private_data;

    if (!dev)
        return -ENXIO;

    // TODO: fix

    /* copy up to dev->dev_size bytes */
    mutex_lock(&dev->mutex);
    res = len < dev->dev_size ? len : dev->dev_size;
    if (copy_to_user(buf, dev->curval, res) != 0) {
        dev_err(dev->dev, "failed to copy to user\n");
        res = -EBADE;
    }
    mutex_unlock(&dev->mutex);

    return res;
}

/**
 * Sets the size of the device. This will also allocate
 * the buffers of the device if needed.
 *
 * @param dev  the device to set the size for
 * @param size the size to set
 * @return 0 on success,
 *         -ENOMEM on failure
 */
static int tpic6c595_set_devsize(struct tpic6c595_dev *dev, long size) {
    int err = 0;

    /* set devices size and reallocate curval-buffer if necessary */
    mutex_lock(&dev->mutex);
    dev->dev_size = size;
    if (dev->dev_size < size) {
        dev->curval = krealloc(dev->curval, size, GFP_KERNEL);

        if (!dev->dev_size)
            err = -ENOMEM;
    }
    mutex_unlock(&dev->mutex);

    return err;
}

/**
 * Configures the device. @see tpic6c595.h for supported requests
 * and their return-values.
 *
 * @param fd  the device to configure
 * @param req the configuration request
 * @param arg the configuration payload
 * @return 0 on success,
 *         -EINVAL on failure
 */
static long tpic6c595_ioctl(struct file *fd, unsigned int req, unsigned long arg) {
    long ret = 0;
    struct tpic6c595_dev *dev = fd->private_data;

    switch (req) {

        case TPIC6C595_IOCTL_BTN:
            dev_err(dev->dev, "setting mode=%ld", arg);
            dev->mode = arg;
            break;

        case TPIC6C595_IOCTL_EFFECT_DURATION:
            dev_info(dev->dev, "setting effect_duration=%ld", arg);
            dev->effect_duration = arg;
            break;

        case TPIC6C595_IOCTL_BITBANG_DELAY:
            dev_info(dev->dev, "setting bitbang_delay=%ld", arg);
            dev->bitbang_delay = arg;
            break;

        case TPIC6C595_IOCTL_SIZE:
            dev_info(dev->dev, "setting dev_size=%ld", arg);
            tpic6c595_set_devsize(dev, arg);
            break;

        default:
            ret = -EINVAL;
            break;
    }

    return ret;
}

static const struct file_operations tpic6c595_fops = {
        .write          = tpic6c595_write,    /* write to the device */
        .read           = tpic6c595_read,     /* current value of the device, e.g. last written bytes */
        .open           = tpic6c595_open,     /* open the device */
        .release        = tpic6c595_release,  /* close the device */
        .llseek         = no_llseek,          /* don't support seek */
        .unlocked_ioctl = tpic6c595_ioctl,    /* primary interface to access the driver from userspace */
};

/* used to map device-tree entries to the matching driver */
static const struct of_device_id tpic_dt_ids[] = {
        { .compatible = "texasinstruments,tpic6c595" },
        { /* sentinel */ },
};

#ifdef TPIC6C595_USE_BUTTON
/**
 * Top-half of the interrupt-handler triggered by the button. If the button
 * was configured using ioctl() to have some functionality the top-half
 * will wake up the bottom-half to be executed.
 *
 * @param irq the interrupt request
 * @param dev_id the device that triggered the interrupt
 * @return IRQ_NONE if interrupts are disabled, IRQ_WAKE_THREAD otherwise
 */
static irqreturn_t tpic6c595_btn_irq(int irq, void *dev_id) {
    struct tpic6c595_dev *dev = dev_id;

    /* btn triggered this irq. ignore the irq if btn is disabled. */
    if (unlikely(dev->mode == TPIC6C595_MODE_BTN_DISABLED))
        return IRQ_NONE;

    /* wake the threaded irq-handler */
    return IRQ_WAKE_THREAD;
}

/**
 * Clears the device, e.g. overwrites it with zeroes.
 *
 * @param dev the device to clear
 */
static void tpic6c595_clear(struct tpic6c595_dev *dev) {
    int i;

    /* fill the buffer with zeroes and write them*/
    for (i = 0; i < dev->dev_size; i++)
        dev->buf[i] = 0x00;
    write_buffer(dev, dev->dev_size);
}

/**
 * Flashes the device, e.g. fills it with zeroes, waits
 * dev->effect_duration and then fills it with ones.
 *
 * @param dev the device to flash
 */
static void tpic6c595_flash(struct tpic6c595_dev *dev) {
    int i;

    /* write zeroes and wait */
    for (i = 0; i < dev->dev_size; i++)
        dev->buf[i] = 0x00;
    write_buffer(dev, dev->dev_size);
    msleep(dev->effect_duration);

    /* write ones and wait */
    for (i = 0; i < dev->dev_size; i++)
        dev->buf[i] = 0xff;
    write_buffer(dev, dev->dev_size);
    msleep(dev->effect_duration);
}

/* helper union to easily convert u64 into bytes */
union u64_to_u8 {
    u64 val;
    u8  parts[sizeof(u64)];
};

/**
 * Chases the devices value, e.g. increases the value until the limit
 * is reached then starts decreasing the value until the value one.
 * This is similar to the 'knightrider'-effect.
 *
 * @param dev the device to apply the chase-effect to
 */
static void tpic6c595_chase(struct tpic6c595_dev *dev) {
    int i;
    union u64_to_u8 tmp;

    /* switch direction on begin/end of value-range*/
    if (dev->chase_curval == 1)
        dev->chase_curdir = 1;
    else if (dev->chase_curval == (1u << (dev->dev_size * 8 - 1)))
        dev->chase_curdir = 0;

    /* increase/decrease value according to current direction */
    if (dev->chase_curdir == 1)
        dev->chase_curval = dev->chase_curval << 1;
    else
        dev->chase_curval = dev->chase_curval >> 1;

    /* copy value to buffer and write */
    tmp.val = dev->chase_curval;
    for (i = 0; i < sizeof(u64); i++)
        dev->buf[i] = tmp.parts[i];
    write_buffer(dev, dev->dev_size);
    msleep(dev->effect_duration);
}

/**
 * Bottom-half of the interrupt-handler. Performs an action according
 * to the devices mode which can be configured through ioctl().
 *
 * @param irq    the interrupt request
 * @param dev_id the device that triggered the interrupt
 * @return always IRQ_HANDLED
 */
static irqreturn_t tpic6c595_btn_irq_threaded(int irq, void *dev_id) {
    struct tpic6c595_dev *dev = dev_id;

    switch (dev->mode) {
        case TPIC6C595_MODE_BTN_CLEAR:
            dev_info(dev->dev, "clearing");
            tpic6c595_clear(dev);
            break;

        case TPIC6C595_MODE_BTN_FLASH:
            dev_info(dev->dev, "flashing");
            tpic6c595_flash(dev);
            break;

        case TPIC6C595_MODE_BTN_CHASER:
            dev_info(dev->dev, "chasing");
            tpic6c595_chase(dev);
            break;

        default:
            dev_err(dev->dev, "unsupported mode=%d\n", dev->mode);
            break;
    }

    return IRQ_HANDLED;
}
#endif

/**
 * Probing. Responsible for the initialization of a concrete
 * device after a device-tree-entry was matched to a device
 * managed by this driver.
 *
 * @param pdev the platform_device to probe
 * @return zero on success,
 *         non-zero error-code on failure
 */
static int tpic6c595_probe(struct platform_device *pdev) {
    int err = 0;
    struct tpic6c595_dev *tpic_dev;
    unsigned long minor;

    pr_info("probing for tpic6c595\n");

    /* allocate memory for device */
    tpic_dev = kzalloc(sizeof(struct tpic6c595_dev), GFP_KERNEL);
    if (!tpic_dev)
        return -ENOMEM;

    /* allocate the devices buffer */
    tpic_dev->buf = kzalloc(TPIC6C595_BUFFER_SIZE, GFP_KERNEL);
    if (!tpic_dev->buf) {
        err = -ENOMEM;
        goto err_free_dev;
    }

    /* init list-entry, lock the list and find the next available minor-number */
    INIT_LIST_HEAD(&tpic_dev->device_entry);
    mutex_lock(&device_list_lock);
    minor = find_first_zero_bit(minors, TPIC6C595_DEV_MINORS);

    /* if available minor found, create device*/
    if (minor >= TPIC6C595_DEV_MINORS)
        err = -ENODEV;
    else {
        tpic_dev->dev_t = MKDEV(TPIC6C595_DEV_MAJOR, minor);
        tpic_dev->dev = device_create(tpic6c595_class, &pdev->dev, tpic_dev->dev_t, tpic_dev, "tpic6c595.%ld", minor);
        err = PTR_ERR_OR_ZERO(tpic_dev->dev);

        /* if device was created, reserve the minor-number and add the device to list*/
        if (err == 0) {
            set_bit(minor, minors);
            list_add(&tpic_dev->device_entry, &device_list);

            /* set private driver data for dev */
            dev_set_drvdata(tpic_dev->dev, tpic_dev);
        }
    }
    mutex_unlock(&device_list_lock);

    /* return early if failed to create device */
    if (err != 0) {
        pr_err("no minor numbers available or failed to create device\n");
        goto err_free_buf;
    }

    /* init & add cdev */
    cdev_init(&tpic_dev->cdev, &tpic6c595_fops);
    if ((err = cdev_add(&tpic_dev->cdev, tpic_dev->dev_t, 1)) < 0) {
        pr_err("failed to add character device\n");
        goto err_free_buf;
    }

    /* set bitbang-delay and allocate devices cur-val buffer */
    tpic_dev->bitbang_delay   = TPIC6C595_BITBANG_UDELAY;
    if (tpic6c595_set_devsize(tpic_dev, 1) < 0) {
        pr_err("failed to set devsize\n");
        goto err_free_buf;
    }

    /* init device mutex */
    mutex_init(&tpic_dev->mutex);

    /* gpio_g */
    tpic_dev->gpio_g = gpiod_get(&pdev->dev, "g", GPIOD_OUT_HIGH);
    if (IS_ERR(tpic_dev->gpio_g)) {
        err = PTR_ERR(tpic_dev->gpio_g);
        pr_err("failed to get gpio_g\n");
        goto err_free_curval;
    }

    /* gpio_srck */
    tpic_dev->gpio_srck = gpiod_get(&pdev->dev, "srck", GPIOD_OUT_HIGH);
    if (IS_ERR(tpic_dev->gpio_srck)) {
        err = PTR_ERR(tpic_dev->gpio_srck);
        pr_err("failed to get gpio_srck\n");
        goto err_free_curval;
    }

    /* gpio_rck */
    tpic_dev->gpio_rck = gpiod_get(&pdev->dev, "rck", GPIOD_OUT_HIGH);
    if (IS_ERR(tpic_dev->gpio_rck)) {
        err = PTR_ERR(tpic_dev->gpio_rck);
        pr_err("failed to get gpio_rck\n");
        goto err_free_curval;
    }

    /* gpio_ser_in */
    tpic_dev->gpio_ser_in = gpiod_get(&pdev->dev, "ser-in", GPIOD_OUT_HIGH);
    if (IS_ERR(tpic_dev->gpio_ser_in)) {
        err = PTR_ERR(tpic_dev->gpio_ser_in);
        pr_err("failed to get gpio_ser_in\n");
        goto err_free_curval;
    }

#ifdef TPIC6C595_USE_BUTTON
    /* defaults used by button */
    tpic_dev->effect_duration = TPIC6C595_EFFECT_DURATION;
    tpic_dev->mode            = TPIC6C595_MODE_BTN_DISABLED;
    tpic_dev->chase_curval    = 1;

    /* gpio_btn */
    tpic_dev->gpio_btn = gpiod_get(&pdev->dev, "btn", GPIOD_IN);
    if (IS_ERR(tpic_dev->gpio_btn)) {
        err = PTR_ERR(tpic_dev->gpio_btn);
        pr_err("failed to get gpio_btn\n");
        goto err_free_curval;
    }

    /* get irq number for gpio_btn */
    tpic_dev->irq_btn = gpiod_to_irq(tpic_dev->gpio_btn);
    if (tpic_dev->irq_btn < 0) {
        err = -EBADSLT;
        pr_err("failed to get irq_btn, err=%d", tpic_dev->irq_btn);
        goto err_free_curval;
    }

    /* register interrupt-handler for gpio_btn */
    err = request_threaded_irq(tpic_dev->irq_btn, tpic6c595_btn_irq, tpic6c595_btn_irq_threaded,
                               IRQF_TRIGGER_LOW | IRQF_ONESHOT, "tpic6c595", tpic_dev);
    if (err) {
        pr_err("failed to request irq %d, err=%d\n", tpic_dev->irq_btn, err);
        goto err_free_curval;
    }
    pr_info("registered interrupt-handler for irq_btn=%d\n", tpic_dev->irq_btn);
#endif

    /* set platform driver data */
    platform_set_drvdata(pdev, tpic_dev);

    pr_info("tpic6c595 probe done\n");

    return 0;

    /* free memory on failure */
    err_free_curval:
    kfree(tpic_dev->curval);
    err_free_buf:
    kfree(tpic_dev->buf);
    err_free_dev:
    kfree(tpic_dev);
    return err;
}

/**
 * Removes a previously probed device. This frees all resources
 * associated with the given device.
 *
 * @param pdev the platform-device to remove
 * @return zero on success,
 *         non-zero error-code on failure
 */
static int tpic6c595_remove(struct platform_device *pdev) {
    struct tpic6c595_dev *dev = platform_get_drvdata(pdev);

    if (!dev) {
        dev_err(&pdev->dev, "failed to get driver-data\n");
        return -ENODEV;
    }

    mutex_lock(&device_list_lock);

    /* delete list-entry and destroy the device, free the minor number */
    list_del(&dev->device_entry);
    device_destroy(tpic6c595_class, dev->dev_t);
    clear_bit(MINOR(dev->dev_t), minors);

    /* free gpios */
    if (dev->gpio_g)
        gpiod_put(dev->gpio_g);
    if (dev->gpio_rck)
        gpiod_put(dev->gpio_rck);
    if (dev->gpio_ser_in)
        gpiod_put(dev->gpio_ser_in);
    if (dev->gpio_srck)
        gpiod_put(dev->gpio_srck);

#ifdef TPIC6C595_USE_BUTTON
    /* unregister button irq and free btn-gpio */
    if (dev->irq_btn >= 0) {
        disable_irq(dev->irq_btn);
        free_irq(dev->irq_btn, dev);
        dev_info(dev->dev, "unregistered irq-handler=%d\n", dev->irq_btn);
    }
    if (dev->gpio_btn)
        gpiod_put(dev->gpio_btn);
#endif

    /* free memory */
    kfree(dev);

    mutex_unlock(&device_list_lock);

    pr_info("removed tpic6c595\n");

    return 0;
}

/* platform-driver representation of this driver */
static struct platform_driver tpic6c595_driver = {
        .driver = {
                .name           = "tpic6c595",
                .of_match_table = of_match_ptr(tpic_dt_ids),
                .owner          = THIS_MODULE,
        },
        .probe  = tpic6c595_probe,
        .remove = tpic6c595_remove,
};

/**
 * Initializes this driver. E.g. creates it's class, registers with the system
 * as a character-device etc.
 *
 * @return zero on success,
 *         non-zero error-code on failure
 */
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
        unregister_chrdev(TPIC6C595_DEV_MAJOR, tpic6c595_driver.driver.name);
        class_destroy(tpic6c595_class);
        pr_err("failed to register platform driver\n");
        return status;
    }

    pr_info("initialized tpic6c595\n");

    return 0;
}
module_init(tpic6c595_init);

/**
 * Uninitializes this driver, e.g. undoes everything
 * that was done during initialization.
 */
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
MODULE_AUTHOR("Joscha Behrmann, <behrmann@hm.edu>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");