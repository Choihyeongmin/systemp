#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include "speed_proto.h"

#define DEVICE_NAME "speed_tx"
static int major;

/* GPIO descriptors */
static struct gpio_desc *gpiod_data_tx;
static struct gpio_desc *gpiod_clk_tx;
static struct gpio_desc *gpiod_button;
static struct gpio_desc *gpiod_data_rx_in;
static struct gpio_desc *gpiod_clk_rx_in;
static int button_irq;
static int clk_rx_irq;

/* Current speed delta value */
static int8_t current_delta = 0;

/* Variable for RX frame */
static speed_frame_t rx_frame;
static bool rx_ready = false;
static DECLARE_WAIT_QUEUE_HEAD(tx_wq);

/* Function to bit-bang a frame over DATA/CLK */
static void tx_send_frame(const speed_frame_t *frame, struct gpio_desc *data, struct gpio_desc *clk)
{
    int i, bit;
    uint8_t *bytes = (uint8_t *)frame;

    for (i = 0; i < FRAME_LEN; i++) {
        for (bit = 7; bit >= 0; bit--) {
            gpiod_set_value(data, (bytes[i] >> bit) & 0x1);
            gpiod_set_value(clk, 1);
            udelay(1);
            gpiod_set_value(clk, 0);
            udelay(1);
        }
    }
}

/* IRQ handler for button press */
static irqreturn_t button_isr(int irq, void *dev_id)
{
    speed_frame_t frame;

    current_delta += 10;
    if (current_delta > 100)
        current_delta = 100;

    frame.start = TX_START_BYTE;
    frame.type  = CMD_ACCEL;
    frame.value = current_delta;
    frame.checksum = calc_checksum(&frame);

    tx_send_frame(&frame, gpiod_data_tx, gpiod_clk_tx);
    return IRQ_HANDLED;
}

/* IRQ handler for RX->TX clock rising edge */
static irqreturn_t clk_rx_isr(int irq, void *dev_id)
{
    static int byte_idx = 0, bit_idx = 7;
    uint8_t *b = (uint8_t*)&rx_frame;
    int bit_val = gpiod_get_value(gpiod_data_rx_in);
    if (bit_val)
        b[byte_idx] |= (1 << bit_idx);

    if (--bit_idx < 0) {
        bit_idx = 7;
        if (++byte_idx >= FRAME_LEN) {
            rx_ready = true;
            byte_idx = 0;
            wake_up_interruptible(&tx_wq);
        }
    }
    return IRQ_HANDLED;
}

/* File operations: open and release */
static int tx_open(struct inode *inode, struct file *file) { return 0; }
static int tx_release(struct inode *inode, struct file *file) { return 0; }

/* write: user can send custom delta */
static ssize_t tx_write(struct file *file, const char __user *buf, size_t count, loff_t *pos)
{
    speed_frame_t frame;
    int8_t user_val;

    if (count < 1)
        return -EINVAL;

    if (copy_from_user(&user_val, buf, 1))
        return -EFAULT;

    frame.start = TX_START_BYTE;
    frame.type  = CMD_ACCEL;
    frame.value = user_val;
    frame.checksum = calc_checksum(&frame);

    tx_send_frame(&frame, gpiod_data_tx, gpiod_clk_tx);
    return 1;
}

/* read: receive ACK from RX */
static ssize_t tx_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    if (count < FRAME_LEN)
        return -EINVAL;
    wait_event_interruptible(tx_wq, rx_ready);
    if (calc_checksum(&rx_frame) != rx_frame.checksum)
        return -EIO;
    if (copy_to_user(buf, &rx_frame, FRAME_LEN))
        return -EFAULT;
    rx_ready = false;
    return FRAME_LEN;
}

static const struct file_operations tx_fops = {
    .owner   = THIS_MODULE,
    .open    = tx_open,
    .release = tx_release,
    .write   = tx_write,
    .read    = tx_read,
};

static int __init speed_tx_init(void)
{
    major = register_chrdev(0, DEVICE_NAME, &tx_fops);
    if (major < 0) {
        pr_err("Failed to register char device\n");
        return major;
    }

    gpiod_data_tx    = gpiod_get(NULL, "data_tx", GPIOD_OUT_LOW);
    gpiod_clk_tx     = gpiod_get(NULL, "clk_tx",  GPIOD_OUT_LOW);
    gpiod_button     = gpiod_get(NULL, "button",  GPIOD_IN);
    gpiod_data_rx_in = gpiod_get(NULL, "data_rx_in", GPIOD_IN);
    gpiod_clk_rx_in  = gpiod_get(NULL, "clk_rx_in",  GPIOD_IN);
    if (IS_ERR(gpiod_data_tx) || IS_ERR(gpiod_clk_tx) || IS_ERR(gpiod_button) ||
        IS_ERR(gpiod_data_rx_in) || IS_ERR(gpiod_clk_rx_in)) {
        pr_err("Failed to get GPIOs\n");
        unregister_chrdev(major, DEVICE_NAME);
        return -ENODEV;
    }

    button_irq = gpiod_to_irq(gpiod_button);
    request_irq(button_irq, button_isr, IRQF_TRIGGER_FALLING, DEVICE_NAME, NULL);

    clk_rx_irq = gpiod_to_irq(gpiod_clk_rx_in);
    request_irq(clk_rx_irq, clk_rx_isr, IRQF_TRIGGER_RISING, DEVICE_NAME, NULL);

    pr_info("speed_tx driver loaded, major=%d\n", major);
    return 0;
}

static void __exit speed_tx_exit(void)
{
    free_irq(button_irq, NULL);
    free_irq(clk_rx_irq, NULL);
    gpiod_put(gpiod_button);
    gpiod_put(gpiod_clk_rx_in);
    gpiod_put(gpiod_data_rx_in);
    gpiod_put(gpiod_clk_tx);
    gpiod_put(gpiod_data_tx);
    unregister_chrdev(major, DEVICE_NAME);
    pr_info("speed_tx driver unloaded\n");
}

module_init(speed_tx_init);
module_exit(speed_tx_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("작성자");
MODULE_DESCRIPTION("Speed TX driver for autonomous speed control");
