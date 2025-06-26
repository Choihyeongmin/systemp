#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include "speed_proto.h"

#define DEVICE_NAME "speed_rx"
static int major;

/* GPIO descriptors */
static struct gpio_desc *gpiod_data_rx;
static struct gpio_desc *gpiod_clk_tx;
static struct gpio_desc *gpiod_data_tx_back;
static struct gpio_desc *gpiod_clk_tx_back;

/* IRQ number */
static int clk_irq;

/* Frame reception state */
static speed_frame_t recv_frame;
static int byte_idx = 0, bit_idx = 7;
static bool frame_ready = false;
static wait_queue_head_t rx_wq;

/* Workqueue for deceleration */
static struct delayed_work decel_work;
#define DECEL_INTERVAL_SEC 1
#define DECEL_STEP         10

/* Bit-bang send for ACK */
static void rx_send_frame(const speed_frame_t *frame)
{
    int i, bit;
    uint8_t *bytes = (uint8_t *)frame;

    for (i = 0; i < FRAME_LEN; i++) {
        for (bit = 7; bit >= 0; bit--) {
            gpiod_set_value(gpiod_data_tx_back, (bytes[i] >> bit) & 0x1);
            gpiod_set_value(gpiod_clk_tx_back, 1);
            udelay(1);
            gpiod_set_value(gpiod_clk_tx_back, 0);
            udelay(1);
        }
    }
}

/* Deceleration work function */
static void decel_work_fn(struct work_struct *work)
{
    if (recv_frame.value > 0) {
        recv_frame.value -= DECEL_STEP;
        if (recv_frame.value < 0)
            recv_frame.value = 0;

        /* Send deceleration ACK */
        speed_frame_t ack = {
            .start    = RX_START_BYTE,
            .type     = STATUS_NORMAL,
            .value    = recv_frame.value,
            .checksum = 0
        };
        ack.checksum = calc_checksum(&ack);
        rx_send_frame(&ack);

        /* Reschedule if still above 0 */
        if (recv_frame.value > 0)
            schedule_delayed_work(&decel_work,
                                  msecs_to_jiffies(DECEL_INTERVAL_SEC * 1000));
    }
}

/* IRQ handler for RX clock rising edge */
static irqreturn_t clk_isr(int irq, void *dev_id)
{
    int bit_val = gpiod_get_value(gpiod_data_rx);
    uint8_t *bytes = (uint8_t *)&recv_frame;

    if (bit_val)
        bytes[byte_idx] |= (1 << bit_idx);

    if (--bit_idx < 0) {
        bit_idx = 7;
        if (++byte_idx >= FRAME_LEN) {
            frame_ready = true;
            byte_idx = 0;
            bit_idx = 7;
            wake_up_interruptible(&rx_wq);

            /* Cancel any pending decel work */
            cancel_delayed_work_sync(&decel_work);

            /* FSM update can be done here, then send ACK */
            speed_frame_t ack = {
                .start    = RX_START_BYTE,
                .type     = STATUS_NORMAL,
                .value    = recv_frame.value, /* echo speed */
                .checksum = 0
            };
            ack.checksum = calc_checksum(&ack);
            rx_send_frame(&ack);

            /* Schedule deceleration */
            INIT_DELAYED_WORK(&decel_work, decel_work_fn);
            schedule_delayed_work(&decel_work,
                                  msecs_to_jiffies(DECEL_INTERVAL_SEC * 1000));
        }
    }
    return IRQ_HANDLED;
}

static int rx_open(struct inode *inode, struct file *file)
{
    init_waitqueue_head(&rx_wq);
    frame_ready = false;
    byte_idx = 0;
    bit_idx = 7;
    return 0;
}

static int rx_release(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t rx_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    if (count < FRAME_LEN)
        return -EINVAL;

    /* Wait for a frame or decel update ack */
    wait_event_interruptible(rx_wq, frame_ready);

    if (calc_checksum(&recv_frame) != recv_frame.checksum)
        return -EIO;

    if (copy_to_user(buf, &recv_frame, FRAME_LEN))
        return -EFAULT;

    frame_ready = false;
    return FRAME_LEN;
}

static const struct file_operations rx_fops = {
    .owner   = THIS_MODULE,
    .open    = rx_open,
    .release = rx_release,
    .read    = rx_read,
};

static int __init speed_rx_init(void)
{
    major = register_chrdev(0, DEVICE_NAME, &rx_fops);
    if (major < 0) {
        pr_err("Failed to register %s\n", DEVICE_NAME);
        return major;
    }

    gpiod_data_rx      = gpiod_get(NULL, "data_tx", GPIOD_IN);
    gpiod_clk_tx       = gpiod_get(NULL, "clk_tx", GPIOD_IN);
    gpiod_data_tx_back = gpiod_get(NULL, "data_rx_out", GPIOD_OUT_LOW);
    gpiod_clk_tx_back  = gpiod_get(NULL, "clk_rx_out", GPIOD_OUT_LOW);
    if (IS_ERR(gpiod_data_rx) || IS_ERR(gpiod_clk_tx) ||
        IS_ERR(gpiod_data_tx_back) || IS_ERR(gpiod_clk_tx_back)) {
        pr_err("Failed to get GPIOs\n");
        unregister_chrdev(major, DEVICE_NAME);
        return -ENODEV;
    }

    clk_irq = gpiod_to_irq(gpiod_clk_tx);
    request_irq(clk_irq, clk_isr, IRQF_TRIGGER_RISING, DEVICE_NAME, NULL);

    pr_info("speed_rx driver loaded, major=%d\n", major);
    return 0;
}

static void __exit speed_rx_exit(void)
{
    cancel_delayed_work_sync(&decel_work);
    free_irq(clk_irq, NULL);
    gpiod_put(gpiod_clk_tx_back);
    gpiod_put(gpiod_data_tx_back);
    gpiod_put(gpiod_clk_tx);
    gpiod_put(gpiod_data_rx);
    unregister_chrdev(major, DEVICE_NAME);
    pr_info("speed_rx driver unloaded\n");
}

module_init(speed_rx_init);
module_exit(speed_rx_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("작성자");
MODULE_DESCRIPTION("Speed RX driver with deceleration for autonomous speed control");
