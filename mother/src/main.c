/*
 * Source file for R25
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/crc.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <app_version.h>
#include <math.h>
#include <string.h>
#include <canscribe/lib/canscribe.h>
#include <kyvernitis/lib/kyvernitis.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/* msg size in relation to cobs serialization */
#define UART_MSG_SIZE        (sizeof(struct mother_cmd_msg) + 2)
#define TX_THREAD_STACK_SIZE 512
#define TX_THREAD_PRIORITY   2

/* queue to store uart messages */
K_MSGQ_DEFINE(uart_msgq, sizeof(struct mother_cmd_msg), 10, 1);

/* Define stack size for uart to can thread */
K_THREAD_STACK_DEFINE(uart_can_thread_stack, TX_THREAD_STACK_SIZE);

/* DT spec for uart */
static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(mother_uart));

/* DT spec for LED */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static uint8_t rx_buf[100];
static int rx_buf_pos;
static uint8_t tx_buf[UART_MSG_SIZE];

void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		return;
	}

	/* read until FIFO empty */
	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		struct mother_cmd_msg msg;

		if (c == 0 && rx_buf_pos > 0) {
			// uart_fifo_read(uart_dev, &c, 1);
			/* terminate the message with 0x00 */
			rx_buf[rx_buf_pos] = 0;

			deserialize(rx_buf, (uint8_t *)&msg, sizeof(msg));

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &msg, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
}

/*
 * Sends raw bytes to uart_dev
 */

void send_to_uart(uint8_t *buf, uint8_t len)
{
	for (int i = 0; i < len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}

/*
 * Validates against crc
 */
bool valid_crc(struct mother_cmd_msg *msg)
{
	uint32_t rcd_crc = msg->crc;
	uint32_t comp_crc =
		crc32_ieee((uint8_t *)&msg, sizeof(struct mother_cmd_msg) - sizeof(uint32_t));

	if (rcd_crc != comp_crc) {
		return false;
	}

	return true;
}

/*
 * This thread sends uart messages
 */

struct k_thread tx_thread_data;

void uart_can_thread(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	int err;
	ARG_UNUSED(err);
	struct mother_cmd_msg tx_msg;

	while (1) {
		/* Some data to send over uart */

		tx_msg.crc = crc32_ieee((uint8_t *)&tx_msg, sizeof(struct can_frame));

		serialize(tx_buf, (uint8_t *)&tx_msg, sizeof(struct mother_cmd_msg));

		send_to_uart(tx_buf, UART_MSG_SIZE);
	}
}

int main()
{
	printk("\nMother: v%s\n\n", APP_VERSION_STRING);

	int err;

	/* Device ready checks */

	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART device not found!");
		return 0;
	}

	if (!gpio_is_ready_dt(&led)) {
		LOG_ERR("Error: Led not ready.");
		return 0;
	}

	/* Configure devices */

	err = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

	if (err < 0) {
		if (err == -ENOTSUP) {
			LOG_ERR("Interrupt-driven UART API support not enabled");
		} else if (err == -ENOSYS) {
			LOG_ERR("UART device does not support interrupt-driven API");
		} else {
			LOG_ERR("Error setting UART callback: %d", err);
		}
		return 0;
	}
	uart_irq_rx_enable(uart_dev);

	if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE) < 0) {
		LOG_ERR("Error: Led not configured");
		return 0;
	}

	LOG_INF("Initialization completed successfully");

	struct mother_cmd_msg msg;

	while (true) {
		if (k_msgq_get(&uart_msgq, &msg, K_FOREVER)) {
			if (!valid_crc(&msg)) {
				continue;
			}

			/* Do something with msg */

			gpio_pin_toggle_dt(&led);
		}
	}
}
