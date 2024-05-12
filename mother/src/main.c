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
#define UART_MSG_SIZE        (sizeof(struct mother_msg) + 2)
#define TX_THREAD_STACK_SIZE 512
#define TX_THREAD_PRIORITY   2

#define PWM_MOTOR_SETUP(pwm_dev_id)                                                                \
	{.dev_spec = PWM_DT_SPEC_GET(pwm_dev_id),                                                  \
	 .min_pulse = DT_PROP(pwm_dev_id, min_pulse),                                              \
	 .max_pulse = DT_PROP(pwm_dev_id, max_pulse)},

/* queue to store uart messages */
K_MSGQ_DEFINE(uart_msgq, sizeof(struct mother_msg), 10, 1);

/* Define stack size for uart to can thread */
K_THREAD_STACK_DEFINE(uart_can_thread_stack, TX_THREAD_STACK_SIZE);

/* DT spec for uart */
static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(mother_uart));

/* DT spec for pwm motors */
struct pwm_motor motor[20] = {DT_FOREACH_CHILD(DT_PATH(pwmmotors), PWM_MOTOR_SETUP)};

/* DT spec for LED */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

/* Timeout message for diff drive */
struct DiffDriveTwist timeout_cmd = {
	.angular_z = 0,
	.linear_x = 0,
};

/* Velocity and PWM ranges */
float vel_range[] = {-3,3};
uint32_t pwm_range[] = {1100, 1900};


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
		struct mother_msg msg;

		if (c == 0x00 && rx_buf_pos > 0) {
			// uart_fifo_read(uart_dev, &c, 1);
			/* terminate the message with 0x00 */
			rx_buf[rx_buf_pos] = 0;
			
			if (rx_buf_pos != (sizeof(struct mother_msg) - 1)) {
				continue;
			}
			// Add deserialization guard
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
bool valid_crc(struct mother_msg *msg)
{
	uint32_t rcd_crc = msg->crc;
	uint32_t comp_crc =
		crc32_ieee((uint8_t *)&msg, sizeof(struct mother_msg) - sizeof(uint32_t));

	if (rcd_crc != comp_crc) {
		return false;
	}

	return true;
}

/*
 * Position feedback callback function
 */ 

int feedback_callback(float *feedback_buffer, int buffer_len, int wheels_per_side)
{
  if (buffer_len < wheels_per_side*2) return 1;
	for (int i = 0; i < wheels_per_side; i++) {
	  feedback_buffer[i] = 1.0f;
	  feedback_buffer[wheels_per_side + i] = 1.0f;
	}
	return 0;
}

/*
 * Velocity callback function
 */

int velocity_callback(const float *velocity_buffer, int buffer_len, int wheels_per_side)
{

	if (buffer_len < wheels_per_side*2) return 1;
	
	for(int i = 0; i < wheels_per_side; i++) {
		if(pwm_motor_write(&(motor[i]), velocity_pwm_interpolation(*(velocity_buffer + i), vel_range, pwm_range))) {
			LOG_ERR("Drive: Unable to write pwm pulse to Left : %d", i);
			return 1;
		}
		if(pwm_motor_write(&(motor[i]), velocity_pwm_interpolation(*(velocity_buffer + wheels_per_side + i), vel_range, pwm_range))) {
			LOG_ERR("Drive: Unable to write pwm pulse to Right : %d", i);
			return 1;
		}

	}
	return 0;
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
	struct mother_msg tx_msg;

	while (1) {
		/* Some data to send over uart */

		tx_msg.crc = crc32_ieee((uint8_t *)&tx_msg, sizeof(struct can_frame));

		serialize(tx_buf, (uint8_t *)&tx_msg, sizeof(struct mother_msg));

		send_to_uart(tx_buf, UART_MSG_SIZE);

		k_sleep(K_SECONDS(1));
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

	for (size_t i = 0U; i < ARRAY_SIZE(motor); i++) {
		if (!pwm_is_ready_dt(&(motor[i].dev_spec))) {
			printk("PWM: Motor %s is not ready\n", motor[i].dev_spec.dev->name);
			return 0;
		}
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

	struct DiffDriveConfig drive_config = {
				.wheel_separation = 0.4f,
				.wheel_separation_multiplier = 1,
				.wheel_radius = 0.15f,
				.wheels_per_side = 2,
				.command_timeout_seconds = 2,
				.left_wheel_radius_multiplier = 1,
				.right_wheel_radius_multiplier = 1,
				.update_type = POSITION_FEEDBACK,
	};

	struct DiffDrive* drive = diffdrive_init(&drive_config, feedback_callback, velocity_callback);

	int64_t time_last_drive_update = 0;
	int64_t drive_timestamp = 0;

	LOG_INF("Initialization completed successfully");

	struct mother_msg msg;

	while (true) {
		if (k_msgq_get(&uart_msgq, &msg, K_SECONDS(2))) {
			LOG_INF("Message Receive Timeout!!");
			diffdrive_update(drive, timeout_cmd, drive_timestamp);

			continue;
		}

		if (!valid_crc(&msg)) {
			continue;
		}


		switch (msg.type) {
		case T_MOTHER_CMD_LA:
			drive_timestamp = k_uptime_get();
			diffdrive_update(drive, msg.cmd.drive_cmd, time_last_drive_update);
			time_last_drive_update = k_uptime_get() - drive_timestamp; 
		}

		gpio_pin_toggle_dt(&led);
	}
}
