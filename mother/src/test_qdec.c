/*
 * Copyright (c) 2022 Valerio Setti <valerio.setti@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

#include <kyvernitis/lib/kyvernitis.h>
#include <zephyr/drivers/gpio.h>
#define PWM_MOTOR_SETUP(pwm_dev_id)                                                                \
	{.dev_spec = PWM_DT_SPEC_GET(pwm_dev_id),                                                  \
	 .min_pulse = DT_PROP(pwm_dev_id, min_pulse),                                              \
	 .max_pulse = DT_PROP(pwm_dev_id, max_pulse)},
struct pwm_motor roboclaw[2] = {DT_FOREACH_CHILD(DT_PATH(pwmmotors), PWM_MOTOR_SETUP)};
static const struct gpio_dt_spec led_1 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

#define QUAD_ENC_EMUL_ENABLED \
	DT_NODE_EXISTS(DT_ALIAS(qenca)) && DT_NODE_EXISTS(DT_ALIAS(qencb))

#if QUAD_ENC_EMUL_ENABLED


#define QUAD_ENC_EMUL_PERIOD 100


static const struct gpio_dt_spec phase_a =
			GPIO_DT_SPEC_GET(DT_ALIAS(qenca), gpios);
static const struct gpio_dt_spec phase_b =
			GPIO_DT_SPEC_GET(DT_ALIAS(qencb), gpios);
static bool toggle_a;

void qenc_emulate_work_handler(struct k_work *work)
{
	toggle_a = !toggle_a;
	if (toggle_a) {
		gpio_pin_toggle_dt(&phase_a);
	} else {
		gpio_pin_toggle_dt(&phase_b);
	}
}

static K_WORK_DEFINE(qenc_emulate_work, qenc_emulate_work_handler);

static void qenc_emulate_timer_handler(struct k_timer *dummy)
{
	k_work_submit(&qenc_emulate_work);
}

static K_TIMER_DEFINE(qenc_emulate_timer, qenc_emulate_timer_handler, NULL);

static void qenc_emulate_init(void)
{
	printk("Quadrature encoder emulator enabled with %u ms period\n",
		QUAD_ENC_EMUL_PERIOD);

	if (!gpio_is_ready_dt(&phase_a)) {
		printk("%s: device not ready.", phase_a.port->name);
		return;
	}
	gpio_pin_configure_dt(&phase_a, GPIO_OUTPUT);

	if (!gpio_is_ready_dt(&phase_b)) {
		printk("%s: device not ready.", phase_b.port->name);
		return;
	}
	gpio_pin_configure_dt(&phase_b, GPIO_OUTPUT);

	k_timer_start(&qenc_emulate_timer, K_MSEC(QUAD_ENC_EMUL_PERIOD / 2),
			K_MSEC(QUAD_ENC_EMUL_PERIOD / 2));
}

#else

static void qenc_emulate_init(void) { };

#endif /* QUAD_ENC_EMUL_ENABLED */

int get_ticks(int64_t *ticks, const struct device *const dev, struct sensor_value *val) {
	int rc;
	rc = sensor_sample_fetch(dev);
	if (rc != 0) {
		printk("Failed to fetch sample (%d)\n", rc);
		return 0;
	}

	rc = sensor_channel_get(dev, SENSOR_CHAN_ALL, val);
	if (rc != 0) {
		printk("Failed to get data (%d)\n", rc);
		return 0;
	}
	
	*ticks = val->val1 * pow(10,6) + val->val2;
}

int main(void)
{
	struct sensor_value val;
	const struct device *const en_fr = DEVICE_DT_GET(DT_ALIAS(en_fr));
	const struct device *const en_fl = DEVICE_DT_GET(DT_ALIAS(en_fl));


	if (!device_is_ready(en_fr)) {
		printk("Qdec Front-Right device is not ready\n");
		return 0;
	}
	if (!device_is_ready(en_fl)) {
		printk("Qdec Front-Left device is not ready\n");
		return 0;
	}

	if (!gpio_is_ready_dt(&led_1)) {
		printk("Error: Led not ready.");
		return 0;
	}
	if (gpio_pin_configure_dt(&led_1, GPIO_OUTPUT_ACTIVE) < 0) {
		printk("Error: Led not configured");
		return 0;
	}
	printk("Quadrature decoder sensor test\n");

	for (size_t i = 0U; i < ARRAY_SIZE(roboclaw); i++) {
		if (!pwm_is_ready_dt(&(roboclaw[i].dev_spec))) {
			printk("PWM: Roboclaw %s is not ready\n", roboclaw[i].dev_spec.dev->name);
			return 0;
		}
	}
	qenc_emulate_init();
	int64_t ticks;
	while (true) {
		get_ticks(&ticks, en_fr, &val);	
		printk("Front-Right: %lld \n", ticks);
		get_ticks(&ticks, en_fl, &val);	
		printk("Front-Left: %lld \n", ticks);


		int pulse = 1900000;
		if(pwm_motor_write(&roboclaw[0], pulse)) {
			printk("Unable to write pwm pulse to PWM Motor : %d", 0);
			return 0;
		}

		if(pwm_motor_write(&roboclaw[1], pulse)) {
			printk("Unable to write pwm pulse to PWM Motor : %d", 1);
			return 0;
		}
		gpio_pin_toggle_dt(&led_1);
		k_msleep(1000);
	}
	return 0;
}
