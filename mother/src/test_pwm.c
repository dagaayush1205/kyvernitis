/*
 * Test file for checking pwm pins
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#include <kyvernitis.h>


struct pwm_motor roboclaw[20] = {
	{
		.dev_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor_1)),
		.min_pulse = DT_PROP(DT_NODELABEL(pwm_motor_1), min_pulse),
		.max_pulse = DT_PROP(DT_NODELABEL(pwm_motor_1), max_pulse)
	},
	{
		.dev_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor_2)),
		.min_pulse = DT_PROP(DT_NODELABEL(pwm_motor_2), min_pulse),
		.max_pulse = DT_PROP(DT_NODELABEL(pwm_motor_2), max_pulse)
	},
	{
		.dev_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor_3)),
		.min_pulse = DT_PROP(DT_NODELABEL(pwm_motor_3), min_pulse),
		.max_pulse = DT_PROP(DT_NODELABEL(pwm_motor_3), max_pulse)
	},
	{
		.dev_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor_4)),
		.min_pulse = DT_PROP(DT_NODELABEL(pwm_motor_4), min_pulse),
		.max_pulse = DT_PROP(DT_NODELABEL(pwm_motor_4), max_pulse)
	},
	{
		.dev_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor_5)),
		.min_pulse = DT_PROP(DT_NODELABEL(pwm_motor_5), min_pulse),
		.max_pulse = DT_PROP(DT_NODELABEL(pwm_motor_5), max_pulse)
	},
	{
		.dev_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor_6)),
		.min_pulse = DT_PROP(DT_NODELABEL(pwm_motor_6), min_pulse),
		.max_pulse = DT_PROP(DT_NODELABEL(pwm_motor_6), max_pulse)
	},
	{
		.dev_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor_7)),
		.min_pulse = DT_PROP(DT_NODELABEL(pwm_motor_7), min_pulse),
		.max_pulse = DT_PROP(DT_NODELABEL(pwm_motor_7), max_pulse)
	},
	{
		.dev_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor_8)),
		.min_pulse = DT_PROP(DT_NODELABEL(pwm_motor_8), min_pulse),
		.max_pulse = DT_PROP(DT_NODELABEL(pwm_motor_8), max_pulse)
	},
	{
		.dev_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor_9)),
		.min_pulse = DT_PROP(DT_NODELABEL(pwm_motor_9), min_pulse),
		.max_pulse = DT_PROP(DT_NODELABEL(pwm_motor_9), max_pulse)
	},
	{
		.dev_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor_10)),
		.min_pulse = DT_PROP(DT_NODELABEL(pwm_motor_10), min_pulse),
		.max_pulse = DT_PROP(DT_NODELABEL(pwm_motor_10), max_pulse)
	},
	{
		.dev_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor_11)),
		.min_pulse = DT_PROP(DT_NODELABEL(pwm_motor_11), min_pulse),
		.max_pulse = DT_PROP(DT_NODELABEL(pwm_motor_11), max_pulse)
	},
	{
		.dev_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor_12)),
		.min_pulse = DT_PROP(DT_NODELABEL(pwm_motor_12), min_pulse),
		.max_pulse = DT_PROP(DT_NODELABEL(pwm_motor_12), max_pulse)
	},
	{
		.dev_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor_13)),
		.min_pulse = DT_PROP(DT_NODELABEL(pwm_motor_13), min_pulse),
		.max_pulse = DT_PROP(DT_NODELABEL(pwm_motor_13), max_pulse)
	},
	{
		.dev_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor_14)),
		.min_pulse = DT_PROP(DT_NODELABEL(pwm_motor_14), min_pulse),
		.max_pulse = DT_PROP(DT_NODELABEL(pwm_motor_14), max_pulse)
	},
	{
		.dev_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor_15)),
		.min_pulse = DT_PROP(DT_NODELABEL(pwm_motor_15), min_pulse),
		.max_pulse = DT_PROP(DT_NODELABEL(pwm_motor_15), max_pulse)
	},
	{
		.dev_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor_16)),
		.min_pulse = DT_PROP(DT_NODELABEL(pwm_motor_16), min_pulse),
		.max_pulse = DT_PROP(DT_NODELABEL(pwm_motor_16), max_pulse)
	},
	{
		.dev_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor_17)),
		.min_pulse = DT_PROP(DT_NODELABEL(pwm_motor_17), min_pulse),
		.max_pulse = DT_PROP(DT_NODELABEL(pwm_motor_17), max_pulse)
	},
	{
		.dev_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor_18)),
		.min_pulse = DT_PROP(DT_NODELABEL(pwm_motor_18), min_pulse),
		.max_pulse = DT_PROP(DT_NODELABEL(pwm_motor_18), max_pulse)
	},
	{
		.dev_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor_19)),
		.min_pulse = DT_PROP(DT_NODELABEL(pwm_motor_19), min_pulse),
		.max_pulse = DT_PROP(DT_NODELABEL(pwm_motor_19), max_pulse)
	},
	{
		.dev_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_motor_20)),
		.min_pulse = DT_PROP(DT_NODELABEL(pwm_motor_20), min_pulse),
		.max_pulse = DT_PROP(DT_NODELABEL(pwm_motor_20), max_pulse)
	},
};

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int main()
{
	printk("Testing all PWM pins\n\n");

	if (!gpio_is_ready_dt(&led)){
		printk("Error: Led not ready\n");
	}

	for (size_t i = 0U; i < ARRAY_SIZE(roboclaw); i++) {
		if (!pwm_is_ready_dt(&(roboclaw[i].dev_spec))) {
			printk("PWM: Roboclaw %s is not ready\n", roboclaw[i].dev_spec.dev->name);
			return 0;
		}
	}

	if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE) < 0)
	{
		printk("Error: Led not configured\n");
		return 0;
	}

	printk("Successfully initialized\n");

	for(size_t i = 0U; i < ARRAY_SIZE(roboclaw); i++) {
		for(uint32_t pulse = 1120000; pulse < 1192000; pulse += 100000)
		{
			if(pwm_motor_write(&roboclaw[i], pulse)) {
				printk("Unable to write pwm pulse to PWM Motor : %d", i);
				return 0;
			}
		}
	}

}
