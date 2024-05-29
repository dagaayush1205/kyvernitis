#include <kyvernitis/lib/kyvernitis.h>
#include <math.h>

// Wrapper around pwm_set_pulse_dt to ensure that pulse_width
// remains under max-min range
int pwm_motor_write(const struct pwm_motor *motor, uint32_t pulse_width)
{
	// wrapper around pwm_set_pulse_dt to ensure that pulse_width 
	// remains under max-min range
	if (pulse_width <= motor->min_pulse)
		pulse_width = motor->min_pulse;
	if (pulse_width >= motor->max_pulse)
		pulse_width = motor->max_pulse;
	
	int ret = pwm_set_pulse_dt(&(motor->dev_spec), pulse_width);
	return ret;
}

/*
 * Wrapper around gpio_pin_set_dt to abstract away stepper motor commands
 * Moves the motor by one step in the commanded direction
 *
 * Ret: 0 on Success.
 * Ret: other then 0 on Failure
 */
int stepper_motor_write(const struct stepper_motor *motor, uint8_t cmd)
{
	int ret = 0;
	switch (cmd) {
	
	case STEPPER_MOTOR_FORWARD:
		// set dir
		ret += gpio_pin_set_dt(&(motor->dir), 1);

		// One step
		ret += gpio_pin_set_dt(&(motor->step), 1);
		k_sleep(K_USEC(500));
		ret += gpio_pin_set_dt(&(motor->step), 0);
		k_sleep(K_USEC(500));
		
		break;

	case STEPPER_MOTOR_BACKWARD:
		// set dir
		ret += gpio_pin_set_dt(&(motor->dir), 0);

		// One step
		ret += gpio_pin_set_dt(&(motor->step), 1);
		k_sleep(K_USEC(500));
		ret += gpio_pin_set_dt(&(motor->step), 0);
		k_sleep(K_USEC(500));
		
		break;
	}

	return ret;
}


/*
 * Wrapper around gpio_pin_set_dt to abstract away motor commands
 *
 * Ret: 0 on Success.
 * Ret: 1 on Failure
 */ 
int dc_motor_write(const struct dc_motor *motor, uint8_t motor_cmd)
{
	switch (motor_cmd){
	
	case DC_MOTOR_FORWARD:
		if (gpio_pin_set_dt(&(motor->input_1), 1))
			return 1;
		if (gpio_pin_set_dt(&(motor->input_2), 0))
			return 1;
		break;

	case DC_MOTOR_BACKWARD:
		if (gpio_pin_set_dt(&(motor->input_1), 0))
			return 1;
		if (gpio_pin_set_dt(&(motor->input_2), 1))
			return 1;
		break;

	case DC_MOTOR_STOP:
		if (gpio_pin_set_dt(&(motor->input_1), 0))
			return 1;
		if (gpio_pin_set_dt(&(motor->input_2), 0))
			return 1;
		break;
	}

	return 0;
}

/*
 * Writes motor command to dc motors with limit switches
 */
int dc_motor_write_lim(const struct dc_motor *motor, uint8_t motor_cmd, const struct gpio_dt_spec *lim)
{
	int ret = 1;
	
	if(!gpio_pin_get_dt(lim)) {
		ret = dc_motor_write(motor, motor_cmd);
	}
	else {
		ret = dc_motor_write(motor, DC_MOTOR_STOP);
	}
	

	return ret;
}


float MQ2_readings(int adc_reading) {
	int ppm = adc_reading * (10000 - 300) / 4096;
	return ppm;
}

float MQ7_readings(int adc_reading) {
	int ppm = adc_reading * (2000 - 20) / 4096;
	return ppm;
}

float MQ136_readings(int adc_reading) {
	float ppm = adc_reading * (100.0 - 1.0) / 4096.0;
	return ppm;
}

float MQ137_readings(int adc_reading) {
	float ppm = adc_reading * (500.0 - 5.0) / 4096.0;
	return ppm;
}

void get_encoder_ticks(int64_t *ticks, const struct device *const dev, struct sensor_value *val) {
	int rc;
	rc = sensor_sample_fetch(dev);
	if (rc != 0) {
		printk("Failed to fetch sample (%d)\n", rc);
	}

	rc = sensor_channel_get(dev, SENSOR_CHAN_ALL, val);
	if (rc != 0) {
		printk("Failed to get data (%d)\n", rc);
	}
	
	*ticks = val->val1 * pow(10,6) + val->val2;
}
