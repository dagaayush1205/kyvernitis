/*
 * Kyervnitis helper macros and function declarations
 */
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>

#include <kyvernitis/lib/drive.h>

#define MAX_ROBOCLAWS   2
#define MAX_SABERTOOTHS 2
#define MAX_SERVOS      5

#define PWM_MOTOR_STOP      1520000
#define SERVO_DEFAULT_STATE 1520000

#define STEPPER_MOTOR_FORWARD  1
#define STEPPER_MOTOR_BACKWARD 2

#define DC_MOTOR_FORWARD  1
#define DC_MOTOR_BACKWARD 2
#define DC_MOTOR_STOP     0

enum MotherMsgType {
	T_MOTHER_CMD_DRIVE,
	T_MOTHER_CMD_ARM,
	T_MOTHER_CMD_LA,
	T_MOTHER_STATUS,
	T_MOTHER_ERROR,
	T_MOTHER_INFO
};

struct pwm_motor {
	const struct pwm_dt_spec dev_spec;
	const uint32_t min_pulse;
	const uint32_t max_pulse;
};

struct stepper_motor {
	const struct gpio_dt_spec dir;
	const struct gpio_dt_spec step;
};

struct dc_motor {
	const struct gpio_dt_spec input_1;
	const struct gpio_dt_spec input_2;
};

struct mother_cmd_msg {
	struct DiffDriveTwist drive_cmd;
	float arm_joint[3];
	uint8_t adaptive_sus_cmd[4];
};

struct mother_status_msg {
	struct DiffDriveStatus odom;
	float arm_joint_status[3];
	uint64_t timestamp;
};

struct mother_msg {
	uint16_t type;
	struct mother_status_msg status;
	struct mother_cmd_msg cmd;
	char info[100];
	uint32_t crc;
};

typedef uint32_t servo_state_t;

// Wrapper around pwm_set_pulse_dt to ensure that pulse_width
// remains under max-min range
int pwm_motor_write(const struct pwm_motor *motor, uint32_t pulse_width);

int stepper_motor_write(const struct stepper_motor *motor, uint8_t cmd);

int dc_motor_write(const struct dc_motor *motor, uint8_t motor_cmd);

int dc_motor_write_lim(const struct dc_motor *motor, uint8_t motor_cmd,
		       const struct gpio_dt_spec *lim);

float MQ2_readings(int adc_reading);

float MQ7_readings(int adc_reading);

float MQ136_readings(int adc_reading);

float MQ137_readings(int adc_reading);

void get_encoder_ticks(int64_t *ticks, const struct device *const dev, struct sensor_value *val);
