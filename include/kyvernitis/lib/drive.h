#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <app_version.h>
#include <stdint.h>

struct DiffDriveStatus {
	float x, y, heading, linear, angular;
};
struct DiffDriveTwist {
	float linear_x;
	float angular_z;
};

enum DiffDriveUpdateType {
	POSITION_FEEDBACK,
	VELOCITY_FEEDBACK
};

struct DiffDriveConfig {
	float wheel_separation;
	int wheels_per_side; // actually 2, but both are controlled by 1 signal
	float wheel_radius;

	int64_t command_timeout_seconds;
	float wheel_separation_multiplier;
	float left_wheel_radius_multiplier;
	float right_wheel_radius_multiplier;
	uint8_t update_type;
};

struct DiffDrive;
struct DiffDrive *diffdrive_init(struct DiffDriveConfig *config,
		     int (*feedback_callback)(float *feedback_buffer, int buffer_len,
					      int wheels_per_side),
		     int (*velocity_callback)(const float *velocity_buffer, int buffer_len,
					      int wheels_per_side));
void diffdrive_update(struct DiffDrive *drive, struct DiffDriveTwist command,
		      int64_t time_taken_by_last_update_seconds);
struct DiffDriveStatus diffdrive_status(struct DiffDrive *drive);
