#include <kyvernitis/lib/drive.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

struct DiffDriveOdometryConfig {
	float wheel_separation, left_wheel_radius, right_wheel_radius;
	int velocity_rolling_window_size;
};

struct DiffDriveOdometry {
	const struct DiffDriveOdometryConfig config;
	int64_t last_command_timestamp;
	float x, y, heading, left_wheel_old_pos, right_wheel_old_pos, linear, angular;
	struct FloatRollingMeanAccumulator *linear_accumulator, *angular_accumulator;
};

int diffdrive_odometry_update_from_velocity(struct DiffDriveOdometry *odom, float left_vel,
					    float right_vel, const int64_t time);

struct DiffDrive {
	const struct DiffDriveConfig config;
	int64_t previous_update_timestamp;
	const struct DiffDriveOdometryConfig odometry_config;
	struct DiffDriveOdometry *odometry;
	int (*feedback_callback)(float *feedback_buffer, int buffer_len, int wheels_per_side);
	int (*velocity_callback)(const float *velocity_buffer, int buffer_len, int wheels_per_side);
};

struct FloatRollingMeanAccumulator* float_rolling_mean_accumulator_init(int rolling_window_size);
struct DiffDriveOdometry* diffdrive_odometry_init(struct DiffDriveOdometryConfig config);
struct DiffDrive *diffdrive_init(struct DiffDriveConfig *config,
		     int (*feedback_callback)(float *feedback_buffer, int buffer_len,
					      int wheels_per_side),
		     int (*velocity_callback)(const float *velocity_buffer, int buffer_len,
					      int wheels_per_side))
{
	struct DiffDriveOdometryConfig odom_config = {.wheel_separation = config->wheel_separation,
						      .left_wheel_radius = config->wheel_radius,
						      .right_wheel_radius = config->wheel_radius,
						      .velocity_rolling_window_size = 10};
	struct DiffDriveOdometry* odom = diffdrive_odometry_init(odom_config);
	struct DiffDrive drive = {.previous_update_timestamp = 0,
				  .odometry = odom,
				  .feedback_callback = feedback_callback,
				  .velocity_callback = velocity_callback};
	struct DiffDrive *heap_drive = (struct DiffDrive *)malloc(sizeof(drive));
	memcpy(heap_drive, &drive, sizeof(*heap_drive));
	memcpy((void *)&heap_drive->config, config, sizeof(heap_drive->config));
	return (void *)heap_drive;
}

struct DiffDriveStatus diffdrive_status(struct DiffDrive *drive) {
	struct DiffDriveStatus status = {.x = drive->odometry->x,
					 .y = drive->odometry->y,
					 .heading = drive->odometry->heading,
					 .linear = drive->odometry->linear,
					 .angular = drive->odometry->angular};
	return status;
}
void diffdrive_update(struct DiffDrive *drive, struct DiffDriveTwist command,
		      int64_t time_taken_by_last_update_seconds)
{
	// Get time since last update
	if (k_uptime_delta(&drive->previous_update_timestamp) >
	    drive->config.command_timeout_seconds * 1000) {
		command.linear_x = 0.0;
		command.angular_z = 0.0;
	}
	const float wheel_separation =
		drive->config.wheel_separation_multiplier * drive->config.wheel_separation;
	const float left_wheel_radius =
		drive->config.left_wheel_radius_multiplier * drive->config.wheel_radius;
	const float right_wheel_radius =
		drive->config.right_wheel_radius_multiplier * drive->config.wheel_radius;

	float linear_command = command.linear_x;
	float angular_command = command.angular_z;
	float left_feedback_mean = 0.0;
	float right_feedback_mean = 0.0;

	const int feedback_buffer_size = drive->config.wheels_per_side * 2;
	// First N elements => Left
	// Next N elements =>  Right
	float *feedback = malloc(sizeof(float) * feedback_buffer_size);
	if (drive->feedback_callback(feedback, feedback_buffer_size,
				     drive->config.wheels_per_side)) {
		// ERROR: Something went wrong while getting feedback
	}
	for (int i = 0; i < drive->config.wheels_per_side; i++) {
		const float left_feedback = feedback[i];
		const float right_feedback = feedback[drive->config.wheels_per_side + i];

		if (isnan(left_feedback) || isnan(right_feedback)) {
			// ERROR: One of the wheels gives invalid feedback
		}

		left_feedback_mean += left_feedback;
		right_feedback_mean += right_feedback;
	}
	free(feedback);
	left_feedback_mean /= drive->config.wheels_per_side;
	right_feedback_mean /= drive->config.wheels_per_side;

	if (drive->config.update_type | POSITION_FEEDBACK) {
		// TODO: odometry update from position
	} else {
		diffdrive_odometry_update_from_velocity(drive->odometry,
							left_feedback_mean * left_wheel_radius *
								time_taken_by_last_update_seconds,
							right_feedback_mean * right_wheel_radius *
								time_taken_by_last_update_seconds,
							drive->previous_update_timestamp);
	}

	const float velocity_left =
		(linear_command - angular_command * wheel_separation / 2.0) / left_wheel_radius;
	const float velocity_right =
		(linear_command + angular_command * wheel_separation / 2.0) / right_wheel_radius;

	float *velocity_buffer = (float *)malloc(sizeof(float) * feedback_buffer_size);
	for (int i = 0; i < drive->config.wheels_per_side; i++) {
		velocity_buffer[i] = velocity_left;
		velocity_buffer[drive->config.wheels_per_side + i] = velocity_right;
	}
	if (drive->velocity_callback(velocity_buffer, feedback_buffer_size,
				     drive->config.wheels_per_side)) {
		// ERROR: Something went wrong writing the velocities
	}
	free(velocity_buffer);
}

struct FloatRollingMeanAccumulator {
	float *buffer;
	const int buffer_len;
	int next_insert;
	float sum;
	int buffer_filled;
};

struct FloatRollingMeanAccumulator* float_rolling_mean_accumulator_init(int rolling_window_size)
{
	float *buffer = calloc(rolling_window_size, sizeof(float));
	struct FloatRollingMeanAccumulator frma = {.buffer = buffer,
						   .buffer_len = rolling_window_size,
						   .buffer_filled = false,
						   .next_insert = 0,
						   .sum = 0.0f};
	struct FloatRollingMeanAccumulator *heap_frma = malloc(sizeof(frma));
	memcpy(heap_frma, &frma, sizeof(*heap_frma));
	return heap_frma;
}

void float_rolling_mean_accumulator_accumulate(struct FloatRollingMeanAccumulator *frma, float val)
{
	frma->sum -= frma->buffer[frma->next_insert];
	frma->sum += val;
	frma->buffer[frma->next_insert] = val;
	frma->next_insert++;
	frma->buffer_filled |= frma->next_insert >= frma->buffer_len;
	frma->next_insert = frma->next_insert % frma->buffer_len;
}

float float_rolling_mean_accumulator_get_rolling_mean(
	const struct FloatRollingMeanAccumulator *frma)
{
	int valid_data_count =
		frma->buffer_filled * frma->buffer_len + !frma->buffer_filled * frma->next_insert;
	// CHECKME: assert(valid_data_count > 0);
	return frma->sum / valid_data_count;
}

void diffdrive_odometry_integrate_runge_kutta2(struct DiffDriveOdometry *odom, float linear,
					       float angular)
{
	const double direction = odom->heading + angular * 0.5;

	/// Runge-Kutta 2nd order integration:
	odom->x += linear * cos(direction);
	odom->y += linear * sin(direction);
	odom->heading += angular;
}

void diffdrive_odometry_integrate_exact(struct DiffDriveOdometry *odom, float linear, float angular)
{
	if (fabs(angular) < 1e-6) {
		diffdrive_odometry_integrate_runge_kutta2(odom, linear, angular);
	} else {
		/// Exact integration (should solve problems when angular is zero):
		const double heading_old = odom->heading;
		const double r = linear / angular;
		odom->heading += angular;
		odom->x += r * (sin(odom->heading) - sin(heading_old));
		odom->y += -r * (cos(odom->heading) - cos(heading_old));
	}
}

struct DiffDriveOdometry* diffdrive_odometry_init(struct DiffDriveOdometryConfig config)
{
	struct FloatRollingMeanAccumulator *linear_frma =
		float_rolling_mean_accumulator_init(config.velocity_rolling_window_size);
	struct FloatRollingMeanAccumulator *angular_frma =
		float_rolling_mean_accumulator_init(config.velocity_rolling_window_size);
	struct DiffDriveOdometry odom = {.last_command_timestamp = 0,
					 .x = 0.0,
					 .y = 0.0,
					 .heading = 0.0,
					 .left_wheel_old_pos = 0.0,
					 .right_wheel_old_pos = 0.0,
					 .linear = 0.0,
					 .angular = 0.0,
					 .linear_accumulator = linear_frma,
					 .angular_accumulator = angular_frma};
	struct DiffDriveOdometry *heap_odom = (struct DiffDriveOdometry *)malloc(sizeof(odom));
	memcpy(heap_odom, &odom, sizeof(*heap_odom));
	memcpy((void *)&heap_odom->config, &config, sizeof(heap_odom->config));
	return (void *)heap_odom;
}

int diffdrive_odometry_update_from_velocity(struct DiffDriveOdometry *odom, float left_vel,
					    float right_vel, const int64_t time)
{
	const float dt = time - odom->last_command_timestamp;
	const float linear = (left_vel + right_vel) * 0.5f;
	const float angular = (right_vel - left_vel) / odom->config.wheel_separation;

	diffdrive_odometry_integrate_exact(odom, linear, angular);
	odom->last_command_timestamp = time;

	float_rolling_mean_accumulator_accumulate(odom->linear_accumulator, linear / dt);
	float_rolling_mean_accumulator_accumulate(odom->angular_accumulator, angular / dt);

	odom->linear = float_rolling_mean_accumulator_get_rolling_mean(odom->linear_accumulator);
	odom->angular = float_rolling_mean_accumulator_get_rolling_mean(odom->angular_accumulator);
	return true;
}

int diffdrive_odometry_update(struct DiffDriveOdometry *odom, float left_pos, float right_pos,
			      const int64_t time)
{
	const float dt = time - odom->last_command_timestamp;
	if (dt < 0.0001) {
		return false;
	}

	const float left_wheel_cur_pos = left_pos * odom->config.left_wheel_radius;
	const float right_wheel_cur_pos = right_pos * odom->config.right_wheel_radius;

	const float left_wheel_est_vel = left_wheel_cur_pos - odom->left_wheel_old_pos;
	const float right_wheel_est_vel = right_wheel_cur_pos - odom->right_wheel_old_pos;

	odom->left_wheel_old_pos = left_wheel_cur_pos;
	odom->right_wheel_old_pos = right_wheel_cur_pos;

	diffdrive_odometry_update_from_velocity(odom, left_wheel_est_vel, right_wheel_est_vel,
						time);
	return true;
}

void diffdrive_odometry_reset_accumulators(struct DiffDriveOdometry *odom)
{
	free(odom->linear_accumulator);
	free(odom->angular_accumulator);

	odom->linear_accumulator =
		float_rolling_mean_accumulator_init(odom->config.velocity_rolling_window_size);
	odom->angular_accumulator =
		float_rolling_mean_accumulator_init(odom->config.velocity_rolling_window_size);
}
