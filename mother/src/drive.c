#include <math.h>
#include <stdbool.h>

struct DiffDriveOdometryConfig {
  int timestamp;
  float x,y,heading, wheel_separation, left_wheel_radius, right_wheel_radius;
  int velocity_rolling_window_size;
};
struct DiffDriveOdometry {
  const struct DiffDriveOdometryConfig config;
  int timestamp;
  float x, y, heading, left_wheel_old_pos, right_wheel_old_pos, linear, angular;
  struct FloatRollingMeanAccumulator *linear_accumulator, *angular_accumulator;
};
int diffdrive_odometry_update_from_velocity(struct DiffDriveOdometry* odom, float left_vel, float right_vel, const int time);
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
  int wheels_per_side;  // actually 2, but both are controlled by 1 signal
  float wheel_radius;

  float wheel_separation_multiplier;
  float left_wheel_radius_multiplier;
  float right_wheel_radius_multiplier;
  int update_type;
};
struct DiffDrive {
  const struct DiffDriveConfig config;
  int previous_update_timestamp;
  const struct DiffDriveOdometryConfig odometry_config;
  struct DiffDriveOdometry* odometry;
  int (*feedback_callback)(float* feedback_buffer, int buffer_len, int wheels_per_side);
  int (*velocity_callback)(const float* velocity_buffer, int buffer_len, int wheels_per_side);
};
void* diffdrive_init(struct DiffDriveConfig* config) {
  // TODO: Make a constructor here
  return (void*)0xDEADBEEF;
}
void diffdrive_update(struct DiffDrive* drive, struct DiffDriveTwist command) {
  // Get time since last update
  const int now_timestamp = 0xDEADBEEF; // TODO: Change this
  const float wheel_separation = drive->config.wheel_separation_multiplier * drive->config.wheel_separation;
  const float left_wheel_radius = drive->config.left_wheel_radius_multiplier * drive->config.wheel_radius;
  const float right_wheel_radius = drive->config.right_wheel_radius_multiplier * drive->config.wheel_radius;
  
  float linear_command = command.linear_x;
  float angular_command = command.angular_z;
  float left_feedback_mean = 0.0;
  float right_feedback_mean = 0.0;

  const int feedback_buffer_size = drive->config.wheels_per_side*2;
  // First N elements => Left
  // Next N elements =>  Right
  float* feedback = malloc(sizeof(float)*feedback_buffer_size); // FIXME: Replace with kmalloc
  if (drive->feedback_callback(
        feedback, feedback_buffer_size, drive->config.wheels_per_side)) {
    // ERROR: Something went wrong while getting feedback
  }
  for (int i = 0; i < drive->config.wheels_per_side; i++) {
    const float left_feedback = feedback[i];
    const float right_feedback = feedback[drive->config.wheels_per_side+i];

    if (isnan(left_feedback) || isnan(right_feedback)) {
      // ERROR: One of the wheels gives invalid feedback
    }

    left_feedback_mean += left_feedback;
    right_feedback_mean += right_feedback;
  }
  free(feedback); // FIXME: Replace with kfree
  left_feedback_mean /= drive->config.wheels_per_side;
  right_feedback_mean /= drive->config.wheels_per_side;

  if (drive->config.update_type | POSITION_FEEDBACK) {
    // TODO: odometry update from position
  } else {
    diffdrive_odometry_update_from_velocity(drive->odometry, left_feedback_mean * left_wheel_radius * 1.0f, // FIXME period.seconds()
                                            right_feedback_mean * right_wheel_radius * 1.0f, // FIXME period.second()
                                            now_timestamp);
  }

  const float velocity_left = (linear_command - angular_command * wheel_separation / 2.0) / left_wheel_radius;
  const float velocity_right = (linear_command + angular_command * wheel_separation / 2.0) / right_wheel_radius;

  float* velocity_buffer = (float*) malloc(sizeof(float)*feedback_buffer_size);
  for (int i = 0; i < drive->config.wheels_per_side; i++) {
    velocity_buffer[i] = velocity_left;
    velocity_buffer[drive->config.wheels_per_side+i] = velocity_right;
  }
  if (drive->velocity_callback(velocity_buffer, feedback_buffer_size, drive->config.wheels_per_side)) {
    // ERROR: Something went wrong writing the velocities
  }
  free(velocity_buffer); // FIXME: replace with kfree
}

struct FloatRollingMeanAccumulator {
  float* buffer;
  const int buffer_len;
  int next_insert;
  float sum;
  int buffer_filled;
  
};
void* float_rolling_mean_accumulator_init(int rolling_window_size) {
  float* buffer = 0x00; // calloc(rolling_window_size, sizeof(float)); FIXME
  struct FloatRollingMeanAccumulator frma = {.buffer = buffer, .buffer_len = rolling_window_size, .buffer_filled = false, .next_insert = 0, .sum = 0.0f};
  void* heap_frma = 0x00; // FIXME: malloc(sizeof(frma));
  // FIXME memcpy(heap_frma, frma);
  return heap_frma;
}
void float_rolling_mean_accumulator_accumulate(struct FloatRollingMeanAccumulator* frma, float val) {
  frma->sum -= frma->buffer[frma->next_insert];
  frma->sum += val;
  frma->buffer[frma->next_insert] = val;
  frma->next_insert++;
  frma->buffer_filled |= frma->next_insert >= frma->buffer_len;
  frma->next_insert = frma->next_insert % frma->buffer_len;
}
float float_rolling_mean_accumulator_get_rolling_mean(const struct FloatRollingMeanAccumulator* frma) {
  int valid_data_count = frma->buffer_filled * frma->buffer_len + !frma->buffer_filled * frma->next_insert;
  // CHECKME: assert(valid_data_count > 0);
  return frma->sum / valid_data_count;
}
void diffdrive_odometry_integrate_runge_kutta2(struct DiffDriveOdometry* odom, float linear, float angular)
{
  const double direction = odom->heading + angular * 0.5;

  /// Runge-Kutta 2nd order integration:
  odom->x += linear * cos(direction);
  odom->y += linear * sin(direction);
  odom->heading += angular;
}

void diffdrive_odometry_integrate_exact(struct DiffDriveOdometry* odom, float linear, float angular)
{
  if (fabs(angular) < 1e-6)
  {
    diffdrive_odometry_integrate_runge_kutta2(odom, linear, angular);
  }
  else
  {
    /// Exact integration (should solve problems when angular is zero):
    const double heading_old = odom->heading;
    const double r = linear / angular;
    odom->heading += angular;
    odom->x += r * (sin(odom->heading) - sin(heading_old));
    odom->y += -r * (cos(odom->heading) - cos(heading_old));
  }
}
void* diffdrive_odometry_init(struct DiffDriveOdometryConfig config) {
  // TODO: diffdrive_reset_accumulators();
  return (void*) 0xDEADBEEF;
}
int diffdrive_odometry_update_from_velocity(struct DiffDriveOdometry* odom, float left_vel, float right_vel, const int time) {
  const float dt = 0xDEADBEEF; // FIXME
  const float linear = (left_vel + right_vel) * 0.5f;
  const float angular = (right_vel - left_vel) / odom->config.wheel_separation;

  diffdrive_odometry_integrate_exact(odom, linear, angular);
  odom->timestamp = time;

  float_rolling_mean_accumulator_accumulate(odom->linear_accumulator, linear/dt);
  float_rolling_mean_accumulator_accumulate(odom->angular_accumulator, angular/dt);

  odom->linear = float_rolling_mean_accumulator_get_rolling_mean(odom->linear_accumulator);
  odom->angular = float_rolling_mean_accumulator_get_rolling_mean(odom->angular_accumulator);
  return true;
}
int diffdrive_odometry_update(struct DiffDriveOdometry* odom, float left_pos, float right_pos, const int time) {
  // TODO: compute dt
  const float dt = 0xDEADBEEF; // FIXME
  if (dt < 0.0001) return false;

  const float left_wheel_cur_pos = left_pos * odom->config.left_wheel_radius;
  const float right_wheel_cur_pos = right_pos * odom->config.right_wheel_radius;

  const float left_wheel_est_vel = left_wheel_cur_pos - odom->left_wheel_old_pos;
  const float right_wheel_est_vel = right_wheel_cur_pos - odom->right_wheel_old_pos;

  odom->left_wheel_old_pos = left_wheel_cur_pos;
  odom->right_wheel_old_pos = right_wheel_cur_pos;

  diffdrive_odometry_update_from_velocity(odom, left_wheel_est_vel, right_wheel_est_vel, time);
  return true;
}
void diffdrive_odometry_reset_accumulators(struct DiffDriveOdometry* odom) {
  // FIXME: dealloc frma before reallocating
  odom->linear_accumulator = float_rolling_mean_accumulator_init(odom->config.velocity_rolling_window_size);
  odom->angular_accumulator = float_rolling_mean_accumulator_init(odom->config.velocity_rolling_window_size);
}
