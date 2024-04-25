#include <kyvernitis/lib/drive.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int feedback_callback(float *feedback_buffer, int buffer_len, int wheels_per_side)
{
  if (buffer_len < wheels_per_side*2) return 1;
	for (int i = 0; i < wheels_per_side; i++) {
	  feedback_buffer[i] = 1.0f;
	  feedback_buffer[wheels_per_side + i] = 1.0f;
	}
	return 0;
}
int velocity_callback(const float *velocity_buffer, int buffer_len, int wheels_per_side)
{
  if (buffer_len < wheels_per_side*2) return 1;
	printf("====WHEEL SPEEDS====\n");
	for (int i = 0; i < wheels_per_side; i++) {
		printf("left: %.2f right: %.2f\n", velocity_buffer + i,
		       velocity_buffer + wheels_per_side + i);
	}
	printf("====================\n");
	return 0;
}
int main(void)
{
	printf("Starting software drive test...\n");
	struct DiffDriveConfig drive_config = {.wheel_radius = 0.3f,
					       .wheels_per_side = 2,
					       .left_wheel_radius_multiplier = 0.2f,
					       .right_wheel_radius_multiplier = 0.2f,
					       .update_type = VELOCITY_FEEDBACK};
	struct DiffDrive* drive = diffdrive_init(&drive_config, feedback_callback, velocity_callback);
	for (int i = 0; i < 10; i++) {
	  struct DiffDriveTwist cmd = {.linear_x = 1.0f, .angular_z = 0.0f};
	  diffdrive_update(drive, cmd, 2);
	}
}
