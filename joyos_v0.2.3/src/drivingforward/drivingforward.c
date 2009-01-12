#include <joyos.h>

#define FORWARD_SPEED 128
#define BACKWARD_SPEED 128
#define TURNING_SPEED 64

//Motor convention, 0 is right, 1 is left
#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1

//Shaft encoder convention, 24 is right, 25 is left
#define RIGHT_ENCODER 24
#define LEFT_ENCODER 25

// State of robot movement determined via readings of shaft encoders
//enum states {TOO_LEFT, TOO_RIGHT, OK};
//enum states current_state = OK;


// usetup is called during the calibration period. It must return before the
// period ends.
int usetup (void) {
	set_auto_halt(0);
	return 0;
}

int umain (void) {
	while(1)
	{
		//printf("\nright: %d, left: %d, diff: %d", encoder_read(RIGHT_ENCODER), encoder_read(LEFT_ENCODER), encoder_read(LEFT_ENCODER) - encoder_read(EIGHT_ENCODER));
		if ((encoder_read(RIGHT_ENCODER) - encoder_read(LEFT_ENCODER) > 100))
		{
			motor_set_vel(RIGHT_MOTOR, TURNING_SPEED);
			motor_set_vel(LEFT_MOTOR, FORWARD_SPEED);
			printf("\nSlight left, diff = %d", encoder_read(RIGHT_ENCODER) - encoder_read(LEFT_ENCODER));
		}
		if ((encoder_read(RIGHT_ENCODER) - encoder_read(LEFT_ENCODER) < -100))
		{
			motor_set_vel(RIGHT_MOTOR, FORWARD_SPEED);
			motor_set_vel(LEFT_MOTOR, TURNING_SPEED);
			printf("\nSlight right, diff = %d", encoder_read(RIGHT_ENCODER) - encoder_read(LEFT_ENCODER));
		}
		else
		{
			motor_set_vel(RIGHT_MOTOR, FORWARD_SPEED);
			motor_set_vel(LEFT_MOTOR, FORWARD_SPEED);
			printf("\nDrive forward, diff = %d", encoder_read(RIGHT_ENCODER) - encoder_read(LEFT_ENCODER));
		}
	}
	return 0;
}
