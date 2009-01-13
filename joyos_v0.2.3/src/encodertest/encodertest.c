#include <joyos.h>

#define TEST_SPEED 32

//Motor convention, 0 is right, 1 is left
#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1

//Shaft encoder convention, 24 is right, 25 is left
#define RIGHT_ENCODER 24
#define LEFT_ENCODER 25

// usetup is called during the calibration period. It must return before the
// period ends.
int usetup (void) {
	set_auto_halt(0);
	return 0;
}

int umain (void) {
	encoder_reset(RIGHT_ENCODER);
	encoder_reset(LEFT_ENCODER);
	
	while(1)
	{
	motor_set_vel(LEFT_MOTOR, TEST_SPEED);
	motor_set_vel(RIGHT_MOTOR, 0);
	
	
	printf("\nRight: %d, Left: %d, %c", encoder_read(RIGHT_ENCODER), encoder_read(LEFT_ENCODER), 3);
	
	if (stop_press()) {
		motor_set_vel(LEFT_MOTOR, 0);
		motor_set_vel(RIGHT_MOTOR, 0);
		encoder_reset(RIGHT_ENCODER);
		encoder_reset(LEFT_ENCODER);
		printf("\nRight: %d, Left: %d, %c", encoder_read(RIGHT_ENCODER), encoder_read(LEFT_ENCODER), 3);
		go_click();
		// Poliwhirl~
	}
	
	pause(100);
	}
	return 0;
}
