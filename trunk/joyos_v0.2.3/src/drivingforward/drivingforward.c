#include <joyos.h>

#define FORWARD_SPEED 64
#define BACKWARD_SPEED 128
#define TURNING_SPEED 96

//Motor convention, 0 is right, 1 is left
#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1

//Shaft encoder convention, 24 is right, 25 is left
#define RIGHT_ENCODER 24
#define LEFT_ENCODER 25

#define KP 1.5//.6*.8
#define KD 0//2*.8/10
#define KI .05//.8*10/8

#define OFFSET_ESTIMATE 4

#include <lib/pid.h>


// usetup is called during the calibration period. It must return before the
// period ends.
int usetup (void) {
	set_auto_halt(0);
	return 0;
}

int umain (void) {

	struct pid_controller controller;
	
	init_pid(&controller, KP, KI, KD, NULL, NULL);
	
	controller.goal = 0;
	
	printf("\nPress go");
	go_click();
	printf("\nStabilizing");
	pause(1000);
	
	printf("\nInitializing");
	gyro_init(8,1357.348162*1028.0/1080.0,5000);
	
	
	while(1)
	{
		float dif = gyro_get_degrees();
		
		printf("\n%f", (double)dif);
		
		float output = update_pid_input(&controller, dif);
		
		printf("  %d", (int)output);
		
		motor_set_vel(RIGHT_MOTOR, FORWARD_SPEED + (int)output + OFFSET_ESTIMATE);
		motor_set_vel(LEFT_MOTOR, FORWARD_SPEED - (int)output - OFFSET_ESTIMATE);
		
		
		if (stop_press()) {
			motor_set_vel(LEFT_MOTOR, 0);
			motor_set_vel(RIGHT_MOTOR, 0);
			init_pid(&controller, KP, KI, KD, NULL, NULL);
			go_click();
			controller.goal = gyro_get_degrees();
		}
	
		pause(50);
		
	}
	return 0;
}
