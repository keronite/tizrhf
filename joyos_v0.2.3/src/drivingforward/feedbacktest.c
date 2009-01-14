#include <joyos.h>

#define FORWARD_SPEED 128
#define BACKWARD_SPEED 128
#define TURNING_SPEED 96

//Motor convention, 0 is right, 1 is left
#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1

//Shaft encoder convention, 24 is right, 25 is left
#define RIGHT_ENCODER 24
#define LEFT_ENCODER 25

#define ENCODER_TO_WHEEL_RATIO 25
#define WHEEL_CIRCUMFERENCE 10.0
#define WHEEL_TRACK 20.0

#define KP .1
#define KD .1
#define KI .1

#include <lib/geartrain.h>

#include <lib/motion.h>

#include <lib/motor_group.h>


// usetup is called during the calibration period. It must return before the
// period ends.
int usetup (void) {
	set_auto_halt(0);
	return 0;
}

int umain (void) {

	MotorGroup right_motor = motor_group_new(1,0,0,0,0,0);
	MotorGroup left_motor = motor_group_new(0,1,0,0,0,0);
	
	MotionController right_motion;
	MotionController left_motion;
	
	motion_init(&right_motion, right_motor, RIGHT_ENCODER, KP, KI, KD);
	motion_init(&left_motion, left_motor, LEFT_ENCODER, KP, KI, KD);
	
	motion_set_goal(&right_motion, 10);
	motion_set_goal(&left_motion, 10);
	
	printf("\nstarting pid");
	
	while(!(motion_goal_reached(&right_motion) && motion_goal_reached(&left_motion))) {
		motion_update(&right_motion);
		motion_update(&left_motion);
	}
	
	motor_set_vel(RIGHT_MOTOR, 0);
	motor_set_vel(LEFT_MOTOR, 0);
	
	printf("\ngoal reached");
	
	while(1);
	
	float test = 1.5;
	
	printf("%f", test);

	return 0;
}
