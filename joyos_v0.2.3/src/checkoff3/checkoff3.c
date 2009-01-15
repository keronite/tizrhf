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

#define KP 1.5
#define KD 0
#define KI .05

#define OFFSET_ESTIMATE 4

#include <lib/pid.h>

enum state_enum {SETUP, MOVING, TURNING} state;

void setup_state();
void moving_state();
void turning_state();

void setup_filter();
void moving_filter();
void turning_filter();

void reset_pid_controller(float goal);
float get_pid_goal();

void soft_stop_motors(int duration);

float target_angle;

struct pid_controller controller;

uint32_t state_time;

// usetup is called during the calibration period. It must return before the
// period ends.
int usetup (void) {
	set_auto_halt(0);
	return 0;
}

int umain (void) {
	
	state = SETUP;
	
	while(1) {
		 switch (state)
		 {
			case (SETUP):
				setup_state();
				break;
		
			case (MOVING):
				moving_state();
				break;
		
			case (TURNING):
				turning_state();
				break;
		 }
	
	}

	

	return 0;
}

void setup_state() {
	while (state == SETUP) {
		
		printf("\nPress go");
		go_click();
		printf("\nStabilizing");
		pause(1000);
		
		printf("\nInitializing");
		gyro_init(8,1357.348162*3838.0/3600.0,5000);
		
		target_angle = 0;
		reset_pid_controller(target_angle);
		
		setup_filter();
	}
}

void moving_state() {
	printf("\nMoving state");
	while(state == MOVING)
	{
		float input = gyro_get_degrees();
		
		float output = update_pid_input(&controller, input);
		
		motor_set_vel(RIGHT_MOTOR, FORWARD_SPEED + (int)output + OFFSET_ESTIMATE);
		motor_set_vel(LEFT_MOTOR, FORWARD_SPEED - (int)output - OFFSET_ESTIMATE);
		
		pause(50);
		
		moving_filter();
	}
}

void turning_state() {
	printf("\nTurning state");
	while(state == TURNING) {
		
		float angle = gyro_get_degrees();
		
		printf("\n%f  %f", angle, target_angle);
		motor_set_vel(RIGHT_MOTOR, (target_angle - angle)/1.5 + 30);
		motor_set_vel(LEFT_MOTOR, (angle - target_angle)/1.5 - 30);
		pause(50);
		
		turning_filter();
	}
}


void reset_pid_controller(float goal) {
	init_pid(&controller, KP, KI, KD, NULL, NULL);
	controller.goal = goal;
}

float get_pid_goal() {
	return controller.goal;
}

void setup_filter() {
	state_time = get_time();
	state = MOVING;
}

void moving_filter() {
	if (get_time() - state_time > 4000) {
		target_angle += 90;
		state = TURNING;
		soft_stop_motors(200);
	}
}

void turning_filter() {
	if (gyro_get_degrees() > target_angle) {
		reset_pid_controller(target_angle);
		state_time = get_time();
		state = MOVING;
		soft_stop_motors(500);
	}
}

void soft_stop_motors(int duration) {
	motor_set_vel(RIGHT_MOTOR, 0);
	motor_set_vel(LEFT_MOTOR, 0);
	pause(duration);
}