#include <joyos.h>

#define FORWARD_SPEED 64
#define BACKWARD_SPEED 128
#define TURNING_SPEED 64

//Motor convention, 0 is right, 1 is left
#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1

//Shaft encoder convention, 24 is right, 25 is left
#define RIGHT_ENCODER 24
#define LEFT_ENCODER 25

#define KP 1.5
#define KD 0
#define KI .05

#define TURNING_THRESHOLD 2

#define OFFSET_ESTIMATE 2

#include <lib/pid.h>


#define ENCODER_TO_WHEEL_RATIO 15
#define WHEEL_CIRCUMFERENCE 25.76
#define WHEEL_TRACK 21.5

#define SQUARE_LENGTH_CM 60

#include <lib/geartrain.h>


enum state_enum {SETUP, MOVING, TURNING, STOP} state;

void setup_state();
void moving_state();
void turning_state();
void stop_state();

void setup_filter();
void moving_filter();
void turning_filter();
void stop_filter();

void reset_pid_controller(float goal);
float get_pid_goal();

void soft_stop_motors(int duration);

float clamp (float val, float min, float max);

float target_angle;

struct pid_controller controller;

uint32_t state_time;

uint16_t left_encoder_base, right_encoder_base;

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
				
			case (STOP):
				stop_state();
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
		gyro_init(8,1357.348162*3838.0/3600.0*1028.0/1080.0,5000);
		
		target_angle = 0;
		reset_pid_controller(target_angle);
		
		setup_filter();
	}
}

void moving_state() {
	printf("\nMoving state");
	left_encoder_base = encoder_read(LEFT_ENCODER);
	right_encoder_base = encoder_read(RIGHT_ENCODER);
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
		
		//printf("\n%f  %f", angle, target_angle);
		
		if (target_angle > angle) {
			motor_set_vel(RIGHT_MOTOR, clamp((target_angle - angle)/2.0 + 40, -TURNING_SPEED, TURNING_SPEED));
			motor_set_vel(LEFT_MOTOR, clamp((angle - target_angle)/2.0 - 40, -TURNING_SPEED, TURNING_SPEED));
		} else {
			motor_set_vel(RIGHT_MOTOR, clamp((target_angle - angle)/2.0 - 40, -TURNING_SPEED, TURNING_SPEED));
			motor_set_vel(LEFT_MOTOR, clamp((angle - target_angle)/2.0 + 40, -TURNING_SPEED, TURNING_SPEED));
		}
		pause(50);
		
		turning_filter();
	}
}

void stop_state() {
	soft_stop_motors(50);
	while(state == STOP) {
		float angle = gyro_get_degrees();
		printf("\n%f", angle);
		pause(50);
		stop_filter();
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
	if (stop_press()) {
		state = STOP;
		return;
	}
	state_time = get_time();
	state = MOVING;
}

void moving_filter() {

	if (stop_press()) {
		state = STOP;
		return;
	}
	uint16_t left_encoder_change = encoder_read(LEFT_ENCODER) - left_encoder_base;
	uint16_t right_encoder_change = encoder_read(RIGHT_ENCODER) - right_encoder_base;
	
	if ((left_encoder_change + right_encoder_change)/2 >= CM_TO_TICKS(SQUARE_LENGTH_CM)) {
		target_angle += 90;
		//target_angle = (int)target_angle%360;
		state = TURNING;
		soft_stop_motors(200);
	}
}

void turning_filter() {

	if (stop_press()) {
		state = STOP;
		return;
	}
	
	if (gyro_get_degrees() > target_angle - TURNING_THRESHOLD && 
			gyro_get_degrees() < target_angle + TURNING_THRESHOLD) {
		reset_pid_controller(target_angle);
		state_time = get_time();
		state = MOVING;
		soft_stop_motors(500);
	}
}

void stop_filter() {
	if (go_press()) {
		state = MOVING;
	}
}

void soft_stop_motors(int duration) {
	motor_set_vel(RIGHT_MOTOR, 0);
	motor_set_vel(LEFT_MOTOR, 0);
	pause(duration);
}

float clamp (float val, float min, float max) {
	if (val < min)
		return min;
	else if (val > max)
		return max;
	else
		return val;
}