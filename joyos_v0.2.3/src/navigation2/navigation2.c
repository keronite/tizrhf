#include <joyos.h>
#include <lib/pid.h>

#define LENGTH 6.0
#define WIDTH 8.0
#define RAD_TO_DEG 57.2957795

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


#define ENCODER_TO_WHEEL_RATIO 15
#define WHEEL_CIRCUMFERENCE 25.76
#define WHEEL_TRACK 21.5
#include <lib/geartrain.h>

enum state_enum {SETUP, PLANNING, MOVING, TURNING, STOP} state;
enum planning_state_enum {INITIAL_REANGLE, FORWARD, END_REANGLE, STOP_PLANNING} planstate;

void setup_state();
void planning_state();
void moving_state();
void turning_state();
void stop_state();

void setup_filter();
void planning_filter(float init_angle, float dist, float poli, float end_angle);
void moving_filter();
void turning_filter();
void stop_filter();

void reset_pid_controller(float goal);
float get_pid_goal();

void soft_stop_motors(int duration);

int sing();

float clamp (float val, float min, float max);

float target_angle;
float target_distance;
struct position init_pos, *ip, goal_pos, *gp;

struct pid_controller controller;

uint32_t state_time;

uint16_t left_encoder_base, right_encoder_base;

struct position {float x; float y; float theta;};

void find_path(struct position *pi, struct position *pt);
float poliwhirl(float angle);
float get_turn_angle(float start, float goal);

int usetup (void) {
	set_auto_halt(0);
	return 0;
}

int umain (void) {

	// initial position and goal values for testing
	ip = &init_pos;
	gp = &goal_pos;
	init_pos.x = 0.5;
	init_pos.y = 0.5;
	init_pos.theta = 20.0;
	goal_pos.x = 2.0;
	goal_pos.y = 3.0;
	goal_pos.theta = 0.0;
	//find_path(ip, gp);

	state = SETUP;
	planstate = INITIAL_REANGLE;

	while(1) {
		 switch (state)
		 {
			case (SETUP):
				setup_state();
				break;

			case (PLANNING):
				planning_state();
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

void planning_state() {
	while(state == PLANNING) {
		printf("\nPlanning state");
		pause(1000);
		float init_angle = init_pos.theta;
		float dist = sqrt(pow((init_pos.x - goal_pos.x), 2)+pow((init_pos.y - goal_pos.y), 2));
		float poli = poliwhirl((goal_pos.y - init_pos.y)/dist);

		//printf("\noriginal pos %d %d", pi->x, pi->y);

		target_distance = dist;
		planning_filter(init_angle, dist, poli, goal_pos.theta);
		/*printf("\ninit angle %f", init_angle);
		pause(1000);
		printf("\nPoliwhirl is %f", end_angle);
		pause(1000);
		printf("\nTurn %f degrees", get_turn_angle(init_angle, end_angle));
		pause(1000);
		printf("\nMove forward %f feet", dist);
		pause(1000);
		printf("\nTurn %f degrees", get_turn_angle(end_angle, pt->theta));
		pause(1000);*/
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
	state = PLANNING;
}

void planning_filter(float init_angle, float dist, float poli, float end_angle) {
	while (state==PLANNING) {
		switch (planstate) {

		case(INITIAL_REANGLE):
			printf("\nWANT TO TURN TO %f", poli);
			pause(1000);
			target_angle = poli;
			planstate = FORWARD;
			state = TURNING;
			break;

		case(FORWARD):
			printf("\nWANT TO MOVE FORWARD %f", dist);
			pause(1000);
			planstate = END_REANGLE;
			target_distance = dist;
			state = MOVING;
			break;

		case(END_REANGLE):
			printf("\nWANT TO TURN TO %f", end_angle);
			pause(1000);
			target_angle = end_angle;
			planstate = STOP_PLANNING;
			state = TURNING;
			break;
		}
	}
}

void moving_filter() {

	if (stop_press()) {
		state = STOP;
		return;
	}
	uint16_t left_encoder_change = encoder_read(LEFT_ENCODER) - left_encoder_base;
	uint16_t right_encoder_change = encoder_read(RIGHT_ENCODER) - right_encoder_base;

	if ((left_encoder_change + right_encoder_change)/2 >= CM_TO_TICKS(target_distance * 30)) {
		//target_angle += 90;
		//target_angle = (int)target_angle%360;
		state = PLANNING;
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
		state = PLANNING;
		if (planstate == STOP_PLANNING) {
			state = STOP;
			return;
		}
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

void find_path(struct position *pi, struct position *pt) {
	float init_angle = pi->theta;
	float dist = sqrt(pow((pi->x - pt->x), 2)+pow((pi->y - pt-> y), 2));
	float end_angle = poliwhirl((pt->y - pi->y)/dist);
	//printf("\noriginal pos %d %d", pi->x, pi->y);

	printf("\ninit angle %f", init_angle);
	pause(1000);
	printf("\nPoliwhirl is %f", end_angle);
	pause(1000);
	printf("\nTurn %f degrees", get_turn_angle(init_angle, end_angle));
	pause(1000);
	printf("\nMove forward %f feet", dist);
	pause(1000);
	printf("\nTurn %f degrees", get_turn_angle(end_angle, pt->theta));
	pause(1000);
}

float poliwhirl(float angle) {
	float x = acos(angle);
	x = x*RAD_TO_DEG;
	if (x < 0) {
		x = x + 360.0;
	}
	return x;
}

float get_turn_angle(float start, float goal) {
	float turn_angle;
	float right_diff;
	if (start <= goal) {
		right_diff = goal - start;
	}
	else {
		right_diff = 360.0 - (start - goal);
	}
	if (right_diff > 180.0) {
		return right_diff - 360.0;
	}
	else return right_diff;
}
