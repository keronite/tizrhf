#include <../src/finalstrategy/actions.h>
#include <../src/finalstrategy/util.h>
#include <happylib.h>
#include <stdlib.h>


#define TURNING_THRESHOLD 2

#define OFFSET_ESTIMATE 2

#include <lib/pid.h>

enum state_enum {PLANNING, MOVING, TURNING, STOP, FOUND, END} state;
enum planning_state_enum {INITIAL_REANGLE, FORWARD, END_REANGLE, STOP_PLANNING} planstate;

void planning_state(Position *ip, Position *gp, bool do_last_turn);
void moving_state();
void turning_state();
void stop_state(Position goal);

void planning_filter(float init_angle, float dist, float poli, float end_angle, bool do_last_turn);
void moving_filter();
void turning_filter();
void stop_filter();

void reset_pid_controller(float goal);
float get_pid_goal();

void soft_stop_motors(int duration);

float target_angle; // target angle variable to be used by all turning functions
float target_distance; // target distance variable to be used by all moving functions

struct pid_controller controller;

//uint32_t state_time;

uint16_t left_encoder_base, right_encoder_base;

float poliwhirl(float angle);
float get_turn_angle(float start, float goal);

Status drive(float distance);
Status turn(float angle);
Status dump_balls(Node* node);

/*
 * Action travel_to, moves to the location specified in the node.
 */
Status travel_to (Node* node) {

	float goal_x = node->position.x;
	float goal_y = node->position.y;
	float goal_theta = node->position.theta;

	uint8_t use_theta = node->use_theta;

	// initial position and goal values for testing
	Position init, goal, *ip, *gp;
	ip = &init;
	gp = &goal;
	init.x = global_position.x;
	init.y = global_position.y;
	init.theta = global_position.theta;
	goal.x = goal_x;
	goal.y = goal_y;
	goal.theta = goal_theta;
	bool do_last_turn = use_theta;

	state = PLANNING;
	planstate = INITIAL_REANGLE;

	while(state != STOP) {
		 switch (state)
		 {
			case (PLANNING):
				planning_state(ip, gp, do_last_turn);
				break;

			case (MOVING):
				moving_state();
				break;

			case (TURNING):
				turning_state();
				break;

			case (STOP):
				break;

			default:
				break;
		 }
	}
	stop_state(goal);
	return SUCCESS;
}

void planning_state(Position *ip, Position *gp, bool do_last_turn) {
	while(state == PLANNING) {
		printf("\nPlanning state");
		pause(1000);
		float init_angle = gyro_get_degrees();
		float dist = sqrt(pow((ip->x - gp->x), 2)+pow((ip->y - gp->y), 2));
		float poli = poliwhirl((gp->y - ip->y)/dist);

		target_distance = dist;
		planning_filter(init_angle, dist, poli, gp->theta, do_last_turn);
	}
}

void moving_state() {
	printf("\nMoving state");

	int motor_multiplier = 1;

	left_encoder_base = encoder_read(LEFT_ENCODER);
	right_encoder_base = encoder_read(RIGHT_ENCODER);

	if (target_distance < 0){
		motor_multiplier = -1;
	}
	while(state == MOVING)
	{
		float input = gyro_get_degrees();

		float output = update_pid_input(&controller, input);

		motor_set_vel(RIGHT_MOTOR, motor_multiplier * (FORWARD_SPEED + (int)output + OFFSET_ESTIMATE));
		motor_set_vel(LEFT_MOTOR, motor_multiplier * (FORWARD_SPEED - (int)output - OFFSET_ESTIMATE));

		pause(50);

		moving_filter();
	}
}

void turning_state() {
	printf("\nTurning state");

	while(state == TURNING) {

		float angle = gyro_get_degrees();

		if (target_angle > angle) {
			motor_set_vel(RIGHT_MOTOR, clamp((target_angle - angle) + 60, -TURNING_SPEED, TURNING_SPEED));
			motor_set_vel(LEFT_MOTOR, clamp((angle - target_angle) - 60, -TURNING_SPEED, TURNING_SPEED));
		} else {
			motor_set_vel(RIGHT_MOTOR, clamp((target_angle - angle) - 60, -TURNING_SPEED, TURNING_SPEED));
			motor_set_vel(LEFT_MOTOR, clamp((angle - target_angle) + 60, -TURNING_SPEED, TURNING_SPEED));
		}
		pause(50);

		turning_filter();
	}
}

void stop_state(Position goal) {
	global_position.x = goal.x;
	global_position.y = goal.y;
}

void reset_pid_controller(float goal) {
	init_pid(&controller, KP, KI, KD, NULL, NULL);
	controller.goal = goal;
}

float get_pid_goal() {
	return controller.goal;
}

void planning_filter(float init_angle, float dist, float poli, float end_angle, bool do_last_turn) {
	while (state==PLANNING) {
		switch (planstate) {

		case(INITIAL_REANGLE):
			printf("\nTurning to %f", (double) poli);
			pause(1000);
			target_angle = poli;
			planstate = FORWARD;
			state = TURNING;
			break;

		case(FORWARD):
			printf("\nMoving forward %f feet", (double) dist);
			pause(1000);
			if (do_last_turn) {
				planstate = END_REANGLE;
			}
			else {
				planstate = STOP_PLANNING;
			}
			target_distance = dist;
			state = MOVING;
			break;

		case(END_REANGLE):
			printf("\nTurning to %f", (double) end_angle);
			pause(1000);
			target_angle = end_angle;
			planstate = STOP_PLANNING;
			state = TURNING;
			break;

		case(STOP_PLANNING):
			state = STOP;
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

	if ((left_encoder_change + right_encoder_change)/2 >= abs(CM_TO_TICKS(target_distance * 30.0 / 12.0))) {
		state = PLANNING;
		if (planstate == STOP_PLANNING) {
			state = STOP;
			soft_stop_motors(200);
			return;
		}
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
		//state_time = get_time();
		state = PLANNING;
		if (planstate == STOP_PLANNING) {
			state = STOP;
			soft_stop_motors(500);
			return;
		}
		soft_stop_motors(500);
	}
}

void stop_filter() {
	return;
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

/*
 * Action drive(distance), moves forward a certain distance.
 */
Status drive(float distance) {
	printf("\nIn function drive()");
	state = MOVING;
	planstate = STOP_PLANNING;
	target_distance = distance;
	while(state != STOP) {
		 switch (state)
		 {
			 case(PLANNING):
				 break;

			 case (MOVING):
				 moving_state();
				 break;

			 case (TURNING):
				 break;

			 case (STOP):
				 break;

			default:
				break;
		 }
	}
	return SUCCESS;
}

/*
 * Action turn(angle), turns to a certain angle.
 */
Status turn(float angle) {
	state = TURNING;
	planstate = STOP_PLANNING;
	target_angle = angle;
	while(state != STOP) {
		 switch (state)
		 {
			 case (PLANNING):
				 break;

			 case (MOVING):
				 break;

			 case (TURNING):
				 turning_state();
				 break;

			 case (STOP):
				 break;

			default:
				break;
		 }
	}
	return SUCCESS;
}

/*
 * dump_balls(), to be called when we wish to actuate
 * servo and deposit balls into goal. Assumes we are near
 * our goal.
 */
Status dump_balls(Node* node) {
	printf("\nActuating servo.");
	servo_set_pos(1, SERVO_POS);
	drive(6);
	//motor_set_vel(RIGHT_MOTOR, 128);
	//motor_set_vel(LEFT_MOTOR, 128);
	uint32_t start = get_time();
	while (get_time() - start < 5000) {
		servo_set_pos(1, SERVO_POS2/2);
		pause(500);
		servo_set_pos(1, SERVO_POS - 10);
		pause(500);
	}
	soft_stop_motors(500);
	return SUCCESS;
}

/*
 * Determines which cardinal direction we face at the
 * beginning of a match.
 */
Status attempt_orient(Node * node) {

	servo_set_pos(FRONT_SERVO, 255);
	pause(1000);
	uint16_t wall_front_dist = irdist_read(FRONT_SHARP);
	bool wall_front = false;
	if (wall_front_dist < 30) {
		wall_front = true;
	}

	servo_set_pos(FRONT_SERVO, 0);
	pause(1000);
	uint16_t wall_right_dist = irdist_read(FRONT_SHARP);
	bool wall_right = false;
	if (wall_right_dist < 30) {
		wall_right = true;
	}

	if (wall_front && wall_right) {
		printf("\n180 %d %d", wall_front_dist, wall_right_dist);
		gyro_set_degrees(180);
	} else if (wall_front && !wall_right) {
		printf("\n-90 %d %d", wall_front_dist, wall_right_dist);
		gyro_set_degrees(-90);
	} else if (!wall_front && wall_right) {
		printf("\n90 %d %d", wall_front_dist, wall_right_dist);
		gyro_set_degrees(90);
	} else {
		printf("\n0 %d %d", wall_front_dist, wall_right_dist);
		gyro_set_degrees(0);
	}
	return SUCCESS;
}


/*
 * Line search looks for the designated line using position estimates
 */

 void moving_line_filter() {

	if (stop_press()) {
		state = STOP;
		return;
	}
	uint16_t left_encoder_change = encoder_read(LEFT_ENCODER) - left_encoder_base;
	uint16_t right_encoder_change = encoder_read(RIGHT_ENCODER) - right_encoder_base;

	//IF FIND LINE, RETURN SUCCESS (DO BEST GUESS CHECK)

	if ((left_encoder_change + right_encoder_change)/2 >= abs(CM_TO_TICKS(target_distance * 30.0 / 12.0))) {
		state = END;
		soft_stop_motors(200);
	}
}


void moving_line_state() {
	printf("\nMoving line state");

	int motor_multiplier = 1;

	left_encoder_base = encoder_read(LEFT_ENCODER);
	right_encoder_base = encoder_read(RIGHT_ENCODER);

	if (target_distance < 0){
		motor_multiplier = -1;
	}
	while(state == MOVING)
	{
		float input = gyro_get_degrees();

		float output = update_pid_input(&controller, input);

		motor_set_vel(RIGHT_MOTOR, motor_multiplier * (FORWARD_SPEED + (int)output + OFFSET_ESTIMATE));
		motor_set_vel(LEFT_MOTOR, motor_multiplier * (FORWARD_SPEED - (int)output - OFFSET_ESTIMATE));

		pause(50);

		moving_line_filter();
	}
}

Status line_search(Node * node) {
	printf("\nLine search");
	state = MOVING;
	target_distance = 100;
	while(state == MOVING) {
		 switch (state)
		 {
			 case (MOVING):
				 moving_state();
				 break;

			 default:
				 break;
		 }
	}
	if (state == FOUND) {
		return SUCCESS;
	} else {
		return FAILURE;
	}
}

/*
 * Attempts to use sharp distance sensors to determine where
 * we are on the game board.
 */
Status get_abs_pos(Node* node) {
	int angle = (int)gyro_get_degrees();

	//printf("\n%d  %d", angle, angle%360);

	servo_set_pos(FRONT_SERVO, degrees_to_servo_units(-angle));
	pause(1000);
	uint8_t x = irdist_read(23)/2.54;

	servo_set_pos(FRONT_SERVO, degrees_to_servo_units(-angle - 90));
	pause(1000);
	uint8_t y = irdist_read(23)/2.54;

	if (0 >= angle%360 && angle%360 < 90) {
		global_position.x = x;
		global_position.y = y;
		printf("\n x = %f, y = %f", (double) global_position.x, (double) global_position.y);
	}
	else if (90 >= angle%360 && angle%360 < 180) {
		global_position.x = BOARD_LENGTH - x;
		global_position.y = y;
		printf("\n x = %f, y = %f", (double) global_position.x, (double) global_position.y);
	}
	else if (180 >= angle%360 && angle%360 < 270) {
		global_position.x = BOARD_LENGTH - x;
		global_position.y = BOARD_WIDTH - y;
		printf("\n x = %f, y = %f", (double) global_position.x, (double) global_position.y);
	}
	else {
		global_position.x = x;
		global_position.y = BOARD_WIDTH - y;
		printf("\n x = %f, y = %f", (double) global_position.x, (double) global_position.y);
	}

	global_position.theta = angle;
	return SUCCESS;
}