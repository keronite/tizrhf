#include <../src/finalstrategy/actions.h>
#include <../src/finalstrategy/util.h>
#include <../src/finalstrategy/globals.h>
#include <happylib.h>
#include <stdlib.h>


#define TURNING_THRESHOLD 1.5

#define OFFSET_ESTIMATE -5

#include <lib/pid.h>

int count = 0;

enum state_enum {PLANNING, MOVING, TURNING, STOP, FOUND, END, FAIL_STATE} state;
enum planning_state_enum {INITIAL_REANGLE, FORWARD, END_REANGLE, STOP_PLANNING} planstate;

void planning_state(Position *ip, Position *gp, bool do_last_turn);
void moving_state(float scale, float start, float end, float dist, Line line, void (*filter)(float,float,float,Line));
void turning_state();
void stop_state(Position goal);

void planning_filter(float init_angle, float dist, float poli, float end_angle, bool do_last_turn);
void moving_filter(float start, float end, float dist, Line line);
void turning_filter();
void stop_filter();

void moving_gather_filter(float start_servo, float end_servo, float dist, Line line);
void reset_pid_controller(float goal);

float target_angle; // target angle variable to be used by all turning functions
float target_distance; // target distance variable to be used by all moving functions

struct pid_controller controller;

uint16_t left_encoder_base, right_encoder_base;

float get_turn_angle(float start, float goal);

Status drive(float distance, float scale);
Status turn(float angle);

uint32_t state_time;

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
				moving_state(1,0,0,0,TOP_LINE,moving_filter);
				break;

			case (TURNING):
				turning_state();
				break;

			case (STOP):
				break;
				
			case(FAIL_STATE):
				return FAILURE;

			default:
				break;
		 }
	}
	stop_state(goal);
	return SUCCESS;
}

void planning_state(Position *ip, Position *gp, bool do_last_turn) {
	while(state == PLANNING) {
		//printf("\nPlanning state");
		//pause(100);
		float init_angle = gyro_get_degrees();
		float dist = sqrt(pow((ip->x - gp->x), 2)+pow((ip->y - gp->y), 2));
		float travel_angle = atan2(-1*(gp->x - ip->x),(gp->y - ip->y))*RAD_TO_DEG;

		target_distance = dist;
		planning_filter(init_angle, dist, travel_angle, gp->theta, do_last_turn);
	}
}

void moving_state(float scale, float start, float end, float dist, Line line, void (*filter)(float,float,float,Line)) {

	state_time = get_time();
	
	//printf("\nMoving state");

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

		motor_set_vel(RIGHT_MOTOR, (float)motor_multiplier * scale * (float)(FORWARD_SPEED + (int)output + OFFSET_ESTIMATE));
		motor_set_vel(LEFT_MOTOR, (float)motor_multiplier * scale * (float)(FORWARD_SPEED - (int)output - OFFSET_ESTIMATE));

		pause(20);

		filter(start,end,dist,line);
		
		if ((get_time() - state_time > 2000) && ((motor_get_current_MA(RIGHT_MOTOR) > 700) || (motor_get_current_MA(LEFT_MOTOR) > 700))){
			state = FAIL_STATE;
			soft_stop_motors(1);
		}
	}
}

void turning_state() {
	//printf("\nTurning state");
	state_time = get_time();

	while(state == TURNING) {

		float angle = gyro_get_degrees();

		if (target_angle > angle) {
			motor_set_vel(RIGHT_MOTOR, clamp((target_angle - angle)/8.0 + 60, -TURNING_SPEED, TURNING_SPEED));
			motor_set_vel(LEFT_MOTOR, clamp((angle - target_angle)/8.0 - 60 + OFFSET_ESTIMATE, -TURNING_SPEED + OFFSET_ESTIMATE, TURNING_SPEED));
		} else {
			motor_set_vel(RIGHT_MOTOR, clamp((target_angle - angle)/8.0 - 60, -TURNING_SPEED, TURNING_SPEED));
			motor_set_vel(LEFT_MOTOR, clamp((angle - target_angle)/8.0 + 60 - OFFSET_ESTIMATE, -TURNING_SPEED, TURNING_SPEED - OFFSET_ESTIMATE));
		}
		pause(50);

		turning_filter();
		if ((get_time() - state_time > 2000) && ((motor_get_current_MA(RIGHT_MOTOR) > 700) || (motor_get_current_MA(LEFT_MOTOR) > 700))){
			state = FAIL_STATE;
			soft_stop_motors(1);
		}
	}
}

void stop_state(Position goal) {
	global_position.x = goal.x;
	global_position.y = goal.y;
}

void planning_filter(float init_angle, float dist, float travel_angle, float end_angle, bool do_last_turn) {
	while (state==PLANNING) {
		switch (planstate) {

		case(INITIAL_REANGLE):
			//printf("\nTurning to %f", (double) travel_angle);
			//pause(100);
			target_angle = travel_angle;
			planstate = FORWARD;
			state = TURNING;
			break;

		case(FORWARD):
			//printf("\nMoving forward %f inches", (double) dist);
			//pause(100);
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
			//printf("\nTurning to %f", (double) end_angle);
			//pause(100);
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

void moving_filter(float start, float end, float dist, Line line) {

	if (stop_press()) {
		state = STOP;
		return;
	}
	uint16_t left_encoder_change = encoder_read(LEFT_ENCODER) - left_encoder_base;
	uint16_t right_encoder_change = encoder_read(RIGHT_ENCODER) - right_encoder_base;

	if ((left_encoder_change + right_encoder_change)/2 >= abs(CM_TO_TICKS(target_distance * 2.54))) {
		state = PLANNING;
		if (planstate == STOP_PLANNING) {
			state = STOP;
			soft_stop_motors(200);
			return;
		}
		hard_brake();
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
			soft_stop_motors(200);
			return;
		}
		soft_stop_motors(200);
	}
}

void stop_filter() {
	return;
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
Status drive(float distance, float speed_scale) {
	//printf("\nIn function drive()");
	state = MOVING;
	planstate = STOP_PLANNING;
	target_distance = distance;
	while(state != STOP) {
		 switch (state)
		 {
			 case(PLANNING):
				 break;

			 case (MOVING):
				 moving_state(speed_scale,0,0,0,TOP_LINE,moving_filter);
				 break;

			 case (TURNING):
				 break;

			 case (STOP):
				 break;
				 
			case(FAIL_STATE):
				return FAILURE;

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
				 
				 
			case(FAIL_STATE):
				return FAILURE;

			default:
				break;
		 }
	}
	return SUCCESS;
}


void reset_pid_controller(float goal) {
	init_pid(&controller, KP, KI, KD, NULL, NULL);
	controller.goal = goal;
}


//////////////////////////////////////////////////////////////////
//##############################################################//
//##############################################################//
//##############################################################//
//##############################################################//
//##############################################################//
//////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////
/*
 * dump_balls(), to be called when we wish to actuate
 * servo and deposit balls into goal. Assumes we are near
 * our goal.
 */
Status dump_balls(Node* node) {

	servo_set_pos(LIFT_SERVO,LIFT_RAISE);
	pause(500);
	while(1) {
		if (digital_read(LIFT_BUMP)) {
			servo_set_pos(LIFT_SERVO, LIFT_TOP);
			break;
		}
	}


///////

	float goal_x;
	float goal_y;
	float goal_theta;
	
	if (count == 0) {
		goal_x = 59.5;
		goal_y = 16;
		goal_theta = 0;
	} else {
		goal_x = 61.5;
		goal_y = 14;
		goal_theta = 0;
	}
	
	count++;

	uint8_t use_theta = false;

	// initial position and goal values for testing
	Position init, goal, *ip, *gp;
	ip = &init;
	gp = &goal;
	init.x = global_position.x;
	init.y = global_position.y;
	init.theta = gyro_get_degrees();
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
				moving_state(DUMPING_SPEED_MULT,JAW_CLOSED,JAW_OPEN,8,TOP_LINE,moving_gather_filter);
				break;

			case (TURNING):
				turning_state();
				break;

			case (STOP):
				break;
				
				
			case(FAIL_STATE):
				return FAILURE;

			default:
				break;
		 }
	}
	stop_state(goal);

//////
	//drive_gather(DUMP_FORWARD_DIST,JAW_CLOSED,JAW_OPEN,DUMPING_SPEED_MULT);

	servo_set_pos(LIFT_SERVO, LIFT_MID);
	soft_stop_motors(1);

	drive(DUMP_REVERSE_DIST, DUMPING_REV_SPEED_MULT);
	servo_set_pos(JAW_SERVO, JAW_CLOSED);
	servo_set_pos(LIFT_SERVO, LIFT_LOWER);
	while(1) {
		if (digital_read(LIFT_BUMP)) {
			servo_set_pos(LIFT_SERVO, LIFT_BOTTOM);
			break;
		}
	}
	return SUCCESS;
}

///////////////////////////////////////////////////////////////////
/*
 * Determines which cardinal direction we face at the
 * beginning of a match.
 */
Status attempt_orient(Node * node) {

	servo_set_pos(FRONT_SERVO, 255);
	pause(500);
	uint16_t wall_front_dist = irdist_read(FRONT_SHARP);
	bool wall_front = false;
	if (wall_front_dist < 50) {
		wall_front = true;
	}

	servo_set_pos(FRONT_SERVO, 0);
	pause(500);
	uint16_t wall_right_dist = irdist_read(FRONT_SHARP);
	bool wall_right = false;
	if (wall_right_dist < 50) {
		wall_right = true;
	}

	if (wall_front && wall_right) {
		global_position.x = 8.5;
		global_position.y = 10;
		gyro_set_degrees(-180);
	} else if (wall_front && !wall_right) {
		global_position.x = 11;
		global_position.y = 8;
		gyro_set_degrees(90);
	} else if (!wall_front && wall_right) {
		global_position.x = 7.5;
		global_position.y = 8;
		gyro_set_degrees(-90);
	} else {
		global_position.x = 8;
		global_position.y = 6.5;
		gyro_set_degrees(0);
	}

	return SUCCESS;
}

///////////////////////////////////////////////////////////////////////
/*
 * Line search looks for the designated line using position estimates
 */

 void moving_line_filter(Line line) {

	if (stop_press()) {
		state = STOP;
		return;
	}
	uint16_t left_encoder_change = encoder_read(LEFT_ENCODER) - left_encoder_base;
	uint16_t right_encoder_change = encoder_read(RIGHT_ENCODER) - right_encoder_base;

	//IF FIND LINE, RETURN SUCCESS (DO BEST GUESS CHECK)
	uint8_t leds = get_led_readings();

	if (leds != 0) {
	//if (maximum x for line) > (our position)
	//60 > our position
		if (CM_TO_TICKS((get_line_position(line).x + 6.0)*2.54) > CM_TO_TICKS(global_position.x*2.54) - (left_encoder_change + right_encoder_change)/2) {
			state = FOUND;
			soft_stop_motors(200);
		}
	}
}


void moving_line_state(Line line) {
	//printf("\nMoving line state");

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

		motor_set_vel(RIGHT_MOTOR, motor_multiplier * .6 *(FORWARD_SPEED + (int)output + OFFSET_ESTIMATE));
		motor_set_vel(LEFT_MOTOR, motor_multiplier * .6 * (FORWARD_SPEED - (int)output - OFFSET_ESTIMATE));

		pause(50);

		moving_line_filter(line);
	}
}

Status line_search(Node * node) {
	//printf("\nLine search");
	state = MOVING;
	target_distance = 100;
	reset_pid_controller(gyro_get_degrees());
	while(state == MOVING) {
		 switch (state)
		 {
			 case (MOVING):
				 moving_line_state(node->line);
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

///////////////////////////////////////////////////////////////////
/*
 * Raises the flag
 */

Status flagbox(Node * node) {
	turn(0);
	motor_set_vel(FLAG_MOTOR, 225);
	motor_set_vel(RIGHT_MOTOR, -65);
	motor_set_vel(LEFT_MOTOR, -65);
	pause(3000);
	motor_set_vel(RIGHT_MOTOR, -15);
	motor_set_vel(LEFT_MOTOR, -15);
	while(1);
	return SUCCESS;
}

////////////////////////////////////////////////////////////////////
/*
 * Pick up a ball
 */

Status acquire_ball(Node * node) {
	//servo_set_pos(JAW_SERVO, 150*1.5);
	//!!!!!!!!

	Position p = get_ball_position(node->ball);
	//float angle = gyro_get_degrees();
	float goal_x = p.x;// + 2.0*sin(angle/RAD_TO_DEG);
	float goal_y = p.y;// - 2.0*cos(angle/RAD_TO_DEG);
	float goal_theta = 0;

	uint8_t use_theta = false;

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
				//printf("\ngx=%.2f, gy=%.2f, d=%.2f",goal.x,goal.y, target_distance);
				//go_click();
				target_distance -= 2.5;
				goal.x += 2.5*sin(target_angle/RAD_TO_DEG);
				goal.y -= 2.5*cos(target_angle/RAD_TO_DEG);
				//printf("\ngx=%.2f, gy=%.2f, d=%.2f",goal.x,goal.y, target_distance);
				//go_click();
				moving_state(.75,JAW_OPEN,JAW_INSIDE,6,TOP_LINE,moving_gather_filter);
				break;

			case (TURNING):
				turning_state();
				pause(250);
				servo_set_pos(JAW_SERVO, JAW_OPEN);//Open servo
				pause(500);
				break;


			case(FAIL_STATE):
				return FAILURE;
				
			case (STOP):
				break;

			default:
				break;
		 }
	}
	stop_state(goal);

	//drive_gather(ACQUIRE_DISTANCE,JAW_OPEN,JAW_INSIDE,ACQUIRE_MULT);//Drive a little


	//!!!!
	servo_set_pos(JAW_SERVO, JAW_CLOSED);
	//float heading = gyro_get_degrees();
	//float deltaX = -ACQUIRE_DISTANCE*sin(heading/(float)RAD_TO_DEG);
	//float deltaY = ACQUIRE_DISTANCE*cos(heading/(float)RAD_TO_DEG);
	//global_position.x += deltaX;
	//global_position.y += deltaY;
	//printf("\n x = %f y = %f", global_position.x, global_position.y);
	//go_click();
	return SUCCESS;
	//Stop
}


void moving_gather_filter(float start_servo, float end_servo, float dist, Line line) {

	if (stop_press()) {
		state = STOP;
		return;
	}
	uint16_t left_encoder_change = encoder_read(LEFT_ENCODER) - left_encoder_base;
	uint16_t right_encoder_change = encoder_read(RIGHT_ENCODER) - right_encoder_base;

	int encoder_avg = (left_encoder_change + right_encoder_change)/2;
	int encoder_goal = abs(CM_TO_TICKS(target_distance * 2.54));

	int encoder_start = abs(CM_TO_TICKS((target_distance - dist)*2.54));

	float ratio = 0;

	if (encoder_avg > encoder_start) {
		ratio = (float)(encoder_avg - encoder_start)/(float)(encoder_goal - encoder_start);

		servo_set_pos(JAW_SERVO, ratio*(end_servo-start_servo) + start_servo);
	}

	if (ratio >= 1) {
		state = PLANNING;
		if (planstate == STOP_PLANNING) {
			state = STOP;
			soft_stop_motors(200);
			return;
		}
		hard_brake();
	}
}

////////////////////////////////////////////////////////////////
/*
 * Sharp-distance positioning when on a line
 */

 //KEVIN: Refactor this when you get the chance.  I wasn't
 //sure if get_abs_pos was deprecated or not

typedef enum {NORTH, SOUTH, WEST, EAST} Orientation;
Orientation get_orientation_front (int angle);
Orientation get_orientation_back (int angle);

Status get_pos_front(Node* node) {
	if (node->use_theta) {
		//printf("\nROFL");
		//go_click();
		turn(node->position.theta);
	}
	
	int angle = (int)gyro_get_degrees();
	int servo_set1 = degrees_to_servo_units(-angle);
	int servo_set2 = degrees_to_servo_units(-angle - 90);
	int a1 = servo_units_to_degrees(servo_set1);
	int a2 = servo_units_to_degrees(servo_set2);
	Orientation s1 = get_orientation_front(angle - a1);
	Orientation s2 = get_orientation_front(angle - a2);
	//printf("\n%d  %d", angle, angle%360);
	float x, y;
	x = 0; y = 0;

	servo_set_pos(FRONT_SERVO, servo_set1);
	pause(500);
	for (int i = 0; i < 30; i++) {
		x = (x*i + irdist_read(FRONT_SHARP)/2.54)/(i+1);
	}

	servo_set_pos(FRONT_SERVO, servo_set2);
	pause(500);
	for (int i = 0; i < 30; i++) {
		y = (y*i + irdist_read(FRONT_SHARP)/2.54)/(i+1);
	}
	//printf("\n1st: %d, %s, 2nd: %d, %s", x, s1, y, s2);


	if (s1 == NORTH) {
		global_position.y = BOARD_Y - x;
	}
	else if (s1 == SOUTH) {
		global_position.y = x;
	}
	if (s2 == EAST) {
		global_position.x = BOARD_X - y;
	}
	else if (s1 == WEST) {
		global_position.x = y;
	}
	global_position.x = global_position.x - 7.8*(sin((50.2 - angle)/RAD_TO_DEG));
	global_position.y = global_position.y - 7.8*(cos((50.2 - angle)/RAD_TO_DEG));
	//global_position.x = 72-18;
	//global_position.y = 18;
	//printf("\nx: %f, y: %f", (double)global_position.x, (double)global_position.y);
	//go_click();
	return SUCCESS;
}

Status get_pos_back(Node* node) {
	if (node->use_theta) {
		//printf("\nROFL");
		//go_click();
		turn(node->position.theta);
	}
	int angle = (((int)gyro_get_degrees())%360 + 360)%360;
	int servo_set1 = degrees_to_servo_units2(angle%90);
	int servo_set2 = degrees_to_servo_units2(angle%90 + 90);
	int a1 = servo_units_to_degrees2(servo_set1);
	int a2 = servo_units_to_degrees2(servo_set2);
	Orientation s1 = get_orientation_back(angle - a1);
	Orientation s2 = get_orientation_back(angle - a2);
	//printf("\n%d  %d", angle, angle%360);
	float x, y;
	x = 0; y = 0;

	servo_set_pos(BACK_SERVO, servo_set1);
	pause(500);
	for (int i = 0; i < 30; i++) {
		x = (x*i + irdist_read(BACK_SHARP)/2.54)/(i+1);
	}

	servo_set_pos(BACK_SERVO, servo_set2);
	pause(500);
	for (int i = 0; i < 30; i++) {
		y = (y*i + irdist_read(BACK_SHARP)/2.54)/(i+1);
	}

	//printf("\n 1st: %d, %s 2nd: %d, %s g: %d", x, s1, y, s2, angle%360);

	if (s1 == NORTH) {
		global_position.y = BOARD_Y - x;
	}
	else if (s1 == SOUTH) {
		global_position.y = x;
	}
	else if (s1 == WEST) {
		global_position.x = x;
	}
	else if (s1 == EAST) {
		global_position.x = BOARD_X - x;
	}
	if (s2 == NORTH) {
		global_position.y = BOARD_Y - y;
	}
	else if (s2 == SOUTH) {
		global_position.y = y;
	}
	else if (s2 == WEST) {
		global_position.x = y;
	}
	else if (s2 == EAST) {
		global_position.x = BOARD_X - y;
	}
	global_position.x = global_position.x - 5.0*(sin((53.2 - angle - 180.0)/RAD_TO_DEG));
	global_position.y = global_position.y - 5.0*(cos((53.2 - angle - 180.0)/RAD_TO_DEG));
	//printf("\nx: %f, y: %f", (double)global_position.x, (double)global_position.y);
	return SUCCESS;
}

Orientation get_orientation_front (int angle) {
	int anglepos;
	if (angle%360 < 0){
		anglepos = angle%360 + 360;
	}
	else {
		anglepos = angle%360;
	}
	Orientation o = NORTH;
	char* os = "U";
	int min_angle_dif = 45;
	if (abs(anglepos) < min_angle_dif) {
		min_angle_dif = abs(anglepos - 0);
		o = NORTH;
		os = "N";
	}
	if (abs(anglepos - 90) < min_angle_dif) {
		min_angle_dif = abs(anglepos - 90);
		o = WEST;
		os = "W";
	}
	if (abs(anglepos - 270) < min_angle_dif) {
		min_angle_dif = abs(anglepos - 270);
		o = EAST;
		os = "E";
	}
	if (abs(anglepos - 180) < min_angle_dif) {
		min_angle_dif = abs(anglepos - 180);
		o = SOUTH;
		os = "S";
	}
	return o;
}

Orientation get_orientation_back (int angle) {
	int anglepos;
	if (angle%360 < 0){
		anglepos = angle%360 + 360;
	}
	else {
		anglepos = angle%360;
	}
	Orientation o = NORTH;
	char* os = "U";
	int min_angle_dif = 45;
	if (abs(anglepos) < min_angle_dif) {
		min_angle_dif = abs(anglepos - 0);
		o = EAST;
		os = "E";
	}
	if (abs(anglepos - 90) < min_angle_dif) {
		min_angle_dif = abs(anglepos - 90);
		o = NORTH;
		os = "N";
	}
	if (abs(anglepos - 270) < min_angle_dif) {
		min_angle_dif = abs(anglepos - 270);
		o = SOUTH;
		os = "S";
	}
	if (abs(anglepos - 180) < min_angle_dif) {
		min_angle_dif = abs(anglepos - 180);
		o = WEST;
		os = "W";
	}
	return o;
}
