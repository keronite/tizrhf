#include <joyos.h>
#include <happylib.h>

uint8_t team_number[2] = {3,0};

#define FORWARD_SPEED 64
#define BACKWARD_SPEED 128
#define TURNING_SPEED 64

//Motor convention, 0 is right, 1 is left
#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1

#define FRONT_SERVO 0

enum state_enum {SETUP, STOP, LOOK_Y} state;

void setup_state();
void stop_state();
void look_y_state();

void setup_filter();
void stop_filter();
void look_y_filter();

void soft_stop_motors(int duration);

int degrees_to_servo_units(int degrees);

float clamp (float val, float min, float max);

uint16_t x, y;

#define SHARP_M		13569
#define SHARP_C		4

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
			case (LOOK_Y):
				look_y_state();
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
		x = 0;
		y = 0;
		irdist_set_calibration (SHARP_M, SHARP_C);
		
		printf("\nPress go");
		go_click();
		printf("\nStabilizing");
		pause(1000);
		
		printf("\nInitializing");
		gyro_init(8,1357.348162*3838.0/3600.0*1028.0/1080.0*1000.0,5000);
		
		setup_filter();
	}
}

void stop_state() {
	soft_stop_motors(50);
	while(state == STOP) {
		printf("\nx=%d, y=%d", x, y);
		pause(50);
		stop_filter();
	}
}

void look_y_state() {
	while(state == LOOK_Y) {
		pause(50);
		
		int angle = (int)gyro_get_degrees();
		
		printf("\n%d  %d", angle, angle%360);
		
		servo_set_pos(FRONT_SERVO, degrees_to_servo_units(-angle));
		pause(1000);
		x = irdist_read(23);
		
		servo_set_pos(FRONT_SERVO, degrees_to_servo_units(-angle - 90));
		pause(1000);
		y = irdist_read(23);
		
		look_y_filter();
	}
}

void setup_filter() {
	state = LOOK_Y;
}


void stop_filter() {
	if (go_press()) {
		state = LOOK_Y;
	}
}

void look_y_filter() {
	state = STOP;
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

int degrees_to_servo_units(int degrees) {
	int servo_angle = (int)((degrees*256.0/90.0) + 255)%512;
	if (servo_angle < 0) {
		servo_angle += 512;
	}
	return servo_angle;
}