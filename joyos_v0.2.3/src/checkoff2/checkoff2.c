#include <joyos.h>

#define FORWARD_SPEED 128
#define BACKWARD_SPEED 128
#define TURNING_SPEED 64

//Motor convention, 0 is right, 1 is left
#define RIGHT 0
#define LEFT 1

//Prototypes
void drive_forward();
void drive_backward();
void turn_right();
void turn_left();
void soft_brake();
void hard_brake();
int stop_button_thread();
int go_button_thread();

enum state_enum {STOP, GO};

enum state_enum state = STOP;

// usetup is called during the calibration period. It must return before the
// period ends.
int usetup (void) {
	set_auto_halt(0);
	return 0;
}

int umain (void) {
	create_thread(stop_button_thread, 32, 127, "stop_thread");
	create_thread(go_button_thread, 32, 127, "go_thread");
	
	printf("\nSTOPPED");
	
	while(1) {
		drive_forward(2000);
		soft_brake(500);
		
		turn_right(500);
		soft_brake(500);
		
		drive_forward(2000);
		soft_brake(500);
		
		drive_backward(2000);
		soft_brake(500);
		
		turn_left(1500);
		soft_brake(500);
		
		drive_forward(2000);
		soft_brake(500);
		
		turn_right(1000);
		soft_brake(500);
	}
	return 0;
}

void drive_forward(uint16_t duration) {
	if (state == STOP) {
		pause(duration);
		return;
	} else {
		printf("\nDriving forward");
		motor_set_vel(RIGHT,FORWARD_SPEED);
		motor_set_vel(LEFT,FORWARD_SPEED);
		pause(duration);
	}
}

void drive_backward(uint16_t duration) {
	if (state == STOP) {
		pause(duration);
		return;
	} else {
		printf("\nDriving backward");
		motor_set_vel(RIGHT,-BACKWARD_SPEED);
		motor_set_vel(LEFT,-BACKWARD_SPEED);
		pause(duration);
	}
}

void turn_left(uint16_t duration) {
	if (state == STOP) {
		pause(duration);
		return;
	} else {
		printf("\nTurning left");
		motor_set_vel(RIGHT,TURNING_SPEED);
		motor_set_vel(LEFT,-TURNING_SPEED);
		pause(duration);
	}
}

void turn_right(uint16_t duration) {
	if (state == STOP) {
		pause(duration);
		return;
	} else {
		printf("\nTurning right");
		motor_set_vel(RIGHT,-TURNING_SPEED);
		motor_set_vel(LEFT,TURNING_SPEED);
		pause(duration);
	}
}

void soft_brake(uint16_t duration) {
	if (state == STOP) {
		pause(duration);
		return;
	} else {
		printf("\nSoft brake");
		motor_set_vel(RIGHT,0);
		motor_set_vel(LEFT,0);
		pause(duration);
	}
}

void hard_brake(uint16_t duration) {
	printf("\nHard brake");
	motor_brake(RIGHT);
	motor_brake(LEFT);
	pause(duration);
}

int stop_button_thread() {
	while(1) {
		stop_click();
		hard_brake(100);
		state = STOP;
		printf("\nSTOPPED");
	}
	return 0;
}

int go_button_thread() {
	while(1) {
		go_click();
		state = GO;
	}
	return 0;
}