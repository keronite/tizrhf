#include <joyos.h>

#define FORWARD_SPEED 128
#define BACKWARD_SPEED 128
#define TURNING_SPEED 64

//Motor convention, 0 is right, 1 is left
#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1

//Digital ports
#define BUMP_SENSOR 0

//Prototypes
uint8_t drive_forward();
uint8_t drive_backward();
uint8_t turn_right();
uint8_t turn_left();
uint8_t soft_brake();
void hard_brake();
uint8_t bump_sensor_hit();

enum state_enum {STOP, FORWARD, BACKWARD, TURN};
enum state_enum state = STOP;
enum state_enum new_state = STOP;


// usetup is called during the calibration period. It must return before the
// period ends.
int usetup (void) {
	set_auto_halt(0);
	return 0;
}

int umain (void) {
//	create_thread(stop_button_thread, 32, 127, "stop_thread");
//	create_thread(go_button_thread, 32, 127, "go_thread");
	
	while(1) {
		switch (state) {
			case (STOP):
			  soft_brake(10, &stop_press);
			  go_click();
			  new_state = FORWARD;
			  break;
		
			case (FORWARD):
			  soft_brake(100, &stop_press);
			  drive_forward(0);
			  while (1) {
				if (stop_press()) {
					new_state = STOP;
					break;
				}
				else if (bump_sensor_hit()) {
					new_state = BACKWARD;
					break;
				}
			  }
			  break;
		
			case (BACKWARD):
			  if (soft_brake(100, &stop_press) && drive_backward(500, &stop_press))
				new_state = TURN;
			  else
				new_state = STOP;
			  break;
		
			case (TURN):
			  if (soft_brake(100, &stop_press) && turn_right(750, &stop_press))
				new_state = FORWARD;
			  else
				new_state = STOP;
			  break;
		}
		
		state = new_state;
	}
	return 0;
}

uint8_t drive_forward(uint16_t duration, uint8_t(*stop_condition)() ){
	printf("\nDriving forward");
	motor_set_vel(RIGHT_MOTOR,FORWARD_SPEED);
	motor_set_vel(LEFT_MOTOR,FORWARD_SPEED);
	uint32_t stop_time = get_time() + duration;
	while((get_time() < stop_time)) {
		if (stop_condition()) {
			return 0;
		}
	}
	return 1;
}

uint8_t drive_backward(uint16_t duration, uint8_t(*stop_condition)() ) {
	printf("\nDriving backward");
	motor_set_vel(RIGHT_MOTOR,-BACKWARD_SPEED);
	motor_set_vel(LEFT_MOTOR,-BACKWARD_SPEED);
	uint32_t stop_time = get_time() + duration;
	while((get_time() < stop_time)) {
		if (stop_condition()) {
			return 0;
		}
	}
	return 1;
}

uint8_t turn_left(uint16_t duration, uint8_t(*stop_condition)() ) {
	printf("\nTurning left");
	motor_set_vel(RIGHT_MOTOR,TURNING_SPEED);
	motor_set_vel(LEFT_MOTOR,-TURNING_SPEED);
	uint32_t stop_time = get_time() + duration;
	while((get_time() < stop_time)) {
		if (stop_condition()) {
			return 0;
		}
	}
	return 1;
}

uint8_t turn_right(uint16_t duration, uint8_t(*stop_condition)() ) {
	printf("\nTurning right");
	motor_set_vel(RIGHT_MOTOR,-TURNING_SPEED);
	motor_set_vel(LEFT_MOTOR,TURNING_SPEED);
	uint32_t stop_time = get_time() + duration;
	while((get_time() < stop_time)) {
		if (stop_condition()) {
			return 0;
		}
	}
	return 1;
}

uint8_t soft_brake(uint16_t duration, uint8_t(*stop_condition)() ) {
	printf("\nSoft brake");
	motor_set_vel(RIGHT_MOTOR,0);
	motor_set_vel(LEFT_MOTOR,0);
	uint32_t stop_time = get_time() + duration;
	while((get_time() < stop_time)) {
		if (stop_condition()) {
			return 0;
		}
	}
	return 1;
}

void hard_brake() {
	printf("\nHard brake");
	motor_brake(RIGHT_MOTOR);
	motor_brake(LEFT_MOTOR);
}


uint8_t bump_sensor_hit() {
	return digital_read(BUMP_SENSOR);
}
