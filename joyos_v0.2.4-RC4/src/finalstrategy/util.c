#include <../src/finalstrategy/util.h>
#include <joyos.h>


Position get_ball_position(Ball ball) {
	Position p;
	p.theta = 0;
	p.x = 0;
	p.y = 0;
	switch(ball) {
		case (SMALL_BALL0):
			p.x = 24;
			p.y = 78;
			break;
		case (SMALL_BALL1):
			p.x = 48;
			p.y = 78;
			break;
		case (SMALL_BALL2):
			p.x = 18;
			p.y = 60;
			break;
		case (SMALL_BALL3):
			p.x = 54;
			p.y = 60;
			break;
		case (SMALL_BALL4):
			p.x = 6;
			p.y = 54;
			break;
		case (SMALL_BALL5):
			p.x = 66;
			p.y = 54;
			break;
		case (SMALL_BALL6):
			p.x = 18;
			p.y = 48;
			break;
		case (SMALL_BALL7):
			p.x = 54;
			p.y = 48;
			break;
		case (SMALL_BALL8):
			p.x = 6;
			p.y = 42;
			break;
		case (SMALL_BALL9):
			p.x = 66;
			p.y = 42;
			break;
		case (SMALL_BALL10):
			p.x = 18;
			p.y = 36;
			break;
		case (SMALL_BALL11):
			p.x = 54;
			p.y = 36;
			break;
		case (SMALL_BALL12):
			p.x = 24;
			p.y = 18;
			break;
		case (SMALL_BALL13):
			p.x = 48;
			p.y = 18;
			break;
		case (LARGE_BALL0):
			p.x = 24;
			p.y = 60;
			break;
		case (LARGE_BALL1):
			p.x = 48;
			p.y = 60;
			break;
		case (LARGE_BALL2):
			p.x = 24;
			p.y = 36;
			break;
		case (LARGE_BALL3):
			p.x = 48;
			p.y = 36;
			break;
	}
	
	return p;
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

void soft_stop_motors(int duration) {
	motor_set_vel(RIGHT_MOTOR, 0);
	motor_set_vel(LEFT_MOTOR, 0);
	pause(duration);
}

void thrash() {
	motor_set_vel(RIGHT_MOTOR,255);
	motor_set_vel(LEFT_MOTOR,255);
	
	motor_set_vel(FLAG_MOTOR,255);
	
	pause(500);
	motor_set_vel(RIGHT_MOTOR,-255);
	motor_set_vel(LEFT_MOTOR,255);
	pause(500);
	motor_set_vel(FLAG_MOTOR,-255);
	motor_set_vel(RIGHT_MOTOR,255);
	motor_set_vel(LEFT_MOTOR,-255);
	pause(500);
	motor_set_vel(RIGHT_MOTOR,-255);
	motor_set_vel(LEFT_MOTOR,-255);
	pause(500);
	motor_set_vel(FLAG_MOTOR,0);
	soft_stop_motors(1);
}

void calibrate_leds() {
	
	for (int i = 0; i < NUM_LEDS; i++) {
		for (int j = 0; j < LED_RESERVED_INDICES + NUM_LED_SAMPLES; j++) {
			led_filter_matrix[i][j] = 0;
		}
	}
	
	uint16_t samples;
	
	printf("\nPlace LEDs on light surface, then press go");
	
	go_click();
	
	float avg_low_read_l = 0;
	float avg_low_read_m = 0;
	float avg_low_read_r = 0;
	
	samples = 0;
	
	while(!stop_press()) {
		uint16_t sample_l = analog_read(LEFT_LED);
		uint16_t sample_m = analog_read(MIDDLE_LED);
		uint16_t sample_r = analog_read(RIGHT_LED);
		
		samples++;
		
		avg_low_read_l = (avg_low_read_l*(samples - 1) + sample_l)/(float)samples;
		avg_low_read_m = (avg_low_read_m*(samples - 1) + sample_m)/(float)samples;
		avg_low_read_r = (avg_low_read_r*(samples - 1) + sample_r)/(float)samples;
		
		printf("\nL:%d  M:%d  R:%d", (int)avg_low_read_l, (int)avg_low_read_m, (int)avg_low_read_r);
		pause(50);
	}
	
	printf("\nPlace LEDs on dark surface, then press go");
	
	go_click();
	
	float avg_high_read_l = 0;
	float avg_high_read_m = 0;
	float avg_high_read_r = 0;
	
	samples = 0;
	
	while(!stop_press()) {
		uint16_t sample_l = analog_read(LEFT_LED);
		uint16_t sample_m = analog_read(MIDDLE_LED);
		uint16_t sample_r = analog_read(RIGHT_LED);
		
		samples++;
		
		avg_high_read_l = (avg_high_read_l*(samples - 1) + sample_l)/(float)samples;
		avg_high_read_m = (avg_high_read_m*(samples - 1) + sample_m)/(float)samples;
		avg_high_read_r = (avg_high_read_r*(samples - 1) + sample_r)/(float)samples;
		
		printf("\nL:%d  M:%d  R:%d", (int)avg_high_read_l, (int)avg_high_read_m, (int)avg_high_read_r);
		pause(50);
	}
	
	led_filter_matrix[LEFT_LED_INDEX][LED_CALIBRATION_INDEX] = (avg_high_read_l + avg_low_read_l)/2;
	led_filter_matrix[MIDDLE_LED_INDEX][LED_CALIBRATION_INDEX] = (avg_high_read_m + avg_low_read_m)/2;
	led_filter_matrix[RIGHT_LED_INDEX][LED_CALIBRATION_INDEX] = (avg_high_read_r + avg_low_read_r)/2;
}
