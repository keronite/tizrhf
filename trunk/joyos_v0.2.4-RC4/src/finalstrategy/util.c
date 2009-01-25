#include <../src/finalstrategy/util.h>
#include <joyos.h>

Position get_line_position(Line line) {
	Position p;
	p.theta = 0;
	p.x = 0;
	p.y = 0;
	switch(line) {
		case(BOTTOM_LINE):
			p.y = 18;
			break;

		case(TOP_LINE):
			p.y = 78;
			break;

		case(LEFT_LINE):
			p.x = 18;
			break;

		case(RIGHT_LINE):
			p.y = 54;
			break;

		case(FLAGBOX_LINE_TOP):
			p.x = 36;
			break;

		case(FLAGBOX_LINE_BOTTOM):
			p.x = 36;
			break;
	}
	return p;
}

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
			p.x = 49;
			p.y = 18;
			break;
		case (LARGE_BALL0):
			p.x = 25;
			p.y = 60;
			break;
		case (LARGE_BALL1):
			p.x = 48.5;
			p.y = 60;
			break;
		case (LARGE_BALL2):
			p.x = 20.5;
			p.y = 35.5;
			break;
		case (LARGE_BALL3):
			p.x = 48;
			p.y = 37.5;
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

int degrees_to_servo_units2(int degrees) {
	int servo_angle = 511 - (int)(degrees*213.0/90.0)%427;
	if (servo_angle < 0) {
		servo_angle += 512;
	}
	return servo_angle;
}

int servo_units_to_degrees(int servo_angle) {
	int degrees = (int) -(servo_angle*180.0/511.0) + 90;
	return degrees;
}

int servo_units_to_degrees2(int servo_angle) {
	int degrees = (int) (180 - ((servo_angle - 85.0)*180.0/426.0));
	return degrees;
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
		led_filter_matrix[i] = 0;
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

	led_filter_matrix[LEFT_LED_INDEX] = (avg_high_read_l + avg_low_read_l)/2;
	led_filter_matrix[MIDDLE_LED_INDEX] = (avg_high_read_m + avg_low_read_m)/2;
	led_filter_matrix[RIGHT_LED_INDEX] = (avg_high_read_r + avg_low_read_r)/2;
}

void hard_brake() {
	//printf("\nHard brake");
	motor_brake(RIGHT_MOTOR);
	motor_brake(LEFT_MOTOR);
}

int sing () {
	while(1) {
		beep(622,1*BEAT);//1
		pause(1*BEAT);

		beep(622,1*BEAT);//2
		pause(1*BEAT);

		beep(622,2*BEAT);//3
		pause(2*BEAT);

		beep(622,1*BEAT);
		pause(1*BEAT);

		beep(622,1*BEAT);
		pause(1*BEAT);

		beep(622,2*BEAT);//6
		pause(2*BEAT);

		beep(523,4*BEAT);
		pause(4*BEAT);

		beep(523,1*BEAT);
		pause(1*BEAT);

		beep(523,1*BEAT);
		pause(1*BEAT);

		beep(622,2*BEAT);//10
		pause(2*BEAT);

		beep(622,1*BEAT);
		pause(1*BEAT);

		beep(622,1*BEAT);
		pause(1*BEAT);

		beep(622,2*BEAT);
		pause(2*BEAT);

		beep(523,4*BEAT);
		pause(4*BEAT);

		beep(784,2*BEAT); //15
		pause(2*BEAT);

		beep(699,2*BEAT);
		pause(2*BEAT);

		beep(784,4*BEAT);
		pause(4*BEAT);

		beep(622,1*BEAT);//part 2: 1
		pause(1*BEAT);

		beep(622,1*BEAT);//2
		pause(1*BEAT);

		beep(622,2*BEAT);//3
		pause(2*BEAT);

		beep(622,1*BEAT);
		pause(1*BEAT);

		beep(622,1*BEAT);
		pause(1*BEAT);

		beep(622,2*BEAT);//6
		pause(2*BEAT);

		beep(523,4*BEAT);
		pause(4*BEAT);

		beep(784,4*BEAT);
		pause(4*BEAT);

		beep(698,4*BEAT);
		pause(4*BEAT);

		beep(622,4*BEAT);
		pause(4*BEAT);

		beep(698,6*BEAT);
		pause(6*BEAT);

		beep(698,1*BEAT);//part 3: 1
		pause(1*BEAT);

		beep(698,1*BEAT);//2
		pause(1*BEAT);

		beep(698,2*BEAT);//3
		pause(2*BEAT);

		beep(698,1*BEAT);
		pause(1*BEAT);

		beep(698,1*BEAT);
		pause(1*BEAT);

		beep(698,2*BEAT);//6
		pause(2*BEAT);

		beep(587,4*BEAT);
		pause(4*BEAT);

		beep(784,4*BEAT);
		pause(4*BEAT);

		beep(698,4*BEAT);
		pause(4*BEAT);

		beep(622,4*BEAT);
		pause(4*BEAT);

		beep(587,4*BEAT);
		pause(6*BEAT);

		beep(523, 4*BEAT);
		pause(4*BEAT);

		beep(523, 2*BEAT);
		pause(2*BEAT);

		beep(784, 2*BEAT);
		pause(2*BEAT);

		beep(923, 2*BEAT);
		pause(2*BEAT);

		beep(880, 6*BEAT);
		pause(6*BEAT);

		beep(523, 4*BEAT);
		pause(4*BEAT);

		beep(523, 2*BEAT);
		pause(2*BEAT);

		beep(784, 2*BEAT);
		pause(2*BEAT);

		beep(923, 2*BEAT);
		pause(2*BEAT);

		beep(880, 4*BEAT);
		pause(4*BEAT);

		beep(1175, 2*BEAT);
		pause(2*BEAT);

		beep(1245, 4*BEAT);
		pause(4*BEAT);
	}

	return 0;
}

uint8_t get_led_readings() {

	uint16_t calibration_r = led_filter_matrix[RIGHT_LED_INDEX];
	uint16_t sample_r = analog_read(RIGHT_LED);

	uint16_t calibration_m = led_filter_matrix[MIDDLE_LED_INDEX];
	uint16_t sample_m = analog_read(MIDDLE_LED);

	uint16_t calibration_l = led_filter_matrix[LEFT_LED_INDEX];
	uint16_t sample_l = analog_read(LEFT_LED);

	uint8_t reading = 0;

	if (sample_r > calibration_r) {
		reading += 1;
	}
	if (sample_m > calibration_m) {
		reading += 2;
	}
	if (sample_l > calibration_l) {
		reading += 4;
	}

	return reading; //00000LMR, 1 if black, 0 if white
}
