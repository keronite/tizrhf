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


#define ENCODER_TO_WHEEL_RATIO 15
#define WHEEL_CIRCUMFERENCE 25.76
#define WHEEL_TRACK 21.5

#define SQUARE_LENGTH_CM 91.44

#include <lib/geartrain.h>


enum state_enum {SETUP, STOP, LINE_FOLLOW} state;

void setup_state();
void stop_state();
void line_follow_state();

void setup_filter();
void stop_filter();
void line_follow_filter();

void soft_stop_motors(int duration);

bool filter_led(uint8_t led_port);

void calibrate_leds();

int sing();

float clamp (float val, float min, float max);


uint16_t left_encoder_base, right_encoder_base;

uint8_t led_reading;

#define NUM_LEDS 3
#define NUM_LED_SAMPLES 5

#define RIGHT_LED 23
#define MIDDLE_LED 22
#define LEFT_LED 21

#define RIGHT_LED_INDEX 0
#define MIDDLE_LED_INDEX 1
#define LEFT_LED_INDEX 2

#define LED_RESERVED_INDICES 2
#define LED_OFFSET_INDEX 0
#define LED_CALIBRATION_INDEX 1

#define LINE_OFFSET_WEAK 5
#define LINE_OFFSET_STRONG 10

uint16_t led_filter_matrix[NUM_LEDS][NUM_LED_SAMPLES + LED_RESERVED_INDICES];

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
			case (LINE_FOLLOW):
				line_follow_state();
				break;
				
			case (STOP):
				stop_state();
				break;
		 }
	
	}

	

	return 0;
}

bool filter_led(uint8_t led_port) {
	uint8_t index;
	switch (led_port)
	{
		case (RIGHT_LED):
			index = RIGHT_LED_INDEX;
			break;
	
	    case (MIDDLE_LED):
			index = MIDDLE_LED_INDEX;
			break;
	
	    case (LEFT_LED):
			index = LEFT_LED_INDEX;
			break;
	
	    default:
			index = MIDDLE_LED_INDEX;
			break;
	}
	
	
	uint8_t offset = (uint8_t)(led_filter_matrix[index][LED_OFFSET_INDEX]);
	if (offset < 2) {offset = 2;}
	uint16_t calibration = led_filter_matrix[index][LED_CALIBRATION_INDEX];
	uint16_t sample = analog_read(led_port);
	uint16_t current_reading = (sample > calibration); //1 if black, 0 if white
	led_filter_matrix[index][offset] = current_reading; //Put the new sample in the array
	offset = (offset + 1 - LED_RESERVED_INDICES)%(NUM_LED_SAMPLES) + LED_RESERVED_INDICES; //Increment offset and keep it in range
	led_filter_matrix[index][LED_OFFSET_INDEX] = offset; //Store the new offset
	
	uint8_t sum = 0; //Count true samples
	for(uint8_t i = LED_RESERVED_INDICES; i < LED_RESERVED_INDICES + NUM_LED_SAMPLES; i++) {
		if (led_filter_matrix[index][i] != 0) {
			sum++;
		}
	}
	
	if (sum > NUM_LED_SAMPLES/2) {
		return true;
	} else {
		return false;
	}
	
}

void setup_state() {
	while (state == SETUP) {
		calibrate_leds();
		setup_filter();
	}
}

void stop_state() {
	soft_stop_motors(50);
	while(state == STOP) {
		pause(50);
		stop_filter();
	}
}

void line_follow_state() {
	led_reading = 0;
	while(state == LINE_FOLLOW) {
		//MSB->LSB, left, middle, right
		  switch (led_reading)
		  {		
		    case (1): //001
		      motor_set_vel(LEFT_MOTOR, FORWARD_SPEED + LINE_OFFSET_STRONG);
		      motor_set_vel(RIGHT_MOTOR, FORWARD_SPEED - LINE_OFFSET_STRONG);
		      break;
		
		    case (2): //010
		      motor_set_vel(LEFT_MOTOR, FORWARD_SPEED);
		      motor_set_vel(RIGHT_MOTOR, FORWARD_SPEED);
		      break;
		
			case (3): //011
		      motor_set_vel(LEFT_MOTOR, FORWARD_SPEED + LINE_OFFSET_WEAK);
		      motor_set_vel(RIGHT_MOTOR, FORWARD_SPEED - LINE_OFFSET_WEAK);
		      break;
		
		    case (4): //100
		      motor_set_vel(LEFT_MOTOR, FORWARD_SPEED - LINE_OFFSET_STRONG);
		      motor_set_vel(RIGHT_MOTOR, FORWARD_SPEED + LINE_OFFSET_STRONG);
		      break;
		
		    case (6): //110
		      motor_set_vel(LEFT_MOTOR, FORWARD_SPEED - LINE_OFFSET_WEAK);
		      motor_set_vel(RIGHT_MOTOR, FORWARD_SPEED + LINE_OFFSET_WEAK);
		      break;
		  }
		  
		pause(10);
		
		line_follow_filter();
	}
}

void setup_filter() {
	if (stop_press()) {
		state = LINE_FOLLOW;
	}
}


void stop_filter() {
	if (go_press()) {
		state = LINE_FOLLOW;
	}
}

void line_follow_filter() {
	bool left = filter_led(LEFT_LED);
	bool middle = filter_led(MIDDLE_LED);
	bool right = filter_led(RIGHT_LED);
	led_reading = (left << 2) + (middle << 1) + (right << 0);
	if (right) {
		printf("\nBLACK");
	} else {
		printf("\nWHITE");
	}
	//Add stop conditions if gyro and shaft suggest might be done
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