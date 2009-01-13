#include <joyos.h>

#define FORWARD_SPEED 128
#define BACKWARD_SPEED 128
#define TURNING_SPEED 64

//Motor convention, 0 is right, 1 is left
#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1

#define GYRO_PORT 8


double calibrate_gyro();

// usetup is called during the calibration period. It must return before the
// period ends.
int usetup (void) {
	set_auto_halt(0);
	return 0;
}

int umain (void) {

	calibrate_gyro();
	
	while(1) {
	
		double angle = 180;
		
		go_click();
		
		motor_set_vel(RIGHT_MOTOR, -TURNING_SPEED);
		motor_set_vel(LEFT_MOTOR, TURNING_SPEED);
		
		while (angle < 270) {
			uint32_t time = get_time();
			printf("\n%f", angle);
			//go_click();
			uint32_t new_time = get_time();
			angle += (double)((analog_read(GYRO_PORT)-505.370)*(int16_t)(new_time - time))/1000.0*0.678674081;
			if (angle > 360) {
				angle -= 360;
			} else if (angle < 0) {
				angle += 360;
			}
			time = new_time;
			//printf("\nsoi");
			//pause(50);
			motor_set_vel(RIGHT_MOTOR, angle - 270 - 30);
			motor_set_vel(LEFT_MOTOR, 270 - angle + 30);
		}
		
		motor_set_vel(RIGHT_MOTOR, 0);
		motor_set_vel(LEFT_MOTOR, 0);
	}
	return 0;
}

double calibrate_gyro() {

	double avg_read = 0;
	uint16_t samples = 0;
	while(!stop_press()) {
		uint16_t sample = analog_read(GYRO_PORT);
		samples++;
		avg_read = (avg_read*(samples - 1) + sample)/(double)samples;
		printf("\n%f", avg_read);
		pause(50);
	}
	return avg_read;
}