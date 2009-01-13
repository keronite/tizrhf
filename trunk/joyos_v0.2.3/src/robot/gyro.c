
#include <gyro.h>

void update(struct gyro *gyro) {
	uint32_t time = get_time();
	
	while (1) {
		uint32_t new_time = get_time();
		gyro->angle += (double)((analog_read(GYRO_PORT)-gyro->offset)*(int16_t)(new_time - time))/1000.0*gyro->angular_vel_scale;
		if (gyro->angle > 360) {
			gyro->angle -= 360;
		} else if (gyro->angle < 0) {
			gyro->angle += 360;
		}
		time = new_time;
	}
	
}

void init_gyro (struct gyro *gyro, uint8_t port, bool calibrate) {
	gyro->port = port;
	gyro->angular_vel_scale = DEFAULT_GYRO_VEL_SCALE;
	gyro->offset = DEFAULT_GYRO_OFFSET;
	gyro->is_calibrating = calibrate;
	gyro->angle = 0;
}

float get_angle (struct gyro *gyro) {
	return gyro->angle;
}

void calibrate_gyro (struct gyro *gyro) {
	gyro->is_calibrating = 0;
	float avg_read = 0;
	uint16_t samples = 0;
	while(gyro->is_calibrating) {
		uint16_t sample = analog_read(GYRO_PORT);
		samples++;
		avg_read = (avg_read*(samples - 1) + sample)/(float)samples;
		pause(50);
	}
	gyro->offset = avg_read;
	gyro->angle = 0;
	update(gyro)
}

void stop_calibrate_gyro (struct gyro *gyro);