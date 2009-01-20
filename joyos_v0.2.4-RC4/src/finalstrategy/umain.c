#include <joyos.h>

#define GYRO_PORT 8
#define LSB_US_PER_DEG 1357.348162*3838.0/3600.0*1028.0/1080.0*1000.0
#define GYRO_CALIB_TIME 5000

uint8_t team_number[2] = {3,0};

void usetup {
	printf_P (PSTR("\nPress go."));
	go_click ();
	printf_P (PSTR("\nStabilizing"));
	pause (500);
	printf_P (PSTR("\nCalibrating offset."));
	gyro_init (GYRO_PORT, LSB_US_PER_DEG, GYRO_CALIB_TIME);
	return 0;
}

void umain() {

}
