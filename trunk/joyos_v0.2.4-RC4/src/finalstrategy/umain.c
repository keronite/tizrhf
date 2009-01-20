#include <joyos.h>
#include <../src/finalstrategy/globals.h>
#include <../src/finalstrategy/node.h>

uint8_t team_number[2] = {3,0};

#define GYRO_PORT 8
#define LSB_US_PER_DEG 1357.348162*3838.0/3600.0*1028.0/1080.0*1000.0
#define GYRO_CALIB_TIME 5000

int usetup () {
	set_auto_halt(0);
	printf_P (PSTR("\nPress go."));
	go_click ();
	printf_P (PSTR("\nStabilizing"));
	pause (500);
	printf_P (PSTR("\nCalibrating offset."));
	gyro_init (GYRO_PORT, LSB_US_PER_DEG, GYRO_CALIB_TIME);
	return 0;
}

int umain () {
	global_position.x = 0;
	global_position.y = 0;
	Position p;
	p.x = 1;
	p.y = 1;
	p.theta = 0;
	Node node0 = travel_node(p, false);
	attempt(&node0);
	while(1);
	return 0;
}
