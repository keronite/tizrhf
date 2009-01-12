#include <joyos.h>

// usetup is called during the calibration period. It must return before the
// period ends.
int usetup (void) {
	set_auto_halt(0);
	return 0;
}

int
umain (void) {
	while(1) {
		printf("\nHit go to activate motors");
		
		//Stop Motors
		motor_set_vel(0,0);
		motor_set_vel(1,0);
		
		go_click();
		
		printf("\nHit stop to stop motors");
		
		//Set motor velocity from frob
		uint16_t pos;
		pos = frob_read()/2;
		motor_set_vel(0,pos);
		motor_set_vel(1,pos);
		
		stop_click();
		
	}
	return 0;
}