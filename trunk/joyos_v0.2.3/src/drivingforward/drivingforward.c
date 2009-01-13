#include <joyos.h>

#define FORWARD_SPEED 128
#define BACKWARD_SPEED 128
#define TURNING_SPEED 96

//Motor convention, 0 is right, 1 is left
#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1

//Shaft encoder convention, 24 is right, 25 is left
#define RIGHT_ENCODER 24
#define LEFT_ENCODER 25

#define kp .5//1.5
#define ln_kd 1
#define ki .05


// usetup is called during the calibration period. It must return before the
// period ends.
int usetup (void) {
	set_auto_halt(0);
	return 0;
}

int umain (void) {
	double prev_vel_r = 0;
	double prev_vel_l = 0;
	int acc = 0;
	int target_speed = 0;
	encoder_reset(RIGHT_ENCODER);
	encoder_reset(LEFT_ENCODER);
	
	while(1)
	{
	double new_vel;
	double delta = 0;
	int dif = encoder_read(RIGHT_ENCODER) - encoder_read(LEFT_ENCODER);
	
	acc += dif;
	
	delta = (target_speed - kp*dif - prev_vel_r);
	new_vel = prev_vel_r + delta*ln_kd - acc*ki;
	motor_set_vel(RIGHT_MOTOR, (int)new_vel);
	prev_vel_r = new_vel;
	
	//printf("\n%d   ", (int)new_vel);
	
	delta = (target_speed + kp*dif - prev_vel_l);
	new_vel = prev_vel_l + delta*ln_kd + acc*ki;
	motor_set_vel(LEFT_MOTOR, (int)new_vel);
	prev_vel_l = new_vel;
	
	//printf("%d  ", (int)new_vel);
	
	//printf("%d %c", dif, 1);
	
	if (target_speed < FORWARD_SPEED) {
		target_speed += 16;
	}
	
	if (stop_press()) {
		motor_set_vel(LEFT_MOTOR, 0);
		motor_set_vel(RIGHT_MOTOR, 0);
		target_speed = 0;
		prev_vel_r = 0;
		prev_vel_l = 0;
		acc = 0;
		encoder_reset(RIGHT_ENCODER);
		encoder_reset(LEFT_ENCODER);
		printf("\n%d %d %d %d", encoder_read(RIGHT_ENCODER), encoder_read(LEFT_ENCODER), encoder_read(26), encoder_read(27));
		go_click();
	}
	
	pause(100);
	/*
		if(prev_vel_r <=120)
		{
			new_vel = prev_vel_r + (FORWARD_SPEED - prev_vel_r)*0.2 + dif;
			motor_set_vel(RIGHT_MOTOR, new_vel);
			prev_vel_r = new_vel - dif;
			printf("\nDiff is %d %c", dif, 1);
			pause(50);
		}
		else
		{
			new_vel = prev_vel_r + dif;
			motor_set_vel(RIGHT_MOTOR, new_vel);
			prev_vel_r = new_vel - dif;
			printf("\nDiff is %d %c", dif, 1);
			pause(50);
		}
		if(prev_vel_l <=120)
		{
			
			new_vel = prev_vel_l + (FORWARD_SPEED - prev_vel_l)*0.2 - dif;
			motor_set_vel(LEFT_MOTOR, new_vel);
			prev_vel_l = new_vel + dif;
			pause(50);
		}
		else
		{
			new_vel = prev_vel_l - dif;
			motor_set_vel(LEFT_MOTOR, new_vel);
			prev_vel_l = new_vel + dif;
			pause(50);
		}
	*/
		
	}
	return 0;
}
