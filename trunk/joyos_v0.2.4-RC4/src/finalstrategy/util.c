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
