#include <joyos.h>

#define LENGTH 6.0
#define WIDTH 8.0
#define RAD_TO_DEG 57.2957795

struct position {float x; float y; float theta;};

void find_path(struct position *pi, struct position *pt);
void_face_goal(struct position *pi, struct position *pt);
float get_turn_angle(float start, float goal);

int usetup (void) {
	set_auto_halt(0);
	return 0;
}

void umain(void) {
	struct position init_pos, *ip, goal_pos, *gp;
	// just testing initial and goal values
	ip = &init_pos;
	gp = &goal_pos;
	init_pos.x = 0.5;
	init_pos.y = 0.5;
	init_pos.theta = 20.0;
	goal_pos.x = 2.0;
	goal_pos.y = 3.0;
	goal_pos.theta = 0.0;
	find_path(ip, gp);
}

void find_path(struct position *pi, struct position *pt) {
	face_goal(pi, pt);
	//printf("\n%f %f %f", pi->x, pi->y, pi->theta);

}

void face_goal(struct position *pi, struct position *pt) {
	float init_angle = pi->theta;
	float dist = sqrt(pow((pi->x - pt->x), 2)+pow((pi->y - pt-> y), 2));
	float end_angle = acos((pt->y - pi->y)/dist);
	//printf("\noriginal pos %d %d", pi->x, pi->y);
	printf("\nTurn angle is %f", get_turn_angle(init_angle, end_angle)*RAD_TO_DEG);
	//printf("\n%f %f %f", pi->x, pi->y, pi->theta);
}

float get_turn_angle(float start, float goal) {
	float turn_angle;
	float right_diff;
	if (start <= goal) {
		right_diff = goal - start;
	}
	else {
		right_diff = 360.0 - (start - goal);
	}
	if (right_diff > 180.0) {
		return right_diff - 360.0;
	}
	else return right_diff;
}