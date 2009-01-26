#include <joyos.h>
#include <../src/finalstrategy/globals.h>
#include <../src/finalstrategy/node.h>
#include <../src/finalstrategy/util.h>
#include <../src/finalstrategy/actions.h>
#include <happylib.h>

uint8_t team_number[2] = {3,0};

#define GYRO_PORT 8
#define LSB_US_PER_DEG 1357.348162*3838.0/3600.0*1028.0/1080.0*1000.0*387.0/360.0*341.0/360.0*727.0/720.0
#define GYRO_CALIB_TIME 5000

Node* create_tree();

int usetup () {
	set_auto_halt(0);
	printf_P (PSTR("\nPress go."));
	go_click ();
	printf_P (PSTR("\nStabilizing"));
	pause (500);
	printf_P (PSTR("\nCalibrating offset."));
	gyro_init (GYRO_PORT, LSB_US_PER_DEG, GYRO_CALIB_TIME);

	irdist_set_calibration (SHARP_M, SHARP_C);

	calibrate_leds();

	global_position.x = 8;
	global_position.y = 6.5;
	return 0;
}

int umain () {

	//Start at the root
	Node * current_node = create_tree();

	//Traverses the node tree.  Each node is a task.
	//The children are in prioritized order.
	//If it succeeds, we move on.
	//If it fails, we try the lower priority nodes
	while(1) {
		for (uint8_t i = 0; i < NODE_CHILDREN; i++) {
			if (current_node->children[i] != NULL) {
				Status s = attempt(current_node->children[i]);
				if (s == SUCCESS) {
					current_node = current_node->children[i];
					//printf("\nx=%f, y=%f",global_position.x,global_position.y);
					//go_click();
					break;
				} else {
					//printf("\nFAILURE umain");
					//go_click();
					drive(-6,1);
					Node * node;
					node = posn_node();
					servo_set_pos(JAW_SERVO,JAW_CLOSED);
					servo_set_pos(LIFT_SERVO, LIFT_LOWER);
					uint8_t bump = 0;
					while(1) {
						pause(5);
						bump += digital_read(LIFT_BUMP);
						if (bump >= 10) {
							servo_set_pos(LIFT_SERVO, LIFT_BOTTOM);
							break;
						}
					}
					node->_attempt(node);
				}
			}
			//if last number here, thrash
		}
	}
	while(1);
	return 0;
}

Node * create_tree() {

	//Create nodes
	Node * root = root_node();
	Node * corner = corner_orient_node();
	Node * grab = acquire_node(LARGE_BALL2);
	Node * grab2 = acquire_node(LARGE_BALL3);
	Node * goal = goal_defend_node();
	Node * posn = posn_node();
	Node * grab3 = acquire_node(LARGE_BALL0);
	Node * grab4 = acquire_node(LARGE_BALL1);
	Node * posn2 = posn_node();
	Node * goal2 = goal_node();
	Node * goal3 = goal_node();
	Node * posn3 = posn_node();
	Node * position_for_flag = travel_node(54,60,90,true);
	Node * line_find = find_line_node(FLAGBOX_LINE_TOP);
	Node * flag = flag_node();
	Node * travel0 = travel_node(18,56,0,false);
	Node * travel1 = travel_node(18,18,0,false);

	//Attach nodes
	add_child(root,corner);
	//add_child(corner,grab);
	//add_child(grab,grab2);

	add_child(corner,grab2);

	add_child(grab2,goal);
	add_child(goal,posn);
	add_child(posn,grab);
	add_child(grab,grab3);
	add_child(grab3,line_find);
	add_child(line_find,flag);
	add_child(flag,travel0);
	add_child(travel0,travel1);
	add_child(travel1,goal2);
	add_child(goal2,posn2);
	add_child(posn2,grab4);
	add_child(grab4,goal3);

	//Node * lol = posn_node();
	//add_child(root, lol);


	return root;
}

