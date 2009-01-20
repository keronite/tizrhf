#include <joyos.h>
#include <../src/finalstrategy/globals.h>
#include <../src/finalstrategy/node.h>

uint8_t team_number[2] = {3,0};

#define GYRO_PORT 8
#define LSB_US_PER_DEG 1357.348162*3838.0/3600.0*1028.0/1080.0*1000.0
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
	
	global_position.x = 0;
	global_position.y = 0;
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
					break;
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
	/*Position p0;
	p0.x = 2;
	p0.y = 2;
	p0.theta = 0;
	Node * travel0 = travel_node(p0, true);
	Position p1;
	p1.x = 2;
	p1.y = 3;
	p1.theta = 0;
	Node * travel1 = travel_node(p1, false);*/
	
	Node * travel0 = goal_node();
	
	//Attach nodes
	add_child(root,travel0);
	//add_child(travel0,travel1);
	
	return root;
}

