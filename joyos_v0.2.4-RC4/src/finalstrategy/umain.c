#include <joyos.h>
#include <../src/finalstrategy/globals.h>
#include <../src/finalstrategy/node.h>

uint8_t team_number[2] = {3,0};

#define GYRO_PORT 8
#define LSB_US_PER_DEG 1357.348162*3838.0/3600.0*1028.0/1080.0*1000.0
#define GYRO_CALIB_TIME 5000

Node construct_strategy();

int usetup () {
	set_auto_halt(0);
	return 0;
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
	printf("\nGo plz");
	go_click();
	//Node current_node = construct_strategy();

	
	Node root = root_node();
	Node test0 = test_node(17);
	add_child(&root,&test0);
	Node * current_node = &root;

	printf("\nGo again plz");
	go_click();
	
	while(1) {
		for (uint8_t i = 0; i < NODE_CHILDREN; i++) {
			printf("\n%d",i);
			go_click();
			if (current_node->children[i] != NULL) {
				printf("\n%d",current_node->children[i]);
				go_click();
				printf("\n%d",current_node->children[i]->use_theta);
				go_click();
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

Node construct_strategy() {

	Node root = root_node();

	/*Position p0;
	p0.x = 1;
	p0.y = 1;
	p0.theta = 0;
	Node node0 = travel_node(p0, false);
	
	Position p1;
	p1.x = 0;
	p1.y = 0;
	p1.theta = 0;
	Node node1 = travel_node(p1, false);
	
	add_child(&root,&node0);
	add_child(&node0,&node1);*/
	
//	Node test0 = test_node(17);
/*	Node test1 = test_node(1);
	Node test2 = test_node(2);
	Node test3 = test_node(3);
	Node test4 = test_node(4);*/
	
//	add_child(&root,test0);
/*	add_child(&test0,&test1);
	add_child(&test1,&test2);
	add_child(&test1,&test3);
	add_child(&test3,&test4);*/
	
	return root;
}