#include <../src/finalstrategy/node.h>
#include <../src/finalstrategy/actions.h>
#include <../src/finalstrategy/util.h>
#include <../src/finalstrategy/globals.h>
#include <happylib.h>
#include <stdlib.h>

Status test_attempt(Node * node);

Status attempt_orient(Node * node);

Status attempt(Node* node) {
	return node->_attempt(node);
}


Node * travel_node(Position destination, uint8_t use_theta) {
	Node * node = (Node*)malloc(sizeof(Node));
	node->_attempt = travel_to;
	node->position = destination;
	node->use_theta = use_theta;
	for (uint8_t i = 0; i < NODE_CHILDREN; i++) {
		node->children[i] = NULL;
	}
	return node;
}

Node * root_node() {
	Node * node = (Node*)malloc(sizeof(Node));
	for (uint8_t i = 0; i < NODE_CHILDREN; i++) {
		node->children[i] = NULL;
	}
	return node;
}

Node * corner_orient_node() {
	Node * node = (Node*)malloc(sizeof(Node));
	node->_attempt = attempt_orient;
	for (uint8_t i = 0; i < NODE_CHILDREN; i++) {
		node->children[i] = NULL;
	}
	return node;
}

Node * goal_node() {
	Node * node = (Node*)malloc(sizeof(Node));
	node->_attempt = dump_balls;
	for (uint8_t i = 0; i < NODE_CHILDREN; i++) {
		node->children[i] = NULL;
	}
	return node;
}

Node * test_node(uint8_t id) {
	Node * node = (Node*)malloc(sizeof(Node));
	node->_attempt = test_attempt;
	node->use_theta = id;
	for (uint8_t i = 0; i < NODE_CHILDREN; i++) {
		node->children[i] = NULL;
	}
	return node;
}

void add_child(Node* parent, Node* child) {
	for (uint8_t i = 0; i < NODE_CHILDREN; i++) {
		if (parent->children[i] == NULL) {
			parent->children[i] = child;
			return;
		}
	}
}

Status test_attempt(Node * node) {
	printf("\nTEST: %d", node->use_theta);
	pause(500);
	while(1) {
		if (go_press()) {
			return SUCCESS;
		}
		else if (stop_press()) {
			return FAILURE;
		}
	}
}

Status attempt_orient(Node * node) {
		
	servo_set_pos(FRONT_SERVO, degrees_to_servo_units(0));
	pause(1000);
	uint16_t wall_front_dist = irdist_read(FRONT_SHARP);
	bool wall_front = false;
	if (wall_front_dist < 30) {
		wall_front = true;
	}
		
	servo_set_pos(FRONT_SERVO, degrees_to_servo_units(90));
	pause(1000);
	uint16_t wall_right_dist = irdist_read(FRONT_SHARP);
	bool wall_right = false;
	if (wall_right_dist < 30) {
		wall_right = true;
	}
		
	if (wall_front && wall_right) {
		gyro_set_degrees(180);
	} else if (wall_front && !wall_right) {
		gyro_set_degrees(-90);
	} else if (!wall_front && wall_right) {
		gyro_set_degrees(90);
	} else {
		gyro_set_degrees(0);
	}
	return SUCCESS;
}