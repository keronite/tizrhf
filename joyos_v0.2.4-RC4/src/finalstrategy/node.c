#include <../src/finalstrategy/node.h>
#include <../src/finalstrategy/actions.h>
#include <../src/finalstrategy/util.h>
#include <../src/finalstrategy/globals.h>
#include <happylib.h>
#include <stdlib.h>

Status test_attempt(Node * node);

Status attempt(Node* node) {
	return node->_attempt(node);
}


Node * travel_node(float x, float y, float theta, uint8_t use_theta) {
	Node * node = (Node*)malloc(sizeof(Node));
	node->_attempt = travel_to;
	Position destination;
	destination.x = x;
	destination.y = y;
	destination.theta = theta;
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

Node * posn_node_front(float angle, uint8_t use_theta) {
	Node * node = (Node*)malloc(sizeof(Node));
	Position p;
	p.theta = angle;
	node->use_theta = use_theta;
	node->position = p;
	node->_attempt = get_pos_front;
	for (uint8_t i = 0; i < NODE_CHILDREN; i++) {
		node->children[i] = NULL;
	}
	return node;
}

Node * posn_node_back() {
	Node * node = (Node*)malloc(sizeof(Node));
	node->_attempt = get_pos_back;
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

Node * flag_node() {
	Node * node = (Node*)malloc(sizeof(Node));
	node->_attempt = flagbox;
	for (uint8_t i = 0; i < NODE_CHILDREN; i++) {
		node->children[i] = NULL;
	}
	return node;
}

Node * find_line_node(Line line) {
	Node * node = (Node*)malloc(sizeof(Node));
	node->_attempt = line_search;
	node->line = line;
	for (uint8_t i = 0; i < NODE_CHILDREN; i++) {
		node->children[i] = NULL;
	}
	return node;
}

Node * acquire_node(Ball ball) {
	Node * node = (Node*)malloc(sizeof(Node));
	node->_attempt = acquire_ball;
	node->ball = ball;
	for (uint8_t i = 0; i < NODE_CHILDREN; i++) {
		node->children[i] = NULL;
	}
	return node;
}
