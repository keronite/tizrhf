#include <../src/finalstrategy/node.h>
#include <../src/finalstrategy/actions.h>

Status test_attempt(Node * node);

Status attempt(Node* node) {
	return node->_attempt(node);
}


Node travel_node(Position destination, uint8_t use_theta) {
	Node node;
	node._attempt = travel_to;
	node.position = destination;
	node.use_theta = use_theta;
	for (uint8_t i = 0; i < NODE_CHILDREN; i++) {
		node.children[i] = NULL;
	}
	return node;
}

Node root_node() {
	Node node;
	for (uint8_t i = 0; i < NODE_CHILDREN; i++) {
		node.children[i] = NULL;
	}
	return node;
}

Node test_node(uint8_t id) {
	Node node;
	node._attempt = test_attempt;
	node.use_theta = id;
	for (uint8_t i = 0; i < NODE_CHILDREN; i++) {
		node.children[i] = NULL;
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