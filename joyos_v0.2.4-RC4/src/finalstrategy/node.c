#include <../src/finalstrategy/node.h>
#include <../src/finalstrategy/actions.h>

Status attempt(Node* node) {
	return node->_attempt(node);
}

/*Status travel_to(Node* node) {
	printf("\nHEY IT WORKED");
	return SUCCESS;
}*/

Node travel_node(Position destination, uint8_t use_theta) {
	Node node;
	node._attempt = travel_to;
	node.position = destination;
	node.use_theta = use_theta;
//	node.children = Node* [NODE_CHILDREN];
	return node;
}