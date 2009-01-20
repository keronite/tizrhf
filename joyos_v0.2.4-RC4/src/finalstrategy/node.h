#ifndef _NODE_H_
#define _NODE_H_

#include <../src/finalstrategy/globals.h>
#include <joyos.h>

#define NODE_CHILDREN 4

typedef struct Node Node;

struct Node {
	Status (*_attempt)(Node* node);
	Position position;
	Ball ball;
	Line line;
	uint8_t use_theta;
	//struct Node* children [NODE_CHILDREN];
};

Node travel_node(Position destination, uint8_t use_theta);
Node flag_node();
Node acquire_node(Ball ball);
Node goal_node();
Node line_node(Line line);

Status attempt(Node* node);

#endif //_NODE_H_