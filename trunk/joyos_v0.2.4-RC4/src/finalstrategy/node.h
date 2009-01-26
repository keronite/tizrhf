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
	Node* children [NODE_CHILDREN];
};

Node * travel_node(float x, float y, float theta, uint8_t use_theta);
Node * flag_node();
Node * acquire_node(Ball ball);
Node * goal_node();
Node * line_node(Line line);
Node * root_node();
Node * test_node(uint8_t id);
Node * find_line_node(Line line);
Node * corner_orient_node();
Node * posn_node_front(float angle, uint8_t use_theta);
Node * posn_node_back(float angle, uint8_t use_theta);
void add_child(Node* parent, Node* child);

Status attempt(Node* node);

#endif //_NODE_H_
