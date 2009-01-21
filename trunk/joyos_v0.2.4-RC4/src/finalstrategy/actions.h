#ifndef _ACTIONS_H_
#define _ACTIONS_H_

#include <../src/finalstrategy/node.h>


Status travel_to(Node* node);
Status dump_balls(Node* node);
Status attempt_orient(Node* node);
Status line_follow(Node * node);

#endif //_ACTIONS_H_