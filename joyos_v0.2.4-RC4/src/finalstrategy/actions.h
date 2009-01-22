#ifndef _ACTIONS_H_
#define _ACTIONS_H_

#include <../src/finalstrategy/node.h>


Status travel_to(Node* node);
Status dump_balls(Node* node);
Status attempt_orient(Node* node);
Status line_follow(Node * node);
Status flagbox(Node * node);
Status line_search(Node * node);
Status get_abs_pos(Node * node);
Status acquire_ball(Node * node);

#endif //_ACTIONS_H_