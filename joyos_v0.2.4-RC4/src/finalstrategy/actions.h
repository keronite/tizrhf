#ifndef _ACTIONS_H_
#define _ACTIONS_H_

#include <../src/finalstrategy/node.h>


Status travel_to(Node* node);
Status dump_balls(Node* node);
Status dump_defend(Node* node);
Status attempt_orient(Node* node);
Status line_follow(Node * node);
Status flagbox(Node * node);
Status line_search(Node * node);
Status get_pos_back(Node * node);
Status acquire_ball(Node * node);
Status acquire_ball_fast(Node * node);
Status get_pos_front(Node* node);
Status drive(float distance, float speed_scale);
Status sharp_pos(Node* node);
Status turn(float angle, bool short_turn);

#endif //_ACTIONS_H_
