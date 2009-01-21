#ifndef _ACTIONS_H_
#define _ACTIONS_H_

#include <../src/finalstrategy/globals.h>
#include <../src/finalstrategy/node.h>
#include <../src/finalstrategy/util.h>
#include <joyos.h>


Status travel_to(Node* node);
Status dump_balls(Node* node);
Status attempt_orient(Node* node);

#endif //_ACTIONS_H_