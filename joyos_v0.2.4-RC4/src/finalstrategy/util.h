#ifndef _UTIL_H_
#define _UTIL_H_

#include <../src/finalstrategy/globals.h>

Position get_ball_position(Ball ball);
float clamp (float val, float min, float max);
int degrees_to_servo_units(int degrees);

#endif //_UTIL_H_