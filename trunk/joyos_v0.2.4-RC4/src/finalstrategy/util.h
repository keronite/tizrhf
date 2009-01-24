#ifndef _UTIL_H_
#define _UTIL_H_

#include <../src/finalstrategy/globals.h>
#include <joyos.h>

Position get_ball_position(Ball ball);
float clamp (float val, float min, float max);
int degrees_to_servo_units(int degrees);
int degrees_to_servo_units2(int degrees);
void soft_stop_motors(int duration);
void thrash();
void calibrate_leds();
void hard_brake();
int servo_units_to_degrees(int servo_angle);
int servo_units_to_degrees2(int servo_angle);
int sing();
Position get_line_position(Line line);
uint8_t get_led_readings();

#endif //_UTIL_H_
