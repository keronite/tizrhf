#ifndef _GLOBALS_H_
#define _GLOBALS_H_

#include <joyos.h>

#define BOARD_X 72.0
#define BOARD_Y 96.0

//Motor convention, 0 is right, 1 is left.
#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1
#define FLAG_MOTOR 2

//Shaft encoder convention, 24 is right, 25 is left.
#define RIGHT_ENCODER 24
#define LEFT_ENCODER 25

//Digital bump sensor readings
#define TOP_BUMP 0
#define BOTTOM_BUMP 1

//PID control constants.
#define KP 1.75
#define KD 0
#define KI .4

//Motor speeds
#define FORWARD_SPEED 164
#define BACKWARD_SPEED 64
#define TURNING_SPEED 164

//Sharp distance sensor
#define SHARP_M 13569
#define SHARP_C 4
#define FRONT_SHARP 23

//Servo ports
#define LIFT_SERVO 0
#define FRONT_SERVO 1
#define BACK_SERVO 2
#define JAW_SERVO 3

//Dumping mechanism constants
#define SERVO_POS 511
#define SERVO_POS2 500
#define BACK_UP_DIST -6.0
#define DUMPING_SPEED 256

//Numeric constants
#define RAD_TO_DEG 57.2957795

//Geartrain
#define ENCODER_TO_WHEEL_RATIO 15
#define WHEEL_CIRCUMFERENCE 25.76
#define WHEEL_TRACK 21.5
#include <lib/geartrain.h>

//LEDs
#define NUM_LEDS 3
#define RIGHT_LED 22
#define MIDDLE_LED 21
#define LEFT_LED 20
#define RIGHT_LED_INDEX 0
#define MIDDLE_LED_INDEX 1
#define LEFT_LED_INDEX 2
#define LINE_OFFSET_WEAK 5
#define LINE_OFFSET_STRONG 10

#define BEAT 100

typedef struct {
	float x;
	float y;
	float theta;
} Position;

typedef enum {
	BOTTOM_LINE,
	TOP_LINE,
	LEFT_LINE,
	RIGHT_LINE,
	FLAGBOX_LINE_TOP,
	FLAGBOX_LINE_BOTTOM,
} Line;

typedef enum {
	SMALL_BALL0,
	SMALL_BALL1,
	SMALL_BALL2,
	SMALL_BALL3,
	SMALL_BALL4,
	SMALL_BALL5,
	SMALL_BALL6,
	SMALL_BALL7,
	SMALL_BALL8,
	SMALL_BALL9,
	SMALL_BALL10,
	SMALL_BALL11,
	SMALL_BALL12,
	SMALL_BALL13,
	LARGE_BALL0,
	LARGE_BALL1,
	LARGE_BALL2,
	LARGE_BALL3
} Ball;

typedef enum {
	SUCCESS,
	FAILURE
} Status;

Position global_position;

uint16_t led_filter_matrix[NUM_LEDS];

#endif //_GLOBALS_H_
