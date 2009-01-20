#ifndef _GLOBALS_H_
#define _GLOBALS_H_

//Motor convention, 0 is right, 1 is left.
#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1

//Shaft encoder convention, 24 is right, 25 is left.
#define RIGHT_ENCODER 24
#define LEFT_ENCODER 25

//PID control constants.
#define KP 1.5
#define KD 0
#define KI .05

//Motor speeds
#define FORWARD_SPEED 164
#define BACKWARD_SPEED 128
#define TURNING_SPEED 164

typedef struct {
	float x;
	float y;
	float theta;
} Position;

typedef enum {
	BOTTOM_LINE,
	TOP_LINE,
	LEFT_LINE,
	RIGHT_LINE
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

#endif //_GLOBALS_H_
