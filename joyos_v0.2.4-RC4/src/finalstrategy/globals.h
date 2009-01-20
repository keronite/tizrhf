#ifndef _GLOBALS_H_
#define _GLOBALS_H_

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