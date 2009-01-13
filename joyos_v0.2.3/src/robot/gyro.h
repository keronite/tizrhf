#ifndef __INCLUDE_GYRO_H__
#define __INCLUDE_GYRO_H__

#include <joyos.h>

/**
 * \file gyro.h
 * \brief Class which interfaces with the gyroscope.  After initialization, the
 * angle of the gyroscope is found by integrating its output, which takes the form
 * of the robot's angular velocity.
 *
 * The gyro runs in a separate thread, and primary interaction should be querying the
 * class for the angle.
 *
 * For most accurate results, the calibration routine should be run before hand, if it
 * is known that the robot is stationary.  Default values are available, though.
 *
 * While we try to prevent it, there may be drift over time.  Best used for single turns
 */

#define DEFAULT_GYRO_OFFSET 505.370
#define DEFAULT_GYRO_VEL_SCALE 0.678674081

/**
 * Gyro structure
 */
struct gyro {
    // calibrated values
	float offset;                 ///< Rest value of the gyro
	float angular_vel_scale;	  ///< Ratio of (deg/sec)/(units received from gyro)

	// angle
	float angle;				  ///< Current heading
	
	// port
	u8int_t port;				  ///< Analog port to which the gyro is connected
	
	// calibration
	bool is_calibrating;		  ///< Are we in a calibration state?
};

/** 
 * Initialize a Gyro structure. This function takes an empty
 * gyro structure and initializes its member variables.
 * Starts a new thread
 *
 * @param gyro      empty gyro to initialize
 * @param port      gyro port number
 * @param calibrate do we calibrate?  If false, use default values
 * */
void init_gyro (struct gyro *gyro, uint8_t port, bool calibrate);

/**
 * Returns the current heading
 *
 * @param gyro		gyro from which to get the angle
 */
float get_angle (struct gyro *gyro);

/**
 * Calibrates the gyro.  It should remain stationary over this time
 *
 * @param gyro      gyro to calibrate
 */
void calibrate_gyro (struct gyro *gyro);

/**
 * Stops calibrating the gyro.  Angle is initialized to zero after this method's termination
 *
 * @param gyro      gyro to stop calibrate
 */
void stop_calibrate_gyro (struct gyro *gyro);


#endif // __INCLUDE_GYRO_H__
