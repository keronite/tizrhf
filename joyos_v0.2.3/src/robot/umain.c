/*
 * The MIT License
 *
 * Copyright (c) 2007 MIT 6.270 Robotics Competition
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

// Include headers from OS
#include <joyos.h>

#include <gyro.h>

// usetup is called during the calibration period. It must return before the
// period ends.
int usetup (void) {
	set_auto_halt(0);
	return 0;
}

// Entry point to contestant code.
// Create threads and return 0.
int umain (void) {
	// Loop forever
	printf("\nPress go");
	go_click();
	printf("\nWait");
	pause(1000);
	printf("\nCalibrating");
	
	gyro_init(8,1357.348162*3838.0/3600.0,10000); //*3600/3450*3600/3663*3600/3730
	motor_set_vel(0, 64);
	motor_set_vel(1, -64);
	while (1) {
		printf("\n%f", (double)gyro_get_degrees());
		pause(50);
		if (gyro_get_degrees() >= 3600) {
			motor_set_vel(0,0);
			motor_set_vel(1,0);
			while(1) {
				printf("\n%f", (double)gyro_get_degrees());
				pause(50);
			}
		}
	}

	// Will never return, but the compiler complains without a return
	// statement.
	return 0;
}

