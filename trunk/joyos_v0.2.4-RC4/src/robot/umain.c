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

// usetup is called during the calibration period. It must return before the
// period ends.
int usetup (void) {
	return 0;
}

// Entry point to contestant code.
// Create threads and return 0.
int
umain (void) {
	// Loop forever
	while (1) {
		// Clear LCD (with \n) and print ROBOTS at top left
		printf("\nROBOTS");
		// Pause for 200 ms
		pause(200);
		// Clear LCD and print ROBOTS at bottom right
		printf("\n                          ROBOTS");
		// Pause for 200 ms
		pause(200);
		// Clear LCD and print ROBOTS at top right
		printf("\n          ROBOTS");
		// Pause for 200 ms
		pause(200);
		// Clear LCD and print ROBOTS at bottom left
		printf("\n                ROBOTS");
		// Pause for 200 ms
		pause(200);
	}

	// Will never return, but the compiler complains without a return
	// statement.
	return 0;
}

