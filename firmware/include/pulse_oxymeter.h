/*
 *	Copyright (c) 2024, Signaloid.
 *
 *	Permission is hereby granted, free of charge, to any person obtaining a copy
 *	of this software and associated documentation files (the "Software"), to deal
 *	in the Software without restriction, including without limitation the rights
 *	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *	copies of the Software, and to permit persons to whom the Software is
 *	furnished to do so, subject to the following conditions:
 *
 *	The above copyright notice and this permission notice shall be included in all
 *	copies or substantial portions of the Software.
 *
 *	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *	SOFTWARE.
 */


#ifndef __PULSE_OXYMETER_H
#define __PULSE_OXYMETER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * 	@brief Initializes the Pulse Oxymeter.
 */
void pulse_oxymeter_init(void);


/**
 * 	@brief Reads the Pulse Oxymeter raw data, and returns them.
 *
 * 	@return uint32_t the raw data, upper 16 bits are the IR, lower 16 bits are the RED.
 */
uint32_t pulse_oxymeter_update(void);

#ifdef __cplusplus
}
#endif

#endif
