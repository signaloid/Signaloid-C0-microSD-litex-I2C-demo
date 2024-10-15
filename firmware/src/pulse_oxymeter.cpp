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


#include "pulse_oxymeter.h"
#include "driver_MAX30100.h"
#include "uart.h"
#include "time.h"


MAX30100 pulseOxymeter;

void
pulse_oxymeter_init(void)
{
	pulseOxymeter.init(
		DEFAULT_OPERATING_MODE,
		DEFAULT_SAMPLING_RATE,
		DEFAULT_LED_PULSE_WIDTH,
		DEFAULT_IR_LED_CURRENT,
		true,
		true
	);
}

uint32_t
pulse_oxymeter_update(void)
{
	/*
	 * 	Read the raw data from the MAX30100
	 */
	fifo_t rawData = pulseOxymeter.readFIFO();

	uart_printf("IR: %*d | RED: %*d\n", 5, rawData.rawIR, 5, rawData.rawRed);

	/*
	 * 	Package the raw data to a uint32_t to return
	 */
	return (uint32_t) (rawData.rawIR << 16 | rawData.rawRed);
}