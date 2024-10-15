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


#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum SSD1306_CONFIG_enum
{
	kSSD1306_CONFIG_ADDR = 0x3C,
} SSD1306_CONFIG_t;

typedef enum SSD1306_CMD_enum
{
	kSSD1306_CMD_SETCONTRAST		   = 0x81,
	kSSD1306_CMD_SET_ENTIRE_DISPLAY_FOLLOW_RAM = 0xA4,
	kSSD1306_CMD_SET_ENTIRE_DISPLAY_ON	   = 0xA5,
	kSSD1306_CMD_SET_NORMAL_DISPLAY		   = 0xA6,
	kSSD1306_CMD_SET_INVERSE_DISPLAY	   = 0xA7,
	kSSD1306_CMD_SET_DISPLAY_OFF		   = 0xAE,
	kSSD1306_CMD_SET_DISPLAY_ON		   = 0xAF,
} SSD1306_CMD_t;

/**
 * 	@brief Clears the display.
 */
void oled_clear(void);


/**
 * 	@brief Initializes the OLED display.
 */
void oled_init(void);


/**
 *	@brief Displays the Signaloid greeting on the OLED display.
 */
void oled_greeting(void);

/**
 * 	@brief Sets up the OLED display graphics for the measurements.
 */
void oled_setup_measurements(void);


/**
 * 	@brief Updates the OLED display with the given IR and red values.
 *
 * 	@param ir
 * 	@param red
 */
void oled_update(uint16_t ir, uint16_t red);

#ifdef __cplusplus
}
#endif

#endif
