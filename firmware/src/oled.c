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


#include <stdbool.h>
#include <string.h>
#include "oled.h"
#include "i2c.h"
#include "driver_ssd1306_basic.h"
#include "signaloid_logo.h"
#include "str_utils.h"

typedef enum OLED_CONF_enum
{
	kOLED_CONF_DISPLAY_WIDTH    = 128,
	kOLED_CONF_DISPLAY_HEIGHT   = 32,
	kOLED_CONF_DRAW_COLOR	    = 1,
	kOLED_CONF_DRAW_FONT	    = SSD1306_FONT_12,
	kOLED_CONF_DRAW_FONT_HEIGHT = 12,
	kOLED_CONF_DRAW_FONT_WIDTH  = 6,
	kOLED_CONF_MARGIN	    = 2,
} OLED_CONF;

void
oled_clear(void)
{
	ssd1306_basic_clear();
}

void
oled_init(void)
{
	ssd1306_basic_init(SSD1306_INTERFACE_IIC, SSD1306_ADDR_SA0_0);
	ssd1306_basic_display_on();
	ssd1306_basic_clear();
}

void
oled_draw_signaloid_logo(void)
{
	ssd1306_basic_picture(
		0,
		((kOLED_CONF_DISPLAY_HEIGHT - kSIGNALOID_LOGO_HEIGHT) / 2),
		0 + kSIGNALOID_LOGO_WIDTH - 1,
		((kOLED_CONF_DISPLAY_HEIGHT - kSIGNALOID_LOGO_HEIGHT) / 2) + kSIGNALOID_LOGO_HEIGHT - 1,
		(uint8_t *) SIGNALOID_LOGO_INVERTED_IMG
	);
}

uint8_t current_x = 0;
uint8_t current_y = 0;

void
oled_greeting(void)
{
	oled_draw_signaloid_logo();

	current_x = kSIGNALOID_LOGO_WIDTH + 4 * kOLED_CONF_MARGIN;
	current_y = ((kOLED_CONF_DISPLAY_HEIGHT / 2) - kOLED_CONF_MARGIN) - kOLED_CONF_DRAW_FONT_HEIGHT;
	ssd1306_basic_string(current_x, current_y, "Signaloid", 9, kOLED_CONF_DRAW_COLOR, (ssd1306_font_t) kOLED_CONF_DRAW_FONT);

	current_y = kOLED_CONF_DISPLAY_HEIGHT / 2;
	ssd1306_basic_rect(current_x, current_y, kOLED_CONF_DISPLAY_WIDTH - 1, current_y, kOLED_CONF_DRAW_COLOR);

	current_y = ((kOLED_CONF_DISPLAY_HEIGHT / 2) + kOLED_CONF_MARGIN);
	ssd1306_basic_string(current_x, current_y, "C0-microSD", 10, kOLED_CONF_DRAW_COLOR, (ssd1306_font_t) kOLED_CONF_DRAW_FONT);

	ssd1306_basic_update();
}

void
oled_setup_measurements(void)
{
	oled_draw_signaloid_logo();

	current_x = kSIGNALOID_LOGO_WIDTH + 4 * kOLED_CONF_MARGIN;
	ssd1306_basic_rect(current_x, 0, current_x, kOLED_CONF_DISPLAY_HEIGHT - 1, kOLED_CONF_DRAW_COLOR);
	current_x += 4 * kOLED_CONF_MARGIN;
}

void
oled_update(uint16_t ir, uint16_t red)
{
	char buf[32];

	current_y = ((kOLED_CONF_DISPLAY_HEIGHT / 2) - kOLED_CONF_MARGIN) - kOLED_CONF_DRAW_FONT_HEIGHT;
	str_utils_format(buf, "IR : %*d", 5, ir);
	ssd1306_basic_string(current_x, current_y, buf, 10, kOLED_CONF_DRAW_COLOR, (ssd1306_font_t) kOLED_CONF_DRAW_FONT);

	current_y = ((kOLED_CONF_DISPLAY_HEIGHT / 2) + kOLED_CONF_MARGIN);
	str_utils_format(buf, "RED: %*d", 5, red);
	ssd1306_basic_string(current_x, current_y, buf, 10, kOLED_CONF_DRAW_COLOR, (ssd1306_font_t) kOLED_CONF_DRAW_FONT);

	ssd1306_basic_update();
}
