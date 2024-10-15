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

#include <generated/csr.h>
#include "time.h"
#include "uart.h"
#include "leds.h"
#include "i2c.h"
#include "oled.h"
#include "pulse_oxymeter.h"

/*
 *	Parameter Definitions
 */
typedef enum
{
	kAppConfigSetupWaitDurationMs	 = 2000,
	kAppConfigLedUpdatePeriodMs	 = 500,
	kAppConfigMax30100UpdatePeriodMs = 10,
	kAppConfigOledUpdatePeriodMs	 = 300,
} AppConfig;

/**
 * 	@brief Interrupt Service Routine
 *
 * 	Handles all interrupts
 *
 * 	* Currently does nothing, because we do not implement interrupts.
 */
void
isr(void)
{
	;
}

/**
 * 	@brief The setup function
 * 	This is called once, before the main loop.
 * 	It is responsible to configure peripherals.
 */
static void
setup(void)
{
	uart_printf("\033c"); /* Clear screen */
	uart_printf("Signaloid C0-microSD I2C Litex example\n\n");
	uart_printf("Setup...\n");

	uart_printf("- Init Timer 0: ");
	timer0_init();
	uart_printf("OK\n");

	uart_printf("- Init LEDs: ");
	leds_init();
	uart_printf("OK\n");

	uart_printf("- Init I2C: ");
	i2c_init();
	uart_printf("OK\n");

	uart_printf("- Scanning I2C devices:\n");
	for (uint16_t i = 0; i <= 0x7f; i++)
	{
		if (!i2c_scan(i))
		{
			continue;
		}
		uart_printf("*    0x%x\n", i);
	}

	uart_printf("- Init OLED: ");
	oled_init();
	uart_printf("OK\n");
	oled_greeting();

	uart_printf("- Init Pulse Oxymeter: ");
	pulse_oxymeter_init();
	uart_printf("OK\n");

	uart_printf("Setup successful.\n\n");

	timer0_delay_ms(kAppConfigSetupWaitDurationMs);
	timer0_set_periodic_mode_ticks(UINT32_MAX - 1);

	oled_clear();
	oled_setup_measurements();
}

uint32_t rawData		   = 0;
timer0_t leds_last_update_time	   = 0;
timer0_t max30100_last_update_time = 0;
timer0_t oled_last_update_time	   = 0;

/**
 * 	@brief The main loop.
 * 	This is called infinitely, and is only interrupted by interrupts handled by
 * 	the Interrupt Service Routine (ISR).
 */
static void
loop(void)
{
	/*
	 * 	Toggle Green LED every 500ms
	 */
	timer0_t current_time = timer0_get_time_passed_since_last_load();
	if (timer0_get_duration_ms(leds_last_update_time, current_time) > kAppConfigLedUpdatePeriodMs)
	{
		if(leds_green_get())
		{
			leds_green_off();
			uart_printf("LED Green: Off\n");
		}
		else
		{
			leds_green_on();
			uart_printf("LED Green: On\n");
		}
		leds_last_update_time = timer0_get_time_passed_since_last_load();
	}

	/*
	 * 	Update Max30100 every 10ms
	 */
	if (timer0_get_duration_ms(max30100_last_update_time, current_time) > kAppConfigMax30100UpdatePeriodMs)
	{
		rawData			  = pulse_oxymeter_update();
		max30100_last_update_time = timer0_get_time_passed_since_last_load();
	}

	/*
	 * 	Update display every 300ms
	 */
	if (timer0_get_duration_ms(oled_last_update_time, current_time) > kAppConfigOledUpdatePeriodMs)
	{
		uint16_t ir  = rawData >> 16;
		uint16_t red = rawData & 0xffff;
		oled_update(ir, red);
		oled_last_update_time = timer0_get_time_passed_since_last_load();
	}
}

/**
 * 	@brief The main entry point
 */
int
main(void)
{
	setup();

	while (1)
	{
		loop();
	}

	return 0;
}
