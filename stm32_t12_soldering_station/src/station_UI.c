/*
 * station_UI.c
 *
 *  Created on: 22.09.2019
 *      Author: NitjsefniDrakkainen
 */

#include "stm32f10x.h"
#include "defs.h"
#include "station.h"
#include "ssd1306_driver.h"
#include "ssd1306_fonts.h"
#include "m_snprintf.h"
#include "station_UI.h"

static volatile uint16_t update_freq_cnt = 0, button_cnt = 0, encoder_flags = 0;
static char line_buffer[64];

void station_button_callback(void)
{
	if (GPIO_ReadInputDataBit(ENC_C_PORT, ENCODER_PIN_C) == RESET) {
		encoder_flags = 1;

	} else {
		debug_print(USART1, "release\r\n");
		encoder_flags |= 1 << 1;
	}
}


void station_ui_input_handler(void)
{
	if (encoder_flags & (1 << 0)) {

		if ( (++button_cnt > 1000) && !(encoder_flags & (1 << 1)) ) {
			button_cnt = 0;
			encoder_flags = 0;
			debug_print(USART1, "longpress\r\n");
		} else if ( (++button_cnt <  1000) && (encoder_flags & (1 << 1)) ) {
			button_cnt = 0;
			encoder_flags = 0;
			if (!station_get_iron_on()) {
				debug_print(USART1, "iron ON\r\n");
				station_iron_on_off(1);
			} else {
				station_iron_on_off(0);
				debug_print(USART1, "iron OFF\r\n");
			}

		}
	}
}

void station_ui_callback(void)
{
	uint8_t mode = station_get_mode(), pwr;
	uint16_t temp;
	int handle_amb;
	if (mode & STATION_INITED) {
		if (++update_freq_cnt > station_get_update_freq()) {
			station_get_disp_values(&temp, &handle_amb, &pwr);
			ssd1306_SetCursor(1 , 0);
			if (station_iron_get_sleep()) {
				m_snprintf(line_buffer, 60, "set: %03d    %s", 273, "SLEEP");
			} else {
				m_snprintf(line_buffer, 60, "set: %03d      %s", 273, station_get_iron_on() ? "ON " : "OFF");
			}
			ssd1306_WriteString(line_buffer, Font_7x10, White);

			ssd1306_SetCursor(1 , 20);
			if ( station_iron_get_connected() ) {
				m_snprintf(line_buffer, 60, "  %03d", temp);
			} else {
				m_snprintf(line_buffer, 60, "  %s", "No Tip");
			}
			ssd1306_WriteString(line_buffer, Font_11x18, White);

			ssd1306_SetCursor(1 , 50);
			m_snprintf(line_buffer, 60, "P: %03d     amb: %02d", pwr, handle_amb);
			ssd1306_WriteString(line_buffer, Font_7x10, White);
			ssd1306_UpdateScreen();
			update_freq_cnt = 0;
		}

	}
}
