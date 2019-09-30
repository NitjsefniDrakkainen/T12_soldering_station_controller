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

#define DISPLAY_MODE_NORMAL

static volatile uint16_t update_freq_cnt = 0, button_cnt = 0, encoder_flags = 0, x = 40;
static char line_buffer[64];

static void _display_operate_mode_normal(void)
{
	uint16_t temp, preset, curr;
	uint8_t pwr;
	int handle_amb;
	station_get_disp_values(&temp, &preset, &handle_amb, &pwr, &curr);

	ssd1306_SetCursor(1 , 5);
	if (station_iron_get_sleep()) {
		m_snprintf(line_buffer, 60, "set: %03d    %s", preset, "SLEEP");
	} else {
		m_snprintf(line_buffer, 60, "set: %03d      %s", preset, station_get_iron_on() ? "ON " : "OFF");
	}
	ssd1306_WriteString(line_buffer, Font_7x10, White);

	ssd1306_SetCursor(1 , 20);
	if ( station_iron_get_connected() ) {
		m_snprintf(line_buffer, 60, "  %03d", temp);
	} else {
		m_snprintf(line_buffer, 60, "  %s", "No Tip");
	}
	ssd1306_WriteString(line_buffer, Font_11x18, White);

	ssd1306_SetCursor(1 , 40);
		m_snprintf(line_buffer, 60, "I: %01d.%02d     ", curr/100, curr%100);
		ssd1306_WriteString(line_buffer, Font_7x10, White);

	ssd1306_SetCursor(1 , 53);
	m_snprintf(line_buffer, 60, "P: %03d     amb: %02d", pwr, handle_amb);
	ssd1306_WriteString(line_buffer, Font_7x10, White);
}

static void _display_operate_mode_simple(void)
{
	uint16_t temp, preset, curr;
	uint8_t pwr;
	int handle_amb;
	station_get_disp_values(&temp, &preset, &handle_amb, &pwr, &curr);

	ssd1306_SetCursor(1 , 1);

	ssd1306_WriteString(line_buffer, Font_7x10, White);

	ssd1306_SetCursor(1 , 30);
	if ( station_iron_get_connected() ) {
		m_snprintf(line_buffer, 60, "  %03d / %03d", temp, preset);
	} else {
		m_snprintf(line_buffer, 60, "  %s", "No Tip");
	}
	ssd1306_WriteString(line_buffer, Font_11x18, White);

	ssd1306_SetCursor(1 , 55);
	if (station_iron_get_sleep()) {
		m_snprintf(line_buffer, 60, "       %s",  "SLEEP");
	} else {
		m_snprintf(line_buffer, 60, "         %s", station_get_iron_on() ? "ON " : "OFF");
	}
}

static void _display_operate_mode_detailed(void)
{

}

static void _display_operate_mode_handler(uint8_t dm)
{
	switch (dm) {
		case 0:
		case 1:
			_display_operate_mode_normal();
			break;
		case 2:
			_display_operate_mode_simple();
		case 3:
			_display_operate_mode_detailed();
		default:
			break;
	}
}

void station_ui_rotary_callback(void)
{
	uint8_t mode = station_get_mode();
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == SET) {

		if (mode & STATION_MODE_OPERATE) {
			station_iron_temp_adjust(-5);
		}

	} else {

		if (mode & STATION_MODE_OPERATE) {
			station_iron_temp_adjust(5);
		}

	}

}

static void _display_menu_mode_handler(uint8_t menu_level)
{

}

static void _display_handler(uint8_t _mode)
{
	uint8_t disp_mode = STATION_GET_DISPLAY_MODE(_mode);
	if (_mode & STATION_MODE_OPERATE) {
		_display_operate_mode_handler(disp_mode);
	} else {
		_display_menu_mode_handler(1);
	}
}


void station_button_callback(void)
{
	if (GPIO_ReadInputDataBit(ENC_C_PORT, ENCODER_PIN_C) == RESET) {
		encoder_flags = 1;

	} else {
		//debug_print(USART1, "release\r\n");
		encoder_flags |= 1 << 1;
	}
}


void station_ui_input_handler(void)
{
	if (encoder_flags & (1 << 0)) {

		if ( (++button_cnt > 1000) && !(encoder_flags & (1 << 1)) ) {
			button_cnt = 0;
			encoder_flags = 0;
			//debug_print(USART1, "longpress\r\n");
		} else if ( (++button_cnt <  1000) && (encoder_flags & (1 << 1)) ) {
			button_cnt = 0;
			encoder_flags = 0;
			if (!station_get_iron_on()) {
				//debug_print(USART1, "iron ON\r\n");
				station_iron_on_off(1);
			} else {
				station_iron_on_off(0);
				//debug_print(USART1, "iron OFF\r\n");
			}

		}
	}
}


void station_ui_callback(void)
{
	uint8_t mode = station_get_mode();

	if (mode & STATION_INITED) {
		if (++update_freq_cnt > station_get_update_freq()) {

			_display_handler(mode);

			ssd1306_WriteString(line_buffer, Font_7x10, White);
			ssd1306_UpdateScreen();
			update_freq_cnt = 0;
		}

	}
}
