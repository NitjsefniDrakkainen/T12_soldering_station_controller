/*
 * periph_init.h
 *
 *  Created on: 16.09.2019
 *      Author: NitjsefniDrakkainen
 */

#ifndef __PERIPH_INIT_H__
#define __PERIPH_INIT_H__

#include <stdint.h>

// Minimum calibration temperature in degrees of Celsius
#define TIP_MIN_TEMP_C 180
// Maximum calibration temperature in degrees of Celsius
#define TIP_MAX_TEMP_C 450
// Maximum possible temperature in internal units
#define TIP_MAX_TEMP_ADC  3700


#define IRON_CONNECTED_FLAG								(1 << 0)
#define IRON_ON_FLAG									(1 << 1)
#define IRON_SLEEP_FLAG									(1 << 2)
#define IRON_COOLING_FLAG								(1 << 3)
#define IRON_PID_PI_MODE_FLAG							(1 << 4)

#define STATION_INITED									(1 << 0)

/*
 * external interface declarations
 */

extern void station_init_periph(void);

extern uint8_t station_get_mode(void);
extern uint16_t station_get_update_freq(void);
extern uint8_t station_get_iron_on();
extern uint8_t station_iron_get_sleep();
extern uint8_t station_iron_get_connected();
extern void station_get_disp_values(uint16_t *t, int *hamb, uint8_t *pwr);

extern void station_iron_on_off(uint8_t on);
extern void station_iron_sleep_on_off(uint8_t val);

extern void station_iron_heat_clbk(void);
extern void station_iron_recalc_clbk(void);
extern void station_iron_conn_clbk(void);
extern void station_controll_callback(void);

#endif /* __PERIPH_INIT_H__ */
