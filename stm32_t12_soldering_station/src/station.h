/*
 * periph_init.h
 *
 *  Created on: 16.09.2019
 *      Author: NitjsefniDrakkainen
 */

#ifndef __PERIPH_INIT_H__
#define __PERIPH_INIT_H__

#include <stdint.h>

#define IRON_CONNECTED_FLAG								(1 << 0)
#define IRON_ON_FLAG									(1 << 1)
#define IRON_OPERATE_PHASE								(1 << 2)
#define IRON_WAIT_PHASE									(1 << 3)
#define IRON_CHECK_PHASE								(1 << 4)

#define STATION_INITED									(1 << 0)
#define STATION_CURR_SENSE_REQ							(1 << 1)

#define SELFTEST_MODE									(1 << 7)
/*
 * external interface declarations
 */

extern void station_init_periph(void);
extern uint8_t station_get_adc_channel(uint8_t chan_nr, uint16_t *pValue);
extern uint8_t station_get_adc_channels(uint8_t nChans, uint16_t *pValue);
extern uint8_t station_get_mode(void);
extern uint16_t station_get_update_freq(void);
extern void station_get_disp_values(uint16_t *t, int *hamb);
extern uint8_t station_get_iron_on();
extern void station_iron_on_off(uint8_t on);
extern void station_iron_tip_handler(void);
extern void station_controll_callback(void);

#endif /* __PERIPH_INIT_H__ */
