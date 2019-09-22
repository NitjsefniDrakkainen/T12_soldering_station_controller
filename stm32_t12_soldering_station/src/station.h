/*
 * periph_init.h
 *
 *  Created on: 16.09.2019
 *      Author: NitjsefniDrakkainen
 */

#ifndef __PERIPH_INIT_H__
#define __PERIPH_INIT_H__

#include <stdint.h>

#define MODE_IDLE								(1u)
#define MODE_IDLE								(1u)

/*
 * external interface declarations
 */

extern void station_init_periph(void);
extern uint8_t station_get_adc_channel(uint8_t chan_nr, uint16_t *pValue);
extern uint8_t station_get_adc_channels(uint8_t nChans, uint16_t *pValue);
extern uint8_t station_get_mode(void);

extern void station_iron_tip_handler(void);

#endif /* __PERIPH_INIT_H__ */
