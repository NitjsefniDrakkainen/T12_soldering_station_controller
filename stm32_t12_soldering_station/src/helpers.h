/*
 * helpers.h
 *
 *  Created on: 23.09.2019
 *      Author: NitjsefniDrakkainen
 */

#ifndef __HELPERS_H__
#define __HELPERS_H__

extern int32_t constrain(int32_t value, int32_t min, int32_t max);
extern int32_t map(int32_t value, int32_t v_min, int32_t v_max, int32_t r_min, int32_t r_max);
extern int32_t exp_mov_average(int32_t *_data, uint8_t coeff, int32_t val);

#endif /* __HELPERS_H__ */
