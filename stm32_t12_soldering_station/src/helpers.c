/*
 * helpers.c
 *
 *  Created on: 23.09.2019
 *      Author: NitjsefniDrakkainen
 */

#include <stdint.h>
#include "helpers.h"

int32_t constrain(int32_t value, int32_t min, int32_t max) {
	if (value < min)	return min;
	if (value > max)	return max;
	return value;
}

int32_t map(int32_t value, int32_t v_min, int32_t v_max, int32_t r_min, int32_t r_max) {
	if (v_min == v_max) return r_min;
	int32_t round = (v_max - v_min) >> 1;
	return ((value - v_min) * (r_max - r_min) + round) / (v_max - v_min) + r_min;
}

/*
 * The exponential average value; Changes the emp_data: the element of the emp_summ array
 */
int32_t exp_mov_average(int32_t *_data, uint8_t coeff, int32_t val) {
	uint8_t round_v = coeff >> 1;
	*_data += val- (*_data + round_v) / coeff;
	return (*_data + round_v) / coeff;
}
