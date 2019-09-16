/*
 * defs.h
 *
 *  Created on: 16.09.2019
 *      Author: NitjsefniDrakkainen
 */

#ifndef __DEFS_H__
#define __DEFS_H__

/*
 * Pinout definitions
 */
#define ADC_TIP_PIN								GPIO_Pin_1
#define ADC_CUR_PIN								GPIO_Pin_2
#define ADC_AMB_IN_PIN							GPIO_Pin_3
#define ADC_AMB_HANDLE_PIN						GPIO_Pin_4

/*
 * ADC Channels definitions
 */

#define ADC_TIP_TEMERATURE_SENSE				ADC_Channel_1
#define ADC_TIP_CURRENT_SENSE					ADC_Channel_2
#define ADC_INTERNAL_AMBIENT_T_SENSE			ADC_Channel_3
#define ADC_EXTERNAL_AMBIENT_T_SENSE			ADC_Channel_4

#endif /* __DEFS_H__ */
