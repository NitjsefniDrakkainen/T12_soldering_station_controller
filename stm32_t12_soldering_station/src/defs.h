/*
 * defs.h
 *
 *  Created on: 16.09.2019
 *      Author: NitjsefniDrakkainen
 */

#ifndef __DEFS_H__
#define __DEFS_H__

#if DEBUG
#pragma message "debug configuration"

#define USART1_TX_PIN 							GPIO_Pin_9
#define USART1_RX_PIN 							GPIO_Pin_10
#define USART1_PORT								GPIOA

#endif

/*
 * Pinout definitions
 */
#define ADC_TIP_PIN								GPIO_Pin_1
#define ADC_CUR_PIN								GPIO_Pin_2
#define ADC_AMB_IN_PIN							GPIO_Pin_3
#define ADC_AMB_HANDLE_PIN						GPIO_Pin_4

#define ADC1_PORT								GPIOA

/*
 * ADC Channels definitions
 */

#define ADC_TIP_TEMERATURE_SENSE				ADC_Channel_1
#define ADC_TIP_CURRENT_SENSE					ADC_Channel_2
#define ADC_INTERNAL_AMBIENT_T_SENSE			ADC_Channel_3
#define ADC_EXTERNAL_AMBIENT_T_SENSE			ADC_Channel_4


#endif /* __DEFS_H__ */
