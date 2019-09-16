/*
 * priph_init.c
 *
 *  Created on: 16.09.2019
 *      Author: NitjsefniDrakkainen
 */

/*
 * includes
 */

/*----------------------------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "defs.h"
#include "periph_init.h"
/*----------------------------------------------------------------------------------------*/

/*
 * static functions declarations
 */

/*----------------------------------------------------------------------------------------*/
static void rcc_init();
static void gpio_init(void);
/*----------------------------------------------------------------------------------------*/

/*
 * extern functions definitions
 */

/*----------------------------------------------------------------------------------------*/

void station_init_periph(void)
{
	rcc_init();
	gpio_init();
}

/*----------------------------------------------------------------------------------------*/

/*
 * static functions definitions
 */

/*----------------------------------------------------------------------------------------*/

void rcc_init()
{
	//GPIO clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	//ADC clock and divider
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);

	//USART1 for debug
#if DEBUG
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#endif
}

void gpio_init()
{
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);

	gpio.GPIO_Pin = ADC_TIP_PIN | ADC_CUR_PIN | ADC_AMB_IN_PIN | ADC_AMB_HANDLE_PIN;
	gpio.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(ADC1_PORT, &gpio);

#if DEBUG
	//UASRT1 TX
	gpio.GPIO_Pin = USART1_TX_PIN;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &gpio);

	//UASRT1 RX
	gpio.GPIO_Pin = USART1_RX_PIN;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(USART1_PORT, &gpio);
#endif

}

/*----------------------------------------------------------------------------------------*/
