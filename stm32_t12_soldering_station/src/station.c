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
#include <station.h>
#include "stm32f10x.h"
#include "defs.h"
/*----------------------------------------------------------------------------------------*/

/*
 * static variables and structures definitions
 */

static struct StationType_t {
	uint16_t adc_values[ADC_CHANNELS_NUM];
} _station;

/*
 * static functions declarations
 */

/*----------------------------------------------------------------------------------------*/
static void rcc_init();
static void gpio_init(void);
static void adc_init(void);
static void nvic_init(void);
static void tim_init(void);
/*----------------------------------------------------------------------------------------*/

/*
 * extern functions definitions
 */

/*----------------------------------------------------------------------------------------*/

void station_init_periph(void)
{
	rcc_init();
	gpio_init();
	adc_init();
	nvic_init();
	tim_init();
	//SysTic interrupt 1ms
	SysTick_Config(SystemCoreClock / 1000);
}

uint8_t station_get_adc_channel(uint8_t chan_nr, uint16_t *pValue)
{
	uint8_t retVal = 1;

	if (chan_nr <= ADC_CHANNELS_NUM) {
		*pValue = _station.adc_values[chan_nr - 1];
		retVal = 0;
	} else {
		retVal = 1;
	}

	return retVal;
}

uint8_t station_get_adc_channels(uint8_t nChans, uint16_t *pValue)
{
	uint8_t retVal = 1;

	if (nChans <= ADC_CHANNELS_NUM) {
		for (uint8_t i = 0; i < nChans; ++i) {
			pValue[i] = _station.adc_values[i];
		}
	}
	return retVal;
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

	//I2C, TIM2 clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 | RCC_APB1Periph_TIM2, ENABLE);

	//DMA1 for adc readings
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	//USART1 for debug
#ifdef DEBUG
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#endif
}

void gpio_init()
{
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);
	//ADC inputs init
	gpio.GPIO_Pin = ADC_TIP_PIN | ADC_CUR_PIN | ADC_AMB_IN_PIN | ADC_AMB_HANDLE_PIN;
	gpio.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(ADC1_PORT, &gpio);

	//I2C1 pins init
	//GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = I2C1_SCL | I2C1_SDA; // SCL, SDA
	gpio.GPIO_Mode = GPIO_Mode_AF_OD;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2C1_PORT, &gpio);

	//PWM pin init
	gpio.GPIO_Pin = TIP_PWM_PIN;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(TIP_PWM_PORT, &gpio);

	//Digital inputs pins init
	gpio.GPIO_Pin = ENCODER_PIN_A | ENCODER_PIN_B | HANDLE_SLEEP_SWITCH;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(ENC_AB_SW_PORT, &gpio);

	gpio.GPIO_Pin = ENCODER_PIN_C;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(ENC_C_PORT, &gpio);

#if DEBUG
	//UASRT1 TX
	gpio.GPIO_Pin = USART1_TX_PIN;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(USART1_PORT, &gpio);

	//UASRT1 RX
	gpio.GPIO_Pin = USART1_RX_PIN;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(USART1_PORT, &gpio);
#endif

}

void adc_init(void)
{
	DMA_InitTypeDef dma;
	ADC_InitTypeDef adc;
	DMA_StructInit(&dma);

	dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	dma.DMA_MemoryBaseAddr = (uint32_t)_station.adc_values;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dma.DMA_DIR = DMA_DIR_PeripheralSRC;
	dma.DMA_BufferSize = ADC_CHANNELS_NUM;
	dma.DMA_Mode = DMA_Mode_Circular;
	DMA_Init(DMA1_Channel1, &dma);

	DMA_Cmd(DMA1_Channel1, ENABLE);

	ADC_StructInit(&adc);

	adc.ADC_ScanConvMode = ENABLE;
	adc.ADC_ContinuousConvMode = ENABLE;
	adc.ADC_NbrOfChannel = ADC_CHANNELS_NUM;
	adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_Init(ADC1, &adc);

	ADC_RegularChannelConfig(ADC1, ADC_TIP_TEMERATURE_SENSE, 1, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_TIP_CURRENT_SENSE, 2, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_INTERNAL_AMBIENT_T_SENSE, 3, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_EXTERNAL_AMBIENT_T_SENSE, 4, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_MCU_TEMPERATURE_SENSE, 5, ADC_SampleTime_7Cycles5);

	ADC_TempSensorVrefintCmd(ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));

	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void nvic_init(void)
{
	NVIC_InitTypeDef nvic;
	EXTI_InitTypeDef exti;

	//encoder A exti
	EXTI_StructInit(&exti);
	exti.EXTI_Line = EXTI_Line9;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Falling;
	exti.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);

	//encoder A exti
	EXTI_StructInit(&exti);
	exti.EXTI_Line = EXTI_Line13;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Falling;
	exti.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);

	//NVIC configuration for TIP_PWM_TIMER
	nvic.NVIC_IRQChannel = TIM2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
}

void tim_init(void)
{
	TIM_TimeBaseInitTypeDef timer;

	//TIM2
	TIM_TimeBaseStructInit(&timer);
	timer.TIM_ClockDivision = 0;
	timer.TIM_CounterMode = TIM_CounterMode_Up;
	timer.TIM_Period = TIP_PWM_TIMER_PERIOD;
	timer.TIM_Prescaler = TIP_PWM_TIMER_PRESCALER;

	TIM_TimeBaseInit(TIP_PWM_TIMER, &timer); //TIM2 interrupt
	TIM_ITConfig(TIP_PWM_TIMER, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIP_PWM_TIMER, ENABLE);
}

/*----------------------------------------------------------------------------------------*/
