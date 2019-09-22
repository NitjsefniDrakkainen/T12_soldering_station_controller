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
#include "m_snprintf.h"
#include "ssd1306_fonts.h"
#include "ssd1306_driver.h"
#include <math.h>
/*----------------------------------------------------------------------------------------*/

/*
 * static variables and structures definitions
 */


static struct StationType_t {
	uint8_t mode_flags;
	uint8_t handle_flags;
	volatile uint8_t state_counter;
	uint16_t lcd_update_freq;
	volatile uint16_t adc_values[ADC_CHANNELS_NUM];
	uint16_t temp;
	int h_ambient;
	volatile uint32_t ambient;
} _station;

/*
 * static functions declarations
 */

/*----------------------------------------------------------------------------------------*/
static int	station_thermistor_calc(void);
static void rcc_init();
static void gpio_init(void);
static void adc_init(void);
static void nvic_init(void);
static void tim_init(void);
static void _i2c_init(void);
/*----------------------------------------------------------------------------------------*/

/*
 * extern functions definitions
 */

/*----------------------------------------------------------------------------------------*/

void station_init_periph(void)
{
	uint32_t delay_cnt = 0;
	_station.mode_flags = 0;
	rcc_init();
	gpio_init();
	adc_init();
	nvic_init();

	_i2c_init();

	//SysTic interrupt 1ms
	SysTick_Config(SystemCoreClock / 1000);

#ifdef DEBUG
	USART_InitTypeDef uart;
	USART_StructInit(&uart);
	uart.USART_BaudRate = 115200;
	USART_Init(USART1, &uart);
	USART_Cmd(USART1, ENABLE);
	debug_print(USART1, "init\r\n");
#endif
	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_SetCursor(1 , 0);
	ssd1306_WriteString("T12 station v_2.0", Font_7x10, White);
	ssd1306_SetCursor(35 , 13);
	ssd1306_WriteString("Init...", Font_11x18, White);
	ssd1306_UpdateScreen();


	for (delay_cnt = 0; delay_cnt < 7200000; ++delay_cnt) ;

	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();
	_station.handle_flags = 0;
	_station.state_counter = 0;
	_station.lcd_update_freq = 100;
	//_station.mode_flags |= STATION_INITED | SELFTEST_MODE;

	debug_print(USART1, "MODE = 0x%2x\r\n", _station.mode_flags);
	tim_init();
	_station.handle_flags |= IRON_OPERATE_PHASE;
	_station.mode_flags |= STATION_INITED;

}

void station_iron_on_off(uint8_t on)
{
	if (_station.handle_flags & IRON_CONNECTED_FLAG) {
		if ( on && !(_station.handle_flags & IRON_ON_FLAG) ) _station.handle_flags |= IRON_ON_FLAG;
		if (!on && (_station.handle_flags & IRON_ON_FLAG) ) _station.handle_flags &= ~IRON_ON_FLAG;
	}
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
		retVal = 0;
	}
	return retVal;
}

uint8_t station_get_mode(void)
{
	return _station.mode_flags;
}

uint16_t station_get_update_freq(void)
{
	return _station.lcd_update_freq;
}

void station_get_disp_values(uint16_t *t, int *hamb)
{
	*t = _station.temp;
	*hamb = _station.h_ambient;
}

uint8_t station_get_iron_on()
{
	return _station.handle_flags & IRON_ON_FLAG;
}

volatile uint32_t counter = 0, t_avg = 0;

void station_iron_tip_handler()
{
	uint16_t _adc[ADC_CHANNELS_NUM];

	if ((_station.mode_flags) & STATION_INITED && (_station.handle_flags & IRON_ON_FLAG)) {

		if ((_station.handle_flags & IRON_OPERATE_PHASE) && ( ++(_station.state_counter) > 20)) {
			_station.state_counter = 0;

			if ( _station.adc_values[ADC_TIP_CURRENT_VAL] < 10 && _station.adc_values[ADC_TIP_TEMERATURE_VAL] > 4000) {
				debug_print(USART1, "disc ON (error)\r\n");
				_station.handle_flags &= ~IRON_CONNECTED_FLAG;
			} else {
				_station.handle_flags |= IRON_CONNECTED_FLAG;
			}

			TIM2->CCR2 = 0;
			_station.handle_flags &= ~IRON_OPERATE_PHASE;
			_station.handle_flags |= IRON_WAIT_PHASE;


		} else if ((_station.handle_flags & IRON_WAIT_PHASE) && ( ++(_station.state_counter) > 7)) {
			_station.state_counter = 0;
			_station.handle_flags &= ~IRON_OPERATE_PHASE;
			_station.handle_flags &= ~IRON_WAIT_PHASE;
			_station.handle_flags |= IRON_CHECK_PHASE;

		} else if ((_station.handle_flags & IRON_CHECK_PHASE)) {
			//_station.state_counter += 1;

			//if ( ++(_station.state_counter) >= 5) {
				_station.temp = _station.adc_values[ADC_TIP_TEMERATURE_VAL];
				t_avg = 0;
				TIM2->CCR2 = 500;
				_station.handle_flags &= ~IRON_CHECK_PHASE;
				_station.handle_flags &= ~IRON_WAIT_PHASE;
				_station.handle_flags |= IRON_OPERATE_PHASE;
				_station.state_counter = 0;
		//}


		} else {
			_station.handle_flags |= IRON_OPERATE_PHASE;
		}

	}

	if (! (_station.handle_flags & IRON_ON_FLAG) ) {

		if (_station.mode_flags & STATION_CURR_SENSE_REQ) {

			if ( ++(_station.state_counter) > 20) {
				_station.mode_flags &= ~STATION_CURR_SENSE_REQ;
				_station.state_counter = 0;

				if ( _station.adc_values[ADC_TIP_CURRENT_VAL] < 10 && _station.adc_values[ADC_TIP_TEMERATURE_VAL]) {
					debug_print(USART1, "disc OFF\r\n");
					_station.handle_flags &= ~IRON_CONNECTED_FLAG;
				} else {
					_station.handle_flags |= IRON_CONNECTED_FLAG;
				}

				TIM2->CCR2 = 0;
			}
		} else {
			if (_station.adc_values[ADC_TIP_CURRENT_VAL] < 10) {
				_station.temp = _station.adc_values[ADC_TIP_TEMERATURE_VAL];
			}
		}
	}
/*
		if ( ( (_station.mode_flags) & SELFTEST_MODE )) {
			if(TIM2->CCR2 > 0) {
				debug_print(USART1, "start!\r\n");
				TIM2->CCR2 = 1500;
			}
			if (++(_station.state_counter) > 250) {
				_station.state_counter = 0;
				TIM2->CCR2 = 0;
				_station.mode_flags &=  ~SELFTEST_MODE;
				debug_print(USART1, "stop!\r\n");
			}
		}
*/
		if (++counter > 40000) {
			counter = 0;
			station_get_adc_channels(5, _adc);
			_station.ambient = _adc[3];
			_station.h_ambient = station_thermistor_calc();

		}
}

static uint16_t cnt_10ms = 0;
static uint16_t cnt_1000ms = 0;
uint8_t cnt = 0;
void station_controll_callback(void)
{
	if ((_station.mode_flags & STATION_INITED) && (++cnt_10ms > 10) ) {

		if ( !(_station.handle_flags & IRON_ON_FLAG) ) {

			TIM2->CCR2 = 100;
			_station.mode_flags |= STATION_CURR_SENSE_REQ;
			//debug_print(USART1, "req start\r\n");
		}

		cnt_10ms = 0;
	}

	if (++cnt_1000ms > 1000) {
		cnt_1000ms = 0;
		debug_print(USART1, "amb = %d, handle = 0x%02x\r\n ",  _station.h_ambient, _station.handle_flags);
	}
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
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);


	//I2C, TIM2 clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 | RCC_APB1Periph_TIM2, ENABLE);

	//DMA1 for adc readings
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	//USART1 for debug
#ifdef DEBUG
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#endif
}

void gpio_init()
{
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);

	//ADC inputs init
	gpio.GPIO_Pin = ADC_TIP_PIN | ADC_CUR_PIN | ADC_AMB_IN_PIN | ADC_AMB_HANDLE_PIN;
	gpio.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(ADC1_PORT, &gpio);

	//I2C1 pins init
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = I2C1_SCL | I2C1_SDA; // SCL, SDA
	gpio.GPIO_Mode = GPIO_Mode_AF_OD;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2C1_PORT, &gpio);

	//PWM pin init
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = TIP_PWM_PIN;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(TIP_PWM_PORT, &gpio);

	//Digital inputs pins init
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = ENCODER_PIN_A | ENCODER_PIN_B;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(ENC_AB_SW_PORT, &gpio);

	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = HANDLE_SLEEP_SWITCH;
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(ENC_AB_SW_PORT, &gpio);

	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = ENCODER_PIN_C;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(ENC_C_PORT, &gpio);

#ifdef DEBUG
	//UASRT1 TX
	GPIO_StructInit(&gpio);
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
	//dma.DMA_MemoryBaseAddr = ((uint32_t)(_station.adc_values));
	dma.DMA_MemoryBaseAddr = ((uint32_t)(_station.adc_values));
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

	ADC_RegularChannelConfig(ADC1, ADC_TIP_TEMERATURE_SENSE, 1, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_TIP_CURRENT_SENSE, 2, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_INTERNAL_AMBIENT_T_SENSE, 3, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_EXTERNAL_AMBIENT_T_SENSE, 4, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_MCU_TEMPERATURE_SENSE, 5, ADC_SampleTime_71Cycles5);

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

	//handle switch exti
	EXTI_StructInit(&exti);
	exti.EXTI_Line = EXTI_Line12;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Falling;
	exti.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);

	//encoder C exti
	EXTI_StructInit(&exti);
	exti.EXTI_Line = EXTI_Line13;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	exti.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);

	//NVIC configuration for TIP_PWM_TIMER
	nvic.NVIC_IRQChannel = TIM2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	//NVIC configuration for Encoder A
	nvic.NVIC_IRQChannel = EXTI9_5_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0x00;
	nvic.NVIC_IRQChannelSubPriority = 0x00;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	//NVIC configuration for Encoder C and handle switch
	nvic.NVIC_IRQChannel = EXTI15_10_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0x00;
	nvic.NVIC_IRQChannelSubPriority = 0x00;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
}

void tim_init(void)
{
	TIM_TimeBaseInitTypeDef timer;
	TIM_OCInitTypeDef  channel;

	//TIM2
	TIM_TimeBaseStructInit(&timer);
	timer.TIM_ClockDivision = 0;
	timer.TIM_CounterMode = TIM_CounterMode_Up;
	timer.TIM_Period = TIP_PWM_TIMER_PERIOD;
	timer.TIM_Prescaler = TIP_PWM_TIMER_PRESCALER;

	TIM_TimeBaseInit(TIP_PWM_TIMER, &timer); //TIM2 interrupt

	TIM_OCStructInit(&channel);
	channel.TIM_OCMode = TIM_OCMode_PWM1;
	channel.TIM_OutputState = TIM_OutputState_Enable;
	channel.TIM_Pulse = 0;
	TIM_OC2Init(TIP_PWM_TIMER, &channel);

	TIM_ITConfig(TIP_PWM_TIMER, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIP_PWM_TIMER, ENABLE);


}

void _i2c_init(void)
{
	I2C_InitTypeDef i2c;

	I2C_StructInit(&i2c);
	i2c.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c.I2C_OwnAddress1 = 0x00;
	i2c.I2C_Ack = I2C_Ack_Enable;
	i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	i2c.I2C_Mode = I2C_Mode_I2C;
	i2c.I2C_ClockSpeed = 400000;

	I2C_Init(I2C1, &i2c);
	I2C_Cmd(I2C1, ENABLE);
}
/*----------------------------------------------------------------------------------------*/

int	station_thermistor_calc(void) {
	static const uint16_t add_resistor	= 10000;				// The additional resistor value (10koHm)
	static const float 	  normal_temp[2]= { 10000, 25 };		// nominal resistance and the nominal temperature
	static const uint16_t beta 			= 3950;     			// The beta coefficient of the thermistor (usually 3000-4000)
	static uint32_t	average 			= 0;					// Previous value of analog read
	static int 		cached_ambient 		= 0;					// Previous value of the temperature

	if (_station.ambient == average)
		return cached_ambient;

	average = _station.ambient;

	// convert the value to resistance
	float resistance = 4096.0 / (float)average - 1.0;
	resistance = (float)add_resistor / resistance;

	float steinhart = resistance / normal_temp[0];			// (R/Ro)
	steinhart = log(steinhart);								// ln(R/Ro)
	steinhart /= beta;										// 1/B * ln(R/Ro)
	steinhart += 1.0 / (normal_temp[1] + 273.15);  			// + (1/To)
	steinhart = 1.0 / steinhart;							// Invert
	steinhart -= 273.15;									// convert to Celsius
	cached_ambient = round(steinhart);
	return cached_ambient;
}
