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
#include "helpers.h"
#include "arm_math.h"
/*----------------------------------------------------------------------------------------*/

/*
 * static variables and structures definitions
 */

#define TEMP_0							(4U)

static   uint32_t ema_buff[6] 		= { 0,  0,  0,  0 , 0 , 0};
static   uint8_t  ema_coeffs[6] 			= { 4, 50, 50, 50 , 10, 200};

struct s_tempReference 	{
	uint16_t	t200, t260, t330, t400;
};

/* Choose PID parameters */
//#define PID_Q15_PARAM_KP        0x4CCD          /* Proporcional = 0,6*/
//#define PID_Q15_PARAM_KI        0x1AA       	/* Integral = 0.013 */
//#define PID_Q15_PARAM_KD        0x1852          /* Derivative = 0.19*/

#define PID_Q15_PARAM_KP        20766          /* Proporcional = 0,63*/
#define PID_Q15_PARAM_KI        34      	/* Integral = 0.0009 */
#define PID_Q15_PARAM_KD        6015         /* Derivative = 0.06*/


static arm_pid_instance_q15 pid_q;



static struct s_tempReference temp_ref = {
	.t200	= 200,
	.t260	= 260,
	.t330	= 330,
	.t400	= 400
};

static struct s_tip {
	uint16_t	t200, t260, t330, t400;				// The internal temperature in reference points
	uint8_t		mask;								// The bit mask: TIP_ACTIVE + TIP_CALIBRATED
	char		name[40];							// T12 tip name suffix, JL02 for T12-JL02
	int8_t		ambient;							// The ambient temperature in Celsius when the tip being calibrated
	uint8_t		crc;								// CRC checksum
} tip;

static struct StationType_t {
	volatile uint8_t mode_flags;
	volatile uint8_t handle_flags;
	uint16_t lcd_update_freq;
	volatile uint16_t temp_avg;
	uint16_t temp_preset;
	uint16_t temp_sleep;
	uint16_t temp_set;
	volatile uint16_t current_avg;

	uint16_t max_power;
	uint16_t power_summ;
	uint16_t disp_power_summ;
	uint16_t avg_power;
	uint16_t dispersion_of_power;
	volatile int h_ambient;
	int32_t  pid_power;
	int32_t  pid_integral_summ;
	volatile uint32_t ambient_avg;
	volatile uint16_t adc_values[ADC_CHANNELS_NUM];
} _station;

/*
 * static functions declarations
 */

/*----------------------------------------------------------------------------------------*/
static int	station_thermistor_calc(void);
//static void station_pid_reset(int temp);
//static int32_t station_pid_power_calc(uint16_t _set, uint16_t _curr);
//static uint32_t station_iron_pwm_calc(uint16_t _curr);
static uint16_t station_iron_pwm_calc(void);
static uint8_t station_get_power(void);
static uint16_t station_get_temp(uint16_t temp, int16_t ambient);
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
	ssd1306_SetCursor(1 , 1);
	ssd1306_WriteString("T12 station v_2.0", Font_7x10, White);
	ssd1306_SetCursor(35 , 13);
	ssd1306_WriteString("Init...", Font_11x18, White);
	ssd1306_UpdateScreen();


	for (delay_cnt = 0; delay_cnt < 7200000; ++delay_cnt) ;

	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();


	_station.mode_flags = 0;
	_station.handle_flags = 0;
	_station.lcd_update_freq = 100;
	_station.temp_preset = 2000;
	_station.max_power = 1600;

	tip.t200 = 1100;
	tip.t260 = 1800;
	tip.t330 = 2500;
	tip.t400 = 3700;
	tip.ambient = 25;

	pid_q.Kp = PID_Q15_PARAM_KP;
	pid_q.Ki = PID_Q15_PARAM_KI;
	pid_q.Kd = PID_Q15_PARAM_KD;

	arm_pid_init_q15(&pid_q, 1);

	tim_init();

	_station.mode_flags |= STATION_INITED;
	_station.mode_flags |= STATION_MODE_OPERATE; // 1 - normal operate mode, 0 - menu mode
	_station.mode_flags |= STATION_DISPLAY_MODE_NORMAL; //TODO:  load settings from eeprom

}

void station_iron_on_off(uint8_t on)
{
	if (_station.handle_flags & IRON_CONNECTED_FLAG) {
		if ( on && !(_station.handle_flags & IRON_ON_FLAG) ) {
			_station.handle_flags |= IRON_ON_FLAG;
			_station.handle_flags &= ~IRON_SLEEP_FLAG;
			_station.temp_set = _station.temp_preset;
			arm_pid_reset_q15(&pid_q);
		}
		if (!on && (_station.handle_flags & IRON_ON_FLAG) ) {
			_station.handle_flags &= ~IRON_ON_FLAG;
			_station.handle_flags &= ~IRON_SLEEP_FLAG;
		}
	} else {
		_station.handle_flags &= ~IRON_ON_FLAG;
	}
}

void station_iron_sleep_on_off(uint8_t val)
{

	if (_station.handle_flags & IRON_ON_FLAG) {

		if (val && !(_station.handle_flags & IRON_SLEEP_FLAG)) {
			_station.handle_flags |= IRON_SLEEP_FLAG;
			_station.temp_set = _station.temp_sleep;
		}
		else if (!val && (_station.handle_flags & IRON_SLEEP_FLAG)) {
			_station.handle_flags &= ~IRON_SLEEP_FLAG;
			_station.temp_set = _station.temp_preset;

		}
	}

	arm_pid_reset_q15(&pid_q);
}

void station_iron_temp_adjust(int8_t val)
{
	if (_station.temp_preset + val >= TIP_MAX_TEMP_ADC) val = _station.temp_preset = TIP_MAX_TEMP_ADC;
	else if (_station.temp_preset + val <= TIP_MIN_TEMP_ADC) _station.temp_preset = TIP_MIN_TEMP_ADC;
	else _station.temp_preset += val;

	_station.temp_set = _station.temp_preset;
	//arm_pid_reset_q15(&pid_q);
}

void pid_d_tune(uint8_t val)
{
	if (val) {
		if (++(pid_q.Kd) >= 0x7FFD) pid_q.Kd = 0x7FFD;
	} else {
		if (--(pid_q.Kd) < 0) pid_q.Kd = 0;
	}
	debug_print(USART1, "p: %d i: %d d: %d, ratio: %d\r\n", pid_q.Kp, pid_q.Ki, pid_q.Kd, (uint16_t)(pid_q.Kp/pid_q.Kd));
	arm_pid_init_q15(&pid_q, 0);
}

uint8_t station_get_mode(void)
{
	return _station.mode_flags;
}

uint16_t station_get_update_freq(void)
{
	return _station.lcd_update_freq;
}

void station_get_disp_values(uint16_t *t, uint16_t *t_preset, int *hamb, uint8_t *pwr, uint16_t *t_current)
{
	*t = station_get_temp(_station.temp_avg, _station.h_ambient);
	*t_preset = station_get_temp(_station.temp_preset, 23);
	*hamb = _station.h_ambient;
	*pwr = station_get_power();
	*t_current = map(TIM2->CCR2, 0, TIP_PWM_TIMER_PERIOD, 0, 300);
}

uint8_t station_iron_get_connected()
{
	return _station.handle_flags & IRON_CONNECTED_FLAG;
}

uint8_t station_get_iron_on(void)
{
	return _station.handle_flags & IRON_ON_FLAG;
}

uint8_t station_iron_get_sleep(void)
{
	return ( (_station.handle_flags & IRON_ON_FLAG) && (_station.handle_flags & IRON_SLEEP_FLAG));
}



void station_iron_heat_clbk()
{
	_station.current_avg = exp_mov_average((int32_t *)(&ema_buff[0]), ema_coeffs[0], _station.adc_values[ADC_TIP_CURRENT_VAL]);

	if ( _station.current_avg < 10 ) {

		_station.handle_flags &= ~IRON_CONNECTED_FLAG;
		_station.handle_flags &= ~IRON_ON_FLAG;
	} else {
		_station.handle_flags |= IRON_CONNECTED_FLAG;
	}

	TIM2->CCR2 = 0;
}

void station_iron_recalc_clbk()
{

	if (_station.handle_flags & IRON_CONNECTED_FLAG) {

		_station.temp_avg = exp_mov_average((int32_t *)(&ema_buff[1]), ema_coeffs[1], _station.adc_values[ADC_TIP_TEMERATURE_VAL]);
		if (_station.handle_flags & IRON_ON_FLAG) {
			/*calc pwm func begin*/

			if (_station.adc_values[ADC_TIP_TEMERATURE_VAL] < 1400) {
				TIM2->CCR2 = _station.max_power;
			} else if (_station.adc_values[ADC_TIP_TEMERATURE_VAL] > _station.temp_set + 10) {
				TIM2->CCR2 = 0;

			} else {
				TIM2->CCR2 = station_iron_pwm_calc();;
			}

		} else {
			TIM2->CCR2 = 0;
		}
	}


}


static uint8_t cnt_10ms = 0;
static uint16_t cnt_1000ms = 0;

void station_controll_callback(void)
{
	if ((_station.handle_flags & IRON_ON_FLAG)) {
		if ( (_station.handle_flags & IRON_SLEEP_FLAG) && (GPIO_ReadInputDataBit(ENC_AB_SW_PORT, HANDLE_SLEEP_SWITCH) == SET) ) {
			//iron is on, switch signal is high - go to operate mode
			_station.handle_flags &= ~IRON_SLEEP_FLAG;
			_station.temp_set = _station.temp_preset;
			//station_pid_reset(_station.temp_set);
		} else if (!(_station.handle_flags & IRON_SLEEP_FLAG) && (GPIO_ReadInputDataBit(ENC_AB_SW_PORT, HANDLE_SLEEP_SWITCH) == RESET) ) {
			//iron is on, switch signal is low - go to sleep mode
			_station.handle_flags |= IRON_SLEEP_FLAG;
			_station.temp_set = _station.temp_sleep;
			//station_pid_reset(_station.temp_set);
		}
	}

	if (++cnt_10ms > 10) {
		_station.ambient_avg = exp_mov_average((int32_t *)(&ema_buff[2]), ema_coeffs[2], _station.adc_values[ADC_EXTERNAL_AMBIENT_T_VAL]);
		_station.h_ambient = station_thermistor_calc();
	}

	if (++cnt_1000ms > 1000) {
		cnt_1000ms = 0;
		//debug_print(USART1, "amb = %d, curr = %d\r\n ",  _station.h_ambient, _station.adc_values[ADC_TIP_CURRENT_VAL]);
	}
}

/*----------------------------------------------------------------------------------------*/

/*
 * static functions definitions
 */

/*----------------------------------------------------------------------------------------*/
#if 0
static void station_pid_reset(int t)
{
	_station.temp_prev_0 = 0;
	_station.pid_power = 0;
	_station.pid_integral_summ = 0;
	_station.handle_flags &= ~IRON_PID_PI_MODE_FLAG;

	if (t > 0 && t < 4096) {
		_station.temp_prev_1 = t;
	} else {
		_station.temp_prev_1 = 0;
	}
	//debug_print(USART1, "reset, cool flag = %d, iterate = %d\r\n", (_station.handle_flags & IRON_COOLING_FLAG) ? 1 : 0, (_station.handle_flags & IRON_PID_PI_MODE_FLAG) ? 1 : 0);
}

int32_t station_pid_power_calc(uint16_t _set, uint16_t _curr)
{
	int32_t diff =  _set - _curr, curr_diff = _station.temp_prev_1 - _curr, d_diff = (_station.temp_prev_0 + _curr - 2 * _station.temp_prev_1 );
	int32_t p_drive, d_drive, i_drive, delta, ret;

	if (!_station.temp_prev_0) {
		if ( !(_station.handle_flags & IRON_PID_PI_MODE_FLAG) && diff < 30 ) {
			_station.handle_flags |= IRON_PID_PI_MODE_FLAG;
			_station.pid_power = 0;
			_station.pid_integral_summ = 0;
			//debug_print(USART1, "iterate, cool flag = %d, iterate = %d\r\n", (_station.handle_flags & IRON_COOLING_FLAG) ? 1 : 0, (_station.handle_flags & IRON_PID_PI_MODE_FLAG) ? 1 : 0);
		}
		_station.pid_integral_summ += diff;
		_station.pid_power = _station.p_factor * diff + _station.i_factor * _station.pid_integral_summ;
	} else {
		p_drive = _station.p_factor * curr_diff;
		d_drive = _station.d_factor * d_diff;
		i_drive = _station.i_factor * diff;
		delta = p_drive + i_drive + d_drive;
		_station.pid_power += delta;
	}

	if (_station.handle_flags & IRON_PID_PI_MODE_FLAG) {
		_station.temp_prev_0 = _station.temp_prev_1;

	}

	_station.temp_prev_1 = _curr;
	ret = _station.pid_power + (1 << (_station.pid_denominator - 1));
	ret = ret >> _station.pid_denominator;
	//debug_print(USART1, "%d %d\r\n",_curr, ret);
	return ret;
}

uint32_t station_iron_pwm_calc(uint16_t _curr)
{
	uint32_t ret = 0;
	int32_t diff;
	if (_station.handle_flags & IRON_ON_FLAG) {

		if ( (_curr > TIP_MAX_TEMP_ADC) || (_curr > (_station.temp_set) + 400 ) ) {
			_station.handle_flags |= IRON_COOLING_FLAG;
			//debug_print(USART1, "cooling, diff = %d\r\n", (int32_t)(_curr - _station.temp_set ));
		}

		if (_station.handle_flags & IRON_COOLING_FLAG) {

			if (_curr < (_station.temp_set - 2)) {

				_station.handle_flags &= ~IRON_COOLING_FLAG;
				station_pid_reset(0);
				//debug_print(USART1, "cool end, cool flag = %d, iterate = %d\r\n", (_station.handle_flags & IRON_COOLING_FLAG) ? 1 : 0, (_station.handle_flags & IRON_PID_PI_MODE_FLAG) ? 1 : 0);
			} else {
				_station.avg_power = exp_mov_average((int32_t *)(&(_station.power_summ)), ema_coeffs[5], 0);
				_station.dispersion_of_power = exp_mov_average((int32_t *)(&(_station.disp_power_summ)), ema_coeffs[5], _station.avg_power * _station.avg_power);
				return 0;
			}
		}

		ret = station_pid_power_calc(_station.temp_set, _curr);
		ret = constrain(ret, 0, _station.max_power);
	}
	_station.avg_power = exp_mov_average((int32_t *)(&(_station.power_summ)), ema_coeffs[5], ret);

	diff = _station.avg_power - ret;
	_station.dispersion_of_power = exp_mov_average((int32_t *)(&(_station.disp_power_summ)), ema_coeffs[5], diff * diff);

	return ret;
}
#endif
/*
static q15_t adc_to_q15 (int32_t _val)
{
	if (_val >= 4096) _val = 4096 - 1;
	if (_val <= (-1 * 4096)) _val = (-1 * 4096) + 1;
	int64_t acc = ((_val << 15)/4096);

	acc =  ((acc * (2 << 14)) >> 15 );

	return (q15_t) (acc);
}
*/

q15_t error_to_q15(uint16_t _val, uint16_t target)
{
	int16_t err = target - _val;

	if (err < -999 ) {
		err = -999;
	}
	int64_t acc = ((err << 15)/(target + 1));

	acc =  ((acc * (2 << 14)) >> 15 );
	return (q15_t) (acc);
}

uint16_t station_iron_pwm_calc(void)
{

	q15_t _in_err, _out_p/*, _adc, _set*/;

	_in_err = error_to_q15(_station.adc_values[ADC_TIP_TEMERATURE_VAL], _station.temp_set);
	_out_p = arm_pid_q15(&pid_q, _in_err);

	if (_out_p < 0 ) _out_p = 0;
	return ( (uint16_t)map(_out_p, 0, 32768, 0, _station.max_power));

}

uint8_t station_get_power(void)
{
	//uint16_t tmp = _station.avg_power;
	//tmp = constrain(tmp, 0, _station.max_power);
	return ((uint8_t)(100*TIM2->CCR2/_station.max_power));
}

static uint16_t station_get_temp(uint16_t temp, int16_t ambient) {
	int16_t tempH = 0;

	// The temperature difference between current ambient temperature and one when the tip being calibrated
	int d = ambient - tip.ambient;
	if (temp < tip.t200) {
	    tempH = map(temp, 0, tip.t200, ambient, temp_ref.t200+d);
	} else if (temp < tip.t260) {
		tempH = map(temp, tip.t200, tip.t260, temp_ref.t200+d, temp_ref.t260+d);
	} else if (temp < tip.t330){
		tempH = map(temp, tip.t260, tip.t330, temp_ref.t260+d, temp_ref.t330+d);
	} else {
		tempH = map(temp, tip.t330, tip.t400, temp_ref.t330+d, temp_ref.t400+d);
	}
	if (tempH < 0) tempH = 0;
	//if (!a_cfg.celsius)
	    //tempH = celsiusToFahrenheit(tempH);
	return tempH;
}

void rcc_init()
{
	//GPIO clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	//ADC clock and divider
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);


	//I2C, TIM2, TIM3 clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 | RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);

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
	/*
	//handle switch exti
	EXTI_StructInit(&exti);
	exti.EXTI_Line = EXTI_Line12;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Falling;
	exti.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
	*/
	//encoder C exti
	EXTI_StructInit(&exti);
	exti.EXTI_Line = EXTI_Line13;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	exti.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);

	//NVIC configuration for TIM3
	nvic.NVIC_IRQChannel = TIM3_IRQn;
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

	//TIM2 base
	TIM_TimeBaseStructInit(&timer);
	timer.TIM_ClockDivision = 0;
	timer.TIM_CounterMode = TIM_CounterMode_Up;
	timer.TIM_Period = TIP_PWM_TIMER_PERIOD /*65535*/;
	timer.TIM_Prescaler = TIP_PWM_TIMER_PRESCALER;

	TIM_TimeBaseInit(TIP_PWM_TIMER, &timer);
	//TIM3 base
	timer.TIM_Period = 399;
	TIM_TimeBaseInit(TIM3, &timer);

	//TIM2 ch2 PWM
	TIM_OCStructInit(&channel);
	channel.TIM_OCMode = TIM_OCMode_PWM1;
	channel.TIM_OutputState = TIM_OutputState_Enable;
	channel.TIM_Pulse = 0;
	TIM_OC2Init(TIP_PWM_TIMER, &channel);

	/* Select the Master Slave Mode */
	TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);

	/* Master Mode selection */
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);

	//TIM3 channel 1 for heating drive
	TIM_OCStructInit(&channel);
	channel.TIM_OCMode = TIM_OCMode_Timing;
	channel.TIM_OutputState = TIM_OutputState_Enable;
	channel.TIM_Pulse = 20;
	TIM_OC1Init(TIM3, &channel);

	//TIM3 channel 2 for measurement phase
	TIM_OCStructInit(&channel);
	channel.TIM_OCMode = TIM_OCMode_Timing;
	channel.TIM_OutputState = TIM_OutputState_Enable;
	channel.TIM_Pulse = 28;
	TIM_OC2Init(TIM3, &channel);

	TIM_ITConfig(TIM3, TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2, ENABLE); //TIM3 interrupt

	/* Slave Mode selection: TIM3 */
	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Gated);
	TIM_SelectInputTrigger(TIM3, TIM_TS_ITR1);

	TIM_Cmd(TIM3, ENABLE);
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

	if (_station.ambient_avg == average)
		return cached_ambient;

	average = _station.ambient_avg;

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
