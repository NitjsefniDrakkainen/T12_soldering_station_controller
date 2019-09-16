/*
 * isrs.c
 *
 *  Created on: 16.09.2019
 *      Author: NitjsefniDrakkainen
 */


/*
 * includes
 */

#include "stm32f10x.h"
#include "defs.h"


/*
 * interrupts handlers definitions
 */

void SysTick_Handler()
{

}
void TIM2_IRQHandler()
{
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}
