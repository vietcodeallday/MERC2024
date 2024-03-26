/**

 * original author: Husamuldeen <https://github.com/hussamaldean>

   ----------------------------------------------------------------------
   	Copyright (C) husamuldeen, 2020

    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.
     
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   ----------------------------------------------------------------------
 */

#include "delay.h"
#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_it.h"

volatile uint64_t ms,rms;
//volatile uint64_t millis_count = 0;
void systick_init_ms(uint32_t frequency) /*Frequency in MHz*/
	{
	TIM_HandleTypeDef htim10;

	    // Disable interrupts
	    __disable_irq();

	    // Initialize TIM10 peripheral
	    htim10.Instance = TIM10;
	    htim10.Init.Prescaler = HAL_RCC_GetPCLK1Freq() / 1000000 - 1; // Assuming PCLK1 frequency in MHz
	    htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	    htim10.Init.Period = (1000000 / frequency) - 1; // Frequency in Hz
	    htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	    HAL_TIM_Base_Init(&htim10);

	    // Enable TIM10 interrupts
	    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0); // Set priority as needed
	    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

	    // Start TIM10 counter
	    HAL_TIM_Base_Start_IT(&htim10);

	    // Re-enable interrupts
	    __enable_irq();
}

uint64_t millis(void)
	{
	__disable_irq();
	rms=ms; //store current ms in rms
	__enable_irq();
	return rms;

	}

void reset_tick(void){
	rms=0;
	ms=0;
}

void delay(uint32_t delay)
	{
	
		uint64_t start=millis();
	do
		{}while(millis()-start!=delay);
	}
