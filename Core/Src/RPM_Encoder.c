#include "stm32f4xx.h"
#include "stdint.h"

#include "RPM_Encoder.h"
#include "delay.h"
const int AF02=0x02;
volatile uint64_t first, second, dir;
float frequency;
float time;
float rpm_out;

float get_rpm(){
	return rpm_out;
	//return TIM3->CNT;
}

//void TIM3_IRQHandler(void)
//	{
//
//	if(TIM_SR_UIF&&TIM3->SR)
//		{
//		if(dir==1)
//			{
//			time=micros()-first;
//			time=(time/1000000);
//			frequency=1/time;
//			rpm_out=frequency*60;
//			reset_micros();
//			first=0;
//			dir=0;
//			}
//		else {
//			first =micros();
//			dir=1;
//			}
//
//		TIM3->SR&=~TIM_SR_UIF;
//		}
//	}
