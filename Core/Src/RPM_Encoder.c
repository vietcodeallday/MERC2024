
#include "RPM_Encoder.h"


volatile uint16_t cnt=0;
volatile uint16_t preCnt=0;
volatile int loop=0;
float get_rpm(void){
	volatile float rp=0;

	uint16_t a=millis();
	while(millis()-a<=500){
		cnt=__HAL_TIM_GET_COUNTER(&htim1);
		if(cnt<preCnt){
			loop++;
			preCnt=0;
		}else preCnt=cnt;
	}
	rp= (loop*65000+cnt)/47000*2*60;
	printf("cnt %d \t loop: %d \r \n",cnt,loop);
	loop=0; cnt=0; preCnt=0;
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	reset_tick();
	return rp;
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
