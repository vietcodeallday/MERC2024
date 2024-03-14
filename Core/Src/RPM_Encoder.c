
#include "RPM_Encoder.h"
	double rpm;
	uint16_t cnt=0;
	uint16_t preCnt=0;
	int loop=0;
double get_rpm(int motor){
	reset_tick();
	switch (motor){
		case MOTOR_1:
			__HAL_TIM_SET_COUNTER(&htim1, 0);
			break;
		case MOTOR_2:
			__HAL_TIM_SET_COUNTER(&htim3, 0);
			break;
		case MOTOR_3:
			__HAL_TIM_SET_COUNTER(&htim4, 0);
			break;
	}
	volatile uint16_t a=millis();

	while(millis()-a<=250){
		switch (motor){
			case MOTOR_1:
				cnt = __HAL_TIM_GET_COUNTER(&htim1);
				break;
			case MOTOR_2:
				cnt = __HAL_TIM_GET_COUNTER(&htim3);
				break;
			case MOTOR_3:
				cnt = __HAL_TIM_GET_COUNTER(&htim4);
				break;
		}
		if(cnt<preCnt){
			loop++;
			preCnt=cnt;
		}else{
			preCnt=cnt;
		}

	}
	rpm= ((double)loop*65000+(double)cnt)/47000*4*60;
//	printf("cnt_1 %f \r \n",((double)loop*65000+(double)cnt)/47000);
//	switch (motor){
//		case MOTOR_1:
//			printf("cnt_1 %d \t loop_1: %d \r \n",cnt,loop);
//			break;
//		case MOTOR_2:
//			printf("cnt_2 %d \t loop_2: %d \r \n",cnt,loop);
//			break;
//		case MOTOR_3:
//			printf("cnt_3 %d \t loop_3: %d \r \n",cnt,loop);
//			break;
//	}
	cnt=0;
	loop=0;
	preCnt=0;
	return rpm;
}

