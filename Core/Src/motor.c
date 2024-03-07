/*
 * motor.c
 *
 *  Created on: Mar 1, 2024
 *      Author: quocv
 */

/**
 * MOTOR 1 |CHA-PC10 |CHB-PC12 |PWM-PB7| Direct-PC3
 * MOTOR 2 |CHA-PC8 |CHB-PC5 |PWM-PA5| Direct-PA12
 * MOTOR 3 |CHA-PB3 |CHB-PA2 |PWM-PB5| Direct-PA3
 */
#include "motor.h"
#include "main.h"
#include "math.h"

//static direction_t state;
//static _motor_t motor={};

//void Run(void){
//	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 80);
//	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 20);
//	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 80);
//
////	HAL_GPIO_WritePin(DIRECTION_1_GPIO_Port, DIRECTION_1_Pin, motor.direction_left);
////	HAL_GPIO_WritePin(DIRECTION_2_GPIO_Port, DIRECTION_2_Pin, motor.direction_right);
//}


void Robot_Move_PID(float Vd, float Theta, float Vtheta){
	float V1, V2, V3;
	double V1_abs, V2_abs, V3_abs, Vmax, Temp;

	V1=Vd*(-0.87*cos(Theta*PI/180)-0.5*sin(Theta*PI/180));
	V2=Vd*(0.87*cos(Theta*PI/180)-0.5*sin(Theta*PI/180));
	V3=Vd*sin(Theta*PI/180);

	V1_abs=fabs(V1);
	V2_abs=fabs(V2);
	V3_abs=fabs(V3);

	Vmax=V1_abs;
	Vmax = (V2_abs > Vmax) ? V2_abs : Vmax;
	Vmax = (V3_abs > Vmax) ? V3_abs : Vmax;

	if (Vmax>10000){
		Temp=10000/Vmax;
		V1= V1*Temp;
		V2= V2*Temp;
		V3= V3*Temp;
	}
	else if (Vmax<10000){
		Temp = fabs(Vd)>fabs(Vtheta) ? fabs(Vd) : fabs(Vtheta);

		Temp=Temp/Vmax;
		V1= V1*Temp;
		V2= V2*Temp;
		V3= V3*Temp;
	 }
	 V1 = Vmax*V1/10000;
	 V2 = Vmax*V2/10000;
	 V3 = Vmax*V3/10000;
	 if (V1<0){
		 //int PWM_1=Caculate_PWM((int)V1);
		 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 20);
		 HAL_GPIO_WritePin(DIRECTION_1_GPIO_Port, DIRECTION_1_Pin, CLOCK_WISE);
	 }
}
int Caculate_PWM(int V){
	int Duty_cycle;
	Duty_cycle=V;
	return Duty_cycle;
}
