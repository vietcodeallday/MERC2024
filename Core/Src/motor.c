/*
 * motor.c
 *
 *  Created on: Mar 1, 2024
 *      Author: quocv
 */

/**
 * MOTOR 1 |CHA-PC10 |CHB-PC12 |PWM-PB7| Direct-PC3
 * MOTOR 2 |CHA-PC8 |CHB-PC5 |PWM-PA5| Direct-PA12
 * MOTOR 3 |CHA-PB3 |CHB-PA2 |PWM-PB5| Direct-PC0
 */
#include "motor.h"

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

float V1, V2, V3=0;


void Robot_Move(float Vd, float Theta, float Vtheta){
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


}
double rpm_to_duty(double rpm){
	return (98.15-0.6*rpm);
}
void set_duty_cycle(int motor, double rpm, double out){
	rpm=rpm+out;
	double duty = rpm_to_duty(rpm);
	if(motor==MOTOR_1){
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, duty);
	}
	if(motor==MOTOR_2){
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, duty);
	}
	if(motor==MOTOR_3){
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, duty);
	}
}
