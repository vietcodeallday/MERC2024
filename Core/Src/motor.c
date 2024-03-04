/*
 * motor.c
 *
 *  Created on: Mar 1, 2024
 *      Author: quocv
 */

#include "motor.h"
#include "main.h"


static direction_t state;
static _motor_t motor={};

void Run(void){
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, motor.speed_right);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, motor.speed_left);
	HAL_GPIO_WritePin(DIRECTION_1_GPIO_Port, DIRECTION_1_Pin, motor.direction_left);
	HAL_GPIO_WritePin(DIRECTION_2_GPIO_Port, DIRECTION_2_Pin, motor.direction_right);
}
void init_State(void){
	state= STOP;
}
void state_Update(direction_t state){
	switch (state){
	case STOP:
		motor.speed_right=Stop;
		motor.speed_left=Stop;
		motor.direction_right=Straight_2;
		motor.direction_left=Straight_1;
		break;
	case RIGHT:
		motor.speed_right=Low;
		motor.speed_left=High;
		motor.direction_right=Back_2;
		motor.direction_left=Straight_1;
		break;
	case LEFT:
		motor.speed_right=High;
		motor.speed_left=Low;
		motor.direction_right=Straight_2;
		motor.direction_left=Back_1;
		break;
	case STRAIGHT:
		motor.speed_right=High;
		motor.speed_left=High;
		motor.direction_right=Straight_2;
		motor.direction_left=Straight_1;
		break;
	case BACK:
		motor.speed_right=Low;
		motor.speed_left=Low;
		motor.direction_right=Back_2;
		motor.direction_left=Back_1;
		break;
	}
}

extern void Direction(direction_t state){
	state_Update(state);
	Run();
}
