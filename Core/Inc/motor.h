/*
 * motor.h
 *
 *  Created on: Mar 1, 2024
 *      Author: quocv
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_
#include "main.h"
#include "math.h"
#include "pid.h"
#include "delay.h"
#include <stdio.h>
extern void set_duty_cycle(double rpm, double out);
extern void Robot_Move(float Vd, float Theta, float Vtheta);

#define CLOCK_WISE 1
#define PI 3.141592654

extern float V1, V2, V3;

#endif /* INC_MOTOR_H_ */
