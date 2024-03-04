/*
 * motor.h
 *
 *  Created on: Mar 1, 2024
 *      Author: quocv
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#define High 20
#define Low 80
#define Normal 60
#define Stop 100
#define Straight_1 0
#define Back_1 1
#define Straight_2 1
#define Back_2 0

typedef struct _motor_t{
	int speed_right;
	int speed_left;
	int direction_right;
	int direction_left;
}_motor_t;
typedef enum {RIGHT, LEFT, STRAIGHT, BACK, STOP} direction_t;
extern void Direction(direction_t state);

#endif /* INC_MOTOR_H_ */
