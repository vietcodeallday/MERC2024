#include "pid.h"
#include "math.h"
#include "stdlib.h"
#include "main.h"
float Kp,Ki,Kd,Ts,Outmin,Outmax
,set_point_motor_1
,set_point_motor_2
,set_point_motor_3
,set_point
,antiwinduperror;
int windup;
float error,prev_input,Ki_sum;


void PID_init(PID_Param_t *par){
	Kp=par->Kp;
	Ki=par->Ki;
	Kd=par->Kd;
	Ts=(par->Ts)/1000;
	set_point_motor_1=par->Set_point_motor_1;
	set_point_motor_2=par->Set_point_motor_2;
	set_point_motor_3=par->Set_point_motor_3;
	antiwinduperror=par->Anti_windup_error;
	Outmin=par->Outmin;
	Outmax=par->Outmax;
	windup=par->Anti_windup;

	if(0==par->Anti_windup_error){antiwinduperror=10;}
	}


float PID_Calculation(int motor, float input)
	{
	set_point=(motor==MOTOR_1)? set_point_motor_1: ((motor==MOTOR_2)? set_point_motor_2:set_point_motor_3);
	error=(set_point-input);

	float out;
	if(Anti_windup_enabled==windup)
		{

		if(antiwinduperror<fabs(error))
			{
			out=Kp*(error)+Kd*(input-prev_input)/Ts;
			}
		else
			{
			out=(Kp*(error)) +( Ki*(Ki_sum)*Ts) -( Kd*(input-prev_input)/Ts);
			}

		}

	else
		{
		out=Kp*(error) + Ki*(Ki_sum)*Ts - Kd*(input-prev_input)/Ts;
		}
	Ki_sum=Ki_sum+(Ki_sum);
	if(out>Outmax){out=Outmax;}
	if(out<Outmin){out=Outmin;}
	prev_input=input;
	return out;
	}
