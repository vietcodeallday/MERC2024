#include "pid.h"
#include "math.h"
#include "stdlib.h"
#include "main.h"
double Kp,Ki,Kd,Ts,Outmin,Outmax
,set_point_motor_1
,set_point_motor_2
,set_point_motor_3
,set_point
,antiwinduperror=0;
int windup=0;
double error
,prev_input_1
,prev_input_2
,prev_input_3
,prev_input
,Ki_sum
,Ki_sum_1
,Ki_sum_2
,Ki_sum_3=0;


void PID_init(PID_Param_t *par){
	Kp=par->Kp;
	Ki=par->Ki;
	Kd=par->Kd;
	Ts=(par->Ts);
	set_point_motor_1=par->Set_point_motor_1;
	set_point_motor_2=par->Set_point_motor_2;
	set_point_motor_3=par->Set_point_motor_3;
	antiwinduperror=par->Anti_windup_error;
	Outmin=par->Outmin;
	Outmax=par->Outmax;
	windup=par->Anti_windup;

	if(0==par->Anti_windup_error){antiwinduperror=10;}
	}


double PID_Calculation(int motor, double input)
	{
//	set_point=(motor==MOTOR_1)? set_point_motor_1: ((motor==MOTOR_2)? set_point_motor_2:set_point_motor_3);
//	prev_input = (motor==MOTOR_1)? prev_input_1:((motor==MOTOR_2)? prev_input_2:prev_input_3);

	switch(motor){
	case MOTOR_1:
		set_point=set_point_motor_1;
		prev_input=prev_input_1;
		Ki_sum=Ki_sum_1;
		break;
	case MOTOR_2:
		set_point=set_point_motor_2;
		prev_input=prev_input_2;
		Ki_sum=Ki_sum_2;
		break;
	case MOTOR_3:
		set_point=set_point_motor_3;
		prev_input=prev_input_3;
		Ki_sum=Ki_sum_3;
		break;
	}

	error=(set_point-input);

	printf("error = %f \r\n\n", error);
	double out;
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
//	if(out>Outmax){out=Outmax;}
//	if(out<Outmin){out=Outmin;}
	if(motor==MOTOR_1){
		Ki_sum_1=Ki_sum;
		prev_input_1=input;
	}else if(motor==MOTOR_2){
		Ki_sum_2=Ki_sum;
		prev_input_2=input;
	}else{
		Ki_sum_3=Ki_sum;
		prev_input_3=input;
	}
	out /= 50;
	return out;
	}
