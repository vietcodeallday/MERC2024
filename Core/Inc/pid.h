#ifndef PID_H_
#define PID_H_


typedef enum
	{
	Anti_windup_disabled=0,
	Anti_windup_enabled
	}Anti_windup_t;


typedef struct
	{
	float Kp;
	float Ki;
	float Kd;
	float Ts;
	float Set_point_motor_1;
	float Set_point_motor_2;
	float Set_point_motor_3;
	float Anti_windup_error;
	float Outmin;
	float Outmax;
	int Anti_windup;

	}PID_Param_t;


extern void PID_init(PID_Param_t *par);
extern float PID_Calculation(int motor, float input);

#endif /* PID_H_ */
