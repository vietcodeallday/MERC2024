#ifndef PID_H_
#define PID_H_


typedef enum
	{
	Anti_windup_disabled=0,
	Anti_windup_enabled
	}Anti_windup_t;


typedef struct
	{
	double Kp;
	double Ki;
	double Kd;
	double Ts;
	double Set_point_motor_1;
	double Set_point_motor_2;
	double Set_point_motor_3;
	double Anti_windup_error;
	double Outmin;
	double Outmax;
	int Anti_windup;

	}PID_Param_t;
extern int ble;
extern double Outmin, Outmax;

extern void PID_init(PID_Param_t *par);
extern double PID_Calculation(int motor, double input);
extern void PID(void);
#endif /* PID_H_ */
