#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

typedef struct 
{
	int ID;															//ID of the motor
	volatile int Encoder_feedback;			//Update by timer routine (unsigned value 0 - 2048)
	volatile int Direction_feedback;		//Update by timer routine (0 or 1)
	
	float DeadBand_CW;
	float DeadBand_CCW;
	
	signed int Encoder_value;						//Encoder value with direction
	signed int Setpoint;								//Desired speed of the motor
	float Kp;
	float Ki;
	float Kd;
	float Error;
	float Error_acc;
	
	uint32_t ADC_raw;
	volatile int flag_update;
}MOTOR_HandlerTypeDef;

extern MOTOR_HandlerTypeDef motor1;
extern MOTOR_HandlerTypeDef motor2;
extern MOTOR_HandlerTypeDef motor3;
extern MOTOR_HandlerTypeDef motor4;

void Update_Encoder_value(MOTOR_HandlerTypeDef *motor);
void MotorInit(MOTOR_HandlerTypeDef *motor, int ID);
void MotorSet_PIDgain(MOTOR_HandlerTypeDef *motor, float Kp, float Ki, float Kd);
void MotorSet_Setpoint(MOTOR_HandlerTypeDef *motor, signed int setPoint);
void MotorSet_speed(MOTOR_HandlerTypeDef *motor, float PWM);
void MotorControl_PID(MOTOR_HandlerTypeDef *motor);
void MotorControl_GetDeadBand(MOTOR_HandlerTypeDef *motor, float set_speed_cw, float set_speed_ccw);
#endif /* __MOTOR_H */
