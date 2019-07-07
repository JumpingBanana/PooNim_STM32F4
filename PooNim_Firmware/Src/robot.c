#include "robot.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

#define ROBOT_RADIUS  0.1025f		// in meter, From center to wheel = 10.25 cm.
#define WHEEL_RADIUS	0.05f			// in meter, Wheel diameter = 100 mm.
#define MAX_ENCODER_SPEED		460	// value of encoder count when motor run at full speed

#define SIN45					0.7071f	//Value of Sin(45) = 0.7071
#define COS45					0.7071f	//Value of Cos(45) = 0.7071

PooNim_HandlerTypeDef PooNim_CMD;

void Robot_CalWheelSpeed(PooNim_HandlerTypeDef *PooNim_CMD, signed int *WheelSpeed_ENC)
{
	//Vx is forward direction
	//Vy is translate to the right direction
	//Motor rotation follow left-hand rule
	float vel_x, vel_y, rot_w;
	float WheelSpeed_0, WheelSpeed_1, WheelSpeed_2, WheelSpeed_3;
	vel_x = PooNim_CMD->vel_x;
	vel_y = PooNim_CMD->vel_y;
	rot_w = PooNim_CMD->rot_w;

	// Calculate wheel speed in range of [-1.0 to 1.0] value
	WheelSpeed_0 = (1.0f*COS45*vel_x)	+ (-1.0f*SIN45*vel_y)	+ (1.0f*ROBOT_RADIUS*rot_w);
	WheelSpeed_1 = (-1.0f*SIN45*vel_x)	+ (-1.0f*COS45*vel_y)	+ (1.0f*ROBOT_RADIUS*rot_w);
	WheelSpeed_2 = (-1.0f*COS45*vel_x)	+ (1.0f*SIN45*vel_y)	+ (1.0f*ROBOT_RADIUS*rot_w);
	WheelSpeed_3 = (1.0f*SIN45*vel_x)	+ (1.0f*COS45*vel_y)	+ (1.0f*ROBOT_RADIUS*rot_w);

	// To use PID, need to scale it to be in range of Encoder value count
	WheelSpeed_ENC[0] = (signed int)(WheelSpeed_0 * MAX_ENCODER_SPEED);
	WheelSpeed_ENC[1] = (signed int)(WheelSpeed_1 * MAX_ENCODER_SPEED);
	WheelSpeed_ENC[2] = (signed int)(WheelSpeed_2 * MAX_ENCODER_SPEED);
	WheelSpeed_ENC[3] = (signed int)(WheelSpeed_3 * MAX_ENCODER_SPEED);
	
	//Limit WheelSpeed not to exceed MAX_ENCODER_SPEED
	for(int i = 0; i < 4; i++)
	{
		if(WheelSpeed_ENC[i] > 1*MAX_ENCODER_SPEED) 
		{
			WheelSpeed_ENC[i] = (signed int)(1*MAX_ENCODER_SPEED);
		}
		if(WheelSpeed_ENC[i] < -1*MAX_ENCODER_SPEED) 
		{
			WheelSpeed_ENC[i] = (signed int)(-1*MAX_ENCODER_SPEED);
		}
	}
	
}
