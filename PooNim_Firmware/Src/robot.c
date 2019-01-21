#include "robot.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

#define ROBOT_RADIUS  0.1025f		//in meter, From center to wheel = 10.25 cm.
#define WHEEL_RADIUS	0.05f		//in meter, Wheel diameter = 100 mm.

#define SIN45					0.7071f	//Value of Sin(45) = 0.7071
#define COS45					0.7071f	//Value of Cos(45) = 0.7071

PooNim_HandlerTypeDef PooNim_CMD;

void Robot_CalWheelSpeed(PooNim_HandlerTypeDef *PooNim_CMD, float *WheelSpeed)
{
	//Vx is forward direction
	//Vy is translate to the right direction
	//Motor rotation follow left-hand rule
	float vel_x, vel_y, rot_w;

	vel_x = PooNim_CMD->vel_x;
	vel_y = PooNim_CMD->vel_y;
	rot_w = PooNim_CMD->rot_w;

	WheelSpeed[0] = (1.0f*COS45*vel_x)	+ (-1.0f*SIN45*vel_y)	+ (1.0f*ROBOT_RADIUS*rot_w);
	WheelSpeed[1] = (-1.0f*SIN45*vel_x)	+ (-1.0f*COS45*vel_y)	+ (1.0f*ROBOT_RADIUS*rot_w);
	WheelSpeed[2] = (-1.0f*COS45*vel_x)	+ (1.0f*SIN45*vel_y)	+ (1.0f*ROBOT_RADIUS*rot_w);
	WheelSpeed[3] = (1.0f*SIN45*vel_x)	+ (1.0f*COS45*vel_y)	+ (1.0f*ROBOT_RADIUS*rot_w);

	
	//This is a dummy code, just for test communication and control motor
	for(int i = 0; i < 4; i++)
	{
		//Limit WheelSpeed not to exceed -1.0 and 1.0
		if(WheelSpeed[i] > 1.0f) WheelSpeed[i] = 1.0f;
		if(WheelSpeed[i] < -1.0f) WheelSpeed[i] = -1.0f;
	}
}
