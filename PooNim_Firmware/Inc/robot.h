#ifndef __ROBOT_H
#define __ROBOT_H

typedef struct {
	float vel_x;		//translation velocity forward direction
	float vel_y;		//translation velocity side-way direction
	float rot_w;		//rotation speed
}PooNim_HandlerTypeDef;

typedef struct {
	float wheel_1;	
	float wheel_2;
	float wheel_3;
	float wheel_4;
}Wheel_HandlerTypeDef;

extern PooNim_HandlerTypeDef PooNim_CMD;

void Robot_CalWheelSpeed(PooNim_HandlerTypeDef *PooNim_CMD, float *WheelSpeed);
#endif /* __ROBOT_H */
