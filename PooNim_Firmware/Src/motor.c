#include "motor.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

#include "tim.h"	//For timer handler

#define PWM12_MAX	33599
#define PWM34_MAX	16799

MOTOR_HandlerTypeDef motor1;
MOTOR_HandlerTypeDef motor2;
MOTOR_HandlerTypeDef motor3;
MOTOR_HandlerTypeDef motor4;

int16_t calEncoder_value(int Encoder_feedback, int Direction_feedback)
{
	// From Encoder_feedback and Direction_feedback calculate Encoder_value
	// which is signed value, indicated direction of turning
	static int16_t Encoder_value;
	
	if(Encoder_feedback == 0)
	{
		Encoder_value = 0;
	}else{
		if(Direction_feedback == 0)
		{
			//Clock-wise direction
			Encoder_value = Encoder_feedback;
		}else{
			//Counter clock-wise direction
			Encoder_value = -1*(2048 - Encoder_feedback);
		}
	}
	return Encoder_value;
}

void MotorUpdate_Encoder(void)
{
	//Timer 1 for motor1
	motor1.Encoder_feedback = __HAL_TIM_GetCounter(&htim1);
	motor1.Direction_feedback = __HAL_TIM_DIRECTION_STATUS(&htim1);
	motor1.Encoder_value = calEncoder_value(motor1.Encoder_feedback, motor1.Direction_feedback);
	motor1.flag_update = 1;	//Set update flag
	
	//Timer 2 for motor2
	motor2.Encoder_feedback = __HAL_TIM_GetCounter(&htim2);
	motor2.Direction_feedback = __HAL_TIM_DIRECTION_STATUS(&htim2);
	motor2.Encoder_value = calEncoder_value(motor2.Encoder_feedback, motor2.Direction_feedback);
	motor2.flag_update = 1;	//Set update flag
	
	//Timer 3 for motor3
	motor3.Encoder_feedback = __HAL_TIM_GetCounter(&htim3);
	motor3.Direction_feedback = __HAL_TIM_DIRECTION_STATUS(&htim3);
	motor3.Encoder_value = calEncoder_value(motor3.Encoder_feedback, motor3.Direction_feedback);
	motor3.flag_update = 1;	//Set update flag
	
	//Timer 4 for motor4
	motor4.Encoder_feedback = __HAL_TIM_GetCounter(&htim4);
	motor4.Direction_feedback = __HAL_TIM_DIRECTION_STATUS(&htim4);
	motor4.Encoder_value = calEncoder_value(motor4.Encoder_feedback, motor4.Direction_feedback);
	motor4.flag_update = 1;	//Set update flag
}

void Update_Encoder_value(MOTOR_HandlerTypeDef *motor){
	//Calculate Encoder value for PID controller to use
	//Motor stopped, value always been 0
	if(motor->Encoder_feedback == 0){
		motor->Encoder_value = 0;
	}else{
		//Motor moved
		if(motor->Direction_feedback == 0){
			//Clock-wise direction
			motor->Encoder_value = motor->Encoder_feedback;
		}else{
			//Counter clock-wise direction
			motor->Encoder_value = -1*(2048 - motor->Encoder_feedback);
		}
	}
}

void MotorInit(MOTOR_HandlerTypeDef *motor, int ID){
	motor->ID = ID;
	motor->Encoder_value = 0;
	motor->Error = 0.0;
	motor->Error_acc = 0.0;
	
	//Dead band of the motor
	motor->DeadBand_CW = 0.0;
	motor->DeadBand_CCW = 0.0;
	
	//Initial value of PID gain
	motor->Kp = 0.002;
	motor->Ki = 0.0000002;
	motor->Kd = 0.0;
}

void MotorSet_PIDgain(MOTOR_HandlerTypeDef *motor, float Kp, float Ki, float Kd){
	motor->Kp = Kp;
	motor->Ki = Ki;
	motor->Kd = Kd;
}

void MotorSet_Setpoint(MOTOR_HandlerTypeDef *motor, signed int setPoint){
	motor->Setpoint = setPoint;
}

void MotorSet_speed(MOTOR_HandlerTypeDef *motor, float PWM){
	//Motor rotation follow left-hand rule
	int PWM_set = 0;
	//PWM value input from -1.0 to 1.0
	if(PWM > 1.0f) PWM = 1.0f;
	if(PWM < -1.0f) PWM = -1.0f;
	
	switch(motor->ID){
		case 1: //MotorDrive 3	
			if(PWM > 0.0f){
			//Counter Clock-wise direction, positive PWM
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);	//Dir3_A -- PC4
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);		//Dir3_B -- PC5
			PWM_set = (int)((PWM + motor->DeadBand_CW) * PWM34_MAX);
			}
			if(PWM == 0.0f){
				//Brake motor
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		//Dir3_A -- PC4
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);		//Dir3_B -- PC5
				PWM_set = 0;
			}
			if(PWM < 0.0f){
				//Clock-wise direction, negative PWM
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);		//Dir3_A -- PC4
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);	//Dir3_B -- PC5
				PWM_set = (int)((-1*PWM - motor->DeadBand_CCW) * PWM34_MAX);
			}

			//printf("%f \t %i \n", PWM, PWM_set);
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, PWM_set);	//PWM3 -- PB14
			break;//Case 1
			
		case 2: //MotorDrive 1
			if(PWM > 0.0f){
			//Counter Clock-wise direction, positive PWM
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);	//Dir1_A -- PE12
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);		//Dir1_B -- PE13
			PWM_set = (int)((PWM + motor->DeadBand_CW) * PWM12_MAX);
			}
			if(PWM == 0.0f){
				//Brake motor
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);		//Dir1_A -- PE12
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);		//Dir1_B -- PE13
				PWM_set = 0;
			}
			if(PWM < 0.0f){
				//Clock-wise direction, negative PWM
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);		//Dir1_A -- PE12
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);	//Dir1_B -- PE13
				PWM_set = (int)((-1*PWM - motor->DeadBand_CCW) * PWM12_MAX);
			}

			//printf("%f \t %i \n", PWM, PWM_set);
			__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, PWM_set);	//PWM1 -- PE5
			break;//Case 2
		
		case 3: //MotorDrive 2
			if(PWM > 0.0f){
			//Counter Clock-wise direction, positive PWM
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);	//Dir2_A -- PE14
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);		//Dir2_B -- PE15
			PWM_set = (int)((PWM + motor->DeadBand_CW) * PWM12_MAX);
			}
			if(PWM == 0.0f){
				//Brake motor
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);		//Dir2_A -- PE14
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);		//Dir2_B -- PE15
				PWM_set = 0;
			}
			if(PWM < 0.0f){
				//Clock-wise direction, negative PWM
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);		//Dir2_A -- PE14
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);	//Dir2_B -- PE15
				PWM_set = (int)((-1*PWM - motor->DeadBand_CCW) * PWM12_MAX);
			}

			//printf("%f \t %i \n", PWM, PWM_set);
			__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, PWM_set);	//PWM2 -- PE6
			break;//Case 3
		
		case 4: //MotorDrive 4
			if(PWM > 0.0f){
			//Counter Clock-wise direction, positive PWM
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);	//Dir4_A -- PE7
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);		//Dir4_B -- PE8
			PWM_set = (int)((PWM + motor->DeadBand_CW) * PWM34_MAX);
			}
			if(PWM == 0.0f){
				//Brake motor
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);		//Dir4_A -- PE7
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);		//Dir4_B -- PE8
				PWM_set = 0;
			}
			if(PWM < 0.0f){
				//Clock-wise direction, negative PWM
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);		//Dir4_A -- PE7
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);	//Dir4_B -- PE8
				PWM_set = (int)((-1*PWM - motor->DeadBand_CCW) * PWM34_MAX);
			}

			//printf("%f \t %i \n", PWM, PWM_set);
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, PWM_set);	//PWM4 -- PB15
			break;//Case 4
	}//END Switch case
	
}

void MotorControl_PID(MOTOR_HandlerTypeDef *motor){
	float P_out, I_out, D_out;

	//Motor Control
	motor->Error = motor->Setpoint - motor->Encoder_value;
	
	//Prevent motor from going crazy when error is too high.
	if((motor->Error >= 150) || (motor->Error <= -150)){
		//Command to drive motor
		MotorSet_speed(motor, 0.0);
		motor->Error = 0;
		motor->Error_acc = 0;
		
	}else{
		P_out = (float)(1.0f*motor->Kp*motor->Error);
		I_out = (float)(1.0f*motor->Ki*motor->Error_acc);
		//Implement D-term later
		D_out = 0.0;
		
		//Command to drive motor
		MotorSet_speed(motor, P_out + I_out + D_out);
		
		//Accumulate error
		motor->Error_acc = motor->Error_acc + motor->Error;
	}
}

void MotorControl_GetDeadBand(MOTOR_HandlerTypeDef *motor, float set_speed_cw, float set_speed_ccw){
	set_speed_cw = 0.0f;
	set_speed_ccw = 0.0f;
	
	while(motor->Encoder_value <= 1){
		MotorSet_speed(motor, 0.0f);
		HAL_Delay(100);
		set_speed_cw += 0.01f;
		MotorSet_speed(motor, set_speed_cw);
		//printf("%i\t%0.2f\n", motor->ID, set_speed_cw);
		HAL_Delay(500);
	}
	//Set motor property
	motor->DeadBand_CW = set_speed_cw;
	MotorSet_speed(motor, set_speed_cw);
	HAL_Delay(10000);	//1000 ms delay
	printf("Clock-wise tuning DONE! Motor ID: %i\t CW: %0.2f\n", motor->ID, set_speed_cw);
	
	while(motor->Encoder_value >= -1){
		MotorSet_speed(motor, 0.0f);
		HAL_Delay(100);
		set_speed_ccw -= 0.01f;
		MotorSet_speed(motor, set_speed_ccw);
		//printf("%i\t%0.2f\n", motor->ID, set_speed_ccw);
		HAL_Delay(500);
	}
	//Set motor property
	motor->DeadBand_CCW = set_speed_ccw;
	MotorSet_speed(motor, set_speed_ccw);
	HAL_Delay(1000);	//1000 ms delay
	printf("Counter clock-wise tuning DONE! Motor ID: %i\t CCW: %0.2f\n", motor->ID, set_speed_ccw);
	
}
