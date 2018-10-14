/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#include "motor.h"	//For motor handler
#include "tim.h"		//For timer handler
#include "gpio.h"		//For buttons

extern uint8_t UART2_TxBuffer[8];
extern uint8_t UART2_RxBuffer[8];
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart2;
/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line2 interrupt.
*/
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	//This is interrupt for Button_R (Red)
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */
	//ButtonState need to be clear by another process
	button_R.PressedCount++;
	button_R.ButtonState = true;
  /* USER CODE END EXTI2_IRQn 1 */
}

/**
* @brief This function handles EXTI line3 interrupt.
*/
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
	//This is interrupt for Button_O (Orange)
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */
	//ButtonState need to be clear by another process
	button_O.PressedCount++;
	button_O.ButtonState = true;
  /* USER CODE END EXTI3_IRQn 1 */
}

/**
* @brief This function handles EXTI line4 interrupt.
*/
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	//This is interrupt for Button_G (Green)
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
	button_G.PressedCount++;
	button_G.ButtonState = true;
  /* USER CODE END EXTI4_IRQn 1 */
}

/**
* @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
*/
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);	//Toggle Blue LED
  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	//This is interrupt for Button_B (Blue)
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
	button_B.PressedCount++;
	button_B.ButtonState = true;
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
	//This should keep receive interrupt alive after routine get called.
	//Interupt every time UART2 receive 8 byte of data
	HAL_UART_Receive_IT(&huart2, (uint8_t *)UART2_RxBuffer, sizeof(UART2_RxBuffer));
  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles TIM5 global interrupt.
*/
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
	/**
	Timer 5 setup as internal interrupt. This function will get call
	every 0.05 sec (20 Hz)
	**/
  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);	//Toggle Discovery's on-board Red LED, PD14
	
	//Update Encoder count
	motor1.Encoder_feedback = __HAL_TIM_GetCounter(&htim1);
	motor1.Direction_feedback = __HAL_TIM_DIRECTION_STATUS(&htim1);
	Update_Encoder_value(&motor1);
	motor1.flag_update = 1;	//Set update flag
	__HAL_TIM_SetCounter(&htim1, 0);
	
	motor2.Encoder_feedback = __HAL_TIM_GetCounter(&htim2);
	motor2.Direction_feedback = __HAL_TIM_DIRECTION_STATUS(&htim2);
	Update_Encoder_value(&motor2);
	motor2.flag_update = 1;	//Set update flag
	__HAL_TIM_SetCounter(&htim2, 0);
	
	motor3.Encoder_feedback = __HAL_TIM_GetCounter(&htim3);
	motor3.Direction_feedback = __HAL_TIM_DIRECTION_STATUS(&htim3);
	Update_Encoder_value(&motor3);
	motor3.flag_update = 1;	//Set update flag
	__HAL_TIM_SetCounter(&htim3, 0);
	
	motor4.Encoder_feedback = __HAL_TIM_GetCounter(&htim4);
	motor4.Direction_feedback = __HAL_TIM_DIRECTION_STATUS(&htim4);
	Update_Encoder_value(&motor4);
	motor4.flag_update = 1;	//Set update flag
	__HAL_TIM_SetCounter(&htim4, 0);
  /* USER CODE END TIM5_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
