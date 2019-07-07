/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include "usart.h"
#include "gpio.h"
#include "common.h"

/* USER CODE BEGIN 0 */
#include "motor.h"	//for motor

uint8_t UART2_TxBuffer[12];		
uint8_t UART2_RxBuffer[10];		
CMD_HandlerTypeDef UART2_CMD;
/* USER CODE END 0 */

UART_HandleTypeDef huart2;		// Externed in common.h
UART_HandleTypeDef huart3;		// Externed in common.h

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_MultiProcessor_Init(&huart3, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
    /**USART3 GPIO Configuration    
    PB11     ------> USART3_RX
    PD8     ------> USART3_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PB11     ------> USART3_RX
    PD8     ------> USART3_TX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8);

  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
/*
*** Interrupt mode IO operation ***    
	===================================
	[..]    
		(+) Send an amount of data in non blocking mode using HAL_UART_Transmit_IT() 
		(+) At transmission end of transfer HAL_UART_TxCpltCallback is executed and user can 
				 add his own code by customization of function pointer HAL_UART_TxCpltCallback
		(+) Receive an amount of data in non blocking mode using HAL_UART_Receive_IT() 
		(+) At reception end of transfer HAL_UART_RxCpltCallback is executed and user can 
				 add his own code by customization of function pointer HAL_UART_RxCpltCallback
		(+) In case of transfer Error, HAL_UART_ErrorCallback() function is executed and user can 
				 add his own code by customization of function pointer HAL_UART_ErrorCallback
*/
//@brief  Rx Transfer completed callbacks.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{	
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);	//Toggle PooNim's on-board Red LED, PD14
	
	UART2_CMD = SerialReceiveCMD();

/*
	uint8_t aByte[2];
	
	//Transmit data back?
	UART2_TxBuffer[0] = 0x5B;	// '['
	UART2_TxBuffer[1] = 0x61;	// 'a'
	//Send Encoder count back
	int16Conv(motor1.Encoder_value, aByte);
	UART2_TxBuffer[2] = aByte[0];
	UART2_TxBuffer[3] = aByte[1];
	
	int16Conv(motor2.Encoder_value, aByte);
	UART2_TxBuffer[4] = aByte[0];
	UART2_TxBuffer[5] = aByte[1];
	
	int16Conv(motor3.Encoder_value, aByte);
	UART2_TxBuffer[6] = aByte[0];
	UART2_TxBuffer[7] = aByte[1];
	
	int16Conv(motor4.Encoder_value, aByte);
	UART2_TxBuffer[8] = aByte[0];
	UART2_TxBuffer[9] = aByte[1];
	
	UART2_TxBuffer[10] = 0x0D;
	UART2_TxBuffer[11] = 0x5D;	// ']'
	//Send out data
	//HAL_UART_Transmit_IT(&huart2, (uint8_t *)UART2_TxBuffer, sizeof(UART2_TxBuffer));
	*/
}

//@brief  Tx Transfer completed callbacks.
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);	//Toggle Blue LED
}

CMD_HandlerTypeDef SerialReceiveCMD(void)
{
	//UART2_RxBuffer is a global variables
	CMD_HandlerTypeDef recv_cmd;
	
	//Check Head and Tail
	if(UART2_RxBuffer[0] == 0x5B && UART2_RxBuffer[9] == 0x5D){
		//Head and Tail
		recv_cmd.head = UART2_RxBuffer[0];
		recv_cmd.tail = UART2_RxBuffer[9];
		//Command ID
		recv_cmd.cmd_id = UART2_RxBuffer[1];
		//Data package
		recv_cmd.data[0] = UART2_RxBuffer[2];
		recv_cmd.data[1] = UART2_RxBuffer[3];
		recv_cmd.data[2] = UART2_RxBuffer[4];
		recv_cmd.data[3] = UART2_RxBuffer[5];
		recv_cmd.data[4] = UART2_RxBuffer[6];
		recv_cmd.data[5] = UART2_RxBuffer[7];
		recv_cmd.data[6] = UART2_RxBuffer[8];
	}else{
		printf("Frame error!\n");
		printf("%i | %i |%i |%i |%i |%i |%i |%i |%i |%i |\n", recv_cmd.head, recv_cmd.cmd_id,
			recv_cmd.data[0], recv_cmd.data[1], recv_cmd.data[2], recv_cmd.data[3],
			recv_cmd.data[4], recv_cmd.data[5], recv_cmd.data[6], recv_cmd.tail);

		UART2_TxBuffer[0] = UART2_RxBuffer[0];
		UART2_TxBuffer[1] = UART2_RxBuffer[1];
		UART2_TxBuffer[2] = UART2_RxBuffer[2];
		UART2_TxBuffer[3] = UART2_RxBuffer[3];
		UART2_TxBuffer[4] = UART2_RxBuffer[4];
		UART2_TxBuffer[5] = UART2_RxBuffer[5];
		UART2_TxBuffer[6] = UART2_RxBuffer[6];
		UART2_TxBuffer[7] = UART2_RxBuffer[7];
		UART2_TxBuffer[8] = UART2_RxBuffer[8];
		UART2_TxBuffer[9] = UART2_RxBuffer[9];
		HAL_UART_Transmit_IT(&huart2, (uint8_t *)UART2_TxBuffer, sizeof(UART2_TxBuffer));
		
	}
	
	return recv_cmd;
}


CMD_HandlerTypeDef GetSerialCMD(void)
{
	return UART2_CMD;
}


void int16Conv(int16_t val_16, uint8_t *aByte)
{
    aByte[0] = (uint8_t)(val_16);
    aByte[1] = (uint8_t)(val_16 >> 8);
}

int16_t uint8Conv(uint8_t *aByte)
{
    int16_t result = ((int16_t)(aByte[1]) << 8) | ((int16_t)(aByte[0]));
    return result;
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
