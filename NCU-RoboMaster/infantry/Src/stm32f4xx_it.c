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
#include "cmsis_os.h"
#include "usart.h"
#include "Motor_USE_CAN.h"
#include "communication.h "
#include "tim.h"
#include "can.h"
/* USER CODE BEGIN 0 */
#include "pidwireless.h"
#include "Motor_USE_CAN.h"
#include "communication.h "
/* USER CODE END 0 */
extern  osThreadId RemoteDataTaskHandle;
extern  osThreadId RefereeDataTaskHandle;
extern  osThreadId	MiniPCDataTaskHandle;
/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

extern DMA_HandleTypeDef hdma_adc1;

extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
//  HAL_IncTick();
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}


/**
* @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
*/
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);

  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA2 stream4 global interrupt.
*/
void DMA2_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream4_IRQn 0 */

  /* USER CODE END DMA2_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream4_IRQn 1 */

  /* USER CODE END DMA2_Stream4_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream1 global interrupt.
*/
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */
	
  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

void USART3_IRQHandler (void)
{
	
    if(__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_RXNE) != RESET)  //接收中断
		{
			
		 // Res=(uint8_t)(huart3.Instance->DR & (uint8_t)0x00FFU);
			
			HAL_UART_RxCpltCallback(&huart3);
			
	   //RENX位在读DR寄存器操作之后就会自动清除，应该不需要这个清除函数
			__HAL_UART_CLEAR_FLAG(&huart3,UART_FLAG_RXNE);
			
		 }
    
}

void UART8_IRQHandler(void)
{
	uint8_t tmp1,tmp2;
	tmp1 = __HAL_UART_GET_FLAG(&huart8, UART_FLAG_IDLE);   //空闲中断中将已收字节数取出后，停止DMA
  tmp2 = __HAL_UART_GET_IT_SOURCE(&huart8, UART_IT_IDLE);
	
   if((tmp1 != RESET) && (tmp2 != RESET))
  { 
		__HAL_DMA_DISABLE(&hdma_uart8_rx);
		
		__HAL_UART_CLEAR_IDLEFLAG(&huart8);
		
		UART8_RX_NUM=(SizeofJY901)-(hdma_uart8_rx.Instance->NDTR);
		
		JY901_Data_Pro();
		__HAL_DMA_SET_COUNTER(&hdma_uart8_rx,SizeofJY901);
    __HAL_DMA_ENABLE(&hdma_uart8_rx);
		
	}
  HAL_UART_IRQHandler(&huart8);
  /* USER CODE BEGIN UART8_IRQn 1 */

  /* USER CODE END UART8_IRQn 1 */
}

void USART1_IRQHandler (void)
{
	 static  BaseType_t  pxHigherPriorityTaskWoken;
	uint8_t tmp1,tmp2;
	tmp1 = __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE);   //空闲中断中将已收字节数取出后，停止DMA
  tmp2 = __HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_IDLE);
	
   if((tmp1 != RESET) && (tmp2 != RESET))
  { 
		__HAL_DMA_DISABLE(&hdma_usart1_rx);
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		
			__HAL_DMA_SET_COUNTER(&hdma_usart1_rx,SizeofRemote);
			__HAL_DMA_ENABLE(&hdma_usart1_rx);
		
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN UART8_IRQn 1 */
   vTaskNotifyGiveFromISR(RemoteDataTaskHandle,&pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);			
	}
  /* USER CODE END UART8_IRQn 1 */
}

void USART2_IRQHandler (void)
{
	 static  BaseType_t  pxHigherPriorityTaskWoken;
	uint8_t tmp1,tmp2;
	tmp1 = __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE);   //空闲中断中将已收字节数取出后，停止DMA
  tmp2 = __HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_IDLE);
	
   if((tmp1 != RESET) && (tmp2 != RESET))
  { 
		__HAL_DMA_DISABLE(&hdma_usart2_rx);
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
		
			__HAL_DMA_SET_COUNTER(&hdma_usart2_rx,SizeofMinipc);
			__HAL_DMA_ENABLE(&hdma_usart2_rx);
		
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN UART8_IRQn 1 */
   vTaskNotifyGiveFromISR(MiniPCDataTaskHandle,&pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);			
	}
  /* USER CODE END UART8_IRQn 1 */
}










/**
* @brief This function handles DMA1 stream5 global interrupt.
*/
/**
* @brief This function handles DMA2 stream2 global interrupt.
*/
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE END DMA2_Stream2_IRQn 1 */
}


void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart8_rx);
  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE END DMA2_Stream2_IRQn 1 */
}


/**
* @brief This function handles CAN1 RX0 interrupts.
*/
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);

  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
* @brief This function handles CAN2 RX0 interrupts.
*/
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}
extern volatile unsigned long long FreeRTOSRunTimeTicks;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if (htim->Instance == TIM5) {
     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

	 		__HAL_TIM_ENABLE(&htim5);
		__HAL_TIM_ENABLE_IT(&htim5,TIM_IT_UPDATE);
  }
  /* USER CODE END Callback 1 */
	 else if(htim==(&htim3))
    {
       FreeRTOSRunTimeTicks++;  //时间节拍计数器加一
    }
}
/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //接收完成            暂时不加任务通知，后续讨论　　_待续
{
	 static  BaseType_t  pxHigherPriorityTaskWoken;
		if(huart == &huart1)
	{
/*  	
		  __HAL_UART_CLEAR_OREFLAG(&huart1);
			__HAL_DMA_SET_COUNTER(&hdma_usart1_rx,SizeofRemote);
			__HAL_DMA_ENABLE(&hdma_usart1_rx);
      vTaskNotifyGiveFromISR(RemoteDataTaskHandle,&pxHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);			
*/

	}else if(huart == &huart2)
	{ 
/*
		__HAL_DMA_SET_COUNTER(&hdma_usart2_rx,SizeofMinipc);
			__HAL_DMA_ENABLE(&hdma_usart2_rx);
		  vTaskNotifyGiveFromISR(MiniPCDataTaskHandle,&pxHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);	
*/
	}else if(huart == &huart3)
	{
			uint8_t Res = 0;
			HAL_UART_Receive(&huart3, &Res, 1,1);//读取接收到的数据
		  /*无线调参的处理函数*/
			PID_UART_IRQHandler(&huart3, Res);
	}
//	else if(huart == &huart6)
//	{
//			__HAL_DMA_SET_COUNTER(&hdma_usart6_rx,SizeofReferee);
//			__HAL_DMA_ENABLE(&hdma_usart6_rx);
////			vTaskNotifyGiveFromISR(RefereeDataTaskHandle,&pxHigherPriorityTaskWoken);
////			portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);	
//		
//	}
	else if(huart == &huart8)
	{
		
	}
	
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan == &hcan1)
	{
		switch(hcan1.pRxMsg->StdId)
		{
			case 0x205:
			{

				if(yaw_get.msg_cnt++ <= 50)
				{
					get_moto_offset(&yaw_get,&hcan1);
				}else{
					yaw_get.msg_cnt = 51;
					get_moto_measure_6623(&yaw_get,&hcan1);
				}
//				yaw_get.angle=(uint16_t)(hcan->pRxMsg->Data[0]<<8 |hcan->pRxMsg->Data[1]) ;
			}break;
			case 0x206:
			{
				if(pit_get.msg_cnt++ <= 50)
				{
					get_moto_offset(&pit_get,&hcan1);
				}else{
					pit_get.msg_cnt = 51;
					get_moto_measure_6623(&pit_get,&hcan1);
				}
			}break;
			case 0x201:
			{
				if(moto_dial_get.msg_cnt++ <= 50)	
				{
					get_moto_offset(&moto_dial_get,&hcan1);
				}
				else{	
					moto_dial_get.msg_cnt=51;	
					get_moto_measure_6623(&moto_dial_get, &hcan1);
				}
			}break;
			default: break;
		}
		if( HAL_BUSY == HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0))//开启中断接收
		{
			/* Enable FIFO 0 overrun and message pending Interrupt */
			__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
		}
	}else if(hcan == &hcan2)
	{
//		HAL_CAN_Receive(&hcan1,CAN_FIFO0,10);
		switch(hcan->pRxMsg->StdId)
		{
			case CAN_3510Moto1_ID:
			case CAN_3510Moto2_ID:
			case CAN_3510Moto3_ID:
			case CAN_3510Moto4_ID:
			{
				static uint8_t i;
				i = hcan->pRxMsg->StdId - CAN_3510Moto1_ID;
				if(moto_chassis_get[i].msg_cnt++ <= 50)	
				{
					get_moto_offset(&moto_chassis_get[i],&hcan2);
				}
				else{		
					moto_chassis_get[i].msg_cnt=51;	
					get_moto_measure_3508(&moto_chassis_get[i], &hcan2);
				}
			}
		}
		if( HAL_BUSY == HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0))//开启中断接收
		{
			/* Enable FIFO 0 overrun and message pending Interrupt */
			__HAL_CAN_ENABLE_IT(&hcan2,CAN_IT_FMP0);
		}	
	}
}


/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
