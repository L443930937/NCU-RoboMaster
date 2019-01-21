#include "BSP.h"


void BSP_Init(void)
{
	
	//引脚和引脚时钟
  MX_GPIO_Init();
	HAL_Delay(1000);
	//dma
  MX_DMA_Init();
	//can
	MX_CAN1_Init();
	MX_CAN2_Init();	
	CanFilter_Init(&hcan1);
	CanFilter_Init(&hcan2);
	//定时器
  MX_TIM5_Init();
  MX_TIM12_Init();
	
//	__HAL_TIM_ENABLE_IT(&htim5,TIM_IT_UPDATE);
	//串口
  MX_UART8_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
	//SPI
	MX_SPI5_Init();
	//使能DMA中断
	HAL_UART_Receive_DMA(&huart1,USART1_RX_DATA,maxsize); //这一步的目的是创建一段接受内存，和CAN的一样
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart6,USART6_RX_DATA,maxsize);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	//HAL_UART_Receive_DMA(&huart8,UART8_RX_DATA,maxsize);
  HAL_UART_Receive_DMA(&huart8,HOST_Buffer.buffer,sizeof(HOST_Buffer.buffer));//Sabar
	__HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);
	//陀螺仪
	 MPU6500_Init();
	//摩擦轮
	// _HAL_TIM_ENABLE(&htim5);
	//((&htim5)->Instance->CR1|=(TIM_CR1_CEN));
	GUN_Init();
	//使能can中断
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
  HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);




}
