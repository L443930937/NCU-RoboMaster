#include "BSP.h"


void BSP_Init(void)
{
	
	//���ź�����ʱ��
  MX_GPIO_Init();
	HAL_Delay(1000);
	//dma
  MX_DMA_Init();
	//can
	MX_CAN1_Init();
	MX_CAN2_Init();	
	CanFilter_Init(&hcan1);
	CanFilter_Init(&hcan2);
	//��ʱ��
  MX_TIM5_Init();
  MX_TIM12_Init();
	
//	__HAL_TIM_ENABLE_IT(&htim5,TIM_IT_UPDATE);
	//����
  MX_UART8_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
	//SPI
	MX_SPI5_Init();
	//ʹ��DMA�ж�
	HAL_UART_Receive_DMA(&huart1,USART1_RX_DATA,maxsize); //��һ����Ŀ���Ǵ���һ�ν����ڴ棬��CAN��һ��
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart6,USART6_RX_DATA,maxsize);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	//HAL_UART_Receive_DMA(&huart8,UART8_RX_DATA,maxsize);
  HAL_UART_Receive_DMA(&huart8,HOST_Buffer.buffer,sizeof(HOST_Buffer.buffer));//Sabar
	__HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);
	//������
	 MPU6500_Init();
	//Ħ����
	// _HAL_TIM_ENABLE(&htim5);
	//((&htim5)->Instance->CR1|=(TIM_CR1_CEN));
	GUN_Init();
	//ʹ��can�ж�
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
  HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);




}
