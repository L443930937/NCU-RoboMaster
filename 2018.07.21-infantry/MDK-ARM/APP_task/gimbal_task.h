#ifndef __gimbal_task_H
#define __gimbal_task_H
/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "pid.h"
#include "communication.h "
#include "Motor_USE_CAN.h"
#include "atom_imu.h"
#include "decode.h"
/* ��ģ�����ⲿ�ṩ���������Ͷ���--------------------------------------------*/

typedef struct{
		int16_t expect;
	  uint8_t ICR_flag;
	  uint8_t ref_flag;
	  uint8_t ref_flag1;
} Pos_Set;



/* ��ģ�����ⲿ�ṩ�ĺ궨��--------------------------------------------------*/

/* ��ģ�����ⲿ�ṩ�Ľӿڳ�������--------------------------------------------*/

extern Pos_Set * yaw_set;
extern Pos_Set * pit_set;

/* ��ģ�����ⲿ�ṩ�Ľӿں���ԭ������----------------------------------------*/
void Gimbal_Task(void const * argument);

/* ȫ��������----------------------------------------------------------------*/

#endif
