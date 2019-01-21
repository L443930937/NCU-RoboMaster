#ifndef __gimbal_task_H
#define __gimbal_task_H
/* 包含头文件----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "pid.h"
#include "communication.h "
#include "Motor_USE_CAN.h"
#include "atom_imu.h"
#include "decode.h"
/* 本模块向外部提供的数据类型定义--------------------------------------------*/

typedef struct{
		int16_t expect;
	  uint8_t ICR_flag;
	  uint8_t ref_flag;
	  uint8_t ref_flag1;
} Pos_Set;



/* 本模块向外部提供的宏定义--------------------------------------------------*/

/* 本模块向外部提供的接口常量声明--------------------------------------------*/

extern Pos_Set * yaw_set;
extern Pos_Set * pit_set;

/* 本模块向外部提供的接口函数原型声明----------------------------------------*/
void Gimbal_Task(void const * argument);

/* 全局配置区----------------------------------------------------------------*/

#endif
