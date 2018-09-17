/* 包含头文件----------------------------------------------------------------*/
#include "chassis_task.h"
/* 内部宏定义----------------------------------------------------------------*/

/* 内部自定义数据类型--------------------------------------------------------*/

/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId Chassis_QueueHandle;

/* 内部常量定义--------------------------------------------------------------
void Chassis_Motor( CAN_HandleTypeDef * hcan,
									  int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
float pid_calc(pid_t* pid, float get, float set);
void motor_move_setvmmps(float  wheel[4],float dstVmmps_X,
													float dstVmmps_Y,float dstVmmps_W);
----------------------------------------------------------------------------
*/
/* 外部变量声明--------------------------------------------------------------*/
moto3508_type * moto_3508_set; 
/* 调用的外部函数原型声明----------------------------------------------------------*/

/* 内部变量------------------------------------------------------------------*/
pid_t pid_3508_pos[4];      //底盘电机位置环
pid_t pid_3508_spd[4];			//底盘电机速度环
/* 内部函数原型声明----------------------------------------------------------*/
void Chassis_pid_init(void)
{
		for(int i=0; i<4; i++)
	{
		PID_struct_init(&pid_3508_pos[i], POSITION_PID, 10000, 2000,
									1.5f,	0.0f,	20.0f);  //4 motos angular rate closeloop.
		pid_3508_pos[i].deadband=300;
	}
		for(int i=0; i<4; i++)
	{ 
		PID_struct_init(&pid_3508_spd[i], POSITION_PID, 10000, 2000,
									1.5f,	0.1f,	0.0f	);  //4 motos angular rate closeloop.
	}
}
/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Chassis_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	底盘控制任务
	*	@retval	
****************************************************************************************/
void Chassis_Contrl_Task(void const * argument)
{
	float  wheel[4];
	Chassis_pid_init();
	while(1)
	{
	
		motor_move_setvmmps(wheel,moto_3508_set->dstVmmps_X,moto_3508_set->dstVmmps_Y,moto_3508_set->dstVmmps_W);
		
		for(int i=0; i<4; i++)
			{		
				pid_calc(&pid_3508_spd[i], moto_chassis_get[i].speed_rpm, wheel[i]);
			}
		/**********功率限制*********/
		
	  /***********end*******/	
			Chassis_Motor(&hcan2,
										pid_3508_spd[0].pos_out,
										pid_3508_spd[1].pos_out, 
										pid_3508_spd[2].pos_out, 
										pid_3508_spd[3].pos_out);
if(0){  //数据发送和任务检测   _待续
//	if(HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_0) == 1)            //弹仓检测  低电平触发发送0为子弹空
																												 //                    发送1为弹夹满
//	{
//		data1 = 1;
//	}
//	else if(HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_0) == 0)
//	{	
//	data1 = 0;
//	}
//	sendata();
//		
//	
//	
//		if(data_pro_task_flag==1) { data_pro_task_flag=0;flag_counter=0;}				//错误检测
//		else flag_counter++;
//		if(flag_counter>100)  { NVIC_SystemReset();}
//		if(flag_counter>200)
//			flag_counter = 0;
	}
		
	  osDelay(5);
  }
}


