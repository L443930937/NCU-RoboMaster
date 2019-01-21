/* 包含头文件----------------------------------------------------------------*/
#include "gun_task.h"
#include "math.h"
/* 内部宏定义----------------------------------------------------------------*/

/* 内部自定义数据类型--------------------------------------------------------*/

/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* 内部常量定义--------------------------------------------------------------*/

/* 外部变量声明--------------------------------------------------------------*/
Heat_Gun_t  ptr_heat_gun_t;
//Power_Heat * power_heat;
/* 外部函数原型声明-----------------------------------------------------------
float pid_calc(pid_t* pid, float get, float set);
void Friction_Wheel_Motor(uint32_t wheelone,uint32_t wheeltwo);
void Allocate_Motor(CAN_HandleTypeDef * hcan,int16_t value);
-----------------------------------------------------------------------------
-*/
/* 内部变量------------------------------------------------------------------*/

pid_t pid_dial_pos  = {0};  //拨盘电机位置环
pid_t pid_dial_spd  = {0};	//拨盘电机速度环
/* 内部函数原型声明----------------------------------------------------------*/
void Gun_Pid_Init()
{
		PID_struct_init(&pid_dial_pos, POSITION_PID, 6000, 5000,
									0.2f,	0.0000f,	2.0f);  
		//pid_pos[i].deadband=500;
		PID_struct_init(&pid_dial_spd, POSITION_PID, 6000, 5000,
									1.5f,	0.1f,	0.0f	);  
		pid_pit_spd.deadband=10;//2.5f,	0.03f,	1.0f	
	
//    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);   //电源引脚 _待续
}
/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gun_Task(void const * argument)
	*	@param
	*	@supplement	枪口热量限制任务
	*	@retval	
****************************************************************************************/
void Gun_Task(void const * argument)
{ 
//		  tFrame   *Frame;
//	    tFrame   *last_Frame;
//	ptr_heat_gun_t.limt_heat=90;
	static int i;
	ptr_heat_gun_t.limt_spd=30;
	uint32_t set_angle=0;
	ptr_heat_gun_t.stop_flg=1;
	ptr_heat_gun_t.heat_down_flg=1;
  Gun_Pid_Init();
  uint8_t motor_stop_flag=0;


	for(;;)
	{

	/*		 if(xQueueReceive(JSYS_QueueHandle,&Frame,0)!=pdFALSE)
			 {
				 printf("Frame->Data.PowerANDHeat.chassisPowerBuffer=%f\n",Frame->Data.PowerANDHeat.chassisPowerBuffer);      			 
			 }
			  
//				else printf("jieshoushibai\n");      			 
			 else printf("Chassis_Queue接收失败！！！\n");*/
		 


		 	   if((minipc_rx.state_flag==1||ptr_heat_gun_t.sht_flg==1)&&motor_stop_flag==1)
					{
						Friction_Wheel_Motor(150,150);
					  set_angle=42125*5;
					  moto_dial_get.round_cnt=0;
						moto_dial_get.offset_angle=moto_dial_get.angle;
            moto_dial_get.total_angle=0;
						minipc_rx.state_flag=0;
						ptr_heat_gun_t.sht_flg=0;
					}
					
					if(moto_dial_get.round_cnt>=5*5||set_angle==0)
					{
						motor_stop_flag = 1;

					}
					else  
					{
						motor_stop_flag = 0;
						
					}

				pid_calc(&pid_dial_pos, moto_dial_get.total_angle, set_angle);	
				pid_calc(&pid_dial_spd,moto_dial_get.speed_rpm ,pid_dial_pos.pos_out);
				Allocate_Motor(&hcan1,pid_dial_spd.pos_out);

			osDelay(5 );
	}
}


