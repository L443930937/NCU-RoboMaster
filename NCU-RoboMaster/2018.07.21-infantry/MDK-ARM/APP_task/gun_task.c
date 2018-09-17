/* 包含头文件----------------------------------------------------------------*/
#include "gun_task.h"
/* 内部宏定义----------------------------------------------------------------*/

/* 内部自定义数据类型--------------------------------------------------------*/

/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* 内部常量定义--------------------------------------------------------------*/

/* 外部变量声明--------------------------------------------------------------*/
Heat_Gun_t * ptr_heat_gun_t;
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
//	ptr_heat_gun_t->limt_heat=90;
	ptr_heat_gun_t->limt_spd=30;
	uint32_t set_angle=0;
	ptr_heat_gun_t->stop_flg=1;
	ptr_heat_gun_t->heat_down_flg=1;



	while(1)
	{

	/*		 if(xQueueReceive(JSYS_QueueHandle,&Frame,0)!=pdFALSE)
			 {
				 printf("Frame->Data.PowerANDHeat.chassisPowerBuffer=%f\n",Frame->Data.PowerANDHeat.chassisPowerBuffer);      			 
			 }
			  
//				else printf("jieshoushibai\n");      			 
			 else printf("Chassis_Queue接收失败！！！\n");*/
		 

         if(ptr_heat_gun_t->roboLevel>0) ptr_heat_gun_t->limt_heat=ptr_heat_gun_t->roboLevel*90;
		        else  ptr_heat_gun_t->limt_heat=90;
		
			   ptr_heat_gun_t->limt_bullet=((ptr_heat_gun_t->limt_heat-ptr_heat_gun_t->rel_heat)/ptr_heat_gun_t->limt_spd)-1;
//							printf("limt_heat:%d\n",ptr_heat_gun_t->limt_heat);
			 	 if(ptr_heat_gun_t->shted_bullet >= ptr_heat_gun_t->limt_bullet)  ptr_heat_gun_t->heat_down_flg=0;
				 else  ptr_heat_gun_t->heat_down_flg=1;
				 
			   if(ptr_heat_gun_t->stop_flg==1&&(ptr_heat_gun_t->rel_heat!=ptr_heat_gun_t->last_rel_heat||ptr_heat_gun_t->rel_heat==0))  
          {
					    ptr_heat_gun_t->shted_bullet=0;
					}
					
				ptr_heat_gun_t->last_rel_heat=ptr_heat_gun_t->rel_heat;
//					printf("limt_bullet:%d\tsht_flg:%d\tstop_flg:%d\n",ptr_heat_gun_t->limt_bullet,ptr_heat_gun_t->sht_flg,ptr_heat_gun_t->stop_flg);
//								              printf("ptr_heat_gun_t->stop_flg==1\n");

					if(ptr_heat_gun_t->heat_down_flg==1&&ptr_heat_gun_t->sht_flg==1&&ptr_heat_gun_t->stop_flg==1)
					{
             Friction_Wheel_Motor(200,200);
						 moto_dial_get.round_cnt=0;
						 moto_dial_get.offset_angle=moto_dial_get.angle;
             moto_dial_get.total_angle=0;		
						 set_angle=42125;
						 ptr_heat_gun_t->sht_flg=0;
						 
					}
				else 	if(ptr_heat_gun_t->heat_down_flg==1&&ptr_heat_gun_t->sht_flg==2&&ptr_heat_gun_t->stop_flg==1)
					{
						 Friction_Wheel_Motor(150,150);
						 moto_dial_get.round_cnt=0;
						 moto_dial_get.offset_angle=moto_dial_get.angle;
             moto_dial_get.total_angle=0;
						 set_angle=42125*3;
						ptr_heat_gun_t->sht_flg=0;
					}
					
      if(set_angle==42125)
		  	{
						if(moto_dial_get.round_cnt>=5||set_angle==0) ptr_heat_gun_t->stop_flg=1; 
								else   ptr_heat_gun_t->stop_flg=0;
			  }
			 else if(set_angle==42125*3)
			 {
				 if(moto_dial_get.round_cnt>=5*3||set_angle==0) ptr_heat_gun_t->stop_flg=1; 
						else   ptr_heat_gun_t->stop_flg=0;
			 }
			 
			 if(ptr_heat_gun_t->heat_down_flg==0)
			 {
						if(moto_dial_get.total_angle>=42125*2) moto_dial_get.total_angle=42125*3;
							else if(moto_dial_get.total_angle>=42125)  moto_dial_get.total_angle=42125*2;
								 else if(moto_dial_get.total_angle>0)    moto_dial_get.total_angle=42125;
			 }
			 
				pid_calc(&pid_dial_pos, moto_dial_get.total_angle, set_angle);	
				pid_calc(&pid_dial_spd,moto_dial_get.speed_rpm ,pid_dial_pos.pos_out);
				Allocate_Motor(&hcan1,pid_dial_spd.pos_out);
			osDelay(10);
	}
}


