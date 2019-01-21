/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "gun_task.h"
/* �ڲ��궨��----------------------------------------------------------------*/

/* �ڲ��Զ�����������--------------------------------------------------------*/

/* ���������Ϣ����----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* �ڲ���������--------------------------------------------------------------*/

/* �ⲿ��������--------------------------------------------------------------*/
Heat_Gun_t * ptr_heat_gun_t;
//Power_Heat * power_heat;
/* �ⲿ����ԭ������-----------------------------------------------------------
float pid_calc(pid_t* pid, float get, float set);
void Friction_Wheel_Motor(uint32_t wheelone,uint32_t wheeltwo);
void Allocate_Motor(CAN_HandleTypeDef * hcan,int16_t value);
-----------------------------------------------------------------------------
-*/
/* �ڲ�����------------------------------------------------------------------*/

pid_t pid_dial_pos  = {0};  //���̵��λ�û�
pid_t pid_dial_spd  = {0};	//���̵���ٶȻ�
/* �ڲ�����ԭ������----------------------------------------------------------*/
void Gun_Pid_Init()
{
		PID_struct_init(&pid_dial_pos, POSITION_PID, 6000, 5000,
									0.2f,	0.0000f,	2.0f);  
		//pid_pos[i].deadband=500;
		PID_struct_init(&pid_dial_spd, POSITION_PID, 6000, 5000,
									1.5f,	0.1f,	0.0f	);  
		pid_pit_spd.deadband=10;//2.5f,	0.03f,	1.0f	
	
//    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);   //��Դ���� _����
}
/* �������岿�� -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gun_Task(void const * argument)
	*	@param
	*	@supplement	ǹ��������������
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
			 else printf("Chassis_Queue����ʧ�ܣ�����\n");*/
		 

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


