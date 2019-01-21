/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "data_pro_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "pid.h"
#include "string.h"
#include "protocol.h"
#include "gun_task.h"
#include "communication.h"
/* �ڲ��궨��----------------------------------------------------------------*/
#define press_times  20
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\
//extern osSemaphoreId Dubs_BinarySemHandle;
/* �ڲ��Զ�����������--------------------------------------------------------*/

/* ���������Ϣ����----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* �ڲ���������--------------------------------------------------------------*/

/* �ⲿ��������--------------------------------------------------------------*/

/* ���õ��ⲿ����ԭ������------------------------------------------------------
	uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);
	uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
------------------------------------------------------------------------------
*/
/* �ڲ�����------------------------------------------------------------------*/
int16_t XY_speed_max = 6000;
int16_t XY_speed_min = -6000; 
int16_t W_speed_max = 3000;
int16_t W_speed_min = -3000; 
uint8_t press_counter;
volatile float remain_power=0.0;   //���̹��� _����
/* �ڲ�����ԭ������-----------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	RemoteControlProcess()
	*	@param
	*	@supplement	��ң�������жԽӣ���ң���������ݽ��д���ʵ�ֶԵ��̡���̨����������Ŀ���
	*	@retval	
****************************************************************************************/
void RemoteControlProcess()  
{


//									printf("ң����ģʽ!!!");
							pit_set->expect = pit_set->expect +(0x400-RC_Ctl.rc.ch3)/100;	
							yaw_set->expect = yaw_set->expect +(0x400-RC_Ctl.rc.ch2)/100;	
							moto_3508_set->dstVmmps_X=0;
							moto_3508_set->dstVmmps_Y=(-(RC_Ctl.rc.ch1-0x400)*30);
							moto_3508_set->dstVmmps_W=((RC_Ctl.rc.ch0-0x400)*30);

					
					if(press_counter>=press_times)
					{
							press_counter=press_times+1;
									if(RC_Ctl.rc.s1==1)
									{
										ptr_heat_gun_t->sht_flg=1;
										press_counter=0;
									}
								
				
							if(RC_Ctl.rc.s1==2)
									{
										HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
									}
					}
}

/***************************************************************************************
**
	*	@brief	MouseKeyControlProcess()
	*	@param
	*	@supplement	�Լ�������ݽ��д���
	*	@retval	
****************************************************************************************/
void MouseKeyControlProcess()
{
	
	if(RC_Ctl.key.v & 0x10 )//�����ٶȵ�λ��ÿ���ٶ�����550
					{
							//p++;//shift������λ
						XY_speed_max = 3000;//(NORMAL_SPEED_MAX)*3.5;
						XY_speed_min = -3000;//(NORMAL_SPEED_MIN)*3.5;
					}
					else if(RC_Ctl.key.v & 0x20)
					{
						//	p--;//ctrl���µ�λ
						XY_speed_max = 4000;//(NORMAL_SPEED_MAX)*5;
						XY_speed_min = -4000;//(NORMAL_SPEED_MIN)*5;
					}else if(RC_Ctl.key.v & 0x4000)
					{
					//v  ��
						XY_speed_max = 2000;
						XY_speed_min = -2000;
					} else if(RC_Ctl.key.v & 0x800)
					{
					//b  ��
						XY_speed_max = 1500;
						XY_speed_min = -1500;
					}

//					i=p/10;
//					if(i<1)i=1;
//					if(i>6)i=6;
			

					if(RC_Ctl.key.v & 0x01)                       moto_3508_set->dstVmmps_Y -= ACC_SPEED;//����W��
					else if(RC_Ctl.key.v & 0x02)                  moto_3508_set->dstVmmps_Y += ACC_SPEED;//����S��
					else{  
							 	if(moto_3508_set->dstVmmps_Y>-DEC_SPEED&&moto_3508_set->dstVmmps_Y<DEC_SPEED) 	 moto_3508_set->dstVmmps_Y = 0;
								if(moto_3508_set->dstVmmps_Y>0) 	                   moto_3508_set->dstVmmps_Y -= DEC_SPEED;
								if(moto_3508_set->dstVmmps_Y<0) 		                 moto_3508_set->dstVmmps_Y += DEC_SPEED;
					}


					if(RC_Ctl.key.v & 0x04)                        moto_3508_set->dstVmmps_X += ACC_SPEED; //����D��
					else if(RC_Ctl.key.v & 0x08)    		           moto_3508_set->dstVmmps_X -= ACC_SPEED;//����A��
					else{
									if(moto_3508_set->dstVmmps_X>-DEC_SPEED&&moto_3508_set->dstVmmps_X<DEC_SPEED) 		moto_3508_set->dstVmmps_X = 0;		
									if(moto_3508_set->dstVmmps_X>0) 	                   moto_3508_set->dstVmmps_X -= DEC_SPEED;
									if(moto_3508_set->dstVmmps_X<0) 		                 moto_3508_set->dstVmmps_X += DEC_SPEED;
					}


					if(RC_Ctl.key.v & 0x1000)   //x ����       shift
					{
						  yaw_set->ICR_flag=0;
					}
				else if(RC_Ctl.mouse.press_r == 1)     //����Ҽ�  ��̨���� hold
					{
						  yaw_set->ICR_flag=2;
					}
				else if(RC_Ctl.key.v & 0x2000) //c  ����    shift
					{
						 yaw_set->ICR_flag=1;
					}
					
					if(yaw_set->ICR_flag==0)
					{
						  yaw_set->expect=0;
      				moto_3508_set->dstVmmps_W = RC_Ctl.mouse.x*75;
					}
					else if(yaw_set->ICR_flag==1||yaw_set->ICR_flag==2)
					{
					if(RC_Ctl.key.v & 0x40)                        moto_3508_set->dstVmmps_W += DEC_SPEED; //����Q��
					else if(RC_Ctl.key.v & 0x80)    		           moto_3508_set->dstVmmps_W -= DEC_SPEED;//����E��
					else{
									if(moto_3508_set->dstVmmps_W>-DEC_SPEED&&moto_3508_set->dstVmmps_W<DEC_SPEED) 		moto_3508_set->dstVmmps_W = 0;		
									if(moto_3508_set->dstVmmps_W>0) 	                   moto_3508_set->dstVmmps_W -= DEC_SPEED;
									if(moto_3508_set->dstVmmps_W<0) 		                 moto_3508_set->dstVmmps_W += DEC_SPEED;
					     }
					if(yaw_set->ICR_flag==1)            yaw_set->expect = yaw_set->expect - RC_Ctl.mouse.x/2;
							 else if(yaw_set->ICR_flag==2)  yaw_set->expect = yaw_set->expect - RC_Ctl.mouse.x;
					}



					//��꣨�ƶ��ٶ�*1000/50��
					pit_set->expect = pit_set->expect+RC_Ctl.mouse.y/2;	

							
					
					if(RC_Ctl.mouse.press_l==1)        //����������
					{
						press_counter++;
							if(press_counter>=10)
			    	{
							press_counter=10+1;
							ptr_heat_gun_t->sht_flg=1;
							press_counter=0;
			    	}
					}
					else 	if(RC_Ctl.key.v & 0x100)     //r��3����
					{
						press_counter++;
						if(press_counter>=5)
			    	{
							press_counter=5+1;
							ptr_heat_gun_t->sht_flg=2;
							press_counter=0; 
						}
					}
				
}


/***************************************************************************************
**
	*	@brief	hard_brak()
	*	@param
	*	@supplement	����ֹͣ����
	*	@retval	
****************************************************************************************/
void hard_brak()
{
	  yaw_set->ICR_flag=0;	
	  pit_set->expect=0;
	  yaw_set->expect=0;
	
		moto_3508_set->dstVmmps_X=0;
		moto_3508_set->dstVmmps_Y=0;
		moto_3508_set->dstVmmps_W=0;
}


/* �������岿�� -------------------------------------------------------------*/
/***************************************************************************************
**
	*	@brief	Data_Pro_task(void const * argument)
	*	@param
	*	@supplement	ң�����ݽ��ռ���������
	*	@retval	
****************************************************************************************/
void Remote_Data_Task(void const * argument)
{
	uint32_t NotifyValue;
	while(1)
	{
		   NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    if(NotifyValue==1)
		{
			  NotifyValue=0;
//		  	printf("Data_Pro_task Running!!!");
			  Remote_Ctrl();
				switch(RC_Ctl.rc.s2)
				{
					case 1: RemoteControlProcess();break; 
					case 2: hard_brak();break;
					case 3: MouseKeyControlProcess();break;
					default :break;
				}					
//				  yaw_set->pit_weizhi-=ptr_jy901_t->final_err;
				
			VAL_LIMIT(moto_3508_set->dstVmmps_X, XY_speed_min, XY_speed_max);
			VAL_LIMIT(moto_3508_set->dstVmmps_Y, XY_speed_min, XY_speed_max);	
			VAL_LIMIT(moto_3508_set->dstVmmps_W, W_speed_min, W_speed_max);
				
					if(pit_set->expect>1000) pit_set->expect=1000;
					if(pit_set->expect<-500) pit_set->expect=-500;
			
				if(yaw_set->ICR_flag==2)
				{
							if(yaw_set->expect>2000) 	yaw_set->expect=2000;
							if(yaw_set->expect<-2000) 	yaw_set->expect=-2000;
				}
            press_counter++;
		}
	}
}

/***************************************************************************************
**
	*	@brief	JSYS_Task(void const * argument)
	*	@param
	*	@supplement	����ϵͳ���ݴ�������
	*	@retval	
****************************************************************************************/
void Referee_Data_Task(void const * argument)
{
	    tFrame   *Frame;
	
	    uint32_t NotifyValue;
	while(1)
	{
				   NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    if(NotifyValue==1)
		{
//			printf("running!!!\n");
			  NotifyValue=0;
//			  JY901_Data_Pro();
        uint8_t *buff=USART6_RX_DATA;
			for(int8_t i=0;i<USART6_RX_NUM;i++)
			{
					if(buff[i]==0xA5)
					{
					   Frame = (tFrame *)&buff[i];
						
					    if( verify_crc16_check_sum((uint8_t *)Frame, Frame->FrameHeader.DataLength + sizeof(tFrameHeader) + sizeof(tCmdID) + sizeof(Frame->CRC16))
		             && verify_crc8_check_sum((uint8_t *)Frame,sizeof(tFrameHeader)))
								 {
									 if(Frame->CmdID==PowerANDHeat)
									 {
										remain_power=Frame->Data.PowerANDHeat.chassisPowerBuffer;
								    ptr_heat_gun_t->rel_heat = Frame->Data.PowerANDHeat.shootHeat0;
										


//                        power_heat=&Frame->Data.PowerANDHeat;
//										 printf("shootHeat0:%d\tchassisPowerBuffer:%f\n",Frame->Data.PowerANDHeat.shootHeat0,Frame->Data.PowerANDHeat.chassisPowerBuffer);
			/*							 if(xQueueSend(JSYS_QueueHandle,&Frame,0)!=pdFALSE)
										 {
											 
										 }
										 else printf("����ʧ�ܣ�����\n");*/
//                      memcpy(&ptr_heat_gun_t->rel_heat,(uint8_t*)&Frame->Data.PowerANDHeat.shootHeat0,sizeof(ptr_heat_gun_t->rel_heat));
//										  memcpy(&ptr_heat_gun_t->remain_power,(uint8_t*)&Frame->Data.PowerANDHeat.chassisPowerBuffer,sizeof(ptr_heat_gun_t->remain_power));
//										 printf("chassisPowerBuffer:%f\n",ptr_power_heat->remain_power);
									 }
									 if(Frame->CmdID==GameInfo)
									 {
										     ptr_heat_gun_t->roboLevel=Frame->Data.GameInfo.roboLevel;
//                       memcpy(&ptr_heat_gun_t->roboLevel,(uint8_t*)&Frame->Data.GameInfo.roboLevel,sizeof(ptr_heat_gun_t->roboLevel));
//                        printf("roboLevel:%d\n",ptr_heat_gun_t->roboLevel);
									 }
									 if(Frame->CmdID==ShootData)
									 {
										 ptr_heat_gun_t->shted_bullet++;
									 }
											 i=i+sizeof(Frame);
								}
					}
				
			}

	 }
 }
}	
