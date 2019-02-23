/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "gun_task.h"
#include "math.h"
#include "SystemState.h"
/* �ڲ��궨��----------------------------------------------------------------*/

/* �ڲ��Զ�����������--------------------------------------------------------*/

/* ���������Ϣ����----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* �ڲ���������--------------------------------------------------------------*/
#define GUN_PERIOD  10
/* �ⲿ��������--------------------------------------------------------------*/
Heat_Gun_t  ptr_heat_gun_t;
extern uint8_t shot_frequency;
volatile uint8_t finish_flag = 0;
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

	osDelay(100);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	Gun_Pid_Init();
	
	uint8_t motor_stop_flag=0;
	int32_t set_angle = 0;
  uint8_t key_flag = 0;
  uint16_t check_count = 0;
  uint8_t check_flag = 0;
  uint8_t back_flag = 0;
  
  /*�趨����*/
  int32_t one_angle = -215;    //�����Ƕ� -122  60
  uint8_t frequency = 40;     //����Ƶ�ʣ�ԽСԽ��
  float   all_speed = -4000;  //�����ٶ�-7000 

	for(;;)
	{
		RefreshTaskOutLineTime(GunTask_ON);
		
 /*�жϷ���ģʽ*/
    switch(ptr_heat_gun_t.sht_flg)
    {
      case 0://��������ģʽ
      {
//        /*ֹͣ״̬��λ�û�*/
//        if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == 1 && key_flag ==0)
//        {
//          moto_dial_get.round_cnt=0;
//          moto_dial_get.offset_angle= moto_dial_get.angle;
//          moto_dial_get.total_angle=0;
//          pid_calc(&pid_dial_pos, moto_dial_get.total_angle,0);	//�趨�Ƕ�ֵΪ0
//        }
//        /*���°���˲�䣬������һ״̬*/
//        else if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == 0 && key_flag ==0)
//        {
//          key_flag = 1;
//        }
        /*ֹͣ״̬��λ�û�*/
        if(key_flag ==0)
        {
          moto_dial_get.round_cnt=0;
          moto_dial_get.offset_angle= moto_dial_get.angle;
          moto_dial_get.total_angle=0;
          pid_calc(&pid_dial_pos, moto_dial_get.total_angle,0);	//�趨�Ƕ�ֵΪ0
        }
        /*���°���˲�䣬������һ״̬*/
        else if(key_flag ==0)
        {
          key_flag = 1;
        }
        /*����֮��*/
//        if(key_flag == 1 || key_flag == 2)
//        {
//          /*���ٹ��̣�δ�������*/
//          if(finish_flag == 0)
//          {
//            key_flag = 2;
//            pid_dial_pos.pos_out = all_speed;
//          }
//          /*������ɣ�����ֹͣ״̬*/
//          else
//          {
//            key_flag = 0;
//            finish_flag = 0;
//          }
//        }
      }break;
      case 1://�̶��Ƕ�ģʽ
      {
        if(motor_stop_flag == 1)//���½Ƕ�
        {
          set_angle = one_angle * 819.1;
          shot_frequency = frequency;
          moto_dial_get.round_cnt=0;
          moto_dial_get.offset_angle= moto_dial_get.angle;
          moto_dial_get.total_angle=0;
          ptr_heat_gun_t.sht_flg=0;
        }
        /*��������ֹͣ�ж�*/
        if(one_angle < 0)//��ת
        {
          if(moto_dial_get.round_cnt <= (one_angle * 0.1 + 1) || set_angle==0)
          {
            motor_stop_flag = 1;
          }
          else  
          {
            motor_stop_flag = 0;
          }
        }
        else             //��ת
        {
          if(moto_dial_get.round_cnt >= (one_angle * 0.1 - 3) || set_angle==0)
          {
            motor_stop_flag = 1;
          }
          else  
          {
            motor_stop_flag = 0;
          }
        }
        /*pidλ�û�*/
        pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	
      }break;
      case 2://����ģʽ
      {
          check_count++;
        if(check_count < 1000)//��ʼʱһֱ��������
        {
          pid_dial_pos.pos_out = all_speed;
        }
        else//��ʼ����ת
        {
          if(check_count == 2000) //���ٹ���ѭ��1000-2000
            check_count = 1000;
          else if(back_flag == 0 && moto_dial_get.speed_rpm < 10)//��ת������ת���ٶ�<100
          {
            check_count = 4000;  
            check_flag = 1;
            back_flag = 1;
          }
          else if(check_count == 4200)
          {
            back_flag = 0;
            check_count = 1000;
          }
          else if(check_count > 3100) //��ת��Ϊ3000������    �������ٹ���
          {
            check_count = 1000;
            check_flag = 0;
          }
          switch(check_flag)
          {
            case 0:
              pid_dial_pos.pos_out = all_speed;//���ٹ���
              break;
            case 1:
              pid_dial_pos.pos_out = -all_speed;//��ת����
              break;
          }
        }

      }break;
    }
     /*�ٶȻ�*/
     pid_calc(&pid_dial_spd,moto_dial_get.speed_rpm ,pid_dial_pos.pos_out);
     /*�����������*/
		 Allocate_Motor(&hcan1,pid_dial_spd.pos_out);
					
        osDelayUntil(&xLastWakeTime,GUN_PERIOD);
	}
}


