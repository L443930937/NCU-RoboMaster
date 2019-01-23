/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "gimbal_task.h"
#include "Power_restriction.h"
/* �ڲ��궨��----------------------------------------------------------------*/

/* �ڲ��Զ�����������--------------------------------------------------------*/
static  int16_t Yaw_Current_Value = 0;
static  int16_t Pitch_Current_Value = 0;
/* ���������Ϣ����----------------------------------------------------------*/

/* �ڲ���������--------------------------------------------------------------*/

/* �ⲿ��������--------------------------------------------------------------*/
Pos_Set  yaw_set;
Pos_Set  pit_set;
int8_t gimbal_disable_flg;

/* ���õ��ⲿ����ԭ������------------------------------------------------------------
void Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch);
float pid_calc(pid_t* pid, float get, float set);
------------------------------------------------------------------------------
*/
/* �ڲ�����------------------------------------------------------------------*/
pid_t pid_yaw       = {0};  //yaw��λ�û�
pid_t pid_yaw_jy901 = {0};  //��������� /*Ŀǰֻ����λ�û�*/ 
pid_t pid_pit       = {0};	//pit��λ�û�
pid_t pid_yaw_spd   = {0};	//yaw���ٶȻ�
pid_t pid_pit_spd   = {0};	//pit���ٶȻ�
pid_t pid_yaw_jy901_spd = {0};
pid_t pid_pit_jy901 = {0};
pid_t pid_pit_jy901_spd = {0};
/* �ڲ�����ԭ������----------------------------------------------------------*/
/**                                                           //����
	**************************************************************
	** Descriptions: ��̨pid��ʼ��
	** Input:  NULL
	** Output: NULL
	**************************************************************
**/

void gimbal_pid_init(void)
{
		/*pitch axis motor pid parameter*/
	PID_struct_init(&pid_pit, POSITION_PID, 5000, 1000,
                  8.0f, 0.02f, 0.5f); 
  PID_struct_init(&pid_pit_jy901_spd, POSITION_PID, 5000, 1000,
                  2.0f, 0.0f, 0.0f );
//	pid_pit_jy901_spd.deadband = 10;
  /* yaw axis motor pid parameter */
//	 PID_struct_init(&pid_yaw, POSITION_PID, 5000, 1000,
//                  10.0f, 0.02f, 10.0f); 
//	 PID_struct_init(&pid_yaw_jy901_spd, POSITION_PID, 5000, 1000,
//                  2.0f, 0.0f, 0.0f );
	//use jy901
  PID_struct_init(&pid_yaw_jy901, POSITION_PID, 5000, 1000,
                  5.0f, 0.02f, 0.5f); //	
  PID_struct_init(&pid_yaw_jy901_spd, POSITION_PID, 5000, 1000,
                  2.0f, 0.0f, 0.0f ); 

//	pid_yaw_jy901_spd.deadband = 10;
}
/* �������岿�� -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gimbal_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	��̨�������
	*	@retval	
****************************************************************************************/
void Gimbal_Contrl_Task(void const * argument)
{
	yaw_set.expect = 0; 
	pit_set.expect = 0;
	yaw_set.mode   = 0;
	gimbal_disable_flg=0;
	Pitch_Current_Value=0;
	Yaw_Current_Value=0;
	gimbal_pid_init();
			
	for(;;)		
    {
	
	

			// ֻ��jy901�ĽǶ�����
			switch(0)
			{
				case 1: ;//����������
				case 2: ;//Զ��������
				case 3:{ //���ģʽ
//					pid_calc(&pid_yaw_jy901,ptr_jy901_t_yaw.final_angle/*yaw_get.total_angle*/,yaw_set.expect_pc);
//					pid_calc(&pid_yaw_jy901_spd,(ptr_jy901_t_angular_velocity.vz), pid_yaw_jy901.pos_out);
//					//pit��
//					pid_calc(&pid_pit, pit_get.total_angle,pit_set.expect_pc);
//					pid_calc(&pid_pit_jy901_spd, (ptr_jy901_t_angular_velocity.vy), pid_pit.pos_out);
				}
				break;
				default:{	//�������
//					pid_calc(&pid_yaw, yaw_get.total_angle, yaw_set.expect);
//					pid_calc(&pid_yaw_jy901_spd,(ptr_jy901_t_angular_velocity.vz), pid_yaw.pos_out);

					yaw_set.expect = minipc_rx.angle_yaw + yaw_set.expect;
					pit_set.expect = minipc_rx.angle_pit + pit_set.expect;
					minipc_rx.angle_yaw = 0;
					minipc_rx.angle_pit = 0;

//					pid_calc(&pid_yaw_jy901,(ptr_jy901_t_yaw.final_angle),yaw_set.expect);
					pid_calc(&pid_yaw_jy901, yaw_get.total_angle,yaw_set.expect);
					pid_calc(&pid_yaw_jy901_spd,(ptr_jy901_t_angular_velocity.vz), pid_yaw_jy901.pos_out);
					//pit��
					pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
					pid_calc(&pid_pit_jy901_spd,(ptr_jy901_t_angular_velocity.vy), pid_pit.pos_out);
					
				}
				break;
			}
     if(1){ /*������*/
//			int16_t  *ptr = angle; //��ʼ��ָ��
//			angle[0]	= (yaw_get.angle);
////			angle[1]	= (ptr_jy901_t->final_angle_yaw);
//			angle[2]	= ((int16_t)pid_pit.pos_out);
//			angle[3]	= (int16_t)(pid_pit_spd.pos_out);
//			/*������ʾ��������������*/
//			vcan_sendware((uint8_t *)ptr,4*sizeof(angle[0]));
			 
//			 printf("\n\r*******gimbal_test***********\n\r");
//			 printf("YAWң������:%d\n\r",yaw_set.expect);
//			 printf("PITң������:%d\n\r",pit_set.expect);			 
//			 printf("CAN_YAW������:%d\n\r",(int16_t)yaw_get.total_angle);
//			 printf("CAN	_PIT������:%d\n\r",(int16_t)pit_get.total_angle);
//			 printf("����������:%d\n\r",imu_data.gz-imu_data_offest.gz);
//			 printf("jy901����:%f\n\r",ptr_jy901_t_yaw.final_angle);
//			 printf("yaw_pid����ֵ:%f\n\r",-pid_yaw_spd.pos_out);
//			 printf("pit_pid����ֵ:%f\n\r",-pid_pit_spd.pos_out);	
//			 printf("\n\r**************DOWN*************\n\r");
//			 printf("yaw_set:%d,yaw_get:%d,out:%d  ",yaw_set.expect,yaw_get.total_angle,(int16_t)(-pid_yaw_spd.pos_out));
//			 printf("pit_set:%d,pit_get:%d,out:%d\n\r\n",pit_set.expect,pit_get.total_angle,(int16_t)(-pid_pit_spd.pos_out));
			 	}                                                                                                        
		 
		  	Pitch_Current_Value=(-pid_pit_jy901_spd.pos_out); 
		    Yaw_Current_Value= (-pid_yaw_jy901_spd.pos_out); 
				if(gimbal_disable_flg==1)
				{
					Cloud_Platform_Motor_Disable(&hcan1);
				}
				else Cloud_Platform_Motor(&hcan1,Yaw_Current_Value,Pitch_Current_Value);

			osDelay(5);
   }
 
}
