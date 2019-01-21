/* 包含头文件----------------------------------------------------------------*/
#include "gimbal_task.h"
/* 内部宏定义----------------------------------------------------------------*/

/* 内部自定义数据类型--------------------------------------------------------*/
static  int16_t Yaw_Current_Value = 0;
static  int16_t Pitch_Current_Value = 0;
/* 任务相关信息定义----------------------------------------------------------*/

/* 内部常量定义--------------------------------------------------------------*/

/* 外部变量声明--------------------------------------------------------------*/
Pos_Set * yaw_set;
Pos_Set * pit_set;
/* 调用的外部函数原型声明------------------------------------------------------------
void Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch);
float pid_calc(pid_t* pid, float get, float set);
------------------------------------------------------------------------------
*/
/* 内部变量------------------------------------------------------------------*/
pid_t pid_yaw       = {0};  //yaw轴位置环
pid_t pid_yaw_jy901 = {0};  //外接陀螺仪 /*目前只用于位置环*/ 
pid_t pid_pit       = {0};	//pit轴位置环
pid_t pid_yaw_spd   = {0};	//yaw轴速度环
pid_t pid_pit_spd   = {0};	//pit轴速度环

pid_t pid_yaw_saber = {0};  //外接陀螺仪 /*目前只用于位置环*/
pid_t pid_yaw_saber_spd = {0};
pid_t pid_pit_saber = {0};
pid_t pid_pit_saber_spd = {0};
/* 内部函数原型声明----------------------------------------------------------*/
/**                                                           //待续
	**************************************************************
	** Descriptions: 云台pid初始化
	** Input:  NULL
	** Output: NULL
	**************************************************************
**/

void gimbal_pid_init(void)
{
		/*pitch axis motor pid parameter*/
	PID_struct_init(&pid_pit, POSITION_PID, 5000, 1000,
                  15, 0.5, 11); 
  PID_struct_init(&pid_pit_spd, POSITION_PID, 5000, 1000,
                  1, 0.0, 0);
  /* yaw axis motor pid parameter */
//  PID_struct_init(&pid_yaw_jy901, POSITION_PID, 5000, 1000,
//                  15, 0.1, 10); 
//  PID_struct_init(&pid_yaw, POSITION_PID, 5000, 1000,
//                  20, 0.2, 0); 
//  PID_struct_init(&pid_yaw_spd, POSITION_PID, 5000, 1000,
//                  1, 0, 0);
  PID_struct_init(&pid_pit_saber_spd, POSITION_PID, 5000, 1000,
                  2.0f, 0.0f, 0.0f );
  PID_struct_init(&pid_yaw_saber, POSITION_PID, 5000, 1000,
                  5.0f, 0.1f, 25.0f); //	
  PID_struct_init(&pid_yaw_saber_spd, POSITION_PID, 5000, 1000,
                  2.5f, 0.0f, 1.0f ); 
	
}
/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gimbal_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	云台电机控制
	*	@retval	
****************************************************************************************/
void Gimbal_Contrl_Task(void const * argument)
{
	yaw_set->expect=0; 
	pit_set->expect=0;
	yaw_set->ICR_flag=0;
	gimbal_pid_init();
//	Atom_switchModeReq(CONFIG_MODE);
//	IMU_Config();		
	while(1)		
    {


	      IMU_Get_Data();

/*Yaw轴底盘分离跟随*/
		 if(yaw_set->ICR_flag==0||yaw_set->ICR_flag==2) 
		 {
			 yaw_set->ref_flag=1;
			if(yaw_set->ref_flag1==1)
				{ 
					yaw_set->expect=0;
				}
			yaw_set->ref_flag1=0;
			if(yaw_set->ICR_flag==2)
				 yaw_set->ICR_flag = 0;
			
//			pid_calc(&pid_yaw,(int16_t)yaw_get.total_angle,yaw_set->expect);
//			pid_calc(&pid_yaw_spd,-(imu_data.gz-imu_data_offest.gz),pid_yaw.pos_out);
      //Saber
			pid_calc(&pid_yaw,(int16_t)saberDataHandle.euler.yaw,yaw_set->expect);
			pid_calc(&pid_yaw_spd,-(saberDataHandle.gyroCal.gyroZ),pid_yaw.pos_out);
		 }
		else if(yaw_set->ICR_flag==1)
		{
			 yaw_set->ref_flag1=1;
			if(yaw_set->ref_flag==1)
			{ 
				yaw_set->expect=0; 
				ptr_jy901_t_yaw->angle_round=0;
				ptr_jy901_t_yaw->first_angle=ptr_jy901_t_yaw->JY901_angle;
				ptr_jy901_t_yaw->final_angle=0;
			}
			 yaw_set->ref_flag=0;
			
//			pid_calc(&pid_yaw_jy901,(int16_t)ptr_jy901_t_yaw->final_angle,yaw_set->expect);
//			pid_calc(&pid_yaw_spd,-(imu_data.gz-imu_data_offest.gz), pid_yaw_jy901.pos_out);
      //Saber
			pid_calc(&pid_yaw_saber,(int16_t)saberDataHandle.euler.yaw,yaw_set->expect);
			pid_calc(&pid_yaw_spd,-(saberDataHandle.gyroCal.gyroZ), pid_yaw_jy901.pos_out);
		}
			//pit轴
			pid_calc(&pid_pit, (int16_t)pit_get.total_angle,pit_set->expect);
			pid_calc(&pid_pit_spd, -(saberDataHandle.gyroCal.gyroY), pid_pit.pos_out);
     if(0){ /*调试用*/
//			int16_t  *ptr = angle; //初始化指针
//			angle[0]	= (pit_set->expect);
//			angle[1]	= (ptr_jy901_t->final_angle_yaw);
//			angle[2]	= ((int16_t)pid_pit.pos_out);
//			angle[3]	= (int16_t)(pid_pit_spd.pos_out);
//			/*用虚拟示波器，发送数据*/
//			vcan_sendware((uint8_t *)ptr,4*sizeof(angle[0]));
		 }
			Yaw_Current_Value=(int16_t)(-pid_yaw_spd.pos_out);
			Pitch_Current_Value=(int16_t)(-pid_pit_spd.pos_out);
		 
			Cloud_Platform_Motor(&hcan1,Yaw_Current_Value ,	Pitch_Current_Value);
     
     printf("Ax = %f , Ay = %f , Az = %f .\n", 
        saberDataHandle.accKal.accX,
        saberDataHandle.accKal.accY,
        saberDataHandle.accKal.accZ);
			osDelay(5);
   }
 
}
