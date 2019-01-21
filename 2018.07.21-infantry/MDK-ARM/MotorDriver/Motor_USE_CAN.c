/**
  ******************************************************************************
  * @file           : Motor_USE_CAN.c
  * @brief          : �����ģ����ʹ��CANͨ�ŵĵ��
  ******************************************************************************
  *ʹ��CANͨѶ�ĵ������̨���   		 ���̵��	 	 	  �������
	*				 	��Ӧ�ͺţ� c620						3508					 C2000
  *�ӿں�����
	*					Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch)
	*					Chassis_Motor( CAN_HandleTypeDef * hcan,
	*								  int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
	*					Allocate_Motor(CAN_HandleTypeDef * hcan,int16_t value)
  *�ӿڱ���: moto_measure_t   moto_chassis_get[4] = {0};//4 �� 3508
	*					 moto_measure_t   moto_dial_get = {0};  //c2006
	*					 moto_measure_t   pit_get;
	*			   	 moto_measure_t   yaw_get;
	*					
  ******************************************************************************
  */
#include "Motor_USE_CAN.h"
/*******************Ħ���ֵ���͵��̵���Ĳ�������***************************/
moto_measure_t   moto_chassis_get[4] = {0};//4 �� 3508
moto_measure_t   moto_dial_get = {0};  //c2006
moto_measure_t   pit_get;
moto_measure_t   yaw_get;
//Ϊcan���ͷֱ𴴽����棬��ֹ���ڷ��͵�ʱ����ֻ��һ���ڴ���໥����
static CanTxMsgTypeDef  Cloud_Platform_Data;
static CanTxMsgTypeDef	 Chassis_Motor_Data;
static CanTxMsgTypeDef  Allocate_Motor_Data;
/**
	**************************************************************
	** Descriptions: ��̨�����������
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN1
	**					yaw:yaw�����ֵ
	**				pitch:pitch����ֵ
	** Output: NULL
	**************************************************************
**/
void Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch)
{
		Cloud_Platform_Data.StdId = 0x1FF;
		Cloud_Platform_Data.IDE = CAN_ID_STD;
		Cloud_Platform_Data.RTR = CAN_RTR_DATA;
		Cloud_Platform_Data.DLC = 0X08;
		
		Cloud_Platform_Data.Data[0] = yaw>>8;
		Cloud_Platform_Data.Data[1] = yaw;
		Cloud_Platform_Data.Data[2] = pitch>>8;
		Cloud_Platform_Data.Data[3] = pitch;
		Cloud_Platform_Data.Data[4] = 0x00;
		Cloud_Platform_Data.Data[5] = 0x00;
		Cloud_Platform_Data.Data[6] = 0x00;
		Cloud_Platform_Data.Data[7] = 0x00;
//	printf("\r\n pitch�� set=3520,fdb=%d,output=%d",prt_Gimbal_measure->GM_Pitch_Position,Pitch_Current_Value);
    hcan->pTxMsg = &Cloud_Platform_Data;
		HAL_CAN_Transmit(hcan,100);
}


/**
	**************************************************************
	** Descriptions: ���̵����������
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN2
	**					iqn:��n�����̵���ĵ���ֵ
	** Output: NULL
	**************************************************************
**/
void Chassis_Motor( CAN_HandleTypeDef * hcan,
									  int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
			Chassis_Motor_Data.DLC = 0x08;
			Chassis_Motor_Data.IDE = CAN_ID_STD;
			Chassis_Motor_Data.RTR = CAN_RTR_DATA;
			Chassis_Motor_Data.StdId = 0x200;

			Chassis_Motor_Data.Data[0]=iq1>>8;
			Chassis_Motor_Data.Data[1]=iq1;
			Chassis_Motor_Data.Data[2]=iq2>>8;
			Chassis_Motor_Data.Data[3]=iq2;
			Chassis_Motor_Data.Data[4]=iq3>>8;
			Chassis_Motor_Data.Data[5]=iq3;
			Chassis_Motor_Data.Data[6]=iq4>>8;
			Chassis_Motor_Data.Data[7]=iq4;
	
			hcan->pTxMsg = &Chassis_Motor_Data;
			HAL_CAN_Transmit(hcan,100);
}	

/**
	**************************************************************
	** Descriptions: ���������������
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN1
	**				value:��������ĵ���ֵ
	** Output: NULL
	**************************************************************
**/
void Allocate_Motor(CAN_HandleTypeDef * hcan,int16_t value)
{

			Allocate_Motor_Data.DLC = 0x08;
			Allocate_Motor_Data.IDE = CAN_ID_STD;
			Allocate_Motor_Data.RTR = CAN_RTR_DATA;
			Allocate_Motor_Data.StdId = 0x200;

			Allocate_Motor_Data.Data[0]=value>>8;
			Allocate_Motor_Data.Data[1]=value;
			Allocate_Motor_Data.Data[2]=0;
			Allocate_Motor_Data.Data[3]=0;
			Allocate_Motor_Data.Data[4]=0;
			Allocate_Motor_Data.Data[5]=0;
			Allocate_Motor_Data.Data[6]=0;
			Allocate_Motor_Data.Data[7]=0;
	
			hcan->pTxMsg = &Allocate_Motor_Data;
			HAL_CAN_Transmit(hcan,100);
}
/**                                                           //����
	**************************************************************
	** Descriptions: ��ȡCANͨѶ�ĵ���ķ���ֵ
	** Input: 	
	**			  ptr:Ŀ�����ݵ��ڴ��ַ
	**				hcan->pRxMsg->Data:���������CAN�����ݵ�����
	** Output: NULL
	**************************************************************
**/
void get_moto_measure(moto_measure_t *ptr,CAN_HandleTypeDef * hcan)
{
	/*BUG!!! dont use this para code*/

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	ptr->real_current  = (int16_t)(hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]);
	ptr->speed_rpm = ptr->real_current;
	ptr->given_current = (int16_t)(hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5]);
	ptr->hall = hcan->pRxMsg->Data[6];
//	ptr->angle = (uint16_t)(prt_CanReceiveData->CAN_RX_Data[0]<<8 | prt_CanReceiveData->CAN_RX_Data[1]) ;  _ע��
//	ptr->real_current  = (int16_t)(prt_CanReceiveData->CAN_RX_Data[2]<<8 | prt_CanReceiveData->CAN_RX_Data[3]);
//	ptr->speed_rpm = ptr->real_current;
//	ptr->given_current = (int16_t)(prt_CanReceiveData->CAN_RX_Data[4]<<8 | prt_CanReceiveData->CAN_RX_Data[5]);
//	ptr->hall = prt_CanReceiveData->CAN_RX_Data[6];
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}
/**
	**************************************************************
	** Descriptions:��ȡ�������ֵ��ƫ��ֵ
	** Input: 	
	**			  ptr:Ŀ�����ݵ��ڴ��ַ
	**				hcan->pRxMsg->Data:���������CAN�����ݵ�����
	** Output: NULL
	**************************************************************
**/
/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr,CAN_HandleTypeDef * hcan)
{
	ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 |hcan->pRxMsg->Data[1]) ;
	ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
/**
	**************************************************************
	** Descriptions: ��ȡ������ܽǶ�ֵ
	** Input: 	
	**			   *P:��Ҫ��ȡ�ܽǶ�ֵ�ĵ�ַ
	**				
	** Output: NULL
	**************************************************************
**/
void get_total_angle(moto_measure_t *p){
	
	int res1, res2, delta;
	if(p->angle < p->last_angle){			//?????
		res1 = p->angle + 8192 - p->last_angle;	//??,delta=+
		res2 = p->angle - p->last_angle;				//??	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//??	delta -
		res2 = p->angle - p->last_angle;				//??	delta +
	}
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}
