#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "spi.h"
#include "usart.h"
#include "atom_imu.h"
#include "decode.h"

#define maxsize 100
extern uint8_t USART1_RX_DATA[(maxsize)];//ң��
extern uint16_t USART1_RX_NUM;
extern uint8_t USART6_RX_DATA[(maxsize)];//����ϵͳ
extern uint16_t USART6_RX_NUM;
extern uint8_t UART8_RX_DATA[(maxsize)];//���������
extern uint16_t UART8_RX_NUM;

/* ��ģ�����ⲿ�ṩ���������Ͷ��� --------------------------------------------*/
typedef struct TY901_t   //���������    ���Ըĳ����õ�������Ľṹ��moto_measure_t  _����
{
	float err;
	float JY901_angle;
	float JY901_angle_last;
	float first_angle;
	float angle_round;
  float final_angle;
  float last_final_angle;
	uint8_t times;
}JY901_t;
///////////////ң��/////////////////////
typedef struct //ң����������ͨ��
		{ 
			int16_t x; //!< Byte 6-7 
			int16_t y; //!< Byte 8-9 
			int16_t z; //!< Byte 10-11 
			uint8_t press_l; //!< Byte 12 
			uint8_t press_r; //!< Byte 13 
    }Mouse; 
	typedef 	struct 
		{ 
	 uint16_t ch0; 
	 uint16_t ch1; 
	 uint16_t ch2; 
	 uint16_t ch3; 
	 uint8_t s1; 
	 uint8_t s2; 
		}Rc; 
	typedef struct 
		{ 
		  uint16_t v; //!< Byte 14-15 
		}Key; 
		typedef struct 
{ 
  Rc rc; 
  Mouse mouse; 
  Key key; 
}RC_Ctl_t; 
////////////////ң��/////////////////////
/*******************mpu6500*********************************/
typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  
  int16_t temp;
  
  int16_t gx;
  int16_t gy;
  int16_t gz;
  
  int16_t mx;
  int16_t my;
  int16_t mz;
}IMUDataTypedef;

/////****************����ϵͳ*******************/
////typedef __packed struct
////{
////	uint8_t SOF;          //������ʼ�ֽڣ��̶�Ϊ0xA5          
////	uint16_t DataLength;  //���ݳ���
////	uint8_t Seq;          //�����
////	uint8_t CRC8;         //֡ͷCRCУ��
////}tFrameHeader;//֡ͷ

////typedef enum                 //ö�����ͣ�����id_���
////{
////	GameInfo = 0x0001,      //����������״̬    ����Ƶ�� 10 Hz
////	DamagedData = 0x0002,             //�˺����ݣ�ʵʱ����
////	ShootData = 0x0003,                //������ݣ�ʵʱ����
////	PowerANDHeat = 0x0004,							//���ʺ���������50hzƵ��
////	RfidData = 0x0005,								//���ؽ������ݼ�⵽RFID��10hz���ڷ���
////	GameData = 0x0006,								//�����������
////	BuffChangeData = 0x0007,					//buff״̬����buff״̬�ı����һ��
////	PositionData = 0x0008,						//������λ����Ϣ��ǹ�ڳ���λ��
////	SelfDefinedData =0x0100, //ѧ���Զ�������      id��_���  
////	Wrong = 0x1301       //ö����Ч��ֻ��Ϊ��ʹ��ö�ٴ�СΪ2�ֽ�
////}tCmdID; 

////typedef __packed struct
////{
////	uint16_t stageRemainTime;       //����ʣ��ʱ�䣨�ӵ���ʱ�����ӿ�ʼ���㣬��λ s��
////	uint8_t gameProgress;     //��������
////	uint8_t roboLevel;        //�����˵ȼ�
////	uint16_t remainHp;        //ʣ��Ѫ��
////	uint16_t maxHp;           //���Ѫ��
////}tGameInfo; //����������״̬��0x0001��

////typedef __packed struct
////{

////	uint8_t armorType :4;
////	uint8_t hurtType : 4;
////	
////}tDamagedData;   //�˺�����(0x002)

////typedef __packed struct
////{
////	uint8_t bulletType;
////	uint8_t bulletFreq;
////	float  bulletSpeed;
////	
////}tShootData;   //�������(0x003)

////typedef __packed struct
////{
////	
//// float chassisVolt;
//// float chassisCurrent;
//// float chassisPower;
//// float chassisPowerBuffer;
////	uint16_t shootHeat0;
////	uint16_t shootHeat1;
////	
////}tPowerANDHeat;   //���ʺ���������50hzƵ��(0x004)


////typedef __packed struct
////{
////	
////	uint8_t cardType;
////	uint8_t cardldx;

////}tRfidData;							//���ؽ������ݼ�⵽RFID��10hz���ڷ���(0x005)

////typedef __packed struct
////{
////	
////	uint8_t winner;

////}tGameData;								//�����������(0x006)

////typedef __packed struct
////{
////	
////	uint16_t buffMusk;

////}tBuffChangeData;					//buff״̬����buff״̬�ı����һ��(0x007)

////typedef __packed struct
////{
////	
////  float x;
////	float y;
////	float z;
////  float yaw;
////	
////}tPositionData;						//������λ����Ϣ��ǹ�ڳ���λ��(0x008)
////typedef __packed struct
////{
////	float data1;
////	float data2;
////	float data3;
////	uint8_t data4;
////}tSelfDefine;                     //�Զ�������(0x100)


////typedef __packed struct
////{
////	tFrameHeader    FrameHeader;
////	tCmdID          CmdID;
////	__packed union 
////	{
////		tGameInfo    			GameInfo;  				  //����������״̬    ����Ƶ�� 10 Hz
////		tDamagedData  		DamagedData;        //�˺����ݣ�ʵʱ����
////		tShootData     		ShootData;          //������ݣ�ʵʱ����
////		tPowerANDHeat			PowerANDHeat;				//���ʺ���������50hzƵ��
////		tRfidData					RfidData;						//���ؽ������ݼ�⵽RFID��10hz���ڷ���
////		tGameData					GameData;						//�����������
////		tBuffChangeData		BuffChangeData;			//buff״̬����buff״̬�ı����һ��
////		tPositionData			PositionData;				//������λ����Ϣ��ǹ�ڳ���λ��
////		tSelfDefine       SelfDefinedData; 		//ѧ���Զ�������      
////	}Data;
////	uint16_t        CRC16;         //֮ǰ��������CRCУ��   ע������ݺ�֮ǰ�����ݿ��ܲ����������Բ�Ҫֱ��ʹ�ã�����Ҫֱ��ʹ�ã������ڴ˸�ֵ
////}tFrame;  //����֡


//////typedef __packed struct
//////{
//////	tFrameHeader    FrameHeader;
//////	tCmdID          CmdID;
//////  tSelfDefine     SelfDefine;
//////	uint16_t        CRC16;         //֮ǰ��������CRCУ��   ע������ݺ�֮ǰ�����ݿ��ܲ����������Բ�Ҫֱ��ʹ�ã�����Ҫֱ��ʹ�ã������ڴ˸�ֵ
//////}tFrame;  //����֡


////typedef __packed struct
////{
////	tFrameHeader    FrameHeader;
////	tCmdID          CmdID;
////	tGameInfo       GameInfo;   
////	uint16_t        CRC16;         //����CRCУ��
////}tGameInfoFrame;  //����������״̬��0x0001��
////typedef __packed struct
////{
////	tFrameHeader    FrameHeader;
////	tCmdID          CmdID;
////	tDamagedData    DamagedData;   
////	uint16_t        CRC16;         //����CRCУ��
////}tDamagedDataFrame; //ʵʱѪ���仯���ݣ�0x0002��
////typedef __packed struct
////{
////	tFrameHeader    FrameHeader;
////	tCmdID          CmdID;
////	tShootData      ShootData;   
////	uint16_t        CRC16;         //����CRCУ��
////}tShootDataFrame;    //�������(0x003)  

////typedef __packed struct
////{
////	tFrameHeader    FrameHeader;
////	tCmdID          CmdID;
////	tPowerANDHeat   PowerANDHeat;   
////	uint16_t        CRC16;         //����CRCУ��
////}tPowerANDHeatFrame;   //���ʺ���������50hzƵ��(0x004)    

////typedef __packed struct
////{
////	tFrameHeader    FrameHeader;
////	tCmdID          CmdID;
////	tRfidData   		RfidData;   
////	uint16_t        CRC16;         //����CRCУ��
////}tRfidDataFrame;			//���ؽ������ݼ�⵽RFID��10hz���ڷ���(0x005)

////typedef __packed struct
////{
////	tFrameHeader    FrameHeader;
////	tCmdID          CmdID;
////	tGameData   		GameData;   
////	uint16_t        CRC16;         //����CRCУ��
////}tGameDataFrame;								//�����������(0x006)

////typedef __packed struct
////{
////	tFrameHeader    FrameHeader;
////	tCmdID          CmdID;
////	tBuffChangeData BuffChangeData;   
////	uint16_t        CRC16;         //����CRCУ��
////}tBuffChangeDataFrame;					//buff״̬����buff״̬�ı����һ��(0x007);	

////typedef __packed struct
////{
////	tFrameHeader    FrameHeader;
////	tCmdID          CmdID;
////	tPositionData   PositionData;   
////	uint16_t        CRC16;         //����CRCУ��
////}tPositionDataFrame;					 	//������λ����Ϣ��ǹ�ڳ���λ��(0x008)

////typedef __packed struct
////{
////	tFrameHeader    FrameHeader;
////	tCmdID          CmdID;
////	tSelfDefine     SelfDefine;   
////	uint16_t        CRC16;         //����CRCУ��
////}tSelfDefineFrame;               //�Զ�������(0x100);	
/* ��ģ�����ⲿ�ṩ�ĺ궨�� --------------------------------------------------*/

/* ��ģ�����ⲿ�ṩ�Ľӿڳ������� --------------------------------------------*/
/**************���������*******************/
extern JY901_t * ptr_jy901_t_yaw;//�������������
extern JY901_t * ptr_jy901_t_pit;

/*****************mpu6500*****************/
extern uint8_t MPU_id;
extern IMUDataTypedef imu_data;
extern IMUDataTypedef imu_data_offest;
/****************ң��********************/
extern RC_Ctl_t RC_Ctl; //ң������

/* ��ģ�����ⲿ�ṩ�Ľӿں���ԭ������ ----------------------------------------*/
//���������
void JY901_Data_Pro(void);
//ң��
void Remote_Ctrl(void);
//mpu6500
uint8_t MPU6500_Init(void);
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data);
uint8_t MPU6500_Read_Reg(uint8_t const reg);
uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len);
void IMU_Get_Data(void);
uint8_t IST8310_Init(void);
uint8_t IST_Reg_Read_By_MPU(uint8_t addr);
//////����ϵͳ
////void DataVerify(void);
////// ʹ�÷��� �����û����� Send_FrameData(SelfDefinedDara, userMessage,tSelfDefineInfo(userMessage)); 
////void Send_FrameData(tCmdID cmdid, uint8_t * pchMessage,uint8_t dwLength); 
////void sendata(void);

/* ȫ�������� ----------------------------------------*/

/*****************mpu**********************/
//mpu Reg -- Map
#define MPU6500_SELF_TEST_XG        (0x00)
#define MPU6500_SELF_TEST_YG        (0x01)
#define MPU6500_SELF_TEST_ZG        (0x02)
#define MPU6500_SELF_TEST_XA        (0x0D)
#define MPU6500_SELF_TEST_YA        (0x0E)
#define MPU6500_SELF_TEST_ZA        (0x0F)
#define MPU6500_XG_OFFSET_H         (0x13)
#define MPU6500_XG_OFFSET_L         (0x14)
#define MPU6500_YG_OFFSET_H         (0x15)
#define MPU6500_YG_OFFSET_L         (0x16)
#define MPU6500_ZG_OFFSET_H         (0x17)
#define MPU6500_ZG_OFFSET_L         (0x18)
#define MPU6500_SMPLRT_DIV          (0x19)
#define MPU6500_CONFIG              (0x1A)
#define MPU6500_GYRO_CONFIG         (0x1B)
#define MPU6500_ACCEL_CONFIG        (0x1C)
#define MPU6500_ACCEL_CONFIG_2      (0x1D)
#define MPU6500_LP_ACCEL_ODR        (0x1E)
#define MPU6500_MOT_THR             (0x1F)
#define MPU6500_FIFO_EN             (0x23)
#define MPU6500_I2C_MST_CTRL        (0x24)
#define MPU6500_I2C_SLV0_ADDR       (0x25)
#define MPU6500_I2C_SLV0_REG        (0x26)
#define MPU6500_I2C_SLV0_CTRL       (0x27)
#define MPU6500_I2C_SLV1_ADDR       (0x28)
#define MPU6500_I2C_SLV1_REG        (0x29)
#define MPU6500_I2C_SLV1_CTRL       (0x2A)
#define MPU6500_I2C_SLV2_ADDR       (0x2B)
#define MPU6500_I2C_SLV2_REG        (0x2C)
#define MPU6500_I2C_SLV2_CTRL       (0x2D)
#define MPU6500_I2C_SLV3_ADDR       (0x2E)
#define MPU6500_I2C_SLV3_REG        (0x2F)
#define MPU6500_I2C_SLV3_CTRL       (0x30)
#define MPU6500_I2C_SLV4_ADDR       (0x31)
#define MPU6500_I2C_SLV4_REG        (0x32)
#define MPU6500_I2C_SLV4_DO         (0x33)
#define MPU6500_I2C_SLV4_CTRL       (0x34)
#define MPU6500_I2C_SLV4_DI         (0x35)
#define MPU6500_I2C_MST_STATUS      (0x36)
#define MPU6500_INT_PIN_CFG         (0x37)
#define MPU6500_INT_ENABLE          (0x38)
#define MPU6500_INT_STATUS          (0x3A)
#define MPU6500_ACCEL_XOUT_H        (0x3B)
#define MPU6500_ACCEL_XOUT_L        (0x3C)
#define MPU6500_ACCEL_YOUT_H        (0x3D)
#define MPU6500_ACCEL_YOUT_L        (0x3E)
#define MPU6500_ACCEL_ZOUT_H        (0x3F)
#define MPU6500_ACCEL_ZOUT_L        (0x40)
#define MPU6500_TEMP_OUT_H          (0x41)
#define MPU6500_TEMP_OUT_L          (0x42)
#define MPU6500_GYRO_XOUT_H         (0x43)
#define MPU6500_GYRO_XOUT_L         (0x44)
#define MPU6500_GYRO_YOUT_H         (0x45)
#define MPU6500_GYRO_YOUT_L         (0x46)
#define MPU6500_GYRO_ZOUT_H         (0x47)
#define MPU6500_GYRO_ZOUT_L         (0x48)
#define MPU6500_EXT_SENS_DATA_00    (0x49)
#define MPU6500_EXT_SENS_DATA_01    (0x4A)
#define MPU6500_EXT_SENS_DATA_02    (0x4B)
#define MPU6500_EXT_SENS_DATA_03    (0x4C)
#define MPU6500_EXT_SENS_DATA_04    (0x4D)
#define MPU6500_EXT_SENS_DATA_05    (0x4E)
#define MPU6500_EXT_SENS_DATA_06    (0x4F)
#define MPU6500_EXT_SENS_DATA_07    (0x50)
#define MPU6500_EXT_SENS_DATA_08    (0x51)
#define MPU6500_EXT_SENS_DATA_09    (0x52)
#define MPU6500_EXT_SENS_DATA_10    (0x53)
#define MPU6500_EXT_SENS_DATA_11    (0x54)
#define MPU6500_EXT_SENS_DATA_12    (0x55)
#define MPU6500_EXT_SENS_DATA_13    (0x56)
#define MPU6500_EXT_SENS_DATA_14    (0x57)
#define MPU6500_EXT_SENS_DATA_15    (0x58)
#define MPU6500_EXT_SENS_DATA_16    (0x59)
#define MPU6500_EXT_SENS_DATA_17    (0x5A)
#define MPU6500_EXT_SENS_DATA_18    (0x5B)
#define MPU6500_EXT_SENS_DATA_19    (0x5C)
#define MPU6500_EXT_SENS_DATA_20    (0x5D)
#define MPU6500_EXT_SENS_DATA_21    (0x5E)
#define MPU6500_EXT_SENS_DATA_22    (0x5F)
#define MPU6500_EXT_SENS_DATA_23    (0x60)
#define MPU6500_I2C_SLV0_DO         (0x63)
#define MPU6500_I2C_SLV1_DO         (0x64)
#define MPU6500_I2C_SLV2_DO         (0x65)
#define MPU6500_I2C_SLV3_DO         (0x66)
#define MPU6500_I2C_MST_DELAY_CTRL  (0x67)
#define MPU6500_SIGNAL_PATH_RESET   (0x68)
#define MPU6500_MOT_DETECT_CTRL     (0x69)
#define MPU6500_USER_CTRL           (0x6A)
#define MPU6500_PWR_MGMT_1          (0x6B)
#define MPU6500_PWR_MGMT_2          (0x6C)
#define MPU6500_FIFO_COUNTH         (0x72)
#define MPU6500_FIFO_COUNTL         (0x73)
#define MPU6500_FIFO_R_W            (0x74)
#define MPU6500_WHO_AM_I            (0x75)	// mpu6500 id = 0x71
#define MPU6500_XA_OFFSET_H         (0x77)
#define MPU6500_XA_OFFSET_L         (0x78)
#define MPU6500_YA_OFFSET_H         (0x7A)
#define MPU6500_YA_OFFSET_L         (0x7B)
#define MPU6500_ZA_OFFSET_H         (0x7D)
#define MPU6500_ZA_OFFSET_L         (0x7E)
	
#define MPU6050_ID									(0x68)
#define MPU6500_ID									(0x71)			// mpu6500 id = 0x70

#define MPU_IIC_ADDR								0x68

/*********************************************************/

//IST8310 REG address
#define IST8310_ADDRESS           0x0E
#define IST8310_DEVICE_ID_A       0x10

//refer to IST8310 datasheet for more informations
#define IST8310_WHO_AM_I          0x00
#define IST8310_R_CONFA           0x0A
#define IST8310_R_CONFB           0x0B
#define IST8310_R_MODE            0x02

#define IST8310_R_XL              0x03
#define IST8310_R_XM              0x04
#define IST8310_R_YL              0x05
#define IST8310_R_YM              0x06
#define IST8310_R_ZL              0x07
#define IST8310_R_ZM              0x08

#define IST8310_AVGCNTL           0x41
#define IST8310_PDCNTL            0x42

#define IST8310_ODR_MODE          0x01

/******************************************/
#define MPU6500_NSS_Low() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU6500_NSS_High() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)


#endif
