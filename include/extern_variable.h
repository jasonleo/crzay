#ifndef __EXTERN_VARIABLE_H__
#define __EXTERN_VARIABLE_H__
#include "NRF24L01.h"
 
 
 
//ϵͳ
extern uint8_t SystemReady_OK;					//ϵͳ��ʼ����ɱ�־
extern uint8_t FLY_ENABLE;						//���п���
extern uint8_t GYRO_OFFSET_OK;					//��̬��������ɱ�־
extern uint8_t ACC_OFFSET_OK;
extern uint8_t IIC_ERROR_CNT;					//iic���������,ÿ��tim3�жϼ�1,�����ȡ�ɹ���0
extern uint8_t	I2C2_BUSY;
volatile extern uint32_t	TIM3_IRQCNT;			//TIM3�жϼ�����
volatile extern uint32_t	TIM2_IRQCNT;			//TIM3�жϼ�����
volatile extern uint8_t 	MPU6050_I2CData_Ready;		//mpu6050��ȡ��ɱ�־,=1��ʾ��ȡ���
extern uint32_t While1_Lasttime;			             	//�洢whileѭ����ʱ��


//2.4G���ͽ�������
extern uint8_t NRF24L01_RXDATA[RX_PLOAD_WIDTH];		//nrf���յ�����
extern uint8_t NRF24L01_TXDATA[TX_PLOAD_WIDTH];		//nrf��Ҫ���͵�����


//RCң��
typedef struct int16_rcget
{
    float ROOL;
    float PITCH;
    int THROTTLE;
    int YAW;
}RC_GETDATA;


extern RC_GETDATA RC_DATA;//���������RC����
                
                
//������
typedef struct int16_xyz
{
    int16_t X;
    int16_t Y;
    int16_t Z;
}S_INT16_XYZ;


typedef union 
{
    int16_t D[3];
    S_INT16_XYZ V;
    
}U_INT16_XYZ;


extern S_INT16_XYZ 	MPU6050_ACC_LAST,MPU6050_GYRO_LAST;		//����һ�ζ�ȡֵ
extern S_INT16_XYZ 	GYRO_OFFSET,ACC_OFFSET;			        //��Ư


//IMU
typedef struct float_xyz
{
    float X;
    float Y;
    float Z;
    
}S_FLOAT_XYZ;


typedef union 
{
    float D[3];
    S_FLOAT_XYZ V;	
    
}U_FLOAT_XYZ;
   

typedef struct float_angle
{
    float Roll;
    float Pitch;
    float Yaw;
}S_FLOAT_ANGLE;

                
                
                
extern S_FLOAT_XYZ ACC_F,GYRO_F;	//����ת�����ACC��λΪG,GYRO��λΪ��/��
extern S_FLOAT_XYZ GYRO_I[3];		//�����ǻ���

extern S_FLOAT_XYZ DIF_ACC;		//��ּ��ٶ�
extern S_FLOAT_XYZ EXP_ANGLE;		//�����Ƕ�
extern S_FLOAT_XYZ DIF_ANGLE;		//�����Ƕ���ʵ�ʽǶȲ�
extern S_FLOAT_ANGLE Q_ANGLE;		//��Ԫ��������ĽǶ�
extern S_INT16_XYZ ACC_AVG,GYRO_AVG;		//���������˲����ACCƽ��ֵ�ʹ�����gyroֵ
extern uint32_t	IMU_CYCTIME;	//���ζ�ȡsystickʱ�ļ���ֵ��ֵ
                
                
#endif //__EXTERN_VARIABLE_H__
                
        



