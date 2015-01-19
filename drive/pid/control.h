#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "stm32f10x.h"


//����PID����
typedef struct PID
{
    float P,
          POUT,
          I,
          IOUT,
          D,
          DOUT,
          IMAX,
          SetPoint,
          NowPoint,
          LastError,
          PrerError;

}PID;

void Controler(void);
void PID_INIT(void);
void PID_Calculate(void);
char  ParameterWrite(void);
void  ParameterRead(void);

extern u16 PIDWriteBuf[3];//д��flash����ʱ���֣���NRF24L01_RXDATA[i]��ֵ 
extern u16 PRWriteBuf[2];//д��flash����ʱ���֣���NRF24L01_RXDATA[i]��ֵ 
extern u16 PowerCouter[3];//��������ͳ��ֵ
extern PID  PID_RP;//����һ��PIID�ṹ��
extern float Yaw_Init;
#endif


