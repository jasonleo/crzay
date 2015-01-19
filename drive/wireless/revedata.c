/*    
      ____                      _____                  +---+
     / ___\                     / __ \                 | R |
    / /                        / /_/ /                 +---+
   / /   ________  ____  ___  / ____/___  ____  __   __
  / /  / ___/ __ `/_  / / _ \/ /   / __ \/ _  \/ /  / /
 / /__/ /  / /_/ / / /_/  __/ /   / /_/ / / / / /__/ /
 \___/_/   \__,_/ /___/\___/_/    \___ /_/ /_/____  /
                                                 / /
                                            ____/ /
                                           /_____/
ReceiveData.c file
��д�ߣ�С��  (Camel)
����E-mail��375836945@qq.com
���뻷����MDK-Lite  Version: 4.23
����ʱ��: 2014-01-28
���ܣ�
1.���պ����ļ�����������2.4G���ݣ�UART1��������
2.�������ݰ����������Ӧ�Ŀ�����
------------------------------------
*/

  
#include "reveData.h"
//#include "imu.h"
#include "motor.h"
#include "led.h"
#include "MPU6050.h"
#include "extern_variable.h"
#include "usart.h"


//����ɻ������б�Ƕ�
#define  Angle_Max  50.0

uint8_t FLY_ENABLE=0;//����ʹ�ܶ�
//������̬�����������ֿ�����ƫ�Ƶȴ����ĳ�ʼ��ƽ��
int  Rool_error_init;      //����ɻ���ɳ���ƫ��Rool_error_init�����������޸�;����ƫ��Rool_error_init�����������޸�
int  Pitch_error_init;     //����ɻ���ɳ�ǰƫ��Pitch_error_init�����������޸�;����ƫ��Pitch_error_init�����������޸�

RC_GETDATA   RC_DATA;	//���������RC����


//��������ReceiveDataFormNRF()
//���룺��
//���: ��
//���������յ���2.4Gң�����ݸ�ֵ����Ӧ�ı���
void ReceiveDataFormNRF(void)
{
    //PITCH
    RC_DATA.PITCH=NRF24L01_RXDATA[2]-50;//��50����������
    RC_DATA.PITCH = (RC_DATA.PITCH/50.0)*Angle_Max+Pitch_error_init;
    RC_DATA.PITCH=(RC_DATA.PITCH > Angle_Max)  ? (Angle_Max):(RC_DATA.PITCH);
    RC_DATA.PITCH=(RC_DATA.PITCH < -Angle_Max) ? (-Angle_Max):(RC_DATA.PITCH);
    //ROOL
    RC_DATA.ROOL=NRF24L01_RXDATA[3]-50;//��50����������
    RC_DATA.ROOL = (RC_DATA.ROOL/50.0)*Angle_Max+Rool_error_init; 
    RC_DATA.ROOL=(RC_DATA.ROOL > Angle_Max)  ? (Angle_Max):(RC_DATA.ROOL);
    RC_DATA.ROOL=(RC_DATA.ROOL < -Angle_Max) ? (-Angle_Max):(RC_DATA.ROOL);

    //YAW
    RC_DATA.YAW = 5+NRF24L01_RXDATA[4]-50;
    //RC_DATA.YAW = 0;                      //YAW�ǿ������
    RC_DATA.YAW = (RC_DATA.YAW/50.0)*Angle_Max;
    RC_DATA.YAW=(RC_DATA.YAW > Angle_Max)  ? (Angle_Max):(RC_DATA.YAW);
    RC_DATA.YAW=(RC_DATA.YAW < -Angle_Max) ? (-Angle_Max):(RC_DATA.YAW);

    RC_DATA.THROTTLE=NRF24L01_RXDATA[0]+(NRF24L01_RXDATA[1]<<8);
    FLY_ENABLE = NRF24L01_RXDATA[31];   //0xA5��0�������Ƿ�ʹ�ܷ��У���ң��������
}


//��������ReceiveDataFormUART()
//���룺��
//���: ��
//���������յ��Ĵ���ң�����ݸ�ֵ����Ӧ�ı���
void ReceiveDataFormUART(void)
{

  
  if(rx_buffer[0]=='a'&&rx_buffer[1]=='s'&&rx_buffer[2]=='d')
  {

//         UART1_Put_Char(rx_buffer[0]);
//         UART1_Put_Char(rx_buffer[1]);
//         UART1_Put_Char(rx_buffer[2]);
  led1_on;led2_on;led3_on;led4_on;
  }
  else {led1_off;led2_off;led3_off;led4_off;}
  


}








