#include "bt.h"
#include "usart.h"
#include "stdio.h"

/************************************************************************/
/* ����  ����ʼ��BTģ��ĵ�Դ                                           */
/* ����  ����                                                           */
/* ����ֵ����                                                           */
/* IO��  ��                                                             */
/*         BT_POWER_EN��PB.2                                            */
/************************************************************************/
void bt_power_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ʹ��GPIOB�˿�ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;		    //BT_POWER_EN-->PB.2�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);				//�����趨������ʼ��GPIOB.2

	/* Ĭ�Ϲر� */
    BT_OFF();                
}
