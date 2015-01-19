#include "Led.h"
#include "usart.h"
#include "config_com.h"

/************************************************************************/
/* ����  ���ĸ�����LED�Ƴ�ʼ��                                          */
/* ����  ����                                                           */
/* ����ֵ����                                                           */
/* IO��  ��                                                             */
/*         Led1��PA.11                                                  */
/*         Led2��PA.8                                                   */
/*         Led3��PB.1                                                   */
/*         Led4��PB.3                                                   */
/************************************************************************/
void led_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ʹ��GPIOA��GPIOB�˿�ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;		    //LED2-->PA.8�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);				//�����趨������ʼ��GPIOA.8

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;			//LED1-->PA.11�˿�����
	GPIO_Init(GPIOA, &GPIO_InitStructure);				//�����趨������ʼ��GPIOA.11

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	    	//LED3-->PB.1�˿�����,�������
	GPIO_Init(GPIOB, &GPIO_InitStructure);	  			//���������IO���ٶ�Ϊ50MHz

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	    	//LED4-->PB.3�˿�����,�������
	GPIO_Init(GPIOB, &GPIO_InitStructure);	  			//���������IO���ٶ�Ϊ50MHz
  
	/* �ر�JATG,ǧ���ܽ�SWDҲ�رգ�����оƬ���� */
    AFIO->MAPR|=2<<24;
    
	/* �ر��ĸ�LED�� */
	led1_off;
	led2_off;
	led3_off;
	led4_off;
    printf("***״̬LED�Ƴ�ʼ�����***\n");
}
