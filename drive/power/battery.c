#include "Battery.h"
#include "usart.h"
#include "stdio.h"

/************************************************************************/
/*                                                                      */
/************************************************************************/
int battery_ad;

/************************************************************************/
/*                                                                      */
/************************************************************************/
/************************************************************************/
/* ����  ����ʼ����ؼ��ADC                                            */
/* ����  ����                                                           */
/* ����ֵ����                                                           */
/* IO��  ��                                                             */
/*         BATTERY VOLTAGE��PB.01                                       */
/* ��ע  ��PB0-->ADCͨ��8                                               */
/************************************************************************/
void battery_check_init(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	//ʹ��ADC1ͨ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_ADC1, ENABLE );	  

	//����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); 

	//PB0 ��Ϊģ��ͨ����������                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   

	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1

	ADC_ResetCalibration(ADC1);	//ʹ�ܸ�λУ׼  

	while(ADC_GetResetCalibrationStatus(ADC1));	//�ȴ���λУ׼����

	ADC_StartCalibration(ADC1);	 //����ADУ׼

	while(ADC_GetCalibrationStatus(ADC1));	 //�ȴ�У׼����
  
  printf("***��ѹ���AD��ʼ���***\n");  
}

/************************************************************************/
/* ����  �����ADCֵ                                                    */
/* ����  ��                                                             */
/*         ch��ʹ��ͨ��(0~16)                                           */
/* ����ֵ��ת�����                                                     */
/************************************************************************/
u16 get_adc(u8 ch)   
{
	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	  			    

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������	

	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������

	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}

/************************************************************************/
/* ����  ������ͨ��chȡtimes��ת��ֵ��ƽ��ֵ                            */
/* ����  ��                                                             */
/*         ch��ʹ��ͨ��(0~16)                                           */
/*         times��ת������                                              */
/* ����ֵ��ͨ��ch��times��ת�����ƽ��ֵ                                */
/************************************************************************/
u16 get_adc_average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=get_adc(ch);
	}
	return temp_val/times;
} 

/************************************************************************/
/* ����  ����ȡ��ص�ѹ��ADֵ                                           */
/* ����  ����                                                           */
/* ����ֵ����ȡ���ĵ�ص�ѹADֵ                                         */
/************************************************************************/
int get_battery_adc(void)
{
	return get_adc_average(8,10);
}

/************************************************************************/
/* ����  ���õ�ADC�����ڲ��¶ȴ��������¶�ֵ                            */
/* ����  ����                                                           */
/* ����ֵ������ֵ3λ�¶�ֵ XXX*0.1C                                     */
/* ��ע  ����ʱû��                                                     */
/************************************************************************/
int get_temp(void)
{				 
	u16 temp_val=0;
	u8 t;
	float temperate; 

	for(t=0;t<20;t++)//��20��,ȡƽ��ֵ
	{
		temp_val+=get_adc(16);//�¶ȴ�����Ϊͨ��16
	}
	temp_val/=20;
	temperate=(float)temp_val*(3.3/4096);//�õ��¶ȴ������ĵ�ѹֵ
	temperate=(1.43-temperate)/0.0043+25;//�������ǰ�¶�ֵ	 
	temperate*=10;//����ʮ��,ʹ��С�����һλ
	return (int)temperate;	 
}





