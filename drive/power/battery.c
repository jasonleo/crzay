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
/* 功能  ：初始化电池检测ADC                                            */
/* 参数  ：无                                                           */
/* 返回值：无                                                           */
/* IO口  ：                                                             */
/*         BATTERY VOLTAGE：PB.01                                       */
/* 备注  ：PB0-->ADC通道8                                               */
/************************************************************************/
void battery_check_init(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	//使能ADC1通道时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_ADC1, ENABLE );	  

	//设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); 

	//PB0 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   

	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1

	ADC_ResetCalibration(ADC1);	//使能复位校准  

	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束

	ADC_StartCalibration(ADC1);	 //开启AD校准

	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
  
  printf("***电压监测AD初始完成***\n");  
}

/************************************************************************/
/* 功能  ：获得ADC值                                                    */
/* 参数  ：                                                             */
/*         ch：使用通道(0~16)                                           */
/* 返回值：转换结果                                                     */
/************************************************************************/
u16 get_adc(u8 ch)   
{
	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道,采样时间为239.5周期	  			    

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能	

	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}

/************************************************************************/
/* 功能  ：计算通道ch取times次转换值后平均值                            */
/* 参数  ：                                                             */
/*         ch：使用通道(0~16)                                           */
/*         times：转换次数                                              */
/* 返回值：通道ch的times次转换结果平均值                                */
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
/* 功能  ：获取电池电压的AD值                                           */
/* 参数  ：无                                                           */
/* 返回值：获取到的电池电压AD值                                         */
/************************************************************************/
int get_battery_adc(void)
{
	return get_adc_average(8,10);
}

/************************************************************************/
/* 功能  ：得到ADC采样内部温度传感器的温度值                            */
/* 参数  ：无                                                           */
/* 返回值：返回值3位温度值 XXX*0.1C                                     */
/* 备注  ：暂时没用                                                     */
/************************************************************************/
int get_temp(void)
{				 
	u16 temp_val=0;
	u8 t;
	float temperate; 

	for(t=0;t<20;t++)//读20次,取平均值
	{
		temp_val+=get_adc(16);//温度传感器为通道16
	}
	temp_val/=20;
	temperate=(float)temp_val*(3.3/4096);//得到温度传感器的电压值
	temperate=(1.43-temperate)/0.0043+25;//计算出当前温度值	 
	temperate*=10;//扩大十倍,使用小数点后一位
	return (int)temperate;	 
}





