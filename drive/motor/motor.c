#include "stm32f10x.h"
#include "motor.h"
#include "usart.h"
#include "stdio.h"

/************************************************************************/
/*                                                                      */
/************************************************************************/
s16 motor1_pwm = 0;
s16 motor2_pwm = 0;
s16 motor3_pwm = 0;
s16 motor4_pwm = 0;
/************************************************************************/
/*                                                                      */
/************************************************************************/
/************************************************************************/
/* ����  ��������·��PWMֵ                                              */
/* ����  ��                                                             */
/*         motor1_pwm��motor1��pwmֵ(TIM_Periodֵ���������뷶Χ0-999)   */
/*         motor1_pwm��motor2��pwmֵ(TIM_Periodֵ���������뷶Χ0-999)   */
/*         motor3_pwm��motor3��pwmֵ(TIM_Periodֵ���������뷶Χ0-999)   */
/*         motor4_pwm��motor4��pwmֵ(TIM_Periodֵ���������뷶Χ0-999)   */
/* ����ֵ����                                                           */
/************************************************************************/
void motor_pwm_update(s16 motor1_pwm,s16 motor2_pwm,s16 motor3_pwm,s16 motor4_pwm)
{	
	/**/
	if(motor1_pwm>=MOTOR_PWM_MAX)	
		motor1_pwm = MOTOR_PWM_MAX;
	if(motor2_pwm>=MOTOR_PWM_MAX)	
		motor2_pwm = MOTOR_PWM_MAX;
	if(motor3_pwm>=MOTOR_PWM_MAX)	
		motor3_pwm = MOTOR_PWM_MAX;
	if(motor4_pwm>=MOTOR_PWM_MAX)	
		motor4_pwm = MOTOR_PWM_MAX;

	if(motor1_pwm<=0)	
		motor1_pwm = 0;
	if(motor2_pwm<=0)	
		motor2_pwm = 0;
	if(motor3_pwm<=0)	
		motor3_pwm = 0;
	if(motor4_pwm<=0)	
		motor4_pwm = 0;
    
	/* �Զ�ʱ���Ĵ�����ֵ */
    TIM2->CCR1 = motor1_pwm;
    TIM2->CCR2 = motor2_pwm;
    TIM2->CCR3 = motor3_pwm;
    TIM2->CCR4 = motor4_pwm;
}

/************************************************************************/
/* ����  �����PWM�Ķ�ʱ��2��ʼ��                                       */
/* ����  ����                                                           */
/* ����ֵ����                                                           */
/************************************************************************/
void motor_pwm_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    u16 pwm_freq = 0;    //���Ƶ��PWMƵ��
    
    /* ʹ��GPIOA�˿�ʱ�� */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/* ʹ�ܸ��õ�����ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	/* ʹ�ܶ�ʱ��2ʱ�� */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 ,ENABLE);
    /* ��λ��ʱ�� */
    TIM_DeInit(TIM2);
    
    /* ����GPIO���� */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* ���ü�ʱ�� */
    pwm_freq = (u16) (SystemCoreClock / 24000000) - 1;
    
	/* ����= 72M/(999+1)/pwm_freq = 3.6K */
    TIM_TimeBaseStructure.TIM_Period = 999;		      //�������Զ�����������ֵ�����ֵ�����PWM���뷶Χ	
    TIM_TimeBaseStructure.TIM_Prescaler = pwm_freq;	  //����Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;	  //����ʱ�ӷָTDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);    //��ʼ��TIM2
    
    /* ����TIM2ΪPWM���ģʽ */
    TIM_OCStructInit(&TIM_OCInitStructure);  //��TIM_OCInitStruct�е�ÿһ��������ȱʡֵ����
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;    
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    
    TIM_OC1Init(TIM2,&TIM_OCInitStructure);
    TIM_OC2Init(TIM2,&TIM_OCInitStructure);
    TIM_OC3Init(TIM2,&TIM_OCInitStructure);
    TIM_OC4Init(TIM2,&TIM_OCInitStructure);
    
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    
    /* ������ʱ�� */
    TIM_Cmd(TIM2,ENABLE);
    printf("***�����ʼ�����***\n");
}
