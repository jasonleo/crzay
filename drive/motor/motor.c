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
/* 功能  ：更新四路的PWM值                                              */
/* 参数  ：                                                             */
/*         motor1_pwm：motor1的pwm值(TIM_Period值决定，输入范围0-999)   */
/*         motor1_pwm：motor2的pwm值(TIM_Period值决定，输入范围0-999)   */
/*         motor3_pwm：motor3的pwm值(TIM_Period值决定，输入范围0-999)   */
/*         motor4_pwm：motor4的pwm值(TIM_Period值决定，输入范围0-999)   */
/* 返回值：无                                                           */
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
    
	/* 对定时器寄存器赋值 */
    TIM2->CCR1 = motor1_pwm;
    TIM2->CCR2 = motor2_pwm;
    TIM2->CCR3 = motor3_pwm;
    TIM2->CCR4 = motor4_pwm;
}

/************************************************************************/
/* 功能  ：输出PWM的定时器2初始化                                       */
/* 参数  ：无                                                           */
/* 返回值：无                                                           */
/************************************************************************/
void motor_pwm_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    u16 pwm_freq = 0;    //控制电机PWM频率
    
    /* 使能GPIOA端口时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/* 使能复用的外设时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	/* 使能定时器2时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 ,ENABLE);
    /* 复位定时器 */
    TIM_DeInit(TIM2);
    
    /* 设置GPIO功能 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* 配置计时器 */
    pwm_freq = (u16) (SystemCoreClock / 24000000) - 1;
    
	/* 周期= 72M/(999+1)/pwm_freq = 3.6K */
    TIM_TimeBaseStructure.TIM_Period = 999;		      //设置在自动重载载周期值，这个值会决定PWM输入范围	
    TIM_TimeBaseStructure.TIM_Prescaler = pwm_freq;	  //设置预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;	  //设置时钟分割：TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);    //初始化TIM2
    
    /* 配置TIM2为PWM输出模式 */
    TIM_OCStructInit(&TIM_OCInitStructure);  //把TIM_OCInitStruct中的每一个参数按缺省值填入
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
    
    /* 启动计时器 */
    TIM_Cmd(TIM2,ENABLE);
    printf("***电机初始化完成***\n");
}
