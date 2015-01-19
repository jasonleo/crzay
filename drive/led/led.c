#include "Led.h"
#include "usart.h"
#include "config_com.h"

/************************************************************************/
/* 功能  ：四个臂上LED灯初始化                                          */
/* 参数  ：无                                                           */
/* 返回值：无                                                           */
/* IO口  ：                                                             */
/*         Led1：PA.11                                                  */
/*         Led2：PA.8                                                   */
/*         Led3：PB.1                                                   */
/*         Led4：PB.3                                                   */
/************************************************************************/
void led_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 使能GPIOA和GPIOB端口时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;		    //LED2-->PA.8端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO口速度为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);				//根据设定参数初始化GPIOA.8

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;			//LED1-->PA.11端口配置
	GPIO_Init(GPIOA, &GPIO_InitStructure);				//根据设定参数初始化GPIOA.11

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	    	//LED3-->PB.1端口配置,推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);	  			//推挽输出，IO口速度为50MHz

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	    	//LED4-->PB.3端口配置,推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);	  			//推挽输出，IO口速度为50MHz
  
	/* 关闭JATG,千万不能将SWD也关闭，否则芯片作废 */
    AFIO->MAPR|=2<<24;
    
	/* 关闭四个LED灯 */
	led1_off;
	led2_off;
	led3_off;
	led4_off;
    printf("***状态LED灯初始化完成***\n");
}
