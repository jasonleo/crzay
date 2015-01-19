#include "bt.h"
#include "usart.h"
#include "stdio.h"

/************************************************************************/
/* 功能  ：初始化BT模块的电源                                           */
/* 参数  ：无                                                           */
/* 返回值：无                                                           */
/* IO口  ：                                                             */
/*         BT_POWER_EN：PB.2                                            */
/************************************************************************/
void bt_power_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 使能GPIOB端口时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;		    //BT_POWER_EN-->PB.2端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);				//根据设定参数初始化GPIOB.2

	/* 默认关闭 */
    BT_OFF();                
}
