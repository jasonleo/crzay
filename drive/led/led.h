#ifndef __LED_H__
#define __LED_H__

//#include "stm32f10x.h"

/************************************************************************/
/*                                                                      */
/************************************************************************/
#define led1_on    GPIO_SetBits(GPIOA, GPIO_Pin_11)
#define led1_off   GPIO_ResetBits(GPIOA, GPIO_Pin_11)

#define led2_on    GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define led2_off   GPIO_ResetBits(GPIOA, GPIO_Pin_8)

#define led3_on    GPIO_SetBits(GPIOB, GPIO_Pin_1)
#define led3_off   GPIO_ResetBits(GPIOB, GPIO_Pin_1)

#define led4_on    GPIO_SetBits(GPIOB, GPIO_Pin_3)
#define led4_off   GPIO_ResetBits(GPIOB, GPIO_Pin_3)

/************************************************************************/
/*                                                                      */
/************************************************************************/
	void led_init(void);

/************************************************************************/
/*                                                                      */
/************************************************************************/
#endif //__LED_H__

