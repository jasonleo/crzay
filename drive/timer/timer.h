#ifndef _tim_H_
#define _tim_H_
#include "stm32f10x.h"

//void TIM4_Init(char clock,int Preiod);//���ڼ��ϵͳ
//void TIM3_Init(char clock,int Preiod);//��ʱ��3�ĳ�ʼ��

void TIM3_Int_Init(u16 arr,u16 psc);
void TIM4_Int_Init(u16 arr,u16 psc);

void TimerNVIC_Configuration(void);//��ʱ���ж�����������
#endif

