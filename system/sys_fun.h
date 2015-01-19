#ifndef __SYS_FUN_H__
#define __SYS_FUN_H__

void NVIC_INIT(void);
char SystemClock_HSI(u8 PLL);
char SystemClock_HSE(u8 PLL);
void PowerOn(void);

extern char SysClock;       //申请存储系统时钟变量，单位MHz

#endif //__SYS_FUN_H__

