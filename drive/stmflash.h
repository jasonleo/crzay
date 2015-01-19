#ifndef __STMFLASH_H__
#define __STMFLASH_H__

#include "stm32f10x.h"

/************************************************************************/
/*                                                                      */
/************************************************************************/
/* 用户根据自己的STM32的FLASH容量大小(单位为K)设置 */
#define STM32_FLASH_SIZE 	64
#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024   //字节
#else 
#define STM_SECTOR_SIZE	2048
#endif

/* FLASH起始地址 */
#define STM32_FLASH_BASE   0x08000000 	//STM32 FLASH的起始地址
#define STM32_FLASH_OFFEST 0x00007c00 	//STM32 FLASH的起始地址

/* FLASH解锁键值 */
#define FLASH_KEY1         0X45670123
#define FLASH_KEY2         0XCDEF89AB

/************************************************************************/
/*                                                                      */
/************************************************************************/
	void stmflash_unlock(void);					  //FLASH解锁
	void stmflash_lock(void);					  //FLASH上锁
	u16  stmflash_read_halfword(u32 faddr);		  //读出半字  
	void stmflash_write(u32 write_addr, u16 *data_buf, u16 num);		//从指定地址开始写入指定长度的数据
	void stmflash_read(u32 read_addr, u16 *data_buf, u16 num);   		//从指定地址开始读出指定长度的数据
	void stmflash_w_lenbyte(u32 write_addr, u32 data_buf, u16 len);	//从指定地址开始写入指定长度的数据
	u32  stmflash_r_lenbyte(u32 read_addr, u16 len);				    //从指定地址开始读出指定长度的数据

	//测试写入
	void test_write(u32 write_addr, u16 data);	

/************************************************************************/
/*                                                                      */
/************************************************************************/

#endif //__STMFLASH_H__


















