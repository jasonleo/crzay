#ifndef __STMFLASH_H__
#define __STMFLASH_H__

#include "stm32f10x.h"

/************************************************************************/
/*                                                                      */
/************************************************************************/
/* �û������Լ���STM32��FLASH������С(��λΪK)���� */
#define STM32_FLASH_SIZE 	64
#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024   //�ֽ�
#else 
#define STM_SECTOR_SIZE	2048
#endif

/* FLASH��ʼ��ַ */
#define STM32_FLASH_BASE   0x08000000 	//STM32 FLASH����ʼ��ַ
#define STM32_FLASH_OFFEST 0x00007c00 	//STM32 FLASH����ʼ��ַ

/* FLASH������ֵ */
#define FLASH_KEY1         0X45670123
#define FLASH_KEY2         0XCDEF89AB

/************************************************************************/
/*                                                                      */
/************************************************************************/
	void stmflash_unlock(void);					  //FLASH����
	void stmflash_lock(void);					  //FLASH����
	u16  stmflash_read_halfword(u32 faddr);		  //��������  
	void stmflash_write(u32 write_addr, u16 *data_buf, u16 num);		//��ָ����ַ��ʼд��ָ�����ȵ�����
	void stmflash_read(u32 read_addr, u16 *data_buf, u16 num);   		//��ָ����ַ��ʼ����ָ�����ȵ�����
	void stmflash_w_lenbyte(u32 write_addr, u32 data_buf, u16 len);	//��ָ����ַ��ʼд��ָ�����ȵ�����
	u32  stmflash_r_lenbyte(u32 read_addr, u16 len);				    //��ָ����ַ��ʼ����ָ�����ȵ�����

	//����д��
	void test_write(u32 write_addr, u16 data);	

/************************************************************************/
/*                                                                      */
/************************************************************************/

#endif //__STMFLASH_H__


















