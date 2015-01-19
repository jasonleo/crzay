#include "stmflash.h"
#include "delay.h"
#include "usart.h"

#include "stdio.h"

/************************************************************************/
/* ˵�����ڲ�flash��ʼ����ģ��EEPROM                                    */
/*       stm32f103t8u6 ��64K Bytes  flash                               */
/*		 С����stm32���һҳ��ʼ��ַ��0x08007c00��������ַ��0x08007fff  */
/*		 ÿһҳ��СΪ1K���ֽ�                                           */
/************************************************************************/

/************************************************************************/
/*                                                                      */
/************************************************************************/
u16 stmflash_buf[STM_SECTOR_SIZE/2];  //�����2K�ֽ�

/************************************************************************/
/*                                                                      */
/************************************************************************/
/* ����STM32��FLASH */
void stmflash_unlock(void)
{
  FLASH_Unlock();	
  printf("***�ڲ�FLASH�������***\n");
}

/* ����STM32��FLASH */
void stmflash_lock(void)
{
  FLASH_Lock();
  printf("***�ڲ�FLASH�������***\n");
}

/************************************************************************/
/* ����  ����ȡָ����ַ�İ���(16λ����)                                 */
/* ����  ��                                                             */
/*		   faddr:����ַ                                                 */
/* ����ֵ��������flash��ֵ                                              */
/************************************************************************/
u16 stmflash_read_halfword(u32 faddr)
{
	return *(vu16*)faddr; 
} 

/************************************************************************/
/* ����  �������Ĵ�ָ����ַ��ʼд��ָ�����ȵ�����                     */
/* ����  ��                                                             */
/*		   write_addr:��ʼ��ַ(�˵�ַ����Ϊ2�ı���)                     */
/*		   data_buf:����ָ��                                            */
/*		   num:����(16λ)��(����Ҫд���16λ���ݵĸ���)                 */
/* ����ֵ����                                                           */
/************************************************************************/
void stmflash_write_nocheck(u32 write_addr, u16 *data_buf, u16 num)   
{ 			 		 
	u16 i;

	for(i=0; i<num; i++)
	{
		FLASH_ProgramHalfWord(write_addr, data_buf[i]);
	    write_addr += 2;  //��ַ����2.
	}  
} 

/************************************************************************/
/* ����  ����ָ����ַ��ʼд��ָ�����ȵ�����                             */
/* ����  ��                                                             */
/*		   write_addr:��ʼ��ַ(�˵�ַ����Ϊ2�ı���)                     */
/*		   data_buf:����ָ��                                            */
/*		   num:����(16λ)��(����Ҫд���16λ���ݵĸ���)                 */
/* ����ֵ����                                                           */
/************************************************************************/
void stmflash_write(u32 write_addr, u16 *data_buf, u16 num)	
{
	u32 sec_pos;	   //������ַ
	u16 sec_off;	   //������ƫ�Ƶ�ַ(16λ�ּ���)
	u16 sec_remain;    //������ʣ���ַ(16λ�ּ���)	   
	u32 off_addr;      //ȥ��0x08000000��ĵ�ַ
 	u16 i;    

	if(write_addr<STM32_FLASH_BASE||(write_addr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))
		return;  //�Ƿ���ַ

	/* ���� */
	stmflash_unlock();

	off_addr   = write_addr-STM32_FLASH_BASE;   //ʵ��ƫ�Ƶ�ַ.
	sec_pos    = off_addr/STM_SECTOR_SIZE;	    //������ַ  0~127 for STM32F103RBT6
	sec_off    = (off_addr%STM_SECTOR_SIZE)/2;	//�������ڵ�ƫ��(16λ��2���ֽ�Ϊ������λ.)
	sec_remain = STM_SECTOR_SIZE/2-sec_off;		//����ʣ��ռ��С 

	if(num<=sec_remain)
		sec_remain = num;  //�����ڸ�������Χ

	while(1) 
	{	
		/* ������������������ */
		stmflash_read(sec_pos*STM_SECTOR_SIZE+STM32_FLASH_BASE, stmflash_buf, STM_SECTOR_SIZE/2);

		for(i=0; i<sec_remain; i++)//У������
		{
			if(stmflash_buf[sec_off+i]!=0XFFFF)
				break;//��Ҫ����  	  
		}

		if(i<sec_remain)//��Ҫ����
		{
			FLASH_ErasePage(sec_pos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//�����������

			for(i=0; i<sec_remain; i++)//����
			{
				stmflash_buf[i+sec_off] = data_buf[i];	  
			}
			/* д���������� */
			stmflash_write_nocheck(sec_pos*STM_SECTOR_SIZE+STM32_FLASH_BASE, stmflash_buf, STM_SECTOR_SIZE/2);  
		}
		else
		{
			/* д�Ѿ������˵�,ֱ��д������ʣ������ */
			stmflash_write_nocheck(write_addr, data_buf, sec_remain);
		}

		if(num==sec_remain)
		{
			break;  //д�������
		}
		else  //д��δ����
		{
			sec_pos++;				//������ַ��1
			sec_off     = 0;				//ƫ��λ��Ϊ0 	 
		   	data_buf   += sec_remain;  	//ָ��ƫ��
			write_addr += sec_remain;	//д��ַƫ��	   
		   	num        -= sec_remain;	//�ֽ�(16λ)���ݼ�

			if(num>(STM_SECTOR_SIZE/2))
				sec_remain=STM_SECTOR_SIZE/2;//��һ����������д����
			else 
				sec_remain=num;//��һ����������д����
		}	 
	}	
	stmflash_lock();//����
}

/************************************************************************/
/* ����  ����ָ����ַ��ʼ����ָ�����ȵ�����                             */
/* ����  ��                                                             */
/*		   read_addr:��ʼ��ַ(�˵�ַ����Ϊ2�ı���)                      */
/*		   data_buf:����ָ��                                            */
/*		   num:����(16λ)��(����Ҫ������16λ���ݵĸ���)                 */
/* ����ֵ����                                                           */
/************************************************************************/
void stmflash_read(u32 read_addr, u16 *data_buf, u16 num)   	
{
	u16 i;
	for(i=0; i<num; i++)
	{
		data_buf[i] = stmflash_read_halfword(read_addr);  //��ȡ2���ֽ�.
		num += 2;  //ƫ��2���ֽ�.	
	}
}

/************************************************************************/
/* ����  ����ָ����ַд������                                           */
/* ����  ��                                                             */
/*		   write_addr:��ʼ��ַ(�˵�ַ����Ϊ2�ı���)                     */
/*		   data:����ָ��                                                */
/* ����ֵ����                                                           */
/************************************************************************/
void test_write(u32 write_addr, u16 data)   	
{
	stmflash_write(write_addr, &data, 1);  //д��һ���� 
}
















