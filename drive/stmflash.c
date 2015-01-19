#include "stmflash.h"
#include "delay.h"
#include "usart.h"

#include "stdio.h"

/************************************************************************/
/* 说明：内部flash初始化，模拟EEPROM                                    */
/*       stm32f103t8u6 ：64K Bytes  flash                               */
/*		 小容量stm32最后一页开始地址：0x08007c00，结束地址：0x08007fff  */
/*		 每一页大小为1K个字节                                           */
/************************************************************************/

/************************************************************************/
/*                                                                      */
/************************************************************************/
u16 stmflash_buf[STM_SECTOR_SIZE/2];  //最多是2K字节

/************************************************************************/
/*                                                                      */
/************************************************************************/
/* 解锁STM32的FLASH */
void stmflash_unlock(void)
{
  FLASH_Unlock();	
  printf("***内部FLASH解锁完成***\n");
}

/* 上锁STM32的FLASH */
void stmflash_lock(void)
{
  FLASH_Lock();
  printf("***内部FLASH上锁完成***\n");
}

/************************************************************************/
/* 功能  ：读取指定地址的半字(16位数据)                                 */
/* 参数  ：                                                             */
/*		   faddr:读地址                                                 */
/* 返回值：读到的flash的值                                              */
/************************************************************************/
u16 stmflash_read_halfword(u32 faddr)
{
	return *(vu16*)faddr; 
} 

/************************************************************************/
/* 功能  ：不检查的从指定地址开始写入指定长度的数据                     */
/* 参数  ：                                                             */
/*		   write_addr:起始地址(此地址必须为2的倍数)                     */
/*		   data_buf:数据指针                                            */
/*		   num:半字(16位)数(就是要写入的16位数据的个数)                 */
/* 返回值：无                                                           */
/************************************************************************/
void stmflash_write_nocheck(u32 write_addr, u16 *data_buf, u16 num)   
{ 			 		 
	u16 i;

	for(i=0; i<num; i++)
	{
		FLASH_ProgramHalfWord(write_addr, data_buf[i]);
	    write_addr += 2;  //地址增加2.
	}  
} 

/************************************************************************/
/* 功能  ：从指定地址开始写入指定长度的数据                             */
/* 参数  ：                                                             */
/*		   write_addr:起始地址(此地址必须为2的倍数)                     */
/*		   data_buf:数据指针                                            */
/*		   num:半字(16位)数(就是要写入的16位数据的个数)                 */
/* 返回值：无                                                           */
/************************************************************************/
void stmflash_write(u32 write_addr, u16 *data_buf, u16 num)	
{
	u32 sec_pos;	   //扇区地址
	u16 sec_off;	   //扇区内偏移地址(16位字计算)
	u16 sec_remain;    //扇区内剩余地址(16位字计算)	   
	u32 off_addr;      //去掉0x08000000后的地址
 	u16 i;    

	if(write_addr<STM32_FLASH_BASE||(write_addr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))
		return;  //非法地址

	/* 解锁 */
	stmflash_unlock();

	off_addr   = write_addr-STM32_FLASH_BASE;   //实际偏移地址.
	sec_pos    = off_addr/STM_SECTOR_SIZE;	    //扇区地址  0~127 for STM32F103RBT6
	sec_off    = (off_addr%STM_SECTOR_SIZE)/2;	//在扇区内的偏移(16位，2个字节为基本单位.)
	sec_remain = STM_SECTOR_SIZE/2-sec_off;		//扇区剩余空间大小 

	if(num<=sec_remain)
		sec_remain = num;  //不大于该扇区范围

	while(1) 
	{	
		/* 读出整个扇区的内容 */
		stmflash_read(sec_pos*STM_SECTOR_SIZE+STM32_FLASH_BASE, stmflash_buf, STM_SECTOR_SIZE/2);

		for(i=0; i<sec_remain; i++)//校验数据
		{
			if(stmflash_buf[sec_off+i]!=0XFFFF)
				break;//需要擦除  	  
		}

		if(i<sec_remain)//需要擦除
		{
			FLASH_ErasePage(sec_pos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//擦除这个扇区

			for(i=0; i<sec_remain; i++)//复制
			{
				stmflash_buf[i+sec_off] = data_buf[i];	  
			}
			/* 写入整个扇区 */
			stmflash_write_nocheck(sec_pos*STM_SECTOR_SIZE+STM32_FLASH_BASE, stmflash_buf, STM_SECTOR_SIZE/2);  
		}
		else
		{
			/* 写已经擦除了的,直接写入扇区剩余区间 */
			stmflash_write_nocheck(write_addr, data_buf, sec_remain);
		}

		if(num==sec_remain)
		{
			break;  //写入结束了
		}
		else  //写入未结束
		{
			sec_pos++;				//扇区地址增1
			sec_off     = 0;				//偏移位置为0 	 
		   	data_buf   += sec_remain;  	//指针偏移
			write_addr += sec_remain;	//写地址偏移	   
		   	num        -= sec_remain;	//字节(16位)数递减

			if(num>(STM_SECTOR_SIZE/2))
				sec_remain=STM_SECTOR_SIZE/2;//下一个扇区还是写不完
			else 
				sec_remain=num;//下一个扇区可以写完了
		}	 
	}	
	stmflash_lock();//上锁
}

/************************************************************************/
/* 功能  ：从指定地址开始读出指定长度的数据                             */
/* 参数  ：                                                             */
/*		   read_addr:起始地址(此地址必须为2的倍数)                      */
/*		   data_buf:数据指针                                            */
/*		   num:半字(16位)数(就是要读出的16位数据的个数)                 */
/* 返回值：无                                                           */
/************************************************************************/
void stmflash_read(u32 read_addr, u16 *data_buf, u16 num)   	
{
	u16 i;
	for(i=0; i<num; i++)
	{
		data_buf[i] = stmflash_read_halfword(read_addr);  //读取2个字节.
		num += 2;  //偏移2个字节.	
	}
}

/************************************************************************/
/* 功能  ：从指定地址写入数据                                           */
/* 参数  ：                                                             */
/*		   write_addr:起始地址(此地址必须为2的倍数)                     */
/*		   data:数据指针                                                */
/* 返回值：无                                                           */
/************************************************************************/
void test_write(u32 write_addr, u16 data)   	
{
	stmflash_write(write_addr, &data, 1);  //写入一个字 
}
















