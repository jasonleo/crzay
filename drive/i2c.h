#ifndef __IIC_H__
#define __IIC_H__

#include "stm32f10x.h"

/************************************************************************/
/*                                                                      */
/************************************************************************/
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))  

//IO口地址映射
//#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
//#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
//#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
//#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
//#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
//#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

//#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
//#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40s011008 
//#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x4001140s8 
//#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
//#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
//#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 


#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 
   	   		   
////驱动接口，GPIO模拟IIC
//SCL-->PB6
//SDA-->PB7
#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=0x80000000;}
#define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=0x30000000;}

//IO操作函数	 
#define IIC_SCL    PBout(6) //SCL
#define IIC_SDA    PBout(7) //SDA	 
#define READ_SDA   PBin(7)  //输入SDA 

/************************************************************************/
/*                                                                      */
/************************************************************************/
	//IIC所有操作函数
	void i2c_init(void);          //初始化IIC的IO口				 
	void i2c_start(void);			   	//发送IIC开始信号
	void i2c_stop(void);	  			//发送IIC停止信号
	void i2c_send_byte(u8 txd);			//IIC发送一个字节
	u8 i2c_read_byte(unsigned char ack);//IIC读取一个字节
	u8 i2c_wait_ack(void); 				//IIC等待ACK信号
	void i2c_ack(void);					//IIC发送ACK信号
	void i2c_nack(void);				//IIC不发送ACK信号

	void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
	u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 
	unsigned char I2C_Readkey(unsigned char I2C_Addr);

	unsigned char i2c_read_onebyte(unsigned char I2C_Addr,unsigned char addr);
	unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
	u8 i2c_write_bytes(u8 dev, u8 reg, u8 length, u8* data);
	u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data);
	u8 IICwriteBit(u8 dev,u8 reg,u8 bitNum,u8 data);
	u8 i2c_read_bytes(u8 dev, u8 reg, u8 length, u8 *data);

/************************************************************************/
/*                                                                      */
/************************************************************************/
#endif  //__IIC_H__








