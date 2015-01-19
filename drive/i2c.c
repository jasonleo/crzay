/*    
功能：
1.初始化软件IIC协议
2.软件IIC协议的引脚，也是STM32硬件IIC的引脚。只是我没有这个功能
------------------------------------
*/
//STM32模拟IIC协议，STM32的硬件IIC有些BUG
//细节有空再改
//最后修改:2014-03-11


#include "I2C.h"
#include "delay.h"
#include "Led.h"
#include "UART1.h"
#include "stdio.h"

/************************************************************************/
/*                                                                      */
/************************************************************************/
/************************************************************************/
/* 功能  ：初始化I2C                                                    */
/* 参数  ：无                                                           */
/* 返回值：无                                                           */
/* IO口  ：                                                             */
/*         SCL：PB.06                                                   */
/*         SDA：PB.07                                                   */
/************************************************************************/
void i2c_init(void)
{			
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 使能GPIOB口的时钟 */
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);			     
 	
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  //PB.06  PB.07	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推免输出       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  
	/* 应用配置到GPIOB */
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* PB.06 PB.07输出高电平 */
	GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);

	printf("***IIC总线初始化完成***\n");
}

/************************************************************************/
/* 功能  ：产生IIC起始信号                                              */
/* 参数  ：无                                                           */
/* 返回值：无                                                           */
/************************************************************************/
void i2c_start(void)
{
	SDA_OUT();  //sda线输出 
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;  //START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;  //钳住I2C总线，准备发送或接收数据 
}

/************************************************************************/
/* 功能  ：产生IIC停止信号                                              */
/* 参数  ：无                                                           */
/* 返回值：无                                                           */
/************************************************************************/
void i2c_stop(void)
{
	SDA_OUT();  //sda线输出
	IIC_SCL=0;
	IIC_SDA=0;  //STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;  //发送I2C总线结束信号
	delay_us(4);							   	
}

/************************************************************************/
/* 功能  ：等待应答信号到来                                             */
/* 参数  ：无                                                           */
/* 返回值：                                                             */
/*         1：接收应答失败                                              */
/*         0：接收应答成功                                              */
/************************************************************************/
u8 i2c_wait_ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;
	delay_us(1);	   
	IIC_SCL=1;
	delay_us(1);	 

	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			i2c_stop();
			return 1;
		}
	  delay_us(1);
	}

	IIC_SCL=0;  //时钟输出0 	   
	return 0;  
} 

/************************************************************************/
/* 功能  ：产生ACK应答                                                  */
/* 参数  ：无                                                           */
/* 返回值：无                                                           */
/************************************************************************/
void i2c_ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}

/************************************************************************/
/* 功能  ：产生NACK应答                                                 */
/* 参数  ：无                                                           */
/* 返回值：无                                                           */
/************************************************************************/
void i2c_nack(void)
{
	IIC_SCL=0;
	SDA_OUT();  
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     

/************************************************************************/
/* 功能  ：IIC发送一个字节                                              */
/* 参数  ：                                                             */
/*         txd：要发送的数据                                            */
/* 返回值：无                                                           */
/************************************************************************/
void i2c_send_byte(u8 txd)
{                        
    u8 t;  

	SDA_OUT(); 	    
    IIC_SCL=0;  //拉低时钟开始数据传输

    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	 

/************************************************************************/
/* 功能  ：IIC读取一个字节                                              */
/* 参数  ：                                                             */
/*         ack：发送ACK的标志(ack=1时，发送ACK，ack=0，发送nACK)        */
/* 返回值：接收到的数据                                                 */
/************************************************************************/
u8 i2c_read_byte(unsigned char ack)
{
	unsigned char i;
	unsigned char receive=0;

	SDA_IN()  ;//SDA设置为输入

    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;

        if(READ_SDA)
			receive++;   
		
		delay_us(2); 
    }

    if (ack)
        i2c_ack();  //发送ACK 
    else
        i2c_nack(); //发送nACK 

    return receive;
}

/************************************************************************/
/* 功能  ：读取指定设备、指定寄存器的一个值                             */
/* 参数  ：                                                             */
/*         i2c_addr：目标设备地址                                       */
/*         addr：寄存器地址                                             */
/* 返回值：读取到的数据                                                 */
/************************************************************************/
unsigned char i2c_read_onebyte(unsigned char i2c_addr,unsigned char addr)
{
	unsigned char res=0;
	
	i2c_start();	

	i2c_send_byte(i2c_addr);  //发送写命令
	res++;
	i2c_wait_ack();

	i2c_send_byte(addr); 
	res++;  //发送地址
	i2c_wait_ack();	  

	//IIC_Stop();  //产生一个停止条件	
	i2c_start();
	i2c_send_byte(i2c_addr+1); 
	res++;          //进入接收模式			   
	i2c_wait_ack();

	res = i2c_read_byte(0);	   
    i2c_stop();  //产生一个停止条件

	return res;
}

/************************************************************************/
/* 功能  ：读取指定设备、指定寄存器的len个值                            */
/* 参数  ：                                                             */
/*         dev：目标设备地址                                            */
/*         reg：寄存器地址                                              */
/*         len：要读的字节数                                            */
/*         data：读出的数据存放的指针                                   */
/* 返回值：实际读到的字节数量                                           */
/************************************************************************/
u8 i2c_read_bytes(u8 dev, u8 reg, u8 len, u8 *data)
{
    u8 count = 0;
	u8 temp;

	i2c_start();
	i2c_send_byte(dev);	   //发送写命令
	i2c_wait_ack();
	i2c_send_byte(reg);   //发送地址
    i2c_wait_ack();	  
	i2c_start();
	i2c_send_byte(dev+1);  //进入接收模式	
	i2c_wait_ack();
	
    for(count=0;count<len;count++){
		 
		 if(count!=(len-1))
		 	temp = i2c_read_byte(1);  //带ACK的读数据
		 else  
			temp = i2c_read_byte(0);  //最后一个字节NACK

		data[count] = temp;
	}

    i2c_stop();  //产生一个停止条件
    
	return count;
}

/************************************************************************/
/* 功能  ：写len个值到指定设备、指定寄存器的                            */
/* 参数  ：                                                             */
/*         dev：目标设备地址                                            */
/*         reg：寄存器地址                                              */
/*         len：要写的字节数                                            */
/*         data：将要写入的数据存放的指针                               */
/* 返回值：返回是否成功                                                 */
/************************************************************************/
u8 i2c_write_bytes(u8 dev, u8 reg, u8 len, u8* data)
{  
 	u8 count = 0;

	i2c_start();
	i2c_send_byte(dev);	   //发送写命令
	i2c_wait_ack();
	i2c_send_byte(reg);   //发送地址
    i2c_wait_ack();	

	for(count=0;count<len;count++)
	{
		i2c_send_byte(data[count]); 
		i2c_wait_ack(); 
	 }

	i2c_stop();//产生一个停止条件

    return 1; //status == 0;
}

/**************************实现函数********************************************
*函数原型:		u8 IICreadByte(u8 dev, u8 reg, u8 *data)
*功　　能:	    读取指定设备 指定寄存器的一个值
输入	dev  目标设备地址
		reg	   寄存器地址
		*data  读出的数据将要存放的地址
返回   1
*******************************************************************************/ 
u8 IICreadByte(u8 dev, u8 reg, u8 *data)
{
	*data=i2c_read_onebyte(dev, reg);

    return 1;
}

/**************************实现函数********************************************
*函数原型:		unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*功　　能:	    写入指定设备 指定寄存器一个字节
输入	dev  目标设备地址
		reg	   寄存器地址
		data  将要写入的字节
返回   1
*******************************************************************************/ 
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
{
    return i2c_write_bytes(dev, reg, 1, &data);
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入	dev  目标设备地址
		reg	   寄存器地址
		bitStart  目标字节的起始位
		length   位长度
		data    存放改变目标字节位的值
返回   成功 为1 
 		失败为0
*******************************************************************************/ 
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

    u8 b;
    if (IICreadByte(dev, reg, &b) != 0) {
        u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入	dev  目标设备地址
		reg	   寄存器地址
		bitNum  要修改目标字节的bitNum位
		data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1 
 		   失败为0
*******************************************************************************/ 
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    
    return IICwriteByte(dev, reg, b);
}


