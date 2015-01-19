/*    
���ܣ�
1.��ʼ�����IICЭ��
2.���IICЭ������ţ�Ҳ��STM32Ӳ��IIC�����š�ֻ����û���������
------------------------------------
*/
//STM32ģ��IICЭ�飬STM32��Ӳ��IIC��ЩBUG
//ϸ���п��ٸ�
//����޸�:2014-03-11


#include "I2C.h"
#include "delay.h"
#include "Led.h"
#include "UART1.h"
#include "stdio.h"

/************************************************************************/
/*                                                                      */
/************************************************************************/
/************************************************************************/
/* ����  ����ʼ��I2C                                                    */
/* ����  ����                                                           */
/* ����ֵ����                                                           */
/* IO��  ��                                                             */
/*         SCL��PB.06                                                   */
/*         SDA��PB.07                                                   */
/************************************************************************/
void i2c_init(void)
{			
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ʹ��GPIOB�ڵ�ʱ�� */
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);			     
 	
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  //PB.06  PB.07	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  
	/* Ӧ�����õ�GPIOB */
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* PB.06 PB.07����ߵ�ƽ */
	GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);

	printf("***IIC���߳�ʼ�����***\n");
}

/************************************************************************/
/* ����  ������IIC��ʼ�ź�                                              */
/* ����  ����                                                           */
/* ����ֵ����                                                           */
/************************************************************************/
void i2c_start(void)
{
	SDA_OUT();  //sda����� 
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;  //START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;  //ǯסI2C���ߣ�׼�����ͻ�������� 
}

/************************************************************************/
/* ����  ������IICֹͣ�ź�                                              */
/* ����  ����                                                           */
/* ����ֵ����                                                           */
/************************************************************************/
void i2c_stop(void)
{
	SDA_OUT();  //sda�����
	IIC_SCL=0;
	IIC_SDA=0;  //STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;  //����I2C���߽����ź�
	delay_us(4);							   	
}

/************************************************************************/
/* ����  ���ȴ�Ӧ���źŵ���                                             */
/* ����  ����                                                           */
/* ����ֵ��                                                             */
/*         1������Ӧ��ʧ��                                              */
/*         0������Ӧ��ɹ�                                              */
/************************************************************************/
u8 i2c_wait_ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
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

	IIC_SCL=0;  //ʱ�����0 	   
	return 0;  
} 

/************************************************************************/
/* ����  ������ACKӦ��                                                  */
/* ����  ����                                                           */
/* ����ֵ����                                                           */
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
/* ����  ������NACKӦ��                                                 */
/* ����  ����                                                           */
/* ����ֵ����                                                           */
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
/* ����  ��IIC����һ���ֽ�                                              */
/* ����  ��                                                             */
/*         txd��Ҫ���͵�����                                            */
/* ����ֵ����                                                           */
/************************************************************************/
void i2c_send_byte(u8 txd)
{                        
    u8 t;  

	SDA_OUT(); 	    
    IIC_SCL=0;  //����ʱ�ӿ�ʼ���ݴ���

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
/* ����  ��IIC��ȡһ���ֽ�                                              */
/* ����  ��                                                             */
/*         ack������ACK�ı�־(ack=1ʱ������ACK��ack=0������nACK)        */
/* ����ֵ�����յ�������                                                 */
/************************************************************************/
u8 i2c_read_byte(unsigned char ack)
{
	unsigned char i;
	unsigned char receive=0;

	SDA_IN()  ;//SDA����Ϊ����

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
        i2c_ack();  //����ACK 
    else
        i2c_nack(); //����nACK 

    return receive;
}

/************************************************************************/
/* ����  ����ȡָ���豸��ָ���Ĵ�����һ��ֵ                             */
/* ����  ��                                                             */
/*         i2c_addr��Ŀ���豸��ַ                                       */
/*         addr���Ĵ�����ַ                                             */
/* ����ֵ����ȡ��������                                                 */
/************************************************************************/
unsigned char i2c_read_onebyte(unsigned char i2c_addr,unsigned char addr)
{
	unsigned char res=0;
	
	i2c_start();	

	i2c_send_byte(i2c_addr);  //����д����
	res++;
	i2c_wait_ack();

	i2c_send_byte(addr); 
	res++;  //���͵�ַ
	i2c_wait_ack();	  

	//IIC_Stop();  //����һ��ֹͣ����	
	i2c_start();
	i2c_send_byte(i2c_addr+1); 
	res++;          //�������ģʽ			   
	i2c_wait_ack();

	res = i2c_read_byte(0);	   
    i2c_stop();  //����һ��ֹͣ����

	return res;
}

/************************************************************************/
/* ����  ����ȡָ���豸��ָ���Ĵ�����len��ֵ                            */
/* ����  ��                                                             */
/*         dev��Ŀ���豸��ַ                                            */
/*         reg���Ĵ�����ַ                                              */
/*         len��Ҫ�����ֽ���                                            */
/*         data�����������ݴ�ŵ�ָ��                                   */
/* ����ֵ��ʵ�ʶ������ֽ�����                                           */
/************************************************************************/
u8 i2c_read_bytes(u8 dev, u8 reg, u8 len, u8 *data)
{
    u8 count = 0;
	u8 temp;

	i2c_start();
	i2c_send_byte(dev);	   //����д����
	i2c_wait_ack();
	i2c_send_byte(reg);   //���͵�ַ
    i2c_wait_ack();	  
	i2c_start();
	i2c_send_byte(dev+1);  //�������ģʽ	
	i2c_wait_ack();
	
    for(count=0;count<len;count++){
		 
		 if(count!=(len-1))
		 	temp = i2c_read_byte(1);  //��ACK�Ķ�����
		 else  
			temp = i2c_read_byte(0);  //���һ���ֽ�NACK

		data[count] = temp;
	}

    i2c_stop();  //����һ��ֹͣ����
    
	return count;
}

/************************************************************************/
/* ����  ��дlen��ֵ��ָ���豸��ָ���Ĵ�����                            */
/* ����  ��                                                             */
/*         dev��Ŀ���豸��ַ                                            */
/*         reg���Ĵ�����ַ                                              */
/*         len��Ҫд���ֽ���                                            */
/*         data����Ҫд������ݴ�ŵ�ָ��                               */
/* ����ֵ�������Ƿ�ɹ�                                                 */
/************************************************************************/
u8 i2c_write_bytes(u8 dev, u8 reg, u8 len, u8* data)
{  
 	u8 count = 0;

	i2c_start();
	i2c_send_byte(dev);	   //����д����
	i2c_wait_ack();
	i2c_send_byte(reg);   //���͵�ַ
    i2c_wait_ack();	

	for(count=0;count<len;count++)
	{
		i2c_send_byte(data[count]); 
		i2c_wait_ack(); 
	 }

	i2c_stop();//����һ��ֹͣ����

    return 1; //status == 0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICreadByte(u8 dev, u8 reg, u8 *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		*data  ���������ݽ�Ҫ��ŵĵ�ַ
����   1
*******************************************************************************/ 
u8 IICreadByte(u8 dev, u8 reg, u8 *data)
{
	*data=i2c_read_onebyte(dev, reg);

    return 1;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*��������:	    д��ָ���豸 ָ���Ĵ���һ���ֽ�
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		data  ��Ҫд����ֽ�
����   1
*******************************************************************************/ 
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
{
    return i2c_write_bytes(dev, reg, 1, &data);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitStart  Ŀ���ֽڵ���ʼλ
		length   λ����
		data    ��Ÿı�Ŀ���ֽ�λ��ֵ
����   �ɹ� Ϊ1 
 		ʧ��Ϊ0
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
		data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
����   �ɹ� Ϊ1 
 		   ʧ��Ϊ0
*******************************************************************************/ 
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    
    return IICwriteByte(dev, reg, b);
}


