#include "sys.h"
#include "usart.h"	  
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/8/18
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*使用microLib的方法*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 rx_buffer[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记

//uart reicer flag
#define b_uart_head  0x80
#define b_rx_over    0x40


u8 U1TxBuffer[256];
u8 U1TxPackage[TX_BUFFER_SIZE];
u8 U1TxCounter=0;
u8 U1RxCounter=0;
u8 U1count=0; 
char TxPackFlag;//发送预定格式数据包标志位

/************************************************************************/
/* 功能  ：串口1初始化                                                  */
/* 参数  ：                                                             */
/*         bound：波特率                                                */
/* 返回值：无                                                           */
/* IO口  ：                                                             */
/*         USART1_TX：PA.9                                              */
/*         USART1_RX：PA.10                                             */
/************************************************************************/
void uart1_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* 使能GPIOA端口时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/* 使能复用的外设时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	/* 使能USART1时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* USART1_TX   PA.9 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  //PA.9 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	 //复用推挽输出 
    GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA9
   
    /* USART1_RX	  PA.10 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  //PA.10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA10

    /* Usart1 NVIC 配置 */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;  //抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);  //根据指定的参数初始化VIC寄存器
  
    /* USART 初始化设置 */
	USART_InitStructure.USART_BaudRate = bound;  //一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;  //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //收发模式
	USART_Init(USART1, &USART_InitStructure);  //初始化串口

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  //开启中断
    USART_Cmd(USART1, ENABLE);                      //使能串口 
}

void USART1_IRQHandler(void)                	//串口1中断服务程序
	{
	u8 Res;
#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
			{
			if(USART_RX_STA&0x4000)//接收到了0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
				}
			else //还没收到0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					rx_buffer[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
					}		 
				}
			}   		 
     } 
#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntExit();  											 
#endif
} 
#endif	


/**************************实现函数********************************************
*函数原型:		void UART1_Put_Char(unsigned char DataToSend)
*功　　能:		RS232发送一个字节
输入参数：
		unsigned char DataToSend   要发送的字节数据
输出参数：没有	
*******************************************************************************/
void UART1_Put_Char(unsigned char DataToSend)
{
	U1TxBuffer[U1count++] = DataToSend;  
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  
}
/**************************实现函数********************************************
*函数原型:		u8 UART1_Get_Char(void)
*功　　能:		RS232接收一个字节  一直等待，直到UART1接收到一个字节的数据。
输入参数：		 没有
输出参数：     UART1接收到的数据	
*******************************************************************************/
u8 UART1_Get_Char(void)
{
	while (!(USART1->SR & USART_FLAG_RXNE));
	return(USART_ReceiveData(USART1));
}


/**************************实现函数********************************************
*函数原型:		void UART2_Put_String(unsigned char *Str)
*功　　能:		RS232发送字符串
输入参数：
		unsigned char *Str   要发送的字符串
输出参数：没有	
*******************************************************************************/
void UART1_Put_String(unsigned char *Str)
{
	//判断Str指向的数据是否有效.
	while(*Str){
	//是否是回车字符 如果是,则发送相应的回车 0x0d 0x0a
	if(*Str=='\r')UART1_Put_Char(0x0d);
		else if(*Str=='\n')UART1_Put_Char(0x0a);
			else UART1_Put_Char(*Str);
	//等待发送完成.
  	//while (!(USART1->SR & USART_FLAG_TXE));
	//指针++ 指向下一个字节.
	Str++;
	}
/*
	//判断Str指向的数据是否有效.
	while(*Str){
	//是否是回车字符 如果是,则发送相应的回车 0x0d 0x0a
	if(*Str=='')USART_SendData(USART1, 0x0d);
		else if(*Str=='\n')USART_SendData(USART1, 0x0a);
			else USART_SendData(USART1, *Str);
	//等待发送完成.
  	while (!(USART1->SR & USART_FLAG_TXE));
	//指针++ 指向下一个字节.
	Str++;
	}		 */
}

/**************************实现函数********************************************
*函数原型:		void UART2_Putc_Hex(uint8_t b)
*功　　能:		RS232以十六进制ASCII码的方式发送一个字节数据
				先将目标字节数据高4位转成ASCCII ，发送，再将低4位转成ASCII发送
				如:0xF2 将发送 " F2 "
输入参数：
		uint8_t b   要发送的字节
输出参数：没有	
*******************************************************************************/
void UART1_Putc_Hex(uint8_t b)
{
      /* 判断目标字节的高4位是否小于10 */
    if((b >> 4) < 0x0a)
        UART1_Put_Char((b >> 4) + '0'); //小于10  ,则相应发送0-9的ASCII
    else
        UART1_Put_Char((b >> 4) - 0x0a + 'A'); //大于等于10 则相应发送 A-F

    /* 判断目标字节的低4位 是否小于10*/
    if((b & 0x0f) < 0x0a)
        UART1_Put_Char((b & 0x0f) + '0');//小于10  ,则相应发送0-9的ASCII
    else
        UART1_Put_Char((b & 0x0f) - 0x0a + 'A');//大于等于10 则相应发送 A-F
   UART1_Put_Char(' '); //发送一个空格,以区分开两个字节
}

/**************************实现函数********************************************
*函数原型:		void UART2_Putw_Hex(uint16_t w)
*功　　能:		RS232以十六进制ASCII码的方式发送一个字的数据.就是发送一个int
				如:0x3456 将发送 " 3456 "
输入参数：
		uint16_t w   要发送的字
输出参数：没有	
*******************************************************************************/
void UART1_Putw_Hex(uint16_t w)
{
	//发送高8位数据,当成一个字节发送
    UART1_Putc_Hex((uint8_t) (w >> 8));
	//发送低8位数据,当成一个字节发送
    UART1_Putc_Hex((uint8_t) (w & 0xff));
}

/**************************实现函数********************************************
*函数原型:		void UART2_Putdw_Hex(uint32_t dw)
*功　　能:		RS232以十六进制ASCII码的方式发送32位的数据.
				如:0xF0123456 将发送 " F0123456 "
输入参数：
		uint32_t dw   要发送的32位数据值
输出参数：没有	
*******************************************************************************/
void UART1_Putdw_Hex(uint32_t dw)
{
    UART1_Putw_Hex((uint16_t) (dw >> 16));
    UART1_Putw_Hex((uint16_t) (dw & 0xffff));
}

/**************************实现函数********************************************
*函数原型:		void UART2_Putw_Dec(uint16_t w)
*功　　能:		RS232以十进制ASCII码的方式发送16位的数据.
				如:0x123 将发送它的十进制数据 " 291 "
输入参数：
		uint16_t w   要发送的16位数据值
输出参数：没有	
*******************************************************************************/
void UART1_Putw_Dec(uint32_t w)
{
    uint32_t num = 100000;
    uint8_t started = 0;

    while(num > 0)
    {
        uint8_t b = w / num;
        if(b > 0 || started || num == 1)
        {
            UART1_Put_Char('0' + b);
            started = 1;
        }
        w -= b * num;

        num /= 10;
    }
}


