

/************************************************************************/
//功能：
//1.初始化定时器3和定时器4
/*2.定时器3-->串口打印各种参数
//3.定时器4-->姿态解算以及PID输出，属于关键中断，将定时器4的主优先级以及从优先级设为最高很有必要
*/
#include "timer.h"
#include "config_com.h"

#define Debug  //调试与否的条件编译


int LedCounter;//LED闪烁计数值
float Compass_HMC[3];

//控制入口
void TIM4_IRQHandler(void)		//1ms中断一次,用于程序读取6050等
{
    if( TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET ) 
    {     
          Controler(); //控制函数
                  
//           HMC58X3_mgetValues(&Compass_HMC[0]);
//          
          LedCounter++;//led闪烁计数值
          if(battery_ad>BATTERY_AD_MIN)//当电池电压在设定值之上时，正常模式
          {
              if(LedCounter==49){ led1_off;led2_off;}   //遥控端使能后，闪灯提示        
              else if(LedCounter==50){LedCounter=0;led1_on;led2_on;}
          }
          else //电池电压低时，闪灯提示
          {
              if(LedCounter==10){ led1_off;led2_off;led3_off;led4_off;}   //遥控端使能后，闪灯提示        
              else if(LedCounter==20){LedCounter=0;led1_on;led2_on;led3_on;led4_on;}
          }
          if(LedCounter>=51)LedCounter=0;
    
          TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);   //清除中断标志   
    }
}



int DebugCounter;             //打印信息输出时间间隔计数值


void TIM3_IRQHandler(void)		//打印中断服务程序
{
    if( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET ) 
    {     
       
  #ifdef Debug
           DebugCounter++;
           battery_ad=get_battery_adc();//电池电压检测  
          if( DebugCounter==500){
            DebugCounter=0;
            printf(" ******************************************************************\n");
            printf(" *       ____                      _____                  +---+   *\n");
            printf(" *      / ___\\                     / __ \\                 | R |   *\n");
            printf(" *     / /                        / /_/ /                 +---+   *\n");
            printf(" *    / /   ________  ____  ___  / ____/___  ____  __   __        *\n");
            printf(" *   / /  / ___/ __ `/_  / / _ \\/ /   / __ \\/ _  \\/ /  / /        *\n");
            printf(" *  / /__/ /  / /_/ / / /_/  __/ /   / /_/ / / / / /__/ /         *\n");
            printf(" *  \\___/_/   \\__,_/ /___/\\___/_/    \\___ /_/ /_/____  /          *\n");
            printf(" *                                                  / /           *\n");
            printf(" *                                             ____/ /            *\n");
            printf(" *                                            /_____/             *\n");
            printf(" ******************************************************************\n");
            printf("\n");
            printf(" Crazepony-II报告：系统正在运行...\n"); 
            printf("\n");
            printf("\n--->机身实时姿态广播信息<---\n");
            printf("\n");
            printf(" 偏航角---> %5.2f°\n",(float)Q_ANGLE.Yaw);
            printf(" 俯仰角---> %5.2f°\n",(float)Q_ANGLE.Pitch);
            printf(" 横滚角---> %5.2f°\n",(float) Q_ANGLE.Roll);
            printf(" ==================\n");
            printf(" X轴期望角度---> %5.2f°\n",(float)EXP_ANGLE.X);
            printf(" Y轴期望角度---> %5.2f°\n",(float)EXP_ANGLE.Y);
            printf(" Z轴期望角度---> %5.2f°\n",(float)EXP_ANGLE.Z);
          
            printf(" ==================\n");
            printf(" Y轴误差角度---> %5.2f°\n",(float)DIF_ANGLE.Y);
            printf(" X轴误差角度---> %5.2f°\n",(float)DIF_ANGLE.X);
            printf("==================\n");
            printf(" X轴加速度---> %5.2fm/s2\n",(float) DMP_DATA.dmp_accx);
            printf(" Y轴加速度---> %5.2fm/s2\n",(float) DMP_DATA.dmp_accy);
            printf(" Z轴加速度---> %5.2fm/s2\n",(float) DMP_DATA.dmp_accz);
            printf(" ==================\n");
            printf(" X轴角速度---> %5.2f °/s\n",(float) DMP_DATA.dmp_gyrox);
            printf(" Y轴角速度---> %5.2f °/s\n",(float) DMP_DATA.dmp_gyroy);
            printf(" Z轴角速度---> %5.2f °/s\n",(float) DMP_DATA.dmp_gyroz);
            printf("==================\n");
            printf(" 电机M1 PWM值---> %d\n",TIM2->CCR1);
            printf(" 电机M2 PWM值---> %d\n",TIM2->CCR2);
            printf(" 电机M3 PWM值---> %d\n",TIM2->CCR3);
            printf(" 电机M4 PWM值---> %d\n",TIM2->CCR4);
            printf("==================\n");
            printf(" 电池电压---> %3.2fv\n",(float) 2*(battery_ad/4.096)*0.0033);//根据采集到的AD值，计算实际电压。硬件上是对电池进行分压后给AD采集的，所以结果要乘以2
            printf("==================\n");
//             printf(" X磁场强度---> %5.2f °/s\n",(float) Compass_HMC[0]);
//             printf(" Y磁场强度---> %5.2f °/s\n",(float) Compass_HMC[1]);
//             printf(" Z磁场强度---> %5.2f °/s\n",(float) Compass_HMC[2]);
                  
#else      
             
#endif
        }
        TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);   //清除中断标志   
    }
}

//定时器3初始化
void TIM3_Int_Init(u16 arr,u16 psc)
{                  
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能

	//定时器TIM3初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位

	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	//中断优先级NVIC设置  //定时器3作为串口打印定时器，优先级低于姿态解算
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


	TIM_Cmd(TIM3, ENABLE);  //使能TIMx					 
}

//定时器4初始化：用来中断处理PID
void TIM4_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //时钟使能

	//定时器TIM3初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位

	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //使能指定的TIM4中断,允许更新中断

	//中断优先级NVIC设置  //定时器4作为姿态解算，优先级高于串口打印
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级1级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级1级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


	TIM_Cmd(TIM4, ENABLE);  //使能TIMx					 
}

////定时器4初始化：用来中断处理PID
//void TIM4_Init(char clock,int Preiod)
//{
//    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  //打开时钟
//    
//    TIM_DeInit(TIM4);
//
//    TIM_TimeBaseStructure.TIM_Period = Preiod;
//    TIM_TimeBaseStructure.TIM_Prescaler = clock-1;//定时1ms
//    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    
//    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
//    TIM_ClearFlag(TIM4,TIM_FLAG_Update);
//
//    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
//    TIM_Cmd(TIM4,ENABLE);
//    printf("定时器4初始化完成...\n");
//    
//}	
//
//
////定时器3初始化
//void TIM3_Init(char clock,int Preiod)
//{
//    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  //打开时钟
//    
//    TIM_DeInit(TIM3);
//
//    TIM_TimeBaseStructure.TIM_Period = Preiod;
//    TIM_TimeBaseStructure.TIM_Prescaler = clock-1;//定时1ms
//    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    
//    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
//    TIM_ClearFlag(TIM3,TIM_FLAG_Update);
//
//    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
//    TIM_Cmd(TIM3,ENABLE);
//    printf("定时器3初始化完成...\n");
//}		


void TimerNVIC_Configuration()
{
    //NVIC_InitTypeDef NVIC_InitStructure;
    
    /* NVIC_PriorityGroup 2 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    ////TIM3
    //NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//定时器3作为串口打印定时器，优先级低于姿态解算
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitStructure);
    ////TIM4
    //NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//定时器4作为姿态解算，优先级高于串口打印
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitStructure);

} 

