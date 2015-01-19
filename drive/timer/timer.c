

/************************************************************************/
//���ܣ�
//1.��ʼ����ʱ��3�Ͷ�ʱ��4
/*2.��ʱ��3-->���ڴ�ӡ���ֲ���
//3.��ʱ��4-->��̬�����Լ�PID��������ڹؼ��жϣ�����ʱ��4�������ȼ��Լ������ȼ���Ϊ��ߺ��б�Ҫ
*/
#include "timer.h"
#include "config_com.h"

#define Debug  //����������������


int LedCounter;//LED��˸����ֵ
float Compass_HMC[3];

//�������
void TIM4_IRQHandler(void)		//1ms�ж�һ��,���ڳ����ȡ6050��
{
    if( TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET ) 
    {     
          Controler(); //���ƺ���
                  
//           HMC58X3_mgetValues(&Compass_HMC[0]);
//          
          LedCounter++;//led��˸����ֵ
          if(battery_ad>BATTERY_AD_MIN)//����ص�ѹ���趨ֵ֮��ʱ������ģʽ
          {
              if(LedCounter==49){ led1_off;led2_off;}   //ң�ض�ʹ�ܺ�������ʾ        
              else if(LedCounter==50){LedCounter=0;led1_on;led2_on;}
          }
          else //��ص�ѹ��ʱ��������ʾ
          {
              if(LedCounter==10){ led1_off;led2_off;led3_off;led4_off;}   //ң�ض�ʹ�ܺ�������ʾ        
              else if(LedCounter==20){LedCounter=0;led1_on;led2_on;led3_on;led4_on;}
          }
          if(LedCounter>=51)LedCounter=0;
    
          TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);   //����жϱ�־   
    }
}



int DebugCounter;             //��ӡ��Ϣ���ʱ��������ֵ


void TIM3_IRQHandler(void)		//��ӡ�жϷ������
{
    if( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET ) 
    {     
       
  #ifdef Debug
           DebugCounter++;
           battery_ad=get_battery_adc();//��ص�ѹ���  
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
            printf(" Crazepony-II���棺ϵͳ��������...\n"); 
            printf("\n");
            printf("\n--->����ʵʱ��̬�㲥��Ϣ<---\n");
            printf("\n");
            printf(" ƫ����---> %5.2f��\n",(float)Q_ANGLE.Yaw);
            printf(" ������---> %5.2f��\n",(float)Q_ANGLE.Pitch);
            printf(" �����---> %5.2f��\n",(float) Q_ANGLE.Roll);
            printf(" ==================\n");
            printf(" X�������Ƕ�---> %5.2f��\n",(float)EXP_ANGLE.X);
            printf(" Y�������Ƕ�---> %5.2f��\n",(float)EXP_ANGLE.Y);
            printf(" Z�������Ƕ�---> %5.2f��\n",(float)EXP_ANGLE.Z);
          
            printf(" ==================\n");
            printf(" Y�����Ƕ�---> %5.2f��\n",(float)DIF_ANGLE.Y);
            printf(" X�����Ƕ�---> %5.2f��\n",(float)DIF_ANGLE.X);
            printf("==================\n");
            printf(" X����ٶ�---> %5.2fm/s2\n",(float) DMP_DATA.dmp_accx);
            printf(" Y����ٶ�---> %5.2fm/s2\n",(float) DMP_DATA.dmp_accy);
            printf(" Z����ٶ�---> %5.2fm/s2\n",(float) DMP_DATA.dmp_accz);
            printf(" ==================\n");
            printf(" X����ٶ�---> %5.2f ��/s\n",(float) DMP_DATA.dmp_gyrox);
            printf(" Y����ٶ�---> %5.2f ��/s\n",(float) DMP_DATA.dmp_gyroy);
            printf(" Z����ٶ�---> %5.2f ��/s\n",(float) DMP_DATA.dmp_gyroz);
            printf("==================\n");
            printf(" ���M1 PWMֵ---> %d\n",TIM2->CCR1);
            printf(" ���M2 PWMֵ---> %d\n",TIM2->CCR2);
            printf(" ���M3 PWMֵ---> %d\n",TIM2->CCR3);
            printf(" ���M4 PWMֵ---> %d\n",TIM2->CCR4);
            printf("==================\n");
            printf(" ��ص�ѹ---> %3.2fv\n",(float) 2*(battery_ad/4.096)*0.0033);//���ݲɼ�����ADֵ������ʵ�ʵ�ѹ��Ӳ�����ǶԵ�ؽ��з�ѹ���AD�ɼ��ģ����Խ��Ҫ����2
            printf("==================\n");
//             printf(" X�ų�ǿ��---> %5.2f ��/s\n",(float) Compass_HMC[0]);
//             printf(" Y�ų�ǿ��---> %5.2f ��/s\n",(float) Compass_HMC[1]);
//             printf(" Z�ų�ǿ��---> %5.2f ��/s\n",(float) Compass_HMC[2]);
                  
#else      
             
#endif
        }
        TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);   //����жϱ�־   
    }
}

//��ʱ��3��ʼ��
void TIM3_Int_Init(u16 arr,u16 psc)
{                  
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��

	//��ʱ��TIM3��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

	//�ж����ȼ�NVIC����  //��ʱ��3��Ϊ���ڴ�ӡ��ʱ�������ȼ�������̬����
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���


	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx					 
}

//��ʱ��4��ʼ���������жϴ���PID
void TIM4_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //ʱ��ʹ��

	//��ʱ��TIM3��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM4�ж�,��������ж�

	//�ж����ȼ�NVIC����  //��ʱ��4��Ϊ��̬���㣬���ȼ����ڴ��ڴ�ӡ
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���


	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIMx					 
}

////��ʱ��4��ʼ���������жϴ���PID
//void TIM4_Init(char clock,int Preiod)
//{
//    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  //��ʱ��
//    
//    TIM_DeInit(TIM4);
//
//    TIM_TimeBaseStructure.TIM_Period = Preiod;
//    TIM_TimeBaseStructure.TIM_Prescaler = clock-1;//��ʱ1ms
//    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    
//    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
//    TIM_ClearFlag(TIM4,TIM_FLAG_Update);
//
//    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
//    TIM_Cmd(TIM4,ENABLE);
//    printf("��ʱ��4��ʼ�����...\n");
//    
//}	
//
//
////��ʱ��3��ʼ��
//void TIM3_Init(char clock,int Preiod)
//{
//    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  //��ʱ��
//    
//    TIM_DeInit(TIM3);
//
//    TIM_TimeBaseStructure.TIM_Period = Preiod;
//    TIM_TimeBaseStructure.TIM_Prescaler = clock-1;//��ʱ1ms
//    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    
//    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
//    TIM_ClearFlag(TIM3,TIM_FLAG_Update);
//
//    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
//    TIM_Cmd(TIM3,ENABLE);
//    printf("��ʱ��3��ʼ�����...\n");
//}		


void TimerNVIC_Configuration()
{
    //NVIC_InitTypeDef NVIC_InitStructure;
    
    /* NVIC_PriorityGroup 2 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    ////TIM3
    //NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//��ʱ��3��Ϊ���ڴ�ӡ��ʱ�������ȼ�������̬����
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitStructure);
    ////TIM4
    //NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//��ʱ��4��Ϊ��̬���㣬���ȼ����ڴ��ڴ�ӡ
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitStructure);

} 

