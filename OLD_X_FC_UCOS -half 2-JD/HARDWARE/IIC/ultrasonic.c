#include "include.h"
#include "ultrasonic.h"
#include "usart.h"
#include "filter.h"
#include "IMU.h"
#include "rc.h"
#include "ms5611.h"
#include "myiic_sonar.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "circle.h"
float T_sonar;

void SONAR_GPIO_Config(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;
	//设置LED使用到得管脚
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

	/*开启GPIOB的外设时钟*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOF时钟
	/*选择要控制的GPIOC引脚*/															   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6	;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//上拉

	/*调用库函数，初始化GPIOB*/
	GPIO_Init(GPIOE, &GPIO_InitStructure);		 

	/*选择要控制的GPIOC引脚*/															   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//上拉
	/*调用库函数，初始化GPIOB*/
	GPIO_Init(GPIOE, &GPIO_InitStructure);		 
	GPIO_ResetBits(GPIOE,GPIO_Pin_7);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource6);//PE2 连接到中断线2
	/* 配置EXTI_Line0 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line6;//LINE0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //上升沿触发 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE0
	EXTI_Init(&EXTI_InitStructure);//配置

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//外部中断4
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);//配置
}
void GPIO_SET_SONAR(u8 num)
{

GPIO_SetBits(GPIOE,GPIO_Pin_7);	
}

void GPIO_RESET_SONAR(u8 num)
{
GPIO_ResetBits(GPIOE,GPIO_Pin_7);
}
u8 GPIO_READ_SONAR(u8 num)
{u8 temp=0;

temp=GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6);
return temp;
}

void Ultrasonic_Init()
{ 
	#if defined(SONAR_USE_UART)   
	Uart5_Init(9600);			//串口5初始化，函数参数为波特率
	#endif
	#if defined(SONAR_USE_TIG)    
	SONAR_GPIO_Config();	ultra_ok = 1;
	#endif
	#if defined(SONAR_USE_SCL)    
	IIC_Init_Sonar();
	ultra_ok=AT24CXX_Check();	
	#endif
}
#include "pwm_in.h"
s8 ultra_start_f,ultra_start_f_rx;;
u8 ultra_time;
u8 ultra_ok = 0;
u8 en_sonar=1;
void Ultra_Duty()
{
	u8 temp[3];

	ultra_time++;
	ultra_time = ultra_time%2;
	if(Rc_Pwm_Inr_mine[RC_THR]>200+1000){
 #if defined(SONAR_USE_UART)
	#if defined(USE_KS103)
		temp[0] = 0xe8;
		temp[1] = 0x02;
		temp[2] = 0xbc;
		Uart5_Send(temp ,0xe8);Delay_us(20);
	  Uart5_Send(temp ,0x02);Delay_us(20);
	  Uart5_Send(temp ,0xbc);
	#elif defined(USE_US100)
		temp[0] = 0x55;
	  Uart5_Send(temp ,0x55);
	#endif
 #endif
	#if defined(SONAR_USE_TRI)
 if(en_sonar){		
	GPIO_SET_SONAR(0);
	Delay_us(20);
	GPIO_RESET_SONAR(0);
 }
	#endif
		
	ultra_start_f = 1;
	//if(ultra_start_f_rx==0)
  ultra_start_f_rx=1;
}
}

void Ultra_Duty_SCL(void)
{ float temp1,temp,temp2;
	static u16 ultra_distance_old;
  u16 range;
//--
	KS103_WriteOneByte(0XE8,0X02,0XB0);//5m
	//KS103_WriteOneByte(0XE8,0X02,0XB8);//11m
//---
	//delay_ms(100); 
	range = KS103_ReadOneByte(0xe8, 0x02);
	range <<= 8;
	range += KS103_ReadOneByte(0xe8, 0x03);
	
	temp1=limit_mine(Roll,45);
	temp2=limit_mine(Pitch,45);
	temp=(float)range*cos(temp1*0.017)*cos(temp2*0.017);
	temp=((temp)<(0)?(0):((temp)>(2000)?(2000):(temp)));
	ultra_distance=Moving_Median(1,5,temp);
	T_sonar=Get_Cycle_T(GET_T_SONAR_SAMPLE);
	ultra_delta = (ultra_distance - ultra_distance_old)/LIMIT(T_sonar,0.0001,1);

	ultra_distance_old = ultra_distance;
	ultra_start_f = 1;
	ultra_start_f_rx=1;
	//delay_ms(10);
}
// float t1r=1;
/* kalman filter states */
double x_pred = 0.0f; // m   0
double v_pred = 0.0f; //       1
double x_post = 0.0f; // m    2
double v_post = 0.0f; // m/s  3
float sonar_raw = 0.0f;  // m
float scale_kal_sonar_v=0.2;
float sonar_filter(float hight,float dt_sonar)
{float x_new;
 static float reg;
	float LPF_1=1;//0.75; 
	float MAX_SPEED=0.5;
	/* no data for long time */
	if (dt_sonar > 0.25f) // more than 2 values lost
	{
		v_pred = 0;
	}

	x_pred = x_post + dt_sonar * v_pred;
	v_pred = v_post;
   v_pred=limit_mine	(v_pred,MAX_SPEED);
	 if(fabs(v_pred) < 0.01)           \
    v_pred = 0;   
	 
	 v_pred=reg*(1-LPF_1)+v_pred*(LPF_1);
	 reg=v_pred;
	 x_new = hight;
	sonar_raw = x_new;
	x_post = x_pred +  0.91* (x_new - x_pred);//0.8461f
	v_post = v_pred +  6.2034f* (x_new - x_pred)*scale_kal_sonar_v;
  v_post=limit_mine(v_post,MAX_SPEED);
	  if(fabs(v_post) < 0.01)           \
    v_post = 0;   
	return x_pred;//m/s
}

// float t1r=1;
/* kalman filter states */
double x_pred_bmp = 0.0f; // m   0
double v_pred_bmp = 0.0f; //       1
double x_post_bmp = 0.0f; // m    2
double v_post_bmp = 0.0f; // m/s  3
float sonar_filter_bmp(float hight,float dt_sonar)
{float x_new;
 static float reg;
	float LPF_1=0.75; 
	float MAX_SPEED=0.5;
	/* no data for long time */
	if (dt_sonar > 0.25f) // more than 2 values lost
	{
		v_pred = 0;
	}

	x_pred_bmp = x_post_bmp + dt_sonar * v_pred_bmp;
	v_pred_bmp = v_post_bmp;
   v_pred_bmp=limit_mine	(v_pred_bmp,MAX_SPEED);
	 if(fabs(v_pred_bmp) < 0.01)           \
    v_pred_bmp = 0;   
	 
	 v_pred_bmp=reg*(1-LPF_1)+v_pred_bmp*(LPF_1);
	 reg=v_pred_bmp;
	 x_new = hight;
	x_post_bmp = x_pred_bmp +  0.91* (x_new - x_pred_bmp);//0.8461f
	v_post_bmp = v_pred_bmp +  6.2034f* (x_new - x_pred_bmp)*scale_kal_sonar_v;
  v_post_bmp=limit_mine(v_post_bmp,MAX_SPEED);
	  if(fabs(v_post_bmp) < 0.01)           \
    v_post_bmp = 0;   
	return x_pred_bmp;//m/s
}

u8 state_dj[5]={0,0,0,0,0};
u8 state_dj_rx[5]={0,0,0,0,0};
u8 IO_STATE[5]={0,0,0,0,0};
u8 IO_STATER[5]={0,0,0,0,0};
u32 TEMP_SONAR=340*3000/200;//-------------------------------------超声波时间参数
//外部中断4服务程序
u32 cnt_sample1r,now_djr[4],lastUpdate_djr[4];

void EXTI9_5_IRQHandler(void)
{     OSIntEnter();  
	float temp,temp1,temp2;
		static int ultra_distance_old;
	
	if(EXTI_GetITStatus(EXTI_Line6) != RESET)
				{
			IO_STATE[3]=GPIO_READ_SONAR(3);
			 switch(state_dj_rx[3])
			 {
				 case 0:if(IO_STATE[3]==1)
				 { 
					 lastUpdate_djr[3] =  GetSysTime_us();
					 state_dj_rx[3]=1; 
				 }
				 break;
				 case 1:		 
				 now_djr[3] = GetSysTime_us();  //读取时间
				 if( now_djr[3] <lastUpdate_djr[3]){cnt_sample1r =  ((float)( now_djr[3]  + (0xffff- lastUpdate_djr[3])) );}
				 else	{ cnt_sample1r =  ((float)( now_djr[3]  - lastUpdate_djr[3])); }
				// lastUpdate_dj[3]= now_dj[3];
					 if(IO_STATE[3]==1)
						 state_dj_rx[3]=0; 
					 else if(IO_STATE[3]==0)
				 {  
				temp1=limit_mine(Roll,45);
				temp2=limit_mine(Pitch,45);
				temp=(float)cnt_sample1r*1000/(TEMP_SONAR)*cos(temp1*0.017)*cos(temp2*0.017);
				temp=((temp)<(0)?(0):((temp)>(4000)?(4000):(temp)));
				 ultra_distance=temp;//Moving_Median(1,5,temp);
				 ultra_ok = 1;ultra_start_f = 0;
					state_dj_rx[3]=0;
		
				 T_sonar=Get_Cycle_T(GET_T_SONAR_SAMPLE);
				//ultra_delta = (ultra_distance - ultra_distance_old)/LIMIT(T_sonar,0.000000001,1);
				//sonar_filter((float) temp/1000,T_sonar);
				//ultra_distance_old = ultra_distance;
				 }
				
				 break;
				 default:
					 state_dj_rx[3]=0;
					 break;
			 }
				
					EXTI_ClearITPendingBit(EXTI_Line6);
	}   
				   OSIntExit();
}


int ultra_distance,ultra_distance_r;
float ultra_delta;
float OFF_H_M100;
void Ultra_Get(u8 com_data)
{
	static u8 ultra_tmp;
	int temp1,temp2,temp;
	float dt;
	static int ultra_distance_old;
	if( ultra_start_f_rx == 1 )
	{
		ultra_tmp = com_data;
		ultra_start_f_rx = 2;
	}
	else if( ultra_start_f_rx == 2 )
	{
		temp = (ultra_tmp<<8) + com_data;
		ultra_start_f_rx = 0; 
		dt=Get_Cycle_T(GET_T_SONAR_SAMPLE);
		temp1=limit_mine(Roll,45);
		temp2=limit_mine(Pitch,45);
		temp=temp;//*cos(temp1*0.017)*cos(temp2*0.017);
	  temp=((temp)<(0)?(0):((temp)>(4500)?(4500):(temp)));
   	//sonar_filter((float) temp/1000.,dt);
	  //ultra_distance=((x_pred*1000)<(0)?(0):((x_pred*1000)>(2500)?(2500):(x_pred*1000)));
			//if(height_ctrl_mode==1)
			//ultra_distance=baroAlt*10;
			//else
		//#if USE_M100
	//	 if(mode.rst_h_m100==0)
	//		 OFF_H_M100=m100.H;
	//	 ultra_distance=LIMIT(1000*(m100.H-OFF_H_M100),0,12000);
	//	#else
		 ultra_distance=temp;//LIMIT(Moving_Median(1,8,temp),0,4000);//temp;//Moving_Median(7,5,LIMIT(temp,0,4500));
	//	#endif
		ultra_ok = 1;
	}
	 
	
	ultra_delta = ultra_distance - ultra_distance_old;
	
	ultra_distance_old = ultra_distance;
	
}


