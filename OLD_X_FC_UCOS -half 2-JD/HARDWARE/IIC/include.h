#ifndef _INCLUDE_H_
#define _INCLUDE_H_

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdarg.h>



#include <stm32f4xx.h>	  
#include "math.h"
#include "time.h"
#include "mpu6050.h"
#include "parameter.h"
#include "delay.h" 
#include "malloc.h"   
#include "usart_fc.h"   
#include "spi_nrf.h"							//nrf24l01??spi.h??u8 Spi_RW(u8 dat)??
#include "rc_mine.h"							//nrf24l01??spi.h??u8 Spi_RW(u8 dat)??
#include "nrf.h"							//nrf24l01??spi.h??u8 Spi_RW(u8 dat)??
#include "ultrasonic.h"
#include "rc.h"
#include "att.h"
#include "height_ctrl.h"
#include "flash.h"
#include "dma.h"

/***************中断优先级******************/
#define NVIC_GROUP NVIC_PriorityGroup_2		//中断分组选择
#define NVIC_PWMIN_P			1		//接收机采集
#define NVIC_PWMIN_S			1
#define NVIC_PWMIN_S2			0
#define NVIC_TIME_P       2		//暂未使用
#define NVIC_TIME_S       0
#define NVIC_UART_P				5		//暂未使用
#define NVIC_UART_S				1
#define NVIC_UART2_P			3		//串口2中断
#define NVIC_UART2_S			1
/***********************************************/
#define RC_PITCH  1
#define RC_ROLL   0
#define RC_YAW    3
#define RC_MODE   6
#define RC_THR    2
#define RC_GEAR   4
//================传感器===================
#define USE_US100           //使用us100型号超声波 
//#define USE_KS103					//使用ks103型号超声波
//#define SONAR_USE_SCL  
//#define SONAR_USE_TIG
#define SONAR_USE_UART 
//==============================================
#define OFFSET_AV_NUM 	50					//校准偏移量时的平均次数。
#define FILTER_NUM 			10					//滑动平均滤波数值个数

#define TO_ANGLE 				0.06103f 		//0.061036 //   4000/65536  +-2000   ???

#define FIX_GYRO_Y 			1.02f				//陀螺仪Y轴固有补偿
#define FIX_GYRO_X 			1.02f				//陀螺仪X轴固有补偿

#define TO_M_S2 				0.23926f   	//   980cm/s2    +-8g   980/4096
#define ANGLE_TO_RADIAN 0.01745329f //*0.01745 = /57.3	角度转弧度

#define MAX_ACC  4096.0f						//+-8G		加速度计量程
#define TO_DEG_S 500.0f      				//T = 2ms  默认为2ms ，数值等于1/T
//================控制=====================
#define MAX_CTRL_ANGLE			25.0f										//遥控能达到的最大角度
#define ANGLE_TO_MAX_AS 		30.0f										//角度误差N时，期望角速度达到最大（可以通过调整CTRL_2的P值调整）
#define CTRL_2_INT_LIMIT 		0.5f *MAX_CTRL_ANGLE		//外环积分幅度
#define MAX_FIX_ANGLE 6

#define MAX_CTRL_ASPEED 	 	300.0f									//ROL,PIT允许的最大控制角速度
#define MAX_CTRL_YAW_SPEED 	30.0f									//YAW允许的最大控制角速度
#define CTRL_1_INT_LIMIT 		0.5f *MAX_CTRL_ASPEED		//内环积分幅度
//================矢量舵机控制=======================
#define MAX_DJ_ANGLE 30  
#define SCALE_DJ 0.5
extern float dj_angle_set,dj_angle_offset[3];
//=================PWM========================
#define MAX_PWM				100			///%	最大PWM输出为100%油门
#define MAX_THR       80 			///%	油门通道最大占比80%，留20%给控制量
#define READY_SPEED   20			///%	解锁后电机转速20%油门
//================系统===================
#define USE_MY_PWM_OUT 0
#define NEW_FLY_BOARD 0  //0—>PWM使用5678
#define PLANE_IS_BIG  0  //0->穿越机
#define USE_CYCLE_HML_CAL  0//0->使用椭球拟合
#define DEBUG_WITHOUT_SB 0
#define GET_TIME_NUM 	(15)		//设置获取时间的数组数量
#define USE_TOE_IN_UNLOCK 0 // 0：默认解锁方式，1：外八解锁方式
//============== DMA使能=========================
#define EN_DMA_UART1 1  //UPLOAD
#define EN_DMA_UART2 1  //SD
#define EN_DMA_UART3 0  //GPS
#define EN_DMA_UART4 0  //FLOW
extern u8 fly_ready,force_Thr_low,mode_change;


#define RtA 		57.324841				
#define AtR    	0.0174533				
#define Acc_G 	0.0011963				
#define Gyro_G 	0.03051756				
#define Gyro_Gr	0.0005426

#define A_X 0
#define A_Y 1
#define A_Z 2
#define G_Y 3
#define G_X 4
#define G_Z 5
#define TEM 6
#define ITEMS 7

#define ROLr 0
#define PITr 1
#define THRr 2
#define YAWr 3
#define AUX1r 4 
#define AUX2r 5
#define AUX3r 6
#define AUX4r 7

enum
{
	SETBIT0 = 0x0001,  SETBIT1 = 0x0002,	SETBIT2 = 0x0004,	 SETBIT3 = 0x0008,
	SETBIT4 = 0x0010,	 SETBIT5 = 0x0020,	SETBIT6 = 0x0040,	 SETBIT7 = 0x0080,
	SETBIT8 = 0x0100,	 SETBIT9 = 0x0200,	SETBIT10 = 0x0400, SETBIT11 = 0x0800,
	SETBIT12 = 0x1000, SETBIT13 = 0x2000,	SETBIT14 = 0x4000, SETBIT15 = 0x8000		
};
//CLR BIT.    Example: a &= CLRBIT0
enum
{
	CLRBIT0 = 0xFFFE,  CLRBIT1 = 0xFFFD,	CLRBIT2 = 0xFFFB,	 CLRBIT3 = 0xFFF7,	
	CLRBIT4 = 0xFFEF,	 CLRBIT5 = 0xFFDF,	CLRBIT6 = 0xFFBF,	 CLRBIT7 = 0xFF7F,
	CLRBIT8 = 0xFEFF,	 CLRBIT9 = 0xFDFF,	CLRBIT10 = 0xFBFF, CLRBIT11 = 0xF7FF,
	CLRBIT12 = 0xEFFF, CLRBIT13 = 0xDFFF,	CLRBIT14 = 0xBFFF, CLRBIT15 = 0x7FFF
};
//CHOSE BIT.  Example: a = b&CHSBIT0
enum
{
	CHSBIT0 = 0x0001,  CHSBIT1 = 0x0002,	CHSBIT2 = 0x0004,	 CHSBIT3 = 0x0008,
	CHSBIT4 = 0x0010,	 CHSBIT5 = 0x0020,	CHSBIT6 = 0x0040,	 CHSBIT7 = 0x0080,
	CHSBIT8 = 0x0100,	 CHSBIT9 = 0x0200,	CHSBIT10 = 0x0400, CHSBIT11 = 0x0800,
	CHSBIT12 = 0x1000, CHSBIT13 = 0x2000,	CHSBIT14 = 0x4000, CHSBIT15 = 0x8000		
};

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).M4同M3类似,只是寄存器地址变了.
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //输出 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //输入

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //输出 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //输入

typedef unsigned short uint16_t;
typedef unsigned char  uint8_t;
typedef unsigned char  uint8;                   /* defined for unsigned 8-bits integer variable 	无符号8位整型变量  */
typedef signed   char  int8;                    /* defined for signed 8-bits integer variable		有符号8位整型变量  */
typedef unsigned short uint16;                  /* defined for unsigned 16-bits integer variable 	无符号16位整型变量 */
typedef signed   short int16;                   /* defined for signed 16-bits integer variable 		有符号16位整型变量 */
typedef unsigned int   uint32;                  /* defined for unsigned 32-bits integer variable 	无符号32位整型变量 */
typedef signed   int   int32;                   /* defined for signed 32-bits integer variable 		有符号32位整型变量 */
typedef float          fp32;                    /* single precision floating point variable (32bits) 单精度浮点数（32位长度） */
typedef double         fp64;                    /* double precision floating point variable (64bits) 双精度浮点数（64位长度） */



//zhong hang
extern float PWM_DJ[4],target_position_task_s[2],target_position_task_e[2],yaw_ctrl_out,Yaw_set_dji,T_SHOOT_CHECK,AVOID[2],exp_height_check,exp_height_head_scan,exp_height_head_shoot;
extern u8 num_need_to_check,state_v_test,en_shoot,SONAR_HEAD_CHECK[2],force_check_pass,cnt_shoot,tar_need_to_check_odroid[3];
extern _st_height_pid_v ultra_ctrl,ultra_ctrl_safe,ultra_ctrl_head;
extern int YUN_PER_OFF;//云台预偏
void Flow_reset_pos(void);
void Flow_save_tar_s(void);
void Flow_set_tar(float set);
//state
#define SG_LOW_CHECK 0
#define SG_MID_CHECK 1
#define SU_UP1 2
#define SU_HOLD 3
#define SD_RETRY_UP 4
#define SD_RETRY_UP_HOLD 5
#define SU_TO_CHECK_POS 6
#define SU_CHECK_TAR 7
#define SU_TO_START_POS 8


//-----------------------------------
#define SD_HOLD 13
#define SD_HOLD_BACK 23
#define SD_HOLD_BREAK 24
#define SD_SHOOT 25
#define SD_MISS_SEARCH 14
#define SD_HOLD2 15
#define SD_HIGH_FAST_DOWN 16
#define SD_CIRCLE_SLOW_DOWN 17
#define SD_CIRCLE_HOLD 18
#define SD_CIRCLE_MID_DOWN  19
#define SD_CHECK_G 20
#define SD_SHUT_DOWN 21
#define SD_SAFE 22
#define SD_TO_HOME 26

//-------------------------------------------------------------------------------------
#define TEST_GPS 0
#define USE_M100 1
//云台初始化位置


extern u16  PWM_DJ0 ;//1680//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1830) -  PWM_DJ[0] );  俯仰
extern u16	 PWM_DJ1 ;//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1500) -  PWM_DJ[1] );  左右
extern float SHOOT_PWM_OFF0,SHOOT_PWM_OFF1,SHOOT_PWM_DEAD0,SHOOT_PWM_DEAD1, Pitch_DJ,Roll_DJ;;//射击偏差cyds@stepholdings.com 

#if USE_M100
#define  OFF_RC_PIT 1500
#define  OFF_RC_ROL 1500
#define  OFF_RC_YAW 1500
#define  OFF_RC_THR 1500
#else
#define  OFF_RC_PIT 1520
#define  OFF_RC_ROL 1520
#define  OFF_RC_YAW 1520
#define  OFF_RC_THR 1520
#endif
extern double home_point[2],check_way_point[2];//={39.962704, 116.3038848};
extern double way_point[2][2],qr_gps_pos[2];//={39.9626144, 116.3038848,  39.962736, 116.303872};
extern double tar_point_globle[2],tar_point_globler[2],tar_now_gps[2];//={39.962704, 116.3038848};
extern u8 gps_target_change,en_vrc;
extern float k_m100[5];
#include "m100.h"
extern float k_m100_gps[3];//=  {2.25,2.25,1}; //p r t
extern float k_m100_scan[3];//= {1,2.25,1};
extern float k_m100_track[3],k_m100_shoot[3];//={1,1,1};
extern float k_m100_laser_avoid,k_m100_yaw;//=0.5;
extern u8 dji_rst_protect,dji_rst;
extern u8 DJI_CONNECT, tar_buf[20],dji_rc_miss;
extern u16 dji_miss_cnt;
extern u16 cnt_m100_data_refresh,S_head;
extern u8 m100_data_refresh;
extern u8 state_set_point;
#define SHOOT_USE_YUN 1
#define DEBUG_IN_ROOM 0
//-------------DEBUG_MODE_SEL------Warning!:Only can choose one mode---------
//#define DEBUG_TARGET_AVOID //debug shoot and track 
//#define DEBUG_TRACK //debug shoot and track 
//#define DEBUG_GPS_NAV
//#define DEBUG_HOLD_HEIGHT
//#define DEBUG_HOLD_WALL
#endif

