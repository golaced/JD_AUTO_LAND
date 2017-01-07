#ifndef _INCLUDE_H_
#define _INCLUDE_H_

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdarg.h>
 
#include "delay.h" 
#include "malloc.h"   
#include "../HARDWARE/DRIVER/time.h"
#include "../HARDWARE/MEMS/mpu6050.h"
#include "../HARDWARE/parameter.h"
#include "../HARDWARE/DRIVER/usart_fc.h"   					
#include "../HARDWARE/DRIVER/rc_mine.h"												
#include "../HARDWARE/MEMS/ultrasonic.h"
#include "../HARDWARE/DRIVER/flash.h"
#include "../HARDWARE/DRIVER/dma.h"
#include "../HARDWARE/CONTROL/att.h"
#include "../HARDWARE/CONTROL/height_ctrl.h"
#include "../HARDWARE/CONTROL/visual_contol.h"
#include "../HARDWARE/FUSHION/imu1.h"
#include "../HARDWARE/FUSHION/alt_kf.h"
#include "../HARDWARE/error.h"
#include "../HARDWARE/MEMS/hml_sample.h"
#include "../HARDWARE/MEMS/ms5611_2.h"
#include "../HARDWARE/FUSHION/alt_kf.h"
#include "../HARDWARE/DRIVER/flash.h"
#include "../HARDWARE/ucos_task.h"
#include "../HARDWARE/CONTROL/eso.h"
#include "../HARDWARE/CONTROL/neuron_pid.h"
#include "../HARDWARE/CONTROL/sonar_avoid.h"
#include "../HARDWARE/CONTROL/visual_contol.h"
#include "../HARDWARE/CONTROL/h_inf.h"
#include "../HARDWARE/CONTROL/att.h"
#include "../HARDWARE/CONTROL/height_ctrl.h"
#include "../HARDWARE/FUSHION/imu1.h"
#include "../HARDWARE/MATH/Quaternion.h"
#include "../HARDWARE/MATH/miniMatrix.h"
#include "../HARDWARE/DRIVER/pwm_out.h"
/***************�ж����ȼ�******************/
#define NVIC_GROUP NVIC_PriorityGroup_2		//�жϷ���ѡ��
/***********************************************/
//================������  ģ��ѡ��===================
//#define USE_US100           //ʹ��us100�ͺų����� 
#define USE_KS103					//ʹ��ks103�ͺų�����
//=================�����������˲�Ƶ��===================
//#define SONAR_SAMPLE1					//0-5m 32ms  no fix
#define SONAR_SAMPLE2					//0-5m 100ms T fix
//#define SONAR_SAMPLE3					//0-11m 68ms no fix
//=================������ͨѶģʽ===================
//#define SONAR_USE_SCL  
//#define SONAR_USE_TIG
#define SONAR_USE_UART    
#define SONAR_HEIGHT 50   //��������װ�߶�mm



#define USE_PXY 0    //δʹ�����޸�


//=======================����궨��==========
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

#define X 0
#define Y 1

#define East 0
#define North 1
//=======================�����궨��==========
#define RtA 		57.324841				
#define AtR    	0.0174533				
#define Acc_G 	0.0011963				
#define Gyro_G 	0.03051756				
#define Gyro_Gr	0.0005426

#define PI                     3.14159265359f //Բ����
#define RAD_DEG            		 57.2957795056f //����ת���ɽǶȵı�������
#define DEG_RAD                0.01745329252f //�Ƕ�ת���ɻ��ȵı�������
#define GRAVITY_MSS                  9.80665f //�����������ٶ�
#define earthRate                0.000072921f //������ת���ٶ�
#define earthRadius                6378145.0f //����뾶
#define earthRadiusInv          1.5678540e-7f //����뾶�ĵ���

#define TO_ANGLE 				0.06103f 		//0.061036 //   4000/65536  +-2000   ???

#define FIX_GYRO_Y 			1.02f				//������Y����в���
#define FIX_GYRO_X 			1.02f				//������X����в���

#define TO_M_S2 				0.23926f   	//   980cm/s2    +-8g   980/4096
#define ANGLE_TO_RADIAN 0.01745329f //*0.01745 = /57.3	�Ƕ�ת����

#define MAX_ACC  4096.0f						//+-8G		���ٶȼ�����
#define TO_DEG_S 500.0f      				//T = 2ms  Ĭ��Ϊ2ms ����ֵ����1/T



//================����=====================
#define MAX_CTRL_ANGLE			25.0f										//ң���ܴﵽ�����Ƕ�
#define ANGLE_TO_MAX_AS 		30.0f										//�Ƕ����Nʱ���������ٶȴﵽ��󣨿���ͨ������CTRL_2��Pֵ������
#define CTRL_2_INT_LIMIT 		0.8f *MAX_CTRL_ANGLE		//�⻷���ַ���
#define MAX_FIX_ANGLE 6

#define MAX_CTRL_ASPEED 	 	300.0f									//ROL,PIT����������ƽ��ٶ�
#define MAX_CTRL_YAW_SPEED 	60.0f									//YAW����������ƽ��ٶ�
#define CTRL_1_INT_LIMIT 		0.8f *MAX_CTRL_ASPEED		//�ڻ����ַ���//0.5

//================ʸ���������=======================δʹ������
#define MAX_DJ_ANGLE 30  
#define SCALE_DJ 0.5
extern float dj_angle_set,dj_angle_offset[3],Angle_Yun[2];
extern float off_yaw;//ң�ط���

//=================PWM========================
//#define HOLD_THR_PWM  450
#define HOLD_THR_PWM  386
#define MAX_PWM				LIMIT(2*HOLD_THR_PWM,0,100)///%	���PWM���Ϊ100%����
#define MAX_THR       80 			///%	����ͨ�����ռ��80%����20%��������
#define READY_SPEED   20      ///%	��������ת��20%����
//================ϵͳ����===================
#define USE_MINI_BOARD  1  //ʹ��MINI OLD-X �ɿذ�
#if USE_MINI_BOARD
#define FLASH_USE_STM32 1  // flash ʹ��stm32 �ڲ�EPROOM
#else
#define FLASH_USE_STM32 0  // flash ʹ��stm32 �ڲ�EPROOM
#endif
#define DRONE_330_ID 3145777//δʹ�����޸�
#define DRONE_350_ID 4915281//δʹ�����޸�

#define USE_RC_GROUND 1  //ʹ��NRF-Gģ��

#define IMU_HML_ADD_500 1//�µĴ����ں�
#define USE_BLE_FOR_APP 1//ֱ��ʹ��FC������APPͨ��

#define TUNING_X 0  //������ģʽ ����0=>��X->ROL��

#define TUNNING_DRONE_CHIP_ID 5308462  //������Ի��� оƬID
#define EN_TIM_INNER  0 //400Hz����  ��BUG����ʹ��
#define EN_ATT_CAL_FC 1 //��FC�������IMU����
#define EN_TIM_IMU  0     //δʹ������
#define USE_RECIVER_MINE 0//ʹ���Լ����ֱ� δʹ������
#define BLE_BAD 0        //δʹ������
#define NEW_FLY_BOARD 0  //0��>PWMʹ��5678   δʹ������
#define PLANE_IS_BIG  0  //			δʹ������ ���޸�
#define USE_CYCLE_HML_CAL  0//0->ʹ���������
#define DEBUG_WITHOUT_SB 0	//����ʱ����Ҫ�ֱ� �ϵ�2s���Զ�����
#define USE_TOE_IN_UNLOCK 0 //   δʹ������ ���޸�
#define DRONE_X6 0  //6��PWM���ģʽ  δʹ������ ���޸�
#define WIN8 0      //ʹ��8�罰  δʹ������ ���޸�
//============== DMAʹ��=========================
#define EN_DMA_UART1 1  //UPLOAD
#define EN_DMA_UART2 0  //FLOW
#define EN_DMA_UART3 0  //GPS
#define EN_DMA_UART4 1 //SD
//===============���ѡ��====================
//#define   ZHOU_550          
//#define   ZHOU_350
#define   ZHOU_300

extern u32 mcuID[3];
extern u8 fly_ready,force_Thr_low;



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


//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).M4ͬM3����,ֻ�ǼĴ�����ַ����.
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO�ڵ�ַӳ��
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
 
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //��� 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //����

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //��� 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //����

typedef unsigned short uint16_t;
typedef unsigned char  uint8_t;
typedef unsigned char  uint8;                   /* defined for unsigned 8-bits integer variable 	�޷���8λ���ͱ���  */
typedef signed   char  int8;                    /* defined for signed 8-bits integer variable		�з���8λ���ͱ���  */
typedef unsigned short uint16;                  /* defined for unsigned 16-bits integer variable 	�޷���16λ���ͱ��� */
typedef signed   short int16;                   /* defined for signed 16-bits integer variable 		�з���16λ���ͱ��� */
typedef unsigned int   uint32;                  /* defined for unsigned 32-bits integer variable 	�޷���32λ���ͱ��� */
typedef signed   int   int32;                   /* defined for signed 32-bits integer variable 		�з���32λ���ͱ��� */
typedef float          fp32;                    /* single precision floating point variable (32bits) �����ȸ�������32λ���ȣ� */
typedef double         fp64;                    /* double precision floating point variable (64bits) ˫���ȸ�������64λ���ȣ� */

extern u8 rc_thr_mid;
extern int16_t BLE_DEBUG[16];
#endif

