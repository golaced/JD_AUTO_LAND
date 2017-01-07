#ifndef _PWM_OUT_H_
#define _PWM_OUT_H_

#include "stm32f4xx.h"
#define MAXMOTORS 		(4)		//电机数量
u8 PWM_Out_Init(uint16_t hz);
void  Set_DJ(float ang1,float ang2,float ang3,float ang4);
void SetPwm(u32 pwm[8],int off[8],u32 min,u32 max);
void SEL_PWM(u8 sel);
void SEL_Init();
u8 PWM_Out_Init_FOR_CAL(uint16_t hz,uint16_t min,uint16_t max);
void SHOOT_Init(void);
void EN_SHOOT(u8 on);
#endif

