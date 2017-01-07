#ifndef _TIME_H_
#define _TIME_H_

#include "stm32f4xx.h"

void TIM_INIT(void);
void sys_time(void);

u16 Get_Time(u8,u16,u16);

float Get_Cycle_T(u8 );

void Cycle_Time_Init(void);

extern volatile uint32_t sysTickUptime;
extern int time_1h,time_1m,time_1s,time_1ms;

void Delay_us(uint32_t);
void Delay_ms(uint32_t);
void SysTick_Configuration(void);
uint32_t GetSysTime_us(void);


void Initial_Timer_SYS(void);
uint32_t micros(void);


#define GET_T_OUTTER 0
#define GET_T_INNER 1
#define GET_T_FLOW_UART 2
#define GET_T_SONAR_SAMPLE 3
#define GET_T_BARO_UKF 4
#define GET_T_HIGH_CONTROL_I 9
#define GET_T_HIGH_CONTROL_O 10

#endif
