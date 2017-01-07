#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "stm32f4xx.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"


extern OS_TMR   * tmr1;			//�����ʱ��1
extern OS_TMR   * tmr2;			//�����ʱ��2
extern OS_TMR   * tmr3;			//�����ʱ��3
void tmr1_callback(OS_TMR *ptmr,void *p_arg); 		  	   
void tmr2_callback(OS_TMR *ptmr,void *p_arg); 	  	   
void tmr3_callback(OS_TMR *ptmr,void *p_arg); 
	
//�����������ȼ� 0->Highest     
//10->��ʼ��������ȼ�����Ϊ���
#define MEMS_TASK_PRIO       			  1 //MEMS
#define INNER_TASK_PRIO       			2 //�ڻ�
#define OUTER_TASK_PRIO       			3 //�⻷
#define BARO_TASK_PRIO       			  4 //��ѹ�Ʋɼ���UKF
#define SONAR_TASK_PRIO       			5 //�������ɼ�
#define M100_TASK_PRIO       			  6 //M100
#define FLOW_TASK_PRIO       			  7 //FLOW
#define NRF_TASK_PRIO       			  8 //��ƵͨѶ
#define UART_TASK_PRIO       			  9 //����ͨѶ

//-----------------------MEMS�����߳�
//���������ջ��С
#define MEMS_STK_SIZE  					64*2
//�����ջ	
extern OS_STK MEMS_TASK_STK[MEMS_STK_SIZE];
//������
void mems_task(void *pdata);

//-----------------------INNER�����߳�
//���������ջ��С
#define INNER_STK_SIZE  					64*3
//�����ջ	
extern OS_STK INNER_TASK_STK[INNER_STK_SIZE];
//������
void inner_task(void *pdata);

//------------------------OUTER�����߳�
//���������ջ��С
#define OUTER_STK_SIZE  					64*3
//�����ջ	
extern OS_STK OUTER_TASK_STK[OUTER_STK_SIZE];
//������
void outer_task(void *pdata);

//------------------------NRF���߳�
//���������ջ��С
#define NRF_STK_SIZE  					64*3
//�����ջ	
extern OS_STK NRF_TASK_STK[NRF_STK_SIZE];
//������
void nrf_task(void *pdata);

//------------------------BARO�߳�
//���������ջ��С
#define BARO_STK_SIZE  					64*3
//�����ջ	
extern OS_STK BARO_TASK_STK[BARO_STK_SIZE];
//������
void baro_task(void *pdata);

//-----------------------SONAR�߳�
//���������ջ��С
#define SONAR_STK_SIZE  					64*10
//�����ջ	
extern OS_STK SONAR_TASK_STK[SONAR_STK_SIZE];
//������
void sonar_task(void *pdata);

//------------------------UART�߳�
//���������ջ��С
#define UART_STK_SIZE  					64*2
//�����ջ	
extern OS_STK UART_TASK_STK[UART_STK_SIZE];
//������
void uart_task(void *pdata);
//

//-----------------------FLOW�߳�
//���������ջ��С
#define FLOW_STK_SIZE  					64*3
//�����ջ	
extern OS_STK FLOW_TASK_STK[FLOW_STK_SIZE];
//������
void flow_task(void *pdata);
//

//-----------------------M100�߳�
//���������ջ��С
#define M100_STK_SIZE  					64*3
//�����ջ	
extern OS_STK M100_TASK_STK[M100_STK_SIZE];
//������
void m100_task(void *pdata);
//

#endif

