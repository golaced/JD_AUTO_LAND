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
#define EKF_TASK_PRIO       			  3 //EKF
#define OUTER_TASK_PRIO       			4 //�⻷
#define ROS_TASK_PRIO       			  5 //DJI
#define IDENT_TASK_PRIO       		  6 //IDENT

#define POS_TASK_PRIO       			  14 //λ��
#define BARO_TASK_PRIO       			  15 //��ѹ�Ʋɼ���UKF
#define SONAR_TASK_PRIO       			16 //�������ɼ�
#define NRF_TASK_PRIO       			  17 //��ƵͨѶ
#define UART_TASK_PRIO       			  18 //����ͨѶ

#define ERROR_TASK_PRIO       			19 //����
//defin START_TASK_PRIO      			  20//��ʼ��������ȼ�����Ϊ���

#define F_CONTROL_1000HZ 0
//----------------Period of TASK in ms (u8)---------------
#if F_CONTROL_1000HZ 
	#define F_INNER 		1
	#define F_OUTTER 		2
	#define F_EKF 			2
#else
	#define F_INNER 		5
	#define F_OUTTER 		10
	#define F_EKF 			10
#endif
#define F_POS 				20
#define F_BARO 				10

#define H_INNER				10
#define H_OUTTER      20

//-----------------------MEMS�����߳�
//���������ջ��С
#define MEMS_STK_SIZE  					64*2
//�����ջ	
extern OS_STK MEMS_TASK_STK[MEMS_STK_SIZE];
//������
void mems_task(void *pdata);


//-----------------------DJI�����߳�
//���������ջ��С
#define ROS_STK_SIZE  					64*2
//�����ջ	
extern OS_STK ROS_TASK_STK[ROS_STK_SIZE];
//������
void ros_task(void *pdata);

//-----------------------IDENT�����߳�
//���������ջ��С
#define IDENT_STK_SIZE  					64*2
//�����ջ	
extern OS_STK IDENT_TASK_STK[IDENT_STK_SIZE];
//������
void ident_task(void *pdata);



//-----------------------INNER�����߳�
//���������ջ��С
#define INNER_STK_SIZE  					64*4
//�����ջ	
extern OS_STK INNER_TASK_STK[INNER_STK_SIZE];
//������
void inner_task(void *pdata);

//------------------------OUTER�����߳�
//���������ջ��С
#define OUTER_STK_SIZE  					64*6
//�����ջ	
extern OS_STK OUTER_TASK_STK[OUTER_STK_SIZE];
//������
void outer_task(void *pdata);

//------------------------EKF�����߳�
//���������ջ��С
#define EKF_STK_SIZE  					64*4
//�����ջ	
extern OS_STK EKF_TASK_STK[EKF_STK_SIZE];
//������
void ekf_task(void *pdata);


//------------------------POS�����߳�
//���������ջ��С
#define POS_STK_SIZE  					64*8
//�����ջ	
extern OS_STK POS_TASK_STK[POS_STK_SIZE];
//������
void pos_task(void *pdata);

//------------------------NRF���߳�
//���������ջ��С
#define NRF_STK_SIZE  					64*3
//�����ջ	
extern OS_STK NRF_TASK_STK[NRF_STK_SIZE];
//������
void nrf_task(void *pdata);

//------------------------BARO�߳�
//���������ջ��С
#define BARO_STK_SIZE  					64*8
//�����ջ	
extern OS_STK BARO_TASK_STK[BARO_STK_SIZE];
//������
void baro_task(void *pdata);

//-----------------------SONAR�߳�
//���������ջ��С
#define SONAR_STK_SIZE  					64*1
//�����ջ	
extern OS_STK SONAR_TASK_STK[SONAR_STK_SIZE];
//������
void sonar_task(void *pdata);

//------------------------UART�߳�
//���������ջ��С
#define UART_STK_SIZE  					64*4
//�����ջ	
extern OS_STK UART_TASK_STK[UART_STK_SIZE];
//������
void uart_task(void *pdata);
//

//-----------------------ERROR�߳�
//���������ջ��С
#define ERROR_STK_SIZE  					64
//�����ջ	
extern OS_STK ERROR_TASK_STK[ERROR_STK_SIZE];
//������
void error_task(void *pdata);
//
#endif

