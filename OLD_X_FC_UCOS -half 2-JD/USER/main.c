#include "include.h" 
#include "iic_soft.h"
#include "hml_sample.h"
#include "ms5611.h"
#include "alt_kf.h"
#include "flash.h"
#include "led_fc.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "ucos_task.h"
#include "pwm_in.h"
#include "stm32f4xx_dma.h"
#include "circle.h"
#include "rng.h"
#include "AttitudeEKF.h"
 /////////////////////////UCOSII������������///////////////////////////////////
//START ����
//�����������ȼ�
#define START_TASK_PRIO      			20 //��ʼ��������ȼ�����Ϊ���
//���������ջ��С
#define START_STK_SIZE  				64
//�����ջ	
OS_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *pdata);	
//////////////////////////////////////////////////////////////////////////////
    
OS_EVENT * msg_key;			//���������¼���	  
OS_EVENT * q_msg;			//��Ϣ����

OS_FLAG_GRP * flags_key;	//�����ź�����
void * MsgGrp[256];			//��Ϣ���д洢��ַ,���֧��256����Ϣ
u8 en_read=1;
int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_GROUP);//����ϵͳ�ж����ȼ�����2
	delay_init(168);  //��ʼ����ʱ����
	SEL_Init();
	PWM_IN_Init();						//��ʼ�����ջ��ɼ�����
	pwmin.max=1500;
	pwmin.min=0;
	pwmin.T=3500;

	//-------------------------Para Init------------------------------------	
	Para_Init();							//������ʼ��
	if(en_read)
	READ_PARM();//��ȡ����

  pwmin.hz=1000000/pwmin.T;
	pwmin.cal_cycle=0;
	pwmin.CALIBRATE=0;
	pwmin.max=1929;pwmin.min=1107;pwmin.T=13600;pwmin.hz=73;
	PWM_Out_Init_FOR_CAL(pwmin.T ,pwmin.min,pwmin.max);//400hz
	SHOOT_Init();
	Initial_Timer_SYS();
	Delay_ms(100);
  I2c_Soft_Init();					//��ʼ��ģ��I2C
	Delay_ms(100);
	MS5611_Init();						//��ѹ�Ƴ�ʼ��
	Delay_ms(100);						//��ʱ
	MPU6050_Init(5);   			//���ٶȼơ������ǳ�ʼ��������20hz��ͨ
	Delay_ms(100);						//��ʱ
	HMC5883L_SetUp();	
	Delay_ms(100);
	MS5611_Init();						//��ѹ�Ƴ�ʼ��
	altUkfInit();
	LED_Init();								//LED���ܳ�ʼ��
	RNG_Init();
	Delay_ms(100);
//------------------------Uart Init-------------------------------------
	Usart1_Init(115200L);			//UP_LINK
	#if EN_DMA_UART1 
	MYDMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff1,SEND_BUF_SIZE1+2,1);//DMA2,STEAM7,CH4,����Ϊ����1,�洢��ΪSendBuff,����Ϊ:SEND_BUF_SIZE.
	#endif
	Usart2_Init(230400L);			//IMU 
	#if EN_DMA_UART2
	MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)SendBuff4,SEND_BUF_SIZE4+2,1);//DMA2,STEAM7,CH4,����Ϊ����1,�洢��ΪSendBuff,����Ϊ:SEND_BUF_SIZE.
	#endif
	#if USE_M100
	Usart4_Init(115200L);
	#else
	Usart4_Init(115200L);     //IMU DJ Link  --RTKGPS
	#endif
	#if EN_DMA_UART4 
	MYDMA_Config(DMA1_Stream4,DMA_Channel_4,(u32)&UART4->DR,(u32)SendBuff2,SEND_BUF_SIZE2+2,0);//DMA2,STEAM7,CH4,����Ϊ����1,�洢��ΪSendBuff,����Ϊ:SEND_BUF_SIZE.
	#endif
	Usart3_Init(230400L);     //VIDEO_LINK
	#if EN_DMA_UART3
	MYDMA_Config(DMA1_Stream3,DMA_Channel_4,(u32)&USART3->DR,(u32)SendBuff3,SEND_BUF_SIZE3+2,2);//DMA2,STEAM7,CH4,����Ϊ����1,�洢��ΪSendBuff,����Ϊ:SEND_BUF_SIZE.
  #endif
	Delay_ms(100);
	Ultrasonic_Init();   			//��������ʼ��Uart5
	Spi1_Init();		
	Nrf24l01_Init(MODEL_RX2,40);//��ƵƵ����ʼ��
  Nrf24l01_Check();
//-----------------------Mode &  Flag init--------------------	
//system
#define AUX_SEL 1
#if AUX_SEL //big
  track.control_k=0.325*0.76;//��̨��������3.8;//1.325;//2;//---------------------circle_K
	track.control_k_miss=0.325*0.76;//��̨��������2.8;//;
#else  //small
  track.control_k=0.45;//��̨��������3.8;//1.325;//2;//---------------------circle_K
	track.control_k_miss=0.45;//��̨��������2.8;//;
#endif
	track.control_yaw  =0.3;//��̨��׼��������
#if USE_M100
  circle.control_k=-0.12;//45/4;//��̨��׼��������1.325;
#else
	circle.control_k=-0.0525;//45/4;//��̨��׼��������1.325;
#endif
	track.forward=58;//ѭ�����ң����
	track.forward_end_dj_pwm=1.6;//�����������
	
	//--------------------
	fly_ready=0;
	mode.cal_sel=0;
	mode.height_upload=0;
	mode.dj_lock=0;
	mode.en_h_mode_switch=1;
	mode.en_dj_cal=0;	
	mode.no_head=0;
//pid
	mode.height_safe=0;	
	mode.hunman_pid=0;
	mode.yaw_imu_control=0;
	mode.en_pid_sb_set=0;
//flow
	mode.hold_use_flow=1;
	mode.flow_hold_position=0;//6  
	mode.flow_hold_position_use_global=0;
	mode.flow_hold_position_high_fix=0;
//thr
	mode.thr_fix=0;
	mode.thr_add_pos=1;
	mode.en_moto_smooth=0;
	mode.en_h_mode_switch=1;
//--sb
	mode.sb_smooth=1;
	mode.en_sensor_equal_flp=1;
	mode.mpu6050_bw_42=1;
	mode.sonar_avoid=0;
	mode.en_fuzzy_angle_pid=0;
	mode.en_sd_save=0;//sd_save_flag
	mode.en_imu_q_update_mine=0;
	mode.en_fuzzy_angle_pid=0;
	mode.flow_hold_position_high_fix=0;
	mode.yaw_sel=1;
	mode.pid_mode=0;
	mode.spid=1;
	mode.en_pid_out_pit=1;
	mode.en_pid_out_rol=1;
	mode.en_pid_out_yaw=1;
	mode.en_pid_fuzzy_p=0;
	mode.en_pid_yaw_angle_control=1;//yaw �Ƕ��⻷
	mode.pit_rol_pid_out_flp=0;
	mode.en_pid_yaw_control_by_pit_rol=1;
	mode.test1=0;
//---circle
  mode.circle_miss_fly=0;
//--
	RX_CH[PITr]=RX_CH[ROLr]=RX_CH[YAWr]=1500;
	RX_CH[THRr]=1000;
	//--ң����ƫִ
	RX_CH_FIX[ROLr]=4;//-15;
	RX_CH_FIX[PITr]=-13;//-1;
	RX_CH_FIX[THRr]=0;//7;
	RX_CH_FIX[YAWr]=1;//11;
	//--
	RX_CH[AUX1r]=500;
	RX_CH[AUX2r]=500;
	RX_CH[AUX3r]=500;
	RX_CH[AUX4r]=500;
  m100.Rc_gear=-4545;
	m100.Rc_mode=-8000;
	//-----------------DMA Init--------------------------
#if EN_DMA_UART4 
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����     
	MYDMA_Enable(DMA1_Stream4,SEND_BUF_SIZE2+2);     //��ʼһ��DMA���䣡
#endif
#if EN_DMA_UART2
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����     
	MYDMA_Enable(DMA1_Stream6,SEND_BUF_SIZE4+2);     //��ʼһ��DMA���䣡	
#endif
#if EN_DMA_UART3
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����     
	MYDMA_Enable(DMA1_Stream3,SEND_BUF_SIZE3+2);     //��ʼһ��DMA���䣡	
#endif
#if EN_DMA_UART1 
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����     
	MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE1+2);     //��ʼһ��DMA���䣡	 
#endif	
	Delay_ms(100);
	//---------------��ʼ��UCOSII--------------------------
//	while(1)
//	{	MPU6050_Read(); 															//��ȡmpu6�ᴫ����
//	  MPU6050_Data_Prepare( 0.005 );			//mpu6�ᴫ�������ݴ���
//		Delay_ms(5);
//		attitude_estimator_ekf_thread_main();
//	}
	OSInit();  	 				
	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//������ʼ����
	OSStart();	    
}
 

//��ʼ����
void start_task(void *pdata)
{
  OS_CPU_SR cpu_sr=0;
	u8 err;	    	    
	pdata = pdata; 	
	msg_key=OSMboxCreate((void*)0);		//������Ϣ����
	q_msg=OSQCreate(&MsgGrp[0],256);	//������Ϣ����
 	flags_key=OSFlagCreate(0,&err); 	//�����ź�����		  
	  
	OSStatInit();					//��ʼ��ͳ������.�������ʱ1��������
	//ע�������ʱ��
	tmr1=OSTmrCreate(10,10,OS_TMR_OPT_PERIODIC,(OS_TMR_CALLBACK)tmr1_callback,0,"tmr1",&err);		//100msִ��һ��  cpuʹ����
	OSTmrStart(tmr1,&err);//���������ʱ��1				 	
	tmr2=OSTmrCreate(10,5,OS_TMR_OPT_PERIODIC,(OS_TMR_CALLBACK)tmr2_callback,0,"tmr2",&err);		//50msִ��һ��  LED&&MODE
	OSTmrStart(tmr2,&err);//���������ʱ��1				 	
 	OS_ENTER_CRITICAL();			//�����ٽ���(�޷����жϴ��)    
 	//ע���߳� 	
  OSTaskCreate(mems_task,(void *)0,(OS_STK*)&MEMS_TASK_STK[MEMS_STK_SIZE-1],MEMS_TASK_PRIO);		
 	OSTaskCreate(inner_task,(void *)0,(OS_STK*)&INNER_TASK_STK[INNER_STK_SIZE-1],INNER_TASK_PRIO);	 
	OSTaskCreate(outer_task,(void *)0,(OS_STK*)&OUTER_TASK_STK[OUTER_STK_SIZE-1],OUTER_TASK_PRIO);
	OSTaskCreate(baro_task,(void *)0,(OS_STK*)&BARO_TASK_STK[BARO_STK_SIZE-1],BARO_TASK_PRIO);
	OSTaskCreate(sonar_task,(void *)0,(OS_STK*)&SONAR_TASK_STK[SONAR_STK_SIZE-1],SONAR_TASK_PRIO);	
	OSTaskCreate(nrf_task,(void *)0,(OS_STK*)&NRF_TASK_STK[NRF_STK_SIZE-1],NRF_TASK_PRIO);
	OSTaskCreate(uart_task,(void *)0,(OS_STK*)&UART_TASK_STK[UART_STK_SIZE-1],UART_TASK_PRIO);
	OSTaskCreate(flow_task,(void *)0,(OS_STK*)&FLOW_TASK_STK[FLOW_STK_SIZE-1],FLOW_TASK_PRIO);
	#if USE_M100
	OSTaskCreate(m100_task,(void *)0,(OS_STK*)&M100_TASK_STK[M100_STK_SIZE-1],M100_TASK_PRIO);
	#endif
	//--
 	OSTaskSuspend(START_TASK_PRIO);	//������ʼ����.
	OS_EXIT_CRITICAL();				//�˳��ٽ���(���Ա��жϴ��)
}
   

//�ź�������������
void flags_task(void *pdata)
{	
	u16 flags;	
	u8 err;	    						 
	while(1)
	{
		flags=OSFlagPend(flags_key,0X001F,OS_FLAG_WAIT_SET_ANY,0,&err);//�ȴ��ź���
 		
		OSFlagPost(flags_key,0X001F,OS_FLAG_CLR,&err);//ȫ���ź�������
 	}
}
   		    


