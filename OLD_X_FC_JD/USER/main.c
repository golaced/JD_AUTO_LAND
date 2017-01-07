#include "../HARDWARE/include.h" 
#include "../HARDWARE/MEMS/hml_sample.h"
#include "../HARDWARE/MEMS/ms5611_2.h"
#include "../HARDWARE/FUSHION/alt_kf.h"
#include "../HARDWARE/DRIVER/flash.h"
#include "../HARDWARE/CONTROL/visual_contol.h"
#include "../HARDWARE/DRIVER/iic.h"
#include "../HARDWARE/ucos_task.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "stm32f4xx_dma.h"
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

uint16_t cpuGetFlashSize(void)
{
   return (*(__IO u16*)(0x1FFF7A22));
}

//��ȡChipID
u32 mcuID[3];
void cpuidGetId(void)
{
    mcuID[0] = *(__IO u32*)(0x1FFF7A10);
    mcuID[1] = *(__IO u32*)(0x1FFF7A14);
    mcuID[2] = *(__IO u32*)(0x1FFF7A18);
}


//
    
OS_EVENT * msg_key;			//���������¼���	  
OS_EVENT * q_msg;			//��Ϣ����

OS_FLAG_GRP * flags_key;	//�����ź�����
void * MsgGrp[256];			//��Ϣ���д洢��ַ,���֧��256����Ϣ
u8 F_MPU6050=98;
int main(void)
{ cpuidGetId();                         //��ȡоƬID
	NVIC_PriorityGroupConfig(NVIC_GROUP);//����ϵͳ�ж����ȼ�����2
	delay_init(168); 					//��ʼ����ʱ����
	LED_Init();
  PWM_Out_Init(400);				//��ʼ������������
	Initial_Timer_SYS();      //usʱ�Ӽ�������ʼ��
	Delay_ms(100);
	#if EN_ATT_CAL_FC					//ʹ��FC������̬����	
		I2c_Soft_Init();					//��ʼ��ģ��I2C
		Delay_ms(100);
		MS5611_Init();						//��ѹ�Ƴ�ʼ��
		F_MPU6050=20;							//MPU6050�˲�Ƶ��
		MPU6050_Init(F_MPU6050);   
		Delay_ms(100);		
    I2C_FastMode=0; 	
		HMC5883L_SetUp();
    I2C_FastMode=1;	
		Delay_ms(100);
	#endif	
//---------------------------------���ڳ�ʼ�� Init-------------------------------------
	Usart1_Init(115200L);			//����
	#if EN_DMA_UART1 
	MYDMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff1,SEND_BUF_SIZE1+2,1);//DMA2,STEAM7,CH4,����Ϊ����1,�洢��ΪSendBuff,����Ϊ:SEND_BUF_SIZE.
	#endif
	Usart2_Init(921600L);			//IMU_LINK
	#if EN_DMA_UART2
	MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)SendBuff2,SEND_BUF_SIZE2+2,1);//DMA2,STEAM7,CH4,����Ϊ����1,�洢��ΪSendBuff,����Ϊ:SEND_BUF_SIZE.
	#endif
	
  Usart4_Init(256000L);     //���ջ�  SD��

	#if EN_DMA_UART4 
	MYDMA_Config(DMA1_Stream4,DMA_Channel_4,(u32)&UART4->DR,(u32)SendBuff4,SEND_BUF_SIZE4+2,0);//DMA2,STEAM7,CH4,����Ϊ����1,�洢��ΪSendBuff,����Ϊ:SEND_BUF_SIZE.
	#endif
	#if USE_PXY										
	Usart3_Init(115200L);  
	#else
	Usart3_Init(115200L);     //  Laser
	#endif
	#if EN_DMA_UART3
	MYDMA_Config(DMA1_Stream3,DMA_Channel_4,(u32)&USART3->DR,(u32)SendBuff3,SEND_BUF_SIZE3+2,2);//DMA2,STEAM7,CH4,����Ϊ����1,�洢��ΪSendBuff,����Ϊ:SEND_BUF_SIZE.
  #endif
  Uart5_Init(230400L);      // ͼ��Odroid  FC-M100
	Delay_ms(100);
//-------------------------flashʼ��------------------------------------	
	Para_Init();				   //PID ������ʼ��		
  #if !FLASH_USE_STM32	
	W25QXX_Init();			
	while(W25QXX_ReadID()!=W25Q32)								//��ⲻ��flash
	Delay_ms(100);
	#endif
	READ_PARM();//��ȡ����
//-----------------------Mode &  Flag init--------------------	
//------visual------------
  mode.use_dji=0;
  circle.control_k=1.325;
  track.control_k=2.25;//1.325;//2;//---------------------circle_K
	track.control_k_miss=2.15;//;
	track.control_yaw  =0.2;
	track.forward=80;
	track.forward_end_dj_pwm=1700;
	fly_ready=0;
//------system----------	
	mode.cal_sel=0;
	mode.height_upload=0;
	mode.dj_lock=0;
	mode.en_h_mode_switch=1;
	mode.en_dj_cal=0;	
	mode.no_head=0;
	mode.en_imu_ekf=0;
	mode.imu_use_mid_down=1;
//--pid
  mode.h_is_fix=1;
	mode.height_safe=0;	
	mode.use_px4_err=1;
	mode.hunman_pid=0;
	mode.yaw_imu_control=0;
	mode.en_pid_sb_set=0;
	mode.en_eso=0;
	mode.en_fuzzy_angle_pid=1;//eso
	mode.en_eso_h_in=0;
//--flow
	mode.hold_use_flow=1;
	mode.flow_hold_position=0;//6  
	mode.flow_hold_position_use_global=0;
	mode.flow_hold_position_high_fix=0;
//--thr
  mode.thr_fix_test=0;//�����½�����
	mode.thr_fix=0;
	mode.thr_add_pos=1;
	mode.en_moto_smooth=0;
//--sb
	mode.sb_smooth=1;
	mode.en_sensor_equal_flp=1;
	mode.mpu6050_bw_42=1;
	mode.sonar_avoid=0;
	mode.en_sd_save=0;//sd_save_flag
	mode.en_imu_q_update_mine=0;
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
//--------------------------ң����ƫִ��ʼ��-----
	RX_CH[PITr]=RX_CH[ROLr]=RX_CH[YAWr]=1500;RX_CH[THRr]=1000;
	//--ң����ƫ��
	RX_CH_FIX[ROLr]=0;//-15;
	RX_CH_FIX[PITr]=0;//-1;
	RX_CH_FIX[THRr]=0;//7;
	RX_CH_FIX[YAWr]=0;//11;
	//--
	RX_CH[AUX1r]=500;
	RX_CH[AUX2r]=500;
	RX_CH[AUX3r]=500;
	RX_CH[AUX4r]=500;
	//-----------------����DMA Init--------------------------
#if EN_DMA_UART4 
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);       
	MYDMA_Enable(DMA1_Stream4,SEND_BUF_SIZE4+2);    
#endif
#if EN_DMA_UART2
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);       
	MYDMA_Enable(DMA1_Stream6,SEND_BUF_SIZE2+2);     	
#endif
#if EN_DMA_UART3
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);      
	MYDMA_Enable(DMA1_Stream3,SEND_BUF_SIZE3+2);  	
#endif
#if EN_DMA_UART1 
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);     
	MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE1+2);     
#endif	
	Delay_ms(50);//�ϵ���ʱ
//-----------------------���Դ���-----------------------------	
//	while(1)
//	{
//	oldx_avoid();
//	Delay_ms(4);
//	}
	//----------------------��ʼ��UCOSII--------------------------
//	#if EN_TIM_INNER   //ʹ��400Hz�ڻ�����
//	TIM3_Int_Init(25-1,8400-1);	//��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����5000��Ϊ500ms 
//	#else
//	TIM3_Int_Init(250-1,8400-1);	//��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����5000��Ϊ500ms 
//  #endif	
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
	OSTaskCreate(outer_task,(void *)0,(OS_STK*)&OUTER_TASK_STK[OUTER_STK_SIZE-1],OUTER_TASK_PRIO);
	#if !EN_TIM_INNER
	OSTaskCreate(inner_task,(void *)0,(OS_STK*)&INNER_TASK_STK[INNER_STK_SIZE-1],INNER_TASK_PRIO);
	#endif
	#if EN_ATT_CAL_FC
	OSTaskCreate(ekf_task,(void *)0,(OS_STK*)&EKF_TASK_STK[EKF_STK_SIZE-1],EKF_TASK_PRIO);
	#endif
	OSTaskCreate(pos_task,(void *)0,(OS_STK*)&POS_TASK_STK[POS_STK_SIZE-1],POS_TASK_PRIO);	
	OSTaskCreate(ros_task,(void *)0,(OS_STK*)&ROS_TASK_STK[ROS_STK_SIZE-1],ROS_TASK_PRIO);	
	OSTaskCreate(nrf_task,(void *)0,(OS_STK*)&NRF_TASK_STK[NRF_STK_SIZE-1],NRF_TASK_PRIO);
	OSTaskCreate(uart_task,(void *)0,(OS_STK*)&UART_TASK_STK[UART_STK_SIZE-1],UART_TASK_PRIO);
	OSTaskCreate(baro_task,(void *)0,(OS_STK*)&BARO_TASK_STK[BARO_STK_SIZE-1],BARO_TASK_PRIO);
	OSTaskCreate(ident_task,(void *)0,(OS_STK*)&IDENT_TASK_STK[IDENT_STK_SIZE-1],IDENT_TASK_PRIO);
	OSTaskCreate(error_task,(void *)0,(OS_STK*)&ERROR_TASK_STK[ERROR_STK_SIZE-1],ERROR_TASK_PRIO);
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
   		    


