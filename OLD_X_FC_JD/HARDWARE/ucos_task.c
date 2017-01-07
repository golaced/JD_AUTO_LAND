#include "../HARDWARE/include.h" 
#include "../HARDWARE/SYS_IDENT/ident.h"
#include "../HARDWARE/ucos_task.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
u8 read_rc_flag;
float YawR,PitchR,RollR;	
float fRPY[3] = {0};
float inner_loop_time,inner_loop_time_time,inner_loop_time_imu;
u16 Rc_Pwm_In[8];
//��ʱ��3�жϷ�����
float GET_T_INNER_TIM_USE1;
u32 rc_read_interupt[2];
void TIM3_IRQHandler(void)//     400Hz����ʱ�ڻ��ж�
{static u8 cnt,cnt1,cnt_init,init;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //����ж�
	{
	#if EN_TIM_INNER
	inner_loop_time_time = Get_Cycle_T(GET_T_INNER_TIM); 						//��ȡ�ڻ�׼ȷ��ִ������	
	if(!init){
	if(cnt_init++>10)
	init=1;
	inner_loop_time_time=0.0025;
	}
	else{
	#if EN_ATT_CAL_FC
	MPU6050_Read(); 															//��ȡmpu6�ᴫ����
	MPU6050_Data_Prepare( inner_loop_time_time );	//mpu6�ᴫ�������ݴ���
	if(cnt++>=4){cnt=0;I2C_FastMode=0;ANO_AK8975_Read();I2C_FastMode=1;	}			  //��ȡ������������	
	if(cnt1++>=4){cnt1=0;
	MS5611_ThreadNew();}													//��ȡ��ѹ��
	#endif		
	CTRL_1( inner_loop_time_time ); 							//�ڻ�����					
	}
	#endif	
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //���ж�
}


//========================�⻷  ������============================
OS_STK INNER_TASK_STK[INNER_STK_SIZE];
float dj[2];
void inner_task(void *pdata)
{	static u8 cnt,cnt1,cnt2;			
  static u8 init,cnt_init;
  u8 i,j;	
 	while(1)
	{	
	float temp = Get_Cycle_T(GET_T_INNER_TIM);								
	if(temp>0.001)
	inner_loop_time_time=temp;
	else
	inner_loop_time_time=(float)F_INNER/1000.;
	if(!init){if(cnt_init++>40)
		init=1;
	inner_loop_time_time=0.005;
	}
	else{
	#if EN_ATT_CAL_FC	                            //ʹ��FCģ�������̬����
	MPU6050_Read(); 															
	MPU6050_Data_Prepare( inner_loop_time_time);			
	if(cnt++>=2){cnt=0;ANO_AK8975_Read();}			
	if(cnt1++>=2){cnt1=0;MS5611_ThreadNew();}	
	#endif
	
	#if !EN_TIM_INNER		
	RC_Duty( inner_loop_time_time , Rc_Pwm_In );//RC�˲�		
	#endif	
	CTRL_1( inner_loop_time_time ); 	
  }
	delay_ms(F_INNER);
	}
}		


//========================�⻷  ������============================
OS_STK OUTER_TASK_STK[OUTER_STK_SIZE];

float outer_loop_time,outer_loop_time_C;
void outer_task(void *pdata)
{	static u8 cnt,cnt1,cnt2;			
  static u8 init,cnt_init;
  u8 i,j;	
 	while(1)
	{	float temp = Get_Cycle_T(GET_T_OUTTER_C);								
		if(temp>0.001)
		outer_loop_time_C=temp;
		else
		outer_loop_time_C=(float)F_OUTTER/1000.;
	if(!init){if(cnt_init++>40)
		init=1;
	outer_loop_time_C=0.01;
	}
	else{
	#if EN_TIM_INNER		
	RC_Duty( outer_loop_time_C , Rc_Pwm_In );		
	#endif	
	//if(!mode.en_h_inf)
	CTRL_2( outer_loop_time_C ); 			//�⻷�Կ���PID										
	// else
	//h_inf_att_out(except_A.x,Roll);
#if EN_TIM_INNER	
	Thr_Ctrl(outer_loop_time_C);// ���ſ���
#endif
  }
#if EN_TIM_INNER	
	delay_ms(5);
#else
	delay_ms(F_OUTTER);
	//OSTimeDly(10*OS_TICKS_PER_SEC/1000);
#endif	

	}
}		


//=======================��̬����������============================
OS_STK EKF_TASK_STK[EKF_STK_SIZE];
float ekf_loop_time;
float Pit_fc,Rol_fc,Yaw_fc,Yaw_fc_q;
int flag_hml[3]={1,-1,1};
void ekf_task(void *pdata)
{	static u8 cnt,cnt1,cnt2;			
  static u8 init,cnt_init;	
	float temp0,temp1,temp2,temp3;
 	while(1)
	{	
	if(!init){if(cnt_init++>40)
		init=1;
	 ekf_loop_time=0.01;
	}
	else
	{ float temp = Get_Cycle_T(GET_T_EKF);								
		if(temp>0.001)
		ekf_loop_time=temp;
		else
		ekf_loop_time=(float)F_EKF/1000.;
			
	if(cnt1++>1){cnt1=0;
  	IMUupdate(0.5f *ekf_loop_time*2,my_deathzoom_2(mpu6050_fc.Gyro_deg.x,0.0), my_deathzoom_2(mpu6050_fc.Gyro_deg.y,0.0), 
		my_deathzoom_2(mpu6050_fc.Gyro_deg.z,0.5), mpu6050_fc.Acc.x, mpu6050_fc.Acc.y, mpu6050_fc.Acc.z,
		ak8975_fc.Mag_Val.x,ak8975_fc.Mag_Val.y,ak8975_fc.Mag_Val.z,
		&temp0,&temp1,&Yaw_fc);		
				}//ֻ���㺽��
	
		MadgwickAHRSupdate(ekf_loop_time,my_deathzoom_2(mpu6050_fc.Gyro_deg.x,0.5)/57.3, my_deathzoom_2(mpu6050_fc.Gyro_deg.y,0.5)/57.3, 
		my_deathzoom_2(mpu6050_fc.Gyro_deg.z,0.5)/57.3,(float)mpu6050_fc.Acc.x/4096., (float)mpu6050_fc.Acc.y/4096., (float)mpu6050_fc.Acc.z/4096.,
		0,0,0,
		//ak8975_fc.Mag_Val.x*flag_hml[0],ak8975_fc.Mag_Val.y*flag_hml[1],ak8975_fc.Mag_Val.z*flag_hml[2],
		&Rol_fc,&Pit_fc,&Yaw_fc_q);//���㸩���ͺ��
		}

#if EN_TIM_INNER	
	delay_ms(5);
#else
	delay_ms(F_EKF);
#endif	
	}
}		


//========================λ�ÿ���  ������============================
OS_STK POS_TASK_STK[POS_STK_SIZE];
float Yaw_DJ,Pitch_DJ;	
void pos_task(void *pdata)
{	static u8 cnt,cnt1,cnt2;			
  static u8 init,cnt_init;	
 	while(1)
	{	
	if(cnt++>5-1){cnt=0;
	if(!mode.use_dji) 
	;//GPS_calc_poshold(0.01*2);//����GPS����
	
 }
	
 
	if(cnt2++>0.01*10&&mode.en_sonar_avoid){cnt2=0;
		;//oldx_avoid();  //���� δʹ��
	}
	
  Positon_control((float)F_POS/1000.);//     �������� 
		
	delay_ms(F_POS);
	}
}		

//=========================��Ƶ  ģʽ�л� ������======================
OS_STK NRF_TASK_STK[NRF_STK_SIZE];
void nrf_task(void *pdata)
{	static u8 KEY_REG[8];						 
	static u16 cnt,cnt2,cnt3;
	u8 i;
 	while(1)
	{
		 if(cnt_loss_nrf++>1500/50){cnt_loss_nrf=1500/50+1;loss_nrf=1;}
		 if(imu_loss_cnt++>1500/50){imu_loss_cnt=1500/50+1;NAV_BOARD_CONNECT=0;}
	
		//---------------use now
		//------------0 1   |   2 3       KEY_SEL
		#if USE_RECIVER_MINE		
		mode.flow_hold_position=KEY_SEL[0];
    #else
    mode.en_sonar_avoid=KEY_SEL[0];		 
    #endif
		mode.en_sd_save=KEY_SEL[1];
		mode.en_pid_sb_set=KEY_SEL[2];//ʹ��PID����	
//-------------------------------------------------	
		#if  DEBUG_WITHOUT_SB
		if(cnt2++>200)//
		{fly_ready=1;cnt2=200+1;}
		#else
			#if !USE_RC_GROUND
				if(Rc_Get_PWM.AUX1>1500)
					fly_ready=1;
					else
					fly_ready=0;
			#else
			fly_ready=KEY_SEL[3];//����
			#endif
		#endif
					
	  //------------7 6 5 4  |  3 2 1 0  KEY
		//mode.trig_flow_spd= KEY[7];//1
		//mode.trig_h_spd=KEY[4];
	 
    mode.flow_f_use_ukfm=1;//KEY[7];
		//mode.baro_f_use_ukfm=1;//
		mode.en_eso_h_in=1;
	  mode.flow_d_acc=KEY[7];//�����ٶȻ����ٶ�D
		//mode.baro_lock=KEY[6];//��ѹ�������
		mode.yaw_sel=!KEY[3];
		//mode.att_ident1=KEY[4];
		//if(Rc_Get_PWM.AUX1>1500)
		//mode.height_safe=1;//mode.en_sd_save=1;
		//else
		//mode.height_safe=0;//	mode.en_sd_save=0;
//		if(mode.flow_hold_position<1)
//			mode.height_safe=1;
//		else{
		#if USE_RC_GROUND
			if(Rc_Get_PWM.AUX1>1500)
			mode.height_safe=1;//mode.en_sd_save=1;
			else
			mode.height_safe=0;//	mode.en_sd_save=0;
		#endif	
			//}
		mode.att_pid_tune=KEY[6]&&KEY[5]&&KEY[3]&&KEY[2]&&KEY[1]&&KEY[0];
		
		if(!mode.att_pid_tune){
		if(KEY[2]!=KEY_REG[2] && !fly_ready &&Thr_Low)//���ٶȼ�У׼����
		ak8975.Mag_CALIBRATED=1;
		}
		
		for(i=0;i<=7;i++)
		KEY_REG[i]=KEY[i];
		delay_ms(50);
	}
}		

//=======================��ѹ���ں�������==================
float baro_task_time;
OS_STK BARO_TASK_STK[BARO_STK_SIZE];
void baro_task(void *pdata)
{	static u8 init;
  static u16 cnt_init;	
 	while(1)
	{
	if(!init){if(cnt_init++>40)
		init=1;
	 baro_task_time=0.020;
	}
	else
	{ float temp = Get_Cycle_T(GET_T_BARO_UKF);							
		if(temp>0.001)
		baro_task_time=temp;
		else
		baro_task_time=(float)F_BARO/1000.;	
 
	  ukf_baro_task1(baro_task_time)	;
	}
	 delay_ms(F_BARO);
  }
}	

//=======================ROS ������==================

OS_STK ROS_TASK_STK[ROS_STK_SIZE];
void ros_task(void *pdata)//ROS TASK
{	 static u8 dj_mode_reg;					  
 	while(1)
	{

	 delay_ms(50);
  }
}	

//=======================IDENT ������==================

OS_STK IDENT_TASK_STK[IDENT_STK_SIZE];
void ident_task(void *pdata)//IDNET TASK
{	 static u8 dj_mode_reg;					  
 	while(1)
	{
		if(mode.att_ident1)
		;//ident();	//δʹ��
		delay_ms(10);
  }
}	

//=======================���� ������===========================
OS_STK  UART_TASK_STK[UART_STK_SIZE];
u8 UART_UP_LOAD_SEL=26;//<------------------------------�ϴ�����ѡ��
u8 force_flow_ble_debug;
u8 state_test=2;
void uart_task(void *pdata)
{	static u8 cnt[4];					 		
 	while(1)
	{
				//To  Odroid ͼ��ģ��
				if(cnt[0]++>2){cnt[0]=0;
						#if EN_DMA_UART3 
					if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)//�ȴ�DMA2_Steam7�������
								{ 
							DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//���DMA2_Steam7������ɱ�־
							data_per_uart3();
						  USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����     
							MYDMA_Enable(DMA1_Stream3,SEND_BUF_SIZE3);     //��ʼһ��DMA���䣡	
								}	
						#else
								UsartSend_GPS(state_test);//Send_IMU_TO_GPS();	
						#endif
							}			
				
				//To  IMUģ��	
				if(cnt[1]++>5){cnt[1]=0;	
				  #if EN_DMA_UART2 					
					if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET)//�ȴ�DMA2_Steam7�������
								{ 
							DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);//���DMA2_Steam7������ɱ�־
							data_per_uart2();
					    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����     
							MYDMA_Enable(DMA1_Stream6,SEND_BUF_SIZE2+2);     //��ʼһ��DMA���䣡	
								}	
					#else
								 GOL_LINK_TASK();	
					#endif
							}					
							
				//BLE UPLOAD��----------------------��������
					
					if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//�ȴ�DMA2_Steam7�������
							{ 	DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//���DMA2_Steam7������ɱ�־
								  SendBuff1_cnt=0;
									#if USE_BLE_FOR_APP			  
									APP_LINK();
									#endif
								  
							if(cnt[2]++>1){cnt[2]=0;
								   Send_IMU_FC();
									#if !BLE_BAD
								    if(mode.att_pid_tune){//PID TUNING
											
											if(mcuID[0]==TUNNING_DRONE_CHIP_ID)
											{
											if(ctrl_2.PID->kp!=0&&KEY[7])	
											data_per_uart1(
											
											0,-except_A.x*10,0,
											#if EN_ATT_CAL_FC										
											0,-Rol_fc*10,0,
											#else
											0,-Roll*10,0,
											#endif
											-ctrl_2.err.y*10,-eso_att_outter[ROLr].disturb*10,0,
											(int16_t)(eso_att_outter_c[ROLr].disturb_u*100),(int16_t)(0*10.0),(int16_t)(0*100.0),0/10,0,0/10,0*0);
											else
											data_per_uart1(
											0,-except_AS.x,0,
											0,-mpu6050.Gyro_deg.x,0,  
											-ctrl_1.err.y,-eso_att_inner[ROLr].disturb,0,
											(int16_t)(eso_att_outter_c[ROLr].disturb_u*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0/10,0,0/10,0*0);	
											
											}else{
											
											if(KEY[7])//OUTTER
											data_per_uart1(
											#if TUNING_X
											0,-except_A.x*10,0,
											#else
											0,except_A.y*10,0,
											#endif
											#if EN_ATT_CAL_FC
											#if TUNING_X
											0,-Rol_fc*10,0,
											#else
											0,Pit_fc*10,0,
											#endif
											#else
											0,-Roll*10,0,
											#endif
											-ctrl_2.err.y*10,-eso_att_outter[ROLr].disturb*10,0,
											(int16_t)(eso_att_outter_c[ROLr].disturb_u*100),(int16_t)(0*10.0),(int16_t)(0*100.0),0/10,0,0/10,0*0);
											else//INNER
											data_per_uart1(
											0,-except_AS.x,0,
											0,-mpu6050.Gyro_deg.x,0,  
											-ctrl_1.err.y,-eso_att_inner[ROLr].disturb,0,
											(int16_t)(eso_att_outter_c[ROLr].disturb_u*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0/10,0,0/10,0*0);
											}
										}
										else if(flow_debug.en_ble_debug||force_flow_ble_debug)//DEBUG  FLOW
											data_per_uart1(
											flow_debug.ax,flow_debug.ay,flow_debug.az,
										  flow_debug.gx,flow_debug.gy,flow_debug.gz,
										  flow_debug.hx,flow_debug.hy,flow_debug.hz,
											(int16_t)(inner_loop_time*10000.0),(int16_t)(outer_loop_time*10000.0),(int16_t)(0*10.0),0/10,0,0/10,0*0);
										else{//DEBUG-------------------------Normal mode--------------------------------
								    switch(UART_UP_LOAD_SEL)
											{
											case 0://BMP UKF
											data_per_uart1(
											ALT_POS_SONAR2*100,baroAlt_fc/10,ALT_POS_BMP_UKF_OLDX*100,
											ALT_VEL_BMP_UKF_OLDX*100,ALT_VEL_BMP_EKF*100,wz_speed/10,
											0*100,ALT_VEL_BMP_UKF_OLDX*100,0,
											//-accz_bmp*100,baro_matlab_data[1]/10,0*100,
											(int16_t)(Yaw_fc*10),(int16_t)(Pit_fc*10.0),(int16_t)(Rol_fc*10.0),thr_test,0,0/10,0);break;	
											case 1://GPS UKF
											data_per_uart1(
											0,imu_nav.gps.Y_UKF,0,
											0,imu_nav.gps.Y_O,0,  
											imu_nav.gps.J,0,0,
											(int16_t)(Yaw*10.0),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),0/10,0,0/10,0*0);break;	
											case 2://SONAR BMP SPEED
											data_per_uart1(
											0,ultra_speed/10,wz_speed/10,
											0,-ALT_VEL_BMP*100,0,  
											Yaw*10,0,0,
											(int16_t)(yaw_mag_view[0]*10.0),(int16_t)(yaw_mag_view[1]*10.0),(int16_t)(yaw_mag_view[2]*10),0/10,0,0/10,0*0);break;	
											case 4://ESO PID OUT
											data_per_uart1(
											0,ctrl_2.out.y*10,0,
											0,eso_att_outter_c[PITr].u*10,0,  
											0,eso_att_outter_c[PITr].disturb_u*10,0,
											(int16_t)(except_A.y*10.0),(int16_t)(eso_att_outter_c[PITr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;
											case 5://ESO PID IN
											data_per_uart1(
											0,ctrl_1.out.y*10,0,
											0,eso_att_inner_c[PITr].u*10,0,  
											0,eso_att_inner_c[PITr].disturb_u*10,0,
											(int16_t)(except_AS.y*10.0),(int16_t)(eso_att_inner_c[PITr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;												
											case 6://FLOW
											data_per_uart1(
											0,imu_nav.flow.speed.x*100,imu_nav.flow.speed.x_f*100,
											0,imu_nav.flow.speed.y*100,imu_nav.flow.speed.y_f*100,  
											0,imu_nav.flow.speed.x_f*100,imu_nav.flow.speed.y_f*100,
											(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),thr_test,0,0/10,0);break;
											case 7://SONAR AVOID
											data_per_uart1(
											sonar_avoid[0],sonar_avoid[1],sonar_avoid[2],
											sonar_avoid[3],sonar_avoid[4],sonar_avoid[5],  
											sonar_avoid[6],sonar_avoid[7],0,
											(int16_t)(sonar_avoid_c[0]),(int16_t)(sonar_avoid_c[1]),(int16_t)(0*10),0/10,0,0/10,0*0);break;		
											case 8://ESO PID HIGH IN
											data_per_uart1(
											0,wz_speed_pid_v_view,ultra_ctrl.err,
											0,eso_att_inner_c[THRr].u,0,  
											0,wz_speed_pid_v.pid_out,0,
											(int16_t)(ultra_ctrl_out_use*10.0),(int16_t)(eso_att_inner_c[THRr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;	
											case 9://CIRCLE
											data_per_uart1(
											0,nav_circle[0],nav_circle[1],
											0,eso_att_inner_c[THRr].u,0,  
											0,wz_speed_pid_v.pid_out,0,
											(int16_t)(ultra_ctrl_out_use*10.0),(int16_t)(eso_att_inner_c[THRr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;		
											case 10://EKF vs ARSH
											data_per_uart1(
											0,Pitch*10,fRPY[1]*10,
											0,Roll*10,fRPY[0]*10,  
											0,Yaw*10,fRPY[2]*10,
											(int16_t)(ultra_ctrl_out_use*10.0),(int16_t)(eso_att_inner_c[THRr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;		
											case 12://CIRCLE
											data_per_uart1(
											0,0,circle.x_flp-MID_X,
											0,0,circle.y_flp-MID_Y,  
											0,nav_circle[0]*10,nav_circle[1]*10,
											(int16_t)(fRPY[2]*10.0),(int16_t)(eso_att_inner_c[THRr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;
											case 13://H_inf vs PID
											data_per_uart1(
											0,0,ctrl_inf_att_out*10,
											0,0,ctrl_1.out.x,  
											0,0,ctrl_inf_att_out2*10,
											(int16_t)(fRPY[2]*10.0),(int16_t)(eso_att_inner_c[THRr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;
											case 14://SONAR_KAL
											data_per_uart1(
											0,exp_height,ultra_distance,
											0,0,ALT_POS_SONAR2*1000,  
											0,ultra_ctrl_out_use,ultra_speed,
											(int16_t)(fRPY[2]*10.0),(int16_t)(eso_att_inner_c[THRr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;
											case 16://SONAR
											data_per_uart1(
											0,0,ultra_distance,
											0,0,ALT_POS_SONAR3*1000,  
											0,ultra_dis_lpf,ALT_POS_SONAR2*1000,
											(int16_t)(fRPY[2]*10.0),(int16_t)(eso_att_inner_c[THRr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;
											case 17://FLOW_BREAK
											data_per_uart1(
											0,0,nav[ROLr]*10,
											0,0,nav[PITr]*10,  
											0,except_A.x*10,except_A.y*10,
											(int16_t)(fRPY[2]*10.0),(int16_t)(eso_att_inner_c[THRr].v1*10.0),(int16_t)(0*10),0/10,0,0/10,0*0);break;
											case 18://IMUVS
											data_per_uart1(
											0,0,fRPY[0]*10,
											0,0,fRPY[1]*10,  
											0,0,0,
											(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),thr_test,0,0/10,0);break;
											case 19://FLOW_POS
											data_per_uart1(
											0,0,target_position[LAT]*100,
											0,0,now_position[LAT]*100,
											0,0,0,
											(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),thr_test,0,0/10,0);break;
										  case 20://H_SPD 
											data_per_uart1(
											0,Rc_Get.PITCH,Rc_Get.ROLL,
											0,Rc_Get.THROTTLE,eso_att_inner_c[THRr].z[1],
											0,wz_speed_pid_v.pid_out,wz_speed_pid_v_view,
											(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),thr_test,0,0/10,0);break;
											case 21://TRIG PID TUNNING 
											data_per_uart1(
											0,flow_matlab_data[0]*1000,flow_matlab_data[1]*1000,
											0,flow_matlab_data[2]*1000,flow_matlab_data[3]*1000,
											0,0,0,
											(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),thr_test,0,0/10,0);break;
											case 22://TRIG PID TUNNING 
											data_per_uart1(
											0,avoid_trace[0]*1,flow_matlab_data[1]*0,
											0,avoid_trace[1]*1,flow_matlab_data[3]*0,
											0,0,0,
											(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),thr_test,0,0/10,0);break;
											case 23://TRIG PID TUNNING 
											data_per_uart1(
											0,0,height_ctrl_out,
											0,ultra_speed/10,ultra_ctrl_out_use/10,
											0,0,0,
											(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),thr_test,0,0/10,0);break;
											case 24://TRIG PID TUNNING 
											data_per_uart1(
											yaw_mag_view[4]*10,yaw_mag_view[0]*10,yaw_mag_view[1]*10,
											yaw_mag_view[3]*10,0,0,
											X_kf_yaw[0]*10,yaw_kf*10,0,
											(int16_t)(Yaw_fc*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),thr_test,0,0/10,0);break;
											case 25://TRIG PID TUNNING 
											data_per_uart1(
											0,0,VEL_UKF_Y*1000,
											0,0,VEL_UKF_X*1000,
											0,acc_body[1]*100,0,
											(int16_t)(Yaw*10),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),thr_test,0,0/10,0);break;
											case 26:
											data_per_uart1(
											ERR_GPS[0],ERR_GPS[1],0  ,
											HEIGHT,EX_HEIGHT,RC_OUT[2]  ,
											ALT_POS_SONAR2*1000,ALT_POS_SONAR_HEAD_LASER_SCANER,EX_AVOID,
										  m100_yaw*10,Pitch*10,Roll*10,0,state_fc*10,state_m100*10,m100_refresh*10000);
											break;
											default:break;
											}
										}
									}
										#endif
							USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����     
							MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE1+2);     //��ʼһ��DMA���䣡	  
							}	
						//}
				
						
				//To  SD��
						static u8 sd_sel;
				if(cnt[3]++>1){cnt[3]=0;
			     #if EN_DMA_UART4 			
					if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)
								{ 
							DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);
							
//							if(!mode.en_sd_save)
//							data_per_uart4(SEND_DEBUG);
//						  else
							switch(sd_sel){
							case 0:sd_sel=1;		
							data_per_uart4(SEND_IMU);
							break;
							case 1:sd_sel=2;
							data_per_uart4(SEND_ALT);
							break;
							case 2:sd_sel=3;
							data_per_uart4(SEND_FLOW);
							break;
							case 3:sd_sel=4;
							data_per_uart4(SEND_PID);
							break;
							case 4:sd_sel=0;
							data_per_uart4(SEND_DEBUG);
							
							}
							USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);    
							MYDMA_Enable(DMA1_Stream4,SEND_BUF_SIZE4+2);   
								}		
					#else
							SD_LINK_TASK2(SEND_IMU);	
					#endif
							}		
				
			delay_ms(5);  
			}
}	

//=======================���ϱ��� ������==================
OS_STK ERROR_TASK_STK[ERROR_STK_SIZE];
void error_task(void *pdata)
{							  
 	while(1)
	{
	LEDRGB();//LED��ʾ
	MEMS_CAL();//У׼IMUģ�鴫����
	Mode_FC();//�ɿ�ģʽ�л�	
		
	if(!fly_ready&&Thr_Low)//δʹ��   ʧ�ر����ж�
	{ ero.ero_rst_att=ero.ero_rst_h=1; ero.ero_att=ero.ero_hight=0;}
  else	
	{
   att_ero_check();
   hight_ero_check();
	}
	
	if(app_connect_fc_loss++>4/0.5)
	app_connect_fc=0;
	
	
	if(circle.lose_cnt++>4/0.5)
	circle.connect=0;
	if(ultra_pid.kp==0||mode.height_safe)
		mode.height_in_speed=1;
	else
		mode.height_in_speed=0;
	
	 delay_ms(500); 
	}
}	

//------------------------------�����ʱ��  δʹ��----------------------------//
OS_TMR   * tmr1;			//�����ʱ��1
OS_TMR   * tmr2;			//�����ʱ��2
OS_TMR   * tmr3;			//�����ʱ��3

//�����ʱ��1�Ļص�����	  OSCPUUsage
//ÿ100msִ��һ��,������ʾCPUʹ���ʺ��ڴ�ʹ����		
 u16 cpuusage=0;
void tmr1_callback(OS_TMR *ptmr,void *p_arg) 
{
	static u8 tcnt=0;	    

	if(tcnt==5)
	{
 		
		cpuusage=0;
		tcnt=0; 
	}
	cpuusage+=OSCPUUsage;
	tcnt++;				    
}

//�����ʱ��2�Ļص�����	��----------------------------	   
void tmr2_callback(OS_TMR *ptmr,void *p_arg) 
{	
static u8 cnt;
	
}

//�����ʱ��3�Ļص�����				  	   
void tmr3_callback(OS_TMR *ptmr,void *p_arg) 
{	
} 


//