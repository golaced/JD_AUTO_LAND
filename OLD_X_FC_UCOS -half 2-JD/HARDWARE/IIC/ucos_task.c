#include "include.h" 
#include "iic_soft.h"
#include "hml_sample.h"
#include "ms5611.h"
#include "ms5611_2.h"
#include "hml5833l.h"
#include "alt_kf.h"
#include "flash.h"
#include "led_fc.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "ucos_task.h"
#include "pwm_in.h"
#include "flow.h"
#include "circle.h"
#include "eso.h"
#include "gps.h"
#include "m100.h"
//==============================传感器 任务函数==========================
OS_STK MEMS_TASK_STK[MEMS_STK_SIZE];
void mems_task(void *pdata)
{		static u8 cnt,cnt1;						 
 	while(1)
	{
	MPU6050_Read(); 															//读取mpu6轴传感器
	MPU6050_Data_Prepare( 0.005 );			//mpu6轴传感器数据处理
	if(cnt++>1){cnt=0;ANO_AK8975_Read();}			//获取电子罗盘数据	
	MS5611_ThreadNew();
	delay_ms(5);
	}
}		
//--------------------

OS_STK INNER_TASK_STK[INNER_STK_SIZE];
u16 Rc_Pwm_In[8];
float PWM_DJ[4]={1615,1500,0};
float inner_loop_time;
float dj_k=0.0018*1.8,dj_k2=0.01015,dj_k_mouse=0.015;

//float k_dj[2][3]={1.6,0.8,3,  3.5,0.5,10};float d_t[2]={3,4.5};
float k_dj[2][3]={1,1.0,0.85,  1,1,0.85};float d_t[2]={0.45,0.45/2};float dt_flt=0.9;
float flt_track[3]={0.75,0.9,0.5};
float DJ_YAW_OFF=0;
int Rc_Pwm_off[8]={2,2,2,3};

float k_yaw_z=0.2;
float k_reset=10;
u8 state_v_test2=13;
float dead_pan_z=4;
float k_pan_z=0;//0.04;
u8 rc_thr_mid=0;
float flt_gro_z=0.8;
float Yaw_Follow_Dead= 25/2;
#if USE_M100
float SHOOT_PWM_OFF0=-66,
#else
float SHOOT_PWM_OFF0=70,
#endif
	SHOOT_PWM_OFF1=6,SHOOT_PWM_DEAD0=80,SHOOT_PWM_DEAD1=60;
#if USE_M100
int YUN_PER_OFF=50;
#else
int YUN_PER_OFF=40;
#endif
float T_SHOOT_CHECK=0.4;//2;
u16 DJ_TEST[3]={1523,1520,1500};
float set_angle_dj[3];
float kp_dj[3]={10,50,8},kd_dj[3],i_yaw;
float k_scan=0.25;//0.25;
u8 SCAN_RANGE=88;//116;
int flag_yun[2]={1,1};
u16 Rc_Pwm_Out_mine_USE[4];
float k_m100_gps[3]=  {2.26,2.26,0.66}; //p r t
float k_m100_down[3]=  {2.26,2.26,0.66}; //p r t
float k_m100_scan[3]= {2.26,2.26,0.66};
float k_m100_track[3]={2.26,0.88,0.66};
float k_m100_shoot[3]={2.26,0.88,0.66};
float k_m100_laser_avoid=0.3888;
float k_m100_yaw=1;
u8 en_yun_track=0;
u16  PWM_DJ0 = 1630;//1680//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1830) -  PWM_DJ[0] );  俯仰
u16	 PWM_DJ1 = 1500;//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1500) -  PWM_DJ[1] );  左右
void inner_task(void *pdata)
{NVIC_InitTypeDef NVIC_InitStructure;
 u8 i;
 static u8 dj_fly_line=0,cnt_20ms;
 static u8 init;	
 static int flag_scan=1;
 	while(1)
	{
	inner_loop_time = Get_Cycle_T(GET_T_INNER); 						//获取内环准确的执行周期
	RC_Duty( inner_loop_time , Rc_Pwm_In );	
	CTRL_1( inner_loop_time ); 										//内环角速度控制。输入：执行周期，期望角速度，测量角速度，角度前馈；输出：电机PWM占空比。<函数未封装>
	if(cnt_20ms++>4-1){cnt_20ms=0;
  GPS_Qr_Control(&gpsx,0.02);		
	}
	if(!init){init=1;	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PWMIN_P;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PWMIN_S;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PWMIN_P;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PWMIN_S;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	 
	}
	//------------------------------介入控制初始化---------------------------------------------------
	
	#if USE_M100
	Rc_Pwm_Inr_mine[RC_PITCH]=(float)LIMIT(m100.Rc_pit,-10000,10000)/10000.*500+1500;	
	Rc_Pwm_Inr_mine[RC_ROLL] =(float)LIMIT(m100.Rc_rol,-10000,10000)/10000.*500+1500;	
	Rc_Pwm_Inr_mine[RC_YAW]=  (float)LIMIT(m100.Rc_yaw,-10000,10000)/10000.*500+1500;	
	Rc_Pwm_Inr_mine[RC_THR]=  (float)LIMIT(m100.Rc_thr,-10000,10000)/10000.*500+1500;	
	Rc_Pwm_Inr_mine[RC_MODE]= (float)LIMIT(m100.Rc_mode,-10000,10000)/8000.*500+1500;	
	Rc_Pwm_Inr_mine[RC_GEAR]= (float)LIMIT(m100.Rc_gear,-10000,10000)/10000.*500+1500;	
	
	for(i=0;i<8;i++){	
	if(Rc_Pwm_Inr_mine[i]<=pwmin.max+200&&Rc_Pwm_Inr_mine[i]>=pwmin.min-200&&i!=RC_THR)
	Rc_Pwm_Out_mine[i]=0.5*Rc_Pwm_Out_mine[i]+0.5*Rc_Pwm_Inr_mine[i];
	}
	#else
	for(i=0;i<8;i++){	
	if(Rc_Pwm_In_mine[i]<=pwmin.max+200&&Rc_Pwm_In_mine[i]>=pwmin.min-200){
	if(i!=RC_THR)//avoid THR 
	Rc_Pwm_Out_mine[i]=0.5*Rc_Pwm_Out_mine[i]+0.5*Rc_Pwm_In_mine[i];
	Rc_Pwm_Inr_mine[i]=Rc_Pwm_In_mine[i];}
	}
	#endif
	//----------------------------------------------------------------------------------------------
	#if USE_M100
	#define MAX_NAV_RC 200//180
	#else
	#define MAX_NAV_RC 180//180
	#endif
	#define DEAD_NAV_RC 100
	#define USE_YAW 1
	#define USE_CIRCLE 1
	#if USE_CIRCLE
	int temp1=(int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT;
	int temp2=(int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL;
	int temp3=(int)Rc_Pwm_Inr_mine[RC_YAW]-OFF_RC_YAW;
	
	#if USE_M100
	
 if(state_v==SU_CHECK_TAR||state_v==SU_TO_START_POS||state_v==SD_TO_HOME)
	 {
	 k_m100[0]=k_m100_gps[0];
	 k_m100[1]=k_m100_gps[1];	 
	 }
	 else if(state_v==SU_TO_CHECK_POS)
	 {
	 k_m100[0]=k_m100_gps[0];
	 if(fabs(nav_Data.gps_ero_dis_lpf[1])>366*2)	 
	 k_m100[1]=k_m100_gps[1]*1.5;
   else	 
	 k_m100[1]=k_m100_gps[1];
	 } 
	 else if(state_v==SD_HOLD||state_v==SD_HOLD_BACK)
	 {
	 k_m100[0]=k_m100_scan[0];
	 if(fabs(nav_Data.gps_ero_dis_lpf[1])>366*2)	 
	 k_m100[1]=k_m100_scan[1]*1.5;	 
	 else
	 k_m100[1]=k_m100_scan[1]; 
	 	 }
	  else if(state_v==SD_HOLD2)
	 {
	 k_m100[0]=k_m100_track[0];
	 k_m100[1]=k_m100_track[1];	 
	 	 }
	 else if(state_v==SD_SHOOT)
	 {
	  k_m100[0]=k_m100_track[0];
		#if SHOOT_USE_YUN	 
		k_m100[1]=k_m100_track[1];
		#else
		if((fabs(ultra_ctrl_head.err1)<86)&&fabs(PWM_DJ[1]-PWM_DJ1)<25)	 
		k_m100[1]=k_m100_shoot[1];
		else
		k_m100[1]=k_m100_track[1];
		#endif	
	 	 }
	  else if(state_v==SD_CIRCLE_MID_DOWN)
	 {
	 k_m100[0]=k_m100_down[0]*0.8;
	 k_m100[1]=k_m100_down[1]*0.8;		 
	 } 
	 else
	 {
	 k_m100[0]=1;
	 k_m100[1]=1;
	 }
	#else
   k_m100[0]=1;
	 k_m100[1]=1;
	#endif
	
	
	
//对圆控制 
	if((fabs(temp1)<DEAD_NAV_RC)&&(fabs(temp2)<DEAD_NAV_RC)&&(fabs(temp3)<DEAD_NAV_RC))
	{ 
		#if USE_M100
		if(!mode.dj_by_hand&&!dji_rst_protect&&ALT_POS_SONAR2>0.3){
		#else
		if(!mode.dj_by_hand&&ALT_POS_SONAR2>0.3){//注意---开启
		#endif
		Rc_Pwm_Out_mine[RC_PITCH]=LIMIT(nav_land[PITr]*k_m100[0]+OFF_RC_PIT,OFF_RC_PIT-MAX_NAV_RC,OFF_RC_PIT+MAX_NAV_RC);//注意遥控偏执
		Rc_Pwm_Out_mine[RC_ROLL] =LIMIT(nav_land[ROLr]*k_m100[1]+OFF_RC_ROL,OFF_RC_ROL-MAX_NAV_RC,OFF_RC_ROL+MAX_NAV_RC);
		Rc_Pwm_Out_mine[RC_YAW]  =LIMIT(yaw_ctrl_out*k_m100_yaw+OFF_RC_YAW,OFF_RC_YAW-MAX_NAV_RC,OFF_RC_YAW+MAX_NAV_RC);	
		}
	}
	#endif
	// 云台控制
	static u8 dj_mode_reg;
	if((mode.dj_by_hand&&!dj_mode_reg))
	{
	 if(KEY[4])
   PWM_DJ[0]=1400;//PWM_DJ0;//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1250) -  PWM_DJ[0] );
	 else
	 PWM_DJ[0]=PWM_DJ0; 
	 PWM_DJ[1]=PWM_DJ1;//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1500) -  PWM_DJ[1] );
	}
	if(mouse.check)
	{
		PWM_DJ[0]+=0.02*my_deathzoom(mouse.y-120,5)*flag_yun[0];
		PWM_DJ[1]+=-0.01*my_deathzoom(mouse.x-160,5)*flag_yun[1];	
	}	
	else if(mode.dj_by_hand){

	if(Rc_Get.ROLL>1000)
	PWM_DJ[0]+=-dj_k*my_deathzoom(Rc_Get.ROLL-1500,80)*flag_yun[0];
	if(Rc_Get.PITCH>1000)
	PWM_DJ[1]+=-dj_k*my_deathzoom(Rc_Get.PITCH-1500,80)*flag_yun[1];	

	}
	else if(state_v==SD_CIRCLE_MID_DOWN&&0)//SU_CHECK_TAR)//下降对圆 垂直云台
	{
	 PWM_DJ[0]=PWM_DJ0;//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1830) -  PWM_DJ[0] );
	
   if(PWM_DJ[1]<PWM_DJ1-SCAN_RANGE)	
	 { flag_scan=1;PWM_DJ[1]=PWM_DJ1-SCAN_RANGE+5; }
   else if(PWM_DJ[1]>PWM_DJ1+SCAN_RANGE)	
	 { flag_scan=-1;PWM_DJ[1]=PWM_DJ1+SCAN_RANGE-5;}
	 PWM_DJ[1]+=flag_scan*k_scan*flag_yun[1];//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1500) -  PWM_DJ[1] );
	}
	else  if(state_v==SD_HOLD)
			{
	 PWM_DJ[0]=PWM_DJ0;//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1830) -  PWM_DJ[0] );
	 PWM_DJ[1]=PWM_DJ1-YUN_PER_OFF*mode.en_yun_per_off;//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1500) -  PWM_DJ[1] );
	}
	#if !SHOOT_USE_YUN
  else if(state_v==SD_SHOOT)
	{
	if((fabs(ultra_ctrl_head.err1)<86)&&fabs(PWM_DJ[1]-PWM_DJ1)<88)
   PWM_DJ[1]=PWM_DJ1;
	}
	#endif
		else if(state_v==SD_HOLD_BACK)
			{
	 PWM_DJ[0]=PWM_DJ0;//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1830) -  PWM_DJ[0] );
   PWM_DJ[1]=PWM_DJ1+YUN_PER_OFF*mode.en_yun_per_off;//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1500) -  PWM_DJ[1] );
	}
	else if(en_yun_track&&track.check&&circle.connect&&(state_v==SD_HOLD_BREAK||state_v==SD_HOLD2||state_v==SG_LOW_CHECK||state_v==SD_SHOOT||state_v==SD_CIRCLE_MID_DOWN)){
	float ero[2],ero2[2],ero3[2],ero4[2];
  static float ero_r[2],ero_r2[2],ero_r3[2];
  ero[0]=my_deathzoom_2(-circle.y+120,30);//,0.5,(float)68/1200);
	ero2[0]=my_deathzoom_2(-circle.y+120,20);//,0.5,(float)68/1200);
	ero3[0]=my_deathzoom_2(-circle.y+120,10);//,0.5,(float)68/1200);	
	ero4[0]=my_deathzoom_2(-circle.y+120,5);//,0.5,(float)68/1200);	
	PWM_DJ[0]+=-(dj_k2*ero[0]*track.control_k)*k_dj[0][0]*flag_yun[0];	//k=2.2
	PWM_DJ[0]+=-(dj_k2*ero2[0])*track.control_k*k_dj[0][1]*flag_yun[0];
	PWM_DJ[0]+=-(dj_k2*ero3[0])*track.control_k*k_dj[0][2]*flag_yun[0];	
	PWM_DJ[0]+=-(dj_k2*ero4[0])*track.control_k*k_dj[0][2]*flag_yun[0];		
	//if(fabs(-circle.y+120)<20)	
		
	PWM_DJ[0]+=-(ero_r2[0])*d_t[0]*flag_yun[0];
  ero_r2[0]=	(-circle.y+120-ero_r[0])*dt_flt+(1-dt_flt)*ero_r3[0]	;
  ero_r3[0]=ero_r2[0];
	ero_r[0]=-circle.y+120;
//-------------------------------------------------------------------//
	ero[1]=my_deathzoom_2(circle.x-160,35);//,0.5,(float)68/1200);
	ero2[1]=my_deathzoom_2(circle.x-160,25);//,0.5,(float)68/1200);
	ero3[1]=my_deathzoom_2(circle.x-160,15);//,0.5,(float)68/1200);	
	ero4[1]=my_deathzoom_2(circle.x-160,5);//,0.5,(float)68/1200);
	PWM_DJ[1]+=-(dj_k2*ero[1]*track.control_k_miss)*k_dj[1][0]*flag_yun[1];	//k=27
	PWM_DJ[1]+=-(dj_k2*ero2[1])*track.control_k_miss*k_dj[1][1]*flag_yun[1];
	PWM_DJ[1]+=-(dj_k2*ero3[1])*track.control_k_miss*k_dj[1][2]*flag_yun[1];
	PWM_DJ[1]+=-(dj_k2*ero4[1])*track.control_k_miss*k_dj[1][2]*flag_yun[1];
 // if(fabs(circle.x-160)<20)	
	PWM_DJ[1]+=-(ero_r2[1])*d_t[1]*flag_yun[1];
  ero_r2[1]=	(circle.x-160-ero_r[1])*dt_flt+(1-dt_flt)*ero_r3[1]	;
	ero_r3[1]=ero_r2[1];
	ero_r[1]=circle.x-160;
	}
	static float temp_gro_z;
	temp_gro_z=flt_gro_z*temp_gro_z+(1-flt_gro_z)*mpu6050.Gyro_deg.z;
	if((track.check&&circle.connect)&&!mode.dj_by_hand)
		PWM_DJ[1]+=-my_deathzoom(temp_gro_z,dead_pan_z)*k_pan_z;//yaw fix
	
	static u16 cnt_miss_track;
	if(!mode.dj_by_hand&&(state_v==SD_HOLD2||state_v==SD_SHOOT||state_v==SG_LOW_CHECK))//云台超时复位
	{
	if(cnt_miss_track++>6.5*2/0.005)//hold2
	{cnt_miss_track=0;
	 if(KEY[4])
   PWM_DJ[0]=1400;//PWM_DJ0;//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1250) -  PWM_DJ[0] );
	 else
	 PWM_DJ[0]=PWM_DJ0; 
	PWM_DJ[1]=PWM_DJ1;}
	else if(circle.check&&circle.connect)
	cnt_miss_track=0;
	}
	else
	cnt_miss_track=0;
	
	PWM_DJ[1]=PWM_DJ1;

	//PWM_DJ[0]=DJ_TEST[0];//Pitch_DJ
	//PWM_DJ[1]=DJ_TEST[1];//ROLL_DJ
	
	/*duoji*/
		float out[3];
	float ero_dj[3];
	static float ero_djr[3];
	
	set_angle_dj[0]=PWM_DJ[0];
	set_angle_dj[2]=PWM_DJ[1];
	
	
	ero_dj[0]=my_deathzoom(set_angle_dj[0]+Pitch_DJ,1);
	
	out[0]=ero_dj[0]*kp_dj[0]+(ero_dj[0]-ero_djr[0])*kd_dj[0];
	
	ero_dj[1]=my_deathzoom(set_angle_dj[1]+Roll,1);
	if(fabs(ero_dj[1])<10)
	out[1]=ero_dj[1]*kp_dj[1]+(ero_dj[1]-ero_djr[1])*kd_dj[1];
	else 	if(fabs(ero_dj[1])<20)
	out[1]=ero_dj[1]*kp_dj[1]*4+(ero_dj[1]-ero_djr[1])*kd_dj[1];
	else
	out[1]=ero_dj[1]*kp_dj[1]*8+(ero_dj[1]-ero_djr[1])*kd_dj[1];
	
	ero_dj[2]=my_deathzoom(set_angle_dj[2]-Yaw,1.5);
	static u16 cnt_dj_delay;
	if(cnt_dj_delay++<200){cnt_dj_delay=202;set_angle_dj[2]=Yaw;}
	//
	
	if(fabs(ero_dj[2])>20)set_angle_dj[2]=Yaw;
	
	static float int_dj;
	 int_dj+=i_yaw*ero_dj[2];
	out[2]=ero_dj[2]*kp_dj[2]+(ero_dj[2]-ero_djr[2])*kd_dj[2]+int_dj;
	
	
	#define YAW_PWM_RANGE 250
	PWM_DJ[0]=LIMIT(PWM_DJ[0],1020,1980);PWM_DJ[1]=LIMIT(PWM_DJ[1],1500-YAW_PWM_RANGE,1500+YAW_PWM_RANGE);
	Rc_Pwm_Out_mine[4]=(1-flt_track[1])*Rc_Pwm_Out_mine[4]+(flt_track[1])*PWM_DJ[0];//out put pit
//-----------------------
	
//	SHOOT_PWM_OFF1=((Rc_Get.AUX4-500)/10)*0.8;//云台起飞偏执

	
//	if(mode.en_yun_per_off)
//	{
////	  if(state_v==SD_HOLD)
////			Rc_Pwm_Out_mine[5]=(1-flt_track[1])*Rc_Pwm_Out_mine[5]+(flt_track[1])*PWM_DJ[1]+DJ_YAW_OFF+SHOOT_PWM_OFF1-YUN_PER_OFF;//out put yaw
////		else if(state_v==SD_HOLD_BACK)
////			Rc_Pwm_Out_mine[5]=(1-flt_track[1])*Rc_Pwm_Out_mine[5]+(flt_track[1])*PWM_DJ[1]+DJ_YAW_OFF+SHOOT_PWM_OFF1+YUN_PER_OFF;//out put yaw
////		else
//			Rc_Pwm_Out_mine[5]=(1-flt_track[1])*Rc_Pwm_Out_mine[5]+(flt_track[1])*PWM_DJ[1]+DJ_YAW_OFF+SHOOT_PWM_OFF1;//out put yaw
//	}	
//	else
	Rc_Pwm_Out_mine[5]=(1-flt_track[1])*Rc_Pwm_Out_mine[5]+(flt_track[1])*PWM_DJ[1]+DJ_YAW_OFF+SHOOT_PWM_OFF1;//out put yaw
	
		
//--------------------Yaw follow  
 #if SHOOT_USE_YUN
	if(fabs((int)PWM_DJ[1]-1500)<Yaw_Follow_Dead)
	{track.dj_fly_line=1;
	PWM_DJ[2]=LIMIT(-track.control_yaw*(my_deathzoom(PWM_DJ[1]-PWM_DJ1,0))+0,0-100,0+100);	
		}
	else{
	PWM_DJ[2]=LIMIT(-track.control_yaw*(my_deathzoom(PWM_DJ[1]-PWM_DJ1,0))+0,0-100,0+100);
		track.dj_fly_line=0;
	}
	#else
	if(
		#if !DEBUG_IN_ROOM
		(fabs(ultra_ctrl_head.err1)<86)&&
	  #endif
		fabs(PWM_DJ[1]-PWM_DJ1)<100)
	PWM_DJ[2]=LIMIT(track.control_yaw*my_deathzoom_2(circle.x-160,5)+0,0-100,0+100);
	else{
	if(fabs((int)PWM_DJ[1]-1500)<Yaw_Follow_Dead)
	{track.dj_fly_line=1;
	PWM_DJ[2]=LIMIT(-track.control_yaw*(my_deathzoom(PWM_DJ[1]-PWM_DJ1,0))+0,0-100,0+100);	
		}
	else{
	PWM_DJ[2]=LIMIT(-track.control_yaw*(my_deathzoom(PWM_DJ[1]-PWM_DJ1,0))+0,0-100,0+100);
		track.dj_fly_line=0;
	}}
	
	#endif
//---------------遥控器旋钮	
/*	
	SHOOT_PWM_DEAD1=Rc_Get.AUX2/10;//上下//0~1000
	SHOOT_PWM_DEAD0=Rc_Get.AUX1/10;//左右
	#if USE_M100
	SHOOT_PWM_OFF0=-((float)Rc_Get.AUX3)*2/10.;
	#else
	SHOOT_PWM_OFF0=-((float)Rc_Get.AUX3)/10.;
	#endif
*/
	//if(mode.dj_yaw_follow&&!mode.dj_by_hand&&fabs((int)Rc_Pwm_In_mine[RC_YAW]-1500)<DEAD_NAV_RC&&track.check&&circle.connect&&state_v==13)
	//Rc_Pwm_Out_mine[RC_YAW]=PWM_DJ[2];//(1-flt_track[2])*Rc_Pwm_Out_mine[RC_YAW]+(flt_track[2])*PWM_DJ[2];
	
	SetPwm(Rc_Pwm_Out_mine,Rc_Pwm_off,pwmin.min,pwmin.max);

	dj_mode_reg=mode.dj_by_hand;
	
	Rc_Pwm_Out_mine_USE[0]=Rc_Pwm_Out_mine[0];
	Rc_Pwm_Out_mine_USE[1]=Rc_Pwm_Out_mine[1];
	Rc_Pwm_Out_mine_USE[2]=Rc_Pwm_Out_mine[2];
	Rc_Pwm_Out_mine_USE[3]=Rc_Pwm_Out_mine[3];
	delay_ms(5);
	}
}		



//========================外环  任务函数============================
OS_STK OUTER_TASK_STK[OUTER_STK_SIZE];
float outer_loop_time;
float Pitch_R,Roll_R,Yaw_R;
void outer_task(void *pdata)
{	static u8 cnt,cnt1,cnt2;						  
 	while(1)
	{	
	outer_loop_time = Get_Cycle_T(GET_T_OUTTER);								//获取外环准确的执行周期
	/*IMU更新姿态。输入：半个执行周期，三轴陀螺仪数据（转换到度每秒），三轴加速度计数据（4096--1G）；输出：ROLPITYAW姿态角*/
 	IMUupdate(0.5f *outer_loop_time,mpu6050.Gyro_deg.x, mpu6050.Gyro_deg.y, mpu6050.Gyro_deg.z, mpu6050.Acc.x, mpu6050.Acc.y, mpu6050.Acc.z,&Roll_R,&Pitch_R,&Yaw_R);		
 	CTRL_2( outer_loop_time ); 														// 外环角度控制。输入：执行周期，期望角度（摇杆量），姿态角度；输出：期望角速度。<函数未封装>
  
  if(cnt1++>9){cnt1=0;GPS_calc_poshold(); 
	//circle_control(2*outer_loop_time);
	}
	#if USE_M100
	if(cnt2++>4-1){
	#else
	if(cnt2++>40-1){
	#endif	
		cnt2=0;
		//GPS_hold(&gpsx,0.2); 
		//GPS_Qr_Control(&gpsx,0.02);
	}
	delay_ms(5);
	}
}		
//=========================射频 任务函数======================
OS_STK NRF_TASK_STK[NRF_STK_SIZE];
u8 en_shoot=0;
void nrf_task(void *pdata)
{							 
	static u8 cnt,cnt2;
 	while(1)
	{
  	Nrf_Check_Event();
		CAL_CHECK();
		PWIN_CAL();
		//------------0 1   |   2 3  KEY_SEL
		
		mode.dj_by_hand=0;//!KEY_SEL[0];//手动控制云台
		mode.en_track_forward=1;//KEY_SEL[1];//使能侧飞
		mode.en_pid_sb_set=KEY_SEL[2];//使能PID设置	
		mode.test3=0;//前向壁障
		//mode.set_point1=KEY_SEL[3];//设置航点
		mode.en_hold2_h_off_fix=0;//=KEY_SEL[3];//上下修正高度
//		if(mode.en_gps)
//		mode.en_dji_h=0;//KEY_SEL[3];	
//		else
		mode.en_dji_h=1;
			//------------7 6 5 4  |  3 2 1 0  KEY
		mode.en_flow_break=0;
		mode.en_shoot= 0;//
		//mode.en_gps=KEY[7];//1
		
		
		//force_check_pass=KEY[7];//强制跳过图像判断部分程序  为无odroid gps巡航测试用
		//mode.auto_fly_up=KEY[5];

		mode.en_rth_mine=0;
		mode.en_yun_per_off=0;//侧飞预偏
		//mode.en_dji_yaw=KEY[4];
		mode.hold_use_flow=0;//KEY[4];//en lock height
		//Mag_CALIBRATED=KEY[3];//avoid with laser
		
		#if USE_M100
		//mode.use_qr_as_gps_tar=KEY_SEL[1];
	  mode.en_dji_yaw=0;//KEY[7];
		mode.en_dji_h=1;//KEY[7];//h
		mode.test3=0;//KEY_SEL[2];//avoid
		mode.en_flow_break=0;
		mode.en_gps=1;
		//force_check_pass=0;
		mode.hold_use_flow=0;		
		mode.en_rth_mine=0;//KEY[7];
		mode.en_qr_land=1;
		en_yun_track=0;//KEY_SEL[1];
		mode.en_land_avoid=KEY_SEL[1];
		mode.land_by_pix=1;//KEY_SEL[3];
		mode.qr_cal_by_px=1;//KEY_SEL[0];
		#endif
		
		//EN_SHOOT(en_shoot||KEY[2]);
			
		//-----
		#if  DEBUG_WITHOUT_SB
		if(cnt2++>150)//
		{fly_ready=1;cnt2=151;}
		#else
		if(Rc_Pwm_Inr_mine[RC_THR]>200)
		fly_ready=1;
		else
		fly_ready=0;//KEY_SEL[3];//解锁
		#endif
		delay_ms(20);
		if(cnt++>1){cnt=0;
		RC_Send_Task();}
	}
}		

//气压计 任务函数
OS_STK BARO_TASK_STK[BARO_STK_SIZE];
void baro_task(void *pdata)
{							  
 	while(1)
	{ static u8 cnt;
	  if(cnt++>1){cnt=0;
		altUkfProcess(0);}//add by gol _ukf brao
		delay_ms(10);  
	}
}	

//=======================超声波 任务函数==================
OS_STK SONAR_TASK_STK[SONAR_STK_SIZE];
void sonar_task(void *pdata)
{							  
 	while(1)
	{if(1){
		#if defined(SONAR_USE_TIG) 
		Ultra_Duty(); 
		#elif defined(SONAR_USE_UART) 
		Ultra_Duty();
		#elif defined(SONAR_USE_SCL)  
		Ultra_Duty_SCL(); 
		#endif
	}
	delay_ms(100);  
	}
}	

#include "AttitudeEKF.h"
//=======================光流 任务函数==================
OS_STK FLOW_TASK_STK[FLOW_STK_SIZE];
#define ACC_SPEED_NUM 25
u8 ACC_SPEED_NUM_USE=10;
u16 acc_cnt[2];
float acc_v[3];
float acc_speed_arr[ACC_SPEED_NUM + 1];
float wz_speed_flow[2];
float w_acc_spd=0.915;
float w_acc_fix=0.1;
void flow_task(void *pdata)
{	
 static float hc_speed_i[2],h_speed[2],wz_speed_0[2],tempacc_lpf[2];				
 float c_nb_dtb[3][3],a_br[3],tmp[3],acc[3];	
 	while(1)
	{
	c_nb_dtb[0][0]=q_nav[0]*q_nav[0]+q_nav[1]*q_nav[1]-q_nav[2]*q_nav[2]-q_nav[3]*q_nav[3];
	c_nb_dtb[0][1]=2*(q_nav[1]*q_nav[2]+q_nav[0]*q_nav[3]);
	c_nb_dtb[0][2]=2*(q_nav[1]*q_nav[3]-q_nav[0]*q_nav[2]);

	c_nb_dtb[1][0]=2*(q_nav[1]*q_nav[2]-q_nav[0]*q_nav[3]);
	c_nb_dtb[1][1]=q_nav[0]*q_nav[0]-q_nav[1]*q_nav[1]+q_nav[2]*q_nav[2]-q_nav[3]*q_nav[3];
	c_nb_dtb[1][2]=2*(q_nav[2]*q_nav[3]+q_nav[0]*q_nav[1]);

	c_nb_dtb[2][0]=2*(q_nav[1]*q_nav[3]+q_nav[0]*q_nav[2]);
	c_nb_dtb[2][1]=2*(q_nav[2]*q_nav[3]-q_nav[1]*q_nav[0]);
	c_nb_dtb[2][2]=q_nav[0]*q_nav[0]-q_nav[1]*q_nav[1]-q_nav[2]*q_nav[2]+q_nav[3]*q_nav[3];

	float sinP,cosP,sinR,cosR,sinY,cosY,pitch,roll;
	float acc_temp[2];
	a_br[0] =(float) mpu6050.Acc.x/4096.;//16438.;
	a_br[1] =(float) mpu6050.Acc.y/4096.;//16438.;
	a_br[2] =(float) mpu6050.Acc.z/4096.;//16438.;
	// acc
	tmp[0] = a_br[0];
	tmp[1] = a_br[1];
	tmp[2] = a_br[2];

	acc_temp[0]=tmp[0]*c_nb_dtb[0][0]+tmp[1]*c_nb_dtb[1][0]+tmp[2]*c_nb_dtb[2][0];
	acc_temp[1]=tmp[0]*c_nb_dtb[0][1]+tmp[1]*c_nb_dtb[1][1]+tmp[2]*c_nb_dtb[2][1];
	acc[0] =-((acc_temp[0] * cos(Yaw*0.0175) - acc_temp[1] * sin(Yaw*0.0175)));// / 10;
	acc[1] =-((acc_temp[1] * cos(Yaw*0.0175) + acc_temp[0] * sin(Yaw*0.0175)));/// 10;

				
	acc_v[0]=acc[0]=-((float)((int)((acc[0])*200)))/200*9.87;
	acc_v[1]=acc[1]=((float)((int)((acc[1])*200)))/200*9.87;
	acc_v[2]=acc[2]=-((float)((int)((acc[2]-1.0f)*200)))/200*9.87;	
			
		
	h_speed[0]=pixel_flow_x2;//h_speed是高度环传到速度环的实测高度方向速度【但可能是错误的，气压计速度不可靠】
	h_speed[1]=pixel_flow_y2;

	tempacc_lpf[0]= acc_v[1];//9800 *T;由于是+-4G共8G，65535/8g=8192 g，加速度，mms2毫米每平方秒
	if(fabs(tempacc_lpf[0])<0.050)tempacc_lpf[0]=0;//简单消除下噪声
	wz_speed_0[0] += tempacc_lpf[0] *0.01;//加速度计积分成速度
	tempacc_lpf[1]= acc_v[0];//9800 *T;由于是+-4G共8G，65535/8g=8192 g，加速度，mms2毫米每平方秒
	if(fabs(tempacc_lpf[1])<0.050)tempacc_lpf[1]=0;//简单消除下噪声
	wz_speed_0[1] += tempacc_lpf[1] *0.01;//加速度计积分成速度	

	wz_speed_0[0] = w_acc_spd	*wz_speed_0[0] + (1-w_acc_spd)*h_speed[0];//气压计垂直速度互补滤波，系数可调
	wz_speed_0[1] = w_acc_spd	*wz_speed_0[1] + (1-w_acc_spd)*h_speed[1];//气压计垂直速度互补滤波，系数可调

	hc_speed_i[0] += 0.4f *0.01 *( h_speed[0] - wz_speed_flow[0] );//速度偏差积分，乘以了0.4系数
	hc_speed_i[0] = LIMIT( hc_speed_i[0], -0.5, 0.5 );//积分限幅	
	hc_speed_i[1] += 0.4f *0.01 *( h_speed[1] - wz_speed_flow[1] );//速度偏差积分，乘以了0.4系数
	hc_speed_i[1] = LIMIT( hc_speed_i[1], -0.5, 0.5 );//积分限幅	
	wz_speed_0[0] += ( 1 / ( 1 + 1 / ( w_acc_fix *3.14f *0.01 ) ) ) *( h_speed[0]  - wz_speed_0[0]   ) ;//0.1实测速度修正加速度算的速度
	imu_nav.flow.speed.x=wz_speed_flow[0]=wz_speed_0[0]  + hc_speed_i[0] ;//经过修正的速度+经过限幅的增量式速度积分
	wz_speed_0[1] += ( 1 / ( 1 + 1 / ( w_acc_fix *3.14f *0.01 ) ) ) *( h_speed[1]  - wz_speed_0[1]   ) ;//0.1实测速度修正加速度算的速度
	imu_nav.flow.speed.y=wz_speed_flow[1]=wz_speed_0[1]  + hc_speed_i[1] ;//经过修正的速度+经过限幅的增量式速度积分
	delay_ms(5);
	}
}	
	
//=======================M100 任务函数==================
OS_STK M100_TASK_STK[M100_STK_SIZE];

u8 en_vrc;
u8 m100_control_mode = 0x4A;
float k_m100[5]={1,1,1,1,1};//pit rol thr yaw avoid
void m100_task(void *pdata)
{		
	static u8 cnt_m100;
		static u8 en_vrcr,flag1;
	static int m100_Rc_gr;
	
		while(1)
	{
	if(cnt_m100++>2-1){cnt_m100=0;
	m100_data(0);
	
		
	if(mode.auto_fly_up==1&&m100_Rc_gr==0)
	{
		m100_obtain_control_long(10);
		m100_take_off_long(10);
	}
  
	if(m100_Rc_gr==1&&mode.auto_fly_up==0)
	{m100_land_control_long(10);}
	m100_Rc_gr=mode.auto_fly_up;
	
  if(m100.Rc_gear<=-9000&&ALT_POS_SONAR2>0.32666)en_vrc=1;
  if(en_vrc&&m100.Rc_gear>-5000)
	en_vrc=0;
	if(en_vrc)
	{   		
	m100_obtain_control(10);
	if(mode.en_dji_yaw)
  m100_contrl(Rc_Pwm_Out_mine_USE[1],Rc_Pwm_Out_mine_USE[0],Rc_Pwm_Out_mine_USE[2],Rc_Pwm_Out_mine_USE[3],m100_control_mode); 
  else	
	m100_contrl(Rc_Pwm_Out_mine_USE[1],Rc_Pwm_Out_mine_USE[0],Rc_Pwm_Out_mine_USE[2],1500,m100_control_mode);
	delay_ms(20); }
	
	if(en_vrc==0&&en_vrcr==1)
		{m100_dis_control_long(5);}

		en_vrcr=en_vrc;
		
	if(dji_rst)
		m100_rst(10);
	}
	delay_ms(5);
	}
}

//=======================串口 任务函数===========================
OS_STK  UART_TASK_STK[UART_STK_SIZE];
u8 UART_UP_LOAD_SEL=4;//<------------------------------UART UPLOAD DATA SEL
u8 state_v_test=0;
u8 num_need_to_check;
void uart_task(void *pdata)
{	static u8 cnt[4];	
  static u8 sd_sel;	
 	while(1)
	{
				//GPS		
				if(cnt[0]++>1){cnt[0]=0;
						#if EN_DMA_UART3 
					if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)//等待DMA2_Steam7传输完成
								{ 
							DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//清除DMA2_Steam7传输完成标志
							data_per_uart3();
						  USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
							MYDMA_Enable(DMA1_Stream3,SEND_BUF_SIZE3);     //开始一次DMA传输！	
								}	
						#else
						#define NUM_OFF_STATE 50	
						#define UART_ODROID_SEL 0
						#if		UART_ODROID_SEL
						static u8 odroid_up_sel=0;
							if(odroid_up_sel){odroid_up_sel=0;
								if(state_v_test!=0)
								UsartSend_GPS(state_v_test+NUM_OFF_STATE);
								else
								UsartSend_GPS(state_v+NUM_OFF_STATE);
							}
							else
								{odroid_up_sel=1;
								UsartSend_GPS(num_need_to_check);}
						#else		
								CPU_LINK_TASK();//to lds.,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,xsodroid
						#endif		
						#endif
							}		
				
				//SD	
				if(cnt[1]++>1){cnt[1]=0;	
				  #if EN_DMA_UART2 					
					if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET)//等待DMA2_Steam7传输完成
								{ 
							DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);//清除DMA2_Steam7传输完成标志
							switch(sd_sel){
								case 0:sd_sel=1;		
							data_per_uart4(SEND_IMU);
								break;
								case 1:sd_sel=0;
							data_per_uart4(SEND_ALT);
								break;
							}
					    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
							MYDMA_Enable(DMA1_Stream6,SEND_BUF_SIZE4+2);     //开始一次DMA传输！	
								}	
					#else
							SD_LINK_TASK2(SEND_IMU);		// GOL_LINK_TASK();	
					#endif
							}					
							GOL_LINK_TASK();
				//UPLOAD			
				if(cnt[2]++>4){cnt[2]=0;			
					if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//等待DMA2_Steam7传输完成
							{ 	DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//清除DMA2_Steam7传输完成标志
									switch(UART_UP_LOAD_SEL)
											{
											case 0:
											data_per_uart1(
											0,baroAlt/10,(int16_t)(ALT_POS_BMP*100),
											0,-ALT_VEL_BMP*100,0,  
											-ALT_VEL_SONAR*100,0,0,
											(int16_t)(Yaw*10.0),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),0/10,0,0/10,0*0);break;	
											case 1:
											data_per_uart1(
											0,imu_nav.gps.Y_UKF,0,
											0,imu_nav.gps.Y_O,0,  
											imu_nav.gps.J,0,0,
											(int16_t)(Yaw*10.0),(int16_t)(Pitch*10.0),(int16_t)(Roll*10.0),0/10,0,0/10,0*0);break;	
											case 2:
											data_per_uart1(
											ultra_distance,ultra_speed/10,wz_speed/10,
											0,-ALT_VEL_BMP*100,0,  
											-ALT_VEL_SONAR*100,0,ALT_POS_SONAR2*1000,
											(int16_t)(thr_in_view*10.0),(int16_t)(thr_use*10.0),(int16_t)(Roll*10.0),0/10,0,0/10,0*0);break;	
											case 3://气压计速度
											data_per_uart1(
											(Rc_Pwm_In_mine[0]),(int16_t)(Rc_Pwm_Inr_mine[0]),(0),
											(Rc_Pwm_In_mine[1]),(int16_t)(Rc_Pwm_Inr_mine[1]),0 ,
											(Rc_Pwm_In_mine[2]),(int16_t)(Rc_Pwm_Inr_mine[2]),0,
											(int16_t)(thr_in_view*10.0),(int16_t)(thr_use*10.0),(int16_t)(Roll*10.0),0/10,0,0/10,0*0);break;	
											case 4:
											data_per_uart1(
											0,pixel_flow_x_pix,0,
											pixel_flow_x2*100,flow_compx,0,  
											wz_speed_flow[0]*100,0,0*1000,
											(int16_t)(thr_in_view*10.0),(int16_t)(thr_use*10.0),(int16_t)(Roll*10.0),0/10,0,0/10,0*0);break;												
											default:break;
											}		
							USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
							MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE1+2);     //开始一次DMA传输！	  
							}	
						}
				//FLOW
				if(cnt[3]++>1){cnt[3]=0;
			     #if EN_DMA_UART4 			
					if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)
								{ 
							DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);
							//data_per_uart2(SEND_ALT);
							USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);    
							MYDMA_Enable(DMA1_Stream4,SEND_BUF_SIZE2+2);   
								}		
					#else
							//SD_LINK_TASK2(SEND_ALT);	
					#endif
							}				
		delay_ms(5);  
	}
}	


//------------------------------软件定时器--------------------------------//
OS_TMR   * tmr1;			//软件定时器1
OS_TMR   * tmr2;			//软件定时器2
OS_TMR   * tmr3;			//软件定时器3

//软件定时器1的回调函数	
//每100ms执行一次,用于显示CPU使用率和内存使用率		
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
 #include "circle.h"
//软件定时器2的回调函数				  50ms	 
u8 dji_rst_protect;
u8 dji_rst;
u8 DJI_CONNECT;
u16 dji_miss_cnt;
u8 dji_rc_miss;
u16 dji_rc_miss_cnt;
void tmr2_callback(OS_TMR *ptmr,void *p_arg) 
{	
static u16 cnt_1,cnt_2;	
static u8 cnt;
	Mode_FC();
	LEDRGB_STATE();
	if(circle.lose_cnt++>4/0.05)
		circle.connect=0;
	if(mouse.lose_cnt++>2/0.05)
		mouse.connect=0;
	
	if(fabs(m100.Pit)>90||fabs(m100.Rol)>90)
	{cnt_1=0;dji_rst_protect=1;}
	
	if(cnt_1>3/0.05)
	{cnt_1=dji_rst_protect=0;}
	else if(dji_rst_protect)
		cnt_1++;
	
	if(dji_miss_cnt++>1/0.05)
	{DJI_CONNECT=0;dji_miss_cnt=65530;}
	if(m100.Rc_mode<-7000&&DJI_CONNECT)
	{cnt_2=0;dji_rst=1;}
	
	if(cnt_2>3/0.05)
	{cnt_2=dji_rst=0;}
	else if(dji_rst)
		cnt_2++;
	
	if(cnt_m100_data_refresh++>0.88/0.05)
	{	cnt_m100_data_refresh=65530;
	  m100_data_refresh=0;}
	
		
	if(dji_rc_miss_cnt++>1/0.05)
	{dji_rc_miss=1;dji_rc_miss_cnt=65530;}
	if(m100.Rc_gear!=0)
	{dji_rc_miss_cnt=0;dji_rc_miss=0;}	
}
//软件定时器3的回调函数				  	   
void tmr3_callback(OS_TMR *ptmr,void *p_arg) 
{	
 
} 


//