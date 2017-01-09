#include "height_ctrl.h"
#include "ultrasonic.h"
#include "my_math.h"
#include "filter.h"
#include "att.h"
#include "ms5611.h"
#include "rc.h"
#include "alt_kf.h"
#include  "pwm_in.h"
#include  "circle.h"
#if PLANE_IS_BIG
#define HOLD_THR 600 
#define ALT_HOLD_THR_RANGE_SCALE 2
#else  ///use now
#define HOLD_THR 420 // 5030 M=600g/310g*4=0.48 *1.1=520 | 6030 M=600g/410*4=0.34 *1.1= 0.37  PA 570
#define ALT_HOLD_THR_RANGE_SCALE LIMIT((1000-HOLD_THR)/250,1,2.5)//PA 250
#endif

#define EXP_Z_SPEED  ( 0.4f *my_deathzoom( (thr-500), 50 ) )
float exp_spd_zv,thr_use,thr_in_view;
_st_height_pid_v wz_speed_pid_v;
_st_height_pid_v wz_speed_pid_v_safe;
_st_height_pid_v wz_speed_pid_v_eso;

_st_height_pid_v ultra_ctrl,ultra_ctrl_safe,ultra_ctrl_head;

_st_height_pid wz_speed_pid,wz_speed_pid_safe;
_st_height_pid ultra_pid_safe,ultra_pid,ultra_pid_head;
 

#define MAX_HEIGH_ERO 800  //check
float exp_height_speed;
float exp_height=3500,//起飞高度
	    exp_height_check=3500,//下降检测的高度
	   	exp_height_front=3500,//导航到第一次看到的高度
			exp_height_back=3500,//not use,
	exp_height_shoot_off;
float exp_height_speed_safe,exp_height_safe;
float ultra_speed,ultra_speed_safe;
float ultra_dis_lpf,ultra_dis_lpf_safe;
float ultra_ctrl_out_safe, ultra_ctrl_out,ultra_ctrl_out_head;
float baro_speed;
float height_ctrl_out,height_ctrl_out_head;
float wz_acc;
float adrc_out;
void Ultra_PID_Init()
{//use 
	#if USE_M100
	ultra_pid.kp = 0.8;//0.45;//0.45;//1.8;//1.65;//1.5;   WT
	ultra_pid.ki = 0.25;//1;//101;//add
	ultra_pid.kd = 6.666;//4.0;//0;
	
	#else
	ultra_pid.kp = 0.8;//0.45;//0.45;//1.8;//1.65;//1.5;   WT
	ultra_pid.ki = 0.25;//1;//101;//add
	ultra_pid.kd = 9;//4.0;//0;
	#endif
	#if USE_M100
	ultra_pid_head.kp = 0.1;//0.03;//1.8;//1.65;//1.5;   WT
	ultra_pid_head.ki = 0.05;//0.450;//1;//101;//add
	ultra_pid_head.kd = 6;//2.5;//0;
	#else
	ultra_pid_head.kp = 0.06;//0.03;//1.8;//1.65;//1.5;   WT
	ultra_pid_head.ki = 0.02;//0.450;//1;//101;//add
	ultra_pid_head.kd = 8;//2.5;//0;
	#endif
	//
  ultra_pid_safe.kp = 50.0;//1.8;//1.65;//1.5;
	ultra_pid_safe.ki = 0.0;//1;//add
	ultra_pid_safe.kd = 0;
}

  
void WZ_Speed_PID_Init()
{//use
	wz_speed_pid.kp = 0.4;//0.5;//0.3; 
	wz_speed_pid.ki = 0.5; 
	wz_speed_pid.kd = 1.5; 
	//
	wz_speed_pid_safe.kp = 120.4;//0.5;//0.3; 
	wz_speed_pid_safe.ki = 45.5; 
	wz_speed_pid_safe.kd = 1.0; 
}

#define BARO_SPEED_NUM 10
float baro_speed_arr[BARO_SPEED_NUM + 1];
u16 baro_cnt;
u8 switch_r;
int wz_acc_ukf;
void height_mode_switch(void)
{
static u8 state;
static u16 cnt,cnt1;	

 if(height_ctrl_mode==2){
	 
	 switch(state)
			{ 
			case 0:
			if(ultra_ok==1){
			if(ALT_POS_SONAR*1000>ULTRA_MAX_HEIGHT)
			{state=1;height_ctrl_mode_use=1;cnt=0;}
			else
				height_ctrl_mode_use=2;
			}
			else
			height_ctrl_mode_use=1;
			break;
			case 1:
			if(ALT_POS_SONAR<ULTRA_MAX_HEIGHT-100)
			cnt++;
			else
			cnt=0;

			if(cnt>800)
			{state=0;height_ctrl_mode_use=2;cnt=0;}

			break;
			}

}
 else
 {	height_ctrl_mode_use=height_ctrl_mode;state=0;}
 height_ctrl_mode_use=height_ctrl_mode;
}
u8 cnt_height=19;
float out_timer_high,in_timer_high;
void Height_Ctrl(float T,float thr)
{			static u8 hs_ctrl_cnt;
	static float wz_speed_t;
	static u8 height_ctrl_start_f,height_mode_reg;
	static u16 hc_start_delay;
	float t_sonar_out;
	float temp;
	
	switch( height_ctrl_start_f )
	{		
		case 0:
		temp=(reference_vr[2] *mpu6050.Acc.z + reference_vr[0] *mpu6050.Acc.x + reference_vr[1 ] *mpu6050.Acc.y - 4096  );
		wz_acc_ukf += ( 1 / ( 1 + 1 / ( 20 *3.14f *T ) ) ) *my_deathzoom( (temp - wz_acc_ukf),25 );
		wz_acc += ( 1 / ( 1 + 1 / ( 20 *3.14f *T ) ) ) *my_deathzoom( (temp - wz_acc),100);//100 );
		
	if(mode.en_h_mode_switch)
				height_mode_switch();
	height_ctrl_mode_use=2;
		 if( height_ctrl_mode_use!=0)
		{ 
	
			hs_ctrl_cnt++;
		  hs_ctrl_cnt = hs_ctrl_cnt%10;
			if(hs_ctrl_cnt == 0)
			{  //----------------------mode switch----------------------
				in_timer_high=Get_Cycle_T(GET_T_HIGH_CONTROL_I);
//				if(height_mode_reg==1&&height_ctrl_mode_use==2)//SONAR<-BMP
//				{exp_height=ultra_dis_lpf=ALT_POS_SONAR*1000;}
//				else if(height_mode_reg==2&&height_ctrl_mode_use==1)//SONAR->BMP
//				{exp_height=ultra_dis_lpf=ALT_POS_BMP*1000;}
//				 height_mode_reg=height_ctrl_mode_use;
				//---------------------safe height--------------------------------
//				static u8 safe_mode_reg;
//				 if(safe_mode_reg==0&&mode.height_safe==1)//SONAR->BMP
//				{exp_height_safe=ultra_dis_lpf_safe=ALT_POS_BMP*1000;}
//				else if(safe_mode_reg==1&&mode.height_safe==0)
//				{
//				if(height_ctrl_mode_use==2)//SONAR<-BMP
//				{exp_height=ultra_dis_lpf=ALT_POS_SONAR*1000;}
//				else if(height_ctrl_mode_use==1)//SONAR->BMP
//				{exp_height=ultra_dis_lpf=ALT_POS_BMP*1000;}
//				
//				}safe_mode_reg=mode.height_safe;
//				//heigh thr test
				static u8 state_thr,cnt_down;
				static float thr_reg;
				thr_in_view=thr;
				if(mode.thr_fix_test)//WT
				{  
					switch(state_thr)
					{
						case 0:
							if(thr>525)
							{state_thr=1;thr_reg=500;cnt_down=0;}
							else
							{thr_use=thr;}	
						break;
						case 1:
							if(thr>525){
								if(thr<thr_reg*0.85)
									cnt_down++;
								else
								{thr_use=thr;cnt_down=0;}
								if(cnt_down>3)state_thr=2;
								if(thr>=thr_reg)
								 thr_reg=thr;
							}
							else
								state_thr=0;
							
							
							break;
						case 2:
							thr_use=500;
							if(thr<550)
								state_thr=0;
						break;
					}
				}
				else
				{thr_use=thr;state_thr=0;thr_reg=500;}
				 //--------------------------------------------------
					//	adrc_out=ADRC( exp_height,ultra_dis_lpf, adrc_out,0.02,500)  ;          // v是控制系统的输入，y是控制系统的输出，反馈给ESO，u是ADRC的输出控制量
						
//				 applyMultirotorAltHold(thr_use,0.02);
				 exp_spd_zv=EXP_Z_SPEED;
//				 if(KEY[6])
//					 height_speed_ctrl(in_timer_high,thr_use,0.4*ultra_ctrl_out,ultra_speed);	
//				 else
						{if(EXP_Z_SPEED!=0){
				    	height_speed_ctrl(in_timer_high,thr_use,( EXP_Z_SPEED ),ultra_speed);//baro_alt_speed *10);///
						//	height_speed_ctrl_Safe(0.02f,thr,EXP_Z_SPEED,ultra_speed_safe); 
						}
							else{
							height_speed_ctrl(in_timer_high,thr_use,0.4*ultra_ctrl_out,ultra_speed);	
						//	height_speed_ctrl_Safe(0.02f,thr,0.4*ultra_ctrl_out_safe,ultra_speed_safe);
							}
						}
					
			}//---end of speed control
			static u8 cnt_100ms;
			if( cnt_100ms++>=cnt_height )//wt
			{
	      out_timer_high=Get_Cycle_T(GET_T_HIGH_CONTROL_O);
				cnt_100ms=0;

				Ultra_Ctrl(out_timer_high,thr_use);//超声波周期100ms
				Ultra_Ctrl_Front(out_timer_high,thr_use);
				//Ultra_Ctrl_Safe(0.1f,thr);//超声波周期100ms
				ultra_start_f = -1;
			}
		}

			if(height_ctrl_mode_use)//定高模式
		{	
			if(mode.height_safe)
			height_ctrl_out =  wz_speed_pid_v_safe.pid_out;//LIMIT(thr-50*LIMIT(Moving_Median(9,5,wz_acc_ukf)/4096,-0.2,0.2)*9.8,0,500);	
			else
			height_ctrl_out = wz_speed_pid_v.pid_out;
		}
		else//手动模式
		{		
		  	height_ctrl_out = thr-50*LIMIT(wz_acc_ukf/4096,-0.2,0.2)*9.8;   
		}
		
		break; //case 1
		
		default: break;
		
	} //switch
}


///test 
float p1=0.4,p2=0.1;//0.35;//0.3;//WT
u8 speed_ctrl_sel=0;
float wz_speed,wz_speed_old,wz_acc_mms2;
float height_thrv;
void height_speed_ctrl(float T,float thr,float exp_z_speed,float h_speed)
{
static float thr_lpf;
float height_thr;
static float lpf_tmp,hc_speed_i,hc_speed_i_2,wz_speed_0,wz_speed_1,wz_speed_2,hc_acc_i;
	
	if(thr<100)wz_speed_pid_v.err_i=0;
	height_thr = LIMIT( ALT_HOLD_THR_RANGE_SCALE * thr , 0, HOLD_THR );
	thr_lpf += ( 1 / ( 1 + 1 / ( 2.0f *3.14f *T ) ) ) *( height_thr - thr_lpf );
switch(speed_ctrl_sel){
	case 0:	
//=================================================
	wz_acc_mms2 = (wz_acc/4096.0f) *9800 + hc_acc_i;//9800 *T;
	wz_speed_0 += my_deathzoom( (wz_acc_mms2 ) ,50) *T;
	wz_speed_0 = 0.99	*wz_speed_0 + 0.01*h_speed;
	hc_acc_i += p1 *T *(h_speed - wz_speed);
	hc_acc_i = LIMIT( hc_acc_i, -500, 500 );	
	wz_speed_0 += ( 1 / ( 1 + 1 / ( p2 *3.14f *T ) ) ) *( h_speed - wz_speed_0  ) ;
	wz_speed=wz_speed_0 + hc_acc_i;
	if( ABS( wz_speed ) < 50 )
	{
		wz_speed = 0;
	}
	break;
	case 1:
//=================================================	
	
	wz_acc_mms2 = (wz_acc/4096.0f) *9800 ;//9800 *T;
	wz_speed_0 += my_deathzoom( wz_acc_mms2 ,100) *T;
	
	hc_speed_i += 0.4f *T *( h_speed - wz_speed_1 );
	hc_speed_i = LIMIT( hc_speed_i, -1500, 1500 );	
	
	wz_speed_0 += ( 1 / ( 1 + 1 / ( 0.4f *3.14f *T ) ) ) *( h_speed - wz_speed_0  ) ;
	wz_speed_1 = wz_speed_0 + hc_speed_i;
	
	if( ABS( wz_speed_1 ) < 50 )
	{
		wz_speed_1 = 0;
	}
	
	wz_speed_2 += my_deathzoom( wz_acc_mms2 ,100) *T;
	

		lpf_tmp += 0.4f *T *( wz_speed_1 - wz_speed ); 
		lpf_tmp = LIMIT( lpf_tmp, -1500, 1500 ); 

	hc_speed_i_2 += 0.01f *T *( wz_speed_1 - wz_speed_2 ); 
	hc_speed_i_2 = LIMIT( hc_speed_i_2, -500, 500 );	
	
	wz_speed_2 += ( 1 / ( 1 + 1 / ( 0.1f *3.14f *T ) ) ) *( wz_speed_1 - wz_speed_2 + hc_speed_i_2 ) ;//*(baro_speed - wz_speed);
	wz_speed = wz_speed_2 + lpf_tmp;
	break;
}
//===============================	
 //	if(KEY[6])
		 wz_speed=my_deathzoom_2( (h_speed) ,50);
  
	//--------------------------------------------------------------------------------
	wz_speed_pid_v.err = wz_speed_pid.kp *( exp_z_speed - wz_speed );
	//wz_speed_pid_v.err_d = 0.002f/T *10*wz_speed_pid.kd * (-wz_acc_mms2) *T;//(wz_speed_pid_v_safe.err - wz_speed_pid_v_safe.err_old);
	wz_speed_pid_v.err_d = wz_speed_pid.kd * (-wz_acc_mms2) *T;
	wz_speed_pid_v.err_i += wz_speed_pid.ki *( exp_z_speed - h_speed ) *T;//*wz_speed_pid.kp 
	wz_speed_pid_v.err_i = LIMIT(wz_speed_pid_v.err_i,-Thr_Weight *200,Thr_Weight *200);//-100~100 //(WT)
//	if(KEY[5])
//	wz_speed_pid_v.pid_out = thr_lpf + Thr_Weight *LIMIT(( wz_speed_pid.kp*0.0 *LIMIT(exp_z_speed,-300,300)+wz_speed_pid_v.err + wz_speed_pid_v.err_d + wz_speed_pid_v.err_i),-400,400);	
//	else
	wz_speed_pid_v.pid_out = thr_lpf + Thr_Weight *LIMIT(( LIMIT(wz_speed_pid.kp *exp_z_speed,-300,300)+wz_speed_pid_v.err + wz_speed_pid_v.err_d + wz_speed_pid_v.err_i),-400,400);
	wz_speed_pid_v.err_old = wz_speed_pid_v.err; 
	wz_speed_old=wz_speed;

}

u8 baro_ctrl_start;
float baro_height,baro_height_old;

float ultra_sp_test[2];

#define EXP_Z_SPEED_RC  ( 0.4f *my_deathzoom( (Rc_Pwm_Inr_mine[RC_THR]-OFF_RC_THR), 50 ) )
int RC_THR_HIGH;
#include "eso.h"
#include "circle.h"
u16 Pitch_Follow_Dead=40;
u16 exp_height1=2000;
void Ultra_Ctrl(float T,float thr)//定高  程序《------------------
{
float ultra_sp_tmp,ultra_dis_tmp;	
	#define MID_THR 500 //摇杆中位PWM
if((state_v==SD_HOLD2||state_v==SD_SHOOT)&&!mode.dj_by_hand&&mode.en_hold2_h_off_fix)
	{
	if(circle.connect&&circle.check)
	exp_height_shoot_off+=-circle.control_k*LIMIT(my_deathzoom(PWM_DJ[0]-(PWM_DJ0+SHOOT_PWM_OFF0),Pitch_Follow_Dead),-80,80)*1;	
	exp_height_shoot_off=LIMIT(exp_height_shoot_off,-400,400);	
  }
else
  exp_height_shoot_off=0;	
	static float off_m100;
 // if(Rc_Pwm_Inr_mine[RC_THR]>1150) 
   //ultra_distance=ALT_POS_SONAR_HEAD*1000/2;
	//ultra_distance=LIMIT(m100.H-off_m100,0,20*1000)*1000;
 static float sonar_ground;
	if(state_v==SG_LOW_CHECK)
	{off_m100=m100.H;
	sonar_ground=ALT_POS_SONAR2;
	}

	//模式转换初始化
	if(mode_change){mode_change=0;
	//exp_height=ALT_POS_SONAR2*1000;
	Flow_reset_pos();
	}	
	
//	exp_height1=600; 
	//ultra_dis_tmp=  ALT_POS_SONAR2*1000;//ultra_distance;
	ultra_dis_tmp=  LIMIT(m100.H-off_m100+sonar_ground,0,20)*1000;	

	ultra_dis_lpf=  ultra_dis_tmp;
		
	if(ultra_pid.ki==0||((Rc_Pwm_Inr_mine[RC_THR]<200+1000))||Rc_Pwm_Inr_mine[RC_THR]<450+1000||Rc_Pwm_Inr_mine[RC_THR]>550+1000||pwmin.sel==0)ultra_ctrl.err_i=0;

	ultra_ctrl.err = ( ultra_pid.kp*LIMIT(my_deathzoom(exp_height - ultra_dis_lpf,25),-400,400) );

	ultra_ctrl.err_i += ultra_pid.ki *ultra_ctrl.err *T;

	ultra_ctrl.err_i = LIMIT(ultra_ctrl.err_i,-1 *ULTRA_INT,1 *ULTRA_INT);

	ultra_ctrl.err_d = ultra_pid.kd *( 0.0f *(-(ALT_VEL_BMP_EKF*1000)*T) + 1.0f *(ultra_ctrl.err - ultra_ctrl.err_old) );
	
	if(mode.en_eso_h_in&&!mode.height_safe){
	HIGH_CONTROL_SPD_ESO(&eso_att_inner_c[THRr],(exp_height),(ultra_dis_lpf),eso_att_inner_c[THRr].u,T,1000);
	ultra_ctrl.pid_out = eso_att_inner_c[THRr].u  + ultra_ctrl.err_d;	 
	}
	else{
	ultra_ctrl.pid_out = ultra_ctrl.err + ultra_ctrl.err_i + ultra_ctrl.err_d;
   }
  //if(S_head>20)
	if(m100_data_refresh)
	ultra_ctrl.pid_out = LIMIT(ultra_ctrl.pid_out,-1000,1000);
	else
	ultra_ctrl.pid_out=0;	
	ultra_ctrl_out = ultra_ctrl.pid_out;//控制输出
	
	ultra_ctrl.err_old = ultra_ctrl.err;
}

#define DEAD_NAV_RC 80
#if USE_M100
float exp_height_head=1800+300,exp_height_head_scan=2150+400,exp_height_head_shoot=1800+350,exp_height_head_check=4666;
#else
float exp_height_head=1800,exp_height_head_scan=2150,exp_height_head_shoot=1800,exp_height_head_check=1800;
#endif
void Ultra_Ctrl_Front(float T,float thr)
{
float ultra_sp_tmp,ultra_dis_tmp;	
u16 exp_height_head_use;


  if(state_v==SD_HOLD||state_v==SD_HOLD_BACK)  
		exp_height_head=exp_height_head_scan;
	else if(state_v==SU_CHECK_TAR)
	  exp_height_head=exp_height_head_check;
	else if(state_v==SU_TO_START_POS)
		exp_height_head=exp_height_head_scan+250;
	else
		exp_height_head=exp_height_head_shoot;
	
	
	
	if(mode.test2)
	ultra_dis_tmp=ALT_POS_SONAR_HEAD_LASER_SCANER*1000;
	else
	ultra_dis_tmp=ALT_POS_SONAR_HEAD*1000;//ultra_distance;
			
	if(ultra_pid_head.ki==0
			||fabs((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)>DEAD_NAV_RC
			||fabs((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)>DEAD_NAV_RC
			||(mode.test3==0&&mode.test2==0))
	ultra_ctrl_head.err_i=0;
 
	ultra_ctrl_head.err = ( ultra_pid_head.kp*LIMIT(exp_height_head - ultra_dis_tmp,-500,500) );	
	ultra_ctrl_head.err1 = ( LIMIT(exp_height_head - ultra_dis_tmp,-500,500) );	// state change use
					
	#if USE_M100		
	if(fabs(exp_height_head - ultra_dis_tmp)>200)		
	ultra_ctrl_head.err_i=0;
	#endif			

	ultra_ctrl_head.err_i += ultra_pid_head.ki *ultra_ctrl_head.err *T;

	ultra_ctrl_head.err_i = LIMIT(ultra_ctrl_head.err_i,-1 *ULTRA_INT,1 *ULTRA_INT);

	ultra_ctrl_head.err_d = ultra_pid_head.kd *((ultra_ctrl_head.err - ultra_ctrl_head.err_old) );
	
	ultra_ctrl_head.pid_out = ultra_ctrl_head.err + ultra_ctrl_head.err_i + ultra_ctrl_head.err_d;

	ultra_ctrl_head.pid_out = LIMIT(ultra_ctrl_head.pid_out,-1000,1000);
		
	if(ultra_dis_tmp<6666&&S_head>20)		
	ultra_ctrl_out_head = -LIMIT(ultra_ctrl_head.pid_out*0.4,-100,80);
	else
	ultra_ctrl_out_head=0;	
	ultra_ctrl_head.err_old = ultra_ctrl_head.err;
}
