
#include "att.h"
#include "height_ctrl.h"
#include "ultrasonic.h"
#include "flash.h"
#include "pwm_out.h"
#include "pwm_in.h"
#include "alt_kf.h"
#include "circle.h"
ctrl_t ctrl_1;
ctrl_t ctrl_2;
ctrl_t ctrl_2_fuzzy;
// Calculate nav_lat and nav_lon from the x and y error and the speed
#define NAV_ANGLE_MAX 0.8*MAX_CTRL_ANGLE
float dj_angle,dj_angle_offset[3]={-2,3,9},dj_angle_set;
#define MAX_FIX_ANGLE_DJ 13
void AUTO_LAND_FLYUP(float T);
void DJ_offset_save(void)
{
//static u8 moder;

//if(mode.en_dj_cal)
//{
//dj_angle_offset[0]=(float)(Rc_Get.AUX1-500)/1000.*MAX_FIX_ANGLE_DJ*2;
//dj_angle_offset[1]=(float)(Rc_Get.AUX2-500)/1000.*MAX_FIX_ANGLE_DJ*2;
//dj_angle_offset[2]=(float)(Rc_Get.AUX3-500)/1000.*MAX_FIX_ANGLE_DJ*4;
//}
//else if(mode.en_dj_cal==0&&moder==1)
//	WRITE_PARM();
//moder=mode.en_dj_cal;
}


float nav[2];
float GPS_angle[2];
float  target_position[2],target_position_task_s[2],target_position_task_e[2],target_position_task_b[2];
float  now_position[2];
float actual_speed[2];
float tar_speed[2],tar_speed_avoidc[2];
float d_flow_watch[2];
float  integrator[2];
float flow_control_out,flow_k=30;

void Flow_reset_pos(void)
{
	integrator[0]=integrator[1]=0;
	target_position[LON]=now_position[LON];//imu_nav.flow.position.east;
	target_position[LAT]=now_position[LAT];//imu_nav.flow.position.west;
}
#define  TEST_MOVE_RANGE_FLOW 5
void Flow_save_tar_s(void)
{
	target_position_task_s[LON]=now_position[LON];//imu_nav.flow.position.east;
	target_position_task_s[LAT]=now_position[LAT];//imu_nav.flow.position.west;
	
	target_position_task_e[LON]=target_position_task_s[LON]-TEST_MOVE_RANGE_FLOW;//imu_nav.flow.position.east;
	target_position_task_e[LAT]=target_position_task_s[LAT]-TEST_MOVE_RANGE_FLOW;//imu_nav.flow.position.west;
}

void Flow_save_tar_b(void)
{
	target_position_task_b[LON]=now_position[LON];//imu_nav.flow.position.east;
	target_position_task_b[LAT]=now_position[LAT];//imu_nav.flow.position.west;
	
}

void Flow_set_tar(float set)
{
	target_position[1]=set;//imu_nav.flow.position.east;
}

void GPS_calc_poshold(void)//  
{
	float p, i, d,f_p;
	float output;
	float target_speed;
	static 	float last_derivative[2];
	float derivative[2];
	static int32_t last_error[2];
	int axis;
	float error_pos[2], rate_error[2];
	float cos_yaw,sin_yaw;	
	static u8  state[2]; 	
			/*
		north    LAT=1     V_West+                             __________
		|   Y+  y                                              P- R- GPS-
		                                        
		|P+
	
    _____	R	+   x         LON=0  V_East+
	

	   
head  |    1 PIT y-   90d in marker
			| 
		   _____  0 ROL x+
			 
			 
	
		*/
		
		
		
		float p_in_use;
		if(Rc_Pwm_Inr_mine[6]<1600)//姿态和GPS模式感度不同
			p_in_use=pid.nav.in.p;
		else
			p_in_use=pid.nav.in.p/2;
		
		
		//xf 是左右运动
		if(pid.nav.in.i==0||mode.en_flow_hold==0||circle.check)
		{integrator[1]=integrator[0]=0;}
	if(mode.flow_hold_position_use_global){	
		 actual_speed[LON]=my_deathzoom_2(imu_nav.flow.speed.east,pid.nav.in.dead)*10;//imu_nav.flow.speed.x;//mm
	   actual_speed[LAT]=my_deathzoom_2(imu_nav.flow.speed.west,pid.nav.in.dead)*10;//imu_nav.flow.speed.y;
		 //now_position[LON]=imu_nav.flow.position.east;
	   //now_position[LAT]=imu_nav.flow.position.west;
	}
	else
	{ if(!mode.flow_sel){
   	 actual_speed[LON]=my_deathzoom_2(imu_nav.flow.speed.y_f,pid.nav.in.dead)*10;//imu_nav.flow.speed.x;//mm
	   actual_speed[LAT]=my_deathzoom_2(imu_nav.flow.speed.x_f,pid.nav.in.dead)*10;//imu_nav.flow.speed.y;
		//if(ultra_dis_lpf>650)
		//	{
		 //pid.nav.in.dead2=10*pid.nav.in.dead;
	  // now_position[LON]+=my_deathzoom_2(imu_nav.flow.speed.y_f,pid.nav.in.dead2)*T;
	   //now_position[LAT]+=my_deathzoom_2(imu_nav.flow.speed.x_f,pid.nav.in.dead2)*T;
	}
	else
	{
	   actual_speed[LON]=my_deathzoom_2(imu_nav.flow.speed.y,pid.nav.in.dead)*10;//imu_nav.flow.speed.x;//mm
	   actual_speed[LAT]=my_deathzoom_2(imu_nav.flow.speed.x,pid.nav.in.dead)*10;//imu_nav.flow.speed.y;
		//if(ultra_dis_lpf>650)
		//	{
		 //pid.nav.in.dead2=10*pid.nav.in.dead;
	   //now_position[LON]+=my_deathzoom_2(imu_nav.flow.speed.east,pid.nav.in.dead2)*T;
	   //now_position[LAT]+=my_deathzoom_2(imu_nav.flow.speed.west,pid.nav.in.dead2)*T;
	}
		//}
		 //pid.nav.in.p=0.55  
	}
	
	
	
	//--------------marker hold  position
	/*head  |  vy  + 1   90d in marker LAT
	    		| 
		       _____   vx  +0            LON
			 
			 */

	  //
	//---------------flow hold position
	float now_position_temp[2];  
	now_position_temp[0]=now_position[0];
	now_position_temp[1]=now_position[1];

	

	
   for (axis = 0; axis < 2; axis++) {
			error_pos[axis]=LIMIT(my_deathzoom_2(target_position[axis]-now_position[axis],pid.nav.out.dead),-15,15);//tar-now //m
			 
				
			switch(axis){
			case 0:
			tar_speed[0]=-error_pos[0]*pid.nav.out.p;  break;
			case 1:
			tar_speed[1]=-error_pos[1]*pid.nav.out.p; break;
			}
			
			switch(axis){
				case 0:
				 rate_error[0] = tar_speed[0] - actual_speed[0];   // calc the speed error
				case 1:
				 rate_error[1] = tar_speed[1] - actual_speed[1];   // calc the speed error
			}
       // rate_error[axis] = tar_speed[axis] - actual_speed[axis];   // calc the speed error
       //-----------rad
        p = rate_error[axis]*p_in_use;//pid.nav.in.p;
			 	 
			 static float reg_ero_pos[2];
				 
			  f_p = (error_pos[axis]-reg_ero_pos[axis])*pid.nav.out.i;
			  reg_ero_pos[axis]= error_pos[axis];
	     if((fabs(CH_filter[ROLr])<100)&&(fabs(CH_filter[PITr])<100)&&fabs(Pitch)<15&&fabs(Roll)<15)
			  integrator[axis] += ((float) my_deathzoom_2(rate_error[axis],pid.nav.in.dead*1.5)* pid.nav.in.i);// *DT;

       i = LIMIT(integrator[axis],-MAX_FIX_ANGLE,MAX_FIX_ANGLE) ;
		 
		
		 derivative[axis] = (rate_error[axis] - last_error[axis]) ;/// DT;
    // discrete low pass filter, cuts out the
    // high frequency noise that can drive the controller crazy
    // derivative[axis] = last_derivative[axis] + (DT / (AC_PID_FILTER + DT)) * (derivative[axis] - last_derivative[axis]);
		 derivative[axis]=0.3* derivative[axis]+ 0.7*last_derivative[axis];
    // update state
    last_error[axis] = rate_error[axis] ;
    last_derivative[axis]= derivative[axis];
    // add in derivative component
       d_flow_watch[axis]= d = derivative[axis]*pid.nav.in.d;//d
		
        //d = constrain_int16(d, -2000, 2000);
        // get rid of noise
        //if (fabs(actual_speed[axis]) < pid.nav.in.dead)
         //   d = 0;
        output =f_p+ p + i + d + tar_speed[axis]*pid.nav.out.d;

        GPS_angle[axis] = limit_mine(output,NAV_ANGLE_MAX);//导航控制角度
     
    }
	float out_temp[2];
		if(mode.flow_hold_position_use_global){	
		cos_yaw=cos(Yaw*0.017);
		sin_yaw=sin(Yaw*0.017);
			out_temp[PITr] = -(GPS_angle[LAT] * cos_yaw + GPS_angle[LON] * sin_yaw);// / 10;
			out_temp[ROLr] = +(GPS_angle[LON] * cos_yaw - GPS_angle[LAT] * sin_yaw);/// 10;
		}
		else{
			out_temp[PITr] = -GPS_angle[LAT];
			out_temp[ROLr] = +GPS_angle[LON];
		}
		

		
		 limit_mine( (out_temp[PITr]) ,NAV_ANGLE_MAX );
		 limit_mine( (out_temp[ROLr]) ,NAV_ANGLE_MAX );

	nav[ROLr]=out_temp[ROLr];
	nav[PITr]=out_temp[PITr];
	
		/*
		north    LAT=1     V_West+                             __________
		|   Y+  y                                              P- R- GPS-
		                                        
		|P+
	
    _____	R	+   x         LON=0  V_East+
	   
head  |    1 PIT y-  RC0
			| 
		   _____  0 ROL x+   RC 1
	
		*/
		flow_control_out=-nav[1]*flow_k;
}




void Ctrl_Para_Init()		//设置默认参数
{
//====================================
	ctrl_1.PID[PIDROLL].kdamp  = 1;
	ctrl_1.PID[PIDPITCH].kdamp = 1;
	ctrl_1.PID[PIDYAW].kdamp 	 = 1;
	
	ctrl_1.FB = 0.20;   //内环  0<fb<1
}

xyz_f_t except_A = {0,0,0};
xyz_f_t except_AR = {0,0,0};
xyz_f_t ctrl_angle_offset = {0,0,0};

xyz_f_t compensation;
#define YAW_ERO_MAX 25
float YAW_NO_HEAD;	
float except_A_SB[2],except_A_SB_lft[2],nav_angle[2];
float scale_lf_sb=5.5;//感度
float scale_lf_nav=5;
float px_v,ix_v;
float yaw_ctrl_out,Yaw_set_dji=0;//178;
void CTRL_2(float T)//角度环
{ float px,py,ix,iy,d;
	static xyz_f_t acc_no_g;
	static xyz_f_t acc_no_g_lpf;
	float nav_angle_lft[2]={0,0};
  float dj_temp;
	static float dj_sb,dj_angle_set_out;
  static u8 flag_yaw_out=0;
  float cos1,sin1;
	float temp,temp_yaw;
	static u8 no_head;
/*   head  |    1 PIT y-    AUX2
	         | 
	         _____  0 ROL x+   AUX1
	
 ROL= 0,
 PIT=1
	*/
//=========================== 期望角度 ========================================
	 except_A_SB_lft[PITr] = -MAX_CTRL_ANGLE  *( my_deathzoom( (CH_filter[ROLr]) ,30 )/500.0f );  
	 except_A_SB_lft[ROLr] =  MAX_CTRL_ANGLE  *( my_deathzoom( (CH_filter[PITr]) ,30 )/500.0f );  
	
	
	 except_A_SB[ROLr]  += scale_lf_sb *T *3.14f * ( except_A_SB_lft[ROLr] - except_A_SB[ROLr] );
//---------------------------NAV_angle------------------------------------	
if(!mode.dj_lock)	{
	if(mode.flow_hold_position&&((fabs(except_A_SB[PITr])<2)&&(fabs(except_A_SB[ROLr])<2))
			&&NAV_BOARD_CONNECT==1)//add by gol 2015.10.22
			{
				if(ALT_POS_SONAR2>0.3){
					if(mode.test1)
				nav_angle_lft[PITr]=nav[PITr];//  -nav_ukf_g[PIT];//nav[PIT];
				nav_angle_lft[ROLr]=nav[ROLr];// nav_ukf_g[ROL];//nav[ROL];	
				}
			}
}
else {//DJ mode
	if(mode.flow_hold_position&&
			((fabs(dj_sb)<2)&&(fabs(except_A_SB[ROLr])<2))
			&&NAV_BOARD_CONNECT==1)//add by gol 2015.10.22
			{
			nav_angle_lft[PITr]=nav[PITr];
			nav_angle_lft[ROLr]=nav[ROLr];	 
			}
}
//	//scale_lf_nav=pid.nav.out.i;
//	if(mode.sb_smooth){
	nav_angle[PITr] += scale_lf_nav *T *3.14f * ( nav_angle_lft[PITr] - nav_angle[PITr] );
	nav_angle[ROLr] += scale_lf_nav *T *3.14f * ( nav_angle_lft[ROLr] - nav_angle[ROLr] );
//	}
//	else{
//	nav_angle[PIT] = nav_angle_lft[PIT] ;
//	nav_angle[ROL] = nav_angle_lft[ROL];
//	}
	//-----------------ATT_PROTECTOR----------------------------------------
	if(fabs(Pitch)>MAX_CTRL_ANGLE*1.5||fabs(Roll)>MAX_CTRL_ANGLE*1.5||ultra_distance<400)
	dj_angle_set=0;
	
	
	//---------DJ_CONTROL
	if(mode.dj_lock){
		dj_sb  += 6 *T *3.14f * ( except_A_SB_lft[PITr] - dj_sb );//fix by gol 2015.11.8
	
	  except_A_SB[PITr]  += 6 *T *3.14f * ( dj_angle_set - except_A_SB[PITr] );//fix by gol 2015.11.8
		if(fabs(Pitch-dj_angle_set)>0.6)
		dj_angle_set_out+= 10 *T *3.14f * ( LIMIT(Pitch,-SCALE_DJ*MAX_DJ_ANGLE,SCALE_DJ*MAX_DJ_ANGLE) - dj_angle_set_out);//fix by gol 2015.11.8  
		dj_temp=-dj_sb+dj_angle_set_out-nav_angle[PITr];
	}
	else
	{ 
		except_A_SB[PITr]  += scale_lf_sb *T*3.14f * ( except_A_SB_lft[PITr] - except_A_SB[PITr] );//fix by gol 2015.11.8
		dj_angle_set_out=0;dj_temp=0;
	}
	
	//--------------
	if(Rc_Pwm_Inr_mine[RC_PITCH]<OFF_RC_PIT-80||Rc_Pwm_Inr_mine[RC_PITCH]>OFF_RC_PIT+80||
		Rc_Pwm_Inr_mine[RC_ROLL]<OFF_RC_ROL-80||Rc_Pwm_Inr_mine[RC_ROLL]>OFF_RC_ROL+80||
	  (circle.check&&circle.connect&&state_v==SD_HOLD2))//光流复位
	{
		Flow_reset_pos();
	}
	
//--------------------------超声波 壁障----------------------------------------------
//	if(mode.sonar_avoid&&((fabs(except_A_SB_lft[PIT])<2)&&(fabs(except_A_SB_lft[ROL])<2))&&	(ultra_dis_lpf>800))//&&need_avoid)
//	{
//	//target_position[LON]=tar_position_avoid[LON];//imu_nav.flow.position.east;
//	//target_position[LAT]=tar_position_avoid[LAT];//imu_nav.flow.position.west;
//		tar_speed_avoidc[LON]=tar_speed_avoid[LON];
//		tar_speed_avoidc[LAT]=tar_speed_avoid[LAT];
//	}
//	else
//	{
//		tar_speed_avoidc[LON]=0;
//		tar_speed_avoidc[LAT]=0;
//	}

	
	/*YAW -180~180     -90  135
		-	 0  + 
			 |
			 |
		-	180 +
	*/
//----------------------------------no head mode -------------------------------
	
	switch(no_head)//add by gol 11.8 (WT)
	{
		case 0:if(Thr_Low == 0 && ultra_distance>350)
		{no_head=1;YAW_NO_HEAD=Yaw;}
		else
			{
			except_AR.x=except_A_SB[ROLr];
			except_AR.y=except_A_SB[PITr];	
			}
		break;
		case 1:
			if(mode.no_head)
			{ 
			temp=Yaw-YAW_NO_HEAD;
			cos1=cos(temp*0.017);
			sin1=sin(temp*0.017);
			except_AR.x=except_A_SB[ROLr]*cos1+except_A_SB[PITr]*sin1;
			except_AR.y=except_A_SB[PITr]*cos1-except_A_SB[ROLr]*sin1;
			}	
			else
			{
			except_AR.x=except_A_SB[ROLr];
			except_AR.y=except_A_SB[PITr];	
			}
			if(Thr_Low == 1 && ultra_distance<200)
				no_head=0;
			break;
	}
//------------------------angle output	--------------------------------------------
	
		//DJ Smooth
	
	dj_angle += ( 1 / ( 1 + 1 / ( 6 *3.14f*0.02 ) ) ) *((dj_temp)- dj_angle) ;
	
	DJ_offset_save();
	//DJ Out
//	Set_DJ(LIMIT(dj_angle,-MAX_DJ_ANGLE,MAX_DJ_ANGLE)+dj_angle_offset[0],
//	LIMIT(Pitch,-90,90),
//	-LIMIT(Roll,-90,90)+dj_angle_offset[2],
//	-LIMIT(dj_angle,-MAX_DJ_ANGLE,MAX_DJ_ANGLE)-dj_angle_offset[1]
//	);
	//---------END DJ

	except_A.x=limit_mine(nav_angle[ROLr]+except_AR.x,MAX_CTRL_ANGLE);	
	if(mode.dj_lock)
	except_A.y=limit_mine(except_AR.y,MAX_CTRL_ANGLE);	
	else
	except_A.y=limit_mine(nav_angle[PITr]+except_AR.y,MAX_CTRL_ANGLE);
	
	
	//--------------------------IMU Control Yaw
	 if( Thr_Low == 0 )//fix by gol 2015.11.7 && ultra_distance>150)
	{
		if((flag_yaw_out==2&&CH_filter[YAWr]>100)||(flag_yaw_out==1&&CH_filter[YAWr]<-100)||flag_yaw_out==0)
		except_AR.z += (s16)( MAX_CTRL_YAW_SPEED *( my_deathzoom_2( (CH_filter[YAWr]) ,40 )/500.0f ) ) *T ;  //50
	}
	else
	{
		except_AR.z += 1 *3.14 *T *( Yaw - except_AR.z );
	}
  
	if(Yaw_set_dji!=0)
			except_A.z  = To_180_degrees(Yaw_set_dji);
	//else if(Rc_Pwm_Inr_mine[RC_THR]<1000+200)
	//	except_A.z  = To_180_degrees(Yaw);
	
 // except_A.z  = To_180_degrees(Yaw_set_dji);//---------------------
  /* 得到角度误差 */
	ctrl_2.err.x =  my_deathzoom_2(To_180_degrees( ctrl_angle_offset.x + except_A.x - Roll  ),0.1);
	ctrl_2.err.y =  my_deathzoom_2(To_180_degrees( ctrl_angle_offset.y + except_A.y - Pitch ),0.1);
	
	// except_A.z  = To_180_degrees(Yaw_set_dji);//---------------------
	if(mode.en_dji_yaw&&ALT_POS_SONAR2>0.3&&fabs(Rc_Pwm_Inr_mine[RC_YAW]-OFF_RC_YAW)<80)
	#if  USE_M100
  ctrl_2.err.z =  my_deathzoom_2(LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - m100.Yaw),-YAW_ERO_MAX,YAW_ERO_MAX),0.1);
  #else	
	ctrl_2.err.z =  my_deathzoom_2(LIMIT(To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw	 ),-YAW_ERO_MAX,YAW_ERO_MAX),0.1);
	#endif
	else
	{ctrl_2.err.z =0;ctrl_2.eliminate_I.z=0;}	
//	ctrl_2.PID[PIDROLL].kd=0;
	//------------------YAW PROTECTOR--------------------------------
//	if(fabs(To_180_degrees(except_A.z - Yaw))>80){
//		except_A.z  = To_180_degrees(Yaw);
//	  ctrl_2.err.z=0;
//	}
	
  if((ctrl_2.err.z)>=YAW_ERO_MAX)
		flag_yaw_out=1;
	else  if((ctrl_2.err.z)<=-YAW_ERO_MAX)
		flag_yaw_out=2;
	else
		flag_yaw_out=0;


px=py=ctrl_2.PID[PIDROLL].kp;
ix=iy=ctrl_2.PID[PIDROLL].ki;
d=ctrl_2.PID[PIDROLL].kd;	

//#if USE_MY_PWM_OUT
	/* 计算角度误差权重 */
	ctrl_2.err_weight.x = ABS(ctrl_2.err.x)/ANGLE_TO_MAX_AS;
	ctrl_2.err_weight.y = ABS(ctrl_2.err.y)/ANGLE_TO_MAX_AS;
	ctrl_2.err_weight.z = ABS(ctrl_2.err.z)/ANGLE_TO_MAX_AS;
	/* 角度误差微分（跟随误差曲线变化）*/
	ctrl_2.err_d.x = 10 *d *(ctrl_2.err.x - ctrl_2.err_old.x) *( 0.005f/T ) *( 0.65f + 0.35f *ctrl_2.err_weight.x );
	ctrl_2.err_d.y = 10 *d *(ctrl_2.err.y - ctrl_2.err_old.y) *( 0.005f/T ) *( 0.65f + 0.35f *ctrl_2.err_weight.y );
	ctrl_2.err_d.z = 10 *ctrl_2.PID[PIDYAW].kd 	 *(ctrl_2.err.z - ctrl_2.err_old.z) *( 0.005f/T ) *( 0.65f + 0.35f *ctrl_2.err_weight.z );
	/* 角度误差积分 */
	ctrl_2.err_i.x +=ix  *ctrl_2.err.x *T;
	ctrl_2.err_i.y +=iy  *ctrl_2.err.y *T;
	ctrl_2.err_i.z += ctrl_2.PID[PIDYAW].ki 	*ctrl_2.err.z *T;
	/* 角度误差积分分离 */
	ctrl_2.eliminate_I.x = Thr_Weight *CTRL_2_INT_LIMIT;
	ctrl_2.eliminate_I.y = Thr_Weight *CTRL_2_INT_LIMIT;
	ctrl_2.eliminate_I.z = 0;//1 *CTRL_2_INT_LIMIT;
	/* 角度误差积分限幅 */
	ctrl_2.err_i.x = LIMIT( ctrl_2.err_i.x, -ctrl_2.eliminate_I.x,ctrl_2.eliminate_I.x );
	ctrl_2.err_i.y = LIMIT( ctrl_2.err_i.y, -ctrl_2.eliminate_I.y,ctrl_2.eliminate_I.y );
	ctrl_2.err_i.z = LIMIT( ctrl_2.err_i.z, -ctrl_2.eliminate_I.z,ctrl_2.eliminate_I.z );
	/* 对用于计算比例项输出的角度误差限幅 */
	ctrl_2.err.x = LIMIT( ctrl_2.err.x, -90, 90 );
	ctrl_2.err.y = LIMIT( ctrl_2.err.y, -90, 90 );
	ctrl_2.err.z = LIMIT( ctrl_2.err.z, -25, 25 );
	/* 角度PID输出 */
	ctrl_2.out.x = px  *( ctrl_2.err.x + ctrl_2.err_d.x + ctrl_2.err_i.x );	//rol
	ctrl_2.out.y = py  *( ctrl_2.err.y + ctrl_2.err_d.y + ctrl_2.err_i.y );  //pit
	ctrl_2.out.z = ctrl_2.PID[PIDYAW].kp   *( ctrl_2.err.z + ctrl_2.err_d.z + ctrl_2.err_i.z );
	
	if(fabs(ctrl_2.err.z)<10)
	yaw_ctrl_out=LIMIT(ctrl_2.out.z*track.forward_end_dj_pwm*15,-50,50) ;
	else if(ctrl_2.err.z>0)
	{yaw_ctrl_out=50;ctrl_2.eliminate_I.z=0;}
  else	
	{yaw_ctrl_out=-50 ;ctrl_2.eliminate_I.z=0;}	
	/* 记录历史数据 */	
	ctrl_2.err_old.x = ctrl_2.err.x;
	ctrl_2.err_old.y = ctrl_2.err.y;
	ctrl_2.err_old.z = ctrl_2.err.z;
//#endif

}

xyz_f_t except_AS;

float g_old[7];
 
void CTRL_1(float T)  //x roll,y pitch,z yaw 角速度  内环  2ms
{float ctrl_angle_out[3],ctrl_angle_weight[3];
	xyz_f_t EXP_LPF_TMP;
	

//	ctrl_angle_out[0]=ctrl_2.out.x;
//	ctrl_angle_out[1]=ctrl_2.out.y;
//	ctrl_angle_out[2]=ctrl_2.out.z;
//	ctrl_angle_weight[0]=ctrl_2.err_weight.x;
//	ctrl_angle_weight[1]=ctrl_2.err_weight.y;
//	ctrl_angle_weight[2]=ctrl_2.err_weight.z;
//	
//#if USE_MY_PWM_OUT	
//	/* 给期望（目标）角速度 */
//	EXP_LPF_TMP.x = MAX_CTRL_ASPEED *(ctrl_angle_out[0]/ANGLE_TO_MAX_AS);//*( (CH_filter[0])/500.0f );//
//	EXP_LPF_TMP.y = MAX_CTRL_ASPEED *(ctrl_angle_out[1]/ANGLE_TO_MAX_AS);//*( (CH_filter[1])/500.0f );//
//	EXP_LPF_TMP.z = MAX_CTRL_ASPEED *(ctrl_angle_out[2]/ANGLE_TO_MAX_AS);
//	
//	except_AS.x = EXP_LPF_TMP.x;//20 *3.14 *T *( EXP_LPF_TMP.x - except_AS.x );//
//	except_AS.y = EXP_LPF_TMP.y;//20 *3.14 *T *( EXP_LPF_TMP.y - except_AS.y );//
//	except_AS.z = EXP_LPF_TMP.z;//20 *3.14 *T *( EXP_LPF_TMP.z - except_AS.z );//
//	/* 期望角速度限幅 */
//	except_AS.x = LIMIT(except_AS.x, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );
//	except_AS.y = LIMIT(except_AS.y, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );
//	except_AS.z = LIMIT(except_AS.z, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );

//	/* 角速度直接微分（角加速度），负反馈可形成角速度的阻尼（阻碍角速度的变化）*/
//	ctrl_1.damp.x = ( mpu6050.Gyro_deg.x - g_old[A_X]) *( 0.002f/T );//ctrl_1.PID[PIDROLL].kdamp
//	ctrl_1.damp.y = (-mpu6050.Gyro_deg.y - g_old[A_Y]) *( 0.002f/T );//ctrl_1.PID[PIDPITCH].kdamp *
//	ctrl_1.damp.z = (-mpu6050.Gyro_deg.z - g_old[A_Z]) *( 0.002f/T );//ctrl_1.PID[PIDYAW].kdamp	 *
//	/* 角速度误差 */
//	ctrl_1.err.x =  ( except_AS.x - mpu6050.Gyro_deg.x ) *(300.0f/MAX_CTRL_ASPEED);
//	ctrl_1.err.y =  ( except_AS.y + mpu6050.Gyro_deg.y ) *(300.0f/MAX_CTRL_ASPEED);  //-y
//	ctrl_1.err.z =  ( except_AS.z + mpu6050.Gyro_deg.z ) *(300.0f/MAX_CTRL_ASPEED);	 //-z
//	/* 角速度误差权重 */
//	ctrl_1.err_weight.x = ABS(ctrl_1.err.x)/MAX_CTRL_ASPEED;
//	ctrl_1.err_weight.y = ABS(ctrl_1.err.y)/MAX_CTRL_ASPEED;
//	ctrl_1.err_weight.z = ABS(ctrl_1.err.z)/MAX_CTRL_YAW_SPEED;
//	/* 角速度微分 */
//	ctrl_1.err_d.x = ( ctrl_1.PID[PIDROLL].kd  *( -10 *ctrl_1.damp.x) *( 0.002f/T ) );
//	ctrl_1.err_d.y = ( ctrl_1.PID[PIDPITCH].kd *( -10 *ctrl_1.damp.y) *( 0.002f/T ) );
//	ctrl_1.err_d.z = ( ctrl_1.PID[PIDYAW].kd   *( -10 *ctrl_1.damp.z) *( 0.002f/T ) );

////	ctrl_1.err_d.x += 40 *3.14 *0.002 *( 10 *ctrl_1.PID[PIDROLL].kd  *(ctrl_1.err.x - ctrl_1.err_old.x) *( 0.002f/T ) - ctrl_1.err_d.x);
////	ctrl_1.err_d.y += 40 *3.14 *0.002 *( 10 *ctrl_1.PID[PIDPITCH].kd *(ctrl_1.err.y - ctrl_1.err_old.y) *( 0.002f/T ) - ctrl_1.err_d.y);
////	ctrl_1.err_d.z += 40 *3.14 *0.002 *( 10 *ctrl_1.PID[PIDYAW].kd   *(ctrl_1.err.z - ctrl_1.err_old.z) *( 0.002f/T ) - ctrl_1.err_d.z);

//	/* 角速度误差积分 */
//	ctrl_1.err_i.x += ctrl_1.PID[PIDROLL].ki  *(ctrl_1.err.x - ctrl_1.damp.x) *T;
//	ctrl_1.err_i.y += ctrl_1.PID[PIDPITCH].ki *(ctrl_1.err.y - ctrl_1.damp.y) *T;
//	ctrl_1.err_i.z += ctrl_1.PID[PIDYAW].ki 	*(ctrl_1.err.z - ctrl_1.damp.z) *T;
//	/* 角速度误差积分分离 */
//	ctrl_1.eliminate_I.x = Thr_Weight *CTRL_1_INT_LIMIT ;
//	ctrl_1.eliminate_I.y = Thr_Weight *CTRL_1_INT_LIMIT ;
//	ctrl_1.eliminate_I.z = Thr_Weight *CTRL_1_INT_LIMIT ;
//	/* 角速度误差积分限幅 */
//	ctrl_1.err_i.x = LIMIT( ctrl_1.err_i.x, -ctrl_1.eliminate_I.x,ctrl_1.eliminate_I.x );
//	ctrl_1.err_i.y = LIMIT( ctrl_1.err_i.y, -ctrl_1.eliminate_I.y,ctrl_1.eliminate_I.y );
//	ctrl_1.err_i.z = LIMIT( ctrl_1.err_i.z, -ctrl_1.eliminate_I.z,ctrl_1.eliminate_I.z );
//	/* 角速度PID输出 */
//	ctrl_1.out.x = 3 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.x),0,1)*except_AS.x + ( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDROLL].kp  *( ctrl_1.err.x + ctrl_1.err_d.x + ctrl_1.err_i.x ) );
//										//*(MAX_CTRL_ASPEED/300.0f);
//	ctrl_1.out.y = 3 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.y),0,1)*except_AS.y + ( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDPITCH].kp *( ctrl_1.err.y + ctrl_1.err_d.y + ctrl_1.err_i.y ) );
//										//*(MAX_CTRL_ASPEED/300.0f);
//	ctrl_1.out.z = 3 *( ctrl_1.FB *LIMIT((0.45f + 0.55f*ctrl_2.err_weight.z),0,1)*except_AS.z + ( 1 - ctrl_1.FB ) *ctrl_1.PID[PIDYAW].kp   *( ctrl_1.err.z + ctrl_1.err_d.z + ctrl_1.err_i.z ) );
//										//*(MAX_CTRL_ASPEED/300.0f);
//										*/
//#endif
	Thr_Ctrl(T);// 油门控制
#if	 USE_MY_PWM_OUT
	All_Out(ctrl_1.out.x,ctrl_1.out.y,ctrl_1.out.z);
#endif

//	ctrl_1.err_old.x = ctrl_1.err.x;
//	ctrl_1.err_old.y = ctrl_1.err.y;
//	ctrl_1.err_old.z = ctrl_1.err.z;

//	g_old[A_X] =  mpu6050.Gyro_deg.x ;
//	g_old[A_Y] = -mpu6050.Gyro_deg.y ;
//	g_old[A_Z] = -mpu6050.Gyro_deg.z ;
}
#include "pwm_in.h"
int baro_to_ground,baro_ground_off;
float thr_value;
u8 Thr_Low,force_Thr_low=0;
float Thr_Weight;
float thr_test;
void Thr_Ctrl(float T)
{	float delta_thr;
	static float thr;
	static float Thr_tmp;
		static u8 cnt_thr_add,fly_ready_r;
	if(!fly_ready)
	   thr=0;		 
	else
    thr = 500 + CH_filter[THRr]; //油门值 0 ~ 1000
	thr=Rc_Pwm_Out_mine[RC_THR]-1000;
//----------Drop protector-----------------
	if(!fly_ready&&500 + CH_filter[THRr]<100)
	force_Thr_low=0;
	if((fabs(Pitch)>45||fabs(Roll)>45)&&fly_ready)
		force_Thr_low=1;
//protect flag init	
	if(fly_ready_r==0&&fly_ready==1&&500 + CH_filter[THRr]>100)
		force_Thr_low=1;
		fly_ready_r=fly_ready;
	
	if(force_Thr_low)
		thr=0;
	
	
	Thr_tmp += 10 *3.14f *T *(thr/400.0f - Thr_tmp); //低通滤波
	Thr_Weight = LIMIT(Thr_tmp,0,1);    							//后边多处分离数据会用到这个值
	
	if( thr < 100 )
	{
		Thr_Low = 1;
	
	}
	else
	{ 
		Thr_Low = 0;
	}
	if( 500 + CH_filter[THRr]<100)	baro_ground_off=ALT_POS_BMP*1000;
	baro_to_ground=LIMIT(ALT_POS_BMP*1000-baro_ground_off,10,8000);
	
	Height_Ctrl(T,thr);
	
	//AUTO_LAND_FLYUP
	AUTO_LAND_FLYUP(T);
	
	
	thr_value = Thr_Weight *height_ctrl_out;   //实际使用值	
	thr_test=thr_value = LIMIT(thr_value,0,10 *MAX_THR *MAX_PWM/100);
}
/*      HEAD
         Y + (0~240)  1
      1  | 2
    _____|______ X+(0~360) 0
      3  | 4
				 |
*/
#include "rng.h"
u8 init_circle_search;
#define MISS_CIRCLE_CONTROL 25
float nav_land_miss[2];
u8 Rate_Max_num=1;
void circle_search(void)
{
	static float Rate[5];
	int flag[2]={0};
	u8 Rate_locate;
	int Y_c,X_c;
	float out[2];
	static u8 Re_map[5]={0,1,2,3};
	Y_c=circle.y_flp;
	X_c=360-circle.x_flp;
if(init_circle_search)
{
	if(Y_c>120)
		 if(X_c>180)
	   Rate_Max_num=3;
		 else
		 Rate_Max_num=4;	 
	else
	   if(X_c>180)
	   Rate_Max_num=1;
		 else
		 Rate_Max_num=2	; 
	
		 
 switch(Rate_Max_num){
	case 1: Re_map[1]=1;Re_map[2]=2;Re_map[3]=3;break;
	case 2: Re_map[1]=2;Re_map[2]=4;Re_map[3]=1;break;
	case 3: Re_map[1]=3;Re_map[2]=1;Re_map[3]=4;break;
	case 4: Re_map[1]=4;Re_map[2]=3;Re_map[3]=2;break;
 }	 
  
init_circle_search=0;
}

u8 random,temp=0;
random=RNG_Get_RandomRange(0,100);
#define RATE_BIG 60
if(random>RATE_BIG)
	Rate_locate=Re_map[1];
else if(random>(100-RATE_BIG)/2)
	Rate_locate=Re_map[2];
else 
  Rate_locate=Re_map[3];




switch(Rate_locate){
	case 1:  flag[0]=-1;flag[1]=1;break;
	case 2:  flag[0]=1; flag[1]=1;break;
	case 3:  flag[0]=-1;flag[1]=-1;break;
	case 4:  flag[0]=1; flag[1]=-1;break;
 }	
	nav_land_miss[0]=nav_land[ROLr]=out[0]=flag[0]*MISS_CIRCLE_CONTROL*circle.control_k_miss;
  nav_land_miss[1]=nav_land[PITr]=out[1]=flag[1]*MISS_CIRCLE_CONTROL*circle.control_k_miss;
 
 
}


#include "pwm_in.h"
u8 mode_change;
u8 state_pass=0;
u16 AUTO_UP_CUARVE[]={1580,1660,1660,1655,1650,1650,1650,1650,1650};
u16 AUTO_DOWN_CUARVE[]={1500,1500-50,1500-150,1500-150,1500-200,1500-200};
u16 AUTO_DOWN_CUARVE1[]={1500-150,1500-150,1500-100,1500-100,1500-80,1500-80};
#if USE_M100
float SONAR_SET_HIGHT =0.18;//0.14;
#else
float SONAR_SET_HIGHT =0.14;
#endif
float AUTO_FLY_HEIGHT =2.5;
float SONAR_CHECK_DEAD =0.1;

float AUTO_LAND_HEIGHT_1= 2.5;// 3.5 //bmp check
float AUTO_LAND_HEIGHT_2= 1.6;//1.8
float AUTO_LAND_HEIGHT_3= 0.0925;

float MINE_LAND_HIGH= 0.3;
float AUTO_LAND_SPEED_DEAD =0.2;
u8 state_v;
u32 cnt[10]={0};
float baro_ground_high;
float nav_land[2];
#define DEAD_NAV_RC 80
#define AVOID_RC 95
#if USE_M100
float AVOID[2]={1.35+0.4,1.35+0.4};
#else
float AVOID[2]={1.35,1.35};
#endif
u8 cnt_shoot=0;
u8 force_check,force_check_pass;
u8 tar_need_to_check_odroid[3]={0,0,66};
u8 tar_buf[20];
u8 tar_cnt;
u8 over_time;
float time_fly;
u8 m100_gps_in=0;
#if USE_M100
#define GPS_ERO 300*0.55
#define GPS_ERO_S 350*0.55
#else
#define GPS_ERO 450
#define GPS_ERO_S 450
#endif
#define NAV_USE_AVOID 0
void AUTO_LAND_FLYUP(float T)
{static u8 state,init,cnt_retry;
 static float sonar_r,bmp_r,bmp_thr;
	static u8 cnt_circle_check=0;
	static u16 thr_sel[3],cnt_miss_track;//跟踪失败
	static u8 flow_head_flag=0;
  static u32 cnt_back_home;
	static u16 fly_cover_cnt;
	static u8 cnt_first_avoid;
	static u8 force_stop;
	u16 i,j;
	time_fly=cnt_back_home*T;
	  u8 temp_pass=!force_check_pass;
	if(!init&&ALT_POS_BMP!=0){init=1;
		baro_ground_high=ALT_POS_BMP;
	}
	if(Rc_Pwm_Inr_mine[RC_THR]<200+1000)
	{sonar_r=SONAR_SET_HIGHT+0.05;bmp_r=ALT_POS_BMP;}
//state_change 
	switch(state)
	{//-------------------------------------------------起飞
		case SG_LOW_CHECK://low thr check
			force_stop=mode.use_qr_as_gps_tar=get_qr_pos=fly_cover_cnt=cnt_shoot=over_time=cnt_back_home=0;
		  tar_need_to_check_odroid[2]=66;
			qr_pos_off[1]=qr_pos_off[0]=0;
		  tar_cnt=0;
		  for(i=0;i<19;i++)
		    tar_buf[i]=66;
			mode.auto_land=0;	cnt_retry=0;
			if(mode.auto_fly_up==1&&pwmin.sel_in==1&&ALT_POS_SONAR2<SONAR_SET_HIGHT+SONAR_CHECK_DEAD&&fabs(Pitch)<10&&fabs(Roll)<10){
					if(Rc_Pwm_Inr_mine[RC_THR]<200+1000)
					{state=SG_MID_CHECK;cnt[0]=0;mode_change=1;thr_sel[1]=thr_sel[0]=cnt[0]=cnt[1]=0;}//to auto fly
				}
			else if(mode.auto_fly_up==0&&pwmin.sel_in==1&&ALT_POS_SONAR2>MINE_LAND_HIGH&&fabs(Pitch)<15&&fabs(Roll)<15){//手动测试起飞
					if((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000))
					{ 
						#if defined(DEBUG_TRACK)  
						state=SD_HOLD2;
						#elif defined(DEBUG_HOLD_HEIGHT) 
						state=SU_CHECK_TAR;
						#elif defined(DEBUG_HOLD_WALL) 
						state=SU_CHECK_TAR;
						#elif defined(DEBUG_TARGET_AVOID)
						state=SU_CHECK_TAR;
						#else
						state=SU_HOLD;
						#endif
						cnt[0]=0;mode_change=1;thr_sel[1]=thr_sel[0]=cnt[0]=cnt[1]=0;
					
					if(gps_data.latitude!=0&&gps_data.longitude!=0&&gpsx.gpssta>=1&&gpsx.rmc_mode=='A'){
					home_point[0]=gps_data.latitude;
					home_point[1]=gps_data.longitude;
					//gps_local_cor_zero[0]=gps_data.latitude;//设置局部坐标系原点
					//gps_local_cor_zero[1]=gps_data.longitude;
					}
					}//to hold || land			
				}

			break;
		//----------------------------------------------自动起飞
		case SG_MID_CHECK://middle thr check
			if(mode.auto_fly_up==1&&ALT_POS_SONAR2<SONAR_SET_HIGHT+SONAR_CHECK_DEAD&&fabs(Pitch)<15&&fabs(Roll)<15){
					if((Rc_Pwm_Inr_mine[RC_THR]>400+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<600+1000))
						cnt[0]++;
					else cnt[0]=0;
					
					if(cnt[0]>2/T)
					{state=SU_UP1;thr_sel[1]=thr_sel[0]=cnt[0]=cnt[1]=0;mode_change=1;
					
					if(gps_data.latitude!=0&&gps_data.longitude!=0&&gpsx.gpssta>=1&&gpsx.rmc_mode=='A'){
					home_point[0]=gps_data.latitude;
					home_point[1]=gps_data.longitude;}
						
					
					}
				}//  
			else if(Rc_Pwm_Inr_mine[RC_THR]<200+1000||mode.auto_fly_up==0)
			{state=SG_LOW_CHECK;mode_change=1;}
			
			if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;mode_change=1;}}
			break;
		case SU_UP1://take off
		if(mode.auto_fly_up==1&&(Rc_Pwm_Inr_mine[RC_THR]>400+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<600+1000)){
				 if(cnt[0]++>2/T||ALT_POS_SONAR2>exp_height_front*0.85)
				 {mode_change=1;state=SU_HOLD;thr_sel[1]=thr_sel[0]=cnt[0]=cnt[1]=0;}	 
	   }
		 else
		 {mode_change=1;state=SU_HOLD;thr_sel[1]=thr_sel[0]=cnt[0]=cnt[1]=0;} 
		  
		 
		 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
		 break;
		case SU_HOLD://keep high
				if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15&&fabs((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&fabs((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
				
							 if(cnt[1]++>4.666/T)
							 {state=SU_TO_CHECK_POS;	qr_local_pos[North]=qr_local_pos[East]=0;	 
							 cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}

					}
			   
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
		break;

		//----------------------------------------巡航--------------------------------------------------	 
		
		case SU_TO_CHECK_POS://循航到检测目标位置 --巡航到降落GPS位置
					 tar_need_to_check_odroid[2]=66;
				 if(fabs(ultra_ctrl.err)<250&&ALT_POS_SONAR2>MINE_LAND_HIGH-0.15&&fabs((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&fabs((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
					if(mode.en_gps){
								if(fabs(nav_Data.gps_ero_dis_lpf[0])<GPS_ERO&&fabs(nav_Data.gps_ero_dis_lpf[1])<GPS_ERO&&(gpsx.gpssta>=1&&gpsx.rmc_mode=='A'))
							{
							 if(cnt[1]++>0.6/T)
							 {
							 if(get_qr_pos&&fabs(qr_local_pos[2]<100)&&circle.connect)
							 state=SU_CHECK_TAR;
							 //else
							 //state=SD_CIRCLE_MID_DOWN;//到达目标后未在过程中发现降落区域处理 
							 
							 cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}

							}	else
								cnt[1]=0;
							} 
						else
						{
						if(cnt[1]++>0.4/T)
						{state=SU_CHECK_TAR;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}}
					}
			    
					if(state_pass)
					{state_pass=0;state=SU_CHECK_TAR;}
					#if defined(DEBUG_TARGET_AVOID)
					state=SU_CHECK_TAR;	
					#endif
					#if defined(DEBUG_TARGET_AVOID)
					#else
					if(over_time==1)
						{state=SD_TO_HOME;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					else if(over_time==2)
					  {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					#endif
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
		break; 
		case SU_CHECK_TAR://  ---飞行过程中看到过降落目标  缓慢下降到3.5后再执行后续降落流程 
					#if defined(DEBUG_HOLD_WALL) 
					force_check_pass=0;
					#elif defined(DEBUG_HOLD_HEIGHT) 
					force_check_pass=0;
					#elif defined(DEBUG_TARGET_AVOID)
					force_check_pass=0;
					#endif
		  
    //	if(((tar_need_to_check_odroid[0]&&tar_need_to_check_odroid[1]!=66&&circle.connect)||force_check_pass)&&
		if(fabs(ultra_ctrl.err)<250&&
				fabs((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&fabs((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15)
					 {
					if(cnt[4]++>0.4/T)
				 {   tar_need_to_check_odroid[2]=tar_need_to_check_odroid[1];
					  #if defined(DEBUG_TARGET_AVOID)
					  tar_buf[tar_cnt++]=tar_need_to_check_odroid[2];
					  state=SU_TO_CHECK_POS;
					  #else				  
					  for(i=0;i<19;i++){
					  if(tar_need_to_check_odroid[2]==tar_buf[i])
						{state=SU_TO_CHECK_POS;break;}	
					  else//use now
					  state=SU_TO_START_POS;//巡航到记录的qr位置
						}
					  #endif
					 cnt_miss_track=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}} 
				 else
					cnt[4]=0; 
			   			 }
				 else
					cnt[4]=0; 
		
				  #if defined(DEBUG_HOLD_WALL) 
					#elif defined(DEBUG_HOLD_HEIGHT) 
				  #elif defined(DEBUG_TARGET_AVOID)
					#else
				 	if(0)//cnt[1]++>666.6666/T)//在预定高度任未检测到则进行返航决策
						{state=SU_TO_START_POS;
							cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
							//--- out_timer_random
							   tar_need_to_check_odroid[2]=2;
								 for(j=0;j<500;j++){		
									 int tar_temp=	RNG_Get_RandomRange(0,9);
									 u8 flag1=0;
											for(i=0;i<19;i++){
											if(tar_temp==tar_buf[i]){
												flag1=1;break;}
											}	
											
											if(!flag1)
											{tar_need_to_check_odroid[2]=tar_temp;break;}
											else
											flag1=0;
								 }
									//
								
						}
					#endif
					#if defined(DEBUG_TARGET_AVOID)
					#else
				 if(over_time==1)
						{state=SD_TO_HOME;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					else if(over_time==2)
					  {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					#endif
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
					break; 		 
				 
		case SU_TO_START_POS://巡航到第一次看到qr的GPS位置8
					 if(mode.qr_cal_by_px)
						 mode.use_qr_as_gps_tar=1;
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15&&fabs((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&fabs((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
					if(mode.en_gps){//use now
								if(((circle.check&&circle.connect)||//过程中看见
									(mode.en_qr_land&&fabs(nav_Data.gps_ero_dis_lpf[0])<GPS_ERO&&fabs(nav_Data.gps_ero_dis_lpf[1])<GPS_ERO))&&(gpsx.gpssta>=1&&gpsx.rmc_mode=='A'))
							{
							 if(cnt[1]++>0.45*2/T)//开始下降
							 {state=SD_CIRCLE_MID_DOWN;m100_gps_in=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1; gps_target_change=1;
							 tar_now_gps[0]=tar_point_globle[0];
							 tar_now_gps[1]=tar_point_globle[1];
							 mode.use_qr_as_gps_tar=1;//使能QR作为目标
							 Flow_save_tar_s();}

							}	else
								cnt[1]=0;
							} 
						else
						{
						if(cnt[1]++>2.5/T)//默认测飞	
						{state=SD_HOLD;m100_gps_in=fly_cover_cnt=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;Flow_save_tar_s();}
						}
					}
			   
					 if(thr_sel[1]++>30/T&&0){
					 state=SU_TO_CHECK_POS;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
					 }
					 
					if(state_pass)
					{state_pass=0;state=SD_HOLD;m100_gps_in=fly_cover_cnt=0;}
					
				 if(over_time==1)
						{state=SD_TO_HOME;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					else if(over_time==2)
					  {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}	 
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
		break; 		 
				 
				 
		//-----------------------------------------对目标---------------------------------------------
    case SD_HOLD://正飞直到看到目标
			
		cnt_retry=0;
		     if((circle.check&&circle.connect)||force_check)
					 cnt_circle_check++;
				 else
					 cnt_circle_check=0;
    	if(cnt_circle_check>3&&
				fabs((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&fabs((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15)
					 {
					if(cnt[4]++>0.4/T)
				 {
					 if(mode.en_flow_break)
					 {state=SD_HOLD_BREAK;Flow_save_tar_b();} 	 
					 else
					 state=SD_HOLD2;
					 cnt_miss_track=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}} 
				 else
					cnt[4]=0; 
			   			 }
				 else
					cnt[0]=0; 
				 
				 flow_head_flag=1;
					
				 #if USE_M100   //warning need test
				 if(thr_sel[2]>5/T)
					 m100_gps_in=1;
				 else if(fabs(nav_Data.gps_ero_dis_lpf[1])<GPS_ERO_S*0.15)
					 thr_sel[2]++;
			
				 #else
				 m100_gps_in=0;
				 #endif
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15&&fabs((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&fabs((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
						if((now_position[1]>target_position_task_s[1]&&(mode.en_gps1==0&&mode.en_gps==0))||
							((mode.en_gps1||mode.en_gps)&&(gpsx.gpssta>=1&&gpsx.rmc_mode=='A')&&((fabs(nav_Data.gps_ero_dis_lpf[0])<GPS_ERO_S&&fabs(nav_Data.gps_ero_dis_lpf[1])<GPS_ERO_S)||m100_gps_in)))
				  {
					  
					 if(cnt[1]++>0.4/T){//&&(tar_point_globler[0]!=tar_point_globle[0])&&(tar_point_globler[1]!=tar_point_globle[1])){
					 state=SD_HOLD_BACK;m100_gps_in=fly_cover_cnt=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
				
					}	else
						cnt[1]=0;
			    }
				 
					 if(thr_sel[1]++>30/T){
					 state=SD_HOLD_BACK;m100_gps_in=fly_cover_cnt=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
					 }
					
					if(state_pass)
					{state_pass=0;state=SD_HOLD_BACK;m100_gps_in=fly_cover_cnt=0;}
					
				 if(over_time==1)
						{state=SD_TO_HOME;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					else if(over_time==2)
					  {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}	
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
					break; 
		case SD_HOLD_BACK://倒飞直到看到目标
			
		cnt_retry=0;
		     if((circle.check&&circle.connect)||force_check)
					 cnt_circle_check++;
				 else
					 cnt_circle_check=0;
    	if(cnt_circle_check>3&&
				fabs((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&fabs((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15)
					 {
							if(cnt[4]++>0.4/T)
							{
									if(mode.en_flow_break)//&&fabs(PWM_DJ[1]-PWM_DJ1)<80)//倒着飞云台中位附近才光流制动
									{state=SD_HOLD_BREAK;Flow_save_tar_b();} 	 
									else
									state=SD_HOLD2;
									cnt_miss_track=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
							}
					 } 
				 else
					cnt[4]=0; 
			   			 }
				 else
					cnt[0]=0; 
				 
				 flow_head_flag=0;
				 
				 #if USE_M100
				 if(thr_sel[2]>5/T)
					 m100_gps_in=1;
				 else if(fabs(nav_Data.gps_ero_dis_lpf[1])<GPS_ERO_S*0.15)
					 thr_sel[2]++;
			
				 #else
				 m100_gps_in=0;
				 #endif
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15&&fabs((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&fabs((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
						if((now_position[1]>target_position_task_s[1]&&(mode.en_gps1==0&&mode.en_gps==0))||
							((mode.en_gps1||mode.en_gps)&&(gpsx.gpssta>=1&&gpsx.rmc_mode=='A')&&((fabs(nav_Data.gps_ero_dis_lpf[0])<GPS_ERO_S&&fabs(nav_Data.gps_ero_dis_lpf[1])<GPS_ERO_S)||m100_gps_in)))
				  {
					  if(cnt[1]++>0.34/T)
						{state=SU_TO_CHECK_POS;m100_gps_in=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					   
					 } else
							cnt[1]=0;
			    }
				 	if(state_pass)
					{state_pass=0;state=SU_TO_CHECK_POS;}
					
					
					 if(thr_sel[1]++>30/T){
					 state=SU_TO_CHECK_POS;m100_gps_in=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
					 }
					
         if(over_time==1)
						{state=SD_TO_HOME;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					else if(over_time==2)
					  {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}					 
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
					break; 		 
		case SD_HOLD_BREAK://光流刹车
			
    	if(fabs((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&fabs((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15&&(fabs(Pitch)<5&&fabs(Roll)<5))
					 {
					if(cnt[4]++>1.5/T)
				 {state=SD_HOLD2;cnt_miss_track=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}} 
				 else
					cnt[4]=0; 
			   			 }
				 else
					cnt[4]=0; 
				 
			   if(cnt[1]++>2.5/0.005)
					{state=SD_HOLD2;cnt_miss_track=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
				
				 if(over_time==1)
						{state=SD_TO_HOME;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					else if(over_time==2)
					  {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}	
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
					break; 	
				 
    case SD_HOLD2://云台跟踪目标控制
		
		cnt_retry=0;
    	if(mode.en_shoot&&fabs(PWM_DJ[1]-PWM_DJ1)<SHOOT_PWM_DEAD1&&fabs(PWM_DJ[0]-(PWM_DJ0+SHOOT_PWM_OFF0))<SHOOT_PWM_DEAD0&&
				//((int)Rc_Pwm_Inr_mine[RC_THR]>400+1000)&&((int)Rc_Pwm_Inr_mine[RC_THR]<600+1000)&&
			   #if !DEBUG_IN_ROOM
			    (fabs(ultra_ctrl_head.err1)<100)&&
			   #endif
					((circle.check &&circle.connect)||force_check)&&
						fabs((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&fabs((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC)//跟踪到位
				{//中位开始下降 其他可以控制
				 if(1){
					 if(cnt[4]++>T_SHOOT_CHECK/T)
				 {state=SD_SHOOT;cnt_shoot=cnt_miss_track=cnt_shoot=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}} 
				 else
					cnt[4]=0; 
			   	}
			 else
					cnt[4]=0; 
			 
			if(mode.en_track_forward)
			{    if(cnt_miss_track++>2/0.005)
							{
							#if defined(DEBUG_TRACK)  
							#else
							if(flow_head_flag)
							state=SD_HOLD;
							else 
							state=SD_HOLD_BACK;	
							#endif
							fly_cover_cnt=cnt_miss_track=cnt_shoot=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
			}
			else
				cnt_miss_track=0;
			
			if((circle.check&&circle.connect)||force_check)
				cnt_miss_track=0;
			
			
			if(over_time==1)
						{state=SD_TO_HOME;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
					else if(over_time==2)
					  {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
			if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
			
		break;
		 case SD_SHOOT://射击模式
   #define SHOOT_OUT_SEL 1
		if(mode.en_shoot&&fabs(PWM_DJ[1]-PWM_DJ1)<SHOOT_PWM_DEAD1&&fabs(PWM_DJ[0]-(PWM_DJ0+SHOOT_PWM_OFF0))<SHOOT_PWM_DEAD0&&
			(fabs(Pitch)<10&&fabs(Roll)<10)&&
		  ((circle.check &&circle.connect)||force_check)&&
				//((int)Rc_Pwm_Inr_mine[RC_THR]>400+1000)&&((int)Rc_Pwm_Inr_mine[RC_THR]<600+1000)&&
						fabs((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&fabs((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC)//跟踪到位
				{
				  if(cnt[4]++>T_SHOOT_CHECK/T)
				 {en_shoot=1;thr_sel[2]=cnt[4]=cnt[1]=0;thr_sel[1]++;}
			#if SHOOT_OUT_SEL	 
				#if defined(DEBUG_TRACK)  
			  #else 
				 if(thr_sel[1]++>8.888*1.666*0.86/T&&(fabs(ultra_ctrl_head.err1)<88))
				{state=SU_TO_CHECK_POS;cnt_shoot=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
				tar_buf[tar_cnt++]=tar_need_to_check_odroid[2];
				}
				#endif
      #endif 				
				} 
					 else
			 {cnt[0]=0;cnt[4]=0; }
			 
			 
			 
			 if(mode.en_track_forward)
			{    if(cnt_miss_track++>2/0.005)
							{state=SD_HOLD2;fly_cover_cnt=cnt_shoot=cnt_miss_track=cnt_shoot=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
			}
			else
				cnt_miss_track=0;
			
			#if defined(DEBUG_TRACK)  
			#else
			#if SHOOT_OUT_SEL
			if(thr_sel[1]++>8.8888*2.666*0.8/T)
			#else
			if(thr_sel[1]++>8.8888*2.666*0.8/T)
			#endif	
				{state=SU_TO_CHECK_POS;cnt_shoot=cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;
				tar_buf[tar_cnt++]=tar_need_to_check_odroid[2];
				}
			#endif	
			if((circle.check&&circle.connect)||force_check)
				cnt_miss_track=0;
			if(mode.dj_by_hand){state=SD_HOLD2;cnt_shoot=en_shoot=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;} 
			
			if(over_time==1)
			{state=SD_TO_HOME;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
			else if(over_time==2)
			{state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}
			if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt_shoot=en_shoot=cnt[3]=0;}}
			
		break;	
		//-------------------------------------自动降落
			case SD_TO_HOME://巡航到作业起点位置
					 
				 if(ALT_POS_SONAR2>MINE_LAND_HIGH-0.15&&fabs((int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT)<DEAD_NAV_RC&&fabs((int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL)<DEAD_NAV_RC){//跟踪到位
					if(mode.en_gps){
								if(fabs(nav_Data.gps_ero_dis_lpf[0])<GPS_ERO&&fabs(nav_Data.gps_ero_dis_lpf[1])<GPS_ERO&&(gpsx.gpssta>=1&&gpsx.rmc_mode=='A'))
							{
							 if(cnt[1]++>1.2/T)
							 {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}

							}	else
								cnt[1]=0;
							} 
						else
						{
						if(cnt[1]++>2.5/T)//
						{state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}}
					}

				    if(over_time==2)
					  {state=SD_CIRCLE_MID_DOWN;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=0;mode_change=1;}	 
				 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}
		break; 		 
	
		case SD_CIRCLE_MID_DOWN://   //下降并对二维码
		  	if(mode.en_land_avoid&&(avoid_color[0]||avoid_color[1]||avoid_color[2]||avoid_color[3]))
				 force_stop=1;
			
				if(((int)Rc_Pwm_Inr_mine[RC_THR]>400+1000)&&((int)Rc_Pwm_Inr_mine[RC_THR]<600+1000)&&ALT_POS_SONAR2<3.5&&force_stop==1&&0){
			   state=SD_SAFE;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=cnt[2]=0;mode_change=1;
				 }
			   else if(((int)Rc_Pwm_Inr_mine[RC_THR]>400+1000)&&((int)Rc_Pwm_Inr_mine[RC_THR]<600+1000)){
				 if(ALT_POS_SONAR2<SONAR_SET_HIGHT+0.25)//&&fabs(ALT_POS_BMP-bmp_r)<0.866)//Sonar check 0.5m
				 {if(cnt[4]++>1/T)
					{state=SD_CHECK_G;cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[1]=cnt[2]=0;mode_change=1;}
				 }
				 else
					 cnt[4]=0;
				}

			 if(pwmin.sel_in==0){if(cnt[3]++>1.5/T){mode_change=1;state=SD_SAFE;cnt[3]=0;}}//restart until land
				break;
		case SD_CHECK_G://shut motor  电机停转检测
			if(fabs(ALT_VEL_SONAR)<AUTO_LAND_SPEED_DEAD&&ALT_POS_SONAR2<SONAR_SET_HIGHT+0.25)//
				cnt[2]++;
			else
				cnt[2]=0;
			if(cnt[2]>1.25/T)
			{cnt_circle_check=thr_sel[1]=thr_sel[2]=cnt[4]=cnt[2]=cnt[1]=0;mode_change=1;state=SD_SHUT_DOWN;}
			
			if(pwmin.sel_in==0){mode_change=1;state=SD_SAFE;}
		break;
		case SD_SHUT_DOWN://reset
    if((Rc_Pwm_Inr_mine[RC_THR]<200+1000)&&ALT_POS_SONAR2<SONAR_SET_HIGHT+0.3)
		{state=SG_LOW_CHECK;cnt_retry=0;}
		
		break;
		//------------------------------------SAFE------------------------------------------------
		case SD_SAFE://safe out
			cnt_retry=0;mode_change=1;flow_head_flag=0;
		if(mode.auto_fly_up==0&&(Rc_Pwm_Inr_mine[RC_THR]<200+1000)&&ALT_POS_SONAR2<SONAR_SET_HIGHT+0.25)
		state=SG_LOW_CHECK;	
		break;
		default:{mode_change=1;state=SD_SAFE;}break;
		
	}

#if defined(DEBUG_HOLD_WALL) 
mode.en_rth_mine=0;
#elif defined(DEBUG_TRACK) 
mode.en_rth_mine=0;
#elif defined(DEBUG_HOLD_HEIGHT) 	
mode.en_rth_mine=0;
#elif defined(DEBUG_TARGET_AVOID) 	
mode.en_rth_mine=0;	
#endif	
//超时处理	
	if(mode.en_rth_mine){
	if(state!=SG_LOW_CHECK)
	cnt_back_home++;
	#if USE_M100
	if(cnt_back_home>6.66*60/T)//超时降落	
	{over_time=2;}
	else if(cnt_back_home>3.666*60/T)//超时间返航
	{over_time=1;}}
	#else
	if(cnt_back_home>6.6*60/T)//超时降落	
	{over_time=2;}
	else if(cnt_back_home>6*60/T)//超时间返航
	{over_time=1;}}
	#endif	
	
#define DEAD_CIRCLE_CHECK 50
//circle ——check	
		if(circle.check&&circle.connect&&fabs(circle_use[0])<DEAD_CIRCLE_CHECK&&fabs(circle_use[1])<DEAD_CIRCLE_CHECK)
			mode.en_circle_nav=1;
		else
			mode.en_circle_nav=0;
	#if USE_M100
  k_m100[4]=k_m100_laser_avoid;
	#else
	k_m100[4]=1;
	#endif	  
		//circle_search();//for test
//-----------------------NAV_OutPut--------------------
		switch(state){
			case SU_TO_CHECK_POS://导航到检查点
				  if(!mode.en_gps){
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
					nav_land[PITr]=ultra_ctrl_out_head;
					else	
					nav_land[PITr]=0;
				  if((!mode.dj_by_hand&&mode.en_track_forward&&mode.hold_use_flow))//高度限制已经在最外边加上
					{
						Flow_set_tar(target_position_task_e[1]*2);
					if(mode.en_dji_yaw)
					{if(fabs(ctrl_2.err.z)<2.5&&(fabs(ultra_ctrl_head.err1)<120))	
							nav_land[ROLr]=track.forward;//flow_control_out;	
							else
							nav_land[ROLr]=0;}
					else
					nav_land[ROLr]=track.forward;//flow_control_out;
					}	
					else  if((!mode.dj_by_hand&&mode.hold_use_flow))
					nav_land[ROLr]=flow_control_out;	
					else
					{nav_land[ROLr]=0;}
					
						if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
				 else if(!mode.dj_by_hand&&mode.en_track_forward)//------------------GPS
				 {
					
						tar_point_globle[0]=check_way_point[0]; tar_point_globle[1]=check_way_point[1];
						nav_land[PITr]=nav_gps[PITr];
						nav_land[ROLr]=nav_gps[ROLr];
					#if NAV_USE_AVOID 
					// 	if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					//else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
					//if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
					#endif	
				 }
			break;
			
			
				case SU_CHECK_TAR://检查数字
				  if(!mode.en_gps){
					
						if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
						nav_land[PITr]=ultra_ctrl_out_head;
						else	
						nav_land[PITr]=0;
						nav_land[ROLr]=0;
						if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
				 else if(!mode.dj_by_hand&&mode.en_track_forward)//------------------GPS
				 {
						#if defined(DEBUG_HOLD_WALL)
						if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
						nav_land[PITr]=ultra_ctrl_out_head;
						else	
						nav_land[PITr]=0;
						nav_land[ROLr]=0;					 
						#elif defined(DEBUG_HOLD_HEIGHT) 
						if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
						nav_land[PITr]=ultra_ctrl_out_head;
						else	
						nav_land[PITr]=0;
						nav_land[ROLr]=0;
						#else
						
						tar_point_globle[0]=check_way_point[0]; tar_point_globle[1]=check_way_point[1];
						
						//if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&ultra_ctrl_out_head!=0&&ALT_POS_SONAR_HEAD>1.88)//前向壁障
						//nav_land[PITr]=ultra_ctrl_out_head;
						//else	
						nav_land[PITr]=nav_gps[PITr];
					  //---------------------------wait for test  检查点使用前向壁障
						#if !NAV_USE_AVOID	
						nav_land[PITr]=nav_gps[PITr];
						#endif
						nav_land[ROLr]=nav_gps[ROLr];
					#endif	
						
					#if NAV_USE_AVOID	
					// if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					//else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
					//if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
				  #endif
				 }
			break;	 	 
				 
			case SU_TO_START_POS://导航到第一次看到码的点
				  if(!mode.en_gps){
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
					nav_land[PITr]=ultra_ctrl_out_head;
					else	
					nav_land[PITr]=0;
				  if((!mode.dj_by_hand&&mode.en_track_forward&&mode.hold_use_flow))//高度限制已经在最外边加上
					{
						Flow_set_tar(target_position_task_e[1]*2);
					if(mode.en_dji_yaw)
					{if(fabs(ctrl_2.err.z)<2.5&&(fabs(ultra_ctrl_head.err1)<120))	
							nav_land[ROLr]=track.forward;//flow_control_out;	
							else
							nav_land[ROLr]=0;}
					else
					nav_land[ROLr]=track.forward;//flow_control_out;
					}	
					else  if((!mode.dj_by_hand&&mode.hold_use_flow))
					nav_land[ROLr]=flow_control_out;	
					else
					{nav_land[ROLr]=0;}
					
						if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
				 else if(!mode.dj_by_hand&&mode.en_track_forward)//------------------GPS
				 {
					
						tar_point_globle[0]=qr_gps_pos[0]; tar_point_globle[1]=qr_gps_pos[1];
						nav_land[PITr]=nav_gps[PITr];
						nav_land[ROLr]=nav_gps[ROLr];
					#if	NAV_USE_AVOID
					 if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
				  #endif
				 }
			break;	 
				 	 
			//
			case SD_HOLD://正飞
				  if(!mode.en_gps){
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
					nav_land[PITr]=ultra_ctrl_out_head;
					else	
					nav_land[PITr]=0;
				  if((!mode.dj_by_hand&&mode.en_track_forward&&mode.hold_use_flow))//高度限制已经在最外边加上
					{
						Flow_set_tar(target_position_task_e[1]*2);
					if(mode.en_dji_yaw)
					{if(fabs(ctrl_2.err.z)<2.5&&(fabs(ultra_ctrl_head.err1)<120))	
							nav_land[ROLr]=track.forward;//flow_control_out;	
							else
							nav_land[ROLr]=0;}
					else{
					if(mode.en_gps1)	
					{nav_land[ROLr]=LIMIT(nav_gps[ROLr],-track.forward,track.forward);tar_point_globle[0]=way_point[1][0]; tar_point_globle[1]=way_point[1][1];}
					else
					nav_land[ROLr]=track.forward;//flow_control_out;
						
					}
					}	
					else  if((!mode.dj_by_hand&&mode.hold_use_flow))
					nav_land[ROLr]=flow_control_out;	
					else
					{nav_land[ROLr]=0;}
					
						if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
				 else  if(!mode.dj_by_hand&&mode.en_track_forward)//------------------GPS
				 {
				  tar_point_globle[0]=way_point[1][0]; tar_point_globle[1]=way_point[1][1];
					 
					if(fly_cover_cnt++>1.5/T){ fly_cover_cnt=5/T;
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
						nav_land[PITr]=ultra_ctrl_out_head;
						else	
						nav_land[PITr]=0;
					}
					else
					{
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
						nav_land[PITr]=LIMIT(ultra_ctrl_out_head,-100,0);
						else	
						nav_land[PITr]=0;
					}
						
					if((fabs(ultra_ctrl_head.err1)<1000))		
			    nav_land[ROLr]=LIMIT(nav_gps[ROLr],-track.forward,track.forward);
					 
					 	if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
			break;
			case SD_HOLD_BACK://倒着飞 for track	
				if(!mode.en_gps){
			    if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])
					nav_land[PITr]=ultra_ctrl_out_head;
					else	
					nav_land[PITr]=0;
				  if((!mode.dj_by_hand&&mode.en_track_forward&&mode.hold_use_flow))
					{
					Flow_set_tar(target_position_task_s[1]*2);		
					if(mode.en_dji_yaw&&(fabs(ultra_ctrl_head.err1)<120))
					{if(fabs(ctrl_2.err.z)<2.5)	
							nav_land[ROLr]=-track.forward;//flow_control_out;	
							else
							nav_land[ROLr]=0;}
					else
					{	
					if(mode.en_gps1)	
					{nav_land[ROLr]=LIMIT(nav_gps[ROLr],-track.forward,track.forward);tar_point_globle[0]=way_point[0][0]; tar_point_globle[1]=way_point[0][1];}
					else	
					nav_land[ROLr]=-track.forward;//flow_control_out;
					}
					}	
					else  if((!mode.dj_by_hand&&mode.hold_use_flow))
					nav_land[ROLr]=flow_control_out;	
					else
					{nav_land[ROLr]=0;}
					
					if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
				}
				else if(!mode.dj_by_hand&&mode.en_track_forward)//------------------GPS
				{
				 tar_point_globle[0]=way_point[0][0]; tar_point_globle[1]=way_point[0][1];
					if(fly_cover_cnt++>1.5/T){ fly_cover_cnt=5/T;
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
						nav_land[PITr]=ultra_ctrl_out_head;
						else	
						nav_land[PITr]=0;
					}
					else
					{
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])//前向壁障
						nav_land[PITr]=LIMIT(ultra_ctrl_out_head,-100,0);
						else	
						nav_land[PITr]=0;
					}
					
					if((fabs(ultra_ctrl_head.err1)<1000))	
			    nav_land[ROLr]=LIMIT(nav_gps[ROLr],-track.forward,track.forward);
					
						if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
				}
			break;
      case SD_HOLD_BREAK://刹车wa  效果不理想
			    if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])
					nav_land[PITr]=ultra_ctrl_out_head;
					else	
					nav_land[PITr]=0;
					if((!mode.dj_by_hand&&mode.hold_use_flow))
					{Flow_set_tar(target_position_task_b[1]);	
					 nav_land[ROLr]=LIMIT(flow_control_out,-track.forward*1.5,track.forward*1.5);	}
					else
					{nav_land[ROLr]=0;}
					
					if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
			break;					
			case SD_HOLD2:
					
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])
					nav_land[PITr]=ultra_ctrl_out_head;
					else	
					nav_land[PITr]=0;
					if(!mode.dj_by_hand&&circle.connect&&circle.check&&mode.en_track_forward)
					{ 
						nav_land[ROLr]=PWM_DJ[2];//云台
					}
					else  if((!mode.dj_by_hand&&mode.hold_use_flow))//光流
					nav_land[ROLr]=flow_control_out;
					else 
					{nav_land[ROLr]=0;}
					
					if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
			break;
			case SD_SHOOT://发射模式
					if((mode.test3||mode.test2)&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])
					nav_land[PITr]=ultra_ctrl_out_head;
					else	
					nav_land[PITr]=0;
					if(!mode.dj_by_hand&&circle.connect&&circle.check&&mode.en_track_forward)
					{ 
						nav_land[ROLr]=PWM_DJ[2];//云台
					}
					else  if((!mode.dj_by_hand&&mode.hold_use_flow))//光流
					nav_land[ROLr]=flow_control_out;
					else 
					{nav_land[ROLr]=0;}
					
				
					if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>88)nav_land[PITr]=-AVOID_RC*k_m100[4];
			break;	

					
			case SD_TO_HOME://导航到home
				  if(!mode.en_gps){
					
					nav_land[PITr]=nav_land[ROLr]=0;
					
						if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
				 else if(!mode.dj_by_hand&&mode.en_track_forward)
				 {
					
						tar_point_globle[0]=home_point[0]; tar_point_globle[1]=home_point[1];
						nav_land[PITr]=nav_gps[PITr];
						nav_land[ROLr]=nav_gps[ROLr];
						
					 //if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					//else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
					//if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
			break;	 		
//circe track
	     case SD_CIRCLE_MID_DOWN://
				  if(!mode.en_gps){
					
					nav_land[PITr]=nav_land[ROLr]=0;
					
						if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0])nav_land[PITr]=-AVOID_RC*k_m100[4];
					if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
				 else if(!mode.dj_by_hand&&mode.en_track_forward)
				 {
					
					 tar_point_globle[0]=check_way_point[0]; tar_point_globle[1]=check_way_point[1];
           //if(mode.use_qr_as_gps_tar){
						nav_land[PITr]=nav_gps[PITr];
						nav_land[ROLr]=nav_gps[ROLr];
					 //}
					 //if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]*0.8&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*1.5*k_m100[4];
					//else if(mode.test3&&ALT_POS_SONAR_HEAD<AVOID[0]&&ALT_POS_SONAR2>0.3&&SONAR_HEAD_CHECK[0]&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
					//if(mode.test2&&ALT_POS_SONAR_HEAD_LASER_SCANER<AVOID[1]&&ALT_POS_SONAR2>0.3&&S_head>125)nav_land[PITr]=-AVOID_RC*k_m100[4];
				 }
			break;	 			
			default:	
					{nav_land[0]=	nav_land[1]=0;}
			break;
		}
	
	
state_v=state;
		
	#if USE_M100
  k_m100[2]=k_m100_gps[2];
	#else
	k_m100[2]=1;
	#endif
u16 Heigh_thr=LIMIT(ultra_ctrl_out*0.4,-100,100)*k_m100[2]+OFF_RC_THR;
//-----------------state_out thr-----------------------
	switch(state)
	{
		case SG_LOW_CHECK:Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];break;
		case SG_MID_CHECK:Rc_Pwm_Out_mine[RC_THR]=LIMIT(Rc_Pwm_Inr_mine[RC_THR],0,1450);break;		
		case SU_UP1://load curve
			   exp_height=exp_height_check;
			if(cnt[1]++>1/T)
			{thr_sel[0]++;cnt[1]=0;}
			Rc_Pwm_Out_mine[RC_THR]=AUTO_UP_CUARVE[thr_sel[0]];
		  break;
		case SU_HOLD://keep height
				exp_height=exp_height_check;
			if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
			break;
			case SU_TO_CHECK_POS://keep height
				exp_height=exp_height_check;
			if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
			break;	
			case SU_CHECK_TAR://slow down
				exp_height=exp_height_check;
			if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
//				if(ALT_POS_SONAR2>3.8)	
//			  
//					Rc_Pwm_Out_mine[RC_THR]=OFF_RC_THR-100*0.4;//=Heigh_thr;
//				else
					Rc_Pwm_Out_mine[RC_THR]=OFF_RC_THR;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
			break;	
			case SU_TO_START_POS://keep height
				exp_height=exp_height_front;
			if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
			break;	
			
		//--------------------------------------目标对准---------------------------------------------
    case SD_HOLD://keep height
			exp_height=exp_height_front;
			if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
		break;
		case SD_HOLD_BACK://keep height
			exp_height=exp_height_back;
		if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
		Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
		else
		Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
		break;
		case SD_HOLD_BREAK://keep height
//		if(flow_head_flag)exp_height=exp_height_back;//重复之前的方向
//					 else exp_height=exp_height_front;
		
		if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
		Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
		else
		Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
		break;			
		case SD_HOLD2://keep height
		if(!flow_head_flag)exp_height=LIMIT(exp_height_back+exp_height_shoot_off,500,2222);//重复之前的方向
					 else exp_height=LIMIT(exp_height_front+exp_height_shoot_off,500,2222);	
		if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
		Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
		else
		Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
		break;
		case SD_SHOOT://keep height
		if(!flow_head_flag)exp_height=LIMIT(exp_height_back+exp_height_shoot_off,500,2222);//重复之前的方向
					 else exp_height=LIMIT(exp_height_front+exp_height_shoot_off,500,2222);	
		if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
		Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
		else
		Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
		break;
		//------------------------自动下降
		case SD_TO_HOME://keep height
				exp_height=exp_height_front;
			if(mode.en_dji_h&&!dji_rst_protect&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
		break;
		case SD_CIRCLE_MID_DOWN://land check
		  if(ALT_POS_SONAR2>0.8){
				 if(circle.check){
						if(ALT_POS_SONAR2>1.6)
						Rc_Pwm_Out_mine[RC_THR]=OFF_RC_THR-100*0.5;
						else
						Rc_Pwm_Out_mine[RC_THR]=OFF_RC_THR-100*0.4;	
					}
				 else
					  Rc_Pwm_Out_mine[RC_THR]=OFF_RC_THR;
		  }
			else
			Rc_Pwm_Out_mine[RC_THR]=OFF_RC_THR-100*0.3;		
			
			if(mode.en_land_avoid&&(avoid_color[0]||avoid_color[1]||avoid_color[2]||avoid_color[3]))
				Rc_Pwm_Out_mine[RC_THR]=OFF_RC_THR;
		break;
		case SD_CHECK_G://shut motor
				#if USE_M100
		  Rc_Pwm_Out_mine[RC_THR]=OFF_RC_THR-100;
		  #else
			Rc_Pwm_Out_mine[RC_THR]=OFF_RC_THR-60;
		  #endif
		break;
		case SD_SHUT_DOWN://reset
      Rc_Pwm_Out_mine[RC_THR]=0+1000;
		break;
		//----------------------------
		case SD_SAFE://safe out
//			if(mode.en_dji_h)//&&((Rc_Pwm_Inr_mine[RC_THR]>450+1000)&&(Rc_Pwm_Inr_mine[RC_THR]<550+1000)))
//			Rc_Pwm_Out_mine[RC_THR]=Heigh_thr;
//			else
			Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];
			break;
		default:Rc_Pwm_Out_mine[RC_THR]=Rc_Pwm_Inr_mine[RC_THR];break;
	}
Rc_Pwm_Out_mine[RC_THR]=LIMIT(Rc_Pwm_Out_mine[RC_THR],pwmin.min,pwmin.max);


static u8 pwmin_selr,en_pid_r;	
	if(pwmin.sel_in!=pwmin_selr)
	mode_change=1;
	
	if(mode.en_dji_h!=en_pid_r)
	mode_change=1;
	
 en_pid_r=mode.en_dji_h;	
 pwmin_selr= pwmin.sel_in;	
}






float motor[MAXMOTORS];
float posture_value[MAXMOTORS];
float curve[MAXMOTORS];
#define MAX_THR_FIX_ANGLE MAX_CTRL_ANGLE
void All_Out(float out_roll,float out_pitch,float out_yaw)
{
	s16 motor_out[MAXMOTORS];
	u8 i;
	float posture_value[MAXMOTORS];
  float curve[MAXMOTORS];
	static float motor_last[MAXMOTORS];

	out_yaw = LIMIT( out_yaw , -5*MAX_THR ,5*MAX_THR ); //50%
	
	posture_value[0] = - out_roll + out_pitch + out_yaw ;
	posture_value[1] = + out_roll + out_pitch - out_yaw ;
	posture_value[2] = + out_roll - out_pitch + out_yaw ;
	posture_value[3] = - out_roll - out_pitch - out_yaw ;
	
	for(i=0;i<4;i++)
	{
		posture_value[i] = LIMIT(posture_value[i], -1000,1000 );
	}
	
	curve[0] = (0.55f + 0.45f *ABS(posture_value[0])/1000.0f) *posture_value[0] ;
	curve[1] = (0.55f + 0.45f *ABS(posture_value[1])/1000.0f) *posture_value[1] ;
	curve[2] = (0.55f + 0.45f *ABS(posture_value[2])/1000.0f) *posture_value[2] ;
	curve[3] = (0.55f + 0.45f *ABS(posture_value[3])/1000.0f) *posture_value[3] ;
	
	int date_throttle	= (thr_value)/cos(LIMIT(Pitch,-MAX_THR_FIX_ANGLE,MAX_THR_FIX_ANGLE)/57.324841	)/cos(LIMIT(Roll,-MAX_THR_FIX_ANGLE,MAX_THR_FIX_ANGLE)/57.324841	);//add  12.9
  motor[0] = date_throttle + Thr_Weight *curve[0] ;
	motor[1] = date_throttle + Thr_Weight *curve[1] ;
	motor[2] = date_throttle + Thr_Weight *curve[2] ;
	motor[3] = date_throttle + Thr_Weight *curve[3] ;
	mode.en_moto_smooth=1;
	  if(mode.en_moto_smooth){
     for(i=0;i<MAXMOTORS;i++){
        if(motor[i] > motor_last[i]) 
					motor[i] = (1 * (int16_t) motor_last[i] + motor[i]) / 2;  //mean of old and new
        else                                         
					motor[i] = motor[i] - (motor_last[i] - motor[i]) * 1; // 2 * new - old
			}
			 for(i=0;i<MAXMOTORS;i++)
					motor_last[i] = motor[i];  //mean of old and new
     
	    }
			
	/* 是否解锁 */
	if(fly_ready)
	{
		if( !Thr_Low )  //油门拉起
		{
			for(i=0;i<4;i++)
			{
				motor[i] = LIMIT(motor[i], (10 *READY_SPEED),(10*MAX_PWM) );
			}
		}
		else						//油门低
		{
			for(i=0;i<4;i++)
			{
				motor[i] = LIMIT(motor[i], 0,(10*MAX_PWM) );
			}
		}
	}
	else
	{
		for(i=0;i<4;i++)
		{
			motor[i] = 0;
		}
	}
	/* xxx */
	#if NEW_FLY_BOARD
	motor_out[0] = (s16)(motor[0]);  
	motor_out[1] = (s16)(motor[1]);	 
	motor_out[2] = (s16)(motor[2]);
	motor_out[3] = (s16)(motor[3]);
	#else
  motor_out[0] = (s16)(motor[0]);  
	motor_out[1] = (s16)(motor[1]);	 
	motor_out[2] = (s16)(motor[2]);
	motor_out[3] = (s16)(motor[3]);
	#endif
	//SetPwm(motor_out,0,1000); //
}
//



