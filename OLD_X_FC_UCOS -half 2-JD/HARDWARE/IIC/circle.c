#include "include.h"
#include "circle.h"
#include "alt_kf.h"

CIRCLE circle,track,mouse;
M100 m100;
float nav_circle[2],nav_circle_last[2];
/*
        y add (0~256)  1
        |
        |


        -------- x decrease (0~320)  0


*/
float circle_check=0.01;
float circle_lfp=1;
float circle_use[2];
void circle_control(float T)
{static u8 state;
 u8 i;
 static int circle_reg[2];
 float p,d,intit,rate_error[2],derivative[2];
 static float last_error[2],last_derivative[2];
 float  distance_use;
 static u16 cnt[3];	
 float out[2];
//state	
	switch(state){
		case 0:if(circle.check&&circle.connect)
			       cnt[0]++;
		       else
						 cnt[0]=0;
		       if(cnt[0]>circle_check/T)
					 {state=1;cnt[0]=0;}
		  break;
		case 1:
			     if(!circle.check||!circle.connect)
						 state=0;
					 
		      break;
		default:state=0;break;
	}
//output	
	switch(state){
		case 0:circle_use[0]=MID_X;circle_use[1]=MID_Y;break;
		case 1:circle_use[0]=circle.x;circle_use[1]=circle.y;break;
		default:circle_use[0]=MID_X;circle_use[1]=MID_Y;break;
	}
	circle.x_flp=circle.x;
	circle.y_flp=circle.y;
	 //if(circle.check&&circle.connect){
	 circle_use[0]=circle.x_flp;circle_use[1]=circle.y_flp; //}
//	 else{
//	 circle_use[0]=MID_X;circle_use[1]=MID_Y;}
//	if(ALT_POS_SONAR2<0.15)
//		 distance_use=0.8;
//	else
		distance_use=ALT_POS_SONAR2;
		//circle_use[0]=circle.x_flp-160;
	   circle_use[0]-=MID_X;
		//circle_use[1]=circle.y_flp-128;
	   circle_use[1]-=MID_Y;
		rate_error[0]=circle_use[0]*distance_use;
		rate_error[1]=circle_use[1]*distance_use;
	static float integrator_circle[2];
 if(pid.circle.in.i==0)
 {
 integrator_circle[0]=integrator_circle[1]=0;
 }
	 for(i=0;i<=1;i++){
			p = rate_error[i]*pid.circle.in.p;
			derivative[i] = (rate_error[i] - last_error[i]) ;/// DT;
			derivative[i]=0.3* derivative[i]+ 0.7*last_derivative[i];
			// update state
			last_error[i] = rate_error[i] ;
			last_derivative[i]= derivative[i];
			// add in derivative component
			d = derivative[i]*pid.circle.in.d;//d
			  integrator_circle[i] += ((float) rate_error[i]* pid.circle.in.i);// *DT;
        intit = LIMIT(integrator_circle[i],-10,10) ;
			//nav_circle[i] = p + d;
  		out[i] =circle_lfp*(p+d+intit)+(1-circle_lfp)*nav_circle_last[i];
		  nav_circle_last[i]=out[i]; 
		  //nav_circle[i]=out[i];
		  //nav_circle[i] += 0. *T *3.14f * ( -nav_circle[i] + out[i] );
 	 }
	 
	 
	 int flag[2]={1,1};
		 
//	 for(i=0;i<2;i++)
//   {
//	 if(circle_use[i]>0)
//	 flag[i]=1;
//	 else if(circle_use[i]<0)
//	 flag[i]=-1;
//	 else
//	 flag[i]=0; 
//	 }	 
//	 if(SPID.YP==1) //2
//	 flag[0]=1;//nav_circle[0]=flag[0]*(float)SPID.YD/10;
//	 else  if(SPID.YP==2)
//	 flag[0]=-1;//*(float)SPID.YD/10;
//	 else
//		flag[0]=0; 
	 nav_circle[0]=LIMIT(flag[0]*circle_use[0]*pid.circle.in.p,-150,150);
 
//	 if(SPID.YI==1) //2
//	 flag[1]=1;//nav_circle[1]=flag[1]*(float)SPID.YD/10;
//	 else  if(SPID.YI==2)
//	 flag[1]=-1;//nav_circle[1]=-flag[1]*(float)SPID.YD/10;
//	 else
//		flag[1]=0; 
	 nav_circle[1]=-1*LIMIT(flag[1]*circle_use[1]*pid.circle.in.p,-150,150);
	 
	 if(!circle.check)
	 nav_circle[0]=nav_circle[1]=0;
// nav_circle[0] =Moving_Median(18,10,out[0]);
// nav_circle[1] =Moving_Median(19,10,out[1]);	 
}




//----------------------------------GPS------------------------
#include "pwm_in.h"
#include "gps.h"
#include "filter.h"
float nav_gps[2];
navUkfStruct_t navUkfData;
navigation_gps nav_Data;

static void navUkfCalcEarthRadius(double lat) {
    double sinLat2;

    sinLat2 = sin(lat * (double)DEG_TO_RAD);
    sinLat2 = sinLat2 * sinLat2;

    navUkfData.r1 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD * ((double)1.0 - (double)NAV_E_2) / pow((double)1.0 - ((double)NAV_E_2 * sinLat2), ((double)3.0 / (double)2.0));
    navUkfData.r2 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD / sqrt((double)1.0 - ((double)NAV_E_2 * sinLat2)) * cos(lat * (double)DEG_TO_RAD);
}

static void navUkfCalcGlobalDistance(double lat, double lon, float *posNorth, float *posEast) {
    *posNorth = (lat - navUkfData.holdLat) * navUkfData.r1;
    *posEast = (lon - navUkfData.holdLon) * navUkfData.r2;
}

static void navUkfResetPosition(float deltaN, float deltaE, float deltaD) {
    int i;

    for (i = 0; i < UKF_HIST; i++) {
	navUkfData.posN[i] += deltaN;
	navUkfData.posE[i] += deltaE;
	navUkfData.posD[i] += deltaD;
    }

//    UKF_POSN += deltaN;
//    UKF_POSE += deltaE;
//    UKF_POSD += deltaD;
}

void navUkfSetGlobalPositionTarget(double lat, double lon) {
    float oldPosN, oldPosE;
    float newPosN, newPosE;

    navUkfCalcGlobalDistance(lat, lon, &oldPosN, &oldPosE);

    navUkfData.holdLat = lat;
    navUkfData.holdLon = lon;

    navUkfCalcGlobalDistance(lat, lon, &newPosN, &newPosE);

    navUkfResetPosition(newPosN - oldPosN, newPosE - oldPosE, 0.0f);
}

static void navUkfCalcLocalDistance(double localPosN, double localPosE, double *posN, double *posE) {
    *posN = localPosN - (float)navUkfData.holdLat;
    *posE = localPosE - (float)navUkfData.holdLon;
}

static void navUkfSetLocalPositionTarget(double posN, double posE) {
    float oldPosN, oldPosE;
    float newPosN, newPosE;

   // navUkfCalcLocalDistance(posN, posE, &oldPosN, &oldPosE);

    navUkfData.holdLat = posN;
    navUkfData.holdLon = posE;

    //navUkfCalcLocalDistance(posN, posE, &newPosN, &newPosE);

   // navUkfResetPosition(newPosN - oldPosN, newPosE - oldPosE, 0.0f);
}

void navUkfSetHereAsPositionTarget(void) {
//    if (navUkfData.flowPosN != 0.0f && navUkfData.flowPosE != 0.0f)
//	navUkfSetLocalPositionTarget(navUkfData.flowPosN, navUkfData.flowPosE);
//    else
	navUkfSetGlobalPositionTarget(gps_data.latitude, gps_data.longitude);
}

_st_height_pid_v qr_ctrl[2];
_st_height_pid qr_pid;

_st_height_pid_v gps_ctrl[2];
_st_height_pid gps_pid;
double lon,lat;
double tar_pos_gps[2];//测试
float flt_gps=1;
u16 MAX_GPS=100*1.5;
u8 gps_target_change;
double home_point[2]={30.8498039, 119.614204};//起飞点
double check_way_point[2]={30.8498172, 119.6145019};//检查点
double way_point[2][2]={
											  30.8498344, 119.6145401,
											  30.8498344, 119.6146316
};
double qr_gps_pos[2]={30.8498172, 119.6145019};//检查点
//double way_point[2][2]={39.9626144, 116.3038848, //正飞终点  GPS航点测试
//												39.962736 , 116.303872};  //倒飞终点

double tar_point_globle[2]={39.9626688, 116.3039488};//全局输出
double tar_now_gps[2];
double tar_point_globler[2]={39.9626688, 116.3039488};//全局输出

double gps_local_cor_zero[2]={39.9626688, 116.3039488};//局部GPS坐标系原点
u8 state_set_point;
u8 set_point1;
void  GPS_hold(nmea_msg *gpsx_in,float T)
{ static u8 init,state;
	float out_temp[2];
	u8 set_gps_point;
	if(!init){init=1;
		gps_pid.kp=0.125;//0.2;
		gps_pid.ki=0.00;//01;
		gps_pid.kd=1.888;
	}
	
	#if USE_M100
	
	 if(m100.Rc_pit>9000&m100.Rc_rol>9000&&m100.Rc_yaw<-9000&&!dji_rst_protect&&state_v==SG_LOW_CHECK&&!en_vrc)
		 set_point1=1;
	 else if(fabs(m100.Rc_pit)<1500&&fabs(m100.Rc_rol)<1500&&fabs(m100.Rc_yaw)<1500)
		 set_point1=0;
	 
	 lon = m100.Lon;
	 lat = m100.Lat;
	 if(m100.GPS_STATUS>=3&&!dji_rst_protect)
	 gpsx.gpssta=1;
	 else
	 gpsx.gpssta=0;	 
	 gpsx.rmc_mode='A';
	#else
	 lon = gpsx.longitude;
	 lat = gpsx.latitude;
  #endif	
	//if(1){
	if(lat!=0&&lon!=0&&gpsx.gpssta>=1&&gpsx.rmc_mode=='A'){//有效数据
	gps_data.latitude=lat;
	gps_data.longitude=lon;
	gps_data.angle=gpsx.angle;	
	
	set_gps_point=mode.set_point1;
  set_gps_point=set_point1;		
	static u16 cnt_delay;	
	switch(state_set_point)	
	{
		case 0:
		if(set_gps_point)	
		{
		state_set_point=1;cnt_delay=0;
		
		}
		
		break;
		case 1:
		check_way_point[0]=gps_data.latitude;
		check_way_point[1]=gps_data.longitude;
		if(!set_gps_point)//&&check_way_point[0]!=gps_data.latitude&&check_way_point[1]!=gps_data.latitude)	
		{
		state_set_point=2;
		}
		break;
		case 2:
		if(set_gps_point)//&&check_way_point[0]!=gps_data.latitude&&check_way_point[1]!=gps_data.latitude)	
		{
		if(cnt_delay++>10){cnt_delay=0;	
		state_set_point=3;
	}
		}
		break;
		case 3:
		way_point[0][0]=gps_data.latitude;
		way_point[0][1]=gps_data.longitude;
		if(!set_gps_point)//&&check_way_point[0]!=gps_data.latitude&&check_way_point[1]!=gps_data.latitude)	
		{
		state_set_point=4;
		}
		break;
		case 4:
		if(set_gps_point)//&&way_point[0][0]!=gps_data.latitude&&way_point[0][1]!=gps_data.latitude)	
		{
		if(cnt_delay++>10){cnt_delay=0;		
		state_set_point=5;
		
		}
		}
		break;
	  case 5:
		way_point[1][0]=gps_data.latitude;
		way_point[1][1]=gps_data.longitude;	
		if(!set_gps_point)	
		{
		if(cnt_delay++>10){cnt_delay=0;	
		Yaw_set_dji=To_180_degrees(m100.Yaw);	
		state_set_point=0;
		//if(KEY[3])
		WRITE_PARM();
		}
		}
		
		break;
	
	}
		

	u8 flag;
	if(Rc_Pwm_Inr_mine[RC_PITCH]<OFF_RC_PIT-80||Rc_Pwm_Inr_mine[RC_PITCH]>OFF_RC_PIT+80||
	Rc_Pwm_Inr_mine[RC_ROLL]<OFF_RC_ROL-80||Rc_Pwm_Inr_mine[RC_ROLL]>OFF_RC_ROL+80)
	flag=1;
	else 
	flag=0;
	
	navUkfCalcEarthRadius(lat);
  if(flag)//悬停测试	
  navUkfSetHereAsPositionTarget();	
	float y[3];

	
	if(mode.en_gps)//设置航点
	navUkfSetLocalPositionTarget( tar_point_globle[0], tar_point_globle[1]);	
	if(tar_pos_gps[0]!=0||tar_pos_gps[1]!=0) 
	navUkfSetLocalPositionTarget( 39.9626144, 116.3038848);
	
	navUkfCalcGlobalDistance(lat, lon, &y[0], &y[1]);//0->north 1->east
	
	if(fabs(y[0]>1000)||fabs(y[1]>1000))
	{
	 navUkfSetHereAsPositionTarget();
	 navUkfCalcGlobalDistance(lat, lon, &y[0], &y[1]);
	}	

  
  float yaw_use;
  #if USE_M100
  yaw_use=Moving_Median(23,3,m100.Yaw);
  #else
  yaw_use=Yaw;		
  #endif	
	navUkfData.yawCos=cos(0.0173*yaw_use);
	navUkfData.yawSin=sin(0.0173*yaw_use);
	
	//0-->wei  1 -->jing
	//filter
	
	
	nav_Data.gps_ero_dis_lpf[0]=flt_gps*Moving_Median(28,3,y[0]*1000)+(1-flt_gps)*nav_Data.gps_ero_dis_lpf[0];
	nav_Data.gps_ero_dis_lpf[1]=flt_gps*Moving_Median(29,3,y[1]*1000)+(1-flt_gps)*nav_Data.gps_ero_dis_lpf[1];
	
	
//	15075434@qq.com  yang-->15877918559
	int max_ero0,max_ero1;
	if(fabs(nav_Data.gps_ero_dis_lpf[0])>800*4)
		max_ero0=600*2;
	else
		max_ero0=600;
	
	if(fabs(nav_Data.gps_ero_dis_lpf[1])>800*4)
		max_ero1=600*2;
	else
		max_ero1=600;
	//PID
  //N
	if(gps_pid.ki==0||flag)gps_ctrl[0].err_i=0;
	gps_ctrl[0].err = ( gps_pid.kp*LIMIT(my_deathzoom(nav_Data.gps_ero_dis_lpf[0],25),-max_ero0,max_ero0) );//mm
	gps_ctrl[0].err_i += gps_pid.ki *gps_ctrl[0].err *T;
	gps_ctrl[0].err_i = LIMIT(gps_ctrl[0].err_i,-1 *ULTRA_INT,1 *ULTRA_INT);
	gps_ctrl[0].err_d = gps_pid.kd *( 0.0f *(-(0*1000)*T) + 1.0f *(gps_ctrl[0].err - gps_ctrl[0].err_old) );
	gps_ctrl[0].pid_out = gps_ctrl[0].err + gps_ctrl[0].err_i + gps_ctrl[0].err_d;
	gps_ctrl[0].pid_out = LIMIT(gps_ctrl[0].pid_out,-1000,1000);
	gps_ctrl[0].err_old = gps_ctrl[0].err;
	//E
	if(gps_pid.ki==0||flag)gps_ctrl[1].err_i=0;
	gps_ctrl[1].err = ( gps_pid.kp*LIMIT(my_deathzoom(nav_Data.gps_ero_dis_lpf[1],25),-max_ero1,max_ero1) );//mm
	gps_ctrl[1].err_i += gps_pid.ki *gps_ctrl[1].err *T;
	gps_ctrl[1].err_i = LIMIT(gps_ctrl[1].err_i,-1 *ULTRA_INT,1 *ULTRA_INT);
	gps_ctrl[1].err_d = gps_pid.kd *( 0.0f *(-(0*1000)*T) + 1.0f *(gps_ctrl[1].err - gps_ctrl[1].err_old) );
	gps_ctrl[1].pid_out = gps_ctrl[1].err + gps_ctrl[1].err_i + gps_ctrl[1].err_d;
	gps_ctrl[1].pid_out = LIMIT(gps_ctrl[1].pid_out,-1000,1000);
	gps_ctrl[1].err_old = gps_ctrl[1].err;
	
	
	nav_Data.holdSpeedN = -gps_ctrl[0].pid_out*0.4 ;
	nav_Data.holdSpeedE = -gps_ctrl[1].pid_out*0.4 ;

	//x-> pitch  y->roll  --->ero
	nav_Data.velX = 	nav_Data.holdSpeedN * navUkfData.yawCos + 	nav_Data.holdSpeedE * navUkfData.yawSin;
	nav_Data.velY = 	nav_Data.holdSpeedE * navUkfData.yawCos - 	nav_Data.holdSpeedN * navUkfData.yawSin;

	out_temp[PITr]=nav_Data.velX;
	out_temp[ROLr]=nav_Data.velY;
	
	tar_point_globler[0]=tar_point_globler[0];
	tar_point_globler[1]=tar_point_globler[1];
				if(gps_target_change){
					if(tar_point_globle[0]!=tar_now_gps[0]||tar_point_globle[1]!=tar_now_gps[1])
						gps_target_change=0;
					}
	}	
	
	
	//0  ROL  1 PIT
	if(gpsx.gpssta>=1&&gpsx.rmc_mode=='A'){//定位有效
	nav_gps[ROLr]=LIMIT(out_temp[ROLr],-MAX_GPS,MAX_GPS);
	nav_gps[PITr]=LIMIT(out_temp[PITr],-MAX_GPS,MAX_GPS);
  }
	else
	{	
	nav_gps[ROLr]=0;
	nav_gps[PITr]=0;
	}
		/*
		north    LAT=1     V_West+                             __________
		|   Y+  y                                              P- R- GPS-
		                                        
		|P+
	
    _____	R	+   x         LON=0  V_East+
	   
head  |    1 PIT y-  RC0
			| 
		   _____  0 ROL x+   RC 1
	
		*/
		
}
u8 get_qr_pos=0;
float qr_local_pos[3];
 void Estimate_land_marker_local_position(float qr_posx,float qr_posy,float qr_yaw,float drone_pos_n_local,float drone_pos_e_local,float drone_yaw,float T)
{
float qr_yaw_in_global;
qr_yaw_in_global=To_180_degrees(drone_yaw-qr_yaw);
float cy=cos(qr_yaw_in_global*0.0173);	
float sy=sin(qr_yaw_in_global*0.0173);		
float x_temp= qr_posx*cy + qr_posy*sy;
float y_temp=	-qr_posx*sy+ qr_posy*cy;
qr_local_pos[2]=sqrt(y_temp*y_temp+x_temp*x_temp);
qr_local_pos[North]=drone_pos_n_local-y_temp;
qr_local_pos[East]=drone_pos_e_local-x_temp;	
}
float drone_local_pos[2];
void Estimate_drone_local_position(double lat_drone,double lon_drone,double lat_cor,double lon_cor,float T)
{
    drone_local_pos[North] = (lat_drone - lat_cor) * navUkfData.r1;
    drone_local_pos[East] =  (lon_drone - lon_cor) * navUkfData.r2;
}
float tar_drone_local_pos[2];
void Estimate_drone_target_local_position(double tar_lat_drone,double tar_lon_drone,double lat_cor,double lon_cor,float T)
{
    tar_drone_local_pos[North] = (tar_lat_drone - lat_cor) * navUkfData.r1;
    tar_drone_local_pos[East] =  (tar_lon_drone - lon_cor) * navUkfData.r2;
}
//LED  		0    						 1    						 2
//Red    状态机不对       没图像						DJI 未连接
//Gre																		  DJI 连接没卫星
//Wit										 有图像					  正常
float yaw_use_gimbal_v;
u8 gps_test=0;
void GPS_Qr_Control(nmea_msg *gpsx_in,float T)
{
	 static u8 init,state;
	float out_temp[2];
	u8 set_gps_point;
	if(!init){init=1;
		gps_pid.kp=0.125;//0.2;
		gps_pid.ki=0.00;//01;
		gps_pid.kd=1.888;

	qr_pid.kp=0.125;//0.2;
	qr_pid.ki=0.00;//01;
	qr_pid.kd=1.888/2;
	}	
	yaw_use_gimbal_v=To_180_degrees(m100.Yaw-(float)(PWM_DJ[1]-1500)/250.*65);
	
	#if USE_M100
	
	 if(m100.Rc_pit>9000&m100.Rc_rol>9000&&m100.Rc_yaw<-9000&&!dji_rst_protect&&state_v==SG_LOW_CHECK&&!en_vrc)
		 set_point1=1;
	 else if(fabs(m100.Rc_pit)<1500&&fabs(m100.Rc_rol)<1500&&fabs(m100.Rc_yaw)<1500)
		 set_point1=0;
	 
	 lon = m100.Lon;
	 lat = m100.Lat;
	 if(m100.GPS_STATUS>=3&&!dji_rst_protect)
	 gpsx.gpssta=1;
	 else
	 gpsx.gpssta=0;	 
	 gpsx.rmc_mode='A';
	#else
	 lon = gpsx.longitude;
	 lat = gpsx.latitude;
  #endif	
	//if(1){
	if(lat!=0&&lon!=0&&gpsx.gpssta>=1&&gpsx.rmc_mode=='A'||gps_test){//有效数据执行控制
	gps_data.latitude=lat;
	gps_data.longitude=lon;
	gps_data.angle=gpsx.angle;	
	
	set_gps_point=mode.set_point1;
  set_gps_point=set_point1;		
	static u16 cnt_delay;	
	switch(state_set_point)	
	{
		case 0:
		if(set_gps_point)	
		{
		state_set_point=1;cnt_delay=0;
		
		}
		
		break;
		case 1:
		check_way_point[0]=gps_data.latitude;
		check_way_point[1]=gps_data.longitude;
		if(!set_gps_point)//&&check_way_point[0]!=gps_data.latitude&&check_way_point[1]!=gps_data.latitude)	
		{
		state_set_point=2;
		}
		break;
		case 2:
		if(set_gps_point)//&&check_way_point[0]!=gps_data.latitude&&check_way_point[1]!=gps_data.latitude)	
		{
		if(cnt_delay++>10){cnt_delay=0;	
		state_set_point=3;
	}
		}
		break;
		case 3:
		way_point[0][0]=gps_data.latitude;
		way_point[0][1]=gps_data.longitude;
		if(!set_gps_point)//&&check_way_point[0]!=gps_data.latitude&&check_way_point[1]!=gps_data.latitude)	
		{
		state_set_point=4;
		}
		break;
		case 4:
		if(set_gps_point)//&&way_point[0][0]!=gps_data.latitude&&way_point[0][1]!=gps_data.latitude)	
		{
		if(cnt_delay++>10){cnt_delay=0;		
		state_set_point=5;
		
		}
		}
		break;
	  case 5:
		way_point[1][0]=gps_data.latitude;
		way_point[1][1]=gps_data.longitude;	
		if(!set_gps_point)	
		{
		if(cnt_delay++>10){cnt_delay=0;	
		Yaw_set_dji=To_180_degrees(m100.Yaw);	
		state_set_point=0;
		//if(KEY[3])
		WRITE_PARM();
		}
		}
		
		break;	
	}
	if(circle.check&&circle.connect&&(state_v==SU_TO_CHECK_POS||state_v==SU_CHECK_TAR)&&get_qr_pos==0)	
	{
		qr_gps_pos[0]=gps_data.latitude;
		qr_gps_pos[1]=gps_data.longitude;
	}
	
	gps_local_cor_zero[0]=check_way_point[0];
	gps_local_cor_zero[1]=check_way_point[1];
	float yaw_use;
  #if USE_M100
  yaw_use=Moving_Median(23,3,m100.Yaw);
  #else
  yaw_use=Yaw;		
  #endif	

	u8 flag;
	if(Rc_Pwm_Inr_mine[RC_PITCH]<OFF_RC_PIT-80||Rc_Pwm_Inr_mine[RC_PITCH]>OFF_RC_PIT+80||
	Rc_Pwm_Inr_mine[RC_ROLL]<OFF_RC_ROL-80||Rc_Pwm_Inr_mine[RC_ROLL]>OFF_RC_ROL+80)
	flag=1;
	else 
	flag=0;
	
	navUkfCalcEarthRadius(lat);
  if(flag)//悬停测试	
  navUkfSetHereAsPositionTarget();	
	float y[3];

	
	if(mode.en_gps)//设置期望航点 GPS航线飞行
	navUkfSetLocalPositionTarget( tar_point_globle[0], tar_point_globle[1]);	
	if(tar_pos_gps[0]!=0||tar_pos_gps[1]!=0) 
	navUkfSetHereAsPositionTarget();//navUkfSetLocalPositionTarget( gps_data.latitude, gps_data.longitude);
	
  

	Estimate_drone_local_position( lat, lon, gps_local_cor_zero[North], gps_local_cor_zero[East], T);
	Estimate_drone_target_local_position( navUkfData.holdLat, navUkfData.holdLon, gps_local_cor_zero[North], gps_local_cor_zero[East], T);
	if(gps_test)
	{
	drone_local_pos[0]=drone_local_pos[1]=0;
	}
	if(circle.check&&circle.connect)	
	{	get_qr_pos=1;
		
	float qr_posx=(float)qr.x/100.;
	float qr_posy=(float)qr.y/100.;
	float qr_posz=(float)qr.z/100.;
	float qr_yaw=qr.yaw;
	float yaw_use_gimbal=To_180_degrees(yaw_use-0*(float)(PWM_DJ[1]-1500)/250.*65);
	//	yaw_use_gimbal_v=yaw_use_gimbal;
		Estimate_land_marker_local_position( qr_posx, qr_posy, qr_yaw, drone_local_pos[North], drone_local_pos[East], yaw_use_gimbal_v, T);
	}
 	
	if(fabs(y[North]>3000)||fabs(y[East]>3000))//1km的限制
	{
	 navUkfSetHereAsPositionTarget();
	 Estimate_drone_target_local_position( navUkfData.holdLat, navUkfData.holdLon, gps_local_cor_zero[North], gps_local_cor_zero[East], T);	
	}	
	
	if(mode.use_qr_as_gps_tar)//目标为qr 局部坐标
	{
		
	tar_drone_local_pos[North]=qr_local_pos[North];
	tar_drone_local_pos[East]= qr_local_pos[East];

	}	
	
  y[North]=drone_local_pos[North]-tar_drone_local_pos[North];
  y[East]= drone_local_pos[East] -tar_drone_local_pos[East];
 
	navUkfData.yawCos=cos(0.0173*yaw_use);
	navUkfData.yawSin=sin(0.0173*yaw_use);
	
	//0-->wei  1 -->jing
	//filter
	
	
	nav_Data.gps_ero_dis_lpf[North]=flt_gps*Moving_Median(28,3,y[North]*1000)+(1-flt_gps)*nav_Data.gps_ero_dis_lpf[North];
	nav_Data.gps_ero_dis_lpf[East]=flt_gps*Moving_Median(29,3,y[East]*1000)+(1-flt_gps)*nav_Data.gps_ero_dis_lpf[East];
	
	
//	15075434@qq.com  yang-->15877918559
	int max_ero0,max_ero1;
	if(fabs(nav_Data.gps_ero_dis_lpf[North])>800*4)
		max_ero0=600*2;
	else
		max_ero0=600;
	
	if(fabs(nav_Data.gps_ero_dis_lpf[East])>800*4)
		max_ero1=600*2;
	else
		max_ero1=600;
	//PID
  //N ->0
	if(gps_pid.ki==0||flag)gps_ctrl[0].err_i=0;
	gps_ctrl[0].err = ( gps_pid.kp*LIMIT(my_deathzoom(nav_Data.gps_ero_dis_lpf[0],15),-max_ero0,max_ero0) );//mm
	gps_ctrl[0].err_i += gps_pid.ki *gps_ctrl[0].err *T;
	gps_ctrl[0].err_i = LIMIT(gps_ctrl[0].err_i,-1 *ULTRA_INT,1 *ULTRA_INT);
	gps_ctrl[0].err_d = gps_pid.kd *( 0.0f *(-(0*1000)*T) + 1.0f *(gps_ctrl[0].err - gps_ctrl[0].err_old) );
	gps_ctrl[0].pid_out = gps_ctrl[0].err + gps_ctrl[0].err_i + gps_ctrl[0].err_d;
	gps_ctrl[0].pid_out = LIMIT(gps_ctrl[0].pid_out,-1000,1000);
	gps_ctrl[0].err_old = gps_ctrl[0].err;
	//E ->1
	if(gps_pid.ki==0||flag)gps_ctrl[1].err_i=0;
	gps_ctrl[1].err = ( gps_pid.kp*LIMIT(my_deathzoom(nav_Data.gps_ero_dis_lpf[1],15),-max_ero1,max_ero1) );//mm
	gps_ctrl[1].err_i += gps_pid.ki *gps_ctrl[1].err *T;
	gps_ctrl[1].err_i = LIMIT(gps_ctrl[1].err_i,-1 *ULTRA_INT,1 *ULTRA_INT);
	gps_ctrl[1].err_d = gps_pid.kd *( 0.0f *(-(0*1000)*T) + 1.0f *(gps_ctrl[1].err - gps_ctrl[1].err_old) );
	gps_ctrl[1].pid_out = gps_ctrl[1].err + gps_ctrl[1].err_i + gps_ctrl[1].err_d;
	gps_ctrl[1].pid_out = LIMIT(gps_ctrl[1].pid_out,-1000,1000);
	gps_ctrl[1].err_old = gps_ctrl[1].err;
	
	
	nav_Data.holdSpeedN = -gps_ctrl[0].pid_out*0.4 ;
	nav_Data.holdSpeedE = -gps_ctrl[1].pid_out*0.4 ;

	//x-> pitch  y->roll  --->ero
	nav_Data.velX = 	nav_Data.holdSpeedN * navUkfData.yawCos + 	nav_Data.holdSpeedE * navUkfData.yawSin;
	nav_Data.velY = 	nav_Data.holdSpeedE * navUkfData.yawCos - 	nav_Data.holdSpeedN * navUkfData.yawSin;

	out_temp[PITr]=nav_Data.velX;
	out_temp[ROLr]=nav_Data.velY;
	
	tar_point_globler[0]=tar_point_globler[0];
	tar_point_globler[1]=tar_point_globler[1];
				if(gps_target_change){
					if(tar_point_globle[0]!=tar_now_gps[0]||tar_point_globle[1]!=tar_now_gps[1])
						gps_target_change=0;
					}
	}	
	
	

	//0  ROL  1 PIT
	if(gpsx.gpssta>=1&&gpsx.rmc_mode=='A'){//定位有效
	nav_gps[ROLr]=out_temp[ROLr];
	nav_gps[PITr]=out_temp[PITr];
  }
	else
	{	
	nav_gps[ROLr]=0;
	nav_gps[PITr]=0;
	}
	float ero_qr_pix[2];
	static float ero_qr_pixr[2];
		if(mode.land_by_pix&&
			((fabs(circle.x-160)<160*0.8&&fabs(circle.y-120)<120*0.8)||
			(avoid_color[0]||avoid_color[1]||avoid_color[2]||avoid_color[3]))
		   &&state_v==SD_CIRCLE_MID_DOWN)//使用图像对准
	{
		if(circle.check){
		ero_qr_pix[ROLr]=my_deathzoom(circle.x-160,5)*((float)qr.z/100.);
		ero_qr_pix[PITr]=-my_deathzoom(circle.y-120,5)*((float)qr.z/100.);	
		nav_gps[ROLr]=ero_qr_pix[ROLr]*qr_pid.kp+(ero_qr_pix[ROLr] - ero_qr_pixr[ROLr])*qr_pid.kd;
	  nav_gps[PITr]=ero_qr_pix[PITr]*qr_pid.kp+(ero_qr_pix[PITr] - ero_qr_pixr[PITr])*qr_pid.kd;
		ero_qr_pixr[ROLr]=ero_qr_pix[ROLr];	
		ero_qr_pixr[PITr]=ero_qr_pix[PITr];	
		}
		else
		{
		ero_qr_pixr[ROLr]=0;	
		ero_qr_pixr[PITr]=0;		
		nav_gps[ROLr]=0;
	  nav_gps[PITr]=0;
		}	
	}
	nav_gps[ROLr]=LIMIT(nav_gps[ROLr],-MAX_GPS,MAX_GPS);
	nav_gps[PITr]=LIMIT(nav_gps[PITr],-MAX_GPS,MAX_GPS);
		/*
		north    LAT=1     V_West+                             __________
		|   Y+  y                                              P- R- GPS-
		                                        
		|P+
	
    _____	R	+   x         LON=0  V_East+
	   
head  |    1 PIT y-  RC0
			| 
		   _____  0 ROL x+   RC 1
	
		*/
		
}