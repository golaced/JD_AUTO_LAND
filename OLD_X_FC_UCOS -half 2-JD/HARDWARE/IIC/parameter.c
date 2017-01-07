

#include "include.h"
#include "mpu6050.h"
#include "hml5833l.h"
#include "att.h"
#include "string.h"
#include "height_ctrl.h"

#define SENSOR_SETUP_FILE      "sensor.bin"
#define PID_SETUP_FILE         "pid.bin"
u8 flash_init_error;
static sensor_setup_t sensor_setup;
static pid_setup_t pid_setup;


static void  Param_SetSettingToFC(void) //fly thr 45% at 4s full 4.4Ah
{
	memcpy(&mpu6050.Acc_Offset,&sensor_setup.Offset.Accel,sizeof(xyz_f_t));
	memcpy(&mpu6050.Gyro_Offset,&sensor_setup.Offset.Gyro,sizeof(xyz_f_t));
	memcpy(&ak8975.Mag_Offset,&sensor_setup.Offset.Mag,sizeof(xyz_f_t));
	mpu6050.Temprea_Offset = sensor_setup.Offset.Temperature;
//--------------------------------------Angle--------------------------------------------------------------------
	//p
	#if PLANE_IS_BIG
	pid_setup.groups.ctrl2.roll.kp=pid_setup.groups.ctrl2.pitch.kp =0.8;	
	#else
  pid_setup.groups.ctrl2.roll.kp=pid_setup.groups.ctrl2.pitch.kp =0.425;//0.425;//<-----------mini0.45 for 4050  0.4 for 6030 WT
	#endif
	SPID.IP=pid_setup.groups.ctrl2.roll.kp*1000;
	//i
	pid_setup.groups.ctrl2.roll.ki =pid_setup.groups.ctrl2.pitch.ki = 0.05;
	SPID.II=pid_setup.groups.ctrl2.roll.ki*1000;
	//d
	#if PLANE_IS_BIG
	pid_setup.groups.ctrl2.roll.kd  = pid_setup.groups.ctrl2.pitch.kd = 0.3;
	#else
	pid_setup.groups.ctrl2.roll.kd  = pid_setup.groups.ctrl2.pitch.kd = 0.3;//<-------------mini
	#endif
	SPID.ID=pid_setup.groups.ctrl2.roll.kd*1000;
	//---------------------------------------GRO----------------------------------------------------------------------
	//p
	#if PLANE_IS_BIG
	pid_setup.groups.ctrl1.roll.kp=pid_setup.groups.ctrl1.pitch.kp = 0.8;	
	#else
	pid_setup.groups.ctrl1.roll.kp=pid_setup.groups.ctrl1.pitch.kp = 0.425;//0.35;//<-----------mini  WT
	#endif
	SPID.OP=pid_setup.groups.ctrl1.roll.kp*1000;
	//i
  pid_setup.groups.ctrl1.roll.ki =pid_setup.groups.ctrl1.pitch.ki = 0.1;
	SPID.OI=pid_setup.groups.ctrl1.roll.ki*1000;
	//d
	pid_setup.groups.ctrl1.roll.kd=pid_setup.groups.ctrl1.pitch.kd = 2.0;
	SPID.OD=pid_setup.groups.ctrl1.roll.kd*1000;
	//YAW------------------------------------------------------------------------------
	pid_setup.groups.ctrl2.yaw.kp   = 0.8;//0.4;//0.8;	
	SPID.YD=pid_setup.groups.ctrl2.yaw.kp*1000;
	pid_setup.groups.ctrl2.yaw.ki   = 0.05;	
	SPID.YI=pid_setup.groups.ctrl2.yaw.ki*1000;
  pid_setup.groups.ctrl2.yaw.kd   = 0;// 0.3;//0.1;//0.3;
	//------------------------------------
	pid_setup.groups.ctrl1.yaw.kp   = 0.8;//1.2;//0.8;
	SPID.YP=pid_setup.groups.ctrl1.yaw.kp*1000;	
	
	pid_setup.groups.ctrl1.yaw.ki   = 0.1;//1.0;//0.5;//0.1;	
	pid_setup.groups.ctrl1.yaw.kd   = 1.0;//1.0;//1.5;	
	pid_setup.groups.ctrl1.roll.kdamp=pid_setup.groups.ctrl1.pitch.kdamp = 1;
	pid_setup.groups.ctrl1.yaw.kdamp   = 1;
	//--------------------------------------------光流定位-----------------------------------------------------------------------------
	  pid.nav.out.p=0;//3.5/2;//4;////0.35;
		pid.nav.out.i=0;//8;//(wt)
		pid.nav.in.p =0.4;//3.5;//0.365;//0.350;//0.4;//fix 2015.10.21 导航均值滤波 阴天//0.025;//150
		pid.nav.in.i =0.1;

		pid.nav.in.d =0.2;//0.250;
		pid.nav.out.dead= 0;//0.4;//位置死区mm
		pid.nav.in.dead=  0.025;//3;//速度死区mm/s
		pid.nav.in.dead2=  pid.nav.in.dead*1.5;//4.5;//速度积分死区mm/s
//-----------------------------壁障----------------------------------
	pid.avoid.out.p=0.1;
//-------------------------------------------------------------------
  memcpy(&ctrl_1.PID[PIDROLL],&pid_setup.groups.ctrl1.roll,sizeof(pid_t));
	memcpy(&ctrl_1.PID[PIDPITCH],&pid_setup.groups.ctrl1.pitch,sizeof(pid_t));
	memcpy(&ctrl_1.PID[PIDYAW],&pid_setup.groups.ctrl1.yaw,sizeof(pid_t));
	
	
	memcpy(&ctrl_2.PID[PIDROLL],&pid_setup.groups.ctrl2.roll,sizeof(pid_t));
	memcpy(&ctrl_2.PID[PIDPITCH],&pid_setup.groups.ctrl2.pitch,sizeof(pid_t));
	memcpy(&ctrl_2.PID[PIDYAW],&pid_setup.groups.ctrl2.yaw,sizeof(pid_t));
}

void Para_Init()
{
	Param_SetSettingToFC();
	Ctrl_Para_Init();
	WZ_Speed_PID_Init();
	Ultra_PID_Init();
}



