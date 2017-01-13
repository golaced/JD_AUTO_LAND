#ifndef __CIRCLE_H
#define __CIRCLE_H	 
#include "stm32f4xx.h" 
#include "include.h" 
#include "gps.h"
 typedef struct
{
int x,y,z,dis;
int center_x,center_y;
float pit,rol,yaw;
int r;
int x_flp,y_flp,z_flp;
u8 check;
u8 connect,lose_cnt;
int control[2];
float control_k;
float control_k_miss; 
float control_yaw;
float forward;
float forward_end_dj_pwm;
u8 dj_fly_line;
}CIRCLE;
extern CIRCLE circle,track,mouse,qr;
extern float nav_circle[2],nav_land[2],qr_pos_off[2];
void circle_control(float T);
#define MID_Y 125
#define MID_X 140
extern float circle_use[2],yaw_use_gimbal_v;
extern float  integrator[2];
void  GPS_hold(nmea_msg *gpsx_in,float T);

 typedef struct
{
 float Pit,Rol,Yaw;
 double Lat,Lon;
 float H,H_Spd;	
 float Spd[2];
 u8 GPS_STATUS;	//>3  5->Best
/*
Flight status val	status name
1	standby
2	take_off
3	in_air
4	landing
5	finish_landing
*/
 u8 STATUS;
 float Bat;	
 int Rc_pit,Rc_rol,Rc_yaw,Rc_thr,Rc_mode,Rc_gear;

}M100;
extern M100 m100;
#define East 1
#define North 0
extern double gps_local_cor_zero[2];//局部GPS坐标系原点
extern float qr_local_pos[3],drone_local_pos[2], qr_local_pos1[3];
extern u8 get_qr_pos;
extern float tar_drone_local_pos[2];
extern _st_height_pid_v qr_ctrl[2];
extern _st_height_pid qr_pid;
void GPS_Qr_Control(nmea_msg *gpsx_in,float T);
#endif











