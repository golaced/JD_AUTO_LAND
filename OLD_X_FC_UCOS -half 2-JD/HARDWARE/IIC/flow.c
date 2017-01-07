
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "flow.h"
#include "filter.h"
#include "ultrasonic.h"
#include "my_math.h"
#include "usart.h"
//#define __ASM asm

#include "core_cm4_simd.h"

float flt=0.3;//0.7258315;//0.235;
float scale_groz=0.1,scale_grox=0.3;//0.4;
float scale_groh=7;
float scale_acc=0.09;//0.2;
float dead_acc=0.002;
float dead_gro=0.055;


u8 flow_fps;

double pixel_flow_x = 0.0f,pixel_flow_x_gro_fix,pixel_flow_y_gro_fix;
double pixel_flow_y = 0.0f;
double pixel_flow_x2 = 0.0f;
double pixel_flow_y2 = 0.0f;
double flow_compx = 0;
double flow_compy = 0;
float pixel_flow_x_pix,pixel_flow_y_pix;
u8 deg_delay=3;
float ultra_distance_flow;
float get_time_between_images;
u8 flow_task_uart(void)
{
u8 qual;
	static float pos_acc_flow[2],pos_flow[2],spd[2];	
	static float apr[3];
	static u8 init,led;
	static float hc_acc_i,wz_speed_0,wz_speed_1,wz_speed_old;
	float pixel_flow_x_t2,pixel_flow_y_t2;
	float f_com[2];
	float tempx,tempy,tempx1,tempy1;
	get_time_between_images = Get_Cycle_T(GET_T_FLOW_UART);	//Get_Cycle_T(0);
	pixel_flow_x_pix=pixel_flow_x=-10*(flow.flow_y.origin) / ((16) / (4.0f * 6.0f) * 1000.0f) / (get_time_between_images);

pixel_flow_x=pixel_flow_x_pix;//Moving_Median(0,3,);
pixel_flow_y_gro_fix=0;//LIMIT((my_deathzoom(flow_rad.integrated_xgyro,dead_gro)*10000*scale_grox),LIMIT(-fabs(pixel_flow_x_pix),-1000,-0.5),LIMIT(fabs(pixel_flow_x_pix),0.5,1000));
pixel_flow_x-=-pixel_flow_y_gro_fix
;//+pixel_flow_x*(fabs(my_deathzoom(mpu6050.Gyro_deg.z,30))*ANGLE_TO_RADIAN *scale_groz);
//+LIMIT(pixel_flow_x*(fabs((float)LIMIT(my_deathzoom(bmp_speed,100)/1000.,-1.5,1.5)) *scale_groh),-150,150);
pixel_flow_y_pix=pixel_flow_y=10*(flow.flow_x.origin) / ((16) / (4.0f * 6.0f) * 1000.0f) / (get_time_between_images);

pixel_flow_y=pixel_flow_y_pix;//Moving_Median(1,3,);//s2.x;
pixel_flow_x_gro_fix=0;//LIMIT((my_deathzoom(flow_rad.integrated_ygyro,dead_gro)*10000*scale_grox),LIMIT(-fabs(pixel_flow_y_pix),-1000,-0.5),LIMIT(fabs(pixel_flow_y_pix),0.5,1000));
pixel_flow_y-=-pixel_flow_x_gro_fix
;//+my_deathzoom(flow_rad.integrated_zgyro,0)*10000*scale_groz;
;//+LIMIT(pixel_flow_y*(fabs((float)LIMIT(my_deathzoom(bmp_speed,100)/1000.,-1.5,1.5)) *scale_groh),-150,150);


flow_compx=flow_compx*(1-flt)+pixel_flow_x*flt; //IIR_I_Filter( pixel_flow_x, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
flow_compy=flow_compy*(1-flt)+pixel_flow_y*flt;  //IIR_I_Filter(pixel_flow_y, InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);

if(!ultra_ok)
	ultra_distance_flow=0.9;
else
	ultra_distance_flow=(ultra_distance/1000.)*0.1+0.9*ultra_distance_flow;
	f_com[0]=flow_compx*ultra_distance_flow*scale_acc/10;
	f_com[1]=flow_compy*ultra_distance_flow*scale_acc/10;

pixel_flow_x2=LIMIT(f_com[0],-5.25,5.25);
pixel_flow_y2=LIMIT(f_com[1],-5.25,5.25);

return	qual;
}