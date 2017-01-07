#ifndef __FLOW_H
#define __FLOW_H	 
#include "stm32f4xx.h" 
#include "include.h" 
	
extern u8 flow_task_uart(void);


extern	double pixel_flow_x,pixel_flow_x_gro_fix;
extern float pixel_flow_x_pix,pixel_flow_y_pix;
extern	double pixel_flow_y;
extern	double pixel_flow_x2;
extern	double pixel_flow_y2;
extern double flow_compx ;
extern double flow_compy ;
extern float get_time_between_images;
extern float wz_speed_flow[2];



//----px4flow

  typedef struct
{
	float average;//Flow in m in x-sensor direction, angular-speed compensated
	float averager;//Flow in m in x-sensor direction, angular-speed compensated
	float originf;
	int16_t origin;
}FLOW_DATA;


  typedef struct
{
	uint64_t  time_sec;
	u8   id;
	FLOW_DATA flow_x;
	FLOW_DATA flow_y;
	FLOW_DATA flow_comp_x;//Flow in m in x-sensor direction, angular-speed compensated
	FLOW_DATA flow_comp_y;
	u8 quality; //Optical flow quality / confidence. 0: bad, 255: maximum quality
	FLOW_DATA hight;//ground_distance	float	Ground distance in m. Positive value: distance known. Negative value: Unknown distance    
  u8 new_data_flag;	
}FLOW;

 typedef struct
{
 uint64_t time_usec; ///< Timestamp (microseconds, synced to UNIX time or since system boot)
 uint32_t integration_time_us; ///< Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
 float integrated_x; ///< Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
 float integrated_y; ///< Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
 float integrated_xgyro; ///< RH rotation around X axis (rad)
 float integrated_ygyro; ///< RH rotation around Y axis (rad)
 float integrated_zgyro; ///< RH rotation around Z axis (rad)
 uint32_t time_delta_distance_us; ///< Time in microseconds since the distance was sampled.
 float distance; ///< Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
 int16_t temperature; ///< Temperature * 100 in centi-degrees Celsius
 uint8_t sensor_id; ///< Sensor ID
 uint8_t quality; ///< Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
}FLOW_RAD;
extern FLOW flow;
extern FLOW_RAD flow_rad;
extern void FLOW_MAVLINK(unsigned char data);
#endif











