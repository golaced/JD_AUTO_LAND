#ifndef __HMC5883L_H
#define __HMC5883L_H

#include "../HARDWARE/define.h"
#include "../HARDWARE/parameter.h"
#include "stdbool.h"

#define HMC58X3_ADDR 0x3C // 7 bit address of the HMC58X3 used with the Wire library
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2
// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_R_XM 3
#define HMC58X3_R_XL 4

#define HMC58X3_R_YM (7)  //!< Register address for YM.
#define HMC58X3_R_YL (8)  //!< Register address for YL.
#define HMC58X3_R_ZM (5)  //!< Register address for ZM.
#define HMC58X3_R_ZL (6)  //!< Register address for ZL.

#define HMC58X3_R_STATUS 9
#define HMC58X3_R_IDA 10
#define HMC58X3_R_IDB 11
#define HMC58X3_R_IDC 12

#define HMC5883L_GAIN_1370          0x00
#define HMC5883L_GAIN_1090          0x01
#define HMC5883L_GAIN_820           0x02
#define HMC5883L_GAIN_660           0x03
#define HMC5883L_GAIN_440           0x04
#define HMC5883L_GAIN_390           0x05
#define HMC5883L_GAIN_330           0x06
#define HMC5883L_GAIN_220           0x07

#define HMC5883L_RATE_0P75          0x00
#define HMC5883L_RATE_1P5           0x01
#define HMC5883L_RATE_3             0x02
#define HMC5883L_RATE_7P5           0x03
#define HMC5883L_RATE_15            0x04
#define HMC5883L_RATE_30            0x05
#define HMC5883L_RATE_75            0x06
extern unsigned char HMC5883_calib;
extern int16_t  HMC5883_maxx,HMC5883_maxy,HMC5883_maxz,
		 HMC5883_minx,HMC5883_miny,HMC5883_minz;

void HMC5883L_SetUp(void);	//��ʼ��
void HMC58X3_getID(char id[3]);	//��оƬID
void HMC58X3_getValues(int16_t *x,int16_t *y,int16_t *z); //��ADC
void HMC58X3_mgetValues(float *arry); //IMU ר�õĶ�ȡ������ֵ
void HMC58X3_getlastValues(int16_t *x,int16_t *y,int16_t *z);
void HMC5883L_Start_Calib(void);
void HMC5883L_Save_Calib(void);
void HMC_FIX_TEST(void);
void HMC_FIX(u8 state);
extern void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z) ;



#define CALIBRATING_MAG_CYCLES              20  //У׼ʱ�����20s

#define AK8975_ADDRESS         0x0c	// 0x18

#define AK8975_WIA     0x00
#define AK8975_HXL     0x03
#define AK8975_HXH     0x04
#define AK8975_HYL     0x05
#define AK8975_HYH     0x06
#define AK8975_HZL     0x07
#define AK8975_HZH     0x08
#define AK8975_CNTL    0x0A

typedef struct 
{ 
	xyz_s16_t Mag_Adc;			//����ֵ
	xyz_f_t Mag_Offset;		//ƫ��ֵ
	xyz_f_t 	Mag_Gain;			//��������	
  xyz_f_t 	Mag_Val;			//�������ֵ
	u8 Mag_CALIBRATED;
}ak8975_t;

extern ak8975_t ak8975,ak8975_fc;

bool ANO_AK8975_Run(void);
void ANO_AK8975_CalOffset_Mag(void);
void ANO_AK8975_Read(void);


extern u8 Mag_CALIBRATED,Mag_CALIBRATED_R;
extern u8 ak8975_ok,HMC5883_calib;
#endif

//------------------End of File----------------------------
