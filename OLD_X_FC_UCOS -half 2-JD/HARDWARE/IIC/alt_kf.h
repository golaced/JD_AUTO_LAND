#ifndef _alt_ukf_h
#define _alt_ukf_h

#include "include.h"
#include "arm_math_m.h"
#define ALT_S           3   // states
#define ALT_M           1   // measurements
#define ALT_V           2   // process noise
#define ALT_N           1   // measurement noise

#define ALT_STATE_POS   0
#define ALT_STATE_VEL   1
#define ALT_STATE_BIAS  2

#define ALT_NOISE_BIAS  0
#define ALT_NOISE_VEL   1

#define ALT_POS         altUkfData.x[ALT_STATE_POS]
#define ALT_VEL         altUkfData.x[ALT_STATE_VEL]
#define ALT_BIAS        altUkfData.x[ALT_STATE_BIAS]

#define ALT_POS_BMP         	altUkfData_bmp.x[ALT_STATE_POS]
#define ALT_VEL_BMP         	altUkfData_bmp.x[ALT_STATE_VEL]
#define ALT_BIAS_BMP        	altUkfData_bmp.x[ALT_STATE_BIAS]
#define ALT_POS_SONAR         altUkfData_sonar.x[ALT_STATE_POS]
#define ALT_VEL_SONAR         altUkfData_sonar.x[ALT_STATE_VEL]
#define ALT_BIAS_SONAR        altUkfData_sonar.x[ALT_STATE_BIAS]
extern float ALT_POS_SONAR2,ALT_POS_SONAR3,ALT_POS_SONAR_HEAD_LASER_SCANER,ALT_VEL_BMP_EKF;
#define NAV_POS_N         	altUkfData_nav_N.x[ALT_STATE_POS]
#define NAV_VEL_N         	altUkfData_nav_N.x[ALT_STATE_VEL]
#define NAV_BIAS_N        	altUkfData_nav_N.x[ALT_STATE_BIAS]


#define SRCDKF_H	(__sqrtf(3.0f) * 3.0f)
#define SRCDKF_RM	0.0001f		// Robbins-Monro stochastic term
typedef float float32_t;


typedef void SRCDKFTimeUpdate_t(float32_t *x_I, float32_t *noise_I, float32_t *x_O, float32_t *u, float32_t dt, int n);
typedef void SRCDKFMeasurementUpdate_t(float32_t *u, float32_t *x, float32_t *noise_I, float32_t *y);

// define all temporary storage here so that it does not need to be allocated each iteration
typedef struct {
	int S;
	int V;
	int M;		// only used for parameter estimation
	int N;		// only used for parameter estimation
	int L;

	float32_t h;
	float32_t hh;
	float32_t w0m, wim, wic1, wic2;
	float32_t rm;

	arm_matrix_instance_f32 Sx;	// state covariance
	arm_matrix_instance_f32 SxT;	// Sx transposed
	arm_matrix_instance_f32 Sv;	// process noise
	arm_matrix_instance_f32 Sn;	// observation noise
	arm_matrix_instance_f32 x;	// state estimate vector
	arm_matrix_instance_f32 Xa;	// augmented sigma points
	float32_t *xIn;
	float32_t *xNoise;
	float32_t *xOut;
	arm_matrix_instance_f32 qrTempS;
	arm_matrix_instance_f32 Y;	// resultant measurements from sigma points
	arm_matrix_instance_f32 y;	// measurement estimate vector
	arm_matrix_instance_f32 qrTempM;
	arm_matrix_instance_f32 Sy;	// measurement covariance
	arm_matrix_instance_f32 SyT;	// Sy transposed
	arm_matrix_instance_f32 SyC;	// copy of Sy
	arm_matrix_instance_f32 Pxy;
	arm_matrix_instance_f32 C1;
	arm_matrix_instance_f32 C1T;
	arm_matrix_instance_f32 C2;
	arm_matrix_instance_f32 D;
	arm_matrix_instance_f32 K;
	arm_matrix_instance_f32 KT;	// only used for param est
	arm_matrix_instance_f32 inov;	// inovation
	arm_matrix_instance_f32 inovT;// only used for param est
	arm_matrix_instance_f32 xUpdate;
	arm_matrix_instance_f32 qrFinal;
	arm_matrix_instance_f32 rDiag;
	arm_matrix_instance_f32 Q, R, AQ;	// scratch

	SRCDKFTimeUpdate_t *timeUpdate;
	SRCDKFMeasurementUpdate_t *map;	// only used for param est
} srcdkf_t;

typedef struct {
    srcdkf_t *kf;
    float *x;               // states
} altUkfStruct_t;

extern altUkfStruct_t altUkfData,altUkfData_sonar,altUkfData_bmp,altUkfData_nav_N;

extern void altUkfInit(void);
extern void altUkfProcess(float measuredPres);
extern float brao_ukf,thr_mine2;
extern void alt_spd(float alt,float spd,float thr);

extern srcdkf_t *srcdkfInit(int s, int m, int v, int n, SRCDKFTimeUpdate_t *timeUpdate) ;
extern float *srcdkfGetState(srcdkf_t *f) ;
extern void srcdkfSetVariance(srcdkf_t *f, float32_t *q, float32_t *v, float32_t *n, int nn);
extern void srcdkfMeasurementUpdate(srcdkf_t *f, float32_t *u, float32_t *ym, int M, int N,
	float32_t *noise, SRCDKFMeasurementUpdate_t *measurementUpdate) ;
extern void srcdkfTimeUpdate(srcdkf_t *f, float32_t *u, float32_t dt) ;
extern void *aqDataCalloc(uint16_t count, uint16_t size) ;
#endif
