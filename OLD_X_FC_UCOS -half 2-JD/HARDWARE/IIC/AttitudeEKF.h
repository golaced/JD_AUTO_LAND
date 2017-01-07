/*
 * AttitudeEKF.h
 *
 * Code generation for function 'AttitudeEKF'
 *
 * C source code generated on: Thu Aug 21 11:17:28 2014
 *
 */

#ifndef __ATTITUDEEKF_H__
#define __ATTITUDEEKF_H__
/* Include files */
#include <math.h>
#include "include.h"

/* Function Declarations */
extern void AttitudeEKF(unsigned char approx_prediction, unsigned char use_inertia_matrix, const unsigned char zFlag[3], float dt, const float z[9], float q_rotSpeed, float q_rotAcc, float q_acc, float q_mag, float r_gyro, float r_accel, float r_mag, const float J[9], float xa_apo[12], float Pa_apo[144], float Rot_matrix[9], float eulerAngles[3], float debugOutput[4]);
extern void AttitudeEKF_initialize(void);
extern void AttitudeEKF_terminate(void);


/*=======================================================================* 
 * Fixed width word size data types:                                     * 
 *   int8_T, int16_T, int32_T     - signed 8, 16, or 32 bit integers     * 
 *   uint8_T, uint16_T, uint32_T  - unsigned 8, 16, or 32 bit integers   * 
 *   real32_T, real64_T           - 32 and 64 bit floating point numbers * 
 *=======================================================================*/

typedef signed char int8_T;
typedef unsigned char uint8_T;
typedef short int16_T;
typedef unsigned short uint16_T;
typedef int int32_T;
typedef unsigned int uint32_T;
typedef float real32_T;
typedef double real64_T;

/*===========================================================================* 
 * Generic type definitions: real_T, time_T, boolean_T, int_T, uint_T,       * 
 *                           ulong_T, char_T and byte_T.                     * 
 *===========================================================================*/

typedef double real_T;
typedef double time_T;
typedef unsigned char boolean_T;
typedef int int_T;
typedef unsigned int uint_T;
typedef unsigned long ulong_T;
typedef char char_T;
typedef char_T byte_T;

/*===========================================================================* 
 * Complex number type definitions                                           * 
 *===========================================================================*/
#define CREAL_T	
   typedef struct {  
      real32_T re;  
      real32_T im;  
   } creal32_T;  

   typedef struct {  
      real64_T re;  
      real64_T im;  
   } creal64_T;  

   typedef struct {  
      real_T re;  
      real_T im;  
   } creal_T;  

   typedef struct {  
      int8_T re;  
      int8_T im;  
   } cint8_T;  

   typedef struct {  
      uint8_T re;  
      uint8_T im;  
   } cuint8_T;  

   typedef struct {  
      int16_T re;  
      int16_T im;  
   } cint16_T;  

   typedef struct {  
      uint16_T re;  
      uint16_T im;  
   } cuint16_T;  

   typedef struct {  
      int32_T re;  
      int32_T im;  
   } cint32_T;  

   typedef struct {  
      uint32_T re;  
      uint32_T im;  
   } cuint32_T;  


/*=======================================================================* 
 * Min and Max:                                                          * 
 *   int8_T, int16_T, int32_T     - signed 8, 16, or 32 bit integers     * 
 *   uint8_T, uint16_T, uint32_T  - unsigned 8, 16, or 32 bit integers   * 
 *=======================================================================*/

#define MAX_int8_T  	((int8_T)(127))
#define MIN_int8_T  	((int8_T)(-128))
#define MAX_uint8_T 	((uint8_T)(255))
#define MIN_uint8_T 	((uint8_T)(0))
#define MAX_int16_T 	((int16_T)(32767))
#define MIN_int16_T 	((int16_T)(-32768))
#define MAX_uint16_T	((uint16_T)(65535))
#define MIN_uint16_T	((uint16_T)(0))
#define MAX_int32_T 	((int32_T)(2147483647))
#define MIN_int32_T 	((int32_T)(-2147483647-1))
#define MAX_uint32_T	((uint32_T)(0xFFFFFFFFU))
#define MIN_uint32_T	((uint32_T)(0))
int attitude_estimator_ekf_thread_main(void);
struct attitude_estimator_ekf_params {
	float r[3];
	float q[4];
	float moment_inertia_J[9];
	int32_t use_moment_inertia;
	float roll_off;
	float pitch_off;
	float yaw_off;
	float mag_decl;
	int acc_comp;
};

struct attitude_estimator_ekf_param_handles {
	double r0, r1, r2;
	double q0, q1, q2, q3;
	double moment_inertia_J[3]; /**< diagonal entries of the matrix */
	double use_moment_inertia;
	double mag_decl;
	double acc_comp;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct attitude_estimator_ekf_param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct attitude_estimator_ekf_param_handles *h, struct attitude_estimator_ekf_params *p);

#endif
/* End of code generation (AttitudeEKF.h) */
