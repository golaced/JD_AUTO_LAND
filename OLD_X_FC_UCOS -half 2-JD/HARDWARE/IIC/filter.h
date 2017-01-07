#ifndef __FILTER_H
#define __FILTER_H

#include "include.h"
//float Moving_Average(u8 item,u8 width_num,float in);
void Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out);
float Moving_Median(u8 item,u8 width_num,float in);
extern double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na);
extern fp32 LPF_1st(fp32 oldData, fp32 newData, fp32 lpf_factor);
#define _xyz_f_t xyz_f_t
void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out);
#endif
