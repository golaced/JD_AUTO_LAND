#ifndef _STC_PP_PID_h_
#define _STC_PP_PID_h_

#include "CFFRELS.h"  //��Ҫ�õ�����������С����
#include "ControlFGR.h"  //�������õĿ��������ʽ


void updateSTCFilterOnly(CFFRELS_T* cffrelsIn, float CDataYFK[], float CDataUFK[], float CDataYE[], float CDataU[]);

void updateSTCPPPID(CFFRELS_T* cffrelsIn, float CDataYFK[], float CDataUFK[], float CDataYE[], float CDataU[]);

float getControlSTCPPPID(CFFRELS_T* cffrelsIn, float CDataYr[], float CDataY[], float CDataU[]);




#endif
