#ifndef _STC_PP_h_
#define _STC_PP_h_

#include "PPCFFRELS.h"  //��Ҫ�õ�����������С����
#include "ControlFGR.h"  //�������õĿ��������ʽ


void updateSTCPPFilterOnly(PPCFFRELS_T* cffrelsIn, float CDataYFK[], float CDataUFK[], float CDataYE[], float CDataU[]);

void updateSTCPP(PPCFFRELS_T* cffrelsIn, float CDataYFK[], float CDataUFK[], float CDataYE[], float CDataU[]);

float getControlSTCPP(PPCFFRELS_T* cffrelsIn, float CDataYr[], float CDataY[], float CDataU[]);




#endif
