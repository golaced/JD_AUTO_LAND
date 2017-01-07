/*********************************************************************************
*                                ɽè�ɿأ�Lynx��
*                             for LynxFly under GPLv2
*
* COPYRIGHT (C) 2012 - 2013, Lynx 84693469@qq.com
*
* Version   	: V1.0
* By        	: Lynx@sia 84693469@qq.com
*
* For       	: Stm32f405RGT6
* Mode      	: Thumb2
* Description   : �������ÿ����������װ
*
*				
* Date          : 2013.XX.XX
*******************************************************************************/
#include "ControlFGR.h"


//һЩ�������ѧ����
#define NAN	 (0.0/0.0)	 //��ЧֵNaN
#define isNAN(x)	 ((x)!=(x))	
#define INF	 (1.0/0.0)	 //�����
#define PINF	 INF	 //�������
#define NINF	 -INF	 //�������
#define isINF(x)	 (((x)==PINF)||((x)==NINF))
#define isPINF(x)	 ((x)==PINF)
#define isNINF(x)	 ((x)==NINF)


/*
 * Name										: ControlFGR
 * Description						: ����F G R����ϵͳ����ʽ���������������FΪ�������������ʽ��RΪ��������������ʽ��GΪ�������������ʽ
 * Entry                  : ��0Ϊ��ǰʱ�̣�����Ŀ�����У�����ϵͳ������У�����ϵͳ�������У���һ��F���У�F���г��ȣ�G���У�G���г��ȣ�R���У�R���г���
 * Return                 : ������
 * Author									: lynx 84693469@qq.com.
 *
 * History
 * ----------------------
 * Rev										: 0.00
 * Date										: 06/15/2013
 *
 * create.
 * ----------------------
 */
float ControlFGR(float CDataYr[], float CDataY[], float CDataU[], float F[], int lengthF, float G[], int lengthG, float R[], int lengthR) {
	
	int i = 0;  //�㶮��
	
	float CSF = 0;  //������͵���ʱ����
	float CSG = 0;
	float CSR = 0;
	
	float tmpOut = 0;
	
	CSF = 0;
	for(i=1;i<lengthF;i++){  //ע���Ǵ�1��ʼ�� ��һ��������
		CSF -= F[i]*CDataU[i];  //ע�⸺�ţ�ע����ʼλ��
	}
	
	CSR = 0;
	for(i=0;i<lengthR;i++){  // �Ժ�Ҫ������ǰ��Ӧ�Ļ�����͵���취�ĸ���
		CSR += R[i]*CDataYr[i];  
	}
	
	CSG = 0;
	for(i=0;i<lengthG;i++){  //ע���Ǵ�1��ʼ��
		CSG -= G[i]*CDataY[i];  //ע�⸺��
	}
	
	//return (CSF+CSG+CSR)/F[0];  //���ظ�ͨ����ֵ ��Ϊ�����������������
	tmpOut = CSF+CSG+CSR;
	if(isNAN(tmpOut)){
		tmpOut = 0;  //��һЩ������ʾ��������ԵĻ�
	}
	return tmpOut;  //���ظ�ͨ����ֵ
	
}

