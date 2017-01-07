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
* Description   : ֱ�ӷ���У����������PID������
*
*				
* Date          : 2013.XX.XX
*******************************************************************************/
#include "STC_PP_PID.h"


//����洢�Ŀ��������ݵĳ��ȣ���ʷ�͵�ǰ����ǰ������Ϊ0�������ֵһ���ô���2����ΪĬ��U��һ���ӳ�
#define I_CONTROL_DATA_LENGTH 10  

//�����������������ݵĴ洢
// float CDataUfk[I_CONTROL_DATA_LENGTH];   //ע��������matlab����˳��Ĳ��죬��β��k�Ķ��Ǵ���һʱ�̿�ʼ�����ݣ��� ��һ�� ��yk(1)��Ӧ���� �ڶ��� ����yk[1]
// float CDataYfk[I_CONTROL_DATA_LENGTH];   //ע��������matlab����˳��Ĳ��죬��β��k�Ķ��Ǵ���һʱ�̿�ʼ�����ݣ��� ��һ�� ��yk(1)��Ӧ���� �ڶ��� ����yk[1]

// float RELS_tmp_V1[CFFRELS_ML_A][1];  //���ھ����������ʱ���� ����
// float RELS_tmp_VT1[1][CFFRELS_ML_A];  //���ھ����������ʱ���� ת������
// float RELS_tmp_UM1[1][1];  //���ھ����������ʱ���� ��λ����



/*
 * Name										: updateSTCFilterOnly
 * Description						: ֱ�ӷ�����������У��PID�Ϳ����� ��ʶ�˲������ݵĸ���
 * Entry                  : cffrelsIn ֱ�ӷ��Ĺ۲����ṹ�壬��0Ϊ��ǰʱ�̣��˲����������У��˲�����������У�����ϵͳ������У�����ϵͳ��������
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
void updateSTCFilterOnly(CFFRELS_T* cffrelsIn, float CDataYFK[], float CDataUFK[], float CDataYE[], float CDataU[]){
	//������¼�м�ֵ����ʱ����
	float tmpFSumAAuf = 0;  //-AA(2:naa+1)*ufk(d+1:d+naa)
	float tmpFSumAAyf = 0;  //-AA(2:naa+1)*yfk(d+1:d+naa)
	float tmpFSumFu = 0;    //deltaF*uk(d:d+ndf)
	float tmpFSumY = 0;     //yk(d)
	
	int i = 0;   //�㶮��
	
	//-AA(2:naa+1)*ufk(d+1:d+naa)
	tmpFSumAAuf = 0;
	for(i=0;i<cffrelsIn->NAA;i++){
		tmpFSumAAuf -= cffrelsIn->AA[0][i+1]*CDataUFK[cffrelsIn->D+i+1]; //ע�⸺�� //NND����1  �������� ҪɱҪ�������
	}
	//deltaF*uk(d:d+ndf)
	tmpFSumFu = 0;
	tmpFSumFu += 1*CDataU[cffrelsIn->D];  //NND����1  �������� ҪɱҪ�������
	tmpFSumFu += -1*CDataU[cffrelsIn->D+1];
	
	//-AA(2:naa+1)*yfk(d+1:d+naa)
	tmpFSumAAyf = 0;
	for(i=0;i<cffrelsIn->NAA;i++){
		tmpFSumAAyf -= cffrelsIn->AA[0][i+1]*CDataYFK[cffrelsIn->D+i+1]; //ע�⸺�� //NND����1  �������� ҪɱҪ�������
	}
	//yk(d)
	tmpFSumY = CDataYE[cffrelsIn->D];   //yk(1)��ӦYE[1]
	
	CDataUFK[cffrelsIn->D] = tmpFSumAAuf+tmpFSumFu;  //ufk(d)=-AA(2:naa+1)*ufk(d+1:d+naa)+deltaF*uk(d:d+ndf); %�˲�������� �����Ѿ����ǡ�
	CDataYFK[cffrelsIn->D] = tmpFSumAAyf+tmpFSumY;   //yfk(d)=-AA(2:naa+1)*yfk(d+1:d+naa)+yk(d);
}




/*
 * Name										: updateSTCPPPID
 * Description						: ֱ�ӷ�����������У��PID�Ϳ����� ��ʶ�˲������ݵĸ��²��۲�õ�����������
 * Entry                  : cffrelsIn ֱ�ӷ��Ĺ۲����ṹ�壬��0Ϊ��ǰʱ�̣��˲����������У��˲�����������У�����ϵͳ��������
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
void updateSTCPPPID(CFFRELS_T* cffrelsIn, float CDataYFK[], float CDataUFK[], float CDataYE[], float CDataU[]){
	//�ȸ����˲���
	updateSTCFilterOnly(cffrelsIn, CDataYFK, CDataUFK, CDataYE, CDataU);
	//���й۲�
	CFFRELS_Update(cffrelsIn, CDataYFK, CDataUFK, 0.995);   //����ȡ��=1
}




/*
 * Name										: getControlSTCPPPID
 * Description						: ֱ�ӷ�����������У��PID�Ϳ����� ��ȡ����������Ŀ�����
 * Entry                  : cffrelsIn ֱ�ӷ��Ĺ۲����ṹ�壬��0Ϊ��ǰʱ�̣�����Ŀ�����У�����ϵͳ������У�����ϵͳ��������
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
 * Rev										: 0.00
 * Date										: 06/18/2013
 *
 * cffrelsIn->NR        ->         cffrelsIn->NR+1
 * ----------------------
 */
float getControlSTCPPPID(CFFRELS_T* cffrelsIn, float CDataYr[], float CDataY[], float CDataU[]) {
	
	float tmpOut = 0;
	
	tmpOut = ControlFGR(CDataYr, CDataY, CDataU, (float*)cffrelsIn->FE, cffrelsIn->NF+1, (float*)cffrelsIn->GE, cffrelsIn->NG+1, (float*)cffrelsIn->R, cffrelsIn->NR+1);
	
	return tmpOut;  //���ظ�ͨ����ֵ
	
}







