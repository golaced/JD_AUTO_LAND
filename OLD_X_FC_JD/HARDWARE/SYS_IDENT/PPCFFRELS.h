#ifndef _PPCFFRELS_h_
#define _PPCFFRELS_h_

//����������С���˹��Ƶ���ض���

#define  PPCFFRELS_ML_A 10  //�����ʶ�� ȫ�ֳ�ʼ���� �����;�������ά�ȣ���ȷ�����ֵ����ÿһ��RELS�ṹ���ڵ�ML



//----------------Data struct---------------------------//

typedef struct {
//varitable
	
	//�⼸������ʼ���Ժ��ǧ��Ҫ���Ҹ��ˣ����ã�����
	int NA;      //����ϵͳA�Ľ״�
	int NB;      //����ϵͳB�Ľ״�
	int D;       //d�ӳ�
	
	//�м����
	int NF;      //F�Ľ���
	int NG;      //G�Ľ���
	int NR;      //R�Ľ���
	int NAM;     //Am�Ľ״�
	int NA0;     //A0�Ľ״�
	int NAA;     //A0*Am�Ľ״�
	int ML;      // ML=NF+1+NG+1 �����ʶ�������;�������ά��(��Ϊnf1ҲҪ��ʶ1��Ϊbe0����ҲҪ��1)
	float AM[1][PPCFFRELS_ML_A];   //Ŀ��ϵͳ��ĸ����ʽϵ��Am
	float A0[1][PPCFFRELS_ML_A];   //Ŀ��ϵͳ��ĸ����ʽϵ��A0
	float AA[1][PPCFFRELS_ML_A];   //Ŀ��ϵͳ��ĸ����ʽϵ��A0*Am
	float BE0;                   //be0
	float FE[1][PPCFFRELS_ML_A];   //��������F
	float GE[1][PPCFFRELS_ML_A];   //��������G
	float R[1][PPCFFRELS_ML_A];                     //��������R
	
	//�۲��õľ���
	float phie[PPCFFRELS_ML_A][1];  //��
	float P[PPCFFRELS_ML_A][PPCFFRELS_ML_A];  //��λ��P
	float thetae_1[PPCFFRELS_ML_A][1];  //�ȵ�ǰһ�ε�ֵ����ʼ��Ϊ��ֵ
	float thetae[PPCFFRELS_ML_A][1];  //�ȵĳ�ֵ
	float K[PPCFFRELS_ML_A][1];  //K
	

//function none
}PPCFFRELS_T;



void PPCFFRELS_init(PPCFFRELS_T* relsIn, int na, int nb, int d, float am1, float am2, float am3);
void PPCFFRELS_initAsSteady(PPCFFRELS_T* relsIn, int na, int nb, int d, float am1, float am2, float am3);
void PPCFFRELS_Update(PPCFFRELS_T* relsIn, float CDataYFK[], float CDataUFK[], float lambda);
void PPCFFRELS_ClcFGR(PPCFFRELS_T* relsIn);



















#endif
