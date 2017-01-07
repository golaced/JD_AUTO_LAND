#ifndef _RELS_h_
#define _RELS_h_

//����������С���˹��Ƶ���ض���

#define  RELS_ML_A 6  //�����ʶ�� ȫ�ֳ�ʼ���� �����;�������ά�ȣ���ȷ�����ֵ����ÿһ��RELS�ṹ���ڵ�ML



//----------------Data struct---------------------------//

typedef struct {
//varitable
	
	//�⼸������ʼ���Ժ��ǧ��Ҫ���Ҹ��ˣ����ã�����
	int NA;      //a�Ľ�����һ
	int NB;      //b�Ľ�����һ
	int NC;      //c�Ľ�����һ
	int D;       //d�ӳ�
	int ML;      // ML=NA+NB+1+NC �����ʶ�������;�������ά��
	
	float phie[RELS_ML_A][1];  //��
	float P[RELS_ML_A][RELS_ML_A];  //��λ��P
	float thetae_1[RELS_ML_A][1];  //�ȵ�ǰһ�ε�ֵ����ʼ��Ϊ��ֵ
	float thetae[RELS_ML_A][1];  //�ȵĳ�ֵ
	float K[RELS_ML_A][1];  //K
	float xie;   //�����Ĺ���ֵ
	

//function none
}RELS_T;



void RELS_init(RELS_T* relsIn, int na, int nb, int nc, int d);
float RELS_Update(RELS_T* relsIn, float CDataYE[], float CDataU[], float CDataXiek[]);
float RELS_Observ(RELS_T* relsIn, float CDataYE[], float CDataU[], float CDataXiek[]);
float FFRELS_Update(RELS_T* relsIn, float CDataYE[], float CDataU[], float CDataXiek[], float lambda);












#endif
