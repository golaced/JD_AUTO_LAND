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
* Description   : ��С���˱�ʶ��װ
*
*				
* Date          : 2013.XX.XX
*******************************************************************************/
#include "RELS.h"
#include "LibMatrix.h"   //�����


/*
 * Name										: RELS_init
 * Description						: ��ʹ�ñ�ʶ֮ǰ��Ҫ���øú�����ʼ������ָ��4����Ҫ�ı�����ע�������ʶ�ĳ�ֵû�����û�ָ��
 * Entry                  : RELS_T�Ľṹ��ָ�룬a�Ľ�����һ��b�Ľ�����һ��c�Ľ�����һ�������ӳ�d
 * Return                 : void
 * Author									: lynx 84693469@qq.com.
 *
 * History
 * ----------------------
 * Rev										: 0.00
 * Date										: 05/31/2013
 *
 * create.
 * ----------------------
 */
void RELS_init(RELS_T* relsIn, int na, int nb, int nc, int d)
{
	int RELS_i = 0;  //ѭ���õı���
	float RELS_tmp_M1[RELS_ML_A][RELS_ML_A];  //���ھ����������ʱ���� ����
	
	if(na<=0 && nb<=0 && nc<=0 && d<=0){
		relsIn->NA = 2;  //���δ�����ĸ��ؼ�������ʹ��Ĭ�ϲ���
		relsIn->NB = 1;
		relsIn->NC = 1;
		relsIn->D = 3;
	}else{
		relsIn->NA = na;  //���ṹ�帳ֵ
		relsIn->NB = nb;
		relsIn->NC = nc;
		relsIn->D = d;
	}
	//����ML
	relsIn->ML = relsIn->NA+relsIn->NB+1+relsIn->NC;
	//RELS��ʼ��
	relsIn->xie = 0;
	//thetae_1=0.001*ones(na+nb+1+nc,1);%�ǳ�С���������˴�����Ϊ0��
	for(RELS_i=0;RELS_i<relsIn->ML;RELS_i++){   
		relsIn->thetae_1[RELS_i][0] = 0.001;  //0.001*1
	}
	//��Ϊ��thetae�ĸ�����ǰ�ˣ�����Ͷ�һ����ʼ����ֹthetae��0
	matrix_copy((float*)relsIn->thetae_1, relsIn->ML, 1, (float*)relsIn->thetae);
	//P=10^6*eye(na+nb+1+nc);  ��ʼ��P
	matrix_eye((float*)RELS_tmp_M1, relsIn->ML);
	matrix_multiply_k((float*)RELS_tmp_M1, 1000000.0, relsIn->ML, relsIn->ML, (float*)relsIn->P); 
}

/*
 * Name										: RELS_Update
 * Description						: ��������������ݸ��µ���������С���˵ı�ʶ��������������۲⵽������ֵ
 * Entry                  : RELS_T�Ľṹ��ָ�룬ϵͳ�����ʱ�����У�����Ϊ0Խ��Խ��ȥ����ϵͳ�����ʱ�����У�����Ϊ0Խ��Խ��ȥ����ϵͳ������ʱ�����У�����Ϊ0Խ��Խ��ȥ��
 * Return                 : ���ݱ��ν���۲⵽��ϵͳ����
 * Author									: lynx 84693469@qq.com.
 *
 * History
 * ----------------------
 * Rev										: 0.00
 * Date										: 05/31/2013
 *
 * create.
 * ----------------------
 * Rev										: 0.00
 * Date										: 06/15/2013
 * CDataU[1+relsIn->D+RELS_i];  //������һ��       ->         CDataU[relsIn->D+RELS_i];  //U��һ������һ������
 * create.
 * ----------------------
 */
float RELS_Update(RELS_T* relsIn, float CDataYE[], float CDataU[], float CDataXiek[])
{
	//�����ȶ��������������ַ������ʱֻ�������Ƶķ�Χ
	float RELS_tmp_V1[RELS_ML_A][1];  //���ھ����������ʱ���� ����
	float RELS_tmp_VT1[1][RELS_ML_A];  //���ھ����������ʱ���� ת������
	float RELS_tmp_VT2[1][RELS_ML_A];  //���ھ����������ʱ���� ת������
	float RELS_tmp_M1[RELS_ML_A][RELS_ML_A];  //���ھ����������ʱ���� ����
	float RELS_tmp_M2[RELS_ML_A][RELS_ML_A];  //���ھ����������ʱ���� ����
	float RELS_tmp_M3[RELS_ML_A][RELS_ML_A];  //���ھ����������ʱ���� ����
	float RELS_tmp_M4[RELS_ML_A][RELS_ML_A];  //���ھ����������ʱ���� ����
	float RELS_tmp_UM1[1][1];  //���ھ����������ʱ���� ��λ����
	float RELS_tmp_U1 = 0;  //���ھ����������ʱ���� �м����
	int RELS_i = 0;  //ѭ���õı���
	
	//----------------------------------------------------------------------------------------
	//thetae_1=thetae(:,k);  ��䱻������Ƶ������Ϊ�˱�֤����ļ��У�������ֲ
	matrix_copy((float*)relsIn->thetae, relsIn->ML, 1, (float*)relsIn->thetae_1);
	//���ȹ���۲�������   phie=[-yk;uk(d:d+nb);xiek];
	//ע������ʵ�ʺ�����һ������Ϊ����Ҫ�ù�ȥ��ֵ��Ԥ�Ȿ�ε������������ȷ���ṩ�����ݳ����㹻
	for(RELS_i=0;RELS_i<relsIn->NA;RELS_i++){   //-yk
		relsIn->phie[RELS_i][0] = -CDataYE[1+RELS_i];  //������һ��
	}
	for(RELS_i=0;RELS_i<(relsIn->NB+1);RELS_i++){   //uk(d:d+nb)
		relsIn->phie[relsIn->NA+RELS_i][0] = CDataU[relsIn->D+RELS_i];  //U��һ������һ������
	}
	for(RELS_i=0;RELS_i<relsIn->NC;RELS_i++){   //xiek
		relsIn->phie[relsIn->NA+(relsIn->NB+1)+RELS_i][0] = CDataXiek[1+RELS_i];  //������һ��
	}
	//K=P*phie/(1+phie'*P*phie);
	matrix_transpose((float*)relsIn->phie, relsIn->ML, 1, (float*)RELS_tmp_VT1);  //phie'
	matrix_multiply((float*)RELS_tmp_VT1, (float*)relsIn->P, 1, relsIn->ML, relsIn->ML, (float*)RELS_tmp_VT2); //phie'*P
	matrix_multiply((float*)RELS_tmp_VT2, (float*)relsIn->phie, 1, relsIn->ML, 1, (float*)RELS_tmp_UM1); //phie'*P*phie
	RELS_tmp_U1 = 1/(1+RELS_tmp_UM1[0][0]);  ///(1+phie'*P*phie)
	matrix_multiply((float*)relsIn->P, (float*)relsIn->phie, relsIn->ML, relsIn->ML, 1, (float*)RELS_tmp_V1); //P*phie
	matrix_multiply_k((float*)RELS_tmp_V1, RELS_tmp_U1, relsIn->ML, 1, (float*)relsIn->K);  //K=P*phie/(1+phie'*P*phie);
	//thetae(:,k)=thetae_1+K*(y(k)-phie'*thetae_1);
	matrix_multiply((float*)RELS_tmp_VT1, (float*)relsIn->thetae_1, 1, relsIn->ML, 1, (float*)RELS_tmp_UM1); //phie'*thetae_1  ǰ���Ѿ����phie'�ˣ�����ֱ����,ǰ��ע�Ᵽ��
	RELS_tmp_U1 = CDataYE[0]-RELS_tmp_UM1[0][0];  //y(k)-phie'*thetae_1
	matrix_multiply_k((float*)relsIn->K, RELS_tmp_U1, relsIn->ML, 1, (float*)RELS_tmp_V1);  //K*(y(k)-phie'*thetae_1)
	matrix_addition((float*)relsIn->thetae_1, (float*)RELS_tmp_V1, relsIn->ML, 1, (float*)relsIn->thetae);  //thetae(:,k)=thetae_1+K*(y(k)-phie'*thetae_1);
	//P=(eye(na+nb+1+nc)-K*phie')*P;
	matrix_multiply((float*)relsIn->K, (float*)RELS_tmp_VT1, relsIn->ML, 1, relsIn->ML, (float*)RELS_tmp_M1); //K*phie' ǰ���Ѿ����phie'�ˣ�����ֱ����,ǰ��ע�Ᵽ��
	matrix_eye((float*)RELS_tmp_M2, relsIn->ML);  //eye(na+nb+1+nc)
	matrix_minus((float*)RELS_tmp_M2, (float*)RELS_tmp_M1, relsIn->ML, relsIn->ML, (float*)RELS_tmp_M3);   //(eye(na+nb+1+nc)-K*phie')
	matrix_multiply((float*)RELS_tmp_M3, (float*)relsIn->P, relsIn->ML, relsIn->ML, relsIn->ML, (float*)RELS_tmp_M4);  //(eye(na+nb+1+nc)-K*phie')*P
	matrix_copy((float*)RELS_tmp_M4, relsIn->ML, relsIn->ML, (float*)relsIn->P);  //P=(eye(na+nb+1+nc)-K*phie')*P;
	//xie=y(k)-phie'*thetae(:,k);
	matrix_multiply((float*)RELS_tmp_VT1, (float*)relsIn->thetae, 1, relsIn->ML, 1, (float*)RELS_tmp_UM1);  //phie'*thetae(:,k)  ǰ���Ѿ����phie'�ˣ�����ֱ����,ǰ��ע�Ᵽ��
	relsIn->xie = CDataYE[0] - RELS_tmp_UM1[0][0];  //xie=y(k)-phie'*thetae(:,k);

// 					%����������С���˷�
// 					phie=[-yk;uk(d:d+nb);xiek];
// 					K=P*phie/(1+phie'*P*phie);
// 					thetae(:,k)=thetae_1+K*(y(k)-phie'*thetae_1);
// 					P=(eye(na+nb+1+nc)-K*phie')*P;
// 					
// 					xie=y(k)-phie'*thetae(:,k);%�������Ĺ���ֵ
// 					
// 					%��ȡ��ʶ����
// 					ae=[1 thetae(1:na,k)']; be=thetae(na+1:na+nb+1,k)'; ce=[1 thetae(na+nb+2:na+nb+1+nc,k)'];
// 					if abs(ce(2))>0.9
// 							ce(2)=sign(ce(2))*0.9;
// 					end
	
	return relsIn->xie;  //���ﷵ��ֵ�ǹ۲⵽�ı��ε�����
	
}

/*
 * Name										: RELS_Observ
 * Description						: ����������ݽṹ��֮ǰ��ʶ�õ���ϵͳ�������۲�����
 * Entry                  : RELS_T�Ľṹ��ָ�룬ϵͳ�����ʱ�����У�����Ϊ0Խ��Խ��ȥ����ϵͳ�����ʱ�����У�����Ϊ0Խ��Խ��ȥ����ϵͳ������ʱ�����У�����Ϊ0Խ��Խ��ȥ��
 * Return                 : �������һ�α�ʶ�����ϵͳ�۲�õ�����������
 * Author									: lynx 84693469@qq.com.
 *
 * History
 * ----------------------
 * Rev										: 0.00
 * Date										: 05/31/2013
 *
 * create.
 * ----------------------
 * Rev										: 0.00
 * Date										: 06/15/2013
 * CDataU[1+relsIn->D+RELS_i];  //������һ��       ->         CDataU[relsIn->D+RELS_i];  //U��һ������һ������
 * create.
 * ----------------------
 */
float RELS_Observ(RELS_T* relsIn, float CDataYE[], float CDataU[], float CDataXiek[])
{
	//�����ȶ��������������ַ������ʱֻ�������Ƶķ�Χ
	float RELS_tmp_VT1[1][RELS_ML_A];  //���ھ����������ʱ���� ת������
	float RELS_tmp_UM1[1][1];  //���ھ����������ʱ���� ��λ����
	int RELS_i = 0;  //ѭ���õı���
	
	//���ȹ���۲�������   phie=[-yk;uk(d:d+nb);xiek];
	//ע������ʵ�ʺ�����һ������Ϊ����Ҫ�ù�ȥ��ֵ��Ԥ�Ȿ�ε������������ȷ���ṩ�����ݳ����㹻
	for(RELS_i=0;RELS_i<relsIn->NA;RELS_i++){   //-yk
		relsIn->phie[RELS_i][0] = -CDataYE[1+RELS_i];  //������һ��
	}
	for(RELS_i=0;RELS_i<(relsIn->NB+1);RELS_i++){   //uk(d:d+nb)
		relsIn->phie[relsIn->NA+RELS_i][0] = CDataU[relsIn->D+RELS_i];  //U��һ������һ������
	}
	for(RELS_i=0;RELS_i<relsIn->NC;RELS_i++){   //xiek
		relsIn->phie[relsIn->NA+(relsIn->NB+1)+RELS_i][0] = CDataXiek[1+RELS_i];  //������һ��
	}
	
	
	matrix_transpose((float*)relsIn->phie, relsIn->ML, 1, (float*)RELS_tmp_VT1);  //phie'
	matrix_multiply((float*)RELS_tmp_VT1, (float*)relsIn->thetae, 1, relsIn->ML, 1, (float*)RELS_tmp_UM1);  //phie'*thetae(:,k)  ǰ���Ѿ����phie'�ˣ�����ֱ����,ǰ��ע�Ᵽ��
	relsIn->xie = CDataYE[0] - RELS_tmp_UM1[0][0];  //xie=y(k)-phie'*thetae(:,k);

	
	return relsIn->xie;  //���ﷵ��ֵ�ǹ۲⵽�ı��ε�����
}

/*
 * Name										: FFRELS_Update
 * Description						: �������������С���˱�ʶ����������������ݸ��µ���������С���˵ı�ʶ��������������۲⵽������ֵ��ֻ�������������ӣ����ಿ����RELSͨ�ã�
 * Entry                  : RELS_T�Ľṹ��ָ�룬ϵͳ�����ʱ�����У�����Ϊ0Խ��Խ��ȥ����ϵͳ�����ʱ�����У�����Ϊ0Խ��Խ��ȥ����ϵͳ������ʱ�����У�����Ϊ0Խ��Խ��ȥ�����������ӣ�0.9-1��
 * Return                 : ���ݱ��ν���۲⵽��ϵͳ����
 * Author									: lynx 84693469@qq.com.
 *
 * History
 * ----------------------
 * Rev										: 0.00
 * Date										: 05/31/2013
 *
 * create.
 * ----------------------
 * Rev										: 0.00
 * Date										: 06/15/2013
 * CDataU[1+relsIn->D+RELS_i];  //������һ��       ->         CDataU[relsIn->D+RELS_i];  //U��һ������һ������
 * create.
 * ----------------------
 */
float FFRELS_Update(RELS_T* relsIn, float CDataYE[], float CDataU[], float CDataXiek[], float lambda)
{
	//�����ȶ��������������ַ������ʱֻ�������Ƶķ�Χ
	float RELS_tmp_V1[RELS_ML_A][1];  //���ھ����������ʱ���� ����
	float RELS_tmp_VT1[1][RELS_ML_A];  //���ھ����������ʱ���� ת������
	float RELS_tmp_VT2[1][RELS_ML_A];  //���ھ����������ʱ���� ת������
	float RELS_tmp_M1[RELS_ML_A][RELS_ML_A];  //���ھ����������ʱ���� ����
	float RELS_tmp_M2[RELS_ML_A][RELS_ML_A];  //���ھ����������ʱ���� ����
	float RELS_tmp_M3[RELS_ML_A][RELS_ML_A];  //���ھ����������ʱ���� ����
	float RELS_tmp_M4[RELS_ML_A][RELS_ML_A];  //���ھ����������ʱ���� ����
	float RELS_tmp_UM1[1][1];  //���ھ����������ʱ���� ��λ����
	float RELS_tmp_U1 = 0;  //���ھ����������ʱ���� �м����
	int RELS_i = 0;  //ѭ���õı���
	
	//�淶������
	if(lambda>1){
		lambda = 1;
	}else if(lambda<0.9){
		lambda = 0.9;
	}
	
	//----------------------------------------------------------------------------------------
	//thetae_1=thetae(:,k);  ��䱻������Ƶ������Ϊ�˱�֤����ļ��У�������ֲ
	matrix_copy((float*)relsIn->thetae, relsIn->ML, 1, (float*)relsIn->thetae_1);
	//���ȹ���۲�������   phie=[-yk;uk(d:d+nb);xiek];
	//ע������ʵ�ʺ�����һ������Ϊ����Ҫ�ù�ȥ��ֵ��Ԥ�Ȿ�ε������������ȷ���ṩ�����ݳ����㹻
	for(RELS_i=0;RELS_i<relsIn->NA;RELS_i++){   //-yk
		relsIn->phie[RELS_i][0] = -CDataYE[1+RELS_i];  //������һ��
	}
	for(RELS_i=0;RELS_i<(relsIn->NB+1);RELS_i++){   //uk(d:d+nb)
		relsIn->phie[relsIn->NA+RELS_i][0] = CDataU[relsIn->D+RELS_i];  //U��һ������һ������
	}
	for(RELS_i=0;RELS_i<relsIn->NC;RELS_i++){   //xiek
		relsIn->phie[relsIn->NA+(relsIn->NB+1)+RELS_i][0] = CDataXiek[1+RELS_i];  //������һ��
	}
	//K=P*phie/(lambda+phie'*P*phie);
	matrix_transpose((float*)relsIn->phie, relsIn->ML, 1, (float*)RELS_tmp_VT1);  //phie'
	matrix_multiply((float*)RELS_tmp_VT1, (float*)relsIn->P, 1, relsIn->ML, relsIn->ML, (float*)RELS_tmp_VT2); //phie'*P
	matrix_multiply((float*)RELS_tmp_VT2, (float*)relsIn->phie, 1, relsIn->ML, 1, (float*)RELS_tmp_UM1); //phie'*P*phie
	RELS_tmp_U1 = 1.0/(lambda+RELS_tmp_UM1[0][0]);  ///(lambda+phie'*P*phie)
	matrix_multiply((float*)relsIn->P, (float*)relsIn->phie, relsIn->ML, relsIn->ML, 1, (float*)RELS_tmp_V1); //P*phie
	matrix_multiply_k((float*)RELS_tmp_V1, RELS_tmp_U1, relsIn->ML, 1, (float*)relsIn->K);  //K=P*phie/(lambda+phie'*P*phie);
	//thetae(:,k)=thetae_1+K*(y(k)-phie'*thetae_1);
	matrix_multiply((float*)RELS_tmp_VT1, (float*)relsIn->thetae_1, 1, relsIn->ML, 1, (float*)RELS_tmp_UM1); //phie'*thetae_1  ǰ���Ѿ����phie'�ˣ�����ֱ����,ǰ��ע�Ᵽ��
	RELS_tmp_U1 = CDataYE[0]-RELS_tmp_UM1[0][0];  //y(k)-phie'*thetae_1
	matrix_multiply_k((float*)relsIn->K, RELS_tmp_U1, relsIn->ML, 1, (float*)RELS_tmp_V1);  //K*(y(k)-phie'*thetae_1)
	matrix_addition((float*)relsIn->thetae_1, (float*)RELS_tmp_V1, relsIn->ML, 1, (float*)relsIn->thetae);  //thetae(:,k)=thetae_1+K*(y(k)-phie'*thetae_1);
	//P=(eye(nf+ng+2)-K*phie')*P/lambda;
	matrix_multiply((float*)relsIn->K, (float*)RELS_tmp_VT1, relsIn->ML, 1, relsIn->ML, (float*)RELS_tmp_M1); //K*phie' ǰ���Ѿ����phie'�ˣ�����ֱ����,ǰ��ע�Ᵽ��
	matrix_eye((float*)RELS_tmp_M2, relsIn->ML);  //eye(na+nb+1+nc)
	matrix_minus((float*)RELS_tmp_M2, (float*)RELS_tmp_M1, relsIn->ML, relsIn->ML, (float*)RELS_tmp_M3);   //(eye(na+nb+1+nc)-K*phie')
	matrix_multiply((float*)RELS_tmp_M3, (float*)relsIn->P, relsIn->ML, relsIn->ML, relsIn->ML, (float*)RELS_tmp_M4);  //(eye(na+nb+1+nc)-K*phie')*P
	matrix_multiply_k((float*)RELS_tmp_M4, 1.0/lambda, relsIn->ML, relsIn->ML, (float*)relsIn->P);  //P=(eye(nf+ng+2)-K*phie')*P/lambda;
	//xie=y(k)-phie'*thetae(:,k);
	matrix_multiply((float*)RELS_tmp_VT1, (float*)relsIn->thetae, 1, relsIn->ML, 1, (float*)RELS_tmp_UM1);  //phie'*thetae(:,k)  ǰ���Ѿ����phie'�ˣ�����ֱ����,ǰ��ע�Ᵽ��
	relsIn->xie = CDataYE[0] - RELS_tmp_UM1[0][0];  //xie=y(k)-phie'*thetae(:,k);

// 					%����������С���˷�
// 					phie=[ufk(d:d+nf);yfk(d:d+ng)];
// 					K=P*phie/(lambda+phie'*P*phie);
// 					thetae(:,k)=thetae_1+K*(y(k)-phie'*thetae_1);
// 					P=(eye(nf+ng+2)-K*phie')*P/lambda;
// 					
// 					xie=y(k)-phie'*thetae(:,k);%�������Ĺ���ֵ
// 					
// 					%��ȡ��ʶ����
// 					ae=[1 thetae(1:na,k)']; be=thetae(na+1:na+nb+1,k)'; ce=[1 thetae(na+nb+2:na+nb+1+nc,k)'];
// 					if abs(ce(2))>0.9
// 							ce(2)=sign(ce(2))*0.9;
// 					end
	
	return relsIn->xie;  //���ﷵ��ֵ�ǹ۲⵽�ı��ε�����
	
}

