#include "ident.h"
#include "../HARDWARE/define.h"
#include "../HARDWARE/FUSHION/imu1.h"
#include "../HARDWARE/CONTROL/att.h"
#include "../HARDWARE/MEMS/mpu6050.h"
//�����������������ݵĴ洢
float CDataYrPitch[I_CONTROL_DATA_LENGTH];
float CDataYrRoll[I_CONTROL_DATA_LENGTH];
float CDataYrYaw[I_CONTROL_DATA_LENGTH];

float CDataYrGPitch[I_CONTROL_DATA_LENGTH];
float CDataYrGRoll[I_CONTROL_DATA_LENGTH];
float CDataYrGYaw[I_CONTROL_DATA_LENGTH];

float CDataYEPitch[I_CONTROL_DATA_LENGTH];
float CDataYERoll[I_CONTROL_DATA_LENGTH];
float CDataYEYaw[I_CONTROL_DATA_LENGTH];

float CDataYGPitch[I_CONTROL_DATA_LENGTH];
float CDataYGRoll[I_CONTROL_DATA_LENGTH];
float CDataYGYaw[I_CONTROL_DATA_LENGTH];

float CDataUPitch[I_CONTROL_DATA_LENGTH];
float CDataURoll[I_CONTROL_DATA_LENGTH];
float CDataUYaw[I_CONTROL_DATA_LENGTH];

//-----------------------------------------------------------------
float RELS_Pitch_xie = 0;   //�����Ĺ���ֵ
float RELS_Pitch_xiek[I_CONTROL_DATA_LENGTH];   //�����Ĺ���ֵ�Ĵ洢
RELS_T rels_pitch;
//-----------------------------------------------------------------
//-----------------------------------------------------------------
//�����˲��洢����
float CDataUfkPitch[I_CONTROL_DATA_LENGTH];
float CDataYfkPitch[I_CONTROL_DATA_LENGTH];
PPCFFRELS_T cffrels_pitch;
float CDataUfkRoll[I_CONTROL_DATA_LENGTH];
float CDataYfkRoll[I_CONTROL_DATA_LENGTH];
PPCFFRELS_T cffrels_roll;

//����ʵ��������ÿ��ͨ���Ŀ�������
float CEPower = 0;
float CERoll = 0;
float CEPitch = 0;
float CEYaw = 0;

//PID�õĻ��ȽǶ�ת��
#define F_C2D (57.29578f)  //����ת�Ƕȵĳ���

void ident(void)
{
//����ѭ������״ֵ̬��ѭ������
int StateRenewI = 0;  
static u8 init;
	if(!init){
	init=1;
			//���洢�����ݸ���ֵ
	memset(CDataYrPitch, 0.0, sizeof(CDataYrPitch));
	memset(CDataYrRoll, 0.0, sizeof(CDataYrRoll));
	memset(CDataYrYaw, 0.0, sizeof(CDataYrYaw));
	
	memset(CDataYrGPitch, 0.0, sizeof(CDataYrGPitch));
	memset(CDataYrGRoll, 0.0, sizeof(CDataYrGRoll));
	memset(CDataYrGYaw, 0.0, sizeof(CDataYrGYaw));
	
	memset(CDataYEPitch, 0.0, sizeof(CDataYEPitch));
	memset(CDataYERoll, 0.0, sizeof(CDataYERoll));
	memset(CDataYEYaw, 0.0, sizeof(CDataYEYaw));
	
	memset(CDataYGPitch, 0.0, sizeof(CDataYGPitch));
	memset(CDataYGRoll, 0.0, sizeof(CDataYGRoll));
	memset(CDataYGYaw, 0.0, sizeof(CDataYGYaw));
	
	memset(CDataUPitch, 0.0, sizeof(CDataUPitch));
	memset(CDataURoll, 0.0, sizeof(CDataURoll));
	memset(CDataUYaw, 0.0, sizeof(CDataUYaw));
		//RELS��ʼ��
	RELS_init(&rels_pitch, RELS_NA, RELS_NB, RELS_NC, RELS_D);
	//��У��PID�Ϳ�������ʼ��������ʹ����̬������ʼ��
	//����Ĭ��Pitch��Roll��ģ����һ���ģ�������ʱ��������Ӧ����
	PPCFFRELS_initAsSteady(&cffrels_pitch, PPCFFRELS_NA, PPCFFRELS_NB, PPCFFRELS_D, PPCFFRELS_AM1, PPCFFRELS_AM2, PPCFFRELS_AM3);   //����Ժ�Ҫ�����ڿ�����������ʼ������
	PPCFFRELS_initAsSteady(&cffrels_roll, PPCFFRELS_NA, PPCFFRELS_NB, PPCFFRELS_D, PPCFFRELS_AM1, PPCFFRELS_AM2, PPCFFRELS_AM3);   //����Ժ�Ҫ�����ڿ�����������ʼ������
	}
	//����ʵ�ʵõ��Ŀ���������ʱ����
	


			//����״̬��ע����������������
		for(StateRenewI=I_CONTROL_DATA_LENGTH-1;StateRenewI>0;StateRenewI--) {  //ѭ�����ڶ�������һ��Ϊ���µı���
			CDataYrPitch[StateRenewI] = CDataYrPitch[StateRenewI-1];
			CDataYrRoll[StateRenewI] = CDataYrRoll[StateRenewI-1];
			CDataYrYaw[StateRenewI] = CDataYrYaw[StateRenewI-1];
			
			CDataYrGPitch[StateRenewI] = CDataYrGPitch[StateRenewI-1];
			CDataYrGRoll[StateRenewI] = CDataYrGRoll[StateRenewI-1];
			CDataYrGYaw[StateRenewI] = CDataYrGYaw[StateRenewI-1];
			
			CDataYEPitch[StateRenewI] = CDataYEPitch[StateRenewI-1];
			CDataYERoll[StateRenewI] = CDataYERoll[StateRenewI-1];
			CDataYEYaw[StateRenewI] = CDataYEYaw[StateRenewI-1];
			
			CDataYGPitch[StateRenewI] = CDataYGPitch[StateRenewI-1];
			CDataYGRoll[StateRenewI] = CDataYGRoll[StateRenewI-1];
			CDataYGYaw[StateRenewI] = CDataYGYaw[StateRenewI-1];
			
			CDataUPitch[StateRenewI] = CDataUPitch[StateRenewI-1];
			CDataURoll[StateRenewI] = CDataURoll[StateRenewI-1];
			CDataUYaw[StateRenewI] = CDataUYaw[StateRenewI-1];
			// ����xiek
			RELS_Pitch_xiek[StateRenewI] = RELS_Pitch_xiek[StateRenewI-1];
			
			//�����˲�����ֵ�����������������������ڵ����ĺ�����
			CDataUfkPitch[StateRenewI] = CDataUfkPitch[StateRenewI-1];
			CDataYfkPitch[StateRenewI] = CDataYfkPitch[StateRenewI-1];
			CDataUfkRoll[StateRenewI] = CDataUfkRoll[StateRenewI-1];
			CDataYfkRoll[StateRenewI] = CDataYfkRoll[StateRenewI-1];
		}
	
		float ControlPitch=except_A.y   ;
		float ControlRoll= except_A.x   ;
    float ControlYaw=	 except_A.z	  ;	
		
		//��ȡ��ε�����
		CDataYrPitch[0] = ControlPitch/F_C2D;//������
		CDataYrRoll[0] = ControlRoll/F_C2D;
		CDataYrYaw[0] = ControlYaw/F_C2D;
		//���ٶȿ���
		CDataYrGPitch[0] = ControlPitch/F_C2D;
		CDataYrGRoll[0] = ControlRoll/F_C2D;
		CDataYrGYaw[0] = ControlYaw/F_C2D;
		
		float numberVX = 0;  //��¼�����ǵ���ʵֵ
		float numberVY = 0;
		float numberVZ = 0;
		float OmegaBX = 0;
		float OmegaBY = 0;
		float OmegaBZ = 0;

		//����Ƕȿ����õ�ŷ���ǣ��������л�����Щ�ɻ󣬾����ó�static�ˣ��������ǲ��õ�
	  float Phi = Roll;//x
		float Theta = Pitch;//y
		float Psi = Yaw;//z

		CDataYEPitch[0] = Theta;
		CDataYERoll[0] = Phi;
		CDataYEYaw[0] = Psi;
		
		OmegaBX=-mpu6050.Gyro_deg.x;
		OmegaBY=mpu6050.Gyro_deg.y;
		OmegaBZ=mpu6050.Gyro_deg.z;
		CDataYGPitch[0] = OmegaBY;
		CDataYGRoll[0] = OmegaBX;
		CDataYGYaw[0] = OmegaBZ;
	
		CDataUPitch[1] = CEPitch;  //ע�������¼����ʵ����һ�ε�ֵ����Ϊ����Ĭ��ϵͳʼ�մ���һ���ӳ�
		CDataURoll[1] = CERoll;
		CDataUYaw[1] = CEYaw;
		
		//����xiek�����Ҳ���ŵ���������,ҲҪע��õ�����ʵ����һ�ε�xie
		RELS_Pitch_xiek[1] = RELS_Pitch_xie;
		
		//����Ĭ�ϻ���ʹ�����ʾ�һ�����ˣ���ʼ������С���˱�ʶ------------------------------------------------
		RELS_Pitch_xie = RELS_Update(&rels_pitch, CDataYEPitch, CDataUPitch, RELS_Pitch_xiek);
		
	  //             �ṹ��           buf����       buf����reg       ��̬��         ������
		updateSTCPP(&cffrels_pitch, CDataYfkPitch, CDataUfkPitch, CDataYEPitch, CDataUPitch);
		updateSTCPP(&cffrels_roll, CDataYfkRoll, CDataUfkRoll, CDataYERoll, CDataURoll);


		//updateSTCPPFilterOnly(&cffrels_pitch, CDataYfkPitch, CDataUfkPitch, CDataYEPitch, CDataUPitch);
		//updateSTCPPFilterOnly(&cffrels_roll, CDataYfkRoll, CDataUfkRoll, CDataYERoll, CDataURoll);
	//out-put
		CEPitch=ctrl_1.out.y;
		CERoll= ctrl_1.out.x;
		CEYaw=  ctrl_1.out.z;
		
#ifdef _IDENTIFICATION_PITCH_
				CERoll = 0;
				CEPitch = ((float)(rand()%200-100))*0.2f;  //������ΧΪ-20~20
				CEYaw = 0; 
	
#endif 
#ifdef _IDENTIFICATION_ROLL_
				CERoll = ((float)(rand()%200-100))*0.4f;  //������ΧΪ-40~40
				CEPitch = 0;
				CEYaw = 0; 

#endif 
#ifdef _IDENTIFICATION_YAW_
				//CERoll = 0;
				//CEPitch = 0;  ����ʹ����PID���Ƶķ�ʽ��������ƽ
				CEYaw = ((float)(rand()%200-100))*0.3f;  //������ΧΪ-30~30

#endif 
#ifdef _IDENTIFICATION_ALL_WITH_CONTROL_
				//CERoll += ((float)(rand()%200-100))*0.01f;  //ֱ�Ӹ������������ϡ�1�İ�����
				CEPitch += ((float)(rand()%200-100))*0.01f;  //ֱ�Ӹ������������ϡ�1�İ�����
				//CEYaw += ((float)(rand()%200-100))*0.01f;  //ֱ�Ӹ������������ϡ�1�İ�����
#endif 				

}