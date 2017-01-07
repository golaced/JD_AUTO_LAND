#include "RELS.h"
#include "CFFRELS.h"
#include "PPCFFRELS.h"
#include "STC_PP.h"
#include "STC_PP_PID.h"

extern float CEPower ;
extern float CERoll ;
extern float CEPitch ;
extern float CEYaw  ;


//#define _IDENTIFICATION_PITCH_  //ȡ������ע������������������ڸ����ǿ��Ƶ�ϵͳ��ʶ���������ʱ�����
//#define _IDENTIFICATION_ROLL_  //ȡ������ע������������������ڹ�ת�ǿ��Ƶ�ϵͳ��ʶ���������ʱ�����
//#define _IDENTIFICATION_YAW_  //ȡ������ע�������������������ƫ���ǿ��Ƶ�ϵͳ��ʶ���������ʱ�����
#define _IDENTIFICATION_ALL_WITH_CONTROL_  //ȡ������ע�������������������3��ŷ���ǿ��Ƶ�ϵͳ��ʶ������зɿصķ������ƺ��ֶ����Ƶ�Ӱ�죬�������ʱ�����


//����洢�Ŀ��������ݵĳ��ȣ���ʷ�͵�ǰ����ǰ������Ϊ0�������ֵһ���ô���2����ΪĬ��U��һ���ӳ�
#define I_CONTROL_DATA_LENGTH 15  

//����������С���˹��Ƶ���ض���
#define  RELS_NA 1       //a�Ľ�����һ
#define  RELS_NB 1       //b�Ľ�����һ
#define  RELS_NC 0       //c�Ľ�����һ
#define  RELS_D 1       //d�ӳ�

//����������С���˹��Ƶ���ض���
#define  PPCFFRELS_NA 1            //a�Ľ�����һ
#define  PPCFFRELS_NB 1            //b�Ľ�����һ
#define  PPCFFRELS_D 1             //d�ӳ�
#define  PPCFFRELS_AM1 1           //Am�ĵ�1������
#define  PPCFFRELS_AM2 -1.3        //Am�ĵ�2������
#define  PPCFFRELS_AM3 0.48        //Am�ĵ�3������
#define  PPCFFRELS_LAMBDA 0.995    //��������



void ident(void);