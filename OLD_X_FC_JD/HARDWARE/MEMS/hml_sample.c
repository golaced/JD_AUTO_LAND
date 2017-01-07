
#include "../HARDWARE/define.h"
#include "../HARDWARE/MEMS/hml_sample.h"
#include "../HARDWARE/parameter.h"
#include "../HARDWARE/DRIVER/iic1.h"
#include "../HARDWARE/DRIVER/iic.h"
#include "../HARDWARE/MATH/my_math.h"
#include "../HARDWARE/MEMS/hml_cal.h"
ak8975_t ak8975 = { {0,0,0},{124,-449,369},{1,0.532,0.486},{0,0,0} };
ak8975_t ak8975_fc = { {0,0,0},{232,-221,-119},{1.17,1.339,1},{0,0,0} };

bool ANO_AK8975_Run(void)
{
	return IIC_Write_1Byte(AK8975_ADDRESS,AK8975_CNTL,0x01);	
}

xyz_f_t XYZ_STRUCT_COPY(float x,float y, float z)
{
	xyz_f_t m ;
	m.x = x;
	m.y = y;
	m.z = z;
	return m;
}
#define  IIR_ORDER     4      //ʹ��IIR�˲����Ľ���
static double b_IIR_hml[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //ϵ��b
static double a_IIR_hml[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//ϵ��a
static double InPut_IIR_hml[3][IIR_ORDER+1] = {0};
static double OutPut_IIR_hml[3][IIR_ORDER+1] = {0};
u8 ak8975_ok;
u8 hml_fix=1;
void ANO_AK8975_Read_Mag_Data(void)
{
	int16_t mag_temp[3];
//u8 ak8975_buffer[6]; //�������ݻ���
	
	I2C_FastMode = 0;

	HMC58X3_getRaw(&mag_temp[0], &mag_temp[1],&mag_temp[2]);

	ak8975_fc.Mag_Adc.x= IIR_I_Filter(mag_temp[0], InPut_IIR_hml[0], OutPut_IIR_hml[0], b_IIR_hml, IIR_ORDER+1, a_IIR_hml, IIR_ORDER+1);
  ak8975_fc.Mag_Adc.y= IIR_I_Filter(mag_temp[1], InPut_IIR_hml[1], OutPut_IIR_hml[1], b_IIR_hml, IIR_ORDER+1, a_IIR_hml, IIR_ORDER+1);
	#if IMU_HML_ADD_500
	if(mag_temp[2]<100)
	#endif	
	ak8975_fc.Mag_Adc.z= IIR_I_Filter(mag_temp[2], InPut_IIR_hml[2], OutPut_IIR_hml[2], b_IIR_hml, IIR_ORDER+1, a_IIR_hml, IIR_ORDER+1);	
	#if IMU_HML_ADD_500
	else
	ak8975_fc.Mag_Adc.z= IIR_I_Filter(mag_temp[2]-500, InPut_IIR_hml[2], OutPut_IIR_hml[2], b_IIR_hml, IIR_ORDER+1, a_IIR_hml, IIR_ORDER+1);
	#endif
	if(hml_fix){
	ak8975_fc.Mag_Val.x = (ak8975_fc.Mag_Adc.x - ak8975_fc.Mag_Offset.x)*ak8975_fc.Mag_Gain.x ;
	ak8975_fc.Mag_Val.y = (ak8975_fc.Mag_Adc.y - ak8975_fc.Mag_Offset.y)*ak8975_fc.Mag_Gain.y ;
	ak8975_fc.Mag_Val.z = (ak8975_fc.Mag_Adc.z - ak8975_fc.Mag_Offset.z)*ak8975_fc.Mag_Gain.z ;
	}
	else
	{
	ak8975_fc.Mag_Val.x = ak8975_fc.Mag_Adc.x;
	ak8975_fc.Mag_Val.y = ak8975_fc.Mag_Adc.y;
	ak8975_fc.Mag_Val.z = ak8975_fc.Mag_Adc.z;
	}	
	
	//�������е����	
	ANO_AK8975_CalOffset_Mag();

}

xyz_f_t ANO_AK8975_Get_Mag(void)
{
	return ak8975_fc.Mag_Val;
}

u8 Mag_CALIBRATED = 0,Mag_CALIBRATED_R=0;;
//�������е����
#define MAX_HML_CAL 500
void ANO_AK8975_CalOffset_Mag(void)
{ static u8 cal_init;
	static xyz_f_t	MagMAX = { -100 , -100 , -100 }, MagMIN = { 100 , 100 , 100 }, MagSum;
	static uint16_t cnt_m=0;
	static u8 hml_cal_temp=0;
  static u8 state_cal_hml;
  float t_h=Get_Cycle_T(GET_T_HML_CAL)+0.0001;	
	
	
	HML_SAMPLE(ak8975_fc.Mag_CALIBRATED,ak8975_fc.Mag_Adc.x,ak8975_fc.Mag_Adc.y,ak8975_fc.Mag_Adc.z,Pit_fc ,Rol_fc,mpu6050.Gyro_deg.x,mpu6050.Gyro_deg.y,mpu6050.Gyro_deg.z,t_h);
	if(ak8975_fc.Mag_CALIBRATED)
	{	
		
		if(!cal_init)
		{
		cal_init=1;
		MagMAX.x=MagMAX.y=MagMAX.z=-100;MagMIN.x=MagMIN.y=MagMIN.z=100;
		}
		
		#if USE_CYCLE_HML_CAL
		if(ABS(ak8975_fc.Mag_Adc.x)<MAX_HML_CAL&&ABS(ak8975_fc.Mag_Adc.y)<MAX_HML_CAL&&ABS(ak8975_fc.Mag_Adc.z)<MAX_HML_CAL)
		    HMC_CAL_HML();
		
		#else
		if(ABS(ak8975_fc.Mag_Adc.x)<MAX_HML_CAL&&ABS(ak8975_fc.Mag_Adc.y)<MAX_HML_CAL&&ABS(ak8975_fc.Mag_Adc.z)<MAX_HML_CAL)
		{
			
			
			
			MagMAX.x = MAX(ak8975_fc.Mag_Adc.x, MagMAX.x);
			MagMAX.y = MAX(ak8975_fc.Mag_Adc.y, MagMAX.y);
			MagMAX.z = MAX(ak8975_fc.Mag_Adc.z, MagMAX.z);
			
			MagMIN.x = MIN(ak8975_fc.Mag_Adc.x, MagMIN.x);
			MagMIN.y = MIN(ak8975_fc.Mag_Adc.y, MagMIN.y);
			MagMIN.z = MIN(ak8975_fc.Mag_Adc.z, MagMIN.z);		
			
			if(cnt_m ++>= CALIBRATING_MAG_CYCLES/(t_h))
			{
				
				cal_init=0;
				ak8975_fc.Mag_Offset.x = (int16_t)((MagMAX.x + MagMIN.x) * 0.5f);
				ak8975_fc.Mag_Offset.y = (int16_t)((MagMAX.y + MagMIN.y) * 0.5f);
				ak8975_fc.Mag_Offset.z = (int16_t)((MagMAX.z + MagMIN.z) * 0.5f);
	
				MagSum.x = MagMAX.x - MagMIN.x;
				MagSum.y = MagMAX.y - MagMIN.y;
				MagSum.z = MagMAX.z - MagMIN.z;
//-----------------CAL by matlab				
				#if IMU_HML_ADD_500
//				ak8975_fc.Mag_Offset.x=173;
//				ak8975_fc.Mag_Offset.y=-177;
//				ak8975_fc.Mag_Offset.z=-63;
//				MagSum.x=232;
//				MagSum.y=225;
//				MagSum.z=170;
				#endif
				float temp_max=MagSum.x ;
				if( MagSum.y>temp_max)
					temp_max=MagSum.y;
			  if( MagSum.z>temp_max)
					temp_max=MagSum.z;
				
				ak8975_fc.Mag_Gain.x =  temp_max/MagSum.x ;
				ak8975_fc.Mag_Gain.y =  temp_max/MagSum.y ;
				ak8975_fc.Mag_Gain.z =  temp_max/MagSum.z ;
				
		  	WRITE_PARM();//Param_SaveMagOffset(&ak8975_fc.Mag_Offset);
				cnt_m = 0;
				ak8975_fc.Mag_CALIBRATED = 0;
			}
		}
		#endif
	
		
	}
	else
	{

	}
}

void ANO_AK8975_Read(void)
{
		//��ȡ������
		ANO_AK8975_Read_Mag_Data();
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/


float HMC5883_lastx,HMC5883_lasty,HMC5883_lastz;
//�����Ʊ궨ֵ
int16_t  HMC5883_maxx=0,HMC5883_maxy=0,HMC5883_maxz=0,
		 HMC5883_minx=-0,HMC5883_miny=-0,HMC5883_minz=-0;
static int16_t offset_mx,offset_my,offset_mz;	
static float mx_scale=1.0,my_scale=1.0,mz_scale=1.0;			


int16_t  HMC5883_FIFO[3][11]; //�������˲�

void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z);

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   unsigned char HMC5883_IS_newdata(void)
*��������:	   ��ȡDRDY ���ţ��ж��Ƿ������һ��ת��
 Low for 250 ��sec when data is placed in the data output registers. 
���������  ��
���������  ������ת���������1  ������� 0
*******************************************************************************/
unsigned char HMC5883_IS_newdata(void)
{
 	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==Bit_SET){
	  return 1;
	 }
	 else return 0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void HMC58X3_FIFO_init(void)
*��������:	   ������ȡ100�����ݣ��Գ�ʼ��FIFO����
���������  ��
���������  ��
*******************************************************************************/
void HMC58X3_FIFO_init(void)
{
  int16_t temp[3];
  unsigned char i;
  for(i=0;i<50;i++){
  HMC58X3_getRaw(&temp[0],&temp[1],&temp[2]);
  Delay_us(200);  //��ʱ�ٶ�ȡ����

  }
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
*��������:	   ����һ�����ݵ�FIFO����
���������  �������������Ӧ��ADCֵ
���������  ��
*******************************************************************************/
void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
{
	unsigned char i ;
	int32_t sum=0;

	for(i=1;i<10;i++){
		HMC5883_FIFO[0][i-1]=HMC5883_FIFO[0][i];
		HMC5883_FIFO[1][i-1]=HMC5883_FIFO[1][i];
		HMC5883_FIFO[2][i-1]=HMC5883_FIFO[2][i];
	}

	HMC5883_FIFO[0][9]=x;
	HMC5883_FIFO[1][9]=y;
	HMC5883_FIFO[2][9]=z;

	sum=0;
	for(i=0;i<10;i++){	//ȡ�����ڵ�ֵ���������ȡƽ��
   		sum+=HMC5883_FIFO[0][i];
	}
	HMC5883_FIFO[0][10]=sum/10;	//��ƽ��ֵ����

	sum=0;
	for(i=0;i<10;i++){
   		sum+=HMC5883_FIFO[1][i];
	}
	HMC5883_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
   		sum+=HMC5883_FIFO[2][i];
	}
	HMC5883_FIFO[2][10]=sum/10;
} //HMC58X3_newValues

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void HMC58X3_writeReg(unsigned char reg, unsigned char val)
*��������:	   дHMC5883L�ļĴ���
���������    reg  �Ĵ�����ַ
			  val   Ҫд���ֵ	
���������  ��
*******************************************************************************/
void HMC58X3_writeReg(unsigned char reg, unsigned char val) {
  IICwriteByte(HMC58X3_ADDR,reg,val);
	//IIC_Write_1Byte(HMC58X3_ADDR,reg,val);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z)
*��������:	   дHMC5883L�ļĴ���
���������    reg  �Ĵ�����ַ
			  val   Ҫд���ֵ	
���������  ��
*******************************************************************************/
void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z) {
   unsigned char vbuff[6];
   vbuff[0]=vbuff[1]=vbuff[2]=vbuff[3]=vbuff[4]=vbuff[5]=0;
   IICreadBytes(HMC58X3_ADDR,HMC58X3_R_XM,6,vbuff);
	//IIC_Read_nByte(HMC58X3_ADDR,HMC58X3_R_XM,6,vbuff);
//	#if MPU_UPSET 
	   HMC58X3_newValues(((int16_t)vbuff[0] << 8) | vbuff[1],((int16_t)(vbuff[4] << 8 | vbuff[5])),-((int16_t)vbuff[2] << 8) | vbuff[3]);
//   HMC58X3_newValues(((int16_t)vbuff[0] << 8) | vbuff[1],-(((int16_t)vbuff[4] << 8) | vbuff[5]),-(((int16_t)vbuff[2] << 8) | vbuff[3]));
	//#endif
   *x = HMC5883_FIFO[0][10];
   *y = HMC5883_FIFO[1][10];
   *z = HMC5883_FIFO[2][10];
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_getValues(int16_t *x,int16_t *y,int16_t *z)
*��������:	   ��ȡ �����Ƶĵ�ǰADCֵ
���������    �������Ӧ�����ָ��	
���������  ��
*******************************************************************************/
void HMC58X3_getlastValues(int16_t *x,int16_t *y,int16_t *z) {
  *x = HMC5883_FIFO[0][10];
  *y = HMC5883_FIFO[1][10]; 
  *z = HMC5883_FIFO[2][10]; 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_mgetValues(float *arry)
*��������:	   ��ȡ У����� ������ADCֵ
���������    �������ָ��	
���������  ��
*******************************************************************************/
void HMC58X3_mgetValues(float *arry) {
  int16_t xr,yr,zr;
  HMC58X3_getRaw(&xr, &yr, &zr);

  arry[0]= HMC5883_lastx=((float)(xr - offset_mx)) * mx_scale;
  arry[1]= HMC5883_lasty=((float)(yr - offset_my)) * my_scale;
  arry[2]= HMC5883_lastz=((float)(zr - offset_mz)) * mz_scale;
	
	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_setGain(unsigned char gain)
*��������:	   ���� 5883L������
���������     Ŀ������ 0-7
���������  ��
*******************************************************************************/
void HMC58X3_setGain(unsigned char gain) { 
  // 0-7, 1 default
  if (gain > 7) return;
  HMC58X3_writeReg(HMC58X3_R_CONFB, gain << 5);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_setMode(unsigned char mode)
*��������:	   ���� 5883L�Ĺ���ģʽ
���������     ģʽ
���������  ��
*******************************************************************************/
void HMC58X3_setMode(unsigned char mode) {
  if (mode > 2) {
    return;
  }
  HMC58X3_writeReg(HMC58X3_R_MODE, mode);
  Delay_us(100);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_init(u8 setmode)
*��������:	   ���� 5883L�Ĺ���ģʽ
���������     ģʽ
���������  ��
*******************************************************************************/
void HMC58X3_init(u8 setmode) {

  if (setmode) {
    HMC58X3_setMode(0);
  }

  HMC58X3_writeReg(HMC58X3_R_CONFA, 0x70); // 8 samples averaged, 75Hz frequency, no artificial bias.
  HMC58X3_writeReg(HMC58X3_R_CONFB, 0x20);//0xA0
  HMC58X3_writeReg(HMC58X3_R_MODE, 0x00);

}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_setDOR(unsigned char DOR)
*��������:	   ���� 5883L�� �����������
���������     ����ֵ
0 -> 0.75Hz  |   1 -> 1.5Hz
2 -> 3Hz     |   3 -> 7.5Hz
4 -> 15Hz    |   5 -> 30Hz
6 -> 75Hz  
���������  ��
*******************************************************************************/
void HMC58X3_setDOR(unsigned char DOR) {
  if (DOR>6) return;
  HMC58X3_writeReg(HMC58X3_R_CONFA,DOR<<2);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_getID(char id[3])
*��������:	   ��ȡоƬ��ID
���������     	ID��ŵ�����
���������  ��
*******************************************************************************/
void HMC58X3_getID(char id[3]) 
{     u8 id_temp[3];
	
		//	IIC_Read_1Byte(HMC58X3_ADDR,HMC58X3_R_IDA,&id_temp[0]);
		//	IIC_Read_1Byte(HMC58X3_ADDR,HMC58X3_R_IDB,&id_temp[1]);
		//	IIC_Read_1Byte(HMC58X3_ADDR,HMC58X3_R_IDC,&id_temp[2]);
	  //  id[0]=id_temp[0];
		//	id[1]=id_temp[1];
		//	id[2]=id_temp[2];
      id[0]=I2C_ReadOneByte(HMC58X3_ADDR,HMC58X3_R_IDA);  
      id[1]=I2C_ReadOneByte(HMC58X3_ADDR,HMC58X3_R_IDB);
      id[2]=I2C_ReadOneByte(HMC58X3_ADDR,HMC58X3_R_IDC);
}   

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5883L_SetUp(void)
*��������:	   ��ʼ�� HMC5883L ʹ֮�������״̬
���������     	
���������  ��
*******************************************************************************/
void HMC5883L_SetUp(void)
{ char id[3];
	HMC58X3_getID(id);
  HMC58X3_init(0); // Don't set mode yet, we'll do that later on.
  HMC58X3_setMode(0);
  //HMC58X3_setDOR(HMC5883L_RATE_15);
	HMC58X3_setDOR(HMC5883L_RATE_75);
	HMC58X3_setGain(HMC5883L_GAIN_440);
  HMC58X3_FIFO_init();

}
//