

#include "include.h"
#include "iic.h"
#include "hml_sample.h"
#include "parameter.h"
#include "my_math.h"
#include "iic_soft.h"
#include "filter.h"
#include "hml5833l.h"

ak8975_t ak8975 = { {0,0,0},{-1,-1,-1},{1,0.8538,0.9389},{0,0,0} };

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
void ANO_AK8975_Read_Mag_Data(void)
{
	int16_t mag_temp[3];
//u8 ak8975_buffer[6]; //�������ݻ���
	
	I2C_FastMode = 0;
	/*
	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HXL,&ak8975_buffer[0]); 
	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HXH,&ak8975_buffer[1]);
	mag_temp[1] = -((((int16_t)ak8975_buffer[1]) << 8) | ak8975_buffer[0]) ;  //������X��

	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HYL,&ak8975_buffer[2]);
	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HYH,&ak8975_buffer[3]);
	mag_temp[0] = -((((int16_t)ak8975_buffer[3]) << 8) | ak8975_buffer[2]) ;  //������Y��

	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HZL,&ak8975_buffer[4]);
	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HZH,&ak8975_buffer[5]);
	mag_temp[2] =  ((((int16_t)ak8975_buffer[5]) << 8) | ak8975_buffer[4]) ;  //������Z��	
	
	ak8975.Mag_Adc.x = mag_temp[0];
	ak8975.Mag_Adc.y = mag_temp[1];
	ak8975.Mag_Adc.z = mag_temp[2];
	*/
	 HMC58X3_getRaw(&mag_temp[0], &mag_temp[1],&mag_temp[2]);
	//ak8975.Mag_Adc.x = mag_temp[0];
	//ak8975.Mag_Adc.y = mag_temp[1];
	//ak8975.Mag_Adc.z = mag_temp[2];
	ak8975.Mag_Adc.x= IIR_I_Filter(mag_temp[0], InPut_IIR_hml[0], OutPut_IIR_hml[0], b_IIR_hml, IIR_ORDER+1, a_IIR_hml, IIR_ORDER+1);
  ak8975.Mag_Adc.y= IIR_I_Filter(mag_temp[1], InPut_IIR_hml[1], OutPut_IIR_hml[1], b_IIR_hml, IIR_ORDER+1, a_IIR_hml, IIR_ORDER+1);
	ak8975.Mag_Adc.z= IIR_I_Filter(mag_temp[2], InPut_IIR_hml[2], OutPut_IIR_hml[2], b_IIR_hml, IIR_ORDER+1, a_IIR_hml, IIR_ORDER+1);
	ak8975.Mag_Val.x = (ak8975.Mag_Adc.x - ak8975.Mag_Offset.x) ;
	ak8975.Mag_Val.y = (ak8975.Mag_Adc.y - ak8975.Mag_Offset.y)*ak8975.Mag_Gain.y ;
	ak8975.Mag_Val.z = (ak8975.Mag_Adc.z - ak8975.Mag_Offset.z)*ak8975.Mag_Gain.z ;
	//�������е����	
	ANO_AK8975_CalOffset_Mag();
	
	//AK8975��������
//	ANO_AK8975_Run();
}

xyz_f_t ANO_AK8975_Get_Mag(void)
{
	return ak8975.Mag_Val;
}





u8 Mag_CALIBRATED = 0,Mag_CALIBRATED_R=0;;
//�������е����

void ANO_AK8975_CalOffset_Mag(void)
{
	static xyz_f_t	MagMAX = { -100 , -100 , -100 }, MagMIN = { 100 , 100 , 100 }, MagSum;
	static uint16_t cnt_m=0;
	static u8 hml_cal_temp=0;
static u8 state_cal_hml;
	switch(state_cal_hml)
	{
		case 0:if(Mag_CALIBRATED_R!=hml_cal_temp)
		{				Mag_CALIBRATED=1;state_cal_hml=1;}break;
		case 1:if(Mag_CALIBRATED==0)
		{			hml_cal_temp=Mag_CALIBRATED_R;state_cal_hml=0;}break;
	}
	
	if(Mag_CALIBRATED)
	{	
		#if USE_CYCLE_HML_CAL
		if(ABS(ak8975.Mag_Adc.x)<500&&ABS(ak8975.Mag_Adc.y)<500&&ABS(ak8975.Mag_Adc.z)<500)
		    HMC_CAL_HML();
		
		#else
		if(ABS(ak8975.Mag_Adc.x)<500&&ABS(ak8975.Mag_Adc.y)<500&&ABS(ak8975.Mag_Adc.z)<500)
		{
			MagMAX.x = MAX(ak8975.Mag_Adc.x, MagMAX.x);
			MagMAX.y = MAX(ak8975.Mag_Adc.y, MagMAX.y);
			MagMAX.z = MAX(ak8975.Mag_Adc.z, MagMAX.z);
			
			MagMIN.x = MIN(ak8975.Mag_Adc.x, MagMIN.x);
			MagMIN.y = MIN(ak8975.Mag_Adc.y, MagMIN.y);
			MagMIN.z = MIN(ak8975.Mag_Adc.z, MagMIN.z);		
			
			if(cnt_m == CALIBRATING_MAG_CYCLES)
			{
				ak8975.Mag_Offset.x = (int16_t)((MagMAX.x + MagMIN.x) * 0.5f);
				ak8975.Mag_Offset.y = (int16_t)((MagMAX.y + MagMIN.y) * 0.5f);
				ak8975.Mag_Offset.z = (int16_t)((MagMAX.z + MagMIN.z) * 0.5f);
	
				MagSum.x = MagMAX.x - MagMIN.x;
				MagSum.y = MagMAX.y - MagMIN.y;
				MagSum.z = MagMAX.z - MagMIN.z;
				
				ak8975.Mag_Gain.y = MagSum.x / MagSum.y;
				ak8975.Mag_Gain.z = MagSum.x / MagSum.z;
				
			WRITE_PARM();//Param_SaveMagOffset(&ak8975.Mag_Offset);//param_Save();//��������
				cnt_m = 0;
				Mag_CALIBRATED = 0;
			}
		}
		#endif
		cnt_m++;
		
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
{
      id[0]=I2C_ReadOneByte(HMC58X3_ADDR,HMC58X3_R_IDA);  
      id[1]=I2C_ReadOneByte(HMC58X3_ADDR,HMC58X3_R_IDB);
      id[2]=I2C_ReadOneByte(HMC58X3_ADDR,HMC58X3_R_IDC);
}   // getID().

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
	HMC58X3_setGain(HMC5883L_GAIN_660);
  HMC58X3_FIFO_init();

}
//