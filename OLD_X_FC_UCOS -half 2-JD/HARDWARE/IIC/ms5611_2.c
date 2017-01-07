#include "include.h"
#include "ms5611_2.h"
#include <math.h>
#include "pwm_in.h"

#undef ALTI_SPEED

#define MS5611Press_OSR  MS561101BA_OSR_4096  //��ѹ��������
#define MS5611Temp_OSR   MS561101BA_OSR_4096  //�¶Ȳ�������

// ��ѹ��״̬��
#define SCTemperature  0x01	  //��ʼ �¶�ת��
#define CTemperatureing  0x02  //����ת���¶�
#define SCPressure  0x03	  //��ʼת�� ��ѹ
#define SCPressureing  0x04	  //����ת����ѹֵ

#define MOVAVG_SIZE  1//10	   //�������10������  5

static uint8_t  Now_doing = SCTemperature;	//��ǰת��״̬
static uint16_t PROM_C[MS561101BA_PROM_REG_COUNT]; //�궨ֵ���
static uint32_t Current_delay=0;	    //ת����ʱʱ�� us 
static uint32_t Start_Convert_Time; //����ת��ʱ�� ʱ�� us 
static int32_t  tempCache;

static float Alt_Offset_m = 0;

//
#define PA_OFFSET_INIT_NUM 50	

static float Alt_offset_Pa=0; //�����0��(���������ƽ��)ʱ ��Ӧ����ѹֵ  ���ֵ����ϵ�ʱ����ѹֵ 
double paOffsetNum = 0; 
uint16_t  paInitCnt=0;
uint8_t paOffsetInited=0;

//interface for outside 
uint8_t Baro_ALT_Updated = 0; //��ѹ�Ƹ߶ȸ�����ɱ�־��
//units (Celsius degrees*100, mbar*100  ).
//��λ [�¶� ��] [��ѹ ��]  [�߶� ��] 
volatile float MS5611_Temperature,MS5611_Pressure,MS5611_Altitude,MS5611_VerticalSpeed;

// ��ʱ��λ us 	  ��ͬ�Ĳ������ȶ�Ӧ��ͬ����ʱֵ
uint32_t MS5611_Delay_us[9] = {
	1500,//MS561101BA_OSR_256 0.9ms  0x00
	1500,//MS561101BA_OSR_256 0.9ms  
	2000,//MS561101BA_OSR_512 1.2ms  0x02
	2000,//MS561101BA_OSR_512 1.2ms
	3000,//MS561101BA_OSR_1024 2.3ms 0x04
	3000,//MS561101BA_OSR_1024 2.3ms
	5000,//MS561101BA_OSR_2048 4.6ms 0x06
	5000,//MS561101BA_OSR_2048 4.6ms
	11000,//MS561101BA_OSR_4096 9.1ms 0x08
};

// FIFO ����					
static float Temp_buffer[MOVAVG_SIZE],Press_buffer[MOVAVG_SIZE],Alt_buffer[MOVAVG_SIZE];
static uint8_t temp_index=0,press_index=0; //����ָ��

//���һ���µ�ֵ�� �¶ȶ��� �����˲�
void MS561101BA_NewTemp(float val) 
{
	Temp_buffer[temp_index] = val;
	temp_index = (temp_index + 1) % MOVAVG_SIZE;
}

//���һ���µ�ֵ�� ��ѹ���� �����˲�
void MS561101BA_NewPress(float val)
{
	Press_buffer[press_index] = val;
	press_index = (press_index + 1) % MOVAVG_SIZE;
}

//���һ���µ�ֵ�� �߶ȶ��� �����˲�
void MS561101BA_NewAlt(float val) 
{
	int16_t i;
	for(i=1;i<MOVAVG_SIZE;i++)
		Alt_buffer[i-1] = Alt_buffer[i];
	Alt_buffer[MOVAVG_SIZE-1] = val;
}

//��ȡ���е�ƽ��ֵ
float MS561101BA_getAvg(float * buff, int size) 
{
	float sum = 0.0;
	int i;
	for(i=0; i<size; i++) 
	{
		sum += buff[i];
	}
	return sum / size;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_readPROM(void)
*��������:	    ��ȡ MS561101B �Ĺ����궨ֵ
��ȡ ��ѹ�Ƶı궨ֵ  ���������¶Ⱥ���ѹ�Ķ���
*******************************************************************************/
void MS561101BA_readPROM(void) 
{
	u8  inth,intl;
	uint8_t i2cret[2];
	int i;
	for (i=0;i<MS561101BA_PROM_REG_COUNT;i++) 
	{
		#ifdef DEBUG_HW_I2C
			i2cRead(MS5611_ADDR, MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE),2, i2cret); 
			PROM_C[i]=i2cret[0]<<8 | i2cret[1];
		#else
			IIC_Start();
			IIC_Send_Byte(MS5611_ADDR);
			IIC_Wait_Ack();
			IIC_Send_Byte(MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));
			IIC_Wait_Ack();	
			IIC_Stop();
			delay_us(5);
			IIC_Start();
			IIC_Send_Byte(MS5611_ADDR+1);  //�������ģʽ	
			delay_us(1);
			IIC_Wait_Ack();
			inth = IIC_Read_Byte(1);  //��ACK�Ķ�����
			delay_us(1);
			intl = IIC_Read_Byte(0);	 //���һ���ֽ�NACK
			IIC_Stop();
			
			PROM_C[i] = (((uint16_t)inth << 8) | intl);
		#endif
	}
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_reset(void)
*��������:	    ���͸�λ��� MS561101B 
*******************************************************************************/
void MS561101BA_reset(void) 
{
	#ifdef DEBUG_HW_I2C
		i2cWrite(MS5611_ADDR, MS561101BA_RESET, 1); 
	#else
	IIC_Start();
    IIC_Send_Byte(MS5611_ADDR); //д��ַ
	IIC_Wait_Ack();
    IIC_Send_Byte(MS561101BA_RESET);//���͸�λ����
	IIC_Wait_Ack();	
    IIC_Stop();
	#endif
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_startConversion(uint8_t command)
*��������:	    ��������ת����� MS561101B
��ѡ�� ת������Ϊ MS561101BA_D1  ת����ѹ
				  MS561101BA_D2  ת���¶�	 
*******************************************************************************/
void MS561101BA_startConversion(uint8_t command) 
{
#ifdef DEBUG_HW_I2C
	i2cWrite(MS5611_ADDR, command, 1); 
#else
	// initialize pressure conversion
	IIC_Start();
	IIC_Send_Byte(MS5611_ADDR); //д��ַ
	IIC_Wait_Ack();
	IIC_Send_Byte(command); //дת������
	IIC_Wait_Ack();	
	IIC_Stop();
#endif
}
#define CMD_ADC_READ            0x00 // ADC read command
/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned long MS561101BA_getConversion(void)
*��������:	    ��ȡ MS561101B ��ת�����	 
*******************************************************************************/
uint32_t MS561101BA_getConversion(void) 
{
	uint32_t conversion = 0;
	u8 temp[3];
	#ifdef DEBUG_HW_I2C
		i2cRead(MS5611_ADDR,CMD_ADC_READ ,3, temp); 
		conversion=temp[0] << 16 | temp[0] <<8 | temp[2];
	#else
	IIC_Start();
	IIC_Send_Byte(MS5611_ADDR); //д��ַ
	IIC_Wait_Ack();
	IIC_Send_Byte(0);// start read sequence
	IIC_Wait_Ack();	
	IIC_Stop();
	
	IIC_Start();
	IIC_Send_Byte(MS5611_ADDR+1);  //�������ģʽ	
	IIC_Wait_Ack();
	temp[0] = IIC_Read_Byte(1);  //��ACK�Ķ�����  bit 23-16
	temp[1] = IIC_Read_Byte(1);  //��ACK�Ķ�����  bit 8-15
	temp[2] = IIC_Read_Byte(0);  //��NACK�Ķ����� bit 0-7
	IIC_Stop();
	conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
	#endif
	return conversion;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_init(void)
*��������:	    ��ʼ�� MS561101B 
*******************************************************************************/
void MS5611_Init(void) 
{  
	MS561101BA_reset(); // ��λ MS561101B 
	delay_ms(100); // ��ʱ 
	MS561101BA_readPROM(); // ��ȡEEPROM �еı궨ֵ ����	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_GetTemperature(void)
*��������:	    ��ȡ �¶�ת�����	 
*******************************************************************************/
void MS561101BA_GetTemperature(void)
{	
	tempCache = MS561101BA_getConversion();	
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		float MS561101BA_get_altitude(void)
*��������:	    ����ǰ����ѹֵת�� �߶ȡ�	 
*******************************************************************************/
float MS561101BA_get_altitude(void)
{
	static float Altitude,AltPre;
	float dz,dt;
	uint32_t current=0;
	static uint32_t tp=0;

	// �Ƿ��ʼ����0����ѹֵ��
	if(Alt_offset_Pa == 0)
	{ 
		if(paInitCnt > PA_OFFSET_INIT_NUM)
		{
			Alt_offset_Pa = paOffsetNum / paInitCnt;
			paOffsetInited=1;
		}
		else
			paOffsetNum += MS5611_Pressure;
		
		paInitCnt++;
		
		Altitude = 0; //�߶� Ϊ 0
		
		return Altitude;
	}
	//����������ϵ�ʱ��λ�õĸ߶�ֵ ����λΪm
	Altitude = 4433000.0 * (1 - pow((MS5611_Pressure / Alt_offset_Pa), 0.1903))*0.01f;
	Altitude = Altitude + Alt_Offset_m ;  //��ƫ��

	#ifdef ALTI_SPEED
	current=micros();
	dt=(tp>0)?((current - tp)/1000000.0f):0;
	tp=current;
	dz=(Altitude-AltPre);
	AltPre=Altitude;	//m
	if(dt>0)
		MS5611_VerticalSpeed =  dz / dt;
#endif
	
	return Altitude; 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_getPressure(void)
*��������:	    ��ȡ ��ѹת����� ������������	 
*******************************************************************************/
//static float lastPress=0,newPress=0;
//static float press_limit_coe = 1;
int baroAlt,baro_off;
u8 baro_set;
void MS561101BA_getPressure(void) 
{
	int64_t off,sens;
	int64_t TEMP,T2,Aux_64,OFF2,SENS2;  // 64 bits
	int32_t rawPress = MS561101BA_getConversion();
	int64_t dT  = tempCache - (((int32_t)PROM_C[4]) << 8);
	
	TEMP = 2000 + (dT * (int64_t)PROM_C[5])/8388608;
	off  = (((int64_t)PROM_C[1]) << 16) + ((((int64_t)PROM_C[3]) * dT) >> 7);
	sens = (((int64_t)PROM_C[0]) << 15) + (((int64_t)(PROM_C[2]) * dT) >> 8);
	
	if (TEMP < 2000)
	{   // second order temperature compensation
		T2 = (((int64_t)dT)*dT) >> 31;
		Aux_64 = (TEMP-2000)*(TEMP-2000);
		OFF2 = (5*Aux_64)>>1;
		SENS2 = (5*Aux_64)>>2;
		TEMP = TEMP - T2;
		off = off - OFF2;
		sens = sens - SENS2;
	}

	//------------- ��ѹ���� ----------------
//	newPress=(((((int64_t)rawPress) * sens) >> 21) - off) / 32768;
//	
//	press_limit_coe = 1.0f; 
//	
//	//����ʱ������ѹֵ���ͣ���ѹ�߶����ߣ�
//	if(ALT_LOCK_FLAG == 0xff && (Math_abs(IMU_Pitch)>15 || Math_abs(IMU_Roll)>15))
//	{		
//		press_limit_coe = 0.01f;   //0.005
//		if(newPress<lastPress)
//			newPress = (1 - press_limit_coe) * lastPress + press_limit_coe * newPress; 
//	}

//	lastPress = newPress;
//	
//	MS5611_Pressure = newPress;
	
	//ԭʼ�ķ���
	MS5611_Pressure = (((((int64_t)rawPress) * sens) >> 21) - off) / 32768;

	//�¶ȶ��д���
	MS561101BA_NewTemp(TEMP*0.01f);
	
	MS5611_Temperature = MS561101BA_getAvg(Temp_buffer,MOVAVG_SIZE); //0.01c
	
	MS5611_Altitude = MS561101BA_get_altitude(); // ��λ��m 

	if(Rc_Pwm_Inr_mine[RC_THR]<200+1000)
	{baro_off=MS5611_Altitude*1000;}
	baroAlt=MS5611_Altitude*1000-baro_off;
	static u8 cnt;
	if(cnt++>100){cnt=100;
		baro_set=1;}
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS5611BA_Routing(void)
*��������:	    MS5611BA �����г��� ����Ҫ���ڵ��� �Ը�����ѹֵ���¶�ֵ�� 
*******************************************************************************/
void MS5611_Thread(void) 
{
	switch(Now_doing)
	{ //��ѯ״̬ ������������ ����Щʲô��
		case SCTemperature:  //�����¶�ת��
			MS561101BA_startConversion(MS561101BA_D2 + MS5611Temp_OSR);
			Current_delay = MS5611_Delay_us[MS5611Temp_OSR] ;//ת��ʱ��
			Start_Convert_Time = micros(); //��ʱ��ʼ
			Now_doing = CTemperatureing;//��һ��״̬
			break;
		case CTemperatureing:  //����ת���� 
			if((micros()-Start_Convert_Time) > Current_delay)
			{ //��ʱʱ�䵽����
				MS561101BA_GetTemperature(); //ȡ�¶�	
				Now_doing = SCPressure;	
			}
			break;
		case SCPressure: //������ѹת��
			MS561101BA_startConversion(MS561101BA_D1 + MS5611Press_OSR);
			Current_delay = MS5611_Delay_us[MS5611Press_OSR];//ת��ʱ��
			Start_Convert_Time = micros();//��ʱ��ʼ
			Now_doing = SCPressureing;//��һ��״̬
			break;
		case SCPressureing:	 //����ת����ѹֵ
			if((micros()-Start_Convert_Time) > Current_delay)
			{ //��ʱʱ�䵽����
				MS561101BA_getPressure();   //���� ����	
				Baro_ALT_Updated = 0xff; 	//�߶ȸ��� ��ɡ�
				Now_doing = SCTemperature;  //��ͷ����
			}
			break;
		default: 
			Now_doing = SCTemperature;
			break;
	}
}


void MS5611_ThreadNew(void) 
{
	switch(Now_doing)
	{ //��ѯ״̬ ������������ ����Щʲô��
 		case SCTemperature:  //�����¶�ת��
			//�����¶�ת��
				MS561101BA_startConversion(MS561101BA_D2 + MS5611Temp_OSR);
				Current_delay = MS5611_Delay_us[MS5611Temp_OSR] ;//ת��ʱ��
				Start_Convert_Time = micros(); //��ʱ��ʼ
				Now_doing = CTemperatureing;//��һ��״̬
 		break;
		
		case CTemperatureing:  //����ת���� 
			if((micros()-Start_Convert_Time) > Current_delay)
			{ //��ʱʱ�䵽����
				MS561101BA_GetTemperature(); //ȡ�¶�	
				//������ѹת��
				MS561101BA_startConversion(MS561101BA_D1 + MS5611Press_OSR);
				Current_delay = MS5611_Delay_us[MS5611Press_OSR];//ת��ʱ��
				Start_Convert_Time = micros();//��ʱ��ʼ
				Now_doing = SCPressureing;//��һ��״̬
			}
			break;
 
		case SCPressureing:	 //����ת����ѹֵ
			if((micros()-Start_Convert_Time) > Current_delay)
			{ //��ʱʱ�䵽����
				MS561101BA_getPressure();   //���� ����	
				Baro_ALT_Updated = 0xff; 	//�߶ȸ��� ��ɡ�
			//	Now_doing = SCTemperature;  //��ͷ����
				//�����¶�ת��
				MS561101BA_startConversion(MS561101BA_D2 + MS5611Temp_OSR);
				Current_delay = MS5611_Delay_us[MS5611Temp_OSR] ;//ת��ʱ��
				Start_Convert_Time = micros(); //��ʱ��ʼ
				Now_doing = CTemperatureing;//��һ��״̬
			}
			break;
		default: 
			Now_doing = CTemperatureing;
			break;
	}
}
//ע�⣬ʹ��ǰȷ��
uint8_t  WaitBaroInitOffset(void)
{
	uint32_t startTime=0;
	uint32_t now=0;
	
	startTime=micros();	//us
  while(!paOffsetInited)
	{
			MS5611_ThreadNew();
			now=micros();
			if((now-startTime)/1000 >= PA_OFFSET_INIT_NUM * 50)	//��ʱ
			{
				return 0;
			}
	}
	
	return 1;
}

//------------------End of File----------------------------
