
#include "include.h"
#include "flash.h"
#include "mpu6050.h"
#include "hml5833l.h"
#include "pwm_in.h"
u8 FLASH_Buffer[SIZE_FLASH_ROOM];
u8 FLASH_READ_BUF[SIZE];
										//����,д������ʱ��,���ܻᵼ�²�����������,�Ӷ����𲿷ֳ���ʧ.��������.
 		/*
//FLASH ��������ʼ��ַ
#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//����0��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//����1��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//����2��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//����3��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//����4��ʼ��ַ, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//����5��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//����6��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//����7��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	//����8��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	//����9��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 	//����10��ʼ��ַ,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 	//����11��ʼ��ַ,128 Kbytes  
STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)TEXT_Buffer,SIZE);
STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)datatemp,SIZE);		
		*/	
 
//��ȡָ����ַ�İ���(16λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}  
//��ȡĳ����ַ���ڵ�flash����
//addr:flash��ַ
//����ֵ:0~11,��addr���ڵ�����
uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10; 
	return FLASH_Sector_11;	
}
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ر�ע��:��ΪSTM32F4������ʵ��̫��,û�취���ر�����������,���Ա�����
//         д��ַ�����0XFF,��ô���Ȳ������������Ҳ�������������.����
//         д��0XFF�ĵ�ַ,�����������������ݶ�ʧ.����д֮ǰȷ��������
//         û����Ҫ����,��������������Ȳ�����,Ȼ����������д. 
//�ú�����OTP����Ҳ��Ч!��������дOTP��!
//OTP�����ַ��Χ:0X1FFF7800~0X1FFF7A0F
//WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ4�ı���!!)
//pBuffer:����ָ��
//NumToWrite:��(32λ)��(����Ҫд���32λ���ݵĸ���.) 
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
  FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
  if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//�Ƿ���ַ
	FLASH_Unlock();									//���� 
  FLASH_DataCacheCmd(DISABLE);//FLASH�����ڼ�,�����ֹ���ݻ���
 		
	addrx=WriteAddr;				//д�����ʼ��ַ
	endaddr=WriteAddr+NumToWrite*4;	//д��Ľ�����ַ
	if(addrx<0X1FFF0000)			//ֻ�����洢��,����Ҫִ�в�������!!
	{
		while(addrx<endaddr)		//ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//�з�0XFFFFFFFF�ĵط�,Ҫ�����������
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V֮��!!
				if(status!=FLASH_COMPLETE)break;	//����������
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//д����
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//д������
			{ 
				break;	//д���쳣
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
  FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
	FLASH_Lock();//����
} 

//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToRead:��(4λ)��
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//��ȡ4���ֽ�.
		ReadAddr+=4;//ƫ��4���ֽ�.	
	}
}

void READ_PARM(void)
{
STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)FLASH_READ_BUF,SIZE);	
mpu6050.Gyro_Offset.x=(vs16)(FLASH_READ_BUF[1]<<8|FLASH_READ_BUF[0]);
mpu6050.Gyro_Offset.y=(vs16)(FLASH_READ_BUF[3]<<8|FLASH_READ_BUF[2]);
mpu6050.Gyro_Offset.z=(vs16)(FLASH_READ_BUF[5]<<8|FLASH_READ_BUF[4]);
	
mpu6050.Acc_Offset.x=(vs16)(FLASH_READ_BUF[7]<<8|FLASH_READ_BUF[6]);
mpu6050.Acc_Offset.y=(vs16)(FLASH_READ_BUF[9]<<8|FLASH_READ_BUF[8]);
mpu6050.Acc_Offset.z=(vs16)(FLASH_READ_BUF[11]<<8|FLASH_READ_BUF[10]);
	
ak8975.Mag_Offset.x=(vs16)(FLASH_READ_BUF[13]<<8|FLASH_READ_BUF[12]);
ak8975.Mag_Offset.y=(vs16)(FLASH_READ_BUF[15]<<8|FLASH_READ_BUF[14]);
ak8975.Mag_Offset.z=(vs16)(FLASH_READ_BUF[17]<<8|FLASH_READ_BUF[16]);
	
ak8975.Mag_Gain.x =(float)((vs16)((FLASH_READ_BUF[19]<<8|FLASH_READ_BUF[18])))/100.;
ak8975.Mag_Gain.y=(float)((vs16)((FLASH_READ_BUF[21]<<8|FLASH_READ_BUF[20])))/100.;
ak8975.Mag_Gain.z =(float)((vs16)((FLASH_READ_BUF[23]<<8|FLASH_READ_BUF[22])))/100.;
	
//dj_angle_offset[0] =(float)((vs16)((FLASH_READ_BUF[25]<<8|FLASH_READ_BUF[24])))/100.;
//dj_angle_offset[1] =(float)((vs16)((FLASH_READ_BUF[27]<<8|FLASH_READ_BUF[26])))/100.;
//dj_angle_offset[2] =(float)((vs16)((FLASH_READ_BUF[29]<<8|FLASH_READ_BUF[28])))/100.;
pwmin.T =	 ((vs16)((FLASH_READ_BUF[25]<<8|FLASH_READ_BUF[24])));
pwmin.min =((vs16)((FLASH_READ_BUF[27]<<8|FLASH_READ_BUF[26])));
pwmin.max =((vs16)((FLASH_READ_BUF[29]<<8|FLASH_READ_BUF[28])));

home_point[0]=(double)((vs32)((FLASH_READ_BUF[30]<<24|FLASH_READ_BUF[31]<<16)|(FLASH_READ_BUF[32]<<8|FLASH_READ_BUF[33])))/10000000.;
home_point[1]=(double)((vs32)((FLASH_READ_BUF[34]<<24|FLASH_READ_BUF[35]<<16)|(FLASH_READ_BUF[36]<<8|FLASH_READ_BUF[37])))/10000000.;

check_way_point[0]=(double)((vs32)((FLASH_READ_BUF[38]<<24|FLASH_READ_BUF[39]<<16)|(FLASH_READ_BUF[40]<<8|FLASH_READ_BUF[41])))/10000000.;
check_way_point[1]=(double)((vs32)((FLASH_READ_BUF[42]<<24|FLASH_READ_BUF[43]<<16)|(FLASH_READ_BUF[44]<<8|FLASH_READ_BUF[45])))/10000000.;

way_point[0][0]=(double)((vs32)((FLASH_READ_BUF[46]<<24|FLASH_READ_BUF[47]<<16)|(FLASH_READ_BUF[48]<<8|FLASH_READ_BUF[49])))/10000000.;
way_point[0][1]=(double)((vs32)((FLASH_READ_BUF[50]<<24|FLASH_READ_BUF[51]<<16)|(FLASH_READ_BUF[52]<<8|FLASH_READ_BUF[53])))/10000000.;

way_point[1][0]=(double)((vs32)((FLASH_READ_BUF[54]<<24|FLASH_READ_BUF[55]<<16)|(FLASH_READ_BUF[56]<<8|FLASH_READ_BUF[57])))/10000000.;
way_point[1][1]=(double)((vs32)((FLASH_READ_BUF[58]<<24|FLASH_READ_BUF[59]<<16)|(FLASH_READ_BUF[60]<<8|FLASH_READ_BUF[61])))/10000000.;

Yaw_set_dji=(float)((vs16)((FLASH_READ_BUF[62]<<8|FLASH_READ_BUF[63])))/10.;
}

void WRITE_PARM(void)
{ 
vs32 _temp32;
int16_t _temp;
u8 cnt=0;

_temp=(int16_t)mpu6050.Gyro_Offset.x;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)mpu6050.Gyro_Offset.y;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)mpu6050.Gyro_Offset.z;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);

_temp=(int16_t)mpu6050.Acc_Offset.x;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)mpu6050.Acc_Offset.y;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)mpu6050.Acc_Offset.z;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)ak8975.Mag_Offset.x;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)ak8975.Mag_Offset.y;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)ak8975.Mag_Offset.z;
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);


_temp=(int16_t)(ak8975.Mag_Gain.x*100);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)(ak8975.Mag_Gain.y*100);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)(ak8975.Mag_Gain.z*100);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);


//_temp=(int16_t)(dj_angle_offset[0]*100);
//FLASH_Buffer[cnt++]=BYTE0(_temp);
//FLASH_Buffer[cnt++]=BYTE1(_temp);
//_temp=(int16_t)(dj_angle_offset[1]*100);
//FLASH_Buffer[cnt++]=BYTE0(_temp);
//FLASH_Buffer[cnt++]=BYTE1(_temp);
//_temp=(int16_t)(dj_angle_offset[2]*100);
//FLASH_Buffer[cnt++]=BYTE0(_temp);
//FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)(pwmin.T);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)(pwmin.min);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
_temp=(int16_t)(pwmin.max);
FLASH_Buffer[cnt++]=BYTE0(_temp);
FLASH_Buffer[cnt++]=BYTE1(_temp);
//home
_temp32=(home_point[0]*10000000);
FLASH_Buffer[cnt++]=BYTE3(_temp32);
FLASH_Buffer[cnt++]=BYTE2(_temp32);
FLASH_Buffer[cnt++]=BYTE1(_temp32);
FLASH_Buffer[cnt++]=BYTE0(_temp32);

_temp32=(home_point[1]*10000000);
FLASH_Buffer[cnt++]=BYTE3(_temp32);
FLASH_Buffer[cnt++]=BYTE2(_temp32);
FLASH_Buffer[cnt++]=BYTE1(_temp32);
FLASH_Buffer[cnt++]=BYTE0(_temp32);
//check
_temp32=(check_way_point[0]*10000000);
FLASH_Buffer[cnt++]=BYTE3(_temp32);
FLASH_Buffer[cnt++]=BYTE2(_temp32);
FLASH_Buffer[cnt++]=BYTE1(_temp32);
FLASH_Buffer[cnt++]=BYTE0(_temp32);

_temp32=(check_way_point[1]*10000000);
FLASH_Buffer[cnt++]=BYTE3(_temp32);
FLASH_Buffer[cnt++]=BYTE2(_temp32);
FLASH_Buffer[cnt++]=BYTE1(_temp32);
FLASH_Buffer[cnt++]=BYTE0(_temp32);

//0
_temp32=(way_point[0][0]*10000000);
FLASH_Buffer[cnt++]=BYTE3(_temp32);
FLASH_Buffer[cnt++]=BYTE2(_temp32);
FLASH_Buffer[cnt++]=BYTE1(_temp32);
FLASH_Buffer[cnt++]=BYTE0(_temp32);

_temp32=(way_point[0][1]*10000000);
FLASH_Buffer[cnt++]=BYTE3(_temp32);
FLASH_Buffer[cnt++]=BYTE2(_temp32);
FLASH_Buffer[cnt++]=BYTE1(_temp32);
FLASH_Buffer[cnt++]=BYTE0(_temp32);

//1
_temp32=(way_point[1][0]*10000000);
FLASH_Buffer[cnt++]=BYTE3(_temp32);
FLASH_Buffer[cnt++]=BYTE2(_temp32);
FLASH_Buffer[cnt++]=BYTE1(_temp32);
FLASH_Buffer[cnt++]=BYTE0(_temp32);

_temp32=(way_point[1][1]*10000000);
FLASH_Buffer[cnt++]=BYTE3(_temp32);
FLASH_Buffer[cnt++]=BYTE2(_temp32);
FLASH_Buffer[cnt++]=BYTE1(_temp32);
FLASH_Buffer[cnt++]=BYTE0(_temp32);

_temp=(int16_t)(Yaw_set_dji*10);
FLASH_Buffer[cnt++]=BYTE1(_temp);
FLASH_Buffer[cnt++]=BYTE0(_temp);
STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)FLASH_Buffer,SIZE);

}










