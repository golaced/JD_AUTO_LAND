
#include "include.h"
#include "flash.h"
#include "mpu6050.h"
#include "hml5833l.h"
#include "pwm_in.h"
u8 FLASH_Buffer[SIZE_FLASH_ROOM];
u8 FLASH_READ_BUF[SIZE];
										//否则,写操作的时候,可能会导致擦除整个扇区,从而引起部分程序丢失.引起死机.
 		/*
//FLASH 扇区的起始地址
#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//扇区0起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//扇区1起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//扇区2起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//扇区3起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//扇区4起始地址, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//扇区5起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//扇区6起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//扇区7起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	//扇区8起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	//扇区9起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 	//扇区10起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 	//扇区11起始地址,128 Kbytes  
STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)TEXT_Buffer,SIZE);
STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)datatemp,SIZE);		
		*/	
 
//读取指定地址的半字(16位数据) 
//faddr:读地址 
//返回值:对应数据.
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}  
//获取某个地址所在的flash扇区
//addr:flash地址
//返回值:0~11,即addr所在的扇区
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
//从指定地址开始写入指定长度的数据
//特别注意:因为STM32F4的扇区实在太大,没办法本地保存扇区数据,所以本函数
//         写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
//         写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
//         没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写. 
//该函数对OTP区域也有效!可以用来写OTP区!
//OTP区域地址范围:0X1FFF7800~0X1FFF7A0F
//WriteAddr:起始地址(此地址必须为4的倍数!!)
//pBuffer:数据指针
//NumToWrite:字(32位)数(就是要写入的32位数据的个数.) 
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
  FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
  if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//非法地址
	FLASH_Unlock();									//解锁 
  FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
 		
	addrx=WriteAddr;				//写入的起始地址
	endaddr=WriteAddr+NumToWrite*4;	//写入的结束地址
	if(addrx<0X1FFF0000)			//只有主存储区,才需要执行擦除操作!!
	{
		while(addrx<endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V之间!!
				if(status!=FLASH_COMPLETE)break;	//发生错误了
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//写数据
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//写入数据
			{ 
				break;	//写入异常
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
  FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
} 

//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToRead:字(4位)数
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=4;//偏移4个字节.	
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










