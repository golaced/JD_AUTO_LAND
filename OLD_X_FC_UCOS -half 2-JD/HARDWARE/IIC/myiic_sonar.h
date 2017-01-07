#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//IIC 驱动函数	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/6/10 
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  


   	   		   
#define SDA_IN_Sonar()  {GPIOD->MODER&=~(3<<(2*2));GPIOD->MODER|=0<<2*2;}	//PB9输入模式
#define SDA_OUT_Sonar() {GPIOD->MODER&=~(3<<(2*2));GPIOD->MODER|=1<<2*2;} //PB9输出模式


//IO操作函数	 
#define IIC_SCL_Sonar    PCout(12) //SCL
#define IIC_SDA_Sonar    PDout(2) //SDA	 
#define READ_SDA_Sonar   PDin(2)  //输入SDA 

//IIC所有操作函数
void IIC_Init_Sonar(void);                //初始化IIC的IO口				 
void IIC_Start_Sonar(void);				//发送IIC开始信号
void IIC_Stop_Sonar(void);	  			//发送IIC停止信号
void IIC_Send_Byte_Sonar(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte_Sonar(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack_Sonar(void); 				//IIC等待ACK信号
void IIC_Ack_Sonar(void);					//IIC发送ACK信号
void IIC_NAck_Sonar(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte_Sonar(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte_Sonar(u8 daddr,u8 addr);	  


#define AT24C01		127
#define AT24C02		255
#define AT24C04		511
#define AT24C08		1023
#define AT24C16		2047
#define AT24C32		4095
#define AT24C64	  8191
#define AT24C128	16383
#define AT24C256	32767  
//Mini STM32开发板使用的是24c02，所以定义EE_TYPE为AT24C02
#define EE_TYPE AT24C02
					  
u8 AT24CXX_ReadOneByte(u16 ReadAddr);							//指定地址读取一个字节

u8 KS103_ReadOneByte(u8 address, u8 reg);

void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite);		//指定地址写入一个字节

void KS103_WriteOneByte(u8 address,u8 reg,u8 command);

void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len);//指定地址开始写入指定长度的数据
u32 AT24CXX_ReadLenByte(u16 ReadAddr,u8 Len);					//指定地址开始读取指定长度数据
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite);	//从指定地址开始写入指定长度的数据
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead);   	//从指定地址开始读出指定长度的数据


u8 AT24CXX_Check(void);  //检查器件


#endif
















