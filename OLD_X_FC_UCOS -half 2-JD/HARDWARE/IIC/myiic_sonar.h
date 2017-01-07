#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//IIC ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2010/6/10 
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  


   	   		   
#define SDA_IN_Sonar()  {GPIOD->MODER&=~(3<<(2*2));GPIOD->MODER|=0<<2*2;}	//PB9����ģʽ
#define SDA_OUT_Sonar() {GPIOD->MODER&=~(3<<(2*2));GPIOD->MODER|=1<<2*2;} //PB9���ģʽ


//IO��������	 
#define IIC_SCL_Sonar    PCout(12) //SCL
#define IIC_SDA_Sonar    PDout(2) //SDA	 
#define READ_SDA_Sonar   PDin(2)  //����SDA 

//IIC���в�������
void IIC_Init_Sonar(void);                //��ʼ��IIC��IO��				 
void IIC_Start_Sonar(void);				//����IIC��ʼ�ź�
void IIC_Stop_Sonar(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte_Sonar(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte_Sonar(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack_Sonar(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack_Sonar(void);					//IIC����ACK�ź�
void IIC_NAck_Sonar(void);				//IIC������ACK�ź�

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
//Mini STM32������ʹ�õ���24c02�����Զ���EE_TYPEΪAT24C02
#define EE_TYPE AT24C02
					  
u8 AT24CXX_ReadOneByte(u16 ReadAddr);							//ָ����ַ��ȡһ���ֽ�

u8 KS103_ReadOneByte(u8 address, u8 reg);

void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite);		//ָ����ַд��һ���ֽ�

void KS103_WriteOneByte(u8 address,u8 reg,u8 command);

void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len);//ָ����ַ��ʼд��ָ�����ȵ�����
u32 AT24CXX_ReadLenByte(u16 ReadAddr,u8 Len);					//ָ����ַ��ʼ��ȡָ����������
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite);	//��ָ����ַ��ʼд��ָ�����ȵ�����
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead);   	//��ָ����ַ��ʼ����ָ�����ȵ�����


u8 AT24CXX_Check(void);  //�������


#endif
















