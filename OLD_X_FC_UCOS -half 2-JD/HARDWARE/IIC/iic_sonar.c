#include "myiic_sonar.h"
#include "delay.h"
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

//初始化IIC
void IIC_Init_Sonar(void)
{					     
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;
	/*开启GPIOB的外设时钟*/
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOF时钟
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOF时钟
 /*选择要控制的GPIOC引脚*/															   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12	;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//上拉

	/*调用库函数，初始化GPIOB*/
  	GPIO_Init(GPIOC, &GPIO_InitStructure);		 
		
	/*选择要控制的GPIOC引脚*/															   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//上拉
	/*调用库函数，初始化GPIOB*/
  GPIO_Init(GPIOD, &GPIO_InitStructure);		 

GPIO_SetBits(GPIOC,GPIO_Pin_12);

GPIO_SetBits(GPIOD,GPIO_Pin_2);
}
//产生IIC起始信号
void IIC_Start_Sonar(void)
{
	SDA_OUT_Sonar();     //sda线输出
	IIC_SDA_Sonar=1;	  	  
	IIC_SCL_Sonar=1;
	delay_us(10);
 	IIC_SDA_Sonar=0;//START:when CLK is high,DATA change form high to low 
	delay_us(10);
	IIC_SCL_Sonar=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop_Sonar(void)
{
	SDA_OUT_Sonar();//sda线输出
	IIC_SCL_Sonar=0;
	IIC_SDA_Sonar=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(10);
	IIC_SCL_Sonar=1; 
	IIC_SDA_Sonar=1;//发送I2C总线结束信号
	delay_us(10);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack_Sonar(void)
{
	u8 ucErrTime=0;
	SDA_IN_Sonar();      //SDA设置为输入  
	IIC_SDA_Sonar=1;delay_us(6);	   
	IIC_SCL_Sonar=1;delay_us(6);	 
	while(READ_SDA_Sonar)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop_Sonar();
			return 1;
		}
	}
	IIC_SCL_Sonar=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack_Sonar(void)
{
	IIC_SCL_Sonar=0;
	SDA_OUT_Sonar();
	IIC_SDA_Sonar=0;
	delay_us(10);
	IIC_SCL_Sonar=1;
	delay_us(10);
	IIC_SCL_Sonar=0;
}
//不产生ACK应答		    
void IIC_NAck_Sonar(void)
{
	IIC_SCL_Sonar=0;
	SDA_OUT_Sonar();
	IIC_SDA_Sonar=1;
	delay_us(10);
	IIC_SCL_Sonar=1;
	delay_us(10);
	IIC_SCL_Sonar=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte_Sonar(u8 txd)
{                        
    u8 t;   
	SDA_OUT_Sonar(); 	    
    IIC_SCL_Sonar=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA_Sonar=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(10);   //对TEA5767这三个延时都是必须的
		IIC_SCL_Sonar=1;
		delay_us(10); 
		IIC_SCL_Sonar=0;	
		delay_us(10);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte_Sonar(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN_Sonar();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL_Sonar=0; 
        delay_us(10);
		IIC_SCL_Sonar=1;
        receive<<=1;
        if(READ_SDA_Sonar)receive++;   
		delay_us(5); 
    }					 
    if (!ack)
        IIC_NAck_Sonar();//发送nACK
    else
        IIC_Ack_Sonar(); //发送ACK   
    return receive;
}


//在AT24CXX指定地址读出一个数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
u8 KS103_ReadOneByte(u8 address, u8 reg)
{				  
	u8 temp=0;		  	    																 
    IIC_Start_Sonar();  
    IIC_Send_Byte_Sonar(address);   //发送低地址
	IIC_Wait_Ack_Sonar();	 
	IIC_Send_Byte_Sonar(reg);   //发送低地址
	IIC_Wait_Ack_Sonar();	   
	IIC_Start_Sonar();  	 	   
	IIC_Send_Byte_Sonar(address + 1);           //进入接收模式			   
	IIC_Wait_Ack_Sonar();	 

	delay_us(50);	   //增加此代码通信成功！！！
    temp=IIC_Read_Byte_Sonar(0);	  //读寄存器3	   
    IIC_Stop_Sonar();//产生一个停止条件	    
	return temp;
}
//在AT24CXX指定地址读出一个数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{				  
	u8 temp=0;		  	    																 
    IIC_Start_Sonar();  
	if(EE_TYPE>AT24C16)
	{
		IIC_Send_Byte_Sonar(0XA0);	   //发送写命令
		IIC_Wait_Ack_Sonar();
		IIC_Send_Byte_Sonar(ReadAddr>>8);//发送高地址	    
	}else IIC_Send_Byte_Sonar(0XA0+((ReadAddr/256)<<1));   //发送器件地址0XA0,写数据 	   
	IIC_Wait_Ack_Sonar(); 
    IIC_Send_Byte_Sonar(ReadAddr%256);   //发送低地址
	IIC_Wait_Ack_Sonar();	    
	IIC_Start_Sonar();  	 	   
	IIC_Send_Byte_Sonar(0XA1);           //进入接收模式			   
	IIC_Wait_Ack_Sonar();	 
    temp=IIC_Read_Byte_Sonar(0);		   
    IIC_Stop_Sonar();//产生一个停止条件	    
	return temp;
}
//在AT24CXX指定地址写入一个数据
//WriteAddr  :写入数据的目的地址    
//DataToWrite:要写入的数据
void KS103_WriteOneByte(u8 address,u8 reg,u8 command)
{				   	  	    																 
    IIC_Start_Sonar();   
	IIC_Send_Byte_Sonar(address);	    //发送写命令
	IIC_Wait_Ack_Sonar();
	IIC_Send_Byte_Sonar(reg);//发送高地址	  
	IIC_Wait_Ack_Sonar();	   
    IIC_Send_Byte_Sonar(command);   //发送低地址
	IIC_Wait_Ack_Sonar(); 	 										  		   
    IIC_Stop_Sonar();//产生一个停止条件 
//	delay_ms(80);	 
}

//在AT24CXX指定地址写入一个数据
//WriteAddr  :写入数据的目的地址    
//DataToWrite:要写入的数据
void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite)
{				   	  	    																 
    IIC_Start_Sonar();  
	if(EE_TYPE>AT24C16)
	{
		IIC_Send_Byte_Sonar(0XA0);	    //发送写命令
		IIC_Wait_Ack_Sonar();
		IIC_Send_Byte_Sonar(WriteAddr>>8);//发送高地址	  
	}else IIC_Send_Byte_Sonar(0XA0+((WriteAddr/256)<<1));   //发送器件地址0XA0,写数据 	 
	IIC_Wait_Ack_Sonar();	   
    IIC_Send_Byte_Sonar(WriteAddr%256);   //发送低地址
	IIC_Wait_Ack_Sonar(); 	 										  		   
	IIC_Send_Byte_Sonar(DataToWrite);     //发送字节							   
	IIC_Wait_Ack_Sonar();  		    	   
    IIC_Stop_Sonar();//产生一个停止条件 
	delay_ms(10);	 
}
//在AT24CXX里面的指定地址开始写入长度为Len的数据
//该函数用于写入16bit或者32bit的数据.
//WriteAddr  :开始写入的地址  
//DataToWrite:数据数组首地址
//Len        :要写入数据的长度2,4
void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//在AT24CXX里面的指定地址开始读出长度为Len的数据
//该函数用于读出16bit或者32bit的数据.
//ReadAddr   :开始读出的地址 
//返回值     :数据
//Len        :要读出数据的长度2,4
u32 AT24CXX_ReadLenByte(u16 ReadAddr,u8 Len)
{  	
	u8 t;
	u32 temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=AT24CXX_ReadOneByte(ReadAddr+Len-t-1); 	 				   
	}
	return temp;												    
}
//检查AT24CXX是否正常
//这里用了24XX的最后一个地址(255)来存储标志字.
//如果用其他24C系列,这个地址要修改
//返回1:检测失败
//返回0:检测成功
u8 AT24CXX_Check(void)
{
	u8 temp;
	temp=AT24CXX_ReadOneByte(255);//避免每次开机都写AT24CXX			   
	if(temp==0X55)return 0;		   
	else//排除第一次初始化的情况
	{
		AT24CXX_WriteOneByte(255,0X55);
	    temp=AT24CXX_ReadOneByte(255);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

//在AT24CXX里面的指定地址开始读出指定个数的数据
//ReadAddr :开始读出的地址 对24c02为0~255
//pBuffer  :数据数组首地址
//NumToRead:要读出数据的个数
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  
//在AT24CXX里面的指定地址开始写入指定个数的数据
//WriteAddr :开始写入的地址 对24c02为0~255
//pBuffer   :数据数组首地址
//NumToWrite:要写入数据的个数
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
	while(NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}
 
































