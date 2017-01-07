#include "myiic_sonar.h"
#include "delay.h"
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

//��ʼ��IIC
void IIC_Init_Sonar(void)
{					     
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;
	/*����GPIOB������ʱ��*/
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOFʱ��
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��GPIOFʱ��
 /*ѡ��Ҫ���Ƶ�GPIOC����*/															   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12	;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����

	/*���ÿ⺯������ʼ��GPIOB*/
  	GPIO_Init(GPIOC, &GPIO_InitStructure);		 
		
	/*ѡ��Ҫ���Ƶ�GPIOC����*/															   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
	/*���ÿ⺯������ʼ��GPIOB*/
  GPIO_Init(GPIOD, &GPIO_InitStructure);		 

GPIO_SetBits(GPIOC,GPIO_Pin_12);

GPIO_SetBits(GPIOD,GPIO_Pin_2);
}
//����IIC��ʼ�ź�
void IIC_Start_Sonar(void)
{
	SDA_OUT_Sonar();     //sda�����
	IIC_SDA_Sonar=1;	  	  
	IIC_SCL_Sonar=1;
	delay_us(10);
 	IIC_SDA_Sonar=0;//START:when CLK is high,DATA change form high to low 
	delay_us(10);
	IIC_SCL_Sonar=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop_Sonar(void)
{
	SDA_OUT_Sonar();//sda�����
	IIC_SCL_Sonar=0;
	IIC_SDA_Sonar=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(10);
	IIC_SCL_Sonar=1; 
	IIC_SDA_Sonar=1;//����I2C���߽����ź�
	delay_us(10);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack_Sonar(void)
{
	u8 ucErrTime=0;
	SDA_IN_Sonar();      //SDA����Ϊ����  
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
	IIC_SCL_Sonar=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
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
//������ACKӦ��		    
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
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte_Sonar(u8 txd)
{                        
    u8 t;   
	SDA_OUT_Sonar(); 	    
    IIC_SCL_Sonar=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA_Sonar=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(10);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL_Sonar=1;
		delay_us(10); 
		IIC_SCL_Sonar=0;	
		delay_us(10);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte_Sonar(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN_Sonar();//SDA����Ϊ����
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
        IIC_NAck_Sonar();//����nACK
    else
        IIC_Ack_Sonar(); //����ACK   
    return receive;
}


//��AT24CXXָ����ַ����һ������
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
u8 KS103_ReadOneByte(u8 address, u8 reg)
{				  
	u8 temp=0;		  	    																 
    IIC_Start_Sonar();  
    IIC_Send_Byte_Sonar(address);   //���͵͵�ַ
	IIC_Wait_Ack_Sonar();	 
	IIC_Send_Byte_Sonar(reg);   //���͵͵�ַ
	IIC_Wait_Ack_Sonar();	   
	IIC_Start_Sonar();  	 	   
	IIC_Send_Byte_Sonar(address + 1);           //�������ģʽ			   
	IIC_Wait_Ack_Sonar();	 

	delay_us(50);	   //���Ӵ˴���ͨ�ųɹ�������
    temp=IIC_Read_Byte_Sonar(0);	  //���Ĵ���3	   
    IIC_Stop_Sonar();//����һ��ֹͣ����	    
	return temp;
}
//��AT24CXXָ����ַ����һ������
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{				  
	u8 temp=0;		  	    																 
    IIC_Start_Sonar();  
	if(EE_TYPE>AT24C16)
	{
		IIC_Send_Byte_Sonar(0XA0);	   //����д����
		IIC_Wait_Ack_Sonar();
		IIC_Send_Byte_Sonar(ReadAddr>>8);//���͸ߵ�ַ	    
	}else IIC_Send_Byte_Sonar(0XA0+((ReadAddr/256)<<1));   //����������ַ0XA0,д���� 	   
	IIC_Wait_Ack_Sonar(); 
    IIC_Send_Byte_Sonar(ReadAddr%256);   //���͵͵�ַ
	IIC_Wait_Ack_Sonar();	    
	IIC_Start_Sonar();  	 	   
	IIC_Send_Byte_Sonar(0XA1);           //�������ģʽ			   
	IIC_Wait_Ack_Sonar();	 
    temp=IIC_Read_Byte_Sonar(0);		   
    IIC_Stop_Sonar();//����һ��ֹͣ����	    
	return temp;
}
//��AT24CXXָ����ַд��һ������
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������
void KS103_WriteOneByte(u8 address,u8 reg,u8 command)
{				   	  	    																 
    IIC_Start_Sonar();   
	IIC_Send_Byte_Sonar(address);	    //����д����
	IIC_Wait_Ack_Sonar();
	IIC_Send_Byte_Sonar(reg);//���͸ߵ�ַ	  
	IIC_Wait_Ack_Sonar();	   
    IIC_Send_Byte_Sonar(command);   //���͵͵�ַ
	IIC_Wait_Ack_Sonar(); 	 										  		   
    IIC_Stop_Sonar();//����һ��ֹͣ���� 
//	delay_ms(80);	 
}

//��AT24CXXָ����ַд��һ������
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������
void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite)
{				   	  	    																 
    IIC_Start_Sonar();  
	if(EE_TYPE>AT24C16)
	{
		IIC_Send_Byte_Sonar(0XA0);	    //����д����
		IIC_Wait_Ack_Sonar();
		IIC_Send_Byte_Sonar(WriteAddr>>8);//���͸ߵ�ַ	  
	}else IIC_Send_Byte_Sonar(0XA0+((WriteAddr/256)<<1));   //����������ַ0XA0,д���� 	 
	IIC_Wait_Ack_Sonar();	   
    IIC_Send_Byte_Sonar(WriteAddr%256);   //���͵͵�ַ
	IIC_Wait_Ack_Sonar(); 	 										  		   
	IIC_Send_Byte_Sonar(DataToWrite);     //�����ֽ�							   
	IIC_Wait_Ack_Sonar();  		    	   
    IIC_Stop_Sonar();//����һ��ֹͣ���� 
	delay_ms(10);	 
}
//��AT24CXX�����ָ����ַ��ʼд�볤��ΪLen������
//�ú�������д��16bit����32bit������.
//WriteAddr  :��ʼд��ĵ�ַ  
//DataToWrite:���������׵�ַ
//Len        :Ҫд�����ݵĳ���2,4
void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//��AT24CXX�����ָ����ַ��ʼ��������ΪLen������
//�ú������ڶ���16bit����32bit������.
//ReadAddr   :��ʼ�����ĵ�ַ 
//����ֵ     :����
//Len        :Ҫ�������ݵĳ���2,4
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
//���AT24CXX�Ƿ�����
//��������24XX�����һ����ַ(255)���洢��־��.
//���������24Cϵ��,�����ַҪ�޸�
//����1:���ʧ��
//����0:���ɹ�
u8 AT24CXX_Check(void)
{
	u8 temp;
	temp=AT24CXX_ReadOneByte(255);//����ÿ�ο�����дAT24CXX			   
	if(temp==0X55)return 0;		   
	else//�ų���һ�γ�ʼ�������
	{
		AT24CXX_WriteOneByte(255,0X55);
	    temp=AT24CXX_ReadOneByte(255);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

//��AT24CXX�����ָ����ַ��ʼ����ָ������������
//ReadAddr :��ʼ�����ĵ�ַ ��24c02Ϊ0~255
//pBuffer  :���������׵�ַ
//NumToRead:Ҫ�������ݵĸ���
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  
//��AT24CXX�����ָ����ַ��ʼд��ָ������������
//WriteAddr :��ʼд��ĵ�ַ ��24c02Ϊ0~255
//pBuffer   :���������׵�ַ
//NumToWrite:Ҫд�����ݵĸ���
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
	while(NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}
 
































