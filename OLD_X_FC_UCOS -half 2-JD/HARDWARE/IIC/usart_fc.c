
#include "include.h"
#include "usart_fc.h"
#include "ultrasonic.h"
#include "hml5833l.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "rc.h"
#include "att.h"
#include "height_ctrl.h"
#include "alt_kf.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "flow.h"
#include "gps.h"
#include "circle.h"


void Usart2_Init(u32 br_num)//--GOL-link
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 

	

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART2, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART2, ENABLE); 
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}


}
RESULT color;
float dt_flow_rx;
u16 data_rate_gol_link;
PID_STA HPID,SPID,FIX_PID,NAV_PID;
struct _PID_SET pid;
RC_GETDATA Rc_Get;
struct _plane plane;
struct _slam slam;
struct _IMU_NAV imu_nav;
struct _MODE mode;
u8 LOCK, KEY[8],KEY_SEL[4];

u8 NAV_BOARD_CONNECT=0;
 void Data_Receive_Anl(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
  if(*(data_buf+2)==0x86)//FLOW_MINE_frame
  {
	 dt_flow_rx= Get_Cycle_T(GET_T_FLOW_UART);	
   NAV_BOARD_CONNECT=1;
	 imu_nav.flow.speed.west = -(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));///10.;//UKF
	 imu_nav.flow.speed.east = -(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7));///10.; 
	 imu_nav.flow.speed.x = -(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/100.;//Origin
	 imu_nav.flow.speed.y = -(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/100.;
	 imu_nav.flow.speed_h.x = (float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/100.;//Origin
	 imu_nav.flow.speed_h.y = (float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/100.;
   //imu_nav.flow.position.flow_qual = ((int16_t)(*(data_buf+12)<<8)|*(data_buf+13));
		
	 color.x = ((int16_t)(*(data_buf+14)<<8)|*(data_buf+15));
	 color.y = ((int16_t)(*(data_buf+16)<<8)|*(data_buf+17));
	 color.h = ((int16_t)(*(data_buf+18)<<8)|*(data_buf+19));
   color.check = *(data_buf+20);
	 imu_nav.flow.position.flow_qual = *(data_buf+21);
		

	}
	else  if(*(data_buf+2)==0x66)//FLOW_MINE_frame
  {
   	
	}
  else if(*(data_buf+2)==0x01)//FLOW_MINE_frame
  {
	 dt_flow_rx= Get_Cycle_T(GET_T_FLOW_UART);	
   NAV_BOARD_CONNECT=1;
	 imu_nav.flow.speed.west = (float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/1000;///10.;
	 imu_nav.flow.speed.east = (float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/1000;///10.; 
	 imu_nav.flow.speed.x = (float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/1000;
	 imu_nav.flow.speed.y = (float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/1000;


	}			
}
 u8 SONAR_HEAD_CHECK[2];
u32 imu_loss_cnt;
u16 S_head;
float Pitch_DJ,Roll_DJ;
float ALT_POS_SONAR_HEAD,ALT_POS_SONAR_HEAD_LASER_SCANER;
 void Data_IMU(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
  if(*(data_buf+2)==0x01)//FLOW_MINE_frame
  {
	  imu_loss_cnt=0;
    NAV_BOARD_CONNECT=1;
		imu_nav.flow.rate=*(data_buf+4);
		imu_nav.flow.speed.y_f=(float)((int16_t)((*(data_buf+5)<<8)|*(data_buf+6)))/1000.;
		imu_nav.flow.speed.x_f=(float)((int16_t)((*(data_buf+7)<<8)|*(data_buf+8)))/1000.;
		imu_nav.flow.speed.y = (float)(int16_t)((*(data_buf+9)<<8)|*(data_buf+10))/1000.;
		imu_nav.flow.speed.x = (float)(int16_t)((*(data_buf+11)<<8)|*(data_buf+12))/1000.;
		now_position[LON]=(float)(int16_t)((*(data_buf+13)<<8)|*(data_buf+14))/100.;//m
		now_position[LAT]=(float)(int16_t)((*(data_buf+15)<<8)|*(data_buf+16))/100.;//m
		//ALT_VEL_SONAR=(float)(int16_t)((*(data_buf+17)<<8)|*(data_buf+18))/1000.;//m
		float temp=(float)(int16_t)((*(data_buf+19)<<8)|*(data_buf+20))/1000.;//m
			if(temp<8.888)
	  ALT_POS_SONAR_HEAD = temp;
		SONAR_HEAD_CHECK[0]=*(data_buf+4);
    S_head=(float)(int16_t)((*(data_buf+33)<<8)|*(data_buf+34));
			
		//ALT_VEL_BMP=(float)(int16_t)((*(data_buf+21)<<8)|*(data_buf+22))/1000.;//m
		//ALT_POS_BMP=(float)(int32_t)((*(data_buf+23)<<24)|(*(data_buf+24)<<16)|(*(data_buf+25)<<8)|*(data_buf+26))/1000.;//m
		
	}	
  else  if(*(data_buf+2)==0x66)//FLOW_MINE_frame
  {
    float temp=(float)(int16_t)((*(data_buf+4)<<8)|*(data_buf+5))/1000.;//m
			if(temp<8.888)
	  ALT_POS_SONAR_HEAD = temp;
    S_head=(float)(int16_t)((*(data_buf+6)<<8)|*(data_buf+7));	
	}
	
 	/* else if(*(data_buf+2)==0x12)//debug
  {
	flow_debug.en_ble_debug= *(data_buf+4);	
	flow_debug.ax=((int16_t)(*(data_buf+5)<<8)|*(data_buf+6));
	flow_debug.ay=((int16_t)(*(data_buf+7)<<8)|*(data_buf+8));
	flow_debug.az=((int16_t)(*(data_buf+9)<<8)|*(data_buf+10));
	flow_debug.gx=((int16_t)(*(data_buf+11)<<8)|*(data_buf+12));
	flow_debug.gy=((int16_t)(*(data_buf+13)<<8)|*(data_buf+14));
	flow_debug.gz=((int16_t)(*(data_buf+15)<<8)|*(data_buf+16));
	flow_debug.hx=((int16_t)(*(data_buf+17)<<8)|*(data_buf+18));
	flow_debug.hy=((int16_t)(*(data_buf+19)<<8)|*(data_buf+20));
  flow_debug.hz=((int16_t)(*(data_buf+21)<<8)|*(data_buf+22));

	}		
	else if(*(data_buf+2)==0x02)//CAL
  {
	  mpu6050.Acc_Offset.x=(int16_t)(*(data_buf+4)<<8)|*(data_buf+5);
    mpu6050.Acc_Offset.y=(int16_t)(*(data_buf+6)<<8)|*(data_buf+7);
		mpu6050.Acc_Offset.z=(int16_t)(*(data_buf+8)<<8)|*(data_buf+9);
		mpu6050.Gyro_Offset.x=(int16_t)(*(data_buf+10)<<8)|*(data_buf+11);
		mpu6050.Gyro_Offset.y=(int16_t)(*(data_buf+12)<<8)|*(data_buf+13);
		mpu6050.Gyro_Offset.z=(int16_t)(*(data_buf+14)<<8)|*(data_buf+15);
		ak8975.Mag_Offset.x=(int16_t)(*(data_buf+16)<<8)|*(data_buf+17);
		ak8975.Mag_Offset.y=(int16_t)(*(data_buf+18)<<8)|*(data_buf+19);
		ak8975.Mag_Offset.z=(int16_t)(*(data_buf+20)<<8)|*(data_buf+21);
		ak8975.Mag_Gain.x=(float)((int16_t)(*(data_buf+22)<<8)|*(data_buf+23))/1000.;
		ak8975.Mag_Gain.y=(float)((int16_t)(*(data_buf+24)<<8)|*(data_buf+25))/1000.;
		ak8975.Mag_Gain.z=(float)((int16_t)(*(data_buf+26)<<8)|*(data_buf+27))/1000.;
	}		
else if(*(data_buf+2)==0x10)//Mems
  {
	  mpu6050.Acc.x=(int16_t)(*(data_buf+4)<<8)|*(data_buf+5);
    mpu6050.Acc.y=(int16_t)(*(data_buf+6)<<8)|*(data_buf+7);
		mpu6050.Acc.z=(int16_t)(*(data_buf+8)<<8)|*(data_buf+9);
		mpu6050.Gyro.x=(int16_t)(*(data_buf+10)<<8)|*(data_buf+11);
		mpu6050.Gyro.y=(int16_t)(*(data_buf+12)<<8)|*(data_buf+13);
		mpu6050.Gyro.z=(int16_t)(*(data_buf+14)<<8)|*(data_buf+15);
		mpu6050.Gyro_deg.x = mpu6050.Gyro.x *TO_ANGLE;
		mpu6050.Gyro_deg.y = mpu6050.Gyro.y *TO_ANGLE;
		mpu6050.Gyro_deg.z = mpu6050.Gyro.z *TO_ANGLE;
		ak8975.Mag_Val.x=(int16_t)(*(data_buf+16)<<8)|*(data_buf+17);
		ak8975.Mag_Val.y=(int16_t)(*(data_buf+18)<<8)|*(data_buf+19);
		ak8975.Mag_Val.z=(int16_t)(*(data_buf+20)<<8)|*(data_buf+21);
		int temp=(int16_t)(*(data_buf+22)<<8)|*(data_buf+23);
		if(temp<3200)
	  ultra_distance = temp;
	  baroAlt=(int32_t)((*(data_buf+24)<<24)|(*(data_buf+25)<<16)|(*(data_buf+26)<<8)|*(data_buf+27));
	}	*/	
	else if(*(data_buf+2)==0x11)//Att
  { //dt_flow_rx= Get_Cycle_T(GET_T_FLOW_UART);	
	  Pitch=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/10.;
    Roll=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/10.;
		Yaw=(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/10.;
		q_nav[0]=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/1000.;
		q_nav[1]=(float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/1000.;
		q_nav[2]=(float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/1000.;
		q_nav[3]=(float)((int16_t)(*(data_buf+16)<<8)|*(data_buf+17))/1000.;
		Pitch_DJ=(float)((int16_t)(*(data_buf+18)<<8)|*(data_buf+19))/10.;
    Roll_DJ= (float)((int16_t)(*(data_buf+20)<<8)|*(data_buf+21))/10.;
		//Pitch_mid_down=(float)((int16_t)(*(data_buf+18)<<8)|*(data_buf+19))/10.;
    //Roll_mid_down=(float)((int16_t)(*(data_buf+20)<<8)|*(data_buf+21))/10.;
		//Yaw_mid_down=(float)((int16_t)(*(data_buf+22)<<8)|*(data_buf+23))/10.;
		//ref_q_imd_down[0]=(float)((int16_t)(*(data_buf+24)<<8)|*(data_buf+25))/1000.;
		//ref_q_imd_down[1]=(float)((int16_t)(*(data_buf+26)<<8)|*(data_buf+27))/1000.;
		//ref_q_imd_down[2]=(float)((int16_t)(*(data_buf+28)<<8)|*(data_buf+29))/1000.;
		//ref_q_imd_down[3]=(float)((int16_t)(*(data_buf+30)<<8)|*(data_buf+31))/1000.;
		ALT_VEL_BMP_EKF=(float)(int16_t)((*(data_buf+32)<<8)|*(data_buf+33))/1000.;//m
		
		
  	//reference_vr[0]=reference_v.x = 2*(q_nav[1]*q_nav[3] - q_nav[0]*q_nav[2]);
  	//reference_vr[1]=reference_v.y = 2*(q_nav[0]*q_nav[1] + q_nav[2]*q_nav[3]);
	  //reference_vr[2]=reference_v.z = 1 - 2*(q_nav[1]*q_nav[1] + q_nav[2]*q_nav[2]);
	  //reference_vr_imd_down[0] = 2*(ref_q_imd_down[1]*ref_q_imd_down[3] - ref_q_imd_down[0]*ref_q_imd_down[2]);
	  //reference_vr_imd_down[1] = 2*(ref_q_imd_down[0]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[3]);
	  //reference_vr_imd_down[2] = 1 - 2*(ref_q_imd_down[1]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[2]);
	
	}		
}
u8 laser_sel=1;

u8 TxBuffer[256];
u8 TxCounter=0;
u8 count=0; 

u8 Rx_Buf[256];	//串口接收缓存
u8 RxBuffer[50];
u8 RxState = 0;
u8 RxBufferNum = 0;
u8 RxBufferCnt = 0;
u8 RxLen = 0;
u8 com_data ;
static u8 _data_len = 0,_data_cnt = 0;
void USART2_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(USART2->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART2->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART2,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);//清除中断标志

		com_data = USART2->DR;
				if(RxState==0&&com_data==0xAA)
		{
			RxState=1;
			RxBuffer[0]=com_data;
		}
		else if(RxState==1&&com_data==0xAF)
		{
			RxState=2;
			RxBuffer[1]=com_data;
		}
		else if(RxState==2&&com_data>0&&com_data<0XF1)
		{
			RxState=3;
			RxBuffer[2]=com_data;
		}
		else if(RxState==3&&com_data<50)
		{
			RxState = 4;
			RxBuffer[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if(RxState==4&&_data_len>0)
		{
			_data_len--;
			RxBuffer[4+_data_cnt++]=com_data;
			if(_data_len==0)
				RxState = 5;
		}
		else if(RxState==5)
		{
			RxState = 0;
			RxBuffer[4+_data_cnt]=com_data;
			Data_IMU(RxBuffer,_data_cnt+5);
		}
		else
			RxState = 0;
	
		//ANO_DT_Data_Receive_Prepare(com_data);
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(USART2,USART_IT_TXE ) )
	{
				
		USART2->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}


		//USART_ClearITPendingBit(USART2,USART_IT_TXE);
	}

   OSIntExit(); 

}

void UsartSend_GOL_LINK(uint8_t ch)
{


	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
USART_SendData(USART2, ch); 
}

static void Send_Data_GOL_LINK(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GOL_LINK(dataToSend[i]);
}


//------------------------------------------------------GOL_LINK----------------------------------------------------

void Send_IMU_TO_FLOW(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//功能字
	data_to_send[_cnt++]=0;//数据量
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = mpu6050.Acc.x;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = mpu6050.Acc.y;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = mpu6050.Acc.z;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);


	
	_temp = q_nav[0]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = q_nav[1]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[2]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[3]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
 	_temp = 0;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  0;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}



void Send_IMU(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x80;//功能字
	data_to_send[_cnt++]=0;//数据量
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(mpu6050.Gyro_deg.x*100);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(mpu6050.Gyro_deg.y*100);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(mpu6050.Gyro_deg.z*100);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);


	
	_temp = (vs16)( mpu6050.Acc.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( mpu6050.Acc.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( mpu6050.Acc.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
		
	_temp = (vs16)LIMIT(ALT_POS_SONAR*1000,0,2500);//ultra_distance;(baro_to_ground);//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
		
	_temp = (vs16)(-ALT_VEL_SONAR*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(-ALT_VEL_BMP*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}  
#include "pwm_in.h"
void Send_RC_TO_IMU(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x15;//功能字
	data_to_send[_cnt++]=0;//数据量
	#if USE_M100
	_temp = (vs16)(Rc_Pwm_Out_mine[RC_PITCH])+20;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Rc_Pwm_Out_mine[RC_ROLL])+20;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Rc_Pwm_Out_mine[RC_YAW])+20;//ultra_ctrl_out_head+1500;//(vs16)(Rc_Pwm_Out_mine[RC_THR])+20;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Rc_Pwm_Out_mine[RC_YAW])+20;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	#else
	_temp = (vs16)(Rc_Pwm_Out_mine[RC_PITCH]);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Rc_Pwm_Out_mine[RC_ROLL]);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Rc_Pwm_Out_mine[RC_THR]);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Rc_Pwm_Out_mine[RC_YAW]);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	#endif
	_temp = (vs16)(ALT_POS_SONAR2*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(exp_height);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(ALT_POS_SONAR_HEAD_LASER_SCANER*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(exp_height_head);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(now_position[1]*100);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(target_position[1]*100);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	_temp = (vs16)nav_Data.gps_ero_dis_lpf[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)nav_Data.gps_ero_dis_lpf[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(state_v);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.GPS_STATUS);
	data_to_send[_cnt++]=BYTE0(_temp);
	#if USE_M100
		_temp = 1;//是否使用经纬100
	#else
	  _temp = 0;
	#endif
	data_to_send[_cnt++]=BYTE0(_temp);


	_temp = (vs16)(m100.Yaw*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(laser_sel);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100_data_refresh);//&&circle.connect);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}

void Send_STATE1_IMU(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//功能字
	data_to_send[_cnt++]=0;//数据量
	#if USE_M100
	  _temp = (vs16)(m100.Rc_gear<-9000);
	#else
	  _temp = (vs16)(Rc_Pwm_Inr_mine[RC_THR]>200+1000);
	#endif
	  data_to_send[_cnt++]=BYTE0(_temp);
		_temp=mpu6050.Acc_CALIBRATE;
		data_to_send[_cnt++]=BYTE0(_temp);
		_temp=mpu6050.Gyro_CALIBRATE;
		data_to_send[_cnt++]=BYTE0(_temp);
		_temp=Mag_CALIBRATED;
	  data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}
void Send_IMU_TO_FLOW2(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//功能字
	data_to_send[_cnt++]=0;//数据量
	
	#if USE_M100
	  _temp = (vs16)(m100.Rc_gear<-9000);
	#else
	  _temp = (vs16)(Rc_Pwm_Inr_mine[RC_THR]>200+1000);
	#endif
	data_to_send[_cnt++]=BYTE0(_temp);	
	_temp =  mpu6050.Acc_CALIBRATE;
	data_to_send[_cnt++]=BYTE0(_temp);	
	_temp =  mpu6050.Gyro_CALIBRATE;
	data_to_send[_cnt++]=BYTE0(_temp);	
	_temp =  Mag_CALIBRATED;
	data_to_send[_cnt++]=BYTE0(_temp);	

	_temp=qr_local_pos[1]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=qr_local_pos[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=drone_local_pos[1]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=drone_local_pos[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=tar_drone_local_pos[1]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=tar_drone_local_pos[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

//	_temp =(vs16)( mpu6050.Acc_I16.x);//ultra_distance;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
// 	_temp = (vs16)(mpu6050.Acc_I16.y);//ultra_distance;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = (vs16)(mpu6050.Acc_I16.z);//ultra_distance;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);


//	
//	_temp = (vs16)(mpu6050.Gyro_I16.x);//q_nav[0]*1000;//ultra_distance;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
// 	_temp = (vs16)(mpu6050.Gyro_I16.y);//q_nav[1]*1000;//ultra_distance;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = (vs16)(mpu6050.Gyro_I16.z);// q_nav[2]*1000;//ultra_distance;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	
//	_temp = (vs16)(ak8975.Mag_Adc.x);//ultra_distance;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
// 	_temp = (vs16)(ak8975.Mag_Adc.y);//ultra_distance;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = (vs16)(ak8975.Mag_Adc.z);//mode.save_video;//ultra_distance;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp =  mpu6050.Acc_CALIBRATE;
//	data_to_send[_cnt++]=BYTE0(_temp);	
//	_temp =  mpu6050.Gyro_CALIBRATE;
//	data_to_send[_cnt++]=BYTE0(_temp);	
//	_temp =  Mag_CALIBRATED;
//	data_to_send[_cnt++]=BYTE0(_temp);		
//	_temp =  mode.save_video;
//	data_to_send[_cnt++]=BYTE0(_temp);	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
		Send_Data_GOL_LINK(data_to_send, _cnt);
}

void GOL_LINK_TASK(void)
{
static u8 cnt[4];

if(cnt[1]++>0)
{cnt[1]=0;
//	Send_STATE1_IMU();
	Send_RC_TO_IMU();
} 
if(cnt[0]++>1)
{cnt[0]=0;
	Send_IMU_TO_FLOW2();
	
} 
}


void Uart5_Init(u32 br_num)//-----video sonar F
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
	
	//配置PC12作为UART5　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PD2作为UART5　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	//配置UART5
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	USART_Init(UART5, &USART_InitStructure);
	


	//使能UART5接收中断
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	//使能USART5
	USART_Cmd(UART5, ENABLE); 
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}

}
u8 Tx5Buffer[256];
u8 Tx5Counter=0;
u8 count5=0; 


void UART5_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(UART5->SR & USART_SR_ORE)//ORE中断
	{
		com_data = UART5->DR;
	}

  //接收中断
	if( USART_GetITStatus(UART5,USART_IT_RXNE) )
	{
		
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);//清除中断标志
		com_data = UART5->DR;
		Ultra_Get(com_data);
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(UART5,USART_IT_TXE ) )
	{
				
		UART5->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			UART5->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}


		USART_ClearITPendingBit(UART5,USART_IT_TXE);
	}
  
   OSIntExit(); 

}

void Uart5_Send(unsigned char *DataToSend ,u8 data_num)
{

while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
USART_SendData(UART5, data_num); ;//USART1, ch); 

}


void Usart1_Init(u32 br_num)//-------UPload_board1
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	/*//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	*/

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

//	//配置PA3  WK  BLE控制端
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; 
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
//	GPIO_SetBits(GPIOA,GPIO_Pin_3);//透传模式

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART1, ENABLE); 
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}


}

void Usart4_Init(u32 br_num)//-------SD_board
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

	

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(UART4, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(UART4, ENABLE);  //使能串口1 
	
	USART_ClearFlag(UART4, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(UART4, ENABLE); 
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}


}



#include "gps.h"

nmea_msg gpsx,gps_data; 		
u8  buf_GPRMC[100]={'G','P','R','M','C',','};//GPS信息
u8  buf_GPRMCt[100]={"GPRMC,144326.00,A,5107.0017737,N,11402.3291611,W,0.080,323.3,210307,0.0,E,A*20"};//GPS信息
u8  buf_GPGGA[100]={'G','P','G','G','A',','};//GPS信息
u8  buf_GPGGAt[100]={"GPGGA,134658.00,5106.9792,N,11402.3003,W,2,09,1.0,1048.47,M,-16.27,M,08,AAAA*60"};//GPS信息
u8  buf_imu_dj[20];
float angle_imu_dj[3];
void IMU_DJ_ANGLE(u8 data)
{
//2.1.2 请求GPRMC
//说明：请求输出时间、日期、定位及地面航向和地面速度
	static u8 state0,cnt_buf;
switch(state0){
			case 0:if(data=='R')
			state0=1;
			break;
			case 1:if(data=='M')
			{state0=2;cnt_buf=0;}
			else
			state0=0;
			break;
			case 2:if(data=='C')
			{state0=3;cnt_buf=0;}
			else
			state0=0;
			break;
			case 3:if(data==',')
			{state0=4;cnt_buf=0;}
			else
			state0=0;
			break;
			case 4:if(data=='*')
			{buf_GPRMC[6+cnt_buf++]=data;state0=5;}
			else if(cnt_buf>90)
			{cnt_buf=0;state0=0;}	
			else
			buf_GPRMC[6+cnt_buf++]=data;
			break;
			case 5:
				#if TEST_GPS
			NMEA_GPRMC_Analysis(&gpsx,buf_GPRMCt);	
			#else
			NMEA_GPRMC_Analysis(&gpsx,buf_GPRMC);	//GPRMC解析	
			#endif
//			for(cnt_buf=6;cnt_buf<sizeof(buf_GPRMC);cnt_buf++)
//			buf_GPRMC[cnt_buf]=0;
			cnt_buf=0;state0=0;
			break;
		}
		
//$GPGGA,134658.00,5106.9792,N,11402.3003,W,2,09,1.0,1048.47,M,-16.27,M,08,AAAA*60
		static u8 state1,cnt_buf1;
switch(state1){
			case 0:if(data=='G')
			state1=1;
			break;
			case 1:if(data=='G')
			{state1=2;cnt_buf1=0;}
			else
			state1=0;
			break;
			case 2:if(data=='A')
			{state1=3;cnt_buf1=0;}
			else
			state1=0;
			break;
			case 3:if(data==',')
			{state1=4;cnt_buf1=0;}
			else
			state1=0;
			break;
			case 4:if(data=='*')
			{buf_GPRMC[6+cnt_buf1++]=data;state1=5;}
			else if(cnt_buf1>90)
			{cnt_buf1=0;state1=0;}	
			else
			buf_GPGGA[6+cnt_buf1++]=data;
			break;
			case 5:
							#if TEST_GPS
			NMEA_GPGGA_Analysis(&gpsx,buf_GPGGAt);	
			#else
			NMEA_GPGGA_Analysis(&gpsx,buf_GPGGA);	//GPRMC解析	
			#endif
//			for(cnt_buf=6;cnt_buf<sizeof(buf_GPRMC);cnt_buf++)
//			buf_GPRMC[cnt_buf]=0;
			cnt_buf1=0;state1=0;
			break;
		}
}

u8 buf_imu_dj[20];
float angle_imu_dj[3];
/*
void IMU_DJ_ANGLE(u8 data)
{
static u8 state=0;
	
switch(state)
{
u8 temp;
static u8 cnt;
	case 0:if(data==0x55)
				state=1;
	  break;
	case 1:if(data==0x53)
	{  state=2;cnt=0;}
	     else
				 state=0;
	  break;
	case 2:
		buf_imu_dj[cnt++]=data;
	 if(cnt>8)
		 state=3;
	 break;
	case 3:
		temp=0x55+0x53+ buf_imu_dj[0]+buf_imu_dj[1]+
										buf_imu_dj[2]+buf_imu_dj[3]+
										buf_imu_dj[4]+buf_imu_dj[5]+
										buf_imu_dj[6]+buf_imu_dj[7];
	  if(temp==buf_imu_dj[8])
		{angle_imu_dj[0]=To_180_degrees((float)(buf_imu_dj[1]<<8|buf_imu_dj[0])/32768.*180);
		 angle_imu_dj[1]=To_180_degrees((float)(buf_imu_dj[2]<<8|buf_imu_dj[3])/32768.*180);
		}
		 state=0;
		break;
}



}*/


u16 cnt_m100_data_refresh;
u8 m100_data_refresh;
 void DATA_M100(u8 *data_buf,u8 num)
{double zen,xiao;
	vs16 rc_value_temp;
	static float m100_hr,m100_attr[3];
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
  if(*(data_buf+2)==0x01)//
  { dji_miss_cnt=0;
		DJI_CONNECT=1;
	  m100.Pit=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/10.;
		m100.Rol=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/10.;
		m100.Yaw=-1*(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/10.;
		
		m100.H=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/1000.;
		
		if(m100.H!=m100_hr||m100_attr[0]!=m100.Pit||m100_attr[1]!=m100.Rol||m100_attr[2]!=m100.Yaw)
		{cnt_m100_data_refresh=0;
		 m100_data_refresh=1;
		}
		m100_hr=m100.H;
		m100_attr[0]=m100.Pit;
		m100_attr[1]=m100.Rol;
		m100_attr[2]=m100.Yaw;
		
		m100.H_Spd=(float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/1000.;
		zen=(*(data_buf+14)<<8)|*(data_buf+15);
		xiao=(double)((u32)(*(data_buf+16)<<24)|(*(data_buf+17)<<16)|(*(data_buf+18)<<8)|*(data_buf+19))/1000000000.;
		m100.Lat=zen+xiao;
		zen=(*(data_buf+20)<<8)|*(data_buf+21);
		xiao=(double)((u32)(*(data_buf+22)<<24)|(*(data_buf+23)<<16)|(*(data_buf+24)<<8)|*(data_buf+25))/1000000000.;
		m100.Lon=zen+xiao;
		
		m100.Bat=(float)((int16_t)(*(data_buf+26)<<8)|*(data_buf+27))/100.;
		m100.Rc_pit=(float)((int16_t)(*(data_buf+28)<<8)|*(data_buf+29));
		m100.Rc_rol=(float)((int16_t)(*(data_buf+30)<<8)|*(data_buf+31));
		m100.Rc_yaw=(float)((int16_t)(*(data_buf+32)<<8)|*(data_buf+33));
		m100.Rc_thr=(float)((int16_t)(*(data_buf+34)<<8)|*(data_buf+45));
		m100.Rc_mode=(float)((int16_t)(*(data_buf+36)<<8)|*(data_buf+37));
		m100.Rc_gear=(float)((int16_t)(*(data_buf+38)<<8)|*(data_buf+39));
		m100.STATUS=*(data_buf+40);		
		m100.GPS_STATUS=*(data_buf+41);
		m100.Spd[0]=(float)((int16_t)(*(data_buf+42)<<8)|*(data_buf+43))/1000.;
		m100.Spd[1]=(float)((int16_t)(*(data_buf+44)<<8)|*(data_buf+45))/1000.;
	}			
}

u8 TxBuffer4[256];
u8 TxCounter4=0;
u8 count4=0; 

u8 Rx_Buf4[256];	//串口接收缓存
u8 RxBuffer4[50];
u8 RxState4 = 0;
u8 RxBufferNum4 = 0;
u8 RxBufferCnt4 = 0;
u8 RxLen4= 0;
static u8 _data_len4 = 0,_data_cnt4 = 0;
void UART4_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(UART4->SR & USART_SR_ORE)//ORE中断
	{
		com_data = UART4->DR;
	}

  //接收中断
	if( USART_GetITStatus(UART4,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART4,USART_IT_RXNE);//清除中断标志

		com_data = UART4->DR;
		#if USE_M100
			if(RxState4==0&&com_data==0xAA)
		{
			RxState4=1;
			RxBuffer4[0]=com_data;
		}
		else if(RxState4==1&&com_data==0xAF)
		{
			RxState4=2;
			RxBuffer4[1]=com_data;
		}
		else if(RxState4==2&&com_data>0&&com_data<0XF1)
		{
			RxState4=3;
			RxBuffer4[2]=com_data;
		}
		else if(RxState4==3&&com_data<50)
		{
			RxState4 = 4;
			RxBuffer4[3]=com_data;
			_data_len4 = com_data;
			_data_cnt4 = 0;
		}
		else if(RxState4==4&&_data_len4>0)
		{
			_data_len4--;
			RxBuffer4[4+_data_cnt4++]=com_data;
			if(_data_len4==0)
				RxState4 = 5;
		}
		else if(RxState4==5)
		{
			RxState4 = 0;
			RxBuffer4[4+_data_cnt4]=com_data;
			DATA_M100(RxBuffer4,_data_cnt4+5);
		}
		else
			RxState4 = 0;
		
		#else
		IMU_DJ_ANGLE(com_data);
		#endif
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(UART4,USART_IT_TXE ) )
	{
				
		UART4->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			UART4->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}


		//USART_ClearITPendingBit(USART2,USART_IT_TXE);
	}
  
   OSIntExit(); 

}

void UsartSend_M100(uint8_t ch)
{

while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
USART_SendData(UART4, ch);  
}


void UsartSend_SD(uint8_t ch)
{

while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
USART_SendData(USART2, ch); ;//USART1, ch); 
}

static void Send_Data_SD(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_SD(dataToSend[i]);
}

void UART2_Put_Char(unsigned char DataToSend)
{

while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, DataToSend); 

}

void UART2_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
{
 	unsigned int temp=0xaF+9;
	char ctemp;
	UART2_Put_Char(0xa5);
	UART2_Put_Char(0x5a);
	UART2_Put_Char(14+8);
	UART2_Put_Char(0xA2);

	if(ax<0)ax=32768-ax;
	ctemp=ax>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=ax;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(ay<0)ay=32768-ay;
	ctemp=ay>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=ay;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(az<0)az=32768-az;
	ctemp=az>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=az;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(gx<0)gx=32768-gx;
	ctemp=gx>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gx;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(gy<0)gy=32768-gy;
	ctemp=gy>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gy;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
//-------------------------
	if(gz<0)gz=32768-gz;
	ctemp=gz>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gz;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(hx<0)hx=32768-hx;
	ctemp=hx>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hx;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(hy<0)hy=32768-hy;
	ctemp=hy>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hy;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(hz<0)hz=32768-hz;
	ctemp=hz>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hz;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	UART2_Put_Char(temp%256);
	UART2_Put_Char(0xaa);
}


void UART2_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
 	unsigned int temp=0xaF+2+2;
	char ctemp;
	UART2_Put_Char(0xa5);
	UART2_Put_Char(0x5a);
	UART2_Put_Char(14+4);
	UART2_Put_Char(0xA1);

	if(yaw<0)yaw=32768-yaw;
	ctemp=yaw>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=yaw;							
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(pitch<0)pitch=32768-pitch;
	ctemp=pitch>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=pitch;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
								 
	if(roll<0)roll=32768-roll;
	ctemp=roll>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=roll;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(alt<0)alt=32768-alt;
	ctemp=alt>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;				
	ctemp=alt;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(tempr<0)tempr=32768-tempr;
	ctemp=tempr>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=tempr;
	UART2_Put_Char(ctemp);	   
	temp+=ctemp;

	if(press<0)press=32768-press;
	ctemp=press>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=press;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	ctemp=IMUpersec>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=IMUpersec;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	UART2_Put_Char(temp%256);
	UART2_Put_Char(0xaa);
}



//------------------------------------------------------GOL_LINK_SD----------------------------------------------------
void Send_IMU_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x05;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = Roll*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Pitch*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Yaw*10	;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[1]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[2]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[3]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	

data_to_send[3] = _cnt-4;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}

void Send_ATT_PID_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = except_A.y*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = except_A.x*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = except_A.z*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	
	
	
	
	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}

void Send_FLOW_PID_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x02;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = target_position[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = target_position[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = now_position[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = now_position[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = tar_speed[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = tar_speed[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = actual_speed[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = actual_speed[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = nav[ROLr]*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = nav[PITr]*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}


void Send_ALT_PID_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x03;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = exp_height;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = ultra_dis_lpf;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = ultra_ctrl_out;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = wz_speed;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = thr_test	;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = 0;//baroAlt	;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}


void Send_MODE_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x04;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = mode.en_sd_save;
	data_to_send[_cnt++]=BYTE0(_temp);

	
	
data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}

void Send_FLOW_USE_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x06;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = imu_nav.flow.speed.east ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = imu_nav.flow.speed.west;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  imu_nav.flow.speed.x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = imu_nav.flow.speed.y ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = imu_nav.flow.position.east	;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = imu_nav.flow.position.west	;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}


void Send_SLAM_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x07;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp = slam.dis[2];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = slam.dis[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = slam.dis[3];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = slam.dis[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = slam.dis[4];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	

	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}


void Send_GPS_SD(void)
{u8 i;	u8 sum = 0;u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x08;//功能字
	data_to_send[_cnt++]=0;//数据量
	_temp32 =  imu_nav.gps.J;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.W;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	_temp =imu_nav.gps.gps_mode;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =imu_nav.gps.star_num;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 =  imu_nav.gps.X_O;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.Y_O;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.X_UKF;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.Y_UKF;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	
	

	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}

//void SD_LINK_TASK(void)
//{
//static u8 cnt[4];
//static u8 flag;
//if(cnt[0]++>1)
//{cnt[0]=0;
//Send_ATT_PID_SD();
//}
//if(cnt[1]++>0)
//{cnt[1]=0;
////	if(flag)
////	{flag=0;
//	Send_IMU_SD();
////}
////	else{flag=1;
////	Send_MODE_SD();
////	}
//}
//if(cnt[2]++>2)
//{cnt[2]=0;
//	//Send_FLOW_PID_SD();
//	Send_FLOW_USE_SD();
//}
//if(cnt[3]++>2)
//{cnt[3]=0;
//	//Send_ALT_PID_SD();
//	Send_GPS_SD();//Send_SLAM_SD();
//}

//}

/*
#define SEND_IMU 0
#define SEND_FLOW 1
#define SEND_GPS 2
#define SEND_ALT 3
*/
void SD_LINK_TASK2(u8 sel)
{
static u8 cnt[4];
static u8 flag;
	
	switch(sel)
	{
		case SEND_IMU:
				Send_IMU_SD();
		break;
		case SEND_FLOW:
				Send_FLOW_USE_SD();
		break;
		case SEND_GPS:
				Send_GPS_SD();
		break;
		case SEND_ALT:
				Send_ALT_PID_SD();
		break;
	}

 Send_MODE_SD();
}

void Usart3_Init(u32 br_num)//-------GPS_board
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 

	
   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART3, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART3, ENABLE); 
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}


}
#include "circle.h"
#include "filter.h"
u16 laser_o[5];
float k_laser=0.8;
float rate_gps_board;
CIRCLE qr;
u8 avoid_color[4];
 void Data_Receive_Anl3(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp,rc_value_temp1;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	if(*(data_buf+2)==0x02)//SLAM_frame
  {
	circle.connect=1;
	circle.lose_cnt=0;
	circle.check=(*(data_buf+4));///10.;
  rc_value_temp=((int16_t)(*(data_buf+5)<<8)|*(data_buf+6));
	if(rc_value_temp<320)
	circle.x=rc_value_temp;//Moving_Median(16,3,rc_value_temp);
	circle.y=(int16_t)(*(data_buf+7));//Moving_Median(17,3,((int16_t)(*(data_buf+7))));
	circle.control[0]=(int8_t)(*(data_buf+8));
	circle.control[1]=(int8_t)(*(data_buf+9));
	circle.r=(int8_t)(*(data_buf+10));
	track.check=(int8_t)(*(data_buf+11));
	}	
	else if(*(data_buf+2)==0x01)//SLAM_frame
  {
  imu_nav.gps.Y_UKF=((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));
	imu_nav.gps.X_UKF=((int16_t)(*(data_buf+6)<<8)|*(data_buf+7));
	imu_nav.gps.Y_O=  ((int16_t)(*(data_buf+8)<<8)|*(data_buf+9));
	imu_nav.gps.X_O=  ((int16_t)(*(data_buf+10)<<8)|*(data_buf+11));
	imu_nav.gps.J=  ((int16_t)(*(data_buf+12)<<8)|*(data_buf+13));
	imu_nav.gps.W=  ((int16_t)(*(data_buf+14)<<8)|*(data_buf+15));	
		
	}		
  else if(*(data_buf+2)==0x03)//Num
  {
	circle.connect=1;
	circle.lose_cnt=0;
	track.check=circle.check=(*(data_buf+4));///10.;
	rc_value_temp=((int16_t)(*(data_buf+5)<<8)|*(data_buf+6));
	if(rc_value_temp<320)
	circle.x=rc_value_temp;//Moving_Median(16,3,rc_value_temp);
	circle.y=(int16_t)(*(data_buf+7));//Moving_Median(17,3,((int16_t)(*(data_buf+7))));
	circle.control[0]=(int8_t)(*(data_buf+8));
	circle.control[1]=(int8_t)(*(data_buf+9));
	circle.r=(int8_t)(*(data_buf+10));
	//track.check=(int8_t)(*(data_buf+11));
	}			
	else if(*(data_buf+2)==0x04)//Mouse
  {
	mouse.connect=1;
	mouse.lose_cnt=0;
	mouse.check=(*(data_buf+4));///10.;
	rc_value_temp1=((int16_t)(*(data_buf+5)<<8)|*(data_buf+6));
	if(rc_value_temp<320)
	mouse.x=rc_value_temp1;//Moving_Median(16,3,rc_value_temp);
	mouse.y=(int16_t)(*(data_buf+7));//Moving_Median(17,3,((int16_t)(*(data_buf+7))));
	mouse.control[0]=(int8_t)(*(data_buf+8));
	mouse.control[1]=(int8_t)(*(data_buf+9));
	mouse.r=(int8_t)(*(data_buf+10));
	//track.check=(int8_t)(*(data_buf+11));
	}		
		else if(*(data_buf+2)==0x05)//TAR
	{			
		if(tar_need_to_check_odroid[0]&&state_v==SU_CHECK_TAR)		
		tar_need_to_check_odroid[1]=*(data_buf+5);
		else
		tar_need_to_check_odroid[1]=66;
		
		tar_need_to_check_odroid[0]=*(data_buf+4);
	
		
  }
		else if(*(data_buf+2)==0x15)//Laser
  {
	
	//mouse.check=(*(data_buf+4));///10.;
	laser_o[0]=LPF_1st(laser_o[0],((int16_t)(*(data_buf+5)<<8)|*(data_buf+6)),k_laser);
	laser_o[1]=LPF_1st(laser_o[1],((int16_t)(*(data_buf+7)<<8)|*(data_buf+8)),k_laser);
	laser_o[2]=LPF_1st(laser_o[2],((int16_t)(*(data_buf+9)<<8)|*(data_buf+10)),k_laser);
	laser_o[3]=LPF_1st(laser_o[3],((int16_t)(*(data_buf+11)<<8)|*(data_buf+12)),k_laser);
	laser_o[4]=LPF_1st(laser_o[4],((int16_t)(*(data_buf+13)<<8)|*(data_buf+14)),k_laser);
  		
		
	if(Pitch<-15 && Pitch >=-25)
		ALT_POS_SONAR_HEAD_LASER_SCANER=laser_o[3];
	else if(Pitch<-25)
		ALT_POS_SONAR_HEAD_LASER_SCANER=laser_o[4];
	else if(Pitch>15 && Pitch <=25)
		ALT_POS_SONAR_HEAD_LASER_SCANER=laser_o[1];
	else if(Pitch>25)
		ALT_POS_SONAR_HEAD_LASER_SCANER=laser_o[0];
	else 
		ALT_POS_SONAR_HEAD_LASER_SCANER=laser_o[2];
	ALT_POS_SONAR_HEAD_LASER_SCANER=(float)ALT_POS_SONAR_HEAD_LASER_SCANER/1000.;
	}		
		else if(*(data_buf+2)==0x21)//Qr land
  {
		
	circle.connect=1;
	circle.lose_cnt=0;
	qr.check=track.check=circle.check=(*(data_buf+4));///10.;
	qr.x=Moving_Median(12,10,((int16_t)(*(data_buf+5)<<8)|*(data_buf+6)));
	qr.y=Moving_Median(13,10,((int16_t)(*(data_buf+7)<<8)|*(data_buf+8)));
	qr.z=Moving_Median(14,10,((int16_t)(*(data_buf+9)<<8)|*(data_buf+10)));
	qr.pit=((int16_t)(*(data_buf+11)<<8)|*(data_buf+12));
	qr.rol=((int16_t)(*(data_buf+13)<<8)|*(data_buf+14));
	qr.yaw=Moving_Median(15,5,((int16_t)(*(data_buf+15)<<8)|*(data_buf+16)));
	
	if(circle.check){
		int temp=((int16_t)(*(data_buf+17)<<8)|*(data_buf+18));
		if(fabs(temp)<1000)
		circle.x=Moving_Median(10,5,temp);
		temp=((int16_t)(*(data_buf+19)<<8)|*(data_buf+20));
		if(fabs(temp)<1000)
		circle.y=Moving_Median(11,5,temp);	
	}
	/*1  0
	
	  2  3*/
	avoid_color[0]=*(data_buf+21);	
	avoid_color[1]=*(data_buf+22);
	avoid_color[2]=*(data_buf+23);
	avoid_color[3]=*(data_buf+24);
	qr.center_x=-((int16_t)(*(data_buf+25)<<8)|*(data_buf+26));	
	qr.center_y=-((int16_t)(*(data_buf+27)<<8)|*(data_buf+28));
	}
}
 



u8 Rx_Buf3[256];	//串口接收缓存
u8 RxBuffer3[50];
u8 RxState3 = 0;
u8 RxBufferNum3 = 0;
u8 RxBufferCnt3 = 0;
u8 RxLen3 = 0;
static u8 _data_len3 = 0,_data_cnt3 = 0;
void USART3_IRQHandler(void)
{  OSIntEnter();  
	u8 com_data;
	
	if(USART3->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART3->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART3,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//清除中断标志

		com_data = USART3->DR;
		
				if(RxState3==0&&com_data==0xAA)
		{
			RxState3=1;
			RxBuffer3[0]=com_data;
		}
		else if(RxState3==1&&com_data==0xAF)
		{
			RxState3=2;
			RxBuffer3[1]=com_data;
		}
		else if(RxState3==2&&com_data>0&&com_data<0XF1)
		{
			RxState3=3;
			RxBuffer3[2]=com_data;
		}
		else if(RxState3==3&&com_data<50)
		{
			RxState3 = 4;
			RxBuffer3[3]=com_data;
			_data_len3 = com_data;
			_data_cnt3 = 0;
		}
		else if(RxState3==4&&_data_len3>0)
		{
			_data_len3--;
			RxBuffer3[4+_data_cnt3++]=com_data;
			if(_data_len3==0)
				RxState3 = 5;
		}
		else if(RxState3==5)
		{
			RxState3 = 0;
			RxBuffer3[4+_data_cnt3]=com_data;
			Data_Receive_Anl3(RxBuffer3,_data_cnt3+5);
		}
		else
			RxState3 = 0;
	
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(USART3,USART_IT_TXE ) )
	{
				
		USART3->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}

		USART_ClearITPendingBit(USART3,USART_IT_TXE);
	}
 OSIntExit();        
}

void UsartSend_GPS(uint8_t ch)
{

while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, ch); ;//USART1, ch); 
}

static void Send_Data_GPS(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GPS(dataToSend[i]);
}


void Send_IMU_TO_GPS(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//功能字
	data_to_send[_cnt++]=0;//数据量
	if(state_v_test!=0)
	_temp = state_v_test;//ultra_distance;
	else
	_temp = state_v;//ultra_distance;	
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = tar_need_to_check_odroid[2];//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Pitch*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(m100.Yaw*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	for(i=0;i<10;i++){
	_temp = tar_buf[i];//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
	}
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	Send_Data_GPS(data_to_send, _cnt);
}

void CPU_LINK_TASK(void)
{
static u8 cnt[4];
static u8 flag;
if(cnt[0]++>0)
{cnt[0]=0;
 Send_IMU_TO_GPS();
}
}



//-------------------------NAV_BOARD_LINK

//------------------------------------------------------GOL_LINK----------------------------------------------------

void UsartSend_GOL_LINK_NAV(uint8_t ch)
{


	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, ch); 
}

static void Send_Data_GOL_LINK_NAV(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GOL_LINK_NAV(dataToSend[i]);
}

void Send_IMU_NAV(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//功能字
	data_to_send[_cnt++]=0;//数据量
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = mpu6050.Acc.x;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = mpu6050.Acc.y;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = mpu6050.Acc.z;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);


	
	_temp = q_nav[0]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = q_nav[1]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[2]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[3]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK_NAV(data_to_send, _cnt);
}


u8 SendBuff1[SEND_BUF_SIZE1];	//发送数据缓冲区
void data_per_uart1(int16_t ax,int16_t ay, int16_t az, int16_t gx,int16_t  gy, int16_t gz,int16_t hx, int16_t hy, int16_t hz,
	int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
u16 i=0; 	
unsigned int temp=0xaF+9;

char ctemp;	
	
SendBuff1[i++]=0xa5;
SendBuff1[i++]=0x5a;
SendBuff1[i++]=14+8;
SendBuff1[i++]=0xA2;

if(ax<0)ax=32768-ax;
ctemp=ax>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=ax;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(ay<0)ay=32768-ay;
ctemp=ay>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=ay;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(az<0)az=32768-az;
ctemp=az>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=az;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(gx<0)gx=32768-gx;
ctemp=gx>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=gx;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(gy<0)gy=32768-gy;
ctemp=gy>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=gy;
SendBuff1[i++]=ctemp;
temp+=ctemp;
//-------------------------
if(gz<0)gz=32768-gz;
ctemp=gz>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=gz;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(hx<0)hx=32768-hx;
ctemp=hx>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=hx;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(hy<0)hy=32768-hy;
ctemp=hy>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=hy;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(hz<0)hz=32768-hz;
ctemp=hz>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=hz;
SendBuff1[i++]=ctemp;
temp+=ctemp;

SendBuff1[i++]=temp%256;
SendBuff1[i++]=(0xaa);
//
 temp=0xaF+2+2;
SendBuff1[i++]=(0xa5);
SendBuff1[i++]=(0x5a);
SendBuff1[i++]=(14+4);
SendBuff1[i++]=(0xA1);


if(yaw<0)yaw=32768-yaw;
ctemp=yaw>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=yaw;							
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(pitch<0)pitch=32768-pitch;
ctemp=pitch>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=pitch;
SendBuff1[i++]=ctemp;
temp+=ctemp;
							 
if(roll<0)roll=32768-roll;
ctemp=roll>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=roll;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(alt<0)alt=32768-alt;
ctemp=alt>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;				
ctemp=alt;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(tempr<0)tempr=32768-tempr;
ctemp=tempr>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=tempr;
SendBuff1[i++]=ctemp;	   
temp+=ctemp;

if(press<0)press=32768-press;
ctemp=press>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=press;
SendBuff1[i++]=ctemp;
temp+=ctemp;

ctemp=IMUpersec>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=IMUpersec;
SendBuff1[i++]=ctemp;
temp+=ctemp;

SendBuff1[i++]=(temp%256);
SendBuff1[i++]=(0xaa);
}


u8 SendBuff2[SEND_BUF_SIZE2];	//发送数据缓冲区
void data_per_uart2(void)
{
	u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
  SendBuff2[_cnt++]=0xAA;
	SendBuff2[_cnt++]=0xAF;
	SendBuff2[_cnt++]=0x01;//功能字
	SendBuff2[_cnt++]=0;//数据量
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	
	_temp = mpu6050.Acc.x;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
 	_temp = mpu6050.Acc.y;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp = mpu6050.Acc.z;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);

	_temp = q_nav[0]*1000;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
 	_temp = q_nav[1]*1000;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[2]*1000;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[3]*1000;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	
	
	
	_temp =  0;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  0;//ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  ultra_distance;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	
	
	
	SendBuff2[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[_cnt++] = sum;
}


u8 SendBuff3[SEND_BUF_SIZE3];	//发送数据缓冲区
void data_per_uart3(void)
{
	u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
  SendBuff3[_cnt++]=0xAA;
	SendBuff3[_cnt++]=0xAF;
	SendBuff3[_cnt++]=0x01;//功能字
	SendBuff3[_cnt++]=0;//数据量
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	
	_temp = mpu6050.Acc.x;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
 	_temp = mpu6050.Acc.y;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp = mpu6050.Acc.z;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);

	_temp = q_nav[0]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
 	_temp = q_nav[1]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[2]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[3]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	
	SendBuff3[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += SendBuff3[i];
	SendBuff3[_cnt++] = sum;
}


u8 SendBuff4[SEND_BUF_SIZE4];	//发送数据缓冲区
void data_per_uart4(u8 sel)
{
	u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;


switch(sel){
	case SEND_ALT:
	SendBuff4[_cnt++]=0xAA;
	SendBuff4[_cnt++]=0xAF;
	SendBuff4[_cnt++]=0x03;//功能字
	SendBuff4[_cnt++]=0;//数据量
	_temp = exp_height;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = ultra_dis_lpf;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = ultra_ctrl_out;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = wz_speed;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
 	_temp = thr_test	;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = ALT_POS_BMP*1000	;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = ALT_POS_SONAR2*1000	;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);

	SendBuff4[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += SendBuff4[i];
	SendBuff4[_cnt++] = sum;
	break;
	case SEND_IMU:
	SendBuff4[_cnt++]=0xAA;
	SendBuff4[_cnt++]=0xAF;
	SendBuff4[_cnt++]=0x05;//功能字
	SendBuff4[_cnt++]=0;//数据量
	_temp = Roll*10;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = Pitch*10;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = Yaw*10	;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.x;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.y;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.z;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.x;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.y;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.z;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=ref_q[0]*1000;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=ref_q[1]*1000;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=ref_q[2]*1000;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=ref_q[3]*1000;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	
	SendBuff4[3] = _cnt-4;
	for( i=0;i<_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[_cnt++] = sum;
	break;
	default:break;
}
}

//flow
u8 mav_flow_sel;
FLOW flow;
FLOW_RAD flow_rad;

u8 FLOW_STATE[4];
u8 flow_buf[27];
u8 flow_buf_rad[45];
   float ByteToFloat(unsigned char* byteArry)
{
  return *((float*)byteArry);
}

void FLOW_MAVLINK(unsigned char data)
{
/*
红色的是起始标志位（stx），在v1.0版本中以“FE”作为起始标志。这个标志位在mavlink消息帧接收端进行消息解码时有用处。

第二个格子代表的是灰色部分（payload，称作有效载荷，要用的数据在有效载荷里面）的字节长度（len），范围从0到255之间。在mavlink消息帧接收端可以用它和实际收到的有效载荷的长度比较，以验证有效载荷的长度是否正确。

第三个格子代表的是本次消息帧的序号（seq），每次发完一个消息，这个字节的内容会加1，加到255后会从0重新开始。这个序号用于mavlink消息帧接收端计算消息丢失比例用的，相当于是信号强度。

第四个格子代表了发送本条消息帧的设备的系统编号（sys），使用PIXHAWK刷PX4固件时默认的系统编号为1，用于mavlink消息帧接收端识别是哪个设备发来的消息。

第五个格子代表了发送本条消息帧的设备的单元编号（comp），使用PIXHAWK刷PX4固件时默认的单元编号为50，用于mavlink消息帧接收端识别是设备的哪个单元发来的消息（暂时没什么用） 。

第六个格子代表了有效载荷中消息包的编号（msg），注意它和序号是不同的，这个字节很重要，mavlink消息帧接收端要根据这个编号来确定有效载荷里到底放了什么消息包并根据编号选择对应的方式来处理有效载荷里的信息包。
      26*/		 
// FE 1A| A2 X X| 64
	
static u8 s_flow=0,data_cnt=0;
float sonar_new;
static float  temp,sonar_lp;
u8 cnt_offset=0;	
u8 get_one_fame=0;
char floattobyte[4];
		switch(s_flow)
	 {
    case 0: if(data==0xFE)
			s_flow=1;
			break;
		case 1: if(data==0x1A||data==0x2C)
				{ s_flow=2;}
			else
			s_flow=0;
			break;
	  case 2:
			if(data_cnt<4)
			{s_flow=2; FLOW_STATE[data_cnt++]=data;}
		  else
			{data_cnt=0;s_flow=3;flow_buf[data_cnt++]=data;}
		 break;
		case 3:
		 if(FLOW_STATE[3]==100){
			if(data_cnt<26)
			{s_flow=3; flow_buf[data_cnt++]=data;}
		  else
			{data_cnt=0;s_flow=4;}
		}
		else if(FLOW_STATE[3]==106){
			if(data_cnt<44)
			{s_flow=3; flow_buf_rad[data_cnt++]=data;}
		  else
			{data_cnt=0;s_flow=4;}
		}
		else
			{data_cnt=0;s_flow=0;}
			 break;
		case 4:get_one_fame=1;s_flow=0;data_cnt=0;break;
		default:s_flow=0;data_cnt=0;break;
	 }//--end of s_uart
		

	 if(get_one_fame)
	 { mav_flow_sel=2;
		 if(FLOW_STATE[3]==100){
		flow.time_sec=(flow_buf[7]<<64)|(flow_buf[6]<<56)|(flow_buf[5]<<48)|(flow_buf[4]<<40)
		 |(flow_buf[3]<<32)|(flow_buf[2]<<16)|(flow_buf[1]<<8)|(flow_buf[0]);
  	 floattobyte[0]=flow_buf[8];
		 floattobyte[1]=flow_buf[9];
		 floattobyte[2]=flow_buf[10];
		 floattobyte[3]=flow_buf[11];
		flow.flow_comp_x.originf =ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf[12];
		 floattobyte[1]=flow_buf[13];
		 floattobyte[2]=flow_buf[14];
		 floattobyte[3]=flow_buf[15];
		flow.flow_comp_y.originf =ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf[16];
		 floattobyte[1]=flow_buf[17];
		 floattobyte[2]=flow_buf[18];
		 floattobyte[3]=flow_buf[19];
//	   if(!GOL_LINK_CONNECT||height_ctrl_mode!=2||x_pred==0)
//			 sonar_new=1;
//		 else
//			 sonar_new=x_pred;//ByteToFloat(floattobyte);//ground_distance	float	Ground distance in m. Positive value: distance known. Negative value: Unknown distance     
	  
		 flow.hight.originf= 0.05f * sonar_new + 0.95f * sonar_lp;
		 sonar_lp = sonar_new;
		 flow.flow_x.origin=(int16_t)((flow_buf[20])|(flow_buf[21]<<8));
	   flow.flow_y.origin=(int16_t)((flow_buf[22])|(flow_buf[23]<<8));
		 flow.id=flow_buf[24];
	   flow.quality=flow_buf[25]; //Optical flow quality / confidence. 0: bad, 255: maximum quality
	//	flow_fix.flow_comp_x.average 		 = IIR_I_Filter(flow_fix.flow_comp_x.originf , InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	//	flow.flow_y.average 		 = IIR_I_Filter(flow.flow_y.origin , InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	//	flow.flow_comp_x.average = IIR_I_Filter(flow.flow_comp_x.originf , InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	//	flow.flow_comp_y.average = IIR_I_Filter(flow.flow_comp_y.originf , InPut_IIR[3], OutPut_IIR[3], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
    flow.new_data_flag	=1;//LED1=!LED1;
		 }
	 else if(FLOW_STATE[3]==106)
	 {
	 	flow_rad.time_usec=(flow_buf_rad[7]<<64)|(flow_buf_rad[6]<<56)|(flow_buf_rad[5]<<48)|(flow_buf_rad[4]<<40)
		 |(flow_buf_rad[3]<<32)|(flow_buf_rad[2]<<16)|(flow_buf_rad[1]<<8)|(flow_buf_rad[0]);
  	 flow_rad.integration_time_us=(flow_buf_rad[11]<<32)|(flow_buf_rad[10]<<16)|(flow_buf_rad[9]<<8)|(flow_buf_rad[8]);
		 floattobyte[0]=flow_buf_rad[12];
		 floattobyte[1]=flow_buf_rad[13];
		 floattobyte[2]=flow_buf_rad[14];
		 floattobyte[3]=flow_buf_rad[15];
		 flow_rad.integrated_x=ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf_rad[16];
		 floattobyte[1]=flow_buf_rad[17];
		 floattobyte[2]=flow_buf_rad[18];
		 floattobyte[3]=flow_buf_rad[19];
		 flow_rad.integrated_y=ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf_rad[20];
		 floattobyte[1]=flow_buf_rad[21];
		 floattobyte[2]=flow_buf_rad[22];
		 floattobyte[3]=flow_buf_rad[23];
		 flow_rad.integrated_xgyro=ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf_rad[24];
		 floattobyte[1]=flow_buf_rad[25];
		 floattobyte[2]=flow_buf_rad[26];
		 floattobyte[3]=flow_buf_rad[27];
		 flow_rad.integrated_ygyro=ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf_rad[28];
		 floattobyte[1]=flow_buf_rad[29];
		 floattobyte[2]=flow_buf_rad[30];
		 floattobyte[3]=flow_buf_rad[31];
		 flow_rad.integrated_zgyro=ByteToFloat(floattobyte);
		 flow_rad.time_delta_distance_us=(flow_buf_rad[35]<<32)|(flow_buf_rad[34]<<16)|(flow_buf_rad[33]<<8)|(flow_buf_rad[32]);
		 floattobyte[0]=flow_buf_rad[36];
		 floattobyte[1]=flow_buf_rad[37];
		 floattobyte[2]=flow_buf_rad[38];
		 floattobyte[3]=flow_buf_rad[39];
		 flow_rad.distance=ByteToFloat(floattobyte);
		 flow_rad.temperature=(flow_buf_rad[41]<<8)|(flow_buf_rad[40]);
		 flow_rad.sensor_id=(flow_buf_rad[42]);
		 flow_rad.quality=(flow_buf_rad[43]);
		 
    flow_task_uart();
		 
	 }	
	 }
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

