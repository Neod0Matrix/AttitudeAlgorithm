#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//author: Neod Anderjon
//====================================================================================================
//MPU6050解析重写IIC

//IO总线定义
#define GyroAPBx_RCCBus			RCC_APB2Periph_GPIOB
#define Gyro_GPIOx				GPIOB
#define Gyro_SCL_Pin			GPIO_Pin_10								//SCL PB10
#define Gyro_SDA_Pin			GPIO_Pin_11								//SDA PB11
#define IO_GYI2C_SCL    		PBout(10) 								//SCL
#define IO_GYI2C_SDA_W    		PBout(11) 								//写SDA	 
#define IO_GYI2C_SDA_R   		PBin(11)  								//读SDA 
 
//MPU使用delay函数
static void mpu_delay_us (void)
{
	delay_us(2);
}

//陀螺仪I2C SDA 模式转换 
static void GyroI2C_SDAModeTransfer (i2c_SDA_RW_Switcher sta)
{
	//寄存器写法
	Gyro_GPIOx -> CRH &= 0XFFFF0FFF;
	Gyro_GPIOx -> CRH |= ((sta == SDA_Ws)? 3 : 8) << 12;//仅兼容11脚
}	
 
//IIC IO初始化
void invI2C_IO_Init (void)
{			
	ucGPIO_Config_Init (GyroAPBx_RCCBus,			
						GPIO_Mode_Out_PP,			
						GPIO_Speed_50MHz,						
						GPIORemapSettingNULL,		
						Gyro_SCL_Pin | Gyro_SDA_Pin,					
						Gyro_GPIOx,				
						IHL,							//初始拉高	
						EBO_Disable);
}

//起始信号
static Bool_ClassType invI2C_Start (void)
{
	GyroI2C_SDAModeTransfer(SDA_Ws);
	IO_GYI2C_SDA_W = 1;	
	//here can select no judge sda level
	if (!IO_GYI2C_SDA_R)
		return False;
	IO_GYI2C_SCL = 1;
	mpu_delay_us();
 	IO_GYI2C_SDA_W = 0;
	//here can select no judge sda level
	if (IO_GYI2C_SDA_R)
		return False;
	mpu_delay_us();
	IO_GYI2C_SCL = 0;
	
	return True;
}

//I2C停止信号	  
static void invI2C_Stop (void)
{
	GyroI2C_SDAModeTransfer(SDA_Ws);
	IO_GYI2C_SCL = 0;
	IO_GYI2C_SDA_W = 0;
	mpu_delay_us();
	IO_GYI2C_SCL = 1; 
	IO_GYI2C_SDA_W = 1;
	mpu_delay_us();					   	
}

//I2C等待应答信号
static Bool_ClassType invI2C_WaitAck (void)
{
	u8 ucErrTime = 0;
	
	GyroI2C_SDAModeTransfer(SDA_Rs);
	IO_GYI2C_SDA_W = 1;
	mpu_delay_us();   
	IO_GYI2C_SCL = 1; 
	mpu_delay_us();
	while (IO_GYI2C_SDA_R)
	{
		ucErrTime++;
		//wait timeout
		if (ucErrTime > 250)
		{
			invI2C_Stop();
			return True;
		}
	}
	IO_GYI2C_SCL = 0;
    
	return False;  
} 

//I2C产生应答/不应答信号
static void invI2C_NoAckorAck (i2cNoAckorAck signal)
{
	IO_GYI2C_SCL = 0;
	GyroI2C_SDAModeTransfer(SDA_Ws);
	/*	variable transfer to i/o signal.
	 *	signal=1 ack; signal=0 noack.
	 *	actual level opposite to signal value.
	**/
	IO_GYI2C_SDA_W = !signal;		
	mpu_delay_us();
	IO_GYI2C_SCL = 1;
	mpu_delay_us();
	IO_GYI2C_SCL = 0;
}

//I2C发送一个字节		  
void invI2C_SendByte (u8 txd)
{                        
    u8 t;   
	
	GyroI2C_SDAModeTransfer(SDA_Ws);
    IO_GYI2C_SCL = 0;
    for (t = 0; t < 8; t++)
    {      
		IO_GYI2C_SDA_W = (txd & 0x80) >> 7;
        txd <<= 1; 	  
		IO_GYI2C_SCL = 1;
		mpu_delay_us();
		IO_GYI2C_SCL = 0;
		mpu_delay_us();
    }	 
} 	 

//I2C读取一个字节
u8 invI2C_ReadByte (i2cNoAckorAck signal)
{
	u8 i, receive = 0;
	
	GyroI2C_SDAModeTransfer(SDA_Rs);
    for (i = 0; i < 8; i++ )
	{
        IO_GYI2C_SCL = 0;
        mpu_delay_us();
		IO_GYI2C_SCL = 1;
        receive <<= 1;
        if (IO_GYI2C_SDA_R)
			receive++;   
		mpu_delay_us();
    }			
	invI2C_NoAckorAck(signal);
	
    return receive;
}
  
//I2C写一个字节
Bool_ClassType invI2C_WriteDevByte (u8 dev, u8 reg, u8 data)
{
	if (!invI2C_Start())
		return True;
	invI2C_SendByte((dev << 1) | 0);
	if (invI2C_WaitAck()) 
        return True;
	invI2C_SendByte(reg);
	if (invI2C_WaitAck()) 
        return True;
	invI2C_SendByte(data);
	if (invI2C_WaitAck()) 
        return True;
	invI2C_Stop();
	
    return False;
}

//I2C读设备寄存器地址一个字节
u8 invI2C_ReadDevByte (u8 dev, u8 reg)
{
	u8 res;
	
	if (!invI2C_Start())
		return True;
	invI2C_SendByte((dev << 1) | 0);	  
	if (invI2C_WaitAck()) 
        return True;
	invI2C_SendByte(reg); 
	if (invI2C_WaitAck()) 
        return True;	  
	if (!invI2C_Start())
		return True;
	invI2C_SendByte((dev << 1) | 1);	     
	if (invI2C_WaitAck()) 
        return True;
	res = invI2C_ReadByte(i2cNoAck);
    invI2C_Stop();

	return res;
}

//DMP库调用I2C写入函数
Bool_ClassType i2cWrite (uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	int i;
	
    if (!invI2C_Start())
		return True;
	invI2C_SendByte((addr << 1) | 0);	//发送器件地址+写命令
    if (invI2C_WaitAck()) 
        return True;
    invI2C_SendByte(reg);
    if (invI2C_WaitAck()) 
        return True;
	for (i = 0; i < len; i++) 
	{
        invI2C_SendByte(data[i]);
        if (invI2C_WaitAck()) 
        return True;
    }
    invI2C_Stop();
	
    return False;
}

//DMP库调用I2C读函数
Bool_ClassType i2cRead (uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{	
	if (!invI2C_Start())
		return True;
	invI2C_SendByte((addr << 1) | 0);
    if (invI2C_WaitAck()) 
        return True;
	invI2C_SendByte(reg);
	if (invI2C_WaitAck()) 
        return True;
    if (!invI2C_Start())
		return True;
	invI2C_SendByte((addr << 1) | 1);
    if (invI2C_WaitAck()) 
        return True;
	//read total register
	while (len)
		*buf = invI2C_ReadByte((len == 1)? i2cNoAck : i2cAck), len--, buf++;
	
    invI2C_Stop();
	
    return False;
}

//====================================================================================================
//code by </MATRIX>@Neod Anderjon
