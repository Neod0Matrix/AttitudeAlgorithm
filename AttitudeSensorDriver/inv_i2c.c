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
//#define IO_GYI2C_SDA_R			GPIO_ReadInputDataBit(Gyro_GPIOx, Gyro_SDA_Pin)
 
//MPU使用delay函数
void mpu_delay_us (void)
{
	delay_us(2);
}

//陀螺仪I2C SDA 模式转换 
void GyroI2C_SDAModeTransfer (i2c_SDA_RW_Switcher sta)
{
	//寄存器写法
	GPIOB -> CRH &= 0XFFFF0FFF;
	GPIOB -> CRH |= (sta == SDA_Ws)? 3 << 12 : 8 << 12;					//仅兼容PB11
	//库函数写法
//	if (sta == SDA_Ws)
//	{
//		ucGPIO_Config_Init (GyroAPBx_RCCBus,			
//							GPIO_Mode_Out_PP,							//数据传输开漏			
//							GPIO_Speed_50MHz,						
//							GPIORemapSettingNULL,			
//							Gyro_SDA_Pin,					
//							Gyro_GPIOx,					
//							NI,				
//							EBO_Disable);
//	}
//	else
//	{
//		ucGPIO_Config_Init (GyroAPBx_RCCBus,			
//							GPIO_Mode_IN_FLOATING,						//数据浮空输入
//							GPIO_Input_Speed,						
//							GPIORemapSettingNULL,			
//							Gyro_SDA_Pin,					
//							Gyro_GPIOx,					
//							NI,				
//							EBO_Disable);
//	}
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
Bool_ClassType invI2C_Start (void)
{
	GyroI2C_SDAModeTransfer(SDA_Ws);
	IO_GYI2C_SDA_W = 1;
	
	if (!IO_GYI2C_SDA_R) 
		return False;	
	
	IO_GYI2C_SCL = 1;
	mpu_delay_us();
 	IO_GYI2C_SDA_W = 0;
	if (IO_GYI2C_SDA_R) 
		return False;
	
	mpu_delay_us();
	IO_GYI2C_SCL = 0;
	
	return True;
}

//I2C停止信号	  
void invI2C_Stop (void)
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
Bool_ClassType invI2C_WaitAck (void)
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
		//等待超时，读取失败
		if (ucErrTime > 250)
		{
			invI2C_Stop();
			return False;
		}
		//delay_us(1);
	}
	IO_GYI2C_SCL = 0;
    
	return True;  
} 

//I2C产生应答信号
void invI2C_ProdAck (void)
{
	IO_GYI2C_SCL = 0;
	GyroI2C_SDAModeTransfer(SDA_Ws);
	IO_GYI2C_SDA_W = 0;
	mpu_delay_us();
	IO_GYI2C_SCL = 1;
	mpu_delay_us();
	IO_GYI2C_SCL = 0;
}
	
//I2C产生不应答信号 
void invI2C_ProdNoAck (void)
{
	IO_GYI2C_SCL = 0;
	GyroI2C_SDAModeTransfer(SDA_Ws);
	IO_GYI2C_SDA_W = 1;
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
		delay_us(1);   
		IO_GYI2C_SCL = 1;
		mpu_delay_us();
		IO_GYI2C_SCL = 0;
		mpu_delay_us();
    }	 
} 	 

//I2C读取一个字节
u8 invI2C_ReadByte (Bool_ClassType ack)
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
		delay_us(2); 
    }			
    if (ack)
        invI2C_ProdAck(); 
    else
        invI2C_ProdNoAck();
	
    return receive;
}
  
//DMP库调用I2C写入函数
Bool_ClassType i2cWrite (uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	int i;
	
    if (!invI2C_Start())
        return True;
	
    //invI2C_SendByte(addr << 1);
	invI2C_SendByte((addr << 1) | 0);	//发送器件地址+写命令
    if (!invI2C_WaitAck()) 
	{
        invI2C_Stop();
        return True;
    }
	
    invI2C_SendByte(reg);
    invI2C_WaitAck();
	for (i = 0; i < len; i++) 
	{
        invI2C_SendByte(data[i]);
        if (!invI2C_WaitAck()) 
		{
            invI2C_Stop();
            return False;
        }
    }
    invI2C_Stop();
	
    return False;
}

//I2C写一个字节
Bool_ClassType invI2C_WriteAByte (u8 dev, u8 reg, u8 data)
{
	if (!invI2C_Start())
        return True;
	invI2C_SendByte((dev << 1) | 0);
	if (!invI2C_WaitAck()) 
	{
        invI2C_Stop();
        return True;
    }
	invI2C_SendByte(reg);
	invI2C_WaitAck();
	invI2C_SendByte(data);
	if (!invI2C_WaitAck()) 
	{
        invI2C_Stop();
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
	
	//invI2C_SendByte(addr << 1);
	invI2C_SendByte((addr << 1) | 0);
    if (!invI2C_WaitAck()) 
	{
        invI2C_Stop();
        return True;
    }
	invI2C_SendByte(reg);
	invI2C_WaitAck();
    invI2C_Start();
    //invI2C_SendByte((addr << 1) + 1);
	invI2C_SendByte((addr << 1) | 1);
    invI2C_WaitAck();
	//read total register
    do	
		*buf = invI2C_ReadByte((len == 1)? False : True), buf++;
	while (len--);
    invI2C_Stop();
	
    return False;
}

//I2C读设备寄存器地址一个字节
u8 invI2C_ReadDevByte (u8 dev, u8 addr)
{
	u8 res = 0;
	
	invI2C_Start();	
	//invI2C_SendByte(dev);	   
	invI2C_SendByte((dev << 1) | 0);	  
	//res++;
	invI2C_WaitAck();
	invI2C_SendByte(addr); 
	//res++; 
	invI2C_WaitAck();	  
	invI2C_Start();
	//invI2C_SendByte(dev + 1); 
	invI2C_SendByte((dev << 1) | 1);	  
	//res++;        
	invI2C_WaitAck();
	res = invI2C_ReadByte(False);	   
    invI2C_Stop();

	return res;
}

//I2C读设备寄存器多个值
u8 invI2C_ReadDevLenBytes (u8 dev, u8 reg, u8 length, u8 *data)
{
    u8 count = 0;
	
	invI2C_Start();
	invI2C_SendByte(dev);	
	invI2C_WaitAck();
	invI2C_SendByte(reg);   
    invI2C_WaitAck();	  
	invI2C_Start();
	invI2C_SendByte(dev + 1); 
	invI2C_WaitAck();
    for (count = 0; count < length; count++)
		data[count] = invI2C_ReadByte((count != length - 1)? True : False);
    invI2C_Stop();
	
    return count;
}

//I2C对设备寄存器写入多个值
Bool_ClassType invI2C_WriteDevLenBytes (u8 dev, u8 reg, u8 length, u8* data)
{  
 	u8 count = 0;
	
	invI2C_Start();
	invI2C_SendByte(dev);
	invI2C_WaitAck();
	invI2C_SendByte(reg);  
    invI2C_WaitAck();	  
	for (count = 0; count < length; count++)
	{
		invI2C_SendByte(data[count]); 
		invI2C_WaitAck();	  		//这里有可能返回false
	}
	invI2C_Stop();

    return True; 
}

//I2C读指定设备寄存器里的指定值
Bool_ClassType invI2C_ReadRegValue (u8 dev, u8 reg, u8 *data)
{
	//有可能返回false
	*data = invI2C_ReadDevByte(dev, reg);
	
    return True;
}

//I2C向指定寄存器设备写值
Bool_ClassType invI2C_WriteRegValue (u8 dev, u8 reg, u8 data)
{
    return invI2C_WriteDevLenBytes(dev, reg, 1, &data);
}

//I2C对指定设备寄存器多位操作
Bool_ClassType invI2C_WriteRegBits (u8 dev, u8 reg, u8 bitStart, u8 length, u8 data)
{
    u8 b, mask = 0;
	
    if (invI2C_ReadRegValue(dev, reg, &b) != 0) 
	{
        mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
		
        return invI2C_WriteRegValue(dev, reg, b);
    } 
	else 
        return False;
}

//I2C对指定寄存器值一位操作
Bool_ClassType invI2C_WriteRegBit (u8 dev, u8 reg, u8 bitNum, u8 data)
{
    u8 b;
	
    invI2C_ReadRegValue(dev, reg, &b);
	//data为0时目标位将被清0否则置位
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	
    return invI2C_WriteRegValue(dev, reg, b);
}

//====================================================================================================
//code by </MATRIX>@Neod Anderjon
