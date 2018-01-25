#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//author: Neod Anderjon
//====================================================================================================
//MPU6050解析重写IIC
 
//陀螺仪IIC SDA 模式转换 
void GyroI2C_SDAMode_Setting (i2c_SDA_RW_Switcher sta)
{
	switch (sta)
	{
	case SDA_Ws: {GPIOB -> CRH &= 0XFFFF0FFF; GPIOB -> CRH |= 3 << 12;} break;
	case SDA_Rs: {GPIOB -> CRH &= 0XFFFF0FFF; GPIOB -> CRH |= 8 << 12;} break;
	}
}	
 
//IIC IO初始化
void invI2C_IO_Init (void)
{			
	//PB10 PB11 推挽输出
	ucGPIO_Config_Init (RCC_APB2Periph_GPIOB,			
						GPIO_Mode_Out_PP,			
						GPIO_Speed_50MHz,						
						GPIORemapSettingNULL,		
						GPIO_Pin_10 | GPIO_Pin_11,					
						GPIOB,				
						IHL,				
						EBO_Disable);
}

//起始信号
Bool_ClassType invI2C_Start (void)
{
	GyroI2C_SDAMode_Setting(SDA_Ws);
	IO_GYI2C_SDA_W = 1;
	if (!IO_GYI2C_SDA_R) 
		return False;	
	
	IO_GYI2C_SCL = 1;
	delay_us(1);
 	IO_GYI2C_SDA_W = 0;
	if (IO_GYI2C_SDA_R) 
		return False;
	
	delay_us(1);
	IO_GYI2C_SCL = 0;
	
	return True;
}

//I2C停止信号	  
void invI2C_Stop (void)
{
	GyroI2C_SDAMode_Setting(SDA_Ws);
	IO_GYI2C_SCL = 0;
	IO_GYI2C_SDA_W = 0;
 	delay_us(1);
	IO_GYI2C_SCL = 1; 
	IO_GYI2C_SDA_W = 1;
	delay_us(1);							   	
}

//I2C等待应答信号
Bool_ClassType invI2C_WaitAck (void)
{
	u8 ucErrTime = 0;
	
	GyroI2C_SDAMode_Setting(SDA_Rs);
	IO_GYI2C_SDA_W = 1;
	delay_us(1);	   
	IO_GYI2C_SCL = 1; 
	delay_us(1);	 
	while (IO_GYI2C_SDA_R)
	{
		ucErrTime++;
		//等待超时，读取失败
		if (ucErrTime > 50)
		{
			invI2C_Stop();
			return False;
		}
		delay_us(1);
	}
	IO_GYI2C_SCL = 0;
    
	return True;  
} 

//I2C产生应答信号
void invI2C_ProdAck (void)
{
	IO_GYI2C_SCL = 0;
	GyroI2C_SDAMode_Setting(SDA_Ws);
	IO_GYI2C_SDA_W = 0;
	delay_us(1);
	IO_GYI2C_SCL = 1;
	delay_us(1);
	IO_GYI2C_SCL = 0;
}
	
//I2C产生不应答信号 
void invI2C_ProdNoAck (void)
{
	IO_GYI2C_SCL = 0;
	GyroI2C_SDAMode_Setting(SDA_Ws);
	IO_GYI2C_SDA_W = 1;
	delay_us(1);
	IO_GYI2C_SCL = 1;
	delay_us(1);
	IO_GYI2C_SCL = 0;
}

//I2C发送一个字节		  
void invI2C_SendByte (u8 txd)
{                        
    u8 t;   
	
	GyroI2C_SDAMode_Setting(SDA_Ws);
    IO_GYI2C_SCL = 0;
    for (t = 0; t < 8; t++)
    {      
		IO_GYI2C_SDA_W = (txd & 0x80) >> 7;
        txd <<= 1; 	  
		delay_us(1);   
		IO_GYI2C_SCL = 1;
		delay_us(1); 
		IO_GYI2C_SCL = 0;
		delay_us(1);
    }	 
} 	 

//I2C读取一个字节
u8 invI2C_ReadByte (u8 ack)
{
	u8 i, receive = 0;
	
	GyroI2C_SDAMode_Setting(SDA_Rs);
    for (i = 0; i < 8; i++ )
	{
        IO_GYI2C_SCL = 0;
        delay_us(2);
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
	
    invI2C_SendByte(addr << 1);
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

//DMP库调用I2C读函数
Bool_ClassType i2cRead (uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{	
    if (!invI2C_Start())
        return True;
	
	invI2C_SendByte(addr << 1);
    if (!invI2C_WaitAck()) 
	{
        invI2C_Stop();
        return True;
    }
	invI2C_SendByte(reg);
	invI2C_WaitAck();
    invI2C_Start();
    invI2C_SendByte((addr << 1) + 1);
    invI2C_WaitAck();
    while (len) 
	{
        if (len == 1)
            *buf = invI2C_ReadByte(0);
        else
            *buf = invI2C_ReadByte(1);
        buf++;
        len--;
    }
    invI2C_Stop();
	
    return False;
}

//I2C读设备寄存器地址一个字节
u8 invI2C_ReadDevByte (u8 dev, u8 addr)
{
	u8 res = 0;
	
	invI2C_Start();	
	invI2C_SendByte(dev);	   
	res++;
	invI2C_WaitAck();
	invI2C_SendByte(addr); 
	res++; 
	invI2C_WaitAck();	  
	invI2C_Start();
	invI2C_SendByte(dev + 1); 
	res++;        
	invI2C_WaitAck();
	res = invI2C_ReadByte(0);	   
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
	{ 
		if (count != length - 1)
			data[count] = invI2C_ReadByte(1);  
		else  
			data[count] = invI2C_ReadByte(0);	 
	}
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
    u8 b, mask;
	
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
