#ifndef __INV_I2C_H__
#define __INV_I2C_H__
#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//author: Neod Anderjon
//====================================================================================================
//MPU6050重写IIC

//IO定义映射：PB10->GYRO_SCL PB11->GYRO_SDA
#define IO_GYI2C_SCL    			PBout(10) //SCL
#define IO_GYI2C_SDA_W    			PBout(11) //写SDA	 
//#define IO_GYI2C_SDA_R   			PBin(11)  //读SDA 
#define IO_GYI2C_SDA_R				GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)

//DMP库调用
Bool_ClassType i2cWrite (uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
Bool_ClassType i2cRead (uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

//I2C所有操作函数
void invI2C_IO_Init (void);
void GyroI2C_SDAMode_Setting (i2c_SDA_RW_Switcher sta);
Bool_ClassType invI2C_Start (void);
void invI2C_Stop (void);
Bool_ClassType invI2C_WaitAck (void);
void invI2C_ProdAck (void);
void invI2C_ProdNoAck (void);
void invI2C_SendByte (u8 txd);
u8 invI2C_ReadByte (u8 ack);
u8 invI2C_ReadDevByte (u8 dev, u8 addr);
u8 invI2C_ReadDevLenBytes (u8 dev, u8 reg, u8 length, u8 *data);
Bool_ClassType invI2C_WriteDevLenBytes (u8 dev, u8 reg, u8 length, u8* data);
Bool_ClassType invI2C_ReadRegValue (u8 dev, u8 reg, u8 *data);
Bool_ClassType invI2C_WriteRegValue (u8 dev, u8 reg, u8 data);
Bool_ClassType invI2C_WriteRegBits (u8 dev, u8 reg, u8 bitStart, u8 length, u8 data);
Bool_ClassType invI2C_WriteRegBit (u8 dev, u8 reg, u8 bitNum, u8 data);

#endif

//====================================================================================================
//code by </MATRIX>@Neod Anderjon
