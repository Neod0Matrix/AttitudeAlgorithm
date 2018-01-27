#ifndef __INV_I2C_H__
#define __INV_I2C_H__
#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//author: Neod Anderjon
//====================================================================================================
//MPU6050重写IIC

//DMP库调用
Bool_ClassType i2cWrite (uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
Bool_ClassType i2cRead (uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

//I2C所有操作函数
void invI2C_IO_Init (void);
void GyroI2C_SDAModeTransfer (i2c_SDA_RW_Switcher sta);
Bool_ClassType invI2C_Start (void);
void invI2C_Stop (void);
Bool_ClassType invI2C_WaitAck (void);
void invI2C_ProdAck (void);
void invI2C_ProdNoAck (void);
void invI2C_SendByte (u8 txd);
u8 invI2C_ReadByte (Bool_ClassType ack);
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
