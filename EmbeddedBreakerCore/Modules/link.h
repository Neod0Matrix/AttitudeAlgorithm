#pragma once
#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//author: Neod Anderjon
//====================================================================================================
/*
	模块对框架EmbeddBreakerCore的链接
	该文件写入对框架的函数调用支持
*/

#include "inv_i2c.h"										//对MPU专门优化的I2C
//@InvenSense DMP Library Call
#include "dmpkey.h"				
#include "dmpmap.h"
#include "inv_mpu_dmp_motion_driver.h"				
#include "inv_mpu.h"
#include "mpu6050.h"										//MPU6050底层驱动
#include "filter.h"											//滤波器

//模块声明
#define _Modules_Type_			"SDP"						//模块类型
#define _Modules_Name_			"AtitudeAlgorithm"			//模块名称
#define _Modules_Version_ 		"v0p4_LTE"					//长期演进版

//模块使用的协议链接，尽量整合到一条
#define MDLS					0x1A
#define Modules_Protocol 		{DH, MDLS, DMAX, DMAX, DMAX, DMAX, DMAX, DMAX, DMAX, DMAX, DMAX, DMAX, DMAX, DMAX, DMAX, DMAX, DMAX, DT}

//urc开源链接编号
typedef enum
{
	empty = 0,
} Modules_SwitchNbr;

//裁去config.h中的定义放到这里来重新定义urc协议长度
#define Module_Add_urcOption_Count	0u
#define Max_Option_Value		(Module_Add_urcOption_Count + FrameDefault_urcOption_Count)			
//裁去ui.h中定义的总切屏数到这里来重新定义
#define Module_Add_oledScreen_Count	1u
#define ScreenPageCount			(Module_Add_oledScreen_Count + FrameDefault_oledScreen_Count)			

//对外API接口
void Modules_UniResConfig (void);							//选项设置，链接到Universal_Resource_Config函数的模块库
void Modules_URCMap (void);									//模块选项映射表，链接到urcMapTable_Print函数
void Modules_urcDebugHandler (u8 ed_status, Modules_SwitchNbr sw_type);//选项处理，链接到pclURC_DebugHandler函数
void Modules_ProtocolTask (void);							//协议调用指令响应，链接到OrderResponse_Handler函数
void OLED_ScreenModules_Const (void);						//OLED常量显示屏，链接到OLED_DisplayInitConst和UIScreen_DisplayHandler函数
void OLED_DisplayModules (void);							//OLED模块调用数据显示，链接到UIScreen_DisplayHandler函数
void Modules_HardwareInit (void);							//硬件底层初始化任务，链接到bspPeriSysCalls函数
void Modules_ExternInterruptInit (void);					//硬件底层外部中断初始化，链接到EXTI_Config_Init函数
void Modules_NonInterruptTask (void);						//模块非中断任务，链接到local_taskmgr.c，默认添加到第二任务
void Modules_InterruptTask (void);							//模块中断任务，链接到time_base.c TIM2_IRQHandler函数中

//====================================================================================================
//code by </MATRIX>@Neod Anderjon