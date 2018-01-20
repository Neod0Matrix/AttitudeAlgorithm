#ifndef __EXTI_H__
#define __EXTI_H__
#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//author: Neod Anderjon
//====================================================================================================
//外部中断驱动代码

//总线设置
#define ARM2Dn_EXTI_Line  	EXTI_Line3								//滑轨下方传感器					
#define ARM2Up_EXTI_Line	EXTI_Line4								//滑轨上方传感器						
#define Stew_EXTI_Line 		EXTI_Line8								//急停	

//EXTI模式配置
void ucEXTI_ModeConfig (
						uint8_t 			io_group, 				//IO分组
						uint8_t 			io_src, 				//IO源
						uint32_t 			line, 					//总线
						EXTIMode_TypeDef 	it_mode, 				//中断模式
						EXTITrigger_TypeDef trigger,				//触发方式
						uint8_t 			irq,					//中断组
						uint8_t 			pp,						//抢占优先级
						uint8_t 			sp						//子优先级
					);						

void EXTI_Config_Init (void);										//EXTI初始化配置

#endif

//====================================================================================================
//code by </MATRIX>@Neod Anderjon

