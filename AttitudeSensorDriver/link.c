#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//author: Neod Anderjon
//====================================================================================================
//模块MotorMotionControl对框架EmbeddBreakerCore的链接
//该文件写入对框架的函数调用支持

Stew_EXTI_Setting			StewEXTI_Switch;

#define MPURunInterval		5000

//链接到Universal_Resource_Config函数的模块库
void ModuleAA_UniResConfig (void)
{
	/*
		急停状态判断复杂，不适合外部中断
		但也有可能普通监测不够快
	*/
	StewEXTI_Switch 	= StewEXTI_Enable;				//StewEXTI_Enable	StewEXTI_Disable
}

//模块选项映射表，链接到urcMapTable_Print函数
void ModuleAA_URCMap (void)
{
	printf("\r\n%02d 	Stew EXTI Setting", urc_stew);
	usart1WaitForDataTransfer();
}

//选项处理，链接到pclURC_DebugHandler函数
void ModuleAA_urcDebugHandler (u8 ed_status, AHRS_SwitchNbr sw_type)
{
	switch (sw_type)
	{
	case urc_stew: 		StewEXTI_Switch	= (Stew_EXTI_Setting)ed_status;		break;	
	}
}

//串口接收数据示例，不调用
void U1RSD_example (void)
{
    u8 t, len;
	
    if (PD_Switch == PD_Enable && Data_Receive_Over)	//接收数据标志
    {
        len = Data_All_Length;							//得到此次接收到的数据长度(字符串个数)
        __ShellHeadSymbol__; U1SD("Controller Get The Data: \r\n");
		if (No_Data_Receive)							//没有数据接收，可以发送
		{
			for (t = 0u; t < len; t++)
			{
				USART_SendData(USART1, USART1_RX_BUF[t]);//将所有数据依次发出	
				usart1WaitForDataTransfer();			//等待发送结束
			}
		}
        U1SD("\r\n");									//插入换行
        USART1_RX_STA = 0u;								//接收状态标记
    }
}

//OLED常量第四屏，链接到OLED_DisplayInitConst和UIScreen_DisplayHandler函数
void OLED_ScreenP4_Const (void)
{	
	OLED_ShowString(strPos(1u), ROW1, (const u8*)"   Attitude   ", Font_Size);	
	OLED_ShowString(strPos(1u), ROW2, (const u8*)"   Algorithm  ", Font_Size);	
	OLED_Refresh_Gram();
}

//OLED AttitudeAlgorithm数据显示
void OLED_DisplayAA (EulerAngleStructure *ea)
{	
	//静态更新，节省进程占用
	static float pitchDisp = 0.f, rollDisp = 0.f, yawDisp = 0.f, tempDisp = 0.f;
	
	//显示俯仰Pitch角度(x轴)
	if (pitchDisp != ea -> pitch)
	{
		pitchDisp = ea -> pitch;
		OLED_ShowString(strPos(0u), ROW1, (const u8*)"P:", Font_Size);
		OLED_ShowNum(strPos(2u), ROW1, pitchDisp, 3u, Font_Size);	
		OLED_ShowString(strPos(5u), ROW1, (const u8*)".", Font_Size);
		OLED_ShowNum(strPos(6u), ROW1, ((u16)(pitchDisp * 10) % 10), 1u, Font_Size);
	}
	//显示翻滚Roll角度(y轴)
	if (rollDisp != ea -> roll)
	{
		rollDisp = ea -> roll;
		OLED_ShowString(strPos(8u), ROW1, (const u8*)"R:", Font_Size);
		OLED_ShowNum(strPos(10u), ROW1, rollDisp, 3u, Font_Size);	
		OLED_ShowString(strPos(13u), ROW1, (const u8*)".", Font_Size);
		OLED_ShowNum(strPos(14u), ROW1, ((u16)(rollDisp * 10) % 10), 1u, Font_Size);
	}
	//显示航向Yaw角度(z轴)
	if (yawDisp != ea -> yaw)
	{
		yawDisp = ea -> yaw;
		OLED_ShowString(strPos(0u), ROW2, (const u8*)"Y:", Font_Size);
		OLED_ShowNum(strPos(2u), ROW2, yawDisp, 3u, Font_Size);	
		OLED_ShowString(strPos(5u), ROW2, (const u8*)".", Font_Size);
		OLED_ShowNum(strPos(6u), ROW2, ((u16)(yawDisp * 10) % 10), 1u, Font_Size);
	}
	//显示MPU芯片温度
	if (tempDisp != MPU_GlobalTemp)
	{
		tempDisp = MPU_GlobalTemp;
		OLED_ShowString(strPos(8u), ROW2, (const u8*)"T:", Font_Size);
		OLED_ShowNum(strPos(10u), ROW2, tempDisp, 3u, Font_Size);	
		OLED_ShowString(strPos(13u), ROW2, (const u8*)".", Font_Size);
		OLED_ShowNum(strPos(14u), ROW2, ((u16)(tempDisp * 10) % 10), 1u, Font_Size);
	}
	OLED_Refresh_Gram();
}

//MPU实时任务，link到time_base.c TIM2_IRQHandler中断函数
void dmpAttitudeAlgorithm_RT (void)
{
	/* ARM platform can set static var here, 8051 don't do it. */
	static u16 runMPUUpdateSem = 0u;			
					
	if ((runMPUUpdateSem++ == TickDivsIntervalus(MPURunInterval) - 1) 
		&& Return_Error_Type == Error_Clear && pwsf != JBoot)
	{
		runMPUUpdateSem = 0u;
		/* 	This call need more optimize for real-time and jump RT call out.
		 *	For test you can push it here and setting debug mode to check it work elapsed time.
		 *	You may need to notice register flag symbol change process.
		**/
		if (!dmpAttitudeAlgorithm(&eas))									
		{
			MPU6050_GetGyroAccelOriginData(&gas);					
			MPU_GlobalTemp = MPU6050_ReadTemperature();				
		}									
	}
}

//====================================================================================================
//code by </MATRIX>@Neod Anderjon
