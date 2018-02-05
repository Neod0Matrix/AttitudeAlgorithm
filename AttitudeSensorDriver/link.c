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

//MPU实时任务
void dmpAttitudeAlgorithm_RT (IMU_MPUINT_Trigger imi_flag)
{
	/* 	This call need more optimize for real-time and jump RT call out.
	 *	For test you can push it here and setting debug mode to check it work elapsed time.
	 *	You may need to notice register flag symbol change process.
	 *  Once call bell will elapsed 50ms, we must judge bell level status.
	**/
	if (pwsf != JBoot && Read_Beep_IO != WARNING 
		/* 	This flag decide whether INT pin trigger function work. 
		 *	If call rt-function in extern interrupt service function,
				then set flag to enable(IMUINT_Enbale), or not set flag
				to disable(IMUINT_Disable).
		**/
		&& ((!imi_flag) || Is_MPUDataTransfer_Finished))	
	{
		/* first @InvenSense DMP call. */
		if (!dmpAttitudeAlgorithm(&eas))					
		{
			/* original chip data read. */
			MPU6050_GetGyroAccelOriginData(&gas);
			/* chip inner temperature read. */
			MPU_GlobalTemp = MPU6050_ReadTemperature();		
		}
	}
}

//注意高级定时器18挂载在APB2总线上，通用定时器2345挂载在APB1总线上
//TIM3定时时基源
#define IMURTFreqDivTimer		TIM3
#define RCC_APB1Periph_TIMERx	RCC_APB1Periph_TIM3			//设置定时器挂载总线
#define Timerx_IRQn				TIM3_IRQn					//定时器中断
//依据MPUDataReadFreq设定
#define imu_TogglePeriod		9999u						//定时器自动重装翻转周期
#define imu_Prescaler			359u						//定时器分频器	

//初始化定时器3用于MPU的实时任务
void TIM3_IMURealTimeWork (FunctionalState control)  
{  
	ucTimerx_InitSetting(	IMURTFreqDivTimer, 
							Timerx_IRQn, 
							RCC_APB1Periph_TIMERx,
							TIMx_GPIO_Remap_NULL,
							imu_TogglePeriod, 
							imu_Prescaler, 
							TIM_CKD_DIV1, 
							TIM_CounterMode_Up, 
							irq_Use, 						
							0x03, 
							0x05, 
							control);
}  

//定时器3中断服务：IMU实时任务
void TIM3_IRQHandler (void)  								
{
#if SYSTEM_SUPPORT_OS										//OS支持
	OSIntEnter();
#endif
	
	if (TIM_GetITStatus(IMURTFreqDivTimer, TIM_IT_Update) != RESET)//检查指定的TIM中断发生与否
	{
		TIM_ClearITPendingBit(IMURTFreqDivTimer, TIM_IT_Update);//清除TIMx的中断待处理位
		
		dmpAttitudeAlgorithm_RT(IMUINT_Disable);
	}
	
#if SYSTEM_SUPPORT_OS										//OS支持
	OSIntExit();    
#endif
}

//====================================================================================================
//code by </MATRIX>@Neod Anderjon
