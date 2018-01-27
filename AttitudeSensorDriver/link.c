#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//author: Neod Anderjon
//====================================================================================================
//模块MotorMotionControl对框架EmbeddBreakerCore的链接
//该文件写入对框架的函数调用支持

Stew_EXTI_Setting			StewEXTI_Switch;

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
	case urc_stew: 		StewEXTI_Switch	= (Stew_EXTI_Setting)ed_status;				break;	
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
	//显示Pitch角度
	OLED_ShowString(strPos(0u), ROW1, (const u8*)"P:", Font_Size);
	OLED_ShowNum(strPos(2u), ROW1, ea -> pitch, 3u, Font_Size);	
	OLED_ShowString(strPos(5u), ROW1, (const u8*)".", Font_Size);
	OLED_ShowNum(strPos(6u), ROW1, ((u16)(ea -> pitch * 10) % 10), 1u, Font_Size);
	
	//显示Roll角度
	OLED_ShowString(strPos(8u), ROW1, (const u8*)"R:", Font_Size);
	OLED_ShowNum(strPos(10u), ROW1, ea -> roll, 3u, Font_Size);	
	OLED_ShowString(strPos(13u), ROW1, (const u8*)".", Font_Size);
	OLED_ShowNum(strPos(14u), ROW1, ((u16)(ea -> roll * 10) % 10), 1u, Font_Size);
	
	//显示Yaw角度
	OLED_ShowString(strPos(0u), ROW2, (const u8*)"Y:", Font_Size);
	OLED_ShowNum(strPos(2u), ROW2, ea -> yaw, 3u, Font_Size);	
	OLED_ShowString(strPos(5u), ROW2, (const u8*)".", Font_Size);
	OLED_ShowNum(strPos(6u), ROW2, ((u16)(ea -> yaw * 10) % 10), 1u, Font_Size);
	
	//显示MPU芯片温度
	OLED_ShowString(strPos(8u), ROW2, (const u8*)"T:", Font_Size);
	OLED_ShowNum(strPos(10u), ROW2, MPU6050_ReadTemperature(), 3u, Font_Size);	
	OLED_ShowString(strPos(13u), ROW2, (const u8*)".", Font_Size);
	OLED_ShowNum(strPos(14u), ROW2, ((u16)(MPU6050_ReadTemperature() * 10) % 10), 1u, Font_Size);
	
	OLED_Refresh_Gram();
}

/*
//串口控制运动算例，对协议算例接口
Motion_Select SingleStepDebug_linker (void)
{
	//两字节算列类型
	Motion_Select SSD_MotionNumber	= (Motion_Select)(
											USART1_RX_BUF[SSD_MoNum_1st] 		* 10u 
										+ 	USART1_RX_BUF[SSD_MoNum_1st + 1]);
	//一字节行距单位
	LineRadSelect SSD_Lrsflag		= (LineRadSelect)(
											USART1_RX_BUF[SSD_DisUnit_1st]);
	//四字节行距长度
	u16 SSD_GetDistance 			= (u16)(
											USART1_RX_BUF[SSD_GetDis_1st] 		* 1000u 
										+ 	USART1_RX_BUF[SSD_GetDis_1st + 1] 	* 100u 
										+ 	USART1_RX_BUF[SSD_GetDis_1st + 2] 	* 10u
										+ 	USART1_RX_BUF[SSD_GetDis_1st + 3]);
	//四字节速度
	u16 SSD_Speed					= (u16)(USART1_RX_BUF[SSD_SpFq_1st]			* 1000u
										+	USART1_RX_BUF[SSD_SpFq_1st + 1] 	* 100u 
										+ 	USART1_RX_BUF[SSD_SpFq_1st + 2] 	* 10u 
										+ 	USART1_RX_BUF[SSD_SpFq_1st + 3]);
	//一字节模式位
	MotorRunMode SSD_Mrmflag		= (MotorRunMode)(USART1_RX_BUF[SSD_Mode_1st]);
	
	//打印标志，算例编号，圈数，急停不显示
	if (SendDataCondition && SSD_MotionNumber != Stew_All)
	{
		__ShellHeadSymbol__; 
		printf("Please Confirm Motion Parameter: ");
		//两个flag四种情况
		if (SSD_Lrsflag == RadUnit && SSD_Mrmflag == LimitRun)
			printf("Motion Type: %02d | Speed: %dHz | Distance: %ddegree | Mode: LimitRun\r\n", SSD_MotionNumber, SSD_Speed, SSD_GetDistance);
		else if (SSD_Lrsflag == RadUnit && SSD_Mrmflag == UnlimitRun)
			printf("Motion Type: %02d | Speed: %dHz | Distance: %ddegree | Mode: UnlimitRun\r\n", SSD_MotionNumber, SSD_Speed, SSD_GetDistance);
		else if (SSD_Lrsflag == LineUnit && SSD_Mrmflag == LimitRun)
			printf("Motion Type: %02d | Speed: %dHz | Distance: %dmm | Mode: LimitRun\r\n", SSD_MotionNumber, SSD_Speed, SSD_GetDistance);
		else if (SSD_Lrsflag == LineUnit && SSD_Mrmflag == UnlimitRun)
			printf("Motion Type: %02d | Speed: %dHz | Distance: %dmm | Mode: UnlimitRun\r\n", SSD_MotionNumber, SSD_Speed, SSD_GetDistance);
		usart1WaitForDataTransfer();		
	}

	switch (SSD_MotionNumber)				
	{
	//急停
	case Stew_All: 		
		MotorBasicDriver(&st_motorAcfg, StopRun); 
		EMERGENCYSTOP;									//协议通信急停
		EMERGENCYSTOP_16;										
		break;
	//上下行基本算例
	case UpMove: 		
		MotorMotionController(SSD_Speed, SSD_GetDistance, Pos_Rev, SSD_Mrmflag, SSD_Lrsflag, &st_motorAcfg); 
		break;			
	case DownMove: 		
		MotorMotionController(SSD_Speed, SSD_GetDistance, Nav_Rev, SSD_Mrmflag, SSD_Lrsflag, &st_motorAcfg); 
		break;
	//重复性测试
	case Repeat: 		
		RepeatTestMotion(&st_motorAcfg); 						
		break;
	}
	
	if (SSD_MotionNumber != Stew_All)
	{
		__ShellHeadSymbol__; U1SD("Order Has Started to Execute\r\n");
	}
	order_bootflag = pcl_error;							//完成工作，协议关闭
	
	return SSD_MotionNumber;							//返回算例号用于其它功能
}
*/

//====================================================================================================
//code by </MATRIX>@Neod Anderjon
