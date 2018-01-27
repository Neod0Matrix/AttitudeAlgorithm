#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//author: Neod Anderjon
//====================================================================================================
//MPU6050基础驱动

EulerAngleStructure eas;

//欧拉角结构体初始化
void EulerAngleStructureInit (EulerAngleStructure *ea)
{
	ea -> pitch = 0.f;
	ea -> roll = 0.f;
	ea -> yaw = 0.f;
}

//MPU6050数据读取寄存更新
void MPU6050_DataRegUpdate (int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
	u8 i, j;
	int16_t data_fifo[6][11];
	int32_t sum = 0;
	
	//FIFO操作
	for (i = 1; i < 10; i++)
		for (j = 0; j < 6; j++)
			data_fifo[j][i - 1] = data_fifo[j][i];
		
	//将新的数据放置到 数据的最后面
	data_fifo[0][9] = ax;
	data_fifo[1][9] = ay;
	data_fifo[2][9] = az;
	data_fifo[3][9] = gx;
	data_fifo[4][9] = gy;
	data_fifo[5][9] = gz;
	
	//求和平均滤波
	for (j = 0; j < 6; j++)
	{
		sum = 0;
		for (i = 0; i < 10; i++)
			sum += data_fifo[j][i];
		data_fifo[j][10] = sum / 10;
	}
}

//MPU自检测试
Bool_ClassType run_self_test (void)
{
    int result = 0;
    long gyro[3], accel[3];
	float sens = 0.f;
	u16 accel_sens = 0;
	
    result = mpu_run_self_test(gyro, accel);
    if (result == 0x0) 							//MPU6500: 0x7
	{
		//Test passed. We can trust the gyro data here, so let's push it down to the DMP.
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
		
		return False;
    }
	else
		return True;
}

//方向矩阵辅助转换函数
static u16 inv_row_2_scale (const signed char *row)
{
    u16 b;

    if (row[0] > 0)
		b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      //error
	
    return b;
}

/**
  * @brief     Fast inverse square-root, to calculate 1/Sqrt(x)
  * @param[in] input:x
  * @retval    1/Sqrt(x)
  */
float invSqrt (float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*) & y;
	
	i = 0x5f3759df - (i >> 1);
	y = *(float*) & i;
	y *= (1.5f - (halfx * pow(y, 2)));
	
	return y;
}

//方向矩阵转换标量
static u16 inv_orientation_matrix_to_scalar (const signed char *mtx)
{
    u16 scalar;
	
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

//MPU6050内置DMP初始化
void MPU6050_SetInnerDMPInit (void)
{ 
	u8 readID[1] = {0};
	//algorithm matrix
	signed char gyro_orientation[9] = {-1, 	0, 	0,
										0, 	-1, 0,
										0, 	0, 	1};

	__ShellHeadSymbol__; U1SD("InvSense DMP Library Function Init ->\r\n");
		
	i2cRead(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, 1, readID);	//设备检查，也可以调用库函数
	if (readID[0] != MPU6050_DEFAULT_ADDRESS)
		U1SD("Mpu Device Register Fatal, Suggest @Reboot\r\n");			//检查设备不成功，建议重启
	
	//@InvSense DMP library call
	if (!mpu_init())
	{
		U1SD("\r\n[0] mpu_device_register complete\r\n");
		if (!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
			U1SD("\r\n[1] mpu_set_sensor complete\r\n");
		if (!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
			U1SD("\r\n[2] mpu_configure_fifo complete\r\n");
		if (!mpu_set_sample_rate(DEFAULT_MPU_HZ))
			U1SD("\r\n[3] mpu_set_sample_rate complete\r\n");
		if (!dmp_load_motion_driver_firmware())
			U1SD("\r\n[4] dmp_load_motion_driver_firmware complete\r\n");
		if (!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
			U1SD("\r\n[5] dmp_set_orientation complete\r\n");
		if (!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
	        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | 
			DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL))
			U1SD("\r\n[6] dmp_enable_feature complete\r\n");
		if (!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
			U1SD("\r\n[7] dmp_set_fifo_rate complete\r\n");
		if (!run_self_test())
			U1SD("\r\n[8] dmp_set_bias complete\r\n");
		if (!mpu_set_dmp_state(ENABLE))
			U1SD("\r\n[9] mpu_set_dmp_state complete\r\n");
	}
}

/*
	设置MPU6050的时钟源
	* CLK_SEL | Clock Source
	* --------+--------------------------------------
	* 0       | Internal oscillator
	* 1       | PLL with X Gyro reference
	* 2       | PLL with Y Gyro reference
	* 3       | PLL with Z Gyro reference
	* 4       | PLL with external 32.768kHz reference
	* 5       | PLL with external 19.2MHz reference
	* 6       | Reserved
	* 7       | Stops the clock and keeps the timing generator in reset
*/
void MPU6050_SetClockSource (uint8_t source)
{
    invI2C_WriteRegBits(devAddr, 
						MPU6050_RA_PWR_MGMT_1, 
						MPU6050_PWR1_CLKSEL_BIT, 
						MPU6050_PWR1_CLKSEL_LENGTH, 
						source);
}

/*
	* Set full-scale gyroscope range.
	* @param range New full-scale gyroscope range value
	* @see getFullScaleRange()
	* @see MPU6050_GYRO_FS_250
	* @see MPU6050_RA_GYRO_CONFIG
	* @see MPU6050_GCONFIG_FS_SEL_BIT
	* @see MPU6050_GCONFIG_FS_SEL_LENGTH
*/
void MPU6050_SetFullScaleGyroRange (uint8_t range) 
{
    invI2C_WriteRegBits(devAddr, 
						MPU6050_RA_GYRO_CONFIG, 
						MPU6050_GCONFIG_FS_SEL_BIT, 
						MPU6050_GCONFIG_FS_SEL_LENGTH, 
						range);
}

//设置MPU6050加速度计的最大量程
void MPU6050_SetFullScaleAccelRange (uint8_t range) 
{
    invI2C_WriteRegBits(devAddr, 
						MPU6050_RA_ACCEL_CONFIG, 
						MPU6050_ACONFIG_AFS_SEL_BIT, 
						MPU6050_ACONFIG_AFS_SEL_LENGTH, 
						range);
}

//读取MPU6050 WHO_AM_I标识将返回0x68(104)
uint8_t MPU6050_GetDeviceID (void) 
{
	uint8_t buffer[14];
	
    invI2C_ReadDevLenBytes(	devAddr,
							MPU6050_RA_WHO_AM_I, 
							1, 
							buffer);
	
    return buffer[0];
}

//设置MPU6050是否进入睡眠模式
void MPU6050_SetSleepEnabled (FunctionalState ctrl) 
{
    invI2C_WriteRegBit(	devAddr, 
						MPU6050_RA_PWR_MGMT_1, 
						MPU6050_PWR1_SLEEP_BIT, 
						(!ctrl));
}

//设置MPU6050是否为AUX I2C线的主机
void MPU6050_SetI2CMasterModeEnabled (FunctionalState ctrl) 
{
    invI2C_WriteRegBit( devAddr, 
						MPU6050_RA_USER_CTRL, 
						MPU6050_USERCTRL_I2C_MST_EN_BIT, 
						(!ctrl));
}

//设置 MPU6050是否为AUX I2C线的主机
void MPU6050_SetI2CBypassEnabled (FunctionalState ctrl) 
{
    invI2C_WriteRegBit(	devAddr, 
						MPU6050_RA_INT_PIN_CFG, 
						MPU6050_INTCFG_I2C_BYPASS_EN_BIT, 
						(!ctrl));
}

//设置MPU6050数据转换完成中断模式
void MPU6050_SetDataInterrupt (void) 
{
	//配置MPU6050的中断模式和中断电平模式
    invI2C_WriteRegBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, 0);
    invI2C_WriteRegBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, 0);
    invI2C_WriteRegBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, MPU_DataTransferFinishedINTLevel);
    invI2C_WriteRegBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, 1);
    //开数据转换完成中断
    invI2C_WriteRegBit(devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, 1);
}

//检测MPU6050 是否已经连接
Bool_ClassType MPU6050_TestConnection (void) 
{
	return (MPU6050_GetDeviceID() == MPU6050_DEFAULT_ADDRESS)? True : False;
}

//初始化MPU6050设备
void MPU6050_DeviceInit (void) 
{
	EulerAngleStructureInit(&eas);
	invI2C_IO_Init();										//I2C接口初始化
    MPU6050_SetClockSource(MPU6050_CLOCK_PLL_XGYRO); 		//设置时钟
    MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_2000);	//陀螺仪最大量程 +-1000度每秒
    MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);		//加速度度最大量程 +-2G
    MPU6050_SetSleepEnabled(ENABLE); 						//进入工作状态
	MPU6050_SetI2CMasterModeEnabled(ENABLE);	 			//不让MPU6050控制AUXI2C
	MPU6050_SetI2CBypassEnabled(ENABLE);	 				//主控制器的I2C与MPU6050的AUXI2C直通，控制器可以直接访问其他设备
	MPU6050_SetInnerDMPInit();								//内置DMP初始化
	MPU6050_SetDataInterrupt();								//设置数据中断模式
}

//读取MPU6050内置温度传感器数据
float MPU6050_ReadTemperature (void)
{	   
	float Temp;
	
	//read once
	Temp = (invI2C_ReadDevByte(devAddr, MPU6050_RA_TEMP_OUT_H) << 8) 
		+ invI2C_ReadDevByte(devAddr, MPU6050_RA_TEMP_OUT_L);
	//transfer to degree celsius
	Temp = ((Temp > 32768)? Temp -= 65536 : Temp) / 340 + 36.53;
	
	return Temp;
}

/**
  * @brief  Read mpu inner dmp attitude info and calculate it.
  * @param  None.
  * @retval Calculate euler successfully or fatal.
  */
void dmpAttitudeAlgorithm (EulerAngleStructure *ea)
{	
	u8 more;
	unsigned long sensor_timestamp;
	long quat[4];										//四元数获取
	short gyro[3], accel[3], sensors;
	float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	//calculation bias

	//read dmp once and write into array memory
	if (Is_MPUDataTransfer_Finished)
	{
		dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
		__ShellHeadSymbol__; 	
		if (sensors & INV_WXYZ_QUAT)
		{    
			U1SD("Attitude Algorithm Method: DMP, Process OK\r\n");	
			q0 = quat[0] / q30;
			q1 = quat[1] / q30;
			q2 = quat[2] / q30;
			q3 = quat[3] / q30;
			
			//matrix transfer
			ea -> pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3f; 	 
			ea -> roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3f; 
			ea -> yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3f;
			
			RadRangeLimitExcess(ea -> pitch);
			RadRangeLimitExcess(ea -> roll);
			RadRangeLimitExcess(ea -> yaw);
		}
		//calculate fatal
		else
			U1SD("Attitude Algorithm Method: DMP, Process Fatal\r\n");	
	}
}

//====================================================================================================
//code by </MATRIX>@Neod Anderjon
