#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//author: Neod Anderjon
//====================================================================================================
//MPU6050基础驱动

GyroAccelStructure gas;
EulerAngleStructure eas;
float MPU_GlobalTemp;

//陀螺仪数据结构体初始化
void GyroAccelStructureInit (GyroAccelStructure *ga)
{
	ga -> gx = 0;
	ga -> gy = 0;
	ga -> gz = 0;
	ga -> ax = 0;
	ga -> ay = 0;
	ga -> az = 0;
}

//欧拉角结构体初始化
void EulerAngleStructureInit (EulerAngleStructure *ea)
{
	ea -> pitch = 0.f;
	ea -> roll = 0.f;
	ea -> yaw = 0.f;
}

//MPU自检测试
Bool_ClassType run_self_test (void)
{
    int result;
    long gyro[3], accel[3];
	
    result = mpu_run_self_test(gyro, accel);
    if (result == 0x3) 							//MPU6500: 0x7
	{
		//Test passed. We can trust the gyro data here, so let's push it down to the DMP.
		float sens;
		u16 accel_sens;
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
u16 inv_row_2_scale (const signed char *row)
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

//陀螺仪方向控制
u16 inv_orientation_matrix_to_scalar (const signed char *mtx)
{
    u16 scalar;
	/*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */
	
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

//MPU6050内置DMP初始化
uint8_t MPU6050_SetInnerDMPInit (void)
{ 
	//algorithm matrix
	static signed char gyro_orientation[9] = {-1, 	0, 	0,
										0, 	-1, 0,
										0, 	0, 	1};
	
	//@InvenSense DMP library call
	if (!mpu_init())
	{
		U1SD("\r\n[0] mpu_device_register complete\r\n");
		if (!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
		{
			U1SD("\r\n[1] mpu_set_sensor complete\r\n");
		}
		else 
			return 1;
		if (!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
		{
			U1SD("\r\n[2] mpu_configure_fifo complete\r\n");
		}
		else
			return 2;
		if (!mpu_set_sample_rate(DEFAULT_MPU_HZ))
		{
			U1SD("\r\n[3] mpu_set_sample_rate complete\r\n");
		}
		else
			return 3;
		if (!dmp_load_motion_driver_firmware())
		{
			U1SD("\r\n[4] dmp_load_motion_driver_firmware complete\r\n");
		}
		else
			return 4;
		if (!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
		{
			U1SD("\r\n[5] dmp_set_orientation complete\r\n");
		}
		else
			return 5;
		if (!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
	        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | 
			DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL))
		{
			U1SD("\r\n[6] dmp_enable_feature complete\r\n");
		}
		else
			return 6;
		if (!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
		{
			U1SD("\r\n[7] dmp_set_fifo_rate complete\r\n");
		}
		else
			return 7;
		if (!run_self_test())
		{
			U1SD("\r\n[8] dmp_set_bias complete\r\n");
		}
		else
			return 8;
		if (!mpu_set_dmp_state(ENABLE))
		{
			U1SD("\r\n[9] mpu_set_dmp_state complete\r\n");
		}
		else
			return 9;
	}
	else
		return 10;
	return 0;
}

//初始化前的短暂延时
void MPU6050_BeforeDelay (void)
{
	u16 i, j;
	
	for (i = 0; i < 1000; i++)
		for (j = 0; j < 1000; j++);
}

//设置MPU6050的数字低通滤波器，lpf:数字低通滤波频率(Hz)
u8 MPU6050_SetDigitalLowFilter (u16 lpf)
{
	u8 data = 0;
	
	if (lpf >= 188)
		data = 1;
	else if (lpf >= 98)
		data = 2;
	else if (lpf >= 42)
		data = 3;
	else if (lpf >= 20)
		data = 4;
	else if (lpf >= 10)
		data = 5;
	else 
		data = 6;
	
	//设置数字低通滤波器  
	return invI2C_WriteDevByte(MPU6050_RA_CONFIG, data);
}

//设置MPU6050的采样率(假定Fs=1KHz)，rate:4~1000(Hz)
u8 MPU6050_SetSampleRate (u16 rate)
{
	u8 data;
	
	if (rate > 1000)
		rate = 1000;
	if (rate < 4) rate = 4;
	data = 1000 / rate - 1;
	//设置数字低通滤波器
	data = invI2C_WriteDevByte(MPU6050_RA_SMPLRT_DIV, data);
	
	//自动设置LPF为采样率的一半
 	return MPU6050_SetDigitalLowFilter(rate / 2);	
}

//初始化MPU6050设备
Bool_ClassType MPU6050_DeviceInit (void) 
{
	GyroAccelStructureInit(&gas);
	EulerAngleStructureInit(&eas);
	invI2C_IO_Init();											//I2C接口初始化
	invI2C_WriteDevByte(MPU6050_RA_PWR_MGMT_1, 0x80);			//复位MPU6050
	delay_ms(100);
	invI2C_WriteDevByte(MPU6050_RA_PWR_MGMT_1, 0x00);			//唤醒MPU6050 
	invI2C_WriteDevByte(MPU6050_RA_GYRO_CONFIG, 3 << 3);		//陀螺仪传感器,±2000dps
	invI2C_WriteDevByte(MPU6050_RA_ACCEL_CONFIG, 0 << 3);		//加速度传感器,±2g
	MPU6050_SetSampleRate(DEFAULT_MPU_HZ);						//设置采样率
	invI2C_WriteDevByte(MPU6050_RA_INT_ENABLE, 0x00);			//关闭所有中断
	invI2C_WriteDevByte(MPU6050_RA_USER_CTRL, 0x00);			//I2C主模式关闭
	invI2C_WriteDevByte(MPU6050_RA_FIFO_EN, 0x00);				//关闭FIFO
	invI2C_WriteDevByte(MPU6050_RA_INT_PIN_CFG, 0x80);			//INT引脚低电平有效
	//检测器件ID
	if (invI2C_ReadDevByte(MPU6050_RA_WHO_AM_I) == MPU6050_DEFAULT_ADDRESS)
	{
		invI2C_WriteDevByte(MPU6050_RA_PWR_MGMT_1, 0x01);		//设置CLKSEL,PLL X轴为参考
		invI2C_WriteDevByte(MPU6050_RA_PWR_MGMT_2, 0x00);		//加速度与陀螺仪都工作
		MPU6050_SetSampleRate(DEFAULT_MPU_HZ);
	}
	else
		return True;
	//内置DMP初始化
	while (MPU6050_SetInnerDMPInit())
	{
		__ShellHeadSymbol__; U1SD("MPU DMP Init Error, Restarting\r\n");
		delay_ms(200);
	}		
	return False;
}

//MPU获取陀螺仪加速度计数据
void MPU6050_GetGyroAccelOriginData (GyroAccelStructure *ga)
{
    u8 i, buf[6] = {0};  
	
	//读陀螺仪
	if (!i2cRead(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_H, 6, buf))
	{
		ga -> gx = ((u16)buf[0] << 8) | buf[1];  
		ga -> gy = ((u16)buf[2] << 8) | buf[3];  
		ga -> gz = ((u16)buf[4] << 8) | buf[5];
	} 	
	//清除缓存
	for (i = 0; i < 6; i++)
		buf[i] = 0;
	//读加速度
	if (!i2cRead(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 6, buf))
	{
		ga -> ax = ((u16)buf[0] << 8) | buf[1];  
		ga -> ay = ((u16)buf[2] << 8) | buf[3];  
		ga -> az = ((u16)buf[4] << 8) | buf[5];
	} 
//	if (No_Data_Receive && PC_Switch == PC_Enable)
//	{
//		printf("\r\ngx: %8d | gy: %8d | gz: %8d\r\n", ga -> gx, ga -> gy, ga -> gz);
//		printf("\r\nax: %8d | ay: %8d | az: %8d\r\n", ga -> ax, ga -> ay, ga -> az);
//		usart1WaitForDataTransfer();		
//	}
}

//读取MPU6050内置温度传感器数据
float MPU6050_ReadTemperature (void)
{	
	u8 buf[2]; 
    short raw;
	float temp;
	
	i2cRead(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_TEMP_OUT_H, 2, buf); 
    raw = ((u16)buf[0] << 8) | buf[1];  
    temp = 36.53 + ((double)raw) / 340; 
	
	return temp;
}

/**
  * @brief  Read mpu inner dmp attitude info and calculate it.
  * @param  None.
  * @retval Calculate euler successfully or fatal.
  */
uint8_t dmpAttitudeAlgorithm (EulerAngleStructure *ea)
{	
	u8 more;
	long quat[4];										//四元数获取
	float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	//calculation bias
	short gyro[3], accel[3], sensors;
	unsigned long sensor_timestamp;

	//read dmp once and write into array memory
	if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))
	{
		__ShellHeadSymbol__; U1SD("Gyroscope Read DMP Fatal\r\n");
		return 1;
	}
	/* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
	 * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
	**/
	/*if (sensors & INV_XYZ_GYRO )
		send_packet(PACKET_TYPE_GYRO, gyro);
	if (sensors & INV_XYZ_ACCEL)
		send_packet(PACKET_TYPE_ACCEL, accel); */
	/* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
	 * The orientation is set by the scalar passed to dmp_set_orientation during initialization. 
	**/
	if (sensors & INV_WXYZ_QUAT)
	{    
		q0 = quat[0] / q30;
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30;
		
		//matrix transfer
		ea -> pitch = (float)asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3f; 	 
		ea -> roll = (float)atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3f; 
		ea -> yaw = (float)atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3f;
		
		//number limit range
		RadRangeLimitExcess(ea -> pitch);
		RadRangeLimitExcess(ea -> roll);
		RadRangeLimitExcess(ea -> yaw);
		
		//com port test
//		__ShellHeadSymbol__;
//		if (No_Data_Receive && PC_Switch == PC_Enable)
//		{
//			printf("Gyroscope Debug Mode, Euler Angle Print: [Pitch: %.2f | Roll: %.2f | Yaw: %.2f]\r\n", 
//					ea -> pitch, ea -> roll, ea -> yaw);			
//			usart1WaitForDataTransfer();		
//		}
		
		return 0;
	}
	else
	{
		__ShellHeadSymbol__; U1SD("Gyroscope DMP Algorithm Fatal\r\n");
		return 2;
	}
}

//====================================================================================================
//code by </MATRIX>@Neod Anderjon
