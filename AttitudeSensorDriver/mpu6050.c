#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//author: Neod Anderjon
//====================================================================================================
//MPU6050基础驱动

GyroAccelStructure gas;
EulerAngleStructure eas;
volatile float MPU_GlobalTemp;

//陀螺仪数据结构体初始化
static void GyroAccelStructureInit (GyroAccelStructure *ga)
{
	ga -> gx = 0;
	ga -> gy = 0;
	ga -> gz = 0;
	ga -> ax = 0;
	ga -> ay = 0;
	ga -> az = 0;
}

//欧拉角结构体初始化
static void EulerAngleStructureInit (EulerAngleStructure *ea)
{
	ea -> pitch = 0.f;
	ea -> roll = 0.f;
	ea -> yaw = 0.f;
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

//陀螺仪方向控制
static u16 inv_orientation_matrix_to_scalar (const signed char *mtx)
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

//MPU自检测试
static Bool_ClassType run_self_test (void)
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

//MPU6050内置DMP初始化
static uint8_t mpu_intrinsic_dmp_init (void)
{ 
	//algorithm matrix
	signed char gyro_orientation[9] = {-1, 	0, 	0,
										0, 	-1, 0,
										0, 	0, 	1};
	
	//@InvenSense DMP library call
	if (!mpu_init())
	{
		if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL)) 			return 1;
		if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))		return 2;
		if (mpu_set_sample_rate(MPUDataReadFreq))					return 3;
		if (dmp_load_motion_driver_firmware())						return 4;
		if (dmp_set_orientation(
			inv_orientation_matrix_to_scalar(gyro_orientation))) 	return 5;
		if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
	        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | 
			DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL))		return 6;
		if (dmp_set_fifo_rate(MPUDataReadFreq))						return 7;
		if (run_self_test())										return 8;
		if (mpu_set_dmp_state(ENABLE))								return 9;
	}
	else															return 10;
																	return 0;
}

/*
	设置MPU6050的数字低通滤波器
	lpf: 数字低通滤波频率，一般是采样率的一半
*/
static u8 MPU6050_SetDigitalLowFilter (u16 lpf)
{
	u8 data;
	
	//数据分段节选
	if (lpf >= 188)		data = 0x01;
	else if (lpf >= 98)	data = 0x02;
	else if (lpf >= 42)	data = 0x03;
	else if (lpf >= 20)	data = 0x04;
	else if (lpf >= 10)	data = 0x05;
	else 				data = 0x06;
	
	//设置数字低通滤波器  
	return invI2C_WriteDevByte(MPUDEVADDR, MPU6050_RA_CONFIG, data);
}

//设置MPU6050的采样率
static u8 MPU6050_SetSampleRate (u16 rate)
{
	u16 srf;
	
	//数据限位，最大实际200hz
	srf = rate;
	if (srf > 200)	srf = 200;
	if (srf < 4) 	srf = 4;
	//设置数字低通滤波器
	invI2C_WriteDevByte(MPUDEVADDR, MPU6050_RA_SMPLRT_DIV, (u8)(1000 / srf - 1));
	
	//自动设置滤波频率为采样率的一半
 	return MPU6050_SetDigitalLowFilter(rate / 2);	
}

//MPU6050 INT脚初始化
void MPU6050_INT_IO_Init (void)
{
	//PB12 MPU6050数据读取中断INT引脚，设置为低电平触发
	ucGPIO_Config_Init (RCC_APB2Periph_GPIOB,			
						GPIO_Mode_IPU,									//一般设置成上拉输入					
						GPIO_Input_Speed,								//无效参数						
						GPIORemapSettingNULL,							
						GPIO_Pin_12,					
						GPIOB,					
						IHL,													
						EBO_Disable);
}

//初始化陀螺仪设备
Bool_ClassType GyroscopeTotalComponentInit (void) 
{
	uint8_t i, dmp_res;
	
	GyroAccelStructureInit(&gas);
	EulerAngleStructureInit(&eas);
	invI2C_IO_Init();													//I2C port init
	invI2C_WriteDevByte(MPUDEVADDR, MPU6050_RA_PWR_MGMT_1, 0x80);		//reset device
	delay_ms(100);
	//wake up device
	invI2C_WriteDevByte(MPUDEVADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_INTERNAL);		 
	invI2C_WriteDevByte(MPUDEVADDR, MPU6050_RA_GYRO_CONFIG, 3 << 3);	//gyroscope sensor, ±2000dps
	invI2C_WriteDevByte(MPUDEVADDR, MPU6050_RA_ACCEL_CONFIG, 0 << 3);	//accelerator sensor, ±2g
	MPU6050_SetSampleRate(MPUDataReadFreq);								//setting sample rate
	invI2C_WriteDevByte(MPUDEVADDR, MPU6050_RA_INT_ENABLE, 0x00);		//setting all interrupt, 0x00 is close
	invI2C_WriteDevByte(MPUDEVADDR, MPU6050_RA_USER_CTRL, 0x00);		//setting device I2C master mode, 0x00 is close
	invI2C_WriteDevByte(MPUDEVADDR, MPU6050_RA_FIFO_EN, 0x01);			//setting data transfer FIFO, 0x00 is close
	invI2C_WriteDevByte(MPUDEVADDR, MPU6050_RA_INT_PIN_CFG, 0x80);		//INT pin low level valid
	/* read device id, if success, setting clock. */
	if (invI2C_ReadDevByte(MPUDEVADDR, MPU6050_RA_WHO_AM_I) == MPUDEVADDR)
	{
		//setting CLKSEL, PLL X-axis, default inner 8M, low precision
		invI2C_WriteDevByte(MPUDEVADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO);
		//setting accelerator and gyroscope all work		
		invI2C_WriteDevByte(MPUDEVADDR, MPU6050_RA_PWR_MGMT_2, 0x00);		
		MPU6050_SetSampleRate(MPUDataReadFreq);
	}
	else
	{
		/* device address read fatal, can't build valid link, directly software restart. */
		__ShellHeadSymbol__; U1SD("MPU Read Device Address Fatal, Restarting\r\n");
		Sys_Soft_Reset();
	}
	
	/* 	Intrinsic DMP init. 
	 * 	MPU Gyroscope init successfully or not is very important, but sometime it may be fatal.
	 *  It must be finish all of pre-setting init process can into a correct work status.
	**/
	__ShellHeadSymbol__; U1SD("MPU Intrinsic DMP Library, Loading...");
	dmp_res = mpu_intrinsic_dmp_init();
	while (dmp_res)
	{
		/* retry count more than setting value, software reset. */
		if (i++ == 5)
		{
			i = 0;
			U1SD("More Fatal, Restarting\r\n");
			Sys_Soft_Reset();
		}
		/* once fatal, get error code and retry. */
		if (No_Data_Receive && PC_Switch == PC_Enable)
		{
			printf("Once Fatal Return Code: %d, Retrying\r\n", dmp_res);
			usart1WaitForDataTransfer();		
		}
		delay_ms(200);
		dmp_res = mpu_intrinsic_dmp_init();
	}		
	U1SD("Successfully\r\n");	
	
	return False;
}

//MPU获取陀螺仪加速度计数据
void MPU6050_GetGyroAccelOriginData (GyroAccelStructure *ga)
{
    u8 i, buf[6] = {0};  
	
	//读陀螺仪
	if (!i2cRead(MPUDEVADDR, MPU6050_RA_GYRO_XOUT_H, 6, buf))
	{
		ga -> gx = ((u16)buf[0] << 8) | buf[1];  
		ga -> gy = ((u16)buf[2] << 8) | buf[3];  
		ga -> gz = ((u16)buf[4] << 8) | buf[5];
	} 	
	//清除缓存
	for (i = 0; i < 6; i++)
		buf[i] = 0;
	//读加速度
	if (!i2cRead(MPUDEVADDR, MPU6050_RA_ACCEL_XOUT_H, 6, buf))
	{
		ga -> ax = ((u16)buf[0] << 8) | buf[1];  
		ga -> ay = ((u16)buf[2] << 8) | buf[3];  
		ga -> az = ((u16)buf[4] << 8) | buf[5];
	} 
	/* print test visual 
	if (No_Data_Receive && PC_Switch == PC_Enable)
	{
		printf("\r\ngx: %8d | gy: %8d | gz: %8d\r\n", ga -> gx, ga -> gy, ga -> gz);
		printf("\r\nax: %8d | ay: %8d | az: %8d\r\n", ga -> ax, ga -> ay, ga -> az);
		usart1WaitForDataTransfer();		
	}*/
}

//读取MPU6050内置温度传感器数据
float MPU6050_ReadTemperature (void)
{	
	u8 buf[2]; 
    volatile short raw;
	volatile float temp;
	
	i2cRead(MPUDEVADDR, MPU6050_RA_TEMP_OUT_H, 2, buf); 
    raw = ((u16)buf[0] << 8) | buf[1];  
	//here transfer to degree celsius
    temp = 36.53f + ((double)raw) / 340.f; 
	/* print test visual 
	if (No_Data_Receive && PC_Switch == PC_Enable)
	{
		printf("\r\nMPU Temperature: %.2f\r\n", temp);
		usart1WaitForDataTransfer();		
	}*/
	
	return temp;
}

/**
  * @brief  Read mpu inner dmp attitude info and calculate it.
  * @param  *ea: A EulerAngleStructure structure type point.
  * @retval Calculate euler successfully or fatal.
  */
uint8_t dmpAttitudeAlgorithm (EulerAngleStructure *ea)
{	
	u8 i, more;
	long quat[4];											
	short gyro[3], accel[3], sensors;
	unsigned long sensor_timestamp;
	volatile float qbias[4] = {1.f, 0.f, 0.f, 0.f};		

	/* 	Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
	 * 	This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
	 * 	Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
	 * 	The orientation is set by the scalar passed to dmp_set_orientation during initialization. 
	 * 	This function run result directly calculate result, we need to avoid the execution is interrupted.
	**/
	if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))
	{
		/* here should give fatal read process up. */
		//__ShellHeadSymbol__; U1SD("Gyroscope Read DMP Fatal\r\n");
		return 1;
	}
	if (sensors & INV_WXYZ_QUAT)
	{    
		/* division 2^30 amplify. */
		for (i = 0; i < 4; i++)
			qbias[i] = quat[i] / q30;
		
		/* 	4 quats number matrix calculate and data result calibration. */
		/* -pi/2<=pitch<=pi/2. */
		ea -> pitch = (float)asin(-2 * qbias[1] * qbias[3] + 2 * qbias[0] * qbias[2]) * RadTransferDegree; 
		AngleRangeLimitExcess(ea -> pitch);		
		/* -pi<=roll<=pi. */
		ea -> roll = (float)atan2(2 * qbias[2] * qbias[3] + 2 * qbias[0] * qbias[1], 
			-2 * qbias[1] * qbias[1] - 2 * qbias[2] * qbias[2] + 1) * RadTransferDegree; 
		AngleRangeLimitExcess(ea -> roll);
		/* -pi<=yaw<=pi. */
		ea -> yaw = (float)atan2(2 * (qbias[1] * qbias[2] + qbias[0] * qbias[3]), 
			qbias[0] * qbias[0] + qbias[1] * qbias[1] - qbias[2] * qbias[2] - qbias[3] * qbias[3]) * RadTransferDegree;
		AngleRangeLimitExcess(ea -> yaw);
		
		/*
		//print into com test visual
		__ShellHeadSymbol__;
		if (No_Data_Receive && PC_Switch == PC_Enable)
		{
			printf("Euler Angle USART Outputs: [Pitch: %.2f | Roll: %.2f | Yaw: %.2f]\r\n", 
					ea -> pitch, ea -> roll, ea -> yaw);			
			usart1WaitForDataTransfer();		
		}
		*/
	
		return 0;
	}
	else
	{
		/* give here up too. */
		//__ShellHeadSymbol__; U1SD("Gyroscope DMP Algorithm Fatal\r\n");
		return 2;
	}
}

//====================================================================================================
//code by </MATRIX>@Neod Anderjon
