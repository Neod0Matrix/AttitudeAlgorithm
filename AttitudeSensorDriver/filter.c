#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//author: Neod Anderjon
//====================================================================================================
//软件滤波器，在不使用DMP直接解析四元数时对原始数据做处理

float K1 = 0.02; 
float angle, angle_dot; 	
float Q_angle = 0.001;			//过程噪声的协方差
float Q_gyro = 0.003;			//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
float R_angle = 0.5;			//测量噪声的协方差 既测量偏差
float dt = 0.005;               
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] = {0, 0, 0, 0};
float PP[2][2] = {{1, 0}, {0, 1}};

//卡尔曼滤波
void Kalman_Filter (float Accel, float Gyro)		
{
	//先验估计
	angle += (Gyro - Q_bias) * dt; 
	// Pk-先验估计误差协方差的微分
	Pdot[0] = Q_angle - PP[0][1] - PP[1][0]; 

	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3] = Q_gyro;
	// Pk-先验估计误差协方差微分的积分
	PP[0][0] += Pdot[0] * dt;   
	PP[0][1] += Pdot[1] * dt;   
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	//zk-先验估计
	Angle_err = Accel - angle;	
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	//后验估计误差协方差
	PP[0][0] -= K_0 * t_0;		 
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
	
	//后验估计	
	angle += K_0 * Angle_err;	 
	Q_bias += K_1 * Angle_err;	 
	//输出值(后验估计)的微分=角速度
	angle_dot = Gyro - Q_bias;	 
}

//====================================================================================================
//code by </MATRIX>@Neod Anderjon
