#ifndef __FILTER_H__
#define __FILTER_H__
#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//author: Neod Anderjon
//====================================================================================================
//软件滤波器，在不使用DMP直接解析四元数时对原始数据做处理	

/*
	一阶互补滤波 first order complementary filter
	am: 	测量输入加速度
	gm: 	测量输入陀螺仪
	k:  	权值系数
	dt:		一阶时间(由程序执行时间确认，一般取0.005左右)
	ao:		加速度滤波输出
*/
#ifndef FirstOrderComplementaryFilter
#define FirstOrderComplementaryFilter(am, gm, k, dt, ao) {ao = am * k + (1 - k) * (ao + gm * dt);}   
#endif
void Kalman_Filter (float Accel, float Gyro);		

#endif

//====================================================================================================
//code by </MATRIX>@Neod Anderjon
