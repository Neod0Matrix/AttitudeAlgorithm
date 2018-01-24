#ifndef __FILTER_H__
#define __FILTER_H__
#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//author: Neod Anderjon
//====================================================================================================
//软件滤波器

extern float angle, angle_dot; 	

void Kalman_Filter (float Accel,float Gyro);		
void FirOrd_Complementary_Filter (float angle_m, float gyro_m);

#endif

//====================================================================================================
//code by </MATRIX>@Neod Anderjon
