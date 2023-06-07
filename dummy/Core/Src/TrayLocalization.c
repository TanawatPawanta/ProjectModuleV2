/*
 * TrayLocalization.c
 *
 *  Created on: May 31, 2023
 *      Author: tanawatp
 */
#include "TrayLocalization.h"
#include "arm_math.h"

void TraySetup(Tray* temp, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	temp->Edge1_X = x1;
	temp->Edge1_Y = y1;
	temp->Edge2_X = x2;
	temp->Edge2_Y = y2;
}
void TrayLocalization(Tray* temp)
{
	uint16_t RefX_case1[3] = {10,30,50};
	uint16_t RefY_case1[3] = {40*8192/120,25*8192/120,10*8192/120};

	uint16_t RefX_case2[3] = {10,25,40};
	uint16_t RefY_case2[3] = {50*8192/120,30*8192/120,10*8192/120};
	int16_t deltaX = (temp->Edge2_X - temp->Edge1_X)*8192/120;
	int16_t deltaY = temp->Edge2_Y - temp->Edge1_Y;
	float32_t lengh = sqrt(pow(deltaX,2) + pow(deltaY,2));

	if((3276 <= lengh)&&(lengh <= 3550))
	{
		temp->Flag = 1;
	}
	else if ((3960 <= lengh)&&(lengh <= 4233))
	{
		temp->Flag = 2;
	}
	else
	{
		temp->Flag = 0;//Lenght Error!!
	}
	float32_t theta;
	arm_atan2_f32(deltaY,deltaX,&theta);
	int8_t i;
	int8_t j;
	uint8_t ind = 0;
	switch(temp->Flag)
	{
	case 1:
		for(i=0;i<3;i++)
		{
			for(j=0;j<3;j++)
			{
				temp->Holes_X[ind] = temp->Edge1_X
								+ arm_cos_f32(theta)*RefX_case1[i]
								- arm_sin_f32(theta)*RefY_case1[j];
				temp->Holes_Y[ind] = temp->Edge1_Y
								+ arm_cos_f32(theta)*RefY_case1[j]
								+ arm_sin_f32(theta)*RefX_case1[i];
				ind += 1;
			}
		}
		break;
	case 2:
		for(i=0;i<3;i++)
		{
			for(j=0;j<3;j++)
			{
				temp->Holes_X[ind] = temp->Edge1_X
								+ arm_cos_f32(theta)*RefX_case2[i]
								- arm_sin_f32(theta)*RefY_case2[j];
				temp->Holes_Y[ind] = temp->Edge1_Y
								+ arm_cos_f32(theta)*RefY_case2[j]
								+ arm_sin_f32(theta)*RefX_case2[i];
				ind += 1;
			}
		}
		break;
	case 0:
		break;
	}
}
