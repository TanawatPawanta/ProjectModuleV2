/*
 * TrayLocalization.c
 *
 *  Created on: May 31, 2023
 *      Author: tanawatp
 */
#include "TrayLocalization.h"
#include "arm_math.h"
#include  "math.h"
void TraySetup(Tray* temp, float32_t x1, float32_t y1, float32_t x2, float32_t y2)
{
	temp->Edge1_X = x1;
	temp->Edge1_Y = y1;
	temp->Edge2_X = x2;
	temp->Edge2_Y = y2;
}
void TrayLocalization(Tray* temp)
{
	float32_t RefX_case1[3] = {10.0,30.0,50.0};
	float32_t RefY_case1[3] = {40*8192.0/120.0,25*8192.0/120.0,10*8192.0/120.0};

	float32_t RefX_case2[3] = {10.0,25.0,40.0};
	float32_t RefY_case2[3] = {50*8192.0/120.0,30*8192.0/120.0,10*8192.0/120.0};
	float32_t deltaX = (temp->Edge2_X - temp->Edge1_X)*8192.0/120.0;
	float32_t deltaY = temp->Edge2_Y - temp->Edge1_Y;
	float32_t lengh = sqrt(pow(deltaX,2) + pow(deltaY,2));

	if((3276 <= lengh)&&(lengh <= 3550))
	{
		temp->Flag = 2; //swap case
	}
	else if ((3960 <= lengh)&&(lengh <= 4233))
	{
		temp->Flag = 1;
	}
	else
	{
		temp->Flag = 0;//Lenght Error!!
	}
	float32_t theta;
	arm_atan2_f32(deltaY,deltaX,&theta);
	temp->angle = theta;
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
								+ (cos(theta)*RefX_case1[i])
								- (sin(theta)*RefY_case1[j]*120.0/8192.0);
				temp->Holes_Y[ind] = temp->Edge1_Y
								+ (cos(theta)*RefY_case1[j])
								+ (sin(theta)*RefX_case1[i]*8192.0/120.0);
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
								+ (cos(theta)*RefX_case2[i])
								- (sin(theta)*RefY_case2[j]*120.0/8192.0);
				temp->Holes_Y[ind] = temp->Edge1_Y
								+ (cos(theta)*RefY_case2[j])
								+ (sin(theta)*RefX_case2[i]*8192.0/120.0);
				ind += 1;
			}
		}
		break;
	case 0:
		break;
	}
}
