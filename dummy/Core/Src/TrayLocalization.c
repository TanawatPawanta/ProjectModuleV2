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

	float32_t Cal_OriginX = x2 * 10;
	float32_t Cal_OriginY = ( (y2*120.0/8192.0) - 350) * 10;
	if(Cal_OriginX < 0){
		Cal_OriginX = 65536 + Cal_OriginX;
	}
	if(Cal_OriginY < 0){
		Cal_OriginY = 65536 + Cal_OriginY;
	}
	temp->ForBaseOriginX = Cal_OriginX ;
	temp->ForBaseOriginY = Cal_OriginY - 100 ;
}
void TrayLocalization(Tray* temp)
{
	float32_t RefX_case2[3] = {10.0,25.0,40.0};
	float32_t RefY_case2[3] = {50*8192.0/120.0,30*8192.0/120.0,10*8192.0/120.0};
	float32_t deltaX = (temp->Edge2_X - temp->Edge1_X)*8192.0/120.0;
	float32_t deltaY = temp->Edge2_Y - temp->Edge1_Y;

	float32_t theta;
	arm_atan2_f32(deltaY,deltaX,&theta);
	temp->angle = theta;

	float32_t Cal_Orientation = -(theta * (180/PI) ) - 90;
	if(Cal_Orientation < 0){
		Cal_Orientation = Cal_Orientation + 360;
	}
	temp->ForBaseOrientation = Cal_Orientation * 100;

	int8_t i;
	int8_t j;
	uint8_t ind = 0;
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
}
