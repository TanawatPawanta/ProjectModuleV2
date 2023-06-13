/*
 * Storage.c
 *
 *  Created on: Jun 9, 2023
 *      Author: tanawatp
 */
#include "Storage.h"
extern u16u8_t registerFrame[200];

void SetHome(OperationVar* temp)
{
	temp->HomingKey = 1;
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,35*500);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
}

void RunX_Axis(int32_t XFinalPos, uint16_t velo, uint16_t accel)
{
	registerFrame[0x41].U16 = XFinalPos;
	registerFrame[0x42].U16 = velo;	//250 mm/s
	registerFrame[0x43].U16 = accel;	//1000 mm/s^2
	registerFrame[0x40].U16 = 0b0010;	//Run
}
