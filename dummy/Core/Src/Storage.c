/*
 * Storage.c
 *
 *  Created on: Jun 9, 2023
 *      Author: tanawatp
 */
#include "Storage.h"

void SetHome(OperationVar* temp)
{
	temp->HomingKey = 1;
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,30*500);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
}
