/*
 * ReadEncoderV2.c
 *
 *  Created on: May 19, 2023
 *      Author: tanawatp
 */
#include "ReadEncoderV2.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;


//extern uint8_t var;


extern  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
extern QEIStructureTypedef QEIData;
extern uint64_t _micros;
extern uint16_t PPR;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){ //get time period
	if(htim == & htim5){
		_micros += UINT32_MAX;
	}
}
uint64_t micros(){ //get time in micros
	return __HAL_TIM_GET_COUNTER(&htim5)+ _micros;
}

void QEIEncoderPositionVelocity_Update(){
	//collect data
	QEIData.timestamp[0] = micros();
	uint32_t couterPosition = __HAL_TIM_GET_COUNTER(&htim2);
	QEIData.data[0] = couterPosition;

	//calculation
	QEIData.QEIPosition = couterPosition % PPR;

	int32_t diffPosition = QEIData.data[0] - QEIData.data[1];
	float difftime = (QEIData.timestamp[0] - QEIData.timestamp[1]);


	//calculate
	QEIData.QEIVelocity = (diffPosition * 1000000*60.0)/(difftime*8192.0);

	//Delay
	QEIData.data[1] = QEIData.data[0];
	QEIData.timestamp[1] = QEIData.timestamp[0];
}

