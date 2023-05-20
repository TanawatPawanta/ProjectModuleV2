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
extern ReadEncoder ReadEncoderParam;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){ //get time period
	if(htim == & htim5){
		ReadEncoderParam._micros += UINT32_MAX;
	}
}
uint64_t micros(){ //get time in micros
	return __HAL_TIM_GET_COUNTER(&htim5)+ ReadEncoderParam._micros;
}

void InitReadEncoder(ReadEncoder* Read, uint32_t samplingtime)
{
	Read->_micros = 0;
	Read->PPR = 8192;
	Read->samplingTime = samplingtime;
	Read->MotorSetDuty = 60;
	Read->Pulse_Compare = 0;
	Read->DIR = 0;
}

void QEIEncoderPositionVelocity_Update(){
	//collect data
	QEIData.timestamp[0] = micros();
	uint32_t couterPosition = __HAL_TIM_GET_COUNTER(&htim2);
	QEIData.data[0] = couterPosition;

	//calculation
	QEIData.QEIPosition = couterPosition % ReadEncoderParam.PPR;

	int32_t diffPosition = QEIData.data[0] - QEIData.data[1];
	float difftime = (QEIData.timestamp[0] - QEIData.timestamp[1]);


	//calculate
	QEIData.QEIVelocity = (diffPosition * 1000000*60.0)/(difftime*8192.0);

	//Delay
	QEIData.data[1] = QEIData.data[0];
	QEIData.timestamp[1] = QEIData.timestamp[0];
}

