/*
 * ReadEncoderV2.c
 *
 *  Created on: May 19, 2023
 *      Author: tanawatp
 */
#include "ReadEncoderV2.h"
#include "arm_math.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

//extern uint8_t var;

extern QEI QEIData;
extern ReadEncoder ReadEncoderParam;

//uint64_t micros()
//{ //get time in micros
//	return __HAL_TIM_GET_COUNTER(&htim5)+ ReadEncoderParam._micros;
//}

void InitReadEncoder(ReadEncoder* Read, uint32_t samplingtime)
{
	Read->MotorSetDuty = 0;
	Read->Pulse_Compare = 0;
	Read->DIR = 0;
}

//void QEIEncoderPositionVelocity_Update(){
//	//collect data
//	QEIData.timestamp[0] = micros();
//	uint32_t couterPosition = __HAL_TIM_GET_COUNTER(&htim2);
//	QEIData.data[0] = couterPosition;
//
//	//calculation
//	QEIData.QEIPosition = couterPosition % ReadEncoderParam.PPR;
//
//	int32_t diffPosition = QEIData.data[0] - QEIData.data[1];
//	float difftime = (QEIData.timestamp[0] - QEIData.timestamp[1]);
//
//
//	//calculate
//	QEIData.QEIVelocity = (diffPosition * 1000000*60.0)/(difftime*8192.0);
//
//	//Delay
//	QEIData.data[1] = QEIData.data[0];
//	QEIData.timestamp[1] = QEIData.timestamp[0];
//}
void QEIGetFeedback(QEI* temp, uint16_t Hz)
{
	temp->QEIPosition = __HAL_TIM_GET_COUNTER(&htim2);
	temp->QEIVelocity = (temp->QEIPosition - temp->QEIPosition_minus) * Hz;
	temp->QEIPosition_minus = temp->QEIPosition;
}
