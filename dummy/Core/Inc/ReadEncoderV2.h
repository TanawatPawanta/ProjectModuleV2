/*
 * ReadEncoder.h
 *
 *  Created on: May 19, 2023
 *      Author: tanawatp
 */

#ifndef INC_READENCODERV2_H_
#define INC_READENCODERV2_H_

#include "main.h"
#include "arm_math.h"

typedef struct
{
	int32_t QEIPosition_minus;
	int32_t QEIPosition;
	float32_t QEIVelocity;
}QEI;

typedef struct ReadEncoderParam
{
	uint8_t MotorSetDuty;
	uint16_t Pulse_Compare;
	uint8_t DIR;
}ReadEncoder;


void InitReadEncoder(ReadEncoder* Read, uint32_t samplingtime);
uint64_t micros();
void QEIGetFeedback(QEI* temp, uint16_t Hz);
//void QEIEncoderPositionVelocity_Update();



#endif /* INC_READENCODERV2_H_ */
