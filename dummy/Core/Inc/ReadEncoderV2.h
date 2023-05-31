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

typedef struct QEI
{
	int64_t data[2];
	uint64_t timestamp[2];

	float32_t QEIPosition;
	float32_t QEIVelocity;
}QEIStructureTypedef;

typedef struct ReadEncoderParam
{
	uint64_t _micros;
	uint16_t PPR;
	uint32_t samplingTime; //us => 1000 Hz
	//PWM
	uint8_t MotorSetDuty;
	uint16_t Pulse_Compare;
	uint8_t DIR;
}ReadEncoder;


void InitReadEncoder(ReadEncoder* Read, uint32_t samplingtime);
uint64_t micros();
void QEIEncoderPositionVelocity_Update();



#endif /* INC_READENCODERV2_H_ */
