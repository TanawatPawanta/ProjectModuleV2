/*
 * PIDController.h
 *
 *  Created on: May 30, 2023
 *      Author: tanawatp
 */

#ifndef INC_PIDCONTROLLER_H_
#define INC_PIDCONTROLLER_H_
#include "main.h"
#include "arm_math.h"

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	int32_t U;
	int32_t Delta_U;
	int32_t U_minus;
	float32_t Error;
	float32_t Error_minus;
	float32_t Error_minus2;
	int8_t MotorDir;

}PID;

void PIDSetup(PID* temp,float32_t Ki, float32_t Kp, float32_t Kd);
void PIDRun(PID* temp, float32_t SensoRead, float32_t Ref);
void MotorWrite();

#endif /* INC_PIDCONTROLLER_H_ */
