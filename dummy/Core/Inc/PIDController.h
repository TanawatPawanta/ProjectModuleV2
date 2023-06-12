/*
 * PIDController.h
 *
 *  Created on: May 30, 2023
 *      Author: tanawatp
 */

#ifndef INC_PIDCONTROLLER_H_
#define INC_PIDCONTROLLER_H_
#include "main.h"
#include "math.h"
#include "arm_math.h"
#include "KalmanFilterV2.h"
#include "ReadEncoderV2.h"
#include "QuinticTrajectory.h"
#include "tim.h"

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	int32_t U;
	int32_t Delta_U;
	int32_t U_minus;
	int32_t Error;
	int32_t Error_minus;
	int32_t Error_minus2;
	int8_t MotorDir;
	float32_t tolerance;
	int8_t ESS;
	int8_t IsSteady;

}PID;

void PIDSetup(PID* temp,float32_t Ki, float32_t Kp, float32_t Kd,float32_t tolerance);
void PIDRun(PID* temp, float32_t SensoRead, float32_t Ref);
void CascadeLoop(PID* Pos, PID* Velo, float32_t PosFeedback, float32_t VeloFeedback, QuinticTraj* TrajReference, float32_t tolerance);
void MotorWrite();

#endif /* INC_PIDCONTROLLER_H_ */
