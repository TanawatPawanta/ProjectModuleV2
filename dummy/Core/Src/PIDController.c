/*
 * PIDController.c
 *
 *  Created on: May 30, 2023
 *      Author: tanawatp
 */

#include "PIDController.h"

void PIDSetup(PID* temp,float32_t Kp, float32_t Ki, float32_t Kd, float32_t tolerance)
{
	temp->Kp = Kp;
	temp->Ki = Ki;
	temp->Kd = Kd;
	temp->tolerance = tolerance;
	temp->U = 0;
	temp->Delta_U = 0;
	temp->U_minus = 0;
	temp->Error = 0;
	temp->Error_minus = 0;
	temp->Error_minus2 = 0;
}

void PIDRun(PID* temp, float32_t Feedback, float32_t Ref)
{
	temp->Error = Ref - Feedback;
	if(temp->Error > 0)
	{
		temp->MotorDir = 0;
	}
	else if (temp->Error < 0)
	{
		temp->MotorDir = 1;
	}
	//U Update
	temp->Delta_U = (temp->Kp + temp->Ki + temp->Kd)*temp->Error
			  -(temp->Kp + 2*temp->Kd)*temp->Error_minus
			  +temp->Kd*temp->Error_minus2;
	temp->U = temp->Delta_U + temp->U_minus;
	temp->U_minus = temp->U;
	//Error Update
	temp->Error_minus2 = temp->Error_minus;
	temp->Error_minus = temp->Error;
}

void CascadeLoop(PID* Pos, PID* Velo, float32_t PosFeedback, float32_t VeloFeedback, QuinticTraj* TrajReference, float32_t tolerance)
{
//	if(fabs(TrajReference->current_pos - PosFeedback) > tolerance)
//	{
		PIDRun(Pos, PosFeedback, TrajReference->current_pos);
		float32_t veloRef = Pos->U + TrajReference->current_velo;
		PIDRun(Velo, VeloFeedback, veloRef);
//	}
//	else {
//		Velo->U = 0;
//	}
}
