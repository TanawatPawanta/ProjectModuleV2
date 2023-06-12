/*
 * QuinticTrajectory.c
 *
 *  Created on: May 25, 2023
 *      Author: tanawatp
 */
#include "QuinticTrajectory.h"

void QuinticSetup(QuinticTraj* temp, float32_t vmax, float32_t amax)
{
	temp->v_max = vmax;
	temp->a_max = amax;
	temp->State = Ready;
	//temp->final_pos = 300;
}
void QuinticGenerator(QuinticTraj* temp,int8_t ess)
{
	//temp->final_pos = temp->final_pos * 8192/120;
	temp->displacement = temp->final_pos - temp->start_pos;
	if(temp->displacement<0)
	{
		temp->Dir = 1;
	}
	else if (temp->displacement>0)
	{
		temp->Dir = 0;
	}
	temp->timeAcc = 0.5*sqrtf(23.094*fabs(temp->displacement)/temp->a_max);
	temp->timeVelo = 1.875*fabs(temp->displacement)/temp->v_max;
	temp->TotalTime = MAX(temp->timeAcc,temp->timeVelo);
	temp->coeff[0] = temp->start_pos;
	temp->coeff[1] = 0;
	temp->coeff[2] = 0;
	temp->coeff[3] = 10.0*temp->displacement/powf(temp->TotalTime,3);
	temp->coeff[4] = -15.0*temp->displacement/powf(temp->TotalTime,4);
	temp->coeff[5] = 6.0*temp->displacement/powf(temp->TotalTime,5);
}
void QuinticEvaluator(QuinticTraj* temp)
{
	float32_t time = temp->time;
	temp->current_pos = temp->coeff[0]
						+ temp->coeff[1]*time
						+ temp->coeff[2]*powf(time,2)
						+ temp->coeff[3]*powf(time,3)
						+ temp->coeff[4]*powf(time,4)
						+ temp->coeff[5]*powf(time,5);

	temp->current_velo = temp->coeff[1]
						+ 2.0*temp->coeff[2]*time
						+ 3.0*temp->coeff[3]*powf(time,2)
						+ 4.0*temp->coeff[4]*powf(time,3)
						+ 5.0*temp->coeff[5]*powf(time,4);

	temp->current_acc = 2.0*temp->coeff[2]
						+ 6.0*temp->coeff[3]*time
						+ 12.0*temp->coeff[4]*powf(time,2)
						+ 20.0*temp->coeff[5]*powf(time,3);
	if(time >= temp->TotalTime)
	{
		temp->start_pos = temp->final_pos;
		temp->current_pos = temp->start_pos;
		temp->current_velo = 0;
		temp->current_acc = 0;
	}
}
void QuinticRun(QuinticTraj* temp, int8_t ess, float32_t dt)
{
	switch(temp->State)
	{
	case Ready:

		if(temp->start_pos != temp->final_pos)
		{
			temp->State = PreCal;
		}
		break;
	case PreCal:
		temp->time = 0;
		QuinticGenerator(temp,ess);
		temp->State = Run;
		break;
	case Run:
		temp->time = temp->time + dt;
		QuinticEvaluator(temp);
		if(temp->time > temp->TotalTime)
		{
			temp->State = Ready;
		}
		break;
	}
}







