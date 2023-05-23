/*
 * Trajectory.c
 *
 *  Created on: May 20, 2023
 *      Author: tanawatp
 */
#include "Trajectory.h"
#include "math.h"

void SetTrajectoryConstrainAndInit(Trajectory* Tj, float32_t  vmax,float32_t  amax)
{
	Tj->complete = 1;
	Tj->v_max = vmax;
	Tj->a_max = amax;
	Tj->final_pos = 800;
	Tj->start_pos = 0;
}

void TrajectoryGenerator(Trajectory* Tj)
{
	if(Tj->final_pos == Tj->start_pos)
	{
		Tj->complete = 2;
	}
	else
	{
		Tj->complete = 0;
		Tj->displacement = Tj->final_pos - Tj->start_pos;
		float32_t ncr = 1.0;
		if(Tj->displacement < 0)
		{
			ncr = -1.0;

		}

		Tj->displacement = fabs(Tj->displacement);

		//Time and Profile calculate
		if(sqrtf(Tj->displacement/Tj->a_max) < Tj->v_max/Tj->a_max)
		{
			Tj->TimeAcc = sqrtf(Tj->displacement/Tj->a_max);
			Tj->TotalTime = 2.0*Tj->TimeAcc;
			Tj->profile = 0;	//Triangle
		}
		else
		{
			Tj->TimeAcc = Tj->v_max/Tj->a_max ;
			Tj->TotalTime = Tj->TimeAcc + Tj->displacement/Tj->v_max ;
			Tj->profile = 1;	//Trapizoid
		}
		Tj->a_max = ncr*Tj->a_max;
	}


}

void TrajectoryEvaluator(Trajectory* Tj,float64_t time)
{
	switch(Tj->profile)
	{
	case 0:	//Triangle
		if( time <= Tj->TimeAcc)
		{
			Tj->current_pos = Tj->start_pos + 0.5*Tj->a_max*pow(time,2);
			Tj->current_velo = Tj->a_max*time;
			Tj->current_acc = Tj->a_max;
		}
		else
		{
			Tj->current_pos =
					Tj->start_pos + Tj->a_max*(Tj->TotalTime*Tj->TimeAcc + Tj->TotalTime*time - 0.5*pow(Tj->TotalTime,2)- pow(Tj->TimeAcc,2)-0.5*pow(time,2));
			Tj->current_velo = Tj->a_max*Tj->TotalTime - Tj->a_max*time;
			Tj->current_acc = -Tj->a_max;
		}
		break;

	case 1:	//Trapizoid
		if(time <= Tj->TimeAcc)
		{
			Tj->current_pos = Tj->start_pos + 0.5*Tj->a_max*pow(time,2);
			Tj->current_velo = Tj->a_max*time;
			Tj->current_acc = Tj->a_max;
		}
		else if ((Tj->TimeAcc < time) && (time < Tj->TotalTime - Tj->TimeAcc))
		{
			Tj->current_pos = Tj->start_pos + 0.5*Tj->a_max*pow(Tj->TimeAcc,2) + Tj->a_max*Tj->TimeAcc*(time - Tj->TimeAcc);
			Tj->current_velo = Tj->a_max*Tj->TimeAcc;
			Tj->current_acc = 0;
		}
		else if ((Tj->TotalTime - Tj->TimeAcc < time)&&(time < Tj->TotalTime))
		{
			Tj->current_pos =
				Tj->start_pos + Tj->a_max*(Tj->TotalTime*Tj->TimeAcc + Tj->TotalTime*time - 0.5*pow(Tj->TotalTime,2)- pow(Tj->TimeAcc,2)-0.5*pow(time,2));
			Tj->current_velo = Tj->a_max*Tj->TotalTime - Tj->a_max*time;
			Tj->current_acc = -Tj->a_max;
		}
		break;
	}
	if(time >= Tj->TotalTime)
	{
		Tj->complete = 2;
		Tj->start_pos = Tj->final_pos;
		Tj->current_pos = Tj->final_pos;
		Tj->current_velo = 0;
		Tj->current_acc = 0;
	}
}
