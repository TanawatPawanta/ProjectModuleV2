/*
 * QuinticTrajectory.h
 *
 *  Created on: May 25, 2023
 *      Author: tanawatp
 */

#ifndef INC_QUINTICTRAJECTORY_H_
#define INC_QUINTICTRAJECTORY_H_
#include "main.h"
#include "math.h"
#include "arm_math.h"
typedef enum
{
	Ready,
	PreCal,
	Run,
}QuicticState;

typedef struct
{
	int32_t start_pos;
	int32_t final_pos;

	int32_t displacement;
	int32_t current_pos;
	int32_t current_velo;
	int32_t current_acc;
	int32_t v_max;
	int32_t a_max;
	int8_t Dir;
	float32_t coeff[6];
	float32_t TotalTime;
	float32_t time;
	float32_t timeAcc;
	float32_t timeVelo;
	QuicticState State;
}QuinticTraj;

void QuinticSetup(QuinticTraj* temp, float32_t vmax, float32_t amax);
void QuinticGenerator(QuinticTraj* temp,int8_t ess);
void QuinticEvaluator(QuinticTraj* temp);
void QuinticRun(QuinticTraj* temp, int8_t ess, float32_t dt);

#endif /* INC_QUINTICTRAJECTORY_H_ */
