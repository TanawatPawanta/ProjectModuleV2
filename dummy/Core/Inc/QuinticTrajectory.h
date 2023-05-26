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
	float32_t start_pos;
	float32_t final_pos;
	float32_t displacement;
	float32_t current_pos;
	float32_t current_velo;
	float32_t current_acc;
	float32_t v_max;
	float32_t a_max;
	float32_t coeff[6];
	float32_t TotalTime;
	float32_t time;
	float32_t timeAcc;
	float32_t timeVelo;
	QuicticState State;
}QuinticTraj;

void QuinticSetup(QuinticTraj* temp, float32_t vmax, float32_t amax);
void QuinticGenerator(QuinticTraj* temp);
void QuinticEvaluator(QuinticTraj* temp);
void QuinticRun(QuinticTraj* temp,float32_t dt);

#endif /* INC_QUINTICTRAJECTORY_H_ */
