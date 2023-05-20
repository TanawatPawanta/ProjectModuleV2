/*
 * Trajectory.h
 *
 *  Created on: May 20, 2023
 *      Author: tanawatp
 */

#ifndef INC_TRAJECTORY_H_
#define INC_TRAJECTORY_H_

#include "main.h"
#include "math.h"
#include "arm_math.h"

typedef struct TrajectoryParameter
{
	float32_t start_pos;
	float32_t final_pos;
	float32_t displacement;
	float32_t  v_max;
	float32_t  a_max;
	float32_t current_pos;
	float32_t current_velo;
	float32_t current_acc;
	float32_t TotalTime;
	float32_t TimeAcc;
	int complete ;
	int profile;
}Trajectory;

void SetTrajectoryConstrainAndInit(Trajectory* Tj, float32_t vmax,float32_t amax);
void TrajectoryGenerator(Trajectory* Tj);
void TrajectoryEvaluator(Trajectory* Tj,float64_t time);


#endif /* INC_TRAJECTORY_H_ */
