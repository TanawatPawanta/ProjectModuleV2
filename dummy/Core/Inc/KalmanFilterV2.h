/*
 * KalmanFilterV2.h
 *
 *  Created on: May 19, 2023
 *      Author: tanawatp
 */

#ifndef INC_KALMANFILTERV2_H_
#define INC_KALMANFILTERV2_H_

#include "main.h"
#include "arm_math.h"


typedef struct KalmanFilterVariable
{
	float32_t x_hat[3];  	// State estimate vector [x, dx, d2x]
	float32_t P[9];      	// Covariance matrix
	float32_t Q[9];      	// Process noise covariance matrix
	float R ;   	// Measurement noise variance
	float var_Q; 	// measurement variance
	float32_t z ;   		// Measurement value
	//Matrix
	float32_t A[3 * 3] ;
	float32_t B[3];
//	float32_t C[1 * 3] = {
//			0, 1, 0};
	float32_t C[3];
	float32_t D ;
	float32_t G[3] ;
	float32_t GT[3];
	float32_t I[9];//Identity
	// Intermediate matrices and vectors
	float32_t x_hat_minus[3];
	float32_t P_minus[3 * 3];
	float32_t S[1 * 1];
	float32_t K[3 * 1];
	float32_t temp3x3A[3 * 3];//9
	float32_t temp3x3B[3 * 3];//9
	float32_t temp3x1[3 * 1];//3
	float32_t temp1x3[3 * 1];//3
	float32_t temp1x1;//1

}Kalman;

void InitKalmanStruct(Kalman* KF,float32_t q,float32_t r);
void kalman_filter();

#endif /* INC_KALMANFILTERV2_H_ */
