/*
 * KalmanFilterV2.c
 *
 *  Created on: May 19, 2023
 *      Author: tanawatp
 */
#include "arm_math.h"
#include "KalmanFilterV2.h"

extern Kalman KF;

extern arm_matrix_instance_f32 mat_A, mat_x_hat, mat_x_hat_minus, mat_B, mat_u, mat_GT, mat_G,eye;
extern arm_matrix_instance_f32 mat_P, mat_P_minus, mat_Q;
extern arm_matrix_instance_f32 mat_C, mat_R, mat_S, mat_K;
extern arm_matrix_instance_f32 mat_temp3x3A,mat_temp3x3B, mat_temp3x1,mat_temp1x3, mat_temp1x1;

void InitKalmanStruct(Kalman* KF,float q,float r)
{
	KF->R = r;
	KF->var_Q = q;
	KF->D = 0;
	float32_t a[9] = {
			1, 0.001, 0.0000005,
			0, 1, 0.001,
			0, 0, 1
	};
	float iden[9] = {
			1, 0, 0,
			0, 1, 0,
			0, 0, 1
	};
	int i;
	for(i=0;i<9;i++)
	{
		KF->A[i] = a[i];
		KF->I[i] = iden[i];
		KF->P[i] = 0;
	}
	float32_t b[3] = {
			0, 0, 0
	};
	float32_t c[3] = {
			0, 1, 0
	};
//	float32_t x[3] = {
//			0, 0, 0
//	};
//	float32_t p[3] = {
//				0, 0, 0
//		};
	float32_t g[3] = {
			0.001 * 0.001 * 0.001 / 6,
			0.0000005,
			0.001
	};
//	float32_t g[3] = {
//			0.0000005,
//			0.001,
//			0.0
//	};
	for(i=0;i<3;i++)
	{
		KF->B[i] = b[i];
		KF->C[i] = c[i];
		KF->G[i] = g[i];
		KF->x_hat[i] = 0;
	}
}

void kalman_filter()
{
    //Process model: x_hat_minus = mat_A*x_hat
    arm_mat_mult_f32(&mat_A, &mat_x_hat, &mat_x_hat_minus);			//A*X
    // If an input control vector is used:
    // arm_mat_mult_f32(&mat_B, &mat_u, &mat_temp3x3A);
    // arm_mat_add_f32(&mat_x_hat_minus, &mat_temp3x3A, &mat_x_hat_minus);

    // Predict covariance matrix: P_minus = A * P * A^T + Q
    arm_mat_trans_f32(&mat_A, &mat_temp3x3A);					    //AT
    arm_mat_mult_f32(&mat_A, &mat_P, &mat_temp3x3B);			    //A*P
    arm_mat_mult_f32(&mat_temp3x3B, &mat_temp3x3A, &mat_P_minus);	//A*P*AT
    arm_mat_trans_f32(&mat_G, &mat_GT);								//GT
    arm_mat_mult_f32(&mat_G, &mat_GT, &mat_Q);						//G*GT
    arm_mat_scale_f32(&mat_Q,KF.var_Q, &mat_Q);						//G*GT*var_q
    arm_mat_add_f32(&mat_P_minus, &mat_Q, &mat_P_minus);			//A * P * A^T + Q

    // Calculate innovation covariance: S = C * P_minus * C^T + R
    arm_mat_mult_f32(&mat_C, &mat_P_minus, &mat_temp1x3);			//C*P
    arm_mat_trans_f32(&mat_C, &mat_temp3x1);						//CT
    arm_mat_mult_f32(&mat_temp1x3, &mat_temp3x1, &mat_temp1x1);		//C*P*C^T
    arm_mat_add_f32(&mat_temp1x1, &mat_R, &mat_S);					//C*P*C^T + R

    //==> Calculate Kalman gain: K = P_minus * C^T * S^(-1)
    arm_mat_inverse_f32(&mat_S, &mat_temp1x1);						//inv(S)
    arm_mat_mult_f32(&mat_P_minus, &mat_temp3x1, &mat_temp3x3A);	//P*CT
    arm_mat_mult_f32(&mat_temp3x3A, &mat_temp1x1, &mat_K);			//P*CT*inv(S)

    //==> Update state estimate: x_hat = x_hat_minus + K * (z - C * x_hat_minus)
    arm_mat_mult_f32(&mat_C, &mat_x_hat_minus, &mat_temp1x1);		//C*X
    //float32_t innovation = z - temp1x1;								//Z-C*X
    arm_mat_scale_f32(&mat_K, KF.z - KF.temp1x1, &mat_temp3x1);			//K*(Z-C*X)
    arm_mat_add_f32(&mat_x_hat_minus, &mat_temp3x1, &mat_x_hat);	//X - K*(Z-C*X) ========> X estimate

    // Update covariance matrix: P = (I - K * C) * P_minus
    arm_mat_mult_f32(&mat_K, &mat_C, &mat_temp3x3B);				//K*C
    arm_mat_sub_f32(&eye, &mat_P_minus, &mat_temp3x3A);				//I - K*C
    arm_mat_mult_f32(&mat_temp3x3A, &mat_P_minus, &mat_P);			//(I - K*C)*P ===========> P estimate
}


