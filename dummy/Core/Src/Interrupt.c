/*
 * Interrupt.c
 *
 *  Created on: May 27, 2023
 *      Author: tanawatp
 */
#include "Interrupt.h"
#include "ReadEncoderV2.h"
#include "QuinticTrajectory.h"
#include "KalmanFilterV2.h"
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;


extern  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
extern QEIStructureTypedef QEIData;
extern ReadEncoder ReadEncoderParam;
extern QuinticTraj QuinticVar;
extern Kalman KF;
extern float32_t ZEstimateVelocity;

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){ //get time period
//	if(htim->Instance == TIM5)
//	{
//		ReadEncoderParam._micros += UINT32_MAX;
//	}
//	else if(htim == &htim3)
//	{
//		QEIData.data[0] = __HAL_TIM_GET_COUNTER(&htim2);
//		QEIData.QEIVelocity = ((QEIData.data[0]-QEIData.data[1]) * 1000000.0*60.0)/(0.001*8192.0);
//		KF.z = QEIData.QEIVelocity;
//		kalman_filter();
//		ZEstimateVelocity = KF.x_hat[1];
//		QuinticRun(&QuinticVar,0.001);
//		QEIData.data[1] = QEIData.data[0];
//		ReadEncoderParam.Pulse_Compare = ReadEncoderParam.MotorSetDuty * 10;
//		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,ReadEncoderParam.Pulse_Compare);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, ReadEncoderParam.DIR);
//	}
//}
