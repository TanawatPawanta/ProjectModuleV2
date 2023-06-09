/*
 * Storage.h
 *
 *  Created on: May 19, 2023
 *      Author: tanawatp
 */

#ifndef INC_STORAGE_H_
#define INC_STORAGE_H_
#include "main.h"
#include "math.h"
#include "arm_math.h"
#include "tim.h"
typedef enum
{
	GoPick,
	GoPlace
}TaskState;

typedef struct
{
	uint16_t waitTime;
	uint8_t CmpltLoop;
	TaskState task;
	uint8_t holeInd;
	uint8_t ProxStop;
	uint8_t HomingKey;

}OperationVar;

typedef enum
{
	Init,
	Homing,
	Home_Ok,
	SetTray,
	PreProcess,
//	TrajectoryCal,
	ControlLoop,
	Waiting,
}OperationState;

void SetHome(OperationVar* temp);

#endif /* INC_STORAGE_H_ */
