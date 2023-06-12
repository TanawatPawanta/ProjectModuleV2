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
#include "ModBusRTU.h"
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
	uint16_t HomeCount;
	uint32_t HomePosOffset;
	int8_t BaseMode;
	int8_t ControllerEnable;
	int8_t	JoyEnable;
	uint8_t testDummy;

}OperationVar;

typedef enum
{
	Init,
	PreHoming,
	Homing,
	Buffer,
	Home_Ok,
	PointMode,
	TrayMode,
	PreProcess,
//	TrajectoryCal,
	ControlLoop,
	GripperWaiting,
	WaitingHome,
}OperationState;

void SetHome(OperationVar* temp);
void RunX_Axis(int32_t XFinalPos, uint16_t velo, uint16_t accel);
#endif /* INC_STORAGE_H_ */
