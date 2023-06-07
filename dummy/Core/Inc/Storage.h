/*
 * Storage.h
 *
 *  Created on: May 19, 2023
 *      Author: tanawatp
 */

#ifndef INC_STORAGE_H_
#define INC_STORAGE_H_

typedef struct
{
	uint16_t waitTime;
	uint8_t CmpltLoop;

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

#endif /* INC_STORAGE_H_ */
