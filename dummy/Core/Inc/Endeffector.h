/*
 * endeffector.h
 *
 *  Created on: Jun 12, 2023
 *      Author: panna
 */

#ifndef INC_ENDEFFECTOR_H_
#define INC_ENDEFFECTOR_H_

#include "i2c.h"

//Endeffector Address
#define SlaveAddr 				(0x15 << 1)

//soft reset sequence
#define SoftReset_Seq1 			0x00
#define SoftReset_Seq2 			0xFF
#define SoftReset_Seq3			0x55
#define SoftReset_Seq4			0xAA

//Emergency Mode
#define EmergencyTrigger		0xFF

#define EmergencyOut_Seq1		0xE5
#define EmergencyOut_Seq2		0x7A
#define EmergencyOut_Seq3		0xFF
#define EmergencyOut_Seq4		0x81

//Test Mode
#define TestMode_Command 		0x01
#define TestMode_On				0x10
#define TestMode_Off			0x00

//Gripper Mode
#define GripperMode_Command		0x10
#define	GripperMode_On			0x13
#define GripperMode_Off			0x8C
#define GripperMode_PickUp		0x5A
#define GripperMode_PlaceDown	0x69

//Send Stamp
extern uint8_t Stamp;

//Read Current Status
extern uint16_t ReadData;

extern uint16_t count;

extern uint8_t Error_Status;
extern uint8_t Emergency_Status;
extern uint8_t TestMode_Status;
extern uint8_t RunMode_Status;
extern uint8_t GripperMode_Status;

typedef enum{
	ReadStatus,
	SoftReset,
	TestOn,
	TestOff,
	GripOn,
	PickUp,
	PlaceDown,
	GripOff,
	EmerTrig,
	EmerExit
}TestState;

extern TestState testState;

void Endeffector_Read_Status();
void Endeffector_Update_Status();

void Endeffector_SoftReset();

void Endeffector_EmergencyTrigger();
void Endeffector_EmergencyExit();


void Endeffector_TestModeOn();
void Endeffector_TestModeOff();

void Endeffector_GripperModeOn();
void Endeffector_GripperModeOff();
void Endeffector_PickUp();
void Endeffector_PlaceDown();

void TestMode();
#endif /* INC_ENDEFFECTOR_H_ */
