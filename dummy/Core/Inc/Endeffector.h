/*
 * Endeffector.h
 *
 *  Created on: May 18, 2023
 *      Author: napat
 */

#ifndef INC_ENDEFFECTOR_H_
#define INC_ENDEFFECTOR_H_

#include "i2c.h"
#include "arm_math.h"
// Salve Address
#define SlaveAddress		(0x15 << 1)

// Soft reset
#define SortReset_seq1		0x00
#define SortReset_seq2		0xFF
#define SortReset_seq3		0x55
#define SortReset_seq4		0xAA

// Emergency
#define Emergency_Trigger	0xF1 // X can be anything between 0xF0 - 0xFF (assign 0xF1)
#define OutEmergency_seq1	0xE5
#define OutEmergency_seq2	0x7A
#define OutEmergency_seq3	0xFF
#define OutEmergency_seq4	0x81

// Test Mode
#define TestMode_Command	0x01
#define TestMode_Close		0x00 // On mode : X can be anything except 0

// Gripper
#define RunMode_Command		0x10
#define RunMode_On			0x13
#define RunMode_Off			0x8C
#define Gripper_PickUp		0x5A
#define Gripper_PlaceDown	0x69


// Read Current Status
extern uint8_t EndEffectorData;

// Check status end effector
extern uint8_t checkError;
extern uint8_t checkEmergency;
extern uint8_t checkTestMode;
extern uint8_t checkRunMode;
extern uint8_t checkGripperStatus;

extern uint8_t returnStatus;
extern uint8_t mode;
//typedef enum{
//	idle,
//	test,
//	run,
//	emergency
//}Protocolstate;

// Read
void CheckStatusEndEffector();
void EndEffectorUpdate();

// Write
void EndEffectorSoftReset();
void EndEffectorEmergencyTrigger();
void EndEffectorEmergencyOut();
void EndEffectorTestModeOn();
void EndEffectorTestModeOff();
void EndEffectorGripperModeOn();
//int EndEffectorPickUp();
void EndEffectorPickUp();
void EndEffectorPlaceDown();
void EndEffectorGripperModeOff();
void Demo();

#endif /* INC_ENDEFFECTOR_H_ */
