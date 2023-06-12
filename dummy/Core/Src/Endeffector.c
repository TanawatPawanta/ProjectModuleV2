/*
 * Endeffector.c
 *
 *  Created on: Jun 12, 2023
 *      Author: panna
 */
#include "Endeffector.h"
#include "i2c.h"

uint8_t Stamp = 0;
uint16_t i = 0;
EndEffMode TestState;
uint8_t ReadData;

uint16_t count = 0;

uint8_t Error_Status;
uint8_t Emergency_Status;
uint8_t TestMode_Status;
uint8_t RunMode_Status;
uint8_t GripperMode_Status;

void Endeffector_Update_Status(){
	Error_Status = (ReadData >> 5) & 0b00000111;
	Emergency_Status = (ReadData >> 4) & 0b00000001;
	TestMode_Status = (ReadData >> 3) & 0b00000001;
	RunMode_Status = (ReadData >> 2) & 0b00000001;
	GripperMode_Status = ReadData & 0b00000011;
}

void Endeffector_Read_Status(){
	if(Stamp == 1 && hi2c1.State == HAL_I2C_STATE_READY){
		HAL_I2C_Master_Receive_IT(&hi2c1, SlaveAddr, &ReadData, sizeof(ReadData));
		count++;
	}

	static uint32_t timestamp = 0;
	while(i < 150){
		if(HAL_GetTick() >= timestamp){
			timestamp = HAL_GetTick() + 10;
			i++;
		}
	}

	Endeffector_Update_Status();
	Stamp = 0;
	i = 0;
}

void Endeffector_SoftReset(){
	static uint8_t CommandSeq[4] = {SoftReset_Seq1, SoftReset_Seq2, SoftReset_Seq3, SoftReset_Seq4};

	if(Stamp == 1 && hi2c1.State == HAL_I2C_STATE_READY){
		HAL_I2C_Master_Transmit_IT(&hi2c1, SlaveAddr, CommandSeq, sizeof(CommandSeq));
	}

	static uint32_t timestamp = 0;
	while(i < 150){
		if(HAL_GetTick() >= timestamp){
			timestamp = HAL_GetTick() + 10;
			i++;
		}
	}
	Endeffector_Read_Status();
	Stamp = 0;
	i = 0;
}

void Endeffector_EmergencyTrigger(){
	static uint8_t EmergencyCommand[1] = {EmergencyTrigger};

	if(Stamp == 1 && hi2c1.State == HAL_I2C_STATE_READY){
		HAL_I2C_Master_Transmit_IT(&hi2c1, SlaveAddr, EmergencyCommand, sizeof(EmergencyCommand));
	}

	static uint32_t timestamp = 0;
	while(i < 150){
		if(HAL_GetTick() >= timestamp){
			timestamp = HAL_GetTick() + 10;
			i++;
		}
	}

	Endeffector_Read_Status();
	Stamp = 0;
	i = 0;
}

void Endeffector_EmergencyExit(){
	static uint8_t EmergencySeq[4] = {EmergencyOut_Seq1, EmergencyOut_Seq2, EmergencyOut_Seq3, EmergencyOut_Seq4};

	if(Stamp == 1 && hi2c1.State == HAL_I2C_STATE_READY){
		HAL_I2C_Master_Transmit_IT(&hi2c1, SlaveAddr, EmergencySeq, sizeof(EmergencySeq));
	}

	static uint32_t timestamp = 0;
	while(i < 150){
		if(HAL_GetTick() >= timestamp){
			timestamp = HAL_GetTick() + 10;
			i++;
		}
	}

	Endeffector_Read_Status();
	Stamp = 0;
	i = 0;
}

void Endeffector_TestModeOn(){
	static uint8_t TestModeON[2] = {TestMode_Command, TestMode_On};

	if(Stamp == 1 && hi2c1.State == HAL_I2C_STATE_READY){
		HAL_I2C_Master_Transmit_IT(&hi2c1, SlaveAddr, TestModeON, sizeof(TestModeON));
	}

	static uint32_t timestamp = 0;
	while(i < 150){
		if(HAL_GetTick() >= timestamp){
			timestamp = HAL_GetTick() + 10;
			i++;
		}
	}

	Endeffector_Read_Status();
	Stamp = 0;
	i = 0;
}

void Endeffector_TestModeOff(){
	static uint8_t TestModeOff[2] = {TestMode_Command, TestMode_Off};

	if(Stamp == 1 && hi2c1.State == HAL_I2C_STATE_READY){
		HAL_I2C_Master_Transmit_IT(&hi2c1, SlaveAddr, TestModeOff, sizeof(TestModeOff));
	}

	static uint32_t timestamp = 0;
	while(i < 150){
		if(HAL_GetTick() >= timestamp){
			timestamp = HAL_GetTick() + 10;
			i++;
		}
	}

	Endeffector_Read_Status();
	Stamp = 0;
	i = 0;
}

void Endeffector_GripperModeOn(){
	static uint8_t GripperOn[2] = {GripperMode_Command, GripperMode_On};

	if(Stamp == 1 && hi2c1.State == HAL_I2C_STATE_READY){
		HAL_I2C_Master_Transmit_IT(&hi2c1, SlaveAddr, GripperOn, sizeof(GripperOn));
	}

	static uint32_t timestamp = 0;
	while(i < 150){
		if(HAL_GetTick() >= timestamp){
			timestamp = HAL_GetTick() + 10;
			i++;
		}
	}

	Endeffector_Read_Status();
	Stamp = 0;
	i = 0;
}

void Endeffector_GripperModeOff(){
	static uint8_t GripperOff[2] = {GripperMode_Command, GripperMode_Off};

	if(Stamp == 1 && hi2c1.State == HAL_I2C_STATE_READY){
		HAL_I2C_Master_Transmit_IT(&hi2c1, SlaveAddr, GripperOff, sizeof(GripperOff));
	}

	static uint32_t timestamp = 0;
	while(i < 150){
		if(HAL_GetTick() >= timestamp){
			timestamp = HAL_GetTick() + 10;
			i++;
		}
	}

	Endeffector_Read_Status();
	Stamp = 0;
	i = 0;
}

void Endeffector_PickUp(){
	static uint8_t PickUp[2] = {GripperMode_Command, GripperMode_PickUp};

	if(Stamp == 1 && hi2c1.State == HAL_I2C_STATE_READY){
		HAL_I2C_Master_Transmit_IT(&hi2c1, SlaveAddr, PickUp, sizeof(PickUp));
	}

	static uint32_t timestamp = 0;
	while(i < 150){
		if(HAL_GetTick() >= timestamp){
			timestamp = HAL_GetTick() + 10;
			i++;
		}
	}

	Endeffector_Read_Status();
	Stamp = 0;
	i = 0;
}

void Endeffector_PlaceDown(){
	static uint8_t PlaceDown[2] = {GripperMode_Command, GripperMode_PlaceDown};

	if(Stamp == 1 && hi2c1.State == HAL_I2C_STATE_READY){
		HAL_I2C_Master_Transmit_IT(&hi2c1, SlaveAddr, PlaceDown, sizeof(PlaceDown));
	}

	static uint32_t timestamp = 0;
	while(i < 150){
		if(HAL_GetTick() >= timestamp){
			timestamp = HAL_GetTick() + 10;
			i++;
		}
	}
	Endeffector_Read_Status();
	Stamp = 0;
	i = 0;
}

void TestMode(){
	switch(TestState){
	case Read_Status:
		Endeffector_Read_Status();
		break;
	case SoftReset:
		Endeffector_SoftReset();
		break;
	case TestModeOn:
		Endeffector_TestModeOn();
		break;
	case TestModeOff:
		Endeffector_TestModeOff();
		break;
	case GripperModeOn:
		Endeffector_GripperModeOn();
		break;
	case PickUp:
		Endeffector_PickUp();
		break;
	case PlaceDown:
		Endeffector_PlaceDown();
		break;
	case GripperModeOff:
		Endeffector_GripperModeOff();
		break;
	case GripperEmerTrigger:
		Endeffector_EmergencyTrigger();
		break;
	case GripperEmerExit:
		Endeffector_EmergencyExit();
		break;
	}
}
