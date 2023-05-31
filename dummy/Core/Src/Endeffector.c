///*
// * Endeffector.c
// *
// *  Created on: May 18, 2023
// *      Author: napat
// */
//#include "Endeffector.h"
////#include "i2c.h"
//#include "arm_math.h"
//#include "main.h"
//// Read Current Status
//uint8_t EndEffectorData;
//
//// Check status end effector
//uint8_t checkError;
//uint8_t checkEmergency;
//uint8_t checkTestMode;
//uint8_t checkRunMode;
//uint8_t checkGripperStatus;
//
//// return check flag
//uint8_t returnStatus;
//uint8_t mode;
//void CheckStatusEndEffector(){
////	EndEffectorData
//	checkError = (EndEffectorData >> 5) & 0b00000111;
//	checkEmergency  = (EndEffectorData >> 4) & 0b00000001;
//	checkTestMode = (EndEffectorData >> 3) & 0b00000001;
//	checkRunMode = (EndEffectorData >> 2) & 0b00000001;
//	checkGripperStatus = EndEffectorData & 0b00000011;
//}
//
//// read status from end effector
//void EndEffectorUpdate(){
//	if(hi2c1.State == HAL_I2C_STATE_READY){
//		HAL_I2C_Master_Receive(&hi2c1, SlaveAddress, &EndEffectorData, 1, HAL_MAX_DELAY);
//	}
//    HAL_Delay(100);
//	CheckStatusEndEffector();
//}
//
//// write
//void EndEffectorSoftReset(){
//	static uint8_t SoftReset[4] = {SortReset_seq1, SortReset_seq2, SortReset_seq3, SortReset_seq4};
////	EndEffectorUpdate();
//	if(hi2c1.State == HAL_I2C_STATE_READY){
//		HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, SoftReset, 4, HAL_MAX_DELAY);
//	}
//    HAL_Delay(100);
//	EndEffectorUpdate();
//}
//
//void EndEffectorEmergencyTrigger(){
//	static uint8_t EmergencyTrigger[1] = {Emergency_Trigger};
//	if(hi2c1.State == HAL_I2C_STATE_READY){
//		HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, EmergencyTrigger, 1, HAL_MAX_DELAY);
//	}
//    HAL_Delay(10);
//	EndEffectorUpdate();
//
//	// if emergency still off
//	if(checkEmergency == 0){
//		EndEffectorSoftReset();
//	}
//}
//
//void EndEffectorEmergencyOut(){
//	static uint8_t EmergencyOut[4] = {OutEmergency_seq1, OutEmergency_seq2, OutEmergency_seq3, OutEmergency_seq4};
//	if(hi2c1.State == HAL_I2C_STATE_READY){
//		HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, EmergencyOut, 4, HAL_MAX_DELAY);
//	}
//    HAL_Delay(10);
//	EndEffectorUpdate();
//
//	// if emergency still on
//	if(checkEmergency == 1){
//		EndEffectorSoftReset();
//	}
//}
//
//void EndEffectorTestModeOn(){
//	static uint8_t TestModeOn[2] = {0x01, 0x11};
//	if(hi2c1.State == HAL_I2C_STATE_READY){
////	HAL_Delay(100);
//		HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, TestModeOn, 2, HAL_MAX_DELAY);
//	}
//    HAL_Delay(10);
//	EndEffectorUpdate();
//}
//
//void EndEffectorTestModeOff(){
//	static uint8_t TestModeOff[2] = {TestMode_Command, TestMode_Close};
//	if(hi2c1.State == HAL_I2C_STATE_READY){
//		HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, TestModeOff, 2, HAL_MAX_DELAY);
//	}
//    HAL_Delay(10);
//	EndEffectorUpdate();
//
//	// if test mode still on
//	if(checkTestMode == 1){
//		EndEffectorSoftReset();
//	}
//}
//
//void EndEffectorGripperModeOn(){
//	static uint8_t GripperOn[] = {RunMode_Command, RunMode_On};
//	if(hi2c1.State == HAL_I2C_STATE_READY){
//		HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, GripperOn, 2, HAL_MAX_DELAY);
//	}
//    HAL_Delay(10);
//	EndEffectorUpdate();
//}
//
//// return 1 picked, return 0 not pick
////int EndEffectorPickUp(){
////	EndEffectorTestModeOn();
////	static uint8_t PickUp[] = {RunMode_Command, Gripper_PickUp};
////	EndEffectorUpdate();
//////    HAL_Delay(100);
////	if(checkGripperStatus == 0b00){ // if gripper status = not pick
////		while(checkGripperStatus != 0b11){ // if gripper status = picked
////			HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, PickUp, 2, HAL_MAX_DELAY);
//////		    HAL_Delay(100);
////			EndEffectorUpdate();
//////		    HAL_Delay(100);
////		}
////		returnStatus = 1;
////		return returnStatus;
////	}
////	returnStatus = 0;
////	return returnStatus;
////}
//
//void EndEffectorPickUp(){
//	static uint8_t PickUp[] = {RunMode_Command, Gripper_PickUp};
//	if(hi2c1.State == HAL_I2C_STATE_READY){
//		HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, PickUp, 2, HAL_MAX_DELAY);
//	}
//    HAL_Delay(10);
//	EndEffectorUpdate();
//}
//
//
//void EndEffectorPlaceDown(){
////	EndEffectorTestModeOn();
//	static uint8_t PlaceDown[] = {RunMode_Command, Gripper_PlaceDown};
//	EndEffectorUpdate();
//    HAL_Delay(10);
//	if(checkGripperStatus == 0b11){ // if gripper status = picked
//		while(checkGripperStatus != 0b00){ // if gripper status = not pick (done pick & place)
//			HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, PlaceDown, 2, HAL_MAX_DELAY);
//		    HAL_Delay(10);
//			EndEffectorUpdate();
//		    HAL_Delay(10);
//		}
//	}
////	static uint8_t PlaceDown[] = {RunMode_Command, Gripper_PlaceDown};
////	HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, PlaceDown, 2, HAL_MAX_DELAY);
////	EndEffectorUpdate();
//}
//
//void EndEffectorGripperModeOff(){
//	static uint8_t GripperOff[] = {RunMode_Command, RunMode_Off};
//	if(hi2c1.State == HAL_I2C_STATE_READY){
//		HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, GripperOff, 2, HAL_MAX_DELAY);
//	}
//	HAL_Delay(10);
//	EndEffectorUpdate();
//}
//
//
//void Demo(){
//	switch(mode){
//	case 0:
//		EndEffectorUpdate();
//		break;
//	case 1:
//		EndEffectorSoftReset();
//		break;
//	case 2:
//		EndEffectorEmergencyTrigger();
//		break;
//	case 3:
//		EndEffectorEmergencyOut();
//		break;
//	case 4:
//		EndEffectorTestModeOn();
//		break;
//	case 5:
//		EndEffectorTestModeOff();
//		break;
//	case 6:
//		EndEffectorGripperModeOn();
//		break;
//	case 7:
//		EndEffectorPickUp();
//		break;
//	case 8:
//		EndEffectorPlaceDown();
//		break;
//	case 9:
//		EndEffectorGripperModeOff();
//		break;
//	}
//}
//
