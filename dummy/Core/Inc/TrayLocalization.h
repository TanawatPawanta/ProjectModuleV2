/*
 * TrayLocalization.h
 *
 *  Created on: May 31, 2023
 *      Author: tanawatp
 */

#ifndef INC_TRAYLOCALIZATION_H_
#define INC_TRAYLOCALIZATION_H_
#include "main.h"
#include "math.h"
#include "arm_math.h"
typedef struct
{
	uint16_t Edge1_X;
	uint16_t Edge1_Y;
	uint16_t Edge2_X;
	uint16_t Edge2_Y;
	uint16_t Holes_X[9];
	uint16_t Holes_Y[9];
	int8_t SetTray;
	int8_t Flag;
	uint16_t Translation[3];
	float32_t Rotation[9];
	float32_t Tranformation[16];
}Tray;

void TraySetup(Tray* temp, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void TrayLocalization(Tray* temp);


#endif /* INC_TRAYLOCALIZATION_H_ */
