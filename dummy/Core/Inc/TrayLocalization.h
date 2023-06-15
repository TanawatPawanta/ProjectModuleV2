/*
 * TrayLocalization.h
 *
 *  Created on: May 31, 2023
 *      Author: tanawatp127.4
 */

#ifndef INC_TRAYLOCALIZATION_H_
#define INC_TRAYLOCALIZATION_H_
#include "main.h"
#include "math.h"
#include "arm_math.h"
typedef struct
{
	float32_t Edge1_X;
	float32_t Edge1_Y;

	float32_t Edge2_X;
	float32_t Edge2_Y;

	uint16_t ForBaseOriginX;
	uint16_t ForBaseOriginY;
	uint16_t ForBaseOrientation;

	float32_t Holes_X[9];
	float32_t Holes_Y[9];
	int8_t SetTray;
	//int8_t Flag;
	float32_t angle;
}Tray;

void TraySetup(Tray* temp, float32_t x1, float32_t y1, float32_t x2, float32_t y2);
void TrayLocalization(Tray* temp);


#endif /* INC_TRAYLOCALIZATION_H_ */
