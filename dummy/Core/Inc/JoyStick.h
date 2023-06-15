/*
 * joystick.h
 *
 *  Created on: May 4, 2023
 *      Author: napat
 */

#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_

#include "stm32f4xx_hal.h"
#include "main.h"

struct joystick
{
	int16_t X;
	int16_t Y;
	uint16_t status;

	int16_t B1Log;
	int16_t B2Log;
	int16_t B3Log;
};
struct position
{
	int16_t P1[2];
	int16_t P2[2];
	int16_t P3[2];
};

typedef enum
{
	TrayP1,
	TrayP2,
	TrayP3,
	TrayP4,
}JoyState;

extern struct joystick Joy;
extern struct position Pick;
extern struct position Place;

extern uint32_t VR[2];
extern int8_t flag;
extern int16_t counter;

extern uint8_t speed;
extern uint8_t StateSpeed;
extern uint8_t enableX;

void CheckJoystick();
int CheckButton();

#endif /* INC_JOYSTICK_H_ */
