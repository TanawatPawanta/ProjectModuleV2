/*
 * joystick.c
 *
 *  Created on: May 4, 2023
 *      Author: napat
 */
#include "joystick.h"

struct joystick Joy = {0};

uint32_t VR[2] = {0};
int8_t flag = 0;
int16_t counter = 0;

void CheckJoystick()
{
	if ((VR[0] >= 1800) && (VR[0] <= 2200) && (VR[1] >= 1800) && (VR[1] <= 2200))
	{
	Joy.X = 0;
	Joy.Y = 0;
   }

  // down
	else if ((VR[0] <= 100) && (VR[1] <= 2200))
	{
	Joy.X = 0;
	Joy.Y = -1;
   }
 // up
  else if ((VR[0] >= 3900) && (VR[1] >= 2100))
	{
	Joy.X = 0;
	Joy.Y = 1;
   }

  // left
  else if ((VR[0] <= 2100) && (VR[1] <= 1500))
	{
	Joy.X = -1;
	Joy.Y = 0;
   }

  // right
  else if ((VR[0] >= 1900) && (VR[1] >= 2100))
   {
	Joy.X = 1;
	Joy.Y = 0;
   }
}

int CheckButton()
{
	// Button 1
		if(Joy.B1Log == 0 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0){
			Joy.status = 1;
			Joy.B1Log = 1;
		}
		// Button 2
		else if(Joy.B2Log ==  0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0){
			Joy.status = 2;
			Joy.B2Log = 1;
	//		UpdatePosition();

		}
		// Button 3
		else if(Joy.B3Log ==  0 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == 0){
			Joy.status = 3;
			Joy.B3Log = 1;
		}

		// Check log
		if(Joy.B1Log == 1 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)){
			Joy.status = 0;
			Joy.B1Log = 0;
			// start
		}
		else if(Joy.B2Log == 1 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)){
			Joy.status = 0;
			Joy.B2Log = 0;
		}
		else if(Joy.B3Log == 1 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)){
			Joy.status = 0;
			Joy.B3Log = 0;
		}

	return Joy.status;
}

