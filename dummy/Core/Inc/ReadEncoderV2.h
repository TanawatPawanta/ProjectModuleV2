/*
 * ReadEncoder.h
 *
 *  Created on: May 19, 2023
 *      Author: tanawatp
 */

#ifndef INC_READENCODERV2_H_
#define INC_READENCODERV2_H_

#include "main.h"

typedef struct QEI
{
	uint64_t data[2];
	uint64_t timestamp[2];

	float QEIPosition;
	float QEIVelocity;
}QEIStructureTypedef;




uint64_t micros();
void QEIEncoderPositionVelocity_Update();



#endif /* INC_READENCODERV2_H_ */
