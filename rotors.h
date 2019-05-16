/*
 * rotors.h
 *
 *  Created on: 14/05/2019
 *      Author: bsl28
 */

#include "control.h"

#ifndef ROTORS_H_
#define ROTORS_H_

#define KP_Y   150
#define KI_Y   50
#define KD_Y   20

#define KP_M   50
#define KI_M   20
#define KD_M   5

#define SCALE   100




static controller alt = {0,0,KP_M,KI_M,KD_M,0,0};
static controller yaw = {0,0,KP_Y,KI_Y,KD_Y,0,0};

static uint16_t main_duty = 0;
static uint16_t secondary_duty = 0;


void
doControl(uint16_t frequency);


#endif /* ROTORS_H_ */
