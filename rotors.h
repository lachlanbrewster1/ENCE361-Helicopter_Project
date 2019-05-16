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
#define KI_Y   100
#define KD_Y   20

#define KP_M   50
#define KI_M   20
#define KD_M   5

#define SCALE   100




extern controller alt;
extern controller yaw;

extern uint16_t main_duty;
extern uint16_t secondary_duty;


void
doControl(uint16_t frequency);


#endif /* ROTORS_H_ */
