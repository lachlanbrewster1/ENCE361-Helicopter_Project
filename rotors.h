/*
 * rotors.h
 *
 *  Created on: 14/05/2019
 *      Author: bsl28
 */

#include "control.h"

#ifndef ROTORS_H_
#define ROTORS_H_

#define KP_Y   350
#define KI_Y   280
#define KD_Y   50

#define KP_M   160
#define KI_M   140
#define KD_M   50

#define SCALE   100




extern controller alt;
extern controller yaw;

extern int32_t main_duty;
extern int32_t secondary_duty;


void
doControl(uint16_t frequency);


#endif /* ROTORS_H_ */
