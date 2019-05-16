/*
 * helistates.h
 *
 *  Created on: 16/05/2019
 *      Author: bsl28
 */

#ifndef HELISTATES_H_
#define HELISTATES_H_


enum heli_states {INIT = 0, STARTUP, LANDED, LANDING, FLYING};


static uint8_t heli_state = INIT;

#endif /* HELISTATES_H_ */
