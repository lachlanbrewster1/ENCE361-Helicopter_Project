/*
 * control.h
 *
 *  Created on: 14/05/2019
 *      Author: bsl28
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdint.h>


int32_t
proportional (int32_t desired, int32_t actual, uint16_t Kp);

int32_t
integral (int32_t desired, int32_t actual, uint16_t Ki, uint16_t frequency);

int32_t
derivative (int32_t desired, int32_t actual, uint16_t Kd, uint16_t frequency);

int32_t
PID (int32_t desired, int32_t actual, uint16_t Kp, uint16_t Ki, uint16_t Kd, uint16_t frequency);



#endif /* CONTROL_H_ */
