/*
 * control.h
 *
 *  Created on: 14/05/2019
 *      Author: bsl28
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdint.h>

typedef struct controller {
    uint16_t desired;
    uint16_t actual;
    uint8_t Kp;
    uint8_t Ki;
    uint8_t Kd;
    int16_t error_integrated;
    int16_t error_previous;
} controller;


int32_t
proportional (controller controlled);

int32_t
integral (controller controlled, uint16_t frequency);

int32_t
derivative (controller controlled, uint16_t frequency);

int32_t
PID (controller controlled, uint16_t frequency);

#endif /* CONTROL_H_ */
