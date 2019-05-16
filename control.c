/*
 * control.c
 *
 *  Created on: 14/05/2019
 *      Author: bsl28
 */

#include <stdint.h>
#include "control.h"

int32_t proportional (controller controlled)
{
    int32_t error = controlled.desired - controlled.actual;
    return error * controlled.Kp;
}

int32_t integral (controller controlled, uint16_t frequency)
{
    int32_t error = controlled.desired - controlled.actual;
    controlled.error_integrated += (2 * error + frequency) / 2 / frequency;
    return controlled.error_integrated * controlled.Ki;
}

int32_t derivative (controller controlled, uint16_t frequency)
{
    int32_t error = controlled.desired - controlled.actual;
    int32_t control = (error - controlled.error_previous) * frequency;
    controlled.error_previous = error;
    return control * controlled.Kd;
}

int32_t PID (controller controlled, uint16_t frequency)
{
    return proportional(controlled) + integral(controlled, frequency) + derivative(controlled, frequency);
}
