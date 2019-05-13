/*
 * control.c
 *
 *  Created on: 14/05/2019
 *      Author: bsl28
 */

#include <stdint.h>
#include "control.h"


static int32_t integrated_error = 0;
static int32_t error_previous = 0;

int32_t proportional (int32_t desired, int32_t actual, uint16_t Kp)
{
    int32_t error = desired - actual;
    return error * Kp;
}

int32_t integral (int32_t desired, int32_t actual, uint16_t Ki, uint16_t frequency)
{
    int32_t error = desired - actual;
    integrated_error += (2 * error + frequency) / 2 / frequency;
    return integrated_error * Ki;
}

int32_t derivative (int32_t desired, int32_t actual, uint16_t Kd, uint16_t frequency)
{
    int32_t error = desired - actual;
    int32_t control = (error - error_previous) * frequency;
    error_previous = error;
    return control * Kd;
}

int32_t PID (int32_t desired, int32_t actual, uint16_t Kp, uint16_t Ki, uint16_t Kd, uint16_t frequency)
{
    return proportional(desired, actual, Kp) + integral(desired, actual, Ki, frequency) + derivative(desired, actual, Kd, frequency);
}
