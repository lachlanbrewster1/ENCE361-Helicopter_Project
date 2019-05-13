/*
 * rotors.c
 *
 *  Created on: 14/05/2019
 *      Author: bsl28
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "pwm.h"
#include "height.h"
#include "yaw.h"
#include "rotors.h"
#include "control.h"






void
doControl(uint16_t desired_height, uint16_t desired_yaw, uint16_t frequency)
{
    int16_t new_desired_yaw = desired_yaw;
    if (desired_yaw - getYaw() > 180)
        new_desired_yaw -= 360;
    else if (desired_yaw - getYaw() < -180)
        new_desired_yaw += 360;

    uint16_t main_duty = PID(desired_height, calculateMeanHeight(), KP_M, KI_M, KD_M, frequency) / SCALE;
    uint16_t secondary_duty = PID(new_desired_yaw, getYaw(), KP_Y, KI_Y, KD_Y, frequency) / SCALE;

    if (main_duty > 80) main_duty = 80;
    if (main_duty < 20) main_duty = 20;

    if (secondary_duty > 80) secondary_duty = 80;
    if (secondary_duty < 20) secondary_duty = 20;

    if (heliState ==  LANDING && altitude < 5) {
        secondary_duty = 0;
        main_duty = 0;
        heliState = LANDED;
    }

    setDutyCycle(main_duty, MAIN_ROTOR);
    setDutyCycle(secondary_duty, SECONDARY_ROTOR);
}
