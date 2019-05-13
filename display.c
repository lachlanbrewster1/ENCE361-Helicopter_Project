/*
 * display.c
 *
 *  Created on: 13/05/2019
 *      Author: lbr63
 */


#include "display.h"
#include "uart.h"
#include "OrbitOLED/OrbitOLEDInterface.h"

#include "main.c" // need to know all the current stats of heli, not sure on proper way

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


void
initialiseDisplay(void)
{
    OLEDInitialise();
    initialiseUSB_UART();
}


void
displayStatusOLED()
{
    char string[17];  // 16 characters across the display

    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf (string, sizeof(string), "Height = %3d%%", altitude);
    // Update line on display.
    OLEDStringDraw (string, 0, 0);

    uint16_t yaw_temp = yaw;
    usnprintf (string, sizeof(string), "Yaw = %3d deg", yaw_temp);
    // Update line on display.
    OLEDStringDraw (string, 0, 1);


    usnprintf (string, sizeof(string), "Main duty = %3d%%", main_duty);
    // Update line on display.
    OLEDStringDraw (string, 0, 2);

    usnprintf (string, sizeof(string), "Tail duty = %3d%%", secondary_duty);
    // Update line on display.
    OLEDStringDraw (string, 0, 3);

}

void
displayStatusUART(void)
{
    char string[31];


        int temp_yaw = yaw;
        int temp_desired_yaw = desired_yaw;


        usnprintf (string, sizeof(string), "yaw %d [%d]\r\n", temp_yaw, temp_desired_yaw);
        UARTSend(string);
        usnprintf (string, sizeof(string), "altitude %d%% [%d%%]\r\n", altitude, desired_altitude);
        UARTSend(string);
        usnprintf (string, sizeof(string), "main %d%% tail %d%%\r\n", main_duty, secondary_duty);
        UARTSend(string);
        switch(heliState) {
            case INIT :
                usnprintf (string, sizeof(string), "mode: init\r\n");
                break;
            case STARTUP :
                usnprintf (string, sizeof(string), "mode: startup\r\n");
                break;
            case LANDED :
                usnprintf (string, sizeof(string), "mode: landed\r\n");
                break;
            case LANDING :
                usnprintf (string, sizeof(string), "mode: landing\r\n");
                break;
            case FLYING :
                usnprintf (string, sizeof(string), "mode: flying\r\n");
                break;
        }
        UARTSend(string);

}



