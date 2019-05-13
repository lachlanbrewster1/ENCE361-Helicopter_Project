//*****************************************************************************
//
// ADCdemo1.c - Simple interrupt driven program which samples with AIN0
//
// Author:  P.J. Bones	UCECE
// Last modified:	8.2.2018
//
//*****************************************************************************
// Based on the 'convert' series from 2016
//*****************************************************************************

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
#include "circBufT.h"
#include "uart.h"
#include "clock.h"
#include "input.h"
#include "display.h"
#include "pwm.h"
#include "input.h"
#include "height.h"
#include "yaw.h"





//*****************************************************************************
// Definitions
//*****************************************************************************




//*****************************************************************************
// Constants
//*****************************************************************************


//*****************************************************************************
// Global variables
//*****************************************************************************
static uint16_t landed_reference;   // voltage when helicopter has landed
static uint16_t altitude;
static int32_t main_duty = 0;
static int32_t secondary_duty = 0;
enum heli_states {INIT = 0, STARTUP, LANDED, LANDING, FLYING};


uint8_t heliState = INIT;

static int16_t desired_altitude = 0;
static float desired_yaw = 0;

static float K_P_Y = 1.5;
static float K_I_Y = 0.5;
static float K_D_Y = 0.2;

static float K_P_M = 0.5;
static float K_I_M = 0.2;
static float K_D_M = 0.05;

void
checkInputStatus (void)
{
    if (checkInput(LEFT) == PUSHED) {
        desired_yaw -= 15;
        if (desired_yaw < 0) desired_yaw += 360;
    }

    if (checkInput(RIGHT) == PUSHED) {
        desired_yaw += 15;
        if (desired_yaw > 360) desired_yaw -= 360;
    }

    if (checkInput(UP) == PUSHED) {
        if (desired_altitude != 100) {
            desired_altitude += 10;
        }
    }

    if (checkInput(DOWN) == PUSHED) {
        if (desired_altitude != 0) {
            desired_altitude -= 10;
        }
    }
}

int
main(void)
 {
	
	initClock ();
	initADC ();
	initialiseDisplay ();
    initInput ();
    initYaw();
    initialisePWM ();
    setLandedRef(); // maybe need to wait for buffer to fill?

    //
    // Enable interrupts to the processor.
	IntMasterEnable();

	while (1)
	{

	    if (SysTickValueGet() % (SysCtlClockGet() / 40) == 0) {
	        updateInput ();       // Poll the buttons
	    }

	    if (yaw_flag){
	        yaw_flag = false;
	        update_yaw();
	    }


		// Calculate the rounded mean of the buffer contents

		altitude = calculateMeanHeight();


        if (checkInput(SW) == PUSHED)
        {
            switch (heliState) {
            case INIT :     main_duty = 22;
                            secondary_duty = 38;
                            setDutyCycle(secondary_duty, SECONDARY_ROTOR);
                            setDutyCycle(main_duty, MAIN_ROTOR);
                            heliState = STARTUP;
                            break;
            case LANDED :   integrated_yaw_error = 0;
                            integrated_alt_error = 0;
                            heliState = FLYING;
                            break;
            case FLYING :   desired_altitude = 0;
                            desired_yaw = 0;
                            heliState = LANDING;
                            break;
            }
        }

        switch (heliState) {
            case STARTUP : if (ref_state) {
                                main_duty = 0;
                                secondary_duty = 0;
                                setDutyCycle(secondary_duty, SECONDARY_ROTOR);
                                setDutyCycle(main_duty, MAIN_ROTOR);
                                heliState = FLYING;
                            }
                            break;
            case FLYING :   checkInputStatus();
                            if (SysTickValueGet() % (SysCtlClockGet() / 20) == 0) {
                                control();
                            }
                            break;
            case LANDING :  if (SysTickValueGet() % (SysCtlClockGet() / 20) == 0) {
                                control();
                            }
                            break;
        }

		if (SysTickValueGet() % (SysCtlClockGet() / 2) == 0) {
		    displayStatusOLED();
		}
		if (SysTickValueGet() % (SysCtlClockGet() / 8) == 0) {
		    displayStatusUART();          //print all heli info through uart
		}
	}
}

