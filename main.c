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
#include "helistates.h"
#include "rotors.h"





//*****************************************************************************
// Definitions
//*****************************************************************************




//*****************************************************************************
// Constants
//*****************************************************************************


//*****************************************************************************
// Global variables
//*****************************************************************************
uint8_t heli_state = INIT;


void
checkInputStatus (void)
{
    if (checkInput(LEFT) == PUSHED) {
        if (yaw.desired == 0)
            yaw.desired += 345;
        yaw.desired -= 15;
    }

    if (checkInput(RIGHT) == PUSHED) {
        yaw.desired += 15;
        if (yaw.desired > 360) yaw.desired -= 360;
    }

    if (checkInput(UP) == PUSHED) {
        if (alt.desired != 100) {
            alt.desired += 10;
        }
    }

    if (checkInput(DOWN) == PUSHED) {
        if (alt.desired != 0) {
            alt.desired -= 10;
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

	    if (getFlag40Hz()) {
	        setFlag40Hz(false);
	        updateInput ();       // Poll the buttons
	    }

	    if (yaw_flag){
	        yaw_flag = false;
	        update_yaw();
	    }


        if (checkInput(SW) == PUSHED)
        {
            switch (heli_state) {
            case INIT :     heli_state = STARTUP;
                            break;
            case LANDED :   yaw.error_integrated = 0;
                            alt.error_integrated = 0;
                            heli_state = FLYING;
                            break;
            case FLYING :   alt.desired = 0;
                            yaw.desired = 0;
                            heli_state = LANDING;
                            break;
            }
        }

        switch (heli_state) {
            case STARTUP :  yaw.desired = yaw.actual + 2;
                            if (yaw.desired > 360)
                                yaw.desired -= 360;
                            if (atRef()) {
                                main_duty = 0;
                                secondary_duty = 0;
                                setDutyCycle(secondary_duty, SECONDARY_ROTOR);
                                setDutyCycle(main_duty, MAIN_ROTOR);
                                heli_state = FLYING;
                            }
                            break;
            case FLYING :   checkInputStatus();
                            if (getFlag20Hz()) {
                                setFlag20Hz(false);
                                doControl(20); //do control at 20Hz
                            }
                            break;
            case LANDING :  if (getFlag20Hz()) {
                                setFlag20Hz(false);
                                doControl(20);  //do control at 20Hz
                            }
                            break;
        }

        uint32_t systick = SysTickValueGet();
        uint32_t period = SysTickPeriodGet();

		if (getFlag2Hz()) {
		    setFlag2Hz(false);
		    displayStatusOLED(alt.actual, yaw.actual, main_duty, secondary_duty);
		}
        if (getFlag8Hz()) {
            setFlag8Hz(false);
		    displayStatusUART(alt.actual, alt.desired, yaw.actual, yaw.desired, main_duty, secondary_duty);          //print all heli info through uart
		}
	}
}

