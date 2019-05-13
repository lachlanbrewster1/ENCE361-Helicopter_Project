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
#include "ADC.h"
#include "display.h"
#include "pwm.h"
#include "input.h"





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
static float yaw = 0;
static bool yaw_A_state = false;
static bool yaw_B_state = false;
static bool ref_state = false;
static volatile bool yaw_flag = false;
static int32_t main_duty = 0;
static int32_t secondary_duty = 0;
enum heli_states {INIT = 0, STARTUP, LANDED, LANDING, FLYING};
static float integrated_yaw_error = 0;
static float integrated_alt_error = 0;
static float alt_error_previous = 0;
static float yaw_error_previous = 0;


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
yawIntHandler(void)
{
    yaw_flag = true;
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
}





void
initYaw(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {
    }

    GPIOIntRegister(GPIO_PORTB_BASE, yawIntHandler);

    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 );

    GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_DIR_MODE_IN);
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
    {
    }

    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIODirModeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN);
}



void update_yaw()
{
    bool new_A = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0);
    bool new_B = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_1);
    bool new_C = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4);

    if (!new_C) {
        yaw = 0;
    }
    else {
        if(!yaw_A_state && !yaw_B_state)
        {
            if (!new_A && new_B)    //  clockwise
                yaw += 360. / 448.;
            else                    // anticlockwise
                yaw -= 360. / 448.;
        }

        if(!yaw_A_state && yaw_B_state)
        {
            if (new_A && new_B)    //  clockwise
                yaw += 360. / 448.;
            else                    // anticlockwise
                yaw -= 360. / 448.;
        }

        if(yaw_A_state && yaw_B_state)
        {
            if (new_A && !new_B)    //  clockwise
                yaw += 360. / 448.;
            else                    // anticlockwise
                yaw -= 360. / 448.;
        }

        if(yaw_A_state && !yaw_B_state)
        {
            if (!new_A && !new_B)    //  clockwise
                yaw += 360. / 448.;
            else                    // anticlockwise
                yaw -= 360. / 448.;
        }



        if (yaw > 360)
            yaw -= 360;
        else if (yaw < 0)
            yaw += 360;
    }

    yaw_A_state = new_A;
    yaw_B_state = new_B;
    ref_state = !new_C;
}


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

void
control (void)
{
    float timestep = 1. / 20;

    float yaw_dif =  desired_yaw - yaw;
    if (yaw_dif > 180)
        yaw_dif -= 360;
    else if (yaw_dif < -180)
        yaw_dif += 360;

    integrated_yaw_error += yaw_dif * timestep;

    float div_error = (yaw_dif - yaw_error_previous) / timestep;

    float div_yaw = K_D_Y * div_error;
    float prop_yaw = K_P_Y * yaw_dif;
    float integral_yaw = K_I_Y * integrated_yaw_error;

    secondary_duty = prop_yaw + integral_yaw + div_yaw;
    if (secondary_duty > 80) secondary_duty = 80;
    if (secondary_duty < 20) secondary_duty = 20;


    float alt_dif = desired_altitude - altitude;
    integrated_alt_error += alt_dif * timestep;

    div_error = (alt_dif - alt_error_previous) / timestep;

    float div_alt = K_D_M * div_error;
    float prop_alt = K_P_M * alt_dif;
    float integral_alt = K_I_M * integrated_alt_error;

    main_duty = prop_alt + integral_alt + div_alt;
    if (main_duty > 80) main_duty = 80;
    if (main_duty < 20) main_duty = 20;

    if (heliState ==  LANDING && altitude < 5) {
        secondary_duty = 0;
        main_duty = 0;
        heliState = LANDED;
    }

    setDutyCycle(secondary_duty, SECONDARY_ROTOR);
    setDutyCycle(main_duty, MAIN_ROTOR);
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


	// SysCtlDelay (SysCtlClockGet() / 6);  // Update display at ~ 2 Hz - Maybe a delay?




	// Calculate and display the rounded mean of the buffer contents
	landed_reference = calculateMeanADC();

    //
    // Enable interrupts to the processor.
	IntMasterEnable();

	while (1)
	{

	    if (g_ulSampCnt % (SAMPLE_RATE_HZ / 40) == 0) {
	        updateInput ();       // Poll the buttons
	    }

	    if (yaw_flag){
	        yaw_flag = false;
	        update_yaw();
	    }


		// Calculate the rounded mean of the buffer contents
	    uint16_t mean = calculateMeanADC();
		altitude = 100 * (landed_reference - mean) / 1000;  //100% *(our new height) / 1000mV


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
                            if (g_ulSampCnt % (SAMPLE_RATE_HZ / 20) == 0) {
                                control();
                            }
                            break;
            case LANDING :  if (g_ulSampCnt % (SAMPLE_RATE_HZ / 20) == 0) {
                                control();
                            }
                            break;
        }

		if (g_ulSampCnt % (SAMPLE_RATE_HZ / 2) == 0) {
		    displayStatusOLED();
		}
		if (g_ulSampCnt % (SAMPLE_RATE_HZ / 8) == 0) {
		    displayStatusUART();          //print all heli info through uart
		}
	}
}

