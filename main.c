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
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "buttons4.h"

//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE 100
#define SAMPLE_RATE_HZ 800

//*****************************************************************************
// Global variables
//*****************************************************************************
static circBuf_t g_inBuffer;		// Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulSampCnt;	// Counter for the interrupts
static uint16_t landed_reference;   // voltage when helicopter has landed
static float yaw_offset = 0;
static bool yaw_A_state = false;
static bool yaw_B_state = false;
static bool yaw_flag = false;


//*****************************************************************************
//
// The interrupt handler for the for SysTick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Initiate a conversion
    //
    ADCProcessorTrigger(ADC0_BASE, 3);
    g_ulSampCnt++;
}

void
yawIntHandler(void)
{
    yaw_flag = true;
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
}

//*****************************************************************************
//
// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
//
//*****************************************************************************
void
ADCIntHandler(void)
{
	uint32_t ulValue;
	
	//
	// Get the single sample from ADC0.  ADC_BASE is defined in
	// inc/hw_memmap.h
	ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);
	//
	// Place it in the circular buffer (advancing write index)
	writeCircBuf (&g_inBuffer, ulValue);
	//
	// Clean up, clearing the interrupt
	ADCIntClear(ADC0_BASE, 3);
}

//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display
//*****************************************************************************
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
    //
    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister(SysTickIntHandler);
    //
    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();
}

void 
initADC (void)
{
    //
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    
    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
  
    //
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // on the ADC sequences and steps, refer to the LM3S1968 datasheet.
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE |
                             ADC_CTL_END);    
                             
    //
    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);
  
    //
    // Register the interrupt handler
    ADCIntRegister (ADC0_BASE, 3, ADCIntHandler);
  
    //
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, 3);
}

void
initDisplay (void)
{
    // intialise the Orbit OLED display
    OLEDInitialise ();
}

void
initYaw(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {
    }

    //GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);
    //GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_DIR_MODE_IN);

    GPIOIntRegister(GPIO_PORTB_BASE, yawIntHandler);

    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 );

    GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_DIR_MODE_IN);
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
}

//*****************************************************************************
//
// Function to display the mean ADC value (10-bit value, note) and sample count.
//
//*****************************************************************************
void
displayMeanVal(uint16_t meanVal, uint32_t count)
{
	char string[17];  // 16 characters across the display

    OLEDStringDraw ("heli height raw", 0, 0);
	
    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf (string, sizeof(string), "Mean ADC = %4d", meanVal);
    // Update line on display.
    OLEDStringDraw (string, 0, 1);

    OLEDStringDraw("                ", 0, 2);
}

void
displayPercentageHeight(uint16_t meanVal, uint32_t count)
{
    char string[17];  // 16 characters across the display

    OLEDStringDraw ("Heli height %", 0, 0);

    int16_t percentage;
    percentage = (landed_reference - meanVal) / 8; //

    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf (string, sizeof(string), "height = %3d%%", percentage);
    // Update line on display.
    OLEDStringDraw (string, 0, 1);

    // DELETE THIS
    uint16_t yaw = yaw_offset;


    usnprintf (string, sizeof(string), "yaw = %3d deg", yaw);
    // Update line on display.
    OLEDStringDraw (string, 0, 2);
}

void clearDisplay()
{
    OLEDStringDraw("                ", 0, 0);
    OLEDStringDraw("                ", 0, 1);
    OLEDStringDraw("                ", 0, 2);
    OLEDStringDraw("                ", 0, 3);
}

void update_yaw()
{
    bool new_A = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0);
    bool new_B = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_1);

    if(!yaw_A_state && !yaw_B_state)
    {
        if (!new_A && new_B)    //  clockwise
            yaw_offset += 360. / 448.;
        else                    // anticlockwise
            yaw_offset -= 360. / 448.;
    }

    if(!yaw_A_state && yaw_B_state)
    {
        if (new_A && new_B)    //  clockwise
            yaw_offset += 360. / 448.;
        else                    // anticlockwise
            yaw_offset -= 360. / 448.;
    }

    if(yaw_A_state && yaw_B_state)
    {
        if (new_A && !new_B)    //  clockwise
            yaw_offset += 360. / 448.;
        else                    // anticlockwise
            yaw_offset -= 360. / 448.;
    }

    if(yaw_A_state && !yaw_B_state)
    {
        if (!new_A && !new_B)    //  clockwise
            yaw_offset += 360. / 448.;
        else                    // anticlockwise
            yaw_offset -= 360. / 448.;
    }


    if (yaw_offset > 360)
        yaw_offset -= 360;
    else if (yaw_offset < 0)
        yaw_offset += 360;

    yaw_A_state = new_A;
    yaw_B_state = new_B;
}


int
main(void)
 {
	uint16_t i;
	int32_t sum;
	uint16_t mean;
    uint8_t displayState = 0;
    uint8_t butState = 0;

	
	initClock ();
	initADC ();
	initDisplay ();
	initCircBuf (&g_inBuffer, BUF_SIZE);
    initButtons ();
    initYaw();


	sum = 0;

	SysCtlDelay (SysCtlClockGet() / 6);  // Update display at ~ 2 Hz



	for (i = 0; i < BUF_SIZE; i++)
	    sum = sum + readCircBuf (&g_inBuffer);
	// Calculate and display the rounded mean of the buffer contents
	landed_reference = (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;

    //
    // Enable interrupts to the processor.
	IntMasterEnable();

	while (1)
	{
	    if (g_ulSampCnt % 40 == 0) {
	        updateButtons ();       // Poll the buttons
	    }

	    if (yaw_flag){
	        yaw_flag = false;
	        update_yaw();
	    }

		//
		// Background task: calculate the (approximate) mean of the values in the
		// circular buffer and display it, together with the sample number.
		sum = 0;
		for (i = 0; i < BUF_SIZE; i++)
			sum = sum + readCircBuf (&g_inBuffer);
		// Calculate and display the rounded mean of the buffer contents
		mean = (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;



        butState = checkButton (LEFT);
        if (butState == PUSHED) {
            landed_reference = mean;
            yaw_offset = 0;
        }


		butState = checkButton (UP);
		if (butState == PUSHED) {
		    displayState = (displayState + 1) % 3;
		}
		if (g_ulSampCnt % 400 == 0) {
            switch (displayState)
            {
            case 0:
                //clearDisplay();
                displayPercentageHeight (mean, g_ulSampCnt);
                break;
            case 1:
                //clearDisplay();
                displayMeanVal (mean, g_ulSampCnt);
                break;
            case 2:
                clearDisplay();
                break;
            }
		}


		//SysCtlDelay (SysCtlClockGet() / 6);  // Update display at ~ 2 Hz
	}
}

