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
#include "uart.h"

//*****************************************************************************
// Definitions
//*****************************************************************************

// PWM configuration
#define PWM_START_DUTY  0
#define PWM_RATE_MIN_DUTY    0
#define PWM_RATE_MAX_DUTY    100
#define PWM_FIXED_RATE_HZ     200
#define PWM_DIVIDER_CODE   SYSCTL_PWMDIV_4

//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Main Rotor PWM: PC5, J4-05
#define PWM_MAIN_BASE        PWM0_BASE
#define PWM_MAIN_GEN         PWM_GEN_3
#define PWM_MAIN_OUTNUM      PWM_OUT_7
#define PWM_MAIN_OUTBIT      PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM  SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE   GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN    GPIO_PIN_5

#define PWM_SECONDARY_BASE        PWM1_BASE
#define PWM_SECONDARY_GEN         PWM_GEN_2
#define PWM_SECONDARY_OUTNUM      PWM_OUT_5
#define PWM_SECONDARY_OUTBIT      PWM_OUT_5_BIT
#define PWM_SECONDARY_PERIPH_PWM  SYSCTL_PERIPH_PWM1
#define PWM_SECONDARY_PERIPH_GPIO SYSCTL_PERIPH_GPIOF
#define PWM_SECONDARY_GPIO_BASE   GPIO_PORTF_BASE
#define PWM_SECONDARY_GPIO_CONFIG GPIO_PF1_M1PWM5
#define PWM_SECONDARY_GPIO_PIN    GPIO_PIN_1

#define MAIN_ROTOR 1
#define SECONDARY_ROTOR 2

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
static uint16_t altitude;
static float yaw = 0;
static bool yaw_A_state = false;
static bool yaw_B_state = false;
static bool ref_state = false;
static volatile bool yaw_flag = false;
static volatile bool swChanged = false;
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

void
swIntHandler(void)
{
    swChanged = true;
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_7);
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
initialisePWM (void)
{
    // Initialise main rotor
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters

    PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, SysCtlClockGet() / PWM_FIXED_RATE_HZ);

    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);

    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);


    // Initialise secondary rotor
    SysCtlPeripheralEnable(PWM_SECONDARY_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_SECONDARY_PERIPH_GPIO);

    GPIOPinConfigure(PWM_SECONDARY_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_SECONDARY_GPIO_BASE, PWM_SECONDARY_GPIO_PIN);

    PWMGenConfigure(PWM_SECONDARY_BASE, PWM_SECONDARY_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    PWMGenPeriodSet(PWM_SECONDARY_BASE, PWM_SECONDARY_GEN, SysCtlClockGet() / PWM_FIXED_RATE_HZ);

    PWMGenEnable(PWM_SECONDARY_BASE, PWM_SECONDARY_GEN);

    PWMOutputState(PWM_SECONDARY_BASE, PWM_SECONDARY_OUTBIT, true);
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

void
initSwInt(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
    {
    }

    GPIOIntRegister(GPIO_PORTA_BASE, swIntHandler);

    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7 );

    GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_DIR_MODE_IN);
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_RISING_EDGE);
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_7);
}

void
display()
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

void printStatus()
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

/********************************************************
 * Function to set the freq, duty cycle of M0PWM7
 ********************************************************/
void
setDutyCycle (uint32_t ui32Duty, uint8_t rotor)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period =
        SysCtlClockGet() / PWM_FIXED_RATE_HZ;

    if (rotor == MAIN_ROTOR){
        PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM,
        ui32Period * ui32Duty / 100);
    }
    else if (rotor == SECONDARY_ROTOR){
        PWMPulseWidthSet(PWM_SECONDARY_BASE, PWM_SECONDARY_OUTNUM,
            ui32Period * ui32Duty / 100);
    }
}

void
checkInput (void)
{
    if (checkButton(LEFT) == PUSHED) {
        desired_yaw -= 15;
        if (desired_yaw < 0) desired_yaw += 360;
    }

    if (checkButton(RIGHT) == PUSHED) {
        desired_yaw += 15;
        if (desired_yaw > 360) desired_yaw -= 360;
    }

    if (checkButton(UP) == PUSHED) {
        if (desired_altitude != 100) {
            desired_altitude += 10;
        }
    }

    if (checkButton(DOWN) == PUSHED) {
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

    float div_error = yaw_dif - yaw_error_previous;

    float div_yaw = K_D_Y * div_error;
    float prop_yaw = K_P_Y * yaw_dif;
    float integral_yaw = K_I_Y * integrated_yaw_error;

    secondary_duty = prop_yaw + integral_yaw + div_yaw;
    if (secondary_duty > 80) secondary_duty = 80;
    if (secondary_duty < 20) secondary_duty = 20;


    float alt_dif = desired_altitude - altitude;
    integrated_alt_error += alt_dif * timestep;

    div_error = alt_dif - alt_error_previous;

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
	uint16_t i;
	int32_t sum;
	uint16_t mean;
	
	initClock ();
	initADC ();
	initDisplay ();
	initCircBuf (&g_inBuffer, BUF_SIZE);
    initButtons ();
    initYaw();
    initSwInt();
    initialisePWM ();
    initialiseUSB_UART();


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
		altitude = 100 * (landed_reference - mean) / SAMPLE_RATE_HZ;


        if (swChanged){
            swChanged = false;

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
            case FLYING :   checkInput();
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
		    display();
		}
		if (g_ulSampCnt % (SAMPLE_RATE_HZ / 8) == 0) {
		    printStatus();          //print all heli info through uart
		}
	}
}

