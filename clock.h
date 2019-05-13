/*
 * clock.h
 *
 *  Created on: 7/05/2019
 *      Author: bsl28
 */

#ifndef CLOCK_H_
#define CLOCK_H_

#define SAMPLE_RATE_HZ 800
#define CLOCK_RATE 20000000

//*****************************************************************************
//
// The interrupt handler for the for SysTick interrupt. Calls interrupt handler for ADC conversion
//
//*****************************************************************************
void SysTickIntHandler(void);

//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC
//*****************************************************************************
void initClock (void);

#endif /* CLOCK_H_ */
