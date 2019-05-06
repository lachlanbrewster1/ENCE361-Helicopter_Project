/*
 * ADC.h
 *
 *  Created on: 7/05/2019
 *      Author: bsl28
 */

#ifndef ADC_H_
#define ADC_H_

#define BUF_SIZE 100

//*****************************************************************************
//
// The interrupt handler for the for ADC conversion interrupt.
//
//*****************************************************************************
void ADCIntHandler(void);

//*****************************************************************************
// Initialisation functions for the ADC
//*****************************************************************************
void initADC (void);


#endif /* ADC_H_ */
