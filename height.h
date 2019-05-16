/*
 * height.h
 *
 *  Created on: 7/05/2019
 *      Author: bsl28
 */

#ifndef HEIGHT_H_
#define HEIGHT_H_

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

uint16_t
calculateMeanHeight(void);

void
setLandedRef(void);


#endif /* HEIGHT_H_ */
