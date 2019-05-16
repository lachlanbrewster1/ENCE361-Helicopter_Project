/*
 * display.h
 *
 *  Created on: 13/05/2019
 *      Author: lbr63
 */



#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <stdint.h>

void initialiseDisplay(void);

void
displayStatusOLED(uint16_t altitude, uint16_t yaw, uint16_t main_duty, uint16_t secondary_duty);

void
displayStatusUART(uint16_t altitude, uint16_t desired_altitude, uint16_t yaw, uint16_t desired_yaw, uint16_t main_duty, uint16_t secondary_duty);



#endif /* DISPLAY_H_ */
