/*
 * ASSI.h
 *
 *  Created on: Aug 20, 2024
 *      Author: anhad
 */

#ifndef INC_ASSI_H_
#define INC_ASSI_H_

#include "usart.h"
#include <stdint.h>

#define NUM_LED 119



void setLED(int led, int RED, int GREEN, int BLUE);
void ws2812_synchronous (int GREEN, int RED, int BLUE);
void WS2812_Send (void);
void ASSI_Off(void);
void ASSI_Ready(void);
void ASSI_Driving(void);
void ASSI_Emergency(void);
void ASSI_Finished(void);


#endif /* INC_ASSI_H_ */

