/*
 * ASSI.c
 *
 *  Created on: Aug 20, 2024
 *      Author: anhad
 */
#include "ASSI.h"
#include "tim.h"
#include "main.h"
#include <stdint.h>
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_ll_usart.h"
uint8_t LED_Data[NUM_LED][4];
uint16_t ASSI_Timecheck = 0;

int toggle = 0;

void setLED (int led, int RED, int GREEN, int BLUE)
{
    LED_Data[led][0] = led;
    LED_Data[led][1] = GREEN;
    LED_Data[led][2] = RED;
    LED_Data[led][3] = BLUE;
}

void ws2812_synchronous (int GREEN, int RED, int BLUE)
{
	//Here I'm bit shifting the values from SetLED as the LED strip reads GRB values not RGB values
    uint32_t color = (GREEN << 8) | (RED<<4) | BLUE;
    //Here, I've made a 12 byte (36 bit) array because when I sent a 24 byte array,
    //Twice as many LED's lit up (i.e. if NUM_LED was 5, then sending 24 bytes for each LED (72 bits)
    //Would light up 10 LED's instead of 5 and these would be the wrong colour
    //Hence, I changed the sendData buffer to be a 12 byte array and this made the right number of LED's light up
    //But the colours being delivered are slightly off and the first LED turns green, (I'm not sure how to resolve this)
    uint8_t sendData[12];
    int indx = 0;

    for (int i = 12; i >=0; i--)
    {
    	//The if statement below shifts the bit present to the least significant bit and checks if this is a 1 or 0
        if (((color >> i) & 0x01) == 1)
            sendData[indx++] = 0b110;  // store 1
        else
            sendData[indx++] = 0b100;  // store 0
    }

    //Calling the DMA transmit function so that data can be sent
    //I'm using DMA as it results in the fastest data transfer and less lag
    HAL_UART_Transmit(&huart1, sendData, 12, HAL_MAX_DELAY);
    //The line below only allows data to be transmitted again after the data beforehand has been transmitted
    while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY); // Wait for DMA transfer to complete
}

void WS2812_Send (void)
{
    for (int i = 0; i < NUM_LED; i++)
    {
        ws2812_synchronous(LED_Data[i][1], LED_Data[i][2], LED_Data[i][3]);
    }
    HAL_Delay(1);
 // Wait for all data to be latched
}

//The functions above are essential to driving the LED's.
//The functions below are specifically for the ASSI status


void ASSI_Off(void)
{

		for (int i=0; i< NUM_LED; i++)
		{
			setLED(i, 0,0,0);
		}
		WS2812_Send();
}

void ASSI_Ready(void)
{
	//This keeps the LED strip a continous yello
		//for (int x = 0; x<250; x+=1) {
			for (int i=0; i<NUM_LED; i++)
			{
				//This sets the LED's to yellow, weirdly one would expect this to turn the strip green
				//But this turns the lights yellow, some tuning may be required for different strips
				setLED(i,230,1,0);
			}
			WS2812_Send();
		//}
	}

void ASSI_Driving(void)
{
	//This is yellow flashing lights
	//Setting the LED's yellow initially
	if (__HAL_TIM_GET_COUNTER(&htim6) - ASSI_Timecheck > 200 && toggle == 0) {
		for (int i =0; i<NUM_LED; i++)
		{
			setLED(i, 230, 1, 0);
		}
		//Sending this data to the strip
		WS2812_Send();
		ASSI_Timecheck = __HAL_TIM_GET_COUNTER(&htim6);
		toggle = 1;
	}
	else if (__HAL_TIM_GET_COUNTER(&htim6) - ASSI_Timecheck > 200 && toggle == 1) {
		for (int i=0; i<NUM_LED; i++)
		{
			//Switching the LED's off
			setLED(i, 0,0,0);
		}
		WS2812_Send();
		ASSI_Timecheck = __HAL_TIM_GET_COUNTER(&htim6);
		toggle = 0;
	}
}

void ASSI_Emergency(void)
{
		//This is blue flashing lights
		//This is the colour data for a dark blue, light blue can be achieved by setLED(i,0,50,255);
		if (__HAL_TIM_GET_COUNTER(&htim7) - ASSI_Timecheck > 200 && toggle == 0) {
			//Setting the LED's blue initially
			for (int i =0; i<NUM_LED; i++)
			{
				setLED(i, 0, 200, 255);
				//The timings are slightly off hence why the G value is 100
			}
			//Sending this data to the strip
			WS2812_Send();
			ASSI_Timecheck = __HAL_TIM_GET_COUNTER(&htim6);
			toggle = 1;
		}
		else if (__HAL_TIM_GET_COUNTER(&htim6) - ASSI_Timecheck > 80 && toggle == 1) {
			for (int i=0; i<NUM_LED; i++)
			{
				//Switching the LED's off
				setLED(i, 0,0,0);
			}
			WS2812_Send();
			ASSI_Timecheck = __HAL_TIM_GET_COUNTER(&htim6);
			toggle = 0;
		}
}

void ASSI_Finished(void)
{
	if (__HAL_TIM_GET_COUNTER(&htim7) - ASSI_Timecheck > 1) {

		for (int i=0; i<NUM_LED; i++)
		{
			//This sets the LED's to a blue colour
			setLED(i,0,200,255);
		}
		WS2812_Send();
	}
}
