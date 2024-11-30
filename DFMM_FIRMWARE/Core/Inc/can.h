/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f4xx_hal_can.h"
#include "tim.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */
// The following global variables are used to store the configuration information for CAN communication off of the DFMM
extern CAN_TxHeaderTypeDef   TxHeader;
extern uint8_t               TxData[8];
extern CAN_RxHeaderTypeDef   RxHeader;
extern uint8_t               RxData[8];
extern uint32_t              TxMailbox;
extern CAN_FilterTypeDef     canfilter_1;
/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */
void CAN_Send_Message(uint8_t Transmission_Data[8]);
/**
  * @brief  This function is used to transmit an array of 8 bytes over the CAN bus.
  * @param  Transmission_Data is an array of 8 bytes, with each byte being one 2 digit hex code.
  * @retval void
  */
void CAN_Send_Message_String(char Transmission_Data[8]);
/**
  * @brief  This function is used to transmit an array of an 8 byte string over the CAN bus.
  * @param  Transmission_Data is a string with maximum 8 bytes, which is transmitted over the CAN Bus.
  * @retval void
  */
void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle);

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle);

void CAN_Set_Filter(uint8_t FilterBank, CAN_FilterTypeDef *canhandler);
/**
  * @brief  This function is used to create a filter for any CAN ID incoming over the CAN Bus.
  * @param  Filter Bank is an integer between 0 and 27, which corresponds to the different filter banks that are available in the microcontroller.
  * 		FIFO_Number is an integer between 0 and 1 that corresponds to which FIFO is being used.
  * 		ID_High_Filter is the hex code corresponding to the upper 16 bits of the CANID.
  * 		canhandler is the pointer for the CAN Filter .
  * @retval void
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
/**
  * @brief  This function is called upon by an interrupt once it reaches a rising edge in the timer count.
  * @param hcan is the pointer for the CAN object being used.
  * @retval void
  */
void ReceiveMessage(CAN_RxHeaderTypeDef *ReceptionHeader, uint8_t *Receive_Data);
/**
  * @brief  This function filters all received messages and changes all global flags based on the 8 byte frame received.
  * @param  ReceptionHeader is the pointer for the CAN reception handler being used.
  * 		ReceiveData is the 8 byte array for the message being received.
  * @retval void
  */
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

