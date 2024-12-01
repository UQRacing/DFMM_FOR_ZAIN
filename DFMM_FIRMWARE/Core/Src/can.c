/* USER CODE BEGIN Header */
#include <string.h>
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
extern volatile uint8_t	 HeartbeatCheck[2];
extern volatile uint8_t length; // Length of message being sent
CAN_TxHeaderTypeDef   TxHeader;
uint8_t               TxData[8];
uint8_t               DataLogger[3];
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[8];
uint32_t              TxMailbox;
CAN_FilterTypeDef canfilter;
/* The following are constant IDs of each control node's messages. Messages from the M150 for example can
 * be from any id in the range 0x510 - 0x530, giving a range of usable channels for messages TO the DFMM. DFMM has
 * standard node ID 0x476.
 */
const uint32_t	     DFMM_ID = 0x446;
const uint32_t	     M150_ID = 0x510;
const uint32_t	     EPOS4_ID = 0x530;
const uint32_t	     RES_ID = 0x550;
const uint32_t	     PC_ID = 0x570;
const uint32_t	     PC_ID_FRAME1 = 0x590; // Data logger frame 1
const uint32_t	     PC_ID_FRAME2 = 0x591; // Data logger frame 2
const uint32_t	     PC_ID_FRAME3 = 0x592; // Data logger frame 3

extern volatile uint8_t EBS_Energy_Check; // Cannister Pressure Check: 1 for ok, 0 for not
extern volatile uint8_t EBS_Brake_Line; // Brake Line Pressure Check: 1 for ok, 0 for not
extern volatile uint8_t Service_Brake_Check; // Redundant
extern volatile uint8_t Steering_Actuator_Check; // Redundant
extern volatile uint8_t Mission_Finished; // Flag for mission finish from PC: 1 if finished, 0 if not
extern volatile uint8_t EBS_Sound; // Flag for EBS buzzer or Driving buzzer playing: 0 for not playing, 1 for EBS buzzer playing, 2 for Driving buzzer playing
extern volatile uint8_t RES; // Flag for RES activated: 0 for not activated, 1 for activated
extern volatile uint8_t R2D; // Flag for R2D switch on RES being switched: 0 if not, 1 for activated
extern volatile uint8_t StrainGauge; // Flag for strain gauge off M150 showing broken spring: 0 if not, 1 for broken
extern volatile uint8_t EBS_Test_Accel_Stop; // Redundant
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
      Error_Handler();
    }
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = DFMM_ID;
  TxHeader.RTR = CAN_RTR_DATA;
  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 4;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
	Error_Handler();
  }
  CAN_Set_Filter(17, &canfilter); // ALL messages sent to the DFMM must have channel id starting with 5
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) // Activates interrupt for received messages.
  {
	Error_Handler();
  }
  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5|GPIO_PIN_6);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void CAN_Send_Message(uint8_t* Transmission_Data)
{
	memset(TxData, 0, sizeof(TxData)); // Clear TxData array
	length = (sizeof(Transmission_Data))/8; // Find length of message being sent
	TxHeader.DLC = length; // Set TxHeader register to have this message length
	int i;
	for(i=0; i<(length-1); i++) {
		*(TxData + i) = *(Transmission_Data + i); // iterate over length of message to copy into TxData array
	}

	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
	   Error_Handler();
	}
}

void CAN_Send_Message_String(char* Transmission_Data)
{
	memset(TxData, 0, sizeof(TxData)); // Clear TxData array
	length = (sizeof(Transmission_Data))/8; // Find length of message being sent
	TxHeader.DLC = length; // Set TxHeader register to have this message length
	int i;
	for(i=0; i<(length-1); i++) {
		*(TxData + i) = (unsigned int)((unsigned char)*(Transmission_Data + i)); // iterate over length of message to copy into TxData array
	}

	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
	   Error_Handler();
	}
}


void CAN_Set_Filter(uint8_t FilterBank, CAN_FilterTypeDef *canhandler) {
    canhandler->FilterActivation = CAN_FILTER_ENABLE;
	canhandler->FilterBank = FilterBank;  // which filter bank to use from the assigned ones
	canhandler->FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canhandler->FilterIdHigh = 0x500<<5;
	canhandler->FilterIdLow = 0;
	canhandler->FilterMaskIdHigh = 0xF00<<5;
	canhandler->FilterMaskIdLow = 0;
	canhandler->FilterMode = CAN_FILTERMODE_IDMASK;
	canhandler->FilterScale = CAN_FILTERSCALE_32BIT;
	canhandler->SlaveStartFilterBank = 14;  // how many filters to assign to the CAN1 (master can)
	HAL_CAN_ConfigFilter(&hcan2, canhandler);
}

void ReceiveMessage(CAN_RxHeaderTypeDef *ReceptionHeader, uint8_t *Receive_Data) {
	/* For the following, the ID matches and other information that switch the flags MUST be changed
	 * based on the car's CAN bus and channel requirements. the overarching if else statement
	 * matches the id of the received frame with the NodeID list that was created above. These constants
	 * can be changed and new ones may be created for separate channels for the same node.
	 * The sub if-else statement (if there is one) matches the received byte of the message at the 7th position to a set of static bytes corresponding to the flag
	 * being switched. The static byte (0x01, 0x02, 0x03, etc) must be changed in accordance with car
	 * messages AND the byte offset must be changed depending on the length of the message and where the byte
	 * is located (7th to 4th, etc).
	 */
	if (ReceptionHeader->StdId == PC_ID_FRAME1) {
		DataLogger[0] = *(Receive_Data); // First data logger byte stored in array DataLogger
	}
	else if (ReceptionHeader->StdId == PC_ID_FRAME2) {
			DataLogger[1] = *(Receive_Data); // Second data logger byte stored in array DataLogger
	}
	else if (ReceptionHeader->StdId == PC_ID_FRAME2) {
				DataLogger[2] = *(Receive_Data); // Third data logger byte stored in array DataLogger
				CAN_Send_Message(DataLogger); // Message is sent
	}
	else if (ReceptionHeader->StdId == M150_ID) {
		HeartbeatCheck[0] = __HAL_TIM_GET_COUNTER(&htim7); // Sets Heartbeat for M150
		if (0x01 == *(Receive_Data + 7)) {
			EBS_Brake_Line = 1;
		}
		else if (0x03 ==*(Receive_Data + 7)) {
			Service_Brake_Check = 1;
		}
		else if (0x04 == *(Receive_Data + 7)) {
			EBS_Sound = 1;
		}
		else if (0x05 == *(Receive_Data + 7)) {
			EBS_Sound = 2;
		}
		else if (0x06 ==*(Receive_Data + 7)) {
			Service_Brake_Check = 2;
		}
		else if (0x07 == *(Receive_Data + 7)) {
			EBS_Brake_Line = 0;
		}
		else if (0x09 == *(Receive_Data + 7)) {
			EBS_Sound = 0;
		}
		else if (0x0A ==*(Receive_Data + 7)) {
			Service_Brake_Check = 0;
		}
		else if (0x0B == *(Receive_Data + 7)) {
			Mission_Finished = 1;
		}
		else if (0x0E == *(Receive_Data + 7)) {
			StrainGauge = 1;
		}
		else if (0x0F == *(Receive_Data + 7)) {
			StrainGauge = 0;
		}
	}

	else if (ReceptionHeader->StdId == RES_ID) {
			if (0x0B == *(Receive_Data + 7)) {
				RES = 1;
			}
			else if (0x0C == *(Receive_Data + 7)) {
				R2D = 1;
			}
			else if (0x0D == *(Receive_Data + 7)) {
				R2D = 0;
			}
		}

	else if (ReceptionHeader->StdId == PC_ID) {
		HeartbeatCheck[1] = __HAL_TIM_GET_COUNTER(&htim7);
		if (0x11 == *(Receive_Data + 7)) {}
		else if (0x01 == *(Receive_Data + 7)) {
			EBS_Energy_Check = 1;
		}
		else if (0x02 == *(Receive_Data + 7)) {
			EBS_Energy_Check = 0;
		}
		else if (0x03 == *(Receive_Data + 7)) {
			EBS_Test_Accel_Stop = 1;
		}
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
		Error_Handler();
  }
  ReceiveMessage(&RxHeader, RxData);
 }


/* USER CODE END 1 */
