/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */



/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
typedef enum Rotary_Switch {
	  MANUAL, // Manual Mission
	  TRACK, // Track Drive Mission
	  EBS_TEST, // EBS Mission
	  INSPECTION  // Inspection Mission
} ROT_SWITCH; // Enum for current state of Rotary Switch
typedef enum EBS_STATE {
	  DEACTIVATED,
	  ACTIVATED
} EBS_STATE; // Enum for EBS readings
typedef struct {
	int TS; // Tractive System State
	ROT_SWITCH R_S; // Mission Select State
	int SA; // Steering Actuator
	int SB; // Service Brake
	EBS_STATE EBS; // Emergency Braking System
	uint8_t ASMS_Status; // Stores the state of the ASMS
} AS__INDICATOR_STATES; // Struct for all current AV state attributes.
typedef enum SYSTEM_FAILURE {
	  SDC_FAILURE,
	  NON_PROG_FAILURE,
	  EBS_FAILURE,
	  IMD_FAULT,
	  BSPD_FAULT,
	  BMS_FAULT,
	  HEARTBEAT_FAULT,
	  SPRING_BROKE,
	  NONE
  } FAILURE_MODE_READING; // Enum for any system failure scenario
typedef enum AV_STATUS {
	READY,
	DRIVING,
	OFF,
	EMERGENCY,
	FINISHED
	} AV_STATE; // Enum for current state of the AV, referenced from FSM in .README;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/**
  * @brief  This function is used to handle the failure-specific information transmitted over CAN.
  * 		Depending on the Failure Mode, the response over CAN will be different.
  * @param  fm is an enum that corresponds to the different types of failures that can occur in the AV.
  * @retval void
  */
void SystemFailureHandler(FAILURE_MODE_READING fm);
/**
  * @brief  This function is used to cycle through the ASSI and EBS outputs of the current AV State.
  * @param  fm is an enum that corresponds to the different types of failures that can occur in the AV.
  * 		indicators is a pointer to the struct containing all state information.
  * 		status is an enum that corresponds to the current state of the AV, according to the FSM in the .README documentation.
  * @retval void
  */
void AV_State_Outputs(AS__INDICATOR_STATES *indicators, AV_STATE status, FAILURE_MODE_READING FM);
/**
  * @brief  This function is used to check what state the Rotary Switch is currently in.
  * @param  RS is a pointer to the Rotary Switch enum that corresponds to all the different states.
  * @retval void
  */
void Read_Rotary(ROT_SWITCH *RS);
/**
  * @brief  This function reads the status of IMD, BSPD and BMS failure pins and changes the corresponding flags.
  * @param  fm is an enum that corresponds to the different types of failures that can occur in the AV.
  *			indicators is a pointer to the struct containing all state information.
  * @retval void
  */
void ReadForFaults(FAILURE_MODE_READING *Fm, AS__INDICATOR_STATES *indicators);
/**
  * @brief  This function is used to activate the EBS by setting the EBS actuator pin high. Also activates the redundant system if EBS not available.
  * @param  indicators is a pointer to the struct containing all state information.
  * @retval void
  */
void EBSActivate(AS__INDICATOR_STATES *indicators);
/**
  * @brief  This function is used to change the EBS variable in the indicator struct to the current state of the EBS: Activated for EBS activation, Deactivated for no EBS activation.
  * @param  indicators is a pointer to the struct containing all state information.
  * @retval void
  */
void EBSCheck(AS__INDICATOR_STATES *indicators);
/**
  * @brief  Performs initial checkup before going into Ready State from Idle.
  * @param  void
  * @retval void
  */
void Initial_Checkup(void);
/**
  * @brief  This function is used to deactivate the EBS by enabling the SDC, which will happen if failures have been resolved.
  * @param  indicators is a pointer to the struct containing all state information.
  * @retval void
  */
void EBSReset(AS__INDICATOR_STATES *indicators);
/**
  * @brief  This function is used to change all variable in the indicator struct to their current state.
  * @param  indicators is a pointer to the struct containing all state information.
  * @retval void
  */
void ReadInputs(AS__INDICATOR_STATES *indicators);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ASSI_BLUE_Pin GPIO_PIN_1
#define ASSI_BLUE_GPIO_Port GPIOC
#define ANL1_Pin GPIO_PIN_1
#define ANL1_GPIO_Port GPIOA
#define ANL2_Pin GPIO_PIN_2
#define ANL2_GPIO_Port GPIOA
#define BMS_FAULT_Pin GPIO_PIN_4
#define BMS_FAULT_GPIO_Port GPIOA
#define BSPD_FLT_Pin GPIO_PIN_5
#define BSPD_FLT_GPIO_Port GPIOA
#define IMD_FAULT_Pin GPIO_PIN_6
#define IMD_FAULT_GPIO_Port GPIOA
#define ASMS_SIG_Pin GPIO_PIN_7
#define ASMS_SIG_GPIO_Port GPIOA
#define AS_CLOSE_SDC_Pin GPIO_PIN_4
#define AS_CLOSE_SDC_GPIO_Port GPIOC
#define _3V3_TS_SWITCH_Pin GPIO_PIN_10
#define _3V3_TS_SWITCH_GPIO_Port GPIOB
#define SDC_CHECK_Pin GPIO_PIN_12
#define SDC_CHECK_GPIO_Port GPIOB
#define AS_SDC_Pin GPIO_PIN_13
#define AS_SDC_GPIO_Port GPIOB
#define WDI_Pin GPIO_PIN_14
#define WDI_GPIO_Port GPIOB
#define WDO_Pin GPIO_PIN_15
#define WDO_GPIO_Port GPIOB
#define ROT_1_SIG_Pin GPIO_PIN_6
#define ROT_1_SIG_GPIO_Port GPIOC
#define ROT_2_SIG_Pin GPIO_PIN_7
#define ROT_2_SIG_GPIO_Port GPIOC
#define ROT_3_SIG_Pin GPIO_PIN_8
#define ROT_3_SIG_GPIO_Port GPIOC
#define ROT_4_SIG_Pin GPIO_PIN_9
#define ROT_4_SIG_GPIO_Port GPIOC
#define WDG_STATUS_Pin GPIO_PIN_8
#define WDG_STATUS_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define JTDI_Pin GPIO_PIN_15
#define JTDI_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
