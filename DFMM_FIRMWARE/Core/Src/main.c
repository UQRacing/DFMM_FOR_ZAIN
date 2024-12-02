/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ASSI.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
AV_STATE av_status = OFF; // Tracks current state of AV (initial state is off)
FAILURE_MODE_READING FAILURE_MODE = NONE; // Stores current error of the AV for error message
AS__INDICATOR_STATES State_Variables; // Stores current state of AV inputs
AS__INDICATOR_STATES* indicators = &State_Variables; // Pointer to the AV inputs struct
uint8_t M150_State_Trigger[8];

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t av_status_hex = 0; // Stores av status as a hex code: 0 for off, 1 for ready, 2 for driving, 3 for finished and 4 for emergency
uint16_t timecheck = 0; // Stores time in order to trigger timeout errors
uint8_t CAN_Bus_Limiter = 0; // Acts as a limit that prevents a large number of error messages being dumped into the CAN Bus.

/* The following variables are global flags triggered by various CAN Channels. These will NEED to be changed
 * depending on the hex address of the channel (refer to the CAN source file)
 */

// Cannister Pressure Check: 1 for ok, 0 for not
volatile uint8_t EBS_Energy_Check = 0;
// Brake Line Pressure Check: 1 for ok, 0 for not
volatile uint8_t EBS_Brake_Line = 0;
// Redundant
volatile uint8_t Service_Brake_Check = 0;
// Redundant
volatile uint8_t Steering_Actuator_Check = 0;
// Flag for mission finish from PC: 1 if finished, 0 if not
volatile uint8_t Mission_Finished = 0;
// Flag for EBS buzzer or Driving buzzer playing: 0 for not playing, 1 for EBS buzzer playing, 2 for Driving buzzer playing
volatile uint8_t EBS_Sound = 0;
// Flag for RES activated: 0 for not activated, 1 for activated
volatile uint8_t RES = 0;
// Flag for R2D switch on RES being switched: 0 if not, 1 for activated
volatile uint8_t R2D = 0;
// Flag for strain gauge off M150 showing broken spring: 0 if not, 1 for broken
volatile uint8_t StrainGauge = 0;
// Redundant
volatile uint8_t EBS_Test_Accel_Stop = 0;
// Stores the time since last heartbeat/message received from node
// PLEASE CHANGE TIMEOUT PERIODS (refer to timer interrupts) AND NODE IDS (refer to CAN source file) IF ERRORS ARE THROWN
volatile uint8_t HeartbeatCheck[3] = {0, 0, 0};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

// LV Systems turn on -------------------------------------

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  // Peripheral initialisation starts -------------------------------------------------------

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_CAN2_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6); // Initialise timer
  HAL_TIM_Base_Start_IT(&htim7); // Initialise timer

  // Peripheral initialisation complete -------------------------------------------------------

  Read_Rotary(&(State_Variables.R_S)); // Reads the current mission of the AV
  ASSI_Off(); // Ensures ASSI output is off

  /* Following is a testing block. Put a breakpoint at the first line when in debug mode, and when you press the resume
   * button, the code will run through this block. For CAN, probe the lines with the Saleas first. See if CAN frames
   * are being sent. For input pins, the state of them is updated in the indicators struct. If you hover over it
   * it will give a list of inputs (TS, SDC, ASMS). It will be 0 if low, 1 if high. The FAILURE_MODE struct will do
   * the same thing but for failure mode pins. Rotary Switch status is stored in indicators struct as well BUT
   * it is active high, so whichever mode is being used will need to be connected to ground.
   */
  while(1) {
	  CAN_Send_Message_String("OOGABOOG");
	  CAN_Send_Datalogger();
	  ReadInputs(indicators);
	  ReadForFaults(&FAILURE_MODE, indicators);
	  HAL_Delay(200);
  }

  // Read all inputs before
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* This is an implementation of a FSM in C. State outputs are executed at the start of every loop, along with the state attribute checks.
	 * A switch block is used for the state transitions for ever possible state.
	 */
	ReadInputs(indicators); // Updates all AV inputs
	ReadForFaults(&FAILURE_MODE, indicators); // Checks all fault readings from AV
	AV_State_Outputs(indicators, av_status, FAILURE_MODE); // Implements ASSI and EBS changes depending on current State

	switch(av_status) {
			case OFF:
				if(indicators->R_S != INSPECTION && indicators->TS == 1
						&& indicators->ASMS_Status == 1) {
					Initial_Checkup();
					av_status = READY;
					av_status_hex = 0x01;
					timecheck = __HAL_TIM_GET_COUNTER(&htim7);
				}
				break;

			case READY:
				if(FAILURE_MODE != NONE || RES == 1) {
					av_status_hex = 0x04;
					av_status = EMERGENCY;
				}
				else if(indicators->ASMS_Status == 0 && indicators->TS == 0 && indicators->EBS == 0) {
					av_status_hex = 0;
					av_status = OFF;
				}
				else if((__HAL_TIM_GET_COUNTER(&htim7) - timecheck) > 500 && indicators->R_S != MANUAL && R2D == 1) {
					if (indicators->R_S == TRACK) {
						CAN_Send_Message_String("DRIVING");  // Change this to whatever CAN message needs to be broadcast in Driving state
					}
					else if (indicators->R_S == EBS_TEST) {
						CAN_Send_Message_String("EBSTEST");  // Change this to whatever CAN message needs to be broadcast in EBS Test state
					}
					else if (indicators->R_S == INSPECTION) {
						CAN_Send_Message_String("INSPECT");  // Change this to whatever CAN message needs to be broadcast in Inspect state
					}
					av_status_hex = 0x02;
					av_status = DRIVING;
				}
				break;

			case DRIVING:
				if(FAILURE_MODE != NONE || RES == 1) {
					av_status_hex = 0x04;
					av_status = EMERGENCY;
				}
				else if(indicators->R_S == EBS_TEST) {
					if(EBS_Test_Accel_Stop == 1) {
						av_status_hex = 0x04;
						av_status = EMERGENCY;
					}
				}
				else if(Mission_Finished == 1) {
					EBSActivate(indicators);
					av_status_hex = 0x03;
					av_status = FINISHED;
				}
				break;

			case EMERGENCY:
				if(EBS_Sound == 0 && indicators->ASMS_Status == 0 && indicators->SB == 0 && indicators->TS == 0 && indicators->R_S == TRACK) {
					av_status = OFF;
					RES = 0;
					av_status_hex = 0;
					FAILURE_MODE = NONE;
				}
				break;

			case FINISHED:
				if(RES == 1) {
					av_status_hex = 0x04;
					av_status = EMERGENCY;
				}
				else if(indicators->ASMS_Status == 0 && indicators->SB == 0) {
					av_status_hex = 0;
					av_status = OFF;
				}
				break;
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle Watchdog
  if (htim == &htim7) // Heartbeat check may be faulty if nodes cannot meet timeout of 2s but still work. Scrap Heartbeat check in this case. Also refer to CAN source file to change Node ids (channel Ids) for respective nodes.
  {
	  int i;
	  	for (i = 0; i < 1; i++) {
	  		if ((__HAL_TIM_GET_COUNTER(&htim6) - HeartbeatCheck[i]) > 2000) {
	  			FAILURE_MODE = HEARTBEAT_FAULT;
	  			}
	  	}
  }

  if (htim == &htim7)
    {
  	  HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin); // Toggle watchdog control pin
    }
}

/**
  * @brief  This function is used to change all variable in the indicator struct to their current state.
  * @param  indicators is a pointer to the struct containing all state information.
  * @retval void
  */
void ReadInputs(AS__INDICATOR_STATES *indicators) {
	Read_Rotary(&(indicators->R_S));
	indicators->TS = HAL_GPIO_ReadPin(_3V3_TS_SWITCH_GPIO_Port, _3V3_TS_SWITCH_Pin);
	indicators->SB = Service_Brake_Check; // Redundant
	indicators->SA = Steering_Actuator_Check; // Redundant
	indicators->ASMS_Status = HAL_GPIO_ReadPin(ASMS_SIG_GPIO_Port, ASMS_SIG_Pin);
	EBSCheck(indicators);
}

/**
  * @brief  This function is used to handle the failure-specific information transmitted over CAN.
  * 		Depending on the Failure Mode, the response over CAN will be different.
  * @param  fm is an enum that corresponds to the different types of failures that can occur in the AV.
  * @retval void
  */
void SystemFailureHandler(FAILURE_MODE_READING fm) {
	switch (fm) {
		case NONE:
			CAN_Send_Message_String("No Err\n");
			break;
		case SDC_FAILURE:
			CAN_Send_Message_String("SDCFlt\n");
			break;
		case NON_PROG_FAILURE:
			CAN_Send_Message_String("NonPrg\n");
			break;
		case EBS_FAILURE:
			CAN_Send_Message_String("EBS   \n");
			break;
		case IMD_FAULT:
			CAN_Send_Message_String("IMD   \n");
			break;
		case BMS_FAULT:
			CAN_Send_Message_String("BMS   \n");
			break;
		case BSPD_FAULT:
			CAN_Send_Message_String("BSPD  \n");
			break;
		case HEARTBEAT_FAULT:
			CAN_Send_Message_String("NodeFLT");
			break;
		case SPRING_BROKE:
			CAN_Send_Message_String("SprgFLT");
			break;
	}
	EBSActivate(indicators);
	Error_Handler();
}

/**
  * @brief  This function is used to cycle through the ASSI and EBS outputs of the current AV State.
  * @param  fm is an enum that corresponds to the different types of failures that can occur in the AV.
  * 		indicators is a pointer to the struct containing all state information.
  * 		status is an enum that corresponds to the current state of the AV, according to the FSM in the .README documentation.
  * @retval void
  */
void AV_State_Outputs(AS__INDICATOR_STATES *indicators, AV_STATE status, FAILURE_MODE_READING FM) {
	switch(status) {
		case (READY):
				ASSI_Ready();
				memset(M150_State_Trigger, 0, sizeof(M150_State_Trigger)); // Clear M150 Trigger array
				M150_State_Trigger[0] = 0xFF;
				CAN_Send_Message_SpecificID(M150_State_Trigger, 0x415); // CHANGE THESE LINES: THEY TRANSMIT AV STATUS ON SPECIFIC M150 CHANNEL TO ENSURE OUTPUTS ON. CONFIRM WHICH CHANNEL RECEIVES AV STATUS MESSAGES ON M150
		case (DRIVING):
				ASSI_Driving();
				memset(M150_State_Trigger, 0, sizeof(M150_State_Trigger)); // Clear M150 Trigger array
				M150_State_Trigger[1] = 0xFF;
				CAN_Send_Message_SpecificID(M150_State_Trigger, 0x415); // CHANGE THESE LINES: THEY TRANSMIT AV STATUS ON SPECIFIC M150 CHANNEL TO ENSURE OUTPUTS ON. CONFIRM WHICH CHANNEL RECEIVES AV STATUS MESSAGES ON M150
		case (EMERGENCY):
				memset(M150_State_Trigger, 0, sizeof(M150_State_Trigger)); // Clear M150 Trigger array
				M150_State_Trigger[2] = 0xFF;
				SystemFailureHandler(FM);
				CAN_Send_Message_SpecificID(M150_State_Trigger, 0x415); // CHANGE THESE LINES: THEY TRANSMIT AV STATUS ON SPECIFIC M150 CHANNEL TO ENSURE OUTPUTS ON. CONFIRM WHICH CHANNEL RECEIVES AV STATUS MESSAGES ON M150
		case (FINISHED):
				ASSI_Finished();
				memset(M150_State_Trigger, 0, sizeof(M150_State_Trigger)); // Clear M150 Trigger array
				M150_State_Trigger[3] = 0xFF;
				CAN_Send_Message_SpecificID(M150_State_Trigger, 0x415); // CHANGE THESE LINES: THEY TRANSMIT AV STATUS ON SPECIFIC M150 CHANNEL TO ENSURE OUTPUTS ON. CONFIRM WHICH CHANNEL RECEIVES AV STATUS MESSAGES ON M150
		case (OFF):
				ASSI_Off();
				memset(M150_State_Trigger, 0, sizeof(M150_State_Trigger)); // Clear M150 Trigger array
				M150_State_Trigger[4] = 0xFF;
				CAN_Send_Message_SpecificID(M150_State_Trigger, 0x415); // CHANGE THESE LINES: THEY TRANSMIT AV STATUS ON SPECIFIC M150 CHANNEL TO ENSURE OUTPUTS ON. CONFIRM WHICH CHANNEL RECEIVES AV STATUS MESSAGES ON M150
	}
}

/**
  * @brief  This function is used to check what state the Rotary Switch is currently in.
  * @param  RS is a pointer to the Rotary Switch enum that corresponds to all the different states.
  * @retval void
  */
void Read_Rotary(ROT_SWITCH* RS) {
	// Rotary switch is active high
	if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(ROT_1_SIG_GPIO_Port, ROT_1_SIG_Pin)) {
		*RS = MANUAL;
	}
	else if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(ROT_2_SIG_GPIO_Port, ROT_2_SIG_Pin)) {
		*RS = TRACK;
	}
	else if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(ROT_3_SIG_GPIO_Port, ROT_3_SIG_Pin)) {
		*RS = EBS_TEST;
	}
	else if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(ROT_4_SIG_GPIO_Port, ROT_4_SIG_Pin)) {
		*RS = INSPECTION;
	}
}

/**
  * @brief  This function reads the status of IMD, BSPD and BMS failure pins and changes the corresponding flags.
  * @param  fm is an enum that corresponds to the different types of failures that can occur in the AV.
  *			indicators is a pointer to the struct containing all state information.
  * @retval void
  */
void ReadForFaults(FAILURE_MODE_READING *Fm, AS__INDICATOR_STATES *indicators) {
	if(HAL_GPIO_ReadPin(BMS_FAULT_GPIO_Port, BMS_FAULT_Pin) == 1) {
		*Fm = BMS_FAULT;
	}
	else if(HAL_GPIO_ReadPin(BSPD_FLT_GPIO_Port, BSPD_FLT_Pin) == 1) {
		*Fm = BSPD_FAULT;
	}
	else if(HAL_GPIO_ReadPin(IMD_FAULT_GPIO_Port, IMD_FAULT_Pin) == 1) {
		*Fm = IMD_FAULT;
	}
	else if(indicators->EBS == 0) {
		*Fm = EBS_FAILURE;
	}
	else if(HAL_GPIO_ReadPin(SDC_CHECK_GPIO_Port, SDC_CHECK_Pin) == 0) {
		*Fm = SDC_FAILURE;
	}
	else if(StrainGauge == 1) {
		*Fm = SPRING_BROKE;
	}
}

/**
  * @brief  This function is used to activate the EBS by setting the EBS actuator pin high. Also activates the redundant system if EBS not available.
  * @param  indicators is a pointer to the struct containing all state information.
  * @retval void
  */
void EBSActivate(AS__INDICATOR_STATES *indicators) {
	EBSCheck(indicators);
	if (indicators->EBS == ACTIVATED) {
		/* Send a message over CAN to activate redundant system*/
		CAN_Send_Message_String("RDNDNT\n");
	}
	else if (indicators->EBS == DEACTIVATED) {
		HAL_GPIO_WritePin(AS_CLOSE_SDC_GPIO_Port, AS_CLOSE_SDC_Pin, GPIO_PIN_RESET);
	}
}

/**
  * @brief  This function is used to deactivate the EBS by enabling the SDC, which will happen if failures have been resolved.
  * @param  indicators is a pointer to the struct containing all state information.
  * @retval void
  */
void EBSReset(AS__INDICATOR_STATES *indicators) {
	EBSCheck(indicators);
	if (indicators->EBS == ACTIVATED) {
		HAL_GPIO_WritePin(AS_CLOSE_SDC_GPIO_Port, AS_CLOSE_SDC_Pin, GPIO_PIN_SET);
	}
}

/**
  * @brief  Performs initial checkup before going into Ready State from Idle.
  * @param  void
  * @retval void
  */
void EBSCheck(AS__INDICATOR_STATES *indicators) {
	if (EBS_Brake_Line == 1) {
		indicators->EBS = ACTIVATED;
	}
	else if (EBS_Brake_Line == 0) {
		indicators->EBS = DEACTIVATED;
	}
}

/**
  * @brief  This function is used to deactivate the EBS by enabling the SDC, which will happen if failures have been resolved.
  * @param  indicators is a pointer to the struct containing all state information.
  * @retval void
  */
void Initial_Checkup(void) {
	  timecheck = __HAL_TIM_GET_COUNTER(&htim7); // You will see this often. This is me setting the current count of the timer to the timecheck variable for future comparison.
	  while (HAL_GPIO_ReadPin(SDC_CHECK_GPIO_Port, SDC_CHECK_Pin) == GPIO_PIN_RESET) {
		  if (__HAL_TIM_GET_COUNTER(&htim7) - timecheck > 150) {
			  // Timeout Condition for if the SDC does not go to high.
			  FAILURE_MODE = SDC_FAILURE;
			  SystemFailureHandler(FAILURE_MODE);
		  }
	  }

	  timecheck = __HAL_TIM_GET_COUNTER(&htim7);
	  CAN_Bus_Limiter = 0; // reset the CAN Bus Error Limiter
	  HAL_NVIC_DisableIRQ(TIM7_IRQn); // Disable Watchdog interrupts
	  HAL_Delay(200);
	  while (HAL_GPIO_ReadPin(SDC_CHECK_GPIO_Port, SDC_CHECK_Pin) == 1) {
		  if (__HAL_TIM_GET_COUNTER(&htim7) - timecheck > 150) {
			  FAILURE_MODE = NON_PROG_FAILURE;
			  SystemFailureHandler(FAILURE_MODE);
		  }
	  }

	  HAL_NVIC_EnableIRQ(TIM7_IRQn); // Enable Watchdog interrupts
	  timecheck = __HAL_TIM_GET_COUNTER(&htim7);
	  while (EBS_Energy_Check == 0) {
		  if (__HAL_TIM_GET_COUNTER(&htim7) - timecheck > 150) {
			  // Timeout Condition if EBS Cannister and line Pressure are not built up.
			  FAILURE_MODE = EBS_FAILURE;
			  SystemFailureHandler(FAILURE_MODE);
		  }
	  }

	  HAL_GPIO_WritePin(AS_CLOSE_SDC_GPIO_Port, AS_CLOSE_SDC_Pin, GPIO_PIN_SET); // Close SDC Relay once checks have been performed.

	  av_status = READY; // Transition into READY state
	  timecheck = __HAL_TIM_GET_COUNTER(&htim7);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  ASSI_Emergency();
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
