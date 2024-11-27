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
AV_STATE av_status = OFF; // Tracks current state of AV
FAILURE_MODE_READING FAILURE_MODE = NONE; // Stores current error of the AV for error message
AS__INDICATOR_STATES State_Variables; // Stores current state of AV inputs
AS__INDICATOR_STATES* indicators = &State_Variables;

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
uint16_t timecheck = 0; // Stores time in order to trigger timeout errors
uint8_t CAN_Bus_Limiter = 0; // Acts as a boolean that prevents a large number of error messages being dumped into the CAN Bus.
int ASMS_Status = 0; // Stores state of ASMS.
// Global flags altered by CAN receive interrupts ----------------
volatile int EBS_Energy_Check = 0;
volatile int EBS_Pressure_Check = 0;
volatile int Service_Brake_Check = 0;
volatile int Steering_Actuator_Check = 0;
volatile int Mission_Finished = 0;
volatile int EBS_Sound = 0;
volatile int RES = 0;
volatile int length = 0;
volatile int R2D = 0;
volatile int EBS_Brake_Line = 0;
volatile int StrainGauge = 0;
volatile int EBS_Test_Accel_Stop = 0;
volatile uint8_t HeartbeatCheck[3] = {0, 0, 0};
volatile uint8_t OVERRIDE = 0;
// --------------------------------------------------------------------
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
// LV Systems turn on -------------------------------------
  Read_Rotary(&(State_Variables.R_S));
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

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
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
	  Error_Handler();
  }
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  while (1) {
	  CAN_Send_Message_String("HELLOSY");
	  HAL_Delay(200);
  }
  ASSI_Off();
  av_status = OFF; // Initial state is Off.

  // Read all inputs before
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* This is an implementation of a FSM in C. State outputs are executed at the start of every loop, along with the state attribute checks.
	 * A switch block is used for the state transitions for ever possible state. There are while loops used in transition statements when the new state attributes rely on external control system changes.
	 */
	ReadInputs(indicators);
	ReadForFaults(&FAILURE_MODE, indicators);
	AV_State_Outputs(indicators, av_status, FAILURE_MODE);

	switch(av_status) {
			case OFF:
				if(indicators->R_S != ROT_4 && indicators->TS == 1
						&& ASMS_Status == 1) {
					CAN_Send_Message_String("READY \n");
				//	while (indicators->SB != 1 && indicators->SA != 1) {}
					Initial_Checkup();
					av_status = READY;
					timecheck = __HAL_TIM_GET_COUNTER(&htim7);
				}
				break;

			case READY:
				if(FAILURE_MODE != NONE || RES == 1) {
					CAN_Send_Message_String("EMGNCY\n");
					CAN_Send_Message_String("TS OFF\n");
					av_status = EMERGENCY;
				}
				else if(ASMS_Status == 0 && indicators->TS == 0 && indicators->EBS == 0) {
					CAN_Send_Message_String("OFF   \n");
					while(indicators->SB != 0 && indicators->SA != 0) {}
					av_status = OFF;
				}
				else if((__HAL_TIM_GET_COUNTER(&htim7) - timecheck) > 500 && indicators->R_S != ROT_1 && R2D == 1) {
					if (indicators->R_S == ROT_2) {
						CAN_Send_Message_String("EBSTEST");
					}
					else if (indicators->R_S == ROT_3) {
						CAN_Send_Message_String("INSPECT");
					}
					else if (indicators->R_S == ROT_4) {
						CAN_Send_Message_String("DRIVING");
					}
			//		while(indicators->EBS == ACTIVATED && indicators->SB != 1 && indicators->SA != 1)
					av_status = DRIVING;
				}
				break;

			case DRIVING:
				if(FAILURE_MODE != NONE || RES == 1) {
					CAN_Send_Message_String("EMGNCY\n");
					CAN_Send_Message_String("TS OFF\n");
					CAN_Send_Message_String("DS OFF\n");
					av_status = EMERGENCY;
				}
				else if(indicators->R_S == ROT_2) {
					if(EBS_Test_Accel_Stop == 1) {
						av_status = EMERGENCY;
					}
				}
				else if(Mission_Finished == 1) {
					EBSActivate(indicators);
					CAN_Send_Message_String("FINISH\n");
					CAN_Send_Message_String("TS OFF\n");
					CAN_Send_Message_String("DS OFF\n");
				//	while(indicators->SA != 0 && indicators->TS != 0 && indicators->R_S != 2) {}
					av_status = FINISHED;
				}
				break;

			case MANUAL:
				if(indicators->TS == 0) {
					av_status = OFF;
					CAN_Send_Message_String("OFF   \n");
				}
				break;

			case EMERGENCY:
				if(EBS_Sound == 0 && ASMS_Status == 0 && indicators->SB == 0 && indicators->TS == 0 && indicators->R_S == ROT_2) {
					av_status = OFF;
					RES = 0;
					CAN_Send_Message_String("OFF   \n");
					FAILURE_MODE = NONE;
				}
				break;

			case FINISHED:
				if(RES == 1) {
					CAN_Send_Message_String("EMGNCY\n");
					av_status = EMERGENCY;
				}
				else if(ASMS_Status == 0 && indicators->SB == 0) {
					CAN_Send_Message_String("OFF   \n");
					while(indicators->TS != 0 && indicators->R_S != ROT_2) {}
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
  if (htim == &htim7)
  {
	  int i;
	  	for (i = 0; i < 3; i++) {
	  		if ((__HAL_TIM_GET_COUNTER(&htim6) - HeartbeatCheck[i]) > 200 || OVERRIDE == 1) {
	  			FAILURE_MODE = HEARTBEAT_FAULT;
	  			}
	  	}
  }

  if (htim == &htim7)
    {
  	  HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
    }
}

void ReadInputs(AS__INDICATOR_STATES *indicators) {
	Read_Rotary(&(indicators->R_S));
	indicators->TS = HAL_GPIO_ReadPin(_3V3_TS_SWITCH_GPIO_Port, _3V3_TS_SWITCH_Pin);
	indicators->SB = Service_Brake_Check;
	indicators->SA = Steering_Actuator_Check;
	EBSCheck(indicators);
	ASMS_Status = HAL_GPIO_ReadPin(ASMS_SIG_GPIO_Port, ASMS_SIG_Pin);
}

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

void AV_State_Outputs(AS__INDICATOR_STATES *indicators, AV_STATE status, FAILURE_MODE_READING FM) {
	switch(status) {
		case (READY):
				ASSI_Ready();
		case (DRIVING):
				ASSI_Driving();
		case (EMERGENCY):
				SystemFailureHandler(FM);
		case (FINISHED):
				ASSI_Finished();
		case (OFF):
				ASSI_Off();
		case (MANUAL):
				ASSI_Off();
	}

	if(ASMS_Status == 0 || (av_status != OFF && av_status != MANUAL)) {
		av_status = OFF;
	}
}

void Read_Rotary(ROT_SWITCH* RS) {
	if(GPIO_PIN_SET == HAL_GPIO_ReadPin(ROT_1_SIG_GPIO_Port, ROT_1_SIG_Pin)) {
		*RS = ROT_1;
	}
	else if (GPIO_PIN_SET == HAL_GPIO_ReadPin(ROT_2_SIG_GPIO_Port, ROT_2_SIG_Pin)) {
		*RS = ROT_2;
	}
	else if (GPIO_PIN_SET == HAL_GPIO_ReadPin(ROT_3_SIG_GPIO_Port, ROT_3_SIG_Pin)) {
		*RS = ROT_3;
	}
	else if (GPIO_PIN_SET == HAL_GPIO_ReadPin(ROT_4_SIG_GPIO_Port, ROT_4_SIG_Pin)) {
		*RS = ROT_4;
	}
}

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

void EBSReset(AS__INDICATOR_STATES *indicators) {
	EBSCheck(indicators);
	if (indicators->EBS == ACTIVATED) {
		HAL_GPIO_WritePin(AS_CLOSE_SDC_GPIO_Port, AS_CLOSE_SDC_Pin, GPIO_PIN_SET);
	}
}

void EBSCheck(AS__INDICATOR_STATES *indicators) {
	if (EBS_Energy_Check == 1 && EBS_Brake_Line == 1) {
		indicators->EBS = ACTIVATED;
	}
	else if (EBS_Energy_Check == 0 || EBS_Pressure_Check == 0) {
		indicators->EBS = DEACTIVATED;
	}
}

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
