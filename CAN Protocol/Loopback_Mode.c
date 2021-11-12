/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @programmar     : Kanishk Kaushal
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* IN CAN PROTOCOL USING STM32, IF INCOMING DATA CORRESPONDS TO THE FILTER ID ONLY THEN DATA WILL BE ALLOWED TO PASS */

/* IF FILTER ID != (STANDARD ID) << 5 | DATA WILL NOT PASS

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint32_t TxMailbox;

uint8_t TxData[8];
uint8_t RxData[8];

uint8_t count = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	count++;
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  HAL_CAN_Start(&hcan);

  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  TxHeader.DLC = 1;     	    		 // 1 BYTE DATA LENGTH | SPECIFIES LENGTH OF DATA THAT NEEDS TO BE SENT
  TxHeader.ExtId = 0;		    		 // ExtId -> EXTENDED ID | BASIC CAN PROTOCOL
  TxHeader.IDE = CAN_ID_STD;    		 // IDE -> IDENTIFIER | STANDARD CAN PROTOCOL | SPECIFIES THE TYPE OF IDENTIFIER FOR THE MESSAGE THAT WILL BE TRANSMITTED
  TxHeader.RTR = CAN_RTR_DATA;  		 // RTR -> REMOTE TRANSMISSION REQUEST | SPECIFIES IF WE ARE TRASNMITTING DATA OR REMOTE FRAME
  TxHeader.StdId = 0x103;				 // StdId -> STANDARD IDENTIFIER | OVER THE CAN BUS EVERY NODE RECEIVES THIS ID
  TxHeader.TransmitGlobalTime = DISABLE; // SPECIFIES GLOBAL TIME

  TxData[0] = 0xf3; // WE WILL SEND THIS DATA TO CAN

  // TO SEND THE DATA WE USE:
  HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

  /* Txmailbox -> TRANSMIT MAIL BOX
   * EMPTY AT FIRST, THEN WE WRITE SOME DATA INTO IT
   * ONCE WE DO THE TRANSMIT, IT GOES INTO THE PENDING STATE
   * FROM THERE IT GETS SCHEDULED AND IS FINALLY TRANSMITTED */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */

static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  CAN_FilterTypeDef canfilterconfig;


  // CAN FILTERS:

  // THESE FILTERS ARE USED TO FILTER THE INCOMING DATA
  // THESE FILTERS ALLOW DATA TO PASS THROUGH ONLY IF IT FOLLOWS CERTAIN CONDITIONS

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE; // ENABLE OR DISABLE THE FILTER
  canfilterconfig.FilterBank = 10;						// SPECIFIES WHICH FILTER BANK WE WANT TO USE | 0 - 13 FOR ONE CAN NODE
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;	// INCOMING DATA W2ILL BE STORED TO FIFO0 | SPECIFIES WHICH FIFO MUST BE ASSIGNED TO THIS SPECIFIC FILTER

  /* WE HAVE 2 FILTER MODES : 1. MASK & 2. IDENTIFIER LIST
   * WE USE MASK MODE
   * MASK MODE ALLOWS US TO LOOK FOR PARTICULAR BITS IN THE IDENTIFIER */

  canfilterconfig.FilterIdHigh = 0x103<<5; 				// REFER TO LINE 232
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x103<<5; 			// LEFT SHIFTED BY 5 AS 5 BITS ARE CONSUMED BY THE EXTENDED ID's BITS
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;	// WE WILL BE USING 32 BIT SCALE -> ALL OUR REGISTERS WILL BE OF 32 BITS

  /* THE OTHER MODE IS "IDENTIFIER LIST"
   * INSTEAD IF HAVING 1 MASK ID REGISTER, WE HAVE 2 ID REGISTERS
   * ALL BITS OF THE INCOMING IDENTIFIERS MUST MATCH THE BITS SPECIFIED IN THE FILTER REGISTERS */

  // IF WE HAVE 2 CAN NODES THEN 0 - 13 ARE ASSIGNED TO THE MASTER CAN & 13 - 27 ARE ASSIGNED TO THE SLAVE CAN
  // FOR SINGLE CAN INTERFACE THIS IS USELESS

  canfilterconfig.SlaveStartFilterBank = 13;

  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */

static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
