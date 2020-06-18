/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "arm_math.h"
#include "canopen_slim/canopenSlim.h"

#ifndef PI
#define PI 3.14159265358979f
#endif
#ifndef PI_2
#define PI_2 6.28318530717959f
#endif
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
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
COSLM_PDOStruct readPDO[2];
COSLM_PDOStruct sendPDO[2];
int32_t readPOS[2];
int32_t sendPOS[2];

uint8_t rxData;

CAN_RxHeaderTypeDef   Rx0Header;
uint8_t               Rx0Data[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx0Header, Rx0Data) == HAL_OK){
    canopenSlim_addRxBuffer(Rx0Header.StdId, Rx0Data);
  }  
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim == &htim1){
    
    switch(rxData){
    case 'q':
      sendPOS[0] += 100;
      rxData = 0x00;
      break;
      
    case 'w':
      sendPOS[0] -= 100;
      rxData = 0x00;
      break;
      
    case 'a':
      sendPOS[1] += 100;
      rxData = 0x00;
      break;
      
    case 's':
      sendPOS[1] -= 100;
      rxData = 0x00;
      break;
      
    case 'z':
      canopenSlim_writeOD_uint8(0x22, 0x2002, 0x07, 0x01, 2);
      canopenSlim_writeOD_uint8(0x23, 0x2002, 0x07, 0x01, 2);
      sendPOS[0] = 0;
      sendPOS[1] = 0;
      rxData = 0x00;
    }
    
    // [[ Send CANOpen Frame ]]
    canopenSlim_sendPDO(0x22, 1, &sendPDO[0]);
    canopenSlim_sendPDO(0x23, 1, &sendPDO[1]);
    canopenSlim_sendSync();
    canopenSlim_readPDO(0x22, 1, &readPDO[0], 2);
    canopenSlim_readPDO(0x23, 1, &readPDO[1], 2);
    
  }else if(htim == &htim2){
    canopenSlim_timerLoop();
  }
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  
  // [[ CAN Init ]]
  // This mcu is CANOpen Master. So it should receive all can frame.
  CAN_FilterTypeDef sFilterConfig0;
  sFilterConfig0.FilterBank = 0;
  sFilterConfig0.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig0.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig0.FilterIdLow = 0x0000;
  sFilterConfig0.FilterMaskIdLow = 0x0000;
  sFilterConfig0.FilterIdHigh = 0x0000;
  sFilterConfig0.FilterMaskIdHigh = 0x0000;
  sFilterConfig0.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig0.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig0);

  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  
  // [[ Timer2 Init ]]
  // Start Timer2 to check timeout of CANOpen response (10kHz)
  HAL_TIM_Base_Start_IT(&htim2);

  // Check [http://info.bluesink.io] to check Object Dictionary Elements.
  // Stop motor drivers
  // OD[2000, 04] = 0x00 (Set Motor driver to Stop state)
  canopenSlim_writeOD_uint8(0x22, 0x2000, 0x04, 0x00, 100);
  canopenSlim_writeOD_uint8(0x23, 0x2000, 0x04, 0x00, 100);
  while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET);
  
  // Startup Batch
  // [[ 0x22 ]]
  // Set TPDO
  // 1. OD[1800, 01] = 0x80000180 + Node ID (Disable TPDO)
  // 2. OD[1A00 ,00] = 0x00 (Disable TPDO Mapping)
  // 3. OD[1A00, 01] = 0x20010220 (Mapping OD[2001, 02] to first entry)
  // 4. OD[1A00, 00] = 0x01 (Enable TPDO Mapping)
  // 5. OD[1800, 01] = 0x180 + Node ID (Enable TPDO)
  canopenSlim_writeOD_uint32(0x22, 0x1800, 0x01, 0x22 | 0x80000180, 100);
  canopenSlim_writeOD_uint8(0x22, 0x1A00, 0x00, 0, 100);
  canopenSlim_writeOD_uint32(0x22, 0x1A00, 0x01, 0x20010220, 100); 
  canopenSlim_writeOD_uint8(0x22, 0x1A00, 0x00, 1, 100);
  canopenSlim_writeOD_uint32(0x22, 0x1800, 0x01, 0x22 | 0x180, 100);

  // Set RPDO
  // 1. OD[1400, 01] = 0x80000200 + Node ID (Disable RPDO)
  // 2. OD[1600 ,00] = 0x00 (Disable RPDO Mapping)
  // 3. OD[1600, 01] = 0x20000220 (Mapping OD[2000, 02] to first entry)
  // 4. OD[1600, 00] = 0x01 (Enable RPDO Mapping)
  // 5. OD[1400, 01] = 0x200 + Node ID (Enable RPDO)  
  canopenSlim_writeOD_uint32(0x22, 0x1400, 0x01, 0x22 | 0x80000200, 100);
  canopenSlim_writeOD_uint8(0x22, 0x1600, 0x00, 0, 100);
  canopenSlim_writeOD_uint32(0x22, 0x1600, 0x01, 0x20000220, 100); 
  canopenSlim_writeOD_uint8(0x22, 0x1600, 0x00, 1, 100);
  canopenSlim_writeOD_uint32(0x22, 0x1400, 0x01, 0x22 | 0x200, 100);  
  
  
  canopenSlim_mappingPDO_init(&sendPDO[0]);
  canopenSlim_mappingPDO_int32(&sendPDO[0], &sendPOS[0]);
  
  canopenSlim_mappingPDO_init(&readPDO[0]);
  canopenSlim_mappingPDO_int32(&readPDO[0], &readPOS[0]);

    // [[ 0x23 ]]
  // 1. OD[1800, 01] = 0x80000180 + Node ID (Disable TPDO)
  // 2. OD[1A00 ,00] = 0x00 (Disable TPDO Mapping)
  // 3. OD[1A00, 01] = 0x20010220 (Mapping OD[2001, 02] to first entry)
  // 4. OD[1A00, 00] = 0x01 (Enable TPDO Mapping)
  // 5. OD[1800, 01] = 0x180 + Node ID (Enable TPDO)
  canopenSlim_writeOD_uint32(0x23, 0x1800, 0x01, 0x23 | 0x80000180, 100);
  canopenSlim_writeOD_uint8(0x23, 0x1A00, 0x00, 0, 100);
  canopenSlim_writeOD_uint32(0x23, 0x1A00, 0x01, 0x20010220, 100); 
  canopenSlim_writeOD_uint8(0x23, 0x1A00, 0x00, 1, 100);
  canopenSlim_writeOD_uint32(0x23, 0x1800, 0x01, 0x23 | 0x180, 100);

  // 1. OD[1400, 01] = 0x80000200 + Node ID (Disable RPDO)
  // 2. OD[1600 ,00] = 0x00 (Disable RPDO Mapping)
  // 3. OD[1600, 01] = 0x20000220 (Mapping OD[2000, 02] to first entry)
  // 4. OD[1600, 00] = 0x01 (Enable RPDO Mapping)
  // 5. OD[1400, 01] = 0x200 + Node ID (Enable RPDO)    
  canopenSlim_writeOD_uint32(0x23, 0x1400, 0x01, 0x23 | 0x80000200, 100);
  canopenSlim_writeOD_uint8(0x23, 0x1600, 0x00, 0, 100);
  canopenSlim_writeOD_uint32(0x23, 0x1600, 0x01, 0x20000220, 100); 
  canopenSlim_writeOD_uint8(0x23, 0x1600, 0x00, 1, 100);
  canopenSlim_writeOD_uint32(0x23, 0x1400, 0x01, 0x23 | 0x200, 100);  

  canopenSlim_mappingPDO_init(&sendPDO[1]);
  canopenSlim_mappingPDO_int32(&sendPDO[1], &sendPOS[1]);
  
  canopenSlim_mappingPDO_init(&readPDO[1]);
  canopenSlim_mappingPDO_int32(&readPDO[1], &readPOS[1]);
  
  // [[ Go to speed control state ]]
  // OD[2000, 04] = 0x04 (Set Motor driver to Position control state)
  canopenSlim_writeOD_uint8(0x22, 0x2000, 0x04, 0x04, 100);
  canopenSlim_writeOD_uint8(0x23, 0x2000, 0x04, 0x04, 100);
  
  // [[ Main timer start ]]
  // Start main control timer (1kHz)
  HAL_TIM_Base_Start_IT(&htim1);
  
  // [[ UART Enable ]]
  HAL_UART_Receive_DMA(&huart2, &rxData, 1);

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
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

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 29999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
