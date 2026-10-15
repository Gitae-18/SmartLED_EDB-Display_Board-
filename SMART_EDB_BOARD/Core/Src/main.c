/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc_ctrl.h"
#include "bl0942.h"
#include "mcs1806.h"
#include "power_monitor.h"
#include "rs485.h"
#include "display_protocol.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONSOLE_REPORT_INTERVAL_MS  1000U
#define CONSOLE_RS485_TEST_PEER_ID  0U
#define CONSOLE_TX_TIMEOUT_MS       100U
#define RUN_LED_TOGGLE_INTERVAL_MS  1000U
#define RS485_ID_SWITCH_ACTIVE_LOW  0U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_NodeTypeDef Node_GPDMA1_Channel5;
DMA_QListTypeDef List_GPDMA1_Channel5;
DMA_HandleTypeDef handle_GPDMA1_Channel5;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
static uint32_t console_last_report_ms;
static uint32_t run_led_last_toggle_ms;
static uint8_t console_rx_byte;
static uint8_t board_dip_id;
static bool board_dip_id_valid;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
static void Console_Init(void);
static void Console_Process(void);
static void Console_PrintStatus(void);
static void Console_RunRS485SelfTest(void);
static void RunLed_Process(void);
static uint8_t Board_ReadRS485Id(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int32_t Console_ToMilli(float value)
{
  float scaled = value * 1000.0f;

  if (scaled >= 2147483647.0f)
  {
    return INT32_MAX;
  }
  if (scaled <= -2147483648.0f)
  {
    return INT32_MIN;
  }
  return (int32_t)scaled;
}

static void Console_Write(const char *text)
{
  if (text != NULL)
  {
    (void)HAL_UART_Transmit(&huart6, (const uint8_t *)text,
                            (uint16_t)strlen(text), CONSOLE_TX_TIMEOUT_MS);
  }
}

static uint8_t Board_ReadRS485Id(void)
{
  static GPIO_TypeDef *const ports[8] = {
      RS485_ID0_GPIO_Port, RS485_ID1_GPIO_Port,
      RS485_ID2_GPIO_Port, RS485_ID3_GPIO_Port,
      RS485_ID4_GPIO_Port, RS485_ID5_GPIO_Port,
      RS485_ID6_GPIO_Port, RS485_ID7_GPIO_Port
  };
  static const uint16_t pins[8] = {
      RS485_ID0_Pin, RS485_ID1_Pin, RS485_ID2_Pin, RS485_ID3_Pin,
      RS485_ID4_Pin, RS485_ID5_Pin, RS485_ID6_Pin, RS485_ID7_Pin
  };
  uint8_t id = 0U;
  uint8_t bit;

  for (bit = 0U; bit < 8U; ++bit)
  {
    bool pin_high = (HAL_GPIO_ReadPin(ports[bit], pins[bit]) == GPIO_PIN_SET);
    bool switch_on = (RS485_ID_SWITCH_ACTIVE_LOW != 0U) ? !pin_high : pin_high;

    if (switch_on)
    {
      id |= (uint8_t)(1U << bit);
    }
  }
  return id;
}

static void RunLed_Process(void)
{
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - run_led_last_toggle_ms) >=
      RUN_LED_TOGGLE_INTERVAL_MS)
  {
    run_led_last_toggle_ms = now;
    HAL_GPIO_TogglePin(RUN_LED_GPIO_Port, RUN_LED_Pin);
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  /* Application module initialization */
  ADC_CTRL_Init();
  BL0942_Init();
  MCS1806_Init();
  RS485_Init();
  board_dip_id = Board_ReadRS485Id();
  board_dip_id_valid = ((board_dip_id > 0U) &&
                        (board_dip_id < RS485_BROADCAST_ID));
  if (board_dip_id_valid)
  {
    RS485_SetLocalId(board_dip_id);
  }
  PowerMonitor_Init();
  DisplayProtocol_Init();
  Console_Init();
  run_led_last_toggle_ms = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    PowerMonitor_Update();
    DisplayProtocol_Process();
    Console_Process();
    RunLed_Process();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Configure LSE Drive Capability
  *  Warning : Only applied when the LSE is disabled.
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 125;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADCDAC;
  PeriphClkInitStruct.PLL2.PLL2Source = RCC_PLL2_SOURCE_HSE;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 100;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 23;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2_VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2_VCORANGE_WIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.PLL2.PLL2ClockOut = RCC_PLL2_DIVR;
  PeriphClkInitStruct.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_PLL2R;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Disable Injected Queue
  */
  HAL_ADCEx_DisableInjectedQueue(&hadc1);

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_5;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;
  sConfigInjected.InjecOversamplingMode = ENABLE;
  sConfigInjected.InjecOversampling.Ratio = ADC_OVERSAMPLING_RATIO_2;
  sConfigInjected.InjecOversampling.RightBitShift = ADC_RIGHTBITSHIFT_1;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel5_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_PrivilegeStateTypeDef privilegeState = {0};
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  privilegeState.rtcPrivilegeFull = RTC_PRIVILEGE_FULL_NO;
  privilegeState.backupRegisterPrivZone = RTC_PRIVILEGE_BKUP_ZONE_NONE;
  privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
  privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
  if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the TimeStamp
  */
  if (HAL_RTCEx_SetTimeStamp(&hrtc, RTC_TIMESTAMPEDGE_RISING, RTC_TIMESTAMPPIN_DEFAULT) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  htim2.Init.Period = 276;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 624;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RX_ENABLE_Pin|RUN_LED_Pin|RS485_DE_Pin|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AC_Ctrl_GPIO_Port, AC_Ctrl_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : TEST_LED_Pin */
  GPIO_InitStruct.Pin = TEST_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TEST_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RX_ENABLE_Pin AC_Ctrl_Pin RUN_LED_Pin RS485_DE_Pin
                           PA12 */
  GPIO_InitStruct.Pin = RX_ENABLE_Pin|AC_Ctrl_Pin|RUN_LED_Pin|RS485_DE_Pin
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* The DIP switch drives a selected ID bit high; idle bits stay low. */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Pin = RS485_ID0_Pin|RS485_ID1_Pin|RS485_ID2_Pin|RS485_ID3_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RS485_ID4_Pin|RS485_ID5_Pin|RS485_ID6_Pin;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RS485_ID7_Pin;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void Console_PrintSignedMilli(char *buffer, size_t capacity,
                                     const char *name, int32_t milli,
                                     const char *unit)
{
  uint32_t magnitude;

  if ((buffer == NULL) || (capacity == 0U))
  {
    return;
  }

  magnitude = (milli < 0) ? (uint32_t)(-(int64_t)milli) : (uint32_t)milli;
  (void)snprintf(buffer, capacity, "%s=%s%lu.%03lu%s",
                 name, (milli < 0) ? "-" : "",
                 (unsigned long)(magnitude / 1000U),
                 (unsigned long)(magnitude % 1000U), unit);
}

static void Console_PrintStatus(void)
{
  const PowerMonitor_Data_t *power = PowerMonitor_GetData();
  const DisplayProtocol_Status_t *protocol = DisplayProtocol_GetStatus();
  const BL0942_Diagnostics_t *meter = BL0942_GetDiagnostics();
  char line[384];
  char voltage[32];
  char current[32];
  char auxiliary[32];
  char dc_voltage[32];
  char ntc_voltage[32];
  char temperature[32];
  char power_text[32];
  char frequency[32];
  uint32_t current_gain;
  int length;

  switch (meter->gain_cr_raw & 0x03U)
  {
    case 0U: current_gain = 1U; break;
    case 1U: current_gain = 4U; break;
    case 2U: current_gain = 16U; break;
    default: current_gain = 24U; break;
  }

  Console_PrintSignedMilli(voltage, sizeof(voltage), "voltage_rms",
                           Console_ToMilli(power->voltage_rms), "V");
  Console_PrintSignedMilli(current, sizeof(current), "current_rms",
                           Console_ToMilli(power->current_rms), "A");
  Console_PrintSignedMilli(auxiliary, sizeof(auxiliary), "current",
                           Console_ToMilli(power->auxiliary_current), "A");
  Console_PrintSignedMilli(dc_voltage, sizeof(dc_voltage), "voltage",
                           Console_ToMilli(power->dc_voltage), "V");
  Console_PrintSignedMilli(ntc_voltage, sizeof(ntc_voltage), "adc_voltage",
                           Console_ToMilli(power->ntc_voltage), "V");
  Console_PrintSignedMilli(temperature, sizeof(temperature), "temperature",
                           Console_ToMilli(power->ntc_temperature_c), "C");
  Console_PrintSignedMilli(power_text, sizeof(power_text), "active_power",
                           Console_ToMilli(power->active_power), "W");
  Console_PrintSignedMilli(frequency, sizeof(frequency), "frequency",
                           Console_ToMilli(power->frequency), "Hz");

  length = snprintf(line, sizeof(line),
      "[STATUS AC uptime=%lums] sensor=BL0942 read_status=%s "
      "%s %s %s %s update_success=%lu update_fail=%lu\r\n",
      (unsigned long)HAL_GetTick(), power->meter_valid ? "OK" : "FAIL",
      voltage, current, power_text, frequency,
      (unsigned long)power->successful_updates,
      (unsigned long)power->failed_updates);
  if (length > 0)
  {
    Console_Write(line);
  }

  length = snprintf(line, sizeof(line),
      "[STATUS DC] voltage_status=%s %s current_sensor=MCS1806 %s\r\n",
      power->dc_voltage_valid ? "OK" : "FAIL",
      dc_voltage, auxiliary);
  if (length > 0)
  {
    Console_Write(line);
  }

  length = snprintf(line, sizeof(line),
      "[STATUS NTC] read_status=%s %s %s adc_raw=%u\r\n",
      power->ntc_valid ? "OK" : "FAIL",
      temperature, ntc_voltage, (unsigned int)power->ntc_raw);
  if (length > 0)
  {
    Console_Write(line);
  }

  length = snprintf(line, sizeof(line),
      "[STATUS RS485] node_id=%u rx_packets=%lu tx_packets=%lu "
      "rx_responses=%lu last_response_cmd=0x%02X protocol_errors=%lu\r\n",
      (unsigned int)RS485_GetLocalId(),
      (unsigned long)protocol->received_packets,
      (unsigned long)protocol->transmitted_packets,
      (unsigned long)protocol->received_responses,
      (unsigned int)protocol->last_response_command,
      (unsigned long)protocol->protocol_errors);
  if (length > 0)
  {
    Console_Write(line);
  }

  length = snprintf(line, sizeof(line),
      "[BL0942 DIAG] last_result=%s register=0x%02X hal_status=%u uart_error=0x%lX "
      "tx_failures=%lu rx_timeouts=%lu rx_errors=%lu checksum_errors=%lu "
      "rx_bytes=%u/4 response_hex=%02X %02X %02X %02X "
      "checksum_expected=%02X checksum_received=%02X\r\n",
      BL0942_ResultName(meter->last_result), (unsigned int)meter->reg,
      (unsigned int)meter->hal_status, (unsigned long)meter->uart_error,
      (unsigned long)meter->tx_failures, (unsigned long)meter->rx_timeouts,
      (unsigned long)meter->rx_errors, (unsigned long)meter->checksum_errors,
      (unsigned int)meter->received_bytes,
      (unsigned int)meter->response[0], (unsigned int)meter->response[1],
      (unsigned int)meter->response[2], (unsigned int)meter->response[3],
      (unsigned int)meter->checksum_expected, (unsigned int)meter->checksum_received);
  if (length > 0)
  {
    Console_Write(line);
  }

  length = snprintf(line, sizeof(line),
      "[BL0942 RAW] I_RMS=0x%06lX V_RMS=0x%06lX WATT=0x%06lX "
      "ENERGY=0x%06lX FREQ=0x%06lX GAIN_CR=0x%06lX "
      "CURRENT_GAIN=x%lu GAIN_READ=%s\r\n",
      (unsigned long)(meter->i_rms_raw & 0xFFFFFFUL),
      (unsigned long)(meter->v_rms_raw & 0xFFFFFFUL),
      (unsigned long)(meter->watt_raw & 0xFFFFFFUL),
      (unsigned long)(meter->energy_raw & 0xFFFFFFUL),
      (unsigned long)(meter->frequency_raw & 0xFFFFFFUL),
      (unsigned long)(meter->gain_cr_raw & 0xFFFFFFUL),
      (unsigned long)current_gain,
      meter->gain_cr_valid ? "OK" : "FAIL");
  if (length > 0)
  {
    Console_Write(line);
  }
}

static void Console_RunRS485SelfTest(void)
{
  RS485_Packet_t source;
  RS485_Packet_t decoded;
  uint8_t frame[RS485_MAX_FRAME_SIZE];
  size_t frame_length;
  bool passed;

  memset(&source, 0, sizeof(source));
  memset(&decoded, 0, sizeof(decoded));
  source.destination_id = RS485_GetLocalId();
  source.source_id = 0x22U;
  source.command = 0x55U;
  source.data_length = 4U;
  source.data[0] = 0x12U;
  source.data[1] = 0x34U;
  source.data[2] = 0x56U;
  source.data[3] = 0x78U;

  frame_length = RS485_EncodeFrame(&source, frame, sizeof(frame));
  passed = (frame_length > 0U) &&
           (RS485_DecodeFrame(frame, frame_length, &decoded) == RS485_FRAME_OK) &&
           (decoded.destination_id == source.destination_id) &&
           (decoded.source_id == source.source_id) &&
           (decoded.command == source.command) &&
           (decoded.data_length == source.data_length) &&
           (memcmp(decoded.data, source.data, source.data_length) == 0);

  Console_Write(passed ?
      "[RS485] frame/CRC software self-test: PASS\r\n" :
      "[RS485] frame/CRC software self-test: FAIL\r\n");
}

static void Console_Init(void)
{
  char id_message[96];

  console_last_report_ms = HAL_GetTick();
  Console_Write("\r\nSMART_EDB_BOARD diagnostic console\r\n"
                "UART6: 115200 baud, 8-N-1\r\n"
                "Commands: h=help, s=status, m=measure now, "
                "t=RS485 self-test, p=ping gateway 0, "
                "o=AC relay on, f=AC relay off, 1/2=set local ID\r\n"
                "RS485 physical test requires a powered peer and correct A/B/GND wiring.\r\n");
  Console_RunRS485SelfTest();
  (void)snprintf(id_message, sizeof(id_message),
                 "[RS485] DIP ID=%u local_id=%u status=%s "
                 "(valid board IDs: 1..254)\r\n",
                 (unsigned int)board_dip_id,
                 (unsigned int)RS485_GetLocalId(),
                 board_dip_id_valid ? "OK" : "INVALID");
  Console_Write(id_message);
}

static void Console_Process(void)
{
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - console_last_report_ms) >= CONSOLE_REPORT_INTERVAL_MS)
  {
    console_last_report_ms = now;
    Console_PrintStatus();
  }

  if (HAL_UART_Receive(&huart6, &console_rx_byte, 1U, 0U) != HAL_OK)
  {
    return;
  }

  switch (console_rx_byte)
  {
    case 's':
    case 'S':
      Console_PrintStatus();
      break;

    case 'm':
    case 'M':
      Console_Write(PowerMonitor_ForceUpdate() ?
                    "[MEASURE] update: OK\r\n" :
                    "[MEASURE] update: FAIL\r\n");
      Console_PrintStatus();
      break;

    case 't':
    case 'T':
      Console_RunRS485SelfTest();
      break;

    case 'p':
    case 'P':
      Console_Write(DisplayProtocol_SendPing(CONSOLE_RS485_TEST_PEER_ID) == HAL_OK ?
                    "[RS485] ping sent to gateway 0; check rsp counter.\r\n" :
                    "[RS485] ping transmit failed.\r\n");
      break;

    case 'o':
    case 'O':
      DisplayProtocol_SetACOutput(true);
      Console_Write("[AC RELAY] ON (PA5=HIGH)\r\n");
      break;

    case 'f':
    case 'F':
      DisplayProtocol_SetACOutput(false);
      Console_Write("[AC RELAY] OFF (PA5=LOW)\r\n");
      break;

    case '1':
      RS485_SetLocalId(1U);
      Console_Write("[RS485] local node ID set to 1.\r\n");
      break;

    case '2':
      RS485_SetLocalId(2U);
      Console_Write("[RS485] local node ID set to 2.\r\n");
      break;

    case 'h':
    case 'H':
      Console_Write("Commands: h=help, s=status, m=measure now, "
                    "t=RS485 self-test, p=ping gateway 0, "
                    "o=AC relay on, f=AC relay off, 1/2=set local ID\r\n");
      break;

    case '\r':
    case '\n':
      break;

    default:
      Console_Write("Unknown command. Press h for help.\r\n");
      break;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param None
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
#ifdef USE_FULL_ASSERT
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
