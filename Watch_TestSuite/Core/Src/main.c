/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main.c
 * @author  MCD Application Team
 * @brief   BLE application with BLE core
 *
  @verbatim
  ==============================================================================
                    ##### IMPORTANT NOTE #####
  ==============================================================================

  This application requests having the stm32wb5x_BLE_Stack_fw.bin binary
  flashed on the Wireless Coprocessor.
  If it is not the case, you need to use STM32CubeProgrammer to load the appropriate
  binary.

  All available binaries are located under following directory:
  /Projects/STM32_Copro_Wireless_Binaries

  Refer to UM2237 to learn how to use/install STM32CubeProgrammer.
  Refer to /Projects/STM32_Copro_Wireless_Binaries/ReleaseNote.html for the
  detailed procedure to change the Wireless Coprocessor binary.

  @endverbatim
 ******************************************************************************
 * @attention
 *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
 * All rights reserved.</center></h2>
 *
  * This software component is licensed by ST under Ultimate Liberty license 
  * SLA0044, the "License"; You may not use this file except in compliance with 
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "app_entry.h"
#include "app_common.h"

#include "stm32_lpm.h"
#include "dbg_trace.h"
#include "hw_conf.h"
#include "otp.h"
#include "ble.h"

#include "er_oled.h"
#include "dotstar.h"
#include "iqs263.h"
#include "si7021.h"
#include "veml7700.h"

#include "p2p_server_app.h"
#include "p2p_stm.h"

#define NUM_PIXELS 12

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;


//THREADS
// esmMain -- launch ESM at random intervals, notify UI, handle
//            timeouts for interaction and alerts.  main thread.
//            on init timeout for time estimate, if mode has changed
//            because of button press (sleep) skip rest, otherwise loop
//            alert.  on timeout for other survey, simply skip rest.
//            wait to launch if not RESTING.

// uiControl -- touch and screen update handling.  Send ui events
//              to BLETX, notify esmMain when ui event complete.
//              loop that check for press and mode; faster loop
//              when not resting.

// alert -- led flash and vibrate when notified by esmMain.

// conditionsPoll -- poll lux/whitelux/temp/humd every 10s, send
//                   to BLETX

// BLERX -- update RTC timestamp.  Set bounds for times of day
//          that are okay to query, sleep for day.

// BLETX -- if queue is empty send data over BLE to phone;
//                            if send data fails, queue.
//          otherwise add data to queue and try to send queue.
//          if success, keep sending data until queue is empty
//          and set queue pointer to NULL.
//          accepts either (1) send conditions
//                         (2) send timestamp (use last_timestamp).
//                         (3) send survey result raw 4bits for answer, 4 bits for survey num

// buttonPressed -- send invalidate data/sleep event to BLERX, end
//                  survey, reset mode, restart timer for esm
//                  poll.  any data between this and previous
//                  survey start event is invalidated.

//QUEUES and Global Data Structs

// ProgramMode -- shared between esmMain and uiControl to
// identify what should be on the screen during touch events
// SURVEY STATE -- shared between esmMain and uiControl to
// identify what should be on the screen (if not time estimate)
// LAST TIME ESTIMATE -- last time that time is checked, start here
// when estimating new time
// NOTIFY TIMEBOUNDS -- time bounds for when to have notifications, intervals to estimate
// BLE TX and RX QUEUE -- send data to and from BLE

// SendQueue -- dynamic data of BLE data that hasn't sent

//UI tools -- (1) time estimate based on previous time
//            (2) survey based on Survey_t, with partial screen updates


/* USER CODE BEGIN PV */
/* Definitions for uiControl */
osThreadId_t uiControlHandle;
const osThreadAttr_t uiControl_attributes = {
  .name = "uiControl",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 8
};
/* Definitions for esmMainl */
osThreadId_t esmMainHandle;
const osThreadAttr_t esmMain_attributes = {
  .name = "esmMain",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 8
};
/* Definitions for buttonPress */
osThreadId_t buttonPressHandle;
const osThreadAttr_t buttonPress_attributes = {
  .name = "buttonPress",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for conditionsPoll */
osThreadId_t conditionsPollHandle;
const osThreadAttr_t conditionsPoll_attributes = {
  .name = "conditionsPoll",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 8
};
/* Definitions for bleTX */
osThreadId_t bleTXHandle;
const osThreadAttr_t bleTX_attributes = {
  .name = "bleTX",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 8
};
/* Definitions for bleRX */
osThreadId_t bleRXHandle;
const osThreadAttr_t bleRX_attributes = {
  .name = "bleRX",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 8
};



/* Definitions for bleTXqueue */
osMessageQueueId_t bleTXqueueHandle;
const osMessageQueueAttr_t bleTXqueue_attributes = {
  .name = "bleTXqueue"
};
/* Definitions for bleRXqueue */
//osMessageQueueId_t bleRXqueueHandle; //global
const osMessageQueueAttr_t bleRXqueue_attributes = {
  .name = "bleRXqueue"
};


/* Definitions for rtcMutex */
//osMutexId_t rtcMutexHandle; //global
const osMutexAttr_t rtcMutex_attributes = {
  .name = "rtcMutex"
};
/* Definitions for timeBoundMutex */
osMutexId_t timeBoundMutexHandle;
const osMutexAttr_t timeBoundMutex_attributes = {
  .name = "timeBoundMutex"
};
/* Definitions for lastSeenMutex */
osMutexId_t lastSeenMutexHandle;
const osMutexAttr_t lastSeenMutex_attributes = {
  .name = "lastSeenMutex"
};
///* Definitions for timeEstimateMutex */
//osMutexId_t timeEstimateMutexHandle;
//const osMutexAttr_t timeEstimateMutex_attributes = {
//  .name = "timeEstimateMutex"
//};
/* Definitions for conditionMutex */
osMutexId_t conditionMutexHandle;
const osMutexAttr_t conditionMutex_attributes = {
  .name = "conditionMutex"
};
/* Definitions for modeMutex */
osMutexId_t modeMutexHandle;
const osMutexAttr_t modeMutex_attributes = {
  .name = "modeMutex"
};
/* Definitions for surveyMutex */
osMutexId_t surveyMutexHandle;
const osMutexAttr_t surveyMutex_attributes = {
  .name = "surveyMutex"
};

/* USER CODE BEGIN PV */


typedef enum {
	LED_NONE,
	LED_OFF,
	LED_TIME,
	LED_TOUCH_TRACK,
	LED_CONFIRM_FLASH,
	LED_SPIRAL,
	LED_OTHER
} LedStatus_t;

typedef struct {
	LedStatus_t currentMode;
	LedStatus_t nextMode;
	TickType_t modeTimeout;
} LedState_t;

LedState_t LedState;
//protected by ledStateMutex


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_RF_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void GlobalState_Init(void);
static void updateInterval(void);
void startUIControl(void *argument);
void startESMMain(void *argument);
void startButtonPress(void *argument);
void startConditionsPoll(void *argument);
void startBLETX(void *argument);
void startBLERX(void *argument);

/* USER CODE BEGIN PFP */
void PeriphClock_Config(void);
static void Reset_Device( void );
static void Reset_IPCC( void );
static void Reset_BackupDomain( void );
static void Init_Exti( void );
static void Config_HSE(void);

int main(void)
{
  /**
   * The OPTVERR flag is wrongly set at power on
   * It shall be cleared before using any HAL_FLASH_xxx() api
   */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);





  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  Reset_Device();
  Config_HSE();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  PeriphClock_Config();
  Init_Exti(); /**< Configure the system Power Mode */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_RF_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  GlobalState_Init();

  //Init Pseudo-Random Number Generation Seed
  srand(0xFA1863A7);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
    /* creation of rtcMutex */
    rtcMutexHandle = osMutexNew(&rtcMutex_attributes);

    /* creation of timeBoundMutex */
    timeBoundMutexHandle = osMutexNew(&timeBoundMutex_attributes);

    /* creation of lastSeenMutex */
    lastSeenMutexHandle = osMutexNew(&lastSeenMutex_attributes);

    /* creation of timeEstimateMutex */
    //timeEstimateMutexHandle = osMutexNew(&timeEstimateMutex_attributes);

    /* creation of conditionMutex */
    conditionMutexHandle = osMutexNew(&conditionMutex_attributes);

    /* creation of modeMutex */
    modeMutexHandle = osMutexNew(&modeMutex_attributes);

    /* creation of surveyMutex */
    surveyMutexHandle = osMutexNew(&surveyMutex_attributes);

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_QUEUES */
      /* creation of bleTXqueue */
      bleTXqueueHandle = osMessageQueueNew (16, sizeof(BLETX_Queue_t), &bleTXqueue_attributes);

      /* creation of bleRXqueue */
      bleRXqueueHandle = osMessageQueueNew (16, sizeof(P2PS_STM_Data_t *), &bleRXqueue_attributes);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
        /* creation of uiControl */
        uiControlHandle = osThreadNew(startUIControl, NULL, &uiControl_attributes);

        /* creation of ESMMain */
        esmMainHandle = osThreadNew(startESMMain, NULL, &esmMain_attributes);

        /* creation of buttonPress */
        buttonPressHandle = osThreadNew(startButtonPress, NULL, &buttonPress_attributes);

        /* creation of conditionsPoll */
        conditionsPollHandle = osThreadNew(startConditionsPoll, NULL, &conditionsPoll_attributes);

        /* creation of bleTX */
        bleTXHandle = osThreadNew(startBLETX, NULL, &bleTX_attributes);

        /* creation of bleRX */
        bleRXHandle = osThreadNew(startBLERX, NULL, &bleRX_attributes);


  /* Init code for STM32_WPAN */
  APPE_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  while (1){}

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP
                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_LPUART1;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSE;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

#if (CFG_USE_SMPS != 0)
  /**
   *  Configure and enable SMPS
   *
   *  The SMPS configuration is not yet supported by CubeMx
   *  when SMPS output voltage is set to 1.4V, the RF output power is limited to 3.7dBm
   *  the SMPS output voltage shall be increased for higher RF output power
   */
  LL_PWR_SMPS_SetStartupCurrent(LL_PWR_SMPS_STARTUP_CURRENT_80MA);
  LL_PWR_SMPS_SetOutputVoltageLevel(LL_PWR_SMPS_OUTPUT_VOLTAGE_1V40);
  LL_PWR_SMPS_Enable();
#endif

  /* USER CODE END Smps */
}

static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00300F38;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RF Initialization Function
  * @param None
  * @retval None
  */
static void MX_RF_Init(void)
{

  /* USER CODE BEGIN RF_Init 0 */

  /* USER CODE END RF_Init 0 */

  /* USER CODE BEGIN RF_Init 1 */

  /* USER CODE END RF_Init 1 */
  /* USER CODE BEGIN RF_Init 2 */

  /* USER CODE END RF_Init 2 */

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

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */


  //Some fancy stuff to reset RTC value on wakeup; don't need it

  /* Disable RTC registers write protection */
  //LL_RTC_DisableWriteProtection(RTC);

  //LL_RTC_WAKEUP_SetClock(RTC, CFG_RTC_WUCKSEL_DIVIDER);

  /* Enable RTC registers write protection */
  //LL_RTC_EnableWriteProtection(RTC);

  /** Initialize RTC and set the Time and Date
    */
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    sTime.Hours = 0x11;
    sTime.Minutes = 0x59;
    sTime.Seconds = 0x29;
    sTime.SubSeconds = 0x0;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
    {
      Error_Handler();
    }
    sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
    sDate.Month = RTC_MONTH_MARCH;
    sDate.Date = 0x29;
    sDate.Year = 0x20;

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
    {
      Error_Handler();
    }


  /* USER CODE END RTC_Init 2 */

}

/* USER CODE BEGIN 4 */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;

  //hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  //hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_RESET_GPIO_Port, OLED_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OLED_RESET_Pin */
  GPIO_InitStruct.Pin = OLED_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OLED_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_1_Pin BUTTON_2_Pin BUTTON_3_Pin */
  GPIO_InitStruct.Pin = BUTTON_1_Pin|BUTTON_2_Pin|BUTTON_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure SPI_NSS pin to be GPIO, pulled up*/
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */


static void GlobalState_Init(){
	GlobalState.timeBound.startHR_BCD = 0x10; //10AM, BCD
	GlobalState.timeBound.endHR_BCD = 0x22;   //10PM, BCD
	GlobalState.timeBound.minInterval = INTERVAL_MIN;   //15min min interval
	GlobalState.timeBound.maxInterval = INTERVAL_MAX;   //90min max interval

	RTC_TimeTypeDef tempTime;
	RTC_DateTypeDef tempDate;
	HAL_RTC_GetTime(&hrtc, &tempTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &tempDate, RTC_FORMAT_BCD);

	//shallow structs so no issues with assignment
	GlobalState.lastSeenTime.time = tempTime;
	GlobalState.lastSeenTime.date = tempDate;

	//GlobalState.timeEstimateSample.time = tempTime;
	//GlobalState.timeEstimateSample.date = tempDate;

	GlobalState.lastConditions.lux = 0.0;
	GlobalState.lastConditions.whiteLux = 0.0;
	GlobalState.lastConditions.temp = 0.0;
	GlobalState.lastConditions.humd = 0.0;

	GlobalState.programMode = 0;

	GlobalState.surveyState.surveyID = SURVEY_NONE;
	char temp_string[10] = "  DRAMSAY.";
	strncpy(GlobalState.surveyState.screenText, temp_string, strlen(temp_string)+1);
	GlobalState.surveyState.screenTextLength = strlen(temp_string);
	memset(GlobalState.surveyState.optionArray, 0, sizeof(GlobalState.surveyState.optionArray));
	GlobalState.surveyState.optionArrayLength = 0;

	GlobalState.currentInterval = 0;
	GlobalState.paused = 0;
	GlobalState.demo = 0;
}

static void updateInterval(){
	uint32_t updateVal = rand() % (GlobalState.timeBound.maxInterval - GlobalState.timeBound.minInterval);
	GlobalState.currentInterval = GlobalState.timeBound.minInterval + updateVal;
}



static inline void set_bit(long *x, int bitNum) {
    *x |= (1L << bitNum);
}

static inline void clear_bit(long *x, int bitNum) {
    *x &= (~(1L << bitNum));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(buttonPressHandle, GPIO_Pin, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

HAL_StatusTypeDef updateLastSeenTime(){

	HAL_StatusTypeDef status_1, status_2;

    osMutexAcquire(lastSeenMutexHandle, portMAX_DELAY);
    status_1 = HAL_RTC_GetTime(&hrtc, &(GlobalState.lastSeenTime.time), RTC_FORMAT_BCD);
    status_2 = HAL_RTC_GetDate(&hrtc, &(GlobalState.lastSeenTime.date), RTC_FORMAT_BCD);
    osMutexRelease(lastSeenMutexHandle);

    return status_1 | status_2;

}

void get_RTC_hrmin(char *dest) {

    RTC_TimeTypeDef cTime;
    RTC_DateTypeDef cDate;

    osMutexAcquire(rtcMutexHandle, portMAX_DELAY);
    HAL_RTC_GetTime(&hrtc, &cTime, RTC_FORMAT_BCD);
    //must get date as well; RTC shadow registers will error if both aren't accessed
    HAL_RTC_GetDate(&hrtc, &cDate, RTC_FORMAT_BCD);
    osMutexRelease(rtcMutexHandle);

    uint8_t  hrs = RTC_Bcd2ToByte(cTime.Hours);
    uint8_t mins = RTC_Bcd2ToByte(cTime.Minutes);

    char time[5];
    sprintf (time, "%02d%02d", hrs, mins);

    strncpy(dest, time, sizeof(time));

}

void get_RTC_hrminsec(char *dest) {

	RTC_TimeTypeDef cTime;
    RTC_DateTypeDef cDate;


	osMutexAcquire(rtcMutexHandle, portMAX_DELAY);
	HAL_RTC_GetTime(&hrtc, &cTime, RTC_FORMAT_BCD);
	//must get date as well; RTC shadow registers will error if both aren't accessed
	HAL_RTC_GetDate(&hrtc, &cDate, RTC_FORMAT_BCD);
	osMutexRelease(rtcMutexHandle);

	uint8_t  hrs = RTC_Bcd2ToByte(cTime.Hours);
	uint8_t mins = RTC_Bcd2ToByte(cTime.Minutes);
	uint8_t secs = RTC_Bcd2ToByte(cTime.Seconds);

	char time[10];
	sprintf (time, "%02d:%02d:%02d", hrs, mins, secs);

	strncpy(dest, time, sizeof(time));

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header startUIControl */
/**
  * @brief  Function implementing the uiControl thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header startUIControl */
void startUIControl(void *argument)
{


  int duty_cycle = 79; //0 is off, up to ~80
  htim1.Instance->CCR2 = duty_cycle;

	//Dotstar Init
	DotStar_InitHandle dotstar;
	dotstar.spiHandle = &hspi1;
	dotstar.numLEDs = NUM_PIXELS;
	dotstar.colorOrder = DOTSTAR_BGR;
	Dotstar_Init(&dotstar);

	ds_clear();  //turn off
	ds_show();

	const uint8_t MAX_BRIGHTNESS = 0xFF; //max brightness, 0x01-0xFF

	ds_setBrightness(0);

  /* USER CODE BEGIN 5 */
  HAL_GPIO_WritePin(OLED_RESET_GPIO_Port, OLED_RESET_Pin, GPIO_PIN_SET);

  uint8_t oled_buf[WIDTH * HEIGHT / 8];

  er_oled_begin();
  er_oled_clear(oled_buf);
  er_oled_string(0, 10, "  DRAMSAY.", 12, 1, oled_buf);
  er_oled_string(0, 28, "resenv | mit", 12, 1, oled_buf);
  er_oled_display(oled_buf);


    osDelay(100);

    uint16_t counter;
    uint32_t color;
    uint8_t r,g,b;

  	//start vibration
  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  	osDelay(pdMS_TO_TICKS(300)); //300ms delay

  	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);


    counter = 0;
    r = 0xFF;
 	g = 0xFF;
 	b = 0xFF;

  	ds_setBrightness(MAX_BRIGHTNESS);
  	for(int l=0;l<24*4;l++) {

		for (int i=0; i< NUM_PIXELS; i++){

			if      (i==(counter+2)%12)	{ color = (r  <<16) | (g  <<8) | b; }
			else if (i==(counter+1)%12) { color = (r/2<<16) | (g/2<<8) | b/2; }
			else if (i==counter)        { color = (r/4<<16) | (g/4<<8) | b/4; }
			else 						{ color = 0x000000; }

			ds_setPixelColor32B(i, color); // 'off' pixel at head
		}

		ds_show();
		r = (r + 11) % 0xFF;
		g = (g + 13) % 0xFF;
		b = (b + 17) % 0xFF;
		counter = (counter+1)%12;
		osDelay(pdMS_TO_TICKS(50));
	}

	//turn off LEDs
	ds_setBrightness(0);
	ds_fill(0x000000, 0, 12);
	ds_show();


  er_oled_clear(oled_buf);
  er_oled_display(oled_buf);

  int16_t current_minute = -1;
  int16_t display_minute = -1;
  int16_t last_display_minute = -1;
  int16_t minute_history[TOUCH_HISTORY_SIZE] = {0};
  uint8_t history_ind = 0;
  int16_t min_minute, max_minute;
  uint8_t hrs, mins, step, i;
  char time[5];

  char display1[12];
  char display2[12];
  char display3[12];

  uint8_t touch_end_count = 0;

  RTC_TimeTypeDef cTime;
  RTC_DateTypeDef cDate;

  BLETX_Queue_t bleSendData;

  //init peripheral (not turbo mode, poll every 250ms, if touch sample at 40Hz until no touch)
  if (setup_iqs263() == HAL_ERROR) {
	  strncpy(errorCondition, "ERR:IQS263ST", sizeof(errorCondition));
	  GlobalState.programMode = MODE_ERROR;
  }

  osDelay(500); //let screen start first

  if (veml_Setup(hi2c1, VEML_5S_POLLING) == HAL_ERROR){
		//error condition
		strncpy(errorCondition, "ERR:VEML7700", sizeof(errorCondition));
		GlobalState.programMode = MODE_ERROR;
  }

  osDelay(10);

  if (si7021_set_config(&hi2c1, SI7021_HEATER_OFF, SI7021_RESOLUTION_RH12_TEMP14) == HAL_ERROR) {
		//error condition
		strncpy(errorCondition, "ERR:SI7021CF", sizeof(errorCondition));
		GlobalState.programMode = MODE_ERROR;
  }

  osDelay(10);

  if (si7021_set_heater_power(&hi2c1, SI7021_HEATER_POWER_3MA) == HAL_ERROR) {
		//error condition
		strncpy(errorCondition, "ERR:SI7021HT", sizeof(errorCondition));
		GlobalState.programMode = MODE_ERROR;
  }

  float lux;
  float whiteLux;
  float humidity;
  float temperature;

  int* coords[4] = {0x0000, 0x0000, 0x0000, 0x0000};
  int angle, min;

    /* Infinite loop */
    for(;;)
    {

     if (GlobalState.programMode){

     iqs263_poll_raw(coords);
     angle = _get_angle(coords);
     min = _get_min_if_pressed(coords);

     sprintf(display1, "%d %d %d %d", coords[0], coords[1], coords[2], coords[3]);
     sprintf(display2, "angle: %3d", angle);
     sprintf(display3, "min  :  %2d", min);

     er_oled_clear(oled_buf);
     er_oled_string(0, 0, display1, 12, 1, oled_buf);
     er_oled_string(0, 12, display2, 12, 1, oled_buf);
     er_oled_string(0, 24, display3, 12, 1, oled_buf);
     er_oled_display(oled_buf);
     osDelay(50);

     } else {


	lux = veml_Get_Lux();
	whiteLux = veml_Get_White_Lux();
	temperature = si7021_measure_temperature(&hi2c1);
	humidity = si7021_measure_humidity(&hi2c1);

	//sprintf(display1, "%2.2f  %2.2f", temperature, humidity);
	sprintf(display1, "lux : %x, %x", temperature, humidity);
	sprintf(display2, "lux : %x", lux);
	sprintf(display3, "wlux: %x", whiteLux);

	er_oled_clear(oled_buf);
	er_oled_string(0, 0, display1, 12, 1, oled_buf);
	er_oled_string(0, 12, display2, 12, 1, oled_buf);
	er_oled_string(0, 24, display3, 12, 1, oled_buf);
	er_oled_display(oled_buf);
	osDelay(50);
    }


     // -- Adjust/Dial in Angle Mapping for Touch Sensor -- //
     //uint16_t current_angle = iqs263_get_angle();
     //er_oled_clear(oled_buf);
     //sprintf (time, "%d", current_angle);
     //er_oled_string(0, 0, time, 12, 1, oled_buf);
     //er_oled_display(oled_buf);
     // /* comment out when doing angle adjustments


     /*
	 current_minute = iqs263_get_min_if_pressed(); //returns -1 if no press
     if (current_minute != -1) { //touch!

       if (!touch_end_count){ //START TOUCH EVENT!

    	   er_oled_clear(oled_buf);

    	   switch (GlobalState.programMode) {
    	   	   case MODE_ESM_TIME_ESTIMATE:
    	   	       xTaskNotifyGive(esmMainHandle);
           		   er_oled_string(0, 0, " GUESS TIME:", 12, 1, oled_buf);
           		   break;
    	   	   case MODE_ESM_SURVEY:
    	   	       er_oled_string(0, 0, GlobalState.surveyState.screenText, 12, 1, oled_buf);
    	   	       break;
    	   	   case MODE_RESTING:
    			   osMutexAcquire(modeMutexHandle, portMAX_DELAY);
    			   GlobalState.programMode = MODE_TIME_ESTIMATE;
    			   osMutexRelease(modeMutexHandle);
    			   //fall through to next case
    	   	   case MODE_TIME_ESTIMATE:
    	   		   er_oled_string(0, 0, " GUESS TIME:", 12, 1, oled_buf);
          		   break;
    	   }

           //if we're guessing the time, on start of touch we need to grab
           //the hour of the last seen time as a starting point.
           if (GlobalState.programMode == MODE_TIME_ESTIMATE ||
               GlobalState.programMode == MODE_ESM_TIME_ESTIMATE){

                osMutexAcquire(lastSeenMutexHandle, portMAX_DELAY);
                cTime = GlobalState.lastSeenTime.time;
                osMutexRelease(lastSeenMutexHandle);

                hrs = RTC_Bcd2ToByte(cTime.Hours);
                mins = 0x00;
           }

           //on immediate touch set history to current minute
           for (i=0; i < TOUCH_HISTORY_SIZE; i++){
        	   minute_history[i] = current_minute;
           }
       }

       touch_end_count = 1;

       //put current minute in history buffer and advance circular index
       minute_history[history_ind] = current_minute;
       history_ind = (history_ind + 1) % TOUCH_HISTORY_SIZE;

       //TWO MODES - fast and slow
       // if large variation (min and max in buffer > 3 min) set display_time to current_time, else average

       //take average of buffer, get min and max; that's what should be displayed
       display_minute = 0;
       min_minute = 60;
       max_minute = 0;
       for (i=0; i< TOUCH_HISTORY_SIZE; i++){
    	   display_minute += minute_history[i];
    	   if (minute_history[i] < min_minute) min_minute = minute_history[i];
    	   if (minute_history[i] > max_minute) max_minute = minute_history[i];
       }

       if (max_minute-min_minute > 4) {
    	   display_minute = current_minute;
       } else {
    	   display_minute /= TOUCH_HISTORY_SIZE;
       }

       //if last displayed is not what should be displayed, display
  	   if (last_display_minute != display_minute) { //UPDATE TOUCH VALUE!

           switch (GlobalState.programMode){
                case MODE_ESM_SURVEY:
                    //clear bottom
                    er_oled_clear_bottom_third(oled_buf);

                    //divide 10-50 min into option steps roughly
                    step = 45 / (GlobalState.surveyState.optionArrayLength+1);
                    for (i=0; i<GlobalState.surveyState.optionArrayLength; i++){
                        //map minute to option
                        if(display_minute >= (15 + i*step) &&
                           display_minute < (15 + (i+1)*step)){
                            er_oled_string(0, 28, GlobalState.surveyState.optionArray[GlobalState.surveyState.optionArrayLength-1-i], 12, 1, oled_buf);
                        }
                    }
                    er_oled_display(oled_buf);

                    break;
                case MODE_TIME_ESTIMATE:
                case MODE_ESM_TIME_ESTIMATE:

                    //hours wrap
                    if (display_minute < 15 &&
                        display_minute >= 0 &&
                        last_display_minute > 45){
                        hrs = (hrs+1)%24;

                    } else if (display_minute > 45 &&
                               last_display_minute < 15){
                        if (hrs==0) {hrs = 23;}
                        else {hrs -= 1;}
                    }

                    //display time on bottom two thirds
                    sprintf (time, "%02d%02d", hrs, display_minute);
                    er_oled_time_twothird(time, oled_buf);

                    break;
           }

           last_display_minute = display_minute;

  	   }


     } else if (touch_end_count > 0){

  	   touch_end_count += 1;//increment touching_end_count

  	   if (touch_end_count >= TOUCH_END_TIMEOUT){  //FINISHED/CONFIRMED TOUCH VALUE!
  		   uint8_t option = 0xFF;

  		   switch (GlobalState.programMode){
  		   	   case MODE_ESM_SURVEY:
  		   		   //grab current option
  		   		   for (i=0; i<GlobalState.surveyState.optionArrayLength; i++){
  		   		     //map minute to option
  		   		     if(last_display_minute >= (15 + i*step) &&
  		   		        last_display_minute < (15 + (i+1)*step)){
  		   		         option = GlobalState.surveyState.optionArrayLength-1-i;
  		   		     }
  		   		   }

  		   		   //send to ble
  		   		   bleSendData.sendType = TX_SURVEY_RESULT;
  		   		   bleSendData.data = (GlobalState.surveyState.surveyID << 8) | option;
  		   		   osMessageQueuePut(bleTXqueueHandle, &bleSendData, 0, 0);

  		   		   //notify main thread
  		   		   xTaskNotifyGive(esmMainHandle);
  		   		   break;

  		   	   case MODE_ESM_TIME_ESTIMATE:
  		   		   //send to ble
  		   		   bleSendData.sendType = TX_TIME_EST;
  		   		   bleSendData.data = (hrs << 8) | last_display_minute;
   		   		   osMessageQueuePut(bleTXqueueHandle, &bleSendData, 0, 0);

   		   		   //notify main thread
   		   		   xTaskNotifyGive(esmMainHandle);
  		   		   break;

  		   	   case MODE_TIME_ESTIMATE:
  		   		   //send to ble
  		   		   bleSendData.sendType = TX_TIME_EST;
  		   		   bleSendData.data = (hrs << 8) | last_display_minute;
   		   		   osMessageQueuePut(bleTXqueueHandle, &bleSendData, 0, 0);

   		   		   //not from main thread, show time (which updates seen time, new interval, back to rest)
  		   		   osMutexAcquire(modeMutexHandle, portMAX_DELAY);
				   GlobalState.programMode = MODE_SHOW_TIME;
				   osMutexRelease(modeMutexHandle);
				   break;
  		   }

  		   touch_end_count = 0;
  		   last_display_minute = -1;
  		   er_oled_clear(oled_buf);
  		   er_oled_display(oled_buf);

  	   }

  	   osDelay(50);


     }else { //no touch, wait for a touch

       switch (GlobalState.programMode){
        case MODE_CANCEL:
    	   //had a 'cancel' button event

    	   er_oled_clear(oled_buf);
   	   	   er_oled_string(0, 0, "  dismiss!", 12, 1, oled_buf);
   	   	   er_oled_string(0, 20, "TIME NOW IS:", 12, 1, oled_buf);
   	   	   er_oled_display(oled_buf);

   	   	   osDelay(1000);

   	   	   osMutexAcquire(rtcMutexHandle, portMAX_DELAY);
   	   	   HAL_RTC_GetTime(&hrtc, &cTime, RTC_FORMAT_BCD);
   	   	   HAL_RTC_GetDate(&hrtc, &cDate, RTC_FORMAT_BCD);
   	   	   osMutexRelease(rtcMutexHandle);

   	   	   hrs = RTC_Bcd2ToByte(cTime.Hours);
   	   	   mins = RTC_Bcd2ToByte(cTime.Minutes);
   	   	   sprintf (time, "%02d%02d", hrs, mins);
   	   	   er_oled_time(time);

	       osMutexAcquire(lastSeenMutexHandle, portMAX_DELAY);
	       GlobalState.lastSeenTime.time = cTime;
	       GlobalState.lastSeenTime.date = cDate;
	       osMutexRelease(lastSeenMutexHandle);

	       bleSendData.sendType = TX_TIME_SEEN;
	       bleSendData.data = 0x0000;
	       osMessageQueuePut(bleTXqueueHandle, &bleSendData, 0, 0);

	       updateInterval();

	       osMutexAcquire(modeMutexHandle, portMAX_DELAY);
	       GlobalState.programMode = MODE_RESTING;
	       osMutexRelease(modeMutexHandle);

	       osDelay(3000);
	       er_oled_clear(oled_buf);
	       er_oled_display(oled_buf);
	       break;

        case MODE_SHOW_TIME:
		   //show time, no cancel, but does the same thing

		   er_oled_clear(oled_buf);
		   er_oled_string(0, 0, " completed", 12, 1, oled_buf);
		   er_oled_string(0, 20, "TIME NOW IS:", 12, 1, oled_buf);
		   er_oled_display(oled_buf);

		   osDelay(1000);

		   osMutexAcquire(rtcMutexHandle, portMAX_DELAY);
		   HAL_RTC_GetTime(&hrtc, &cTime, RTC_FORMAT_BCD);
		   HAL_RTC_GetDate(&hrtc, &cDate, RTC_FORMAT_BCD);
		   osMutexRelease(rtcMutexHandle);

		   hrs = RTC_Bcd2ToByte(cTime.Hours);
		   mins = RTC_Bcd2ToByte(cTime.Minutes);
		   sprintf (time, "%02d%02d", hrs, mins);
		   er_oled_time(time);

		   osMutexAcquire(lastSeenMutexHandle, portMAX_DELAY);
		   GlobalState.lastSeenTime.time = cTime;
		   GlobalState.lastSeenTime.date = cDate;
		   osMutexRelease(lastSeenMutexHandle);

		   bleSendData.sendType = TX_TIME_SEEN;
		   bleSendData.data = 0x0000;
		   osMessageQueuePut(bleTXqueueHandle, &bleSendData, 0, 0);

		   updateInterval();

		   osMutexAcquire(modeMutexHandle, portMAX_DELAY);
		   GlobalState.programMode = MODE_RESTING;
		   osMutexRelease(modeMutexHandle);

		   osDelay(3000);
		   er_oled_clear(oled_buf);
		   er_oled_display(oled_buf);
		   break;

       case MODE_ERROR:
    	   //ERROR condition: print condition and loop forever

    	   er_oled_clear(oled_buf);
    	   er_oled_string(0, 12, errorCondition, 12, 1, oled_buf);
    	   er_oled_display(oled_buf);
    	   for (;;){}
    	   break;

       case MODE_CLEAR:
    	   //Timeout, notify and wait until clear
    	   //show time and restart

		   er_oled_clear(oled_buf);
		   er_oled_string(0, 10, "  TIMEOUT!", 12, 1, oled_buf);
		   er_oled_string(0, 28, " hit button", 12, 1, oled_buf);
		   er_oled_display(oled_buf);

		   osDelay(500);
    	   break;

       case MODE_ESM_TIME_ESTIMATE:
    	  er_oled_clear(oled_buf);
    	  er_oled_string(0, 0, " GUESS TIME:", 12, 1, oled_buf);
    	  er_oled_display(oled_buf);
    	  osDelay(100);
    	  break;

       case MODE_ESM_SURVEY:
    	  er_oled_clear(oled_buf);
    	  er_oled_string(0, 0, GlobalState.surveyState.screenText, 12, 1, oled_buf);
    	  er_oled_display(oled_buf);
    	  osDelay(100);
    	  break;

       case MODE_RESTING:
		   osDelay(250);
		   if (GlobalState.paused){
			   er_oled_clear(oled_buf);
			   er_oled_string(0, 14, "   paused", 12, 1, oled_buf);
			   er_oled_display(oled_buf);
		   }
		   break;

       default:
    	   osDelay(20);
    	   break;
       }

     }
     comment out when doing angle adjustments -- */

    }



  /* USER CODE END startUIControl */
}


uint8_t check_time_bounds(uint8_t curr_hrs){
	//check time bounds, account for wrap (i.e. give time bounds of 10a-3a)

	uint8_t starthr = RTC_Bcd2ToByte(GlobalState.timeBound.startHR_BCD);
	uint8_t endhr = RTC_Bcd2ToByte(GlobalState.timeBound.endHR_BCD);

	if (starthr < endhr){
		return (curr_hrs >= starthr && curr_hrs < endhr);
	} else {
		return (curr_hrs >= starthr || curr_hrs < endhr);
	}

}

/* USER CODE BEGIN Header_startESMMain */
/**
* @brief Function implementing the esmMain thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startESMMain */
void startESMMain(void *argument)
{
  /* USER CODE BEGIN startESMMain */


  const BLETX_Queue_t bleSendInit = {TX_SURVEY_INITIALIZED, 0x0000};

  uint32_t notification;

  RTC_TimeTypeDef cTime;
  RTC_DateTypeDef cDate;

  /* Infinite loop */
  for(;;)
  {

	//only check time 3 times a min to see if we need a survey
    osDelay(20000); //20 sec delay

    //Grab current time
    osMutexAcquire(rtcMutexHandle, portMAX_DELAY);
    HAL_RTC_GetTime(&hrtc, &cTime, RTC_FORMAT_BCD);
    HAL_RTC_GetDate(&hrtc, &cDate, RTC_FORMAT_BCD);
    osMutexRelease(rtcMutexHandle);

    uint8_t curr_hrs = RTC_Bcd2ToByte(cTime.Hours);
    uint8_t curr_min = RTC_Bcd2ToByte(cTime.Minutes);

    //Get a date in YYMMDD format so we can easily compare relative values
    uint32_t curr_date = (cDate.Year << 16) | (cDate.Month << 8) | cDate.Date;

    //Grab last seen time hrs/min and date
    uint8_t last_hrs = RTC_Bcd2ToByte(GlobalState.lastSeenTime.time.Hours);
    uint8_t last_min = RTC_Bcd2ToByte(GlobalState.lastSeenTime.time.Minutes);
    uint32_t last_date = (GlobalState.lastSeenTime.date.Year << 16) |
    					 (GlobalState.lastSeenTime.date.Month << 8) |
						  GlobalState.lastSeenTime.date.Date;

    uint8_t sameDayFlag = 1;

    //Going to do time in minutes for ease.  60 min *24 hours = 1440 min / day

    uint16_t current_time_in_min = (60*curr_hrs + curr_min);

    uint16_t thresh_time_in_min = (60*last_hrs + last_min + GlobalState.currentInterval);
	if (thresh_time_in_min >= 1440){
		thresh_time_in_min %= 1440;
		sameDayFlag = 0;
	}

	// if thresh time doesn't roll over, we can safely assume that any time <= thresh_time
	// should trigger (sameDayFlag=1)
	// if we *do* roll over, any day that is greater than the last_seen_day should trigger

    //Init Survey:
    // TIME BOUNDS for current time hrs
    // curr_time == last_time + interval  (minute resolution, this is checked every 15 sec).
    // programMode is RESTING
    // not GlobalState.paused
    if (GlobalState.programMode == MODE_RESTING && !GlobalState.paused &&
    	check_time_bounds(curr_hrs) && current_time_in_min >= thresh_time_in_min &&
		(sameDayFlag | (curr_date > last_date )) ){

    	//send TX_SURVEY_INITIALIZED
    	osMessageQueuePut(bleTXqueueHandle, &bleSendInit, 0, 0);

    	//set program mode to MODE_ESM_TIME_ESTIMATE
    	osMutexAcquire(modeMutexHandle, portMAX_DELAY);
    	GlobalState.programMode = MODE_ESM_TIME_ESTIMATE;
    	osMutexRelease(modeMutexHandle);

    	//clear UI notification flags
    	xTaskNotifyStateClear(NULL);

    	uint8_t continue_flag = 1;

    	//(1) Alert Loop and ESM_TIME_ESTIMATE
    	uint8_t keep_alerting = 1;
    	while(keep_alerting){

    		//alert
    		//wait for notification from UI thread that indicates start of user interaction
    		notification = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(ALERT_TIMEOUT));
       	    if (notification){ //not a timeout, interaction started
        		keep_alerting = 0;
     		}
    	    if (GlobalState.programMode != MODE_ESM_TIME_ESTIMATE){
    	    	//button press has changed mode and canceled interaction. want to exit alert loop.
    	    	keep_alerting = 0;
    	    	continue_flag = 0;
    	    }

    	    osDelay(10);
        }

    	//SECOND SCREEN FOR ESM - FOCUS
    	if (continue_flag){
    		//(2) Wait for notification from UI thread that indicates confirmed input
    		notification = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(INTERACTION_TIMEOUT));

    		if (GlobalState.programMode != MODE_ESM_TIME_ESTIMATE){
    			//if our mode has changed, we had a dismiss/snooze event
    			continue_flag = 0;
    		}
    	}

    	if (continue_flag){
    		if (notification){//not timed out, had a confirmed event
    			//set up next interaction
    			//set up survey
    			osMutexAcquire(surveyMutexHandle, portMAX_DELAY);
    			strncpy(GlobalState.surveyState.screenText, "    FOCUS?", strlen("    FOCUS?") + 1);
    			GlobalState.surveyState.screenTextLength = strlen("    FOCUS?");
    			GlobalState.surveyState.surveyID = SURVEY_FOCUS;
    			memcpy(GlobalState.surveyState.optionArray, opts_arousal, sizeof(opts_arousal));
    			GlobalState.surveyState.optionArrayLength = 5;
    			osMutexRelease(surveyMutexHandle);

    			//programMode
    			osMutexAcquire(modeMutexHandle, portMAX_DELAY);
    			GlobalState.programMode = MODE_ESM_SURVEY;
    			osMutexRelease(modeMutexHandle);

    		} else {//timed out due to inactivity
    			continue_flag = 0;

    			osMutexAcquire(modeMutexHandle, portMAX_DELAY);
    			GlobalState.programMode = MODE_CLEAR;
    			osMutexRelease(modeMutexHandle);
    		}
    	}

    	//THIRD SCREEN FOR ESM - ALERTNESS/AROUSAL
		if (continue_flag){
			//(2) Wait for notification from UI thread that indicates confirmed input
			notification = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(INTERACTION_TIMEOUT));

			if (GlobalState.programMode != MODE_ESM_SURVEY){
				//if our mode has changed, we had a dismiss/snooze event
				continue_flag = 0;
			}
		}

		if (continue_flag){
			if (notification){//not timed out, had a confirmed event
				//set up next interaction
				//set up survey
				osMutexAcquire(surveyMutexHandle, portMAX_DELAY);
				strncpy(GlobalState.surveyState.screenText, " ALERTNESS?", strlen(" ALERTNESS?") + 1);
				GlobalState.surveyState.screenTextLength = strlen(" ALERTNESS?");
				GlobalState.surveyState.surveyID = SURVEY_AROUSAL;
				memcpy(GlobalState.surveyState.optionArray, opts_arousal, sizeof(opts_arousal));
				GlobalState.surveyState.optionArrayLength = 5;
				osMutexRelease(surveyMutexHandle);

			} else {//timed out due to inactivity
				continue_flag = 0;

				osMutexAcquire(modeMutexHandle, portMAX_DELAY);
				GlobalState.programMode = MODE_CLEAR;
				osMutexRelease(modeMutexHandle);
			}
		}

		//FOURTH SCREEN FOR ESM - EMOTION/VALENCE
		if (continue_flag){
			//(2) Wait for notification from UI thread that indicates confirmed input
			notification = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(INTERACTION_TIMEOUT));

			if (GlobalState.programMode != MODE_ESM_SURVEY){
				//if our mode has changed, we had a dismiss/snooze event
				continue_flag = 0;
			}
		}

		if (continue_flag){
			if (notification){//not timed out, had a confirmed event
				//set up next interaction
				//set up survey
				osMutexAcquire(surveyMutexHandle, portMAX_DELAY);
				strncpy(GlobalState.surveyState.screenText, "  EMOTION?", strlen("  EMOTION?") + 1);
				GlobalState.surveyState.screenTextLength = strlen("  EMOTION?");
				GlobalState.surveyState.surveyID = SURVEY_VALENCE;
				memcpy(GlobalState.surveyState.optionArray, opts_valence, sizeof(opts_valence));
				GlobalState.surveyState.optionArrayLength = 5;
				osMutexRelease(surveyMutexHandle);

			} else {//timed out due to inactivity
				continue_flag = 0;

				osMutexAcquire(modeMutexHandle, portMAX_DELAY);
				GlobalState.programMode = MODE_CLEAR;
				osMutexRelease(modeMutexHandle);
			}
		}

		//FIFTH SCREEN FOR ESM - COG LOAD
		if (continue_flag){
			//(2) Wait for notification from UI thread that indicates confirmed input
			notification = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(INTERACTION_TIMEOUT));

			if (GlobalState.programMode != MODE_ESM_SURVEY){
				//if our mode has changed, we had a dismiss/snooze event
				continue_flag = 0;
			}
		}

		if (continue_flag){
			if (notification){//not timed out, had a confirmed event
				//set up next interaction
				//set up survey
				osMutexAcquire(surveyMutexHandle, portMAX_DELAY);
				strncpy(GlobalState.surveyState.screenText, "COG. EFFORT?", strlen("COG. EFFORT?") + 1);
				GlobalState.surveyState.screenTextLength = strlen("COG. EFFORT?");
				GlobalState.surveyState.surveyID = SURVEY_COGLOAD;
				memcpy(GlobalState.surveyState.optionArray, opts_arousal, sizeof(opts_arousal));
				GlobalState.surveyState.optionArrayLength = 5;
				osMutexRelease(surveyMutexHandle);

			} else {//timed out due to inactivity
				continue_flag = 0;

				osMutexAcquire(modeMutexHandle, portMAX_DELAY);
				GlobalState.programMode = MODE_CLEAR;
				osMutexRelease(modeMutexHandle);
			}
		}

		//SIXTH SCREEN FOR ESM - STRESS
		if (continue_flag){
			//(2) Wait for notification from UI thread that indicates confirmed input
			notification = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(INTERACTION_TIMEOUT));

			if (GlobalState.programMode != MODE_ESM_SURVEY){
				//if our mode has changed, we had a dismiss/snooze event
				continue_flag = 0;
			}
		}

		if (continue_flag){
			if (notification){//not timed out, had a confirmed event
				//set up next interaction
				//set up survey
				osMutexAcquire(surveyMutexHandle, portMAX_DELAY);
				strncpy(GlobalState.surveyState.screenText, "  STRESS?", strlen("  STRESS?") + 1);
				GlobalState.surveyState.screenTextLength = strlen("  STRESS?");
				GlobalState.surveyState.surveyID = SURVEY_STRESS;
				memcpy(GlobalState.surveyState.optionArray, opts_arousal, sizeof(opts_arousal));
				GlobalState.surveyState.optionArrayLength = 5;
				osMutexRelease(surveyMutexHandle);

			} else {//timed out due to inactivity
				continue_flag = 0;

				osMutexAcquire(modeMutexHandle, portMAX_DELAY);
				GlobalState.programMode = MODE_CLEAR;
				osMutexRelease(modeMutexHandle);
			}
		}

		//SEVENTH SCREEN FOR ESM - CAFFEINE
		if (continue_flag){
			//(2) Wait for notification from UI thread that indicates confirmed input
			notification = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(INTERACTION_TIMEOUT));

			if (GlobalState.programMode != MODE_ESM_SURVEY){
				//if our mode has changed, we had a dismiss/snooze event
				continue_flag = 0;
			}
		}

		if (continue_flag){
			if (notification){//not timed out, had a confirmed event
				//set up next interaction
				//set up survey
				osMutexAcquire(surveyMutexHandle, portMAX_DELAY);
				strncpy(GlobalState.surveyState.screenText, " CAFFEINE?", strlen(" CAFFEINE?") + 1);
				GlobalState.surveyState.screenTextLength = strlen(" CAFFEINE?");
				GlobalState.surveyState.surveyID = SURVEY_CAFFEINE;
				memcpy(GlobalState.surveyState.optionArray, opts_yes, sizeof(opts_yes));
				GlobalState.surveyState.optionArrayLength = 2;
				osMutexRelease(surveyMutexHandle);

			} else {//timed out due to inactivity
				continue_flag = 0;

				osMutexAcquire(modeMutexHandle, portMAX_DELAY);
				GlobalState.programMode = MODE_CLEAR;
				osMutexRelease(modeMutexHandle);
			}
		}

		//EIGHTH SCREEN FOR ESM - EXERCISE
		if (continue_flag){
			//(2) Wait for notification from UI thread that indicates confirmed input
			notification = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(INTERACTION_TIMEOUT));

			if (GlobalState.programMode != MODE_ESM_SURVEY){
				//if our mode has changed, we had a dismiss/snooze event
				continue_flag = 0;
			}
		}

		if (continue_flag){
			if (notification){//not timed out, had a confirmed event
				//set up next interaction
				//set up survey
				osMutexAcquire(surveyMutexHandle, portMAX_DELAY);
				strncpy(GlobalState.surveyState.screenText, " EXERCISE?", strlen(" EXERCISE?") + 1);
				GlobalState.surveyState.screenTextLength = strlen(" EXERCISE?");
				GlobalState.surveyState.surveyID = SURVEY_EXERCISE;
				memcpy(GlobalState.surveyState.optionArray, opts_yes, sizeof(opts_yes));
				GlobalState.surveyState.optionArrayLength = 2;
				osMutexRelease(surveyMutexHandle);

			} else {//timed out due to inactivity
				continue_flag = 0;

				osMutexAcquire(modeMutexHandle, portMAX_DELAY);
				GlobalState.programMode = MODE_CLEAR;
				osMutexRelease(modeMutexHandle);
			}
		}

		//NINETH SCREEN FOR ESM - TIME CUE
		if (continue_flag){
			//(2) Wait for notification from UI thread that indicates confirmed input
			notification = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(INTERACTION_TIMEOUT));

			if (GlobalState.programMode != MODE_ESM_SURVEY){
				//if our mode has changed, we had a dismiss/snooze event
				continue_flag = 0;
			}
		}

		if (continue_flag){
			if (notification){//not timed out, had a confirmed event
				//set up next interaction
				//set up survey
				osMutexAcquire(surveyMutexHandle, portMAX_DELAY);
				strncpy(GlobalState.surveyState.screenText, " TIME CUE?", strlen(" TIME CUE?") + 1);
				GlobalState.surveyState.screenTextLength = strlen(" TIME CUE?");
				GlobalState.surveyState.surveyID = SURVEY_TIMECUE;
				memcpy(GlobalState.surveyState.optionArray, opts_yes, sizeof(opts_yes));
				GlobalState.surveyState.optionArrayLength = 2;
				osMutexRelease(surveyMutexHandle);

			} else {//timed out due to inactivity
				continue_flag = 0;

				osMutexAcquire(modeMutexHandle, portMAX_DELAY);
				GlobalState.programMode = MODE_CLEAR;
				osMutexRelease(modeMutexHandle);
			}
		}

		//TENTH SCREEN FOR ESM - LOCATION
		if (continue_flag){
			//(2) Wait for notification from UI thread that indicates confirmed input
			notification = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(INTERACTION_TIMEOUT));

			if (GlobalState.programMode != MODE_ESM_SURVEY){
				//if our mode has changed, we had a dismiss/snooze event
				continue_flag = 0;
			}
		}

		if (continue_flag){
			if (notification){//not timed out, had a confirmed event
				//set up next interaction
				//set up survey
				osMutexAcquire(surveyMutexHandle, portMAX_DELAY);
				strncpy(GlobalState.surveyState.screenText, " LOCATION?", strlen(" LOCATION?") + 1);
				GlobalState.surveyState.screenTextLength = strlen(" LOCATION?");
				GlobalState.surveyState.surveyID = SURVEY_LOCATE;
				memcpy(GlobalState.surveyState.optionArray, opts_location, sizeof(opts_location));
				GlobalState.surveyState.optionArrayLength = 2;
				osMutexRelease(surveyMutexHandle);

			} else {//timed out due to inactivity
				continue_flag = 0;

				osMutexAcquire(modeMutexHandle, portMAX_DELAY);
				GlobalState.programMode = MODE_CLEAR;
				osMutexRelease(modeMutexHandle);
			}
		}

		//ELEVENTH SCREEN FOR ESM - THERMAL SENSATION
		if (continue_flag){
			//(2) Wait for notification from UI thread that indicates confirmed input
			notification = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(INTERACTION_TIMEOUT));

			if (GlobalState.programMode != MODE_ESM_SURVEY){
				//if our mode has changed, we had a dismiss/snooze event
				continue_flag = 0;
			}
		}

		if (continue_flag){
			if (notification){//not timed out, had a confirmed event
				//set up next interaction
				//set up survey
				osMutexAcquire(surveyMutexHandle, portMAX_DELAY);
				strncpy(GlobalState.surveyState.screenText, " SENSATION?", strlen(" SENSATION?") + 1);
				GlobalState.surveyState.screenTextLength = strlen(" SENSATION?");
				GlobalState.surveyState.surveyID = SURVEY_TSENSE;
				memcpy(GlobalState.surveyState.optionArray, opts_thermalsense, sizeof(opts_thermalsense));
				GlobalState.surveyState.optionArrayLength = 7;
				osMutexRelease(surveyMutexHandle);

			} else {//timed out due to inactivity
				continue_flag = 0;

				osMutexAcquire(modeMutexHandle, portMAX_DELAY);
				GlobalState.programMode = MODE_CLEAR;
				osMutexRelease(modeMutexHandle);
			}
		}

		//TWELFTH SCREEN FOR ESM - THERMAL PREF
		if (continue_flag){
			//(2) Wait for notification from UI thread that indicates confirmed input
			notification = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(INTERACTION_TIMEOUT));

			if (GlobalState.programMode != MODE_ESM_SURVEY){
				//if our mode has changed, we had a dismiss/snooze event
				continue_flag = 0;
			}
		}

		if (continue_flag){
			if (notification){//not timed out, had a confirmed event
				//set up next interaction
				//set up survey
				osMutexAcquire(surveyMutexHandle, portMAX_DELAY);
				strncpy(GlobalState.surveyState.screenText, "TEMP PREFER?", strlen("TEMP PREFER?") + 1);
				GlobalState.surveyState.screenTextLength = strlen("TEMP PREFER?");
				GlobalState.surveyState.surveyID = SURVEY_TCOMFORT;
				memcpy(GlobalState.surveyState.optionArray, opts_thermalcomfort, sizeof(opts_thermalcomfort));
				GlobalState.surveyState.optionArrayLength = 3;
				osMutexRelease(surveyMutexHandle);

			} else {//timed out due to inactivity
				continue_flag = 0;

				osMutexAcquire(modeMutexHandle, portMAX_DELAY);
				GlobalState.programMode = MODE_CLEAR;
				osMutexRelease(modeMutexHandle);
			}
		}

		//FINISH ESM; SHOW TIME
		if (continue_flag){
			//(2) Wait for notification from UI thread that indicates confirmed input
			notification = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(INTERACTION_TIMEOUT));

			if (GlobalState.programMode != MODE_ESM_SURVEY){
				//if our mode has changed, we had a dismiss/snooze event
				continue_flag = 0;
			}
		}

		if (continue_flag){
			if (notification){//not timed out, had a confirmed event
				//completed survey with no problem!

				//back to rest, show time
				osMutexAcquire(modeMutexHandle, portMAX_DELAY);
				GlobalState.programMode = MODE_SHOW_TIME;
				osMutexRelease(modeMutexHandle);;

			} else {//timed out due to inactivity
				continue_flag = 0;

				osMutexAcquire(modeMutexHandle, portMAX_DELAY);
				GlobalState.programMode = MODE_CLEAR;
				osMutexRelease(modeMutexHandle);
			}
		}

    } else {//not time for a survey

    	//get hour for last_seen + interval (trigger) time
    	uint8_t extra_hours = (last_min + GlobalState.currentInterval) / 60;
    	last_hrs = (last_hrs + extra_hours) % 24;

    	//if next trigger is outside of timebound (i.e. last seen time is end of the day) AND
    	//its the morning (curr_hr = startHR), put it in paused mode, wait for user to initiate.
    	if (!check_time_bounds(last_hrs) && (curr_hrs == RTC_Bcd2ToByte(GlobalState.timeBound.startHR_BCD)) && !GlobalState.paused){
    		GlobalState.paused = 1;
    		const BLETX_Queue_t bleSendPause = {TX_BEGIN_PAUSE, 0x0000};
     		osMessageQueuePut(bleTXqueueHandle, &bleSendPause, 0, 0);


    	}

    }


  }
  /* USER CODE END startESMMain */
}

/* USER CODE BEGIN Header_startButtonPress */
/**
* @brief Function implementing the buttonPress thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startButtonPress */
void startButtonPress(void *argument)
{
  /* USER CODE BEGIN startButtonPress */
  /* Infinite loop */

  //Buttons are PULLED UP and drop to 0 when pressed
  uint8_t buttonState[] = {1, 1, 1};
  uint32_t callingPin = 0x00;

  const BLETX_Queue_t bleSendData = {TX_PREVIOUS_INVALID, 0x0000};

  for(;;)
  {
	//wait for rising or falling edge trigger, put calling pin in callingPin
	xTaskNotifyWait(0x00, 0x00, &callingPin, portMAX_DELAY);

	//check state of pin
	GPIO_PinState first_read = HAL_GPIO_ReadPin(GPIOB, callingPin);

	//wait 50ms
    osDelay(50);

    //check again (debounce) to get a good reading
	if (first_read == HAL_GPIO_ReadPin(GPIOB, callingPin)){

		//check that we are actually changing state; the edge trigger should only call
		//when this happens (except during debouncing) so we expect this to be true
		//almost always

		//callingPin can be used as bitmask Pin 5/4/3 give 1000000/10000/1000

		if (callingPin == 0b1000 && first_read != buttonState[0]) { //button 1 trigger
		  //set buttonState
		  buttonState[0] = first_read;

		  //do stuff if button pressed
		  if (!first_read){
			  if (GlobalState.programMode){
			   GlobalState.programMode = 0;
			  } else {
			   GlobalState.programMode = 1;
			  }
		  }

		}
		if (callingPin == 0b10000 && first_read != buttonState[1]) { //button 2 trigger
		    //set buttonState
		    buttonState[1] = first_read;

		    //do stuff if button pressed
		    if (!first_read){
		    	  if (GlobalState.programMode){
				   GlobalState.programMode = 0;
				  } else {
				   GlobalState.programMode = 1;
				  }

		    }
		}
		if (callingPin == 0b100000 && first_read != buttonState[2]) { //button 3 trigger
		    //set buttonState
		    buttonState[2] = first_read;

		    //do stuff if button pressed
		    if (!first_read){
		    	  if (GlobalState.programMode){
				   GlobalState.programMode = 0;
				  } else {
				  GlobalState.programMode = 1;
				  }
		    }
		}

	}

  }
  /* USER CODE END startButtonPress */
}


/* USER CODE BEGIN Header_startConditionsPoll */
/**
* @brief Function implementing the conditionsPoll thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startConditionsPoll */
void startConditionsPoll(void *argument)
{
	for(;;){
	osDelay(15000);
	}
/* USER CODE END startConditionsPoll */
}


/* USER CODE BEGIN Header_startBLETX */
/**
* @brief Function implementing the bleTX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startBLETX */
void startBLETX(void *argument)
{
  /* USER CODE BEGIN startBLETX */

  //TX_TIME_SEEN - data in GlobalState.lastSeenTime
  //TX_TEMP_HUMD - construct both TX_TEMP_HUMD/TX_LUX_WHITELUX
  //               with data in GlobalState.lastConditions

  //TX_PREVIOUS_INVALID -- just send that timestamped
  //TX_SURVEY_INITIALIZED -- just send that timestamped

  //TX_TIME_EST - use data in sendData, first byte is hr second byte is min
  //TX_TIMESTAMP_UPDATE -- use data in sendData for error, to send
  //TX_SURVEY_RESULT -- use data in sendData, first byte is survey/second is answer

  BLETX_Queue_t sendData;

  UnsentQueueAddress_t DataQueue = NULL;

  RTC_TimeTypeDef cTime;
  RTC_DateTypeDef cDate;
  uint16_t sendval[10] = {0};
  uint16_t lightval[4] = {0};
  uint8_t numBytes = 0;

  /* Infinite loop */
  for(;;)
  {
        if (osMessageQueueGet(bleTXqueueHandle, &sendData, NULL, osWaitForever) == osOK){

          //construct timestamped data to send
          osMutexAcquire(rtcMutexHandle, portMAX_DELAY);
          HAL_RTC_GetTime(&hrtc, &cTime, RTC_FORMAT_BCD);
          HAL_RTC_GetDate(&hrtc, &cDate, RTC_FORMAT_BCD);
          osMutexRelease(rtcMutexHandle);

          switch (sendData.sendType){
            case TX_PREVIOUS_INVALID:
            case TX_SURVEY_INITIALIZED:
            case TX_BEGIN_PAUSE:

            	sendval[4] = (cDate.WeekDay << (8*1)) | cDate.Month;
            	sendval[3] = (cDate.Date << (8*1)) | cDate.Year;
            	sendval[2] = (cTime.Hours << (8*1)) | cTime.Minutes;
            	sendval[1] = (cTime.Seconds << (8*1)) | cTime.TimeFormat;
            	sendval[0] = sendData.sendType;
            	numBytes = 10;
            	break;

            case TX_TIME_EST:
            case TX_TIMESTAMP_UPDATE:
            case TX_SURVEY_RESULT:
            	sendval[5] = (cDate.WeekDay << (8*1)) | cDate.Month;
            	sendval[4] = (cDate.Date << (8*1)) | cDate.Year;
            	sendval[3] = (cTime.Hours << (8*1)) | cTime.Minutes;
            	sendval[2] = (cTime.Seconds << (8*1)) | cTime.TimeFormat;
            	sendval[1] = sendData.sendType;
            	sendval[0] = sendData.data;
            	numBytes = 12;
            	break;

            case TX_TIME_SEEN:
            	sendval[8] = (cDate.WeekDay << (8*1)) | cDate.Month;
            	sendval[7] = (cDate.Date << (8*1)) | cDate.Year;
            	sendval[6] = (cTime.Hours << (8*1)) | cTime.Minutes;
            	sendval[5] = (cTime.Seconds << (8*1)) | cTime.TimeFormat;
            	sendval[4] = sendData.sendType;

                osMutexAcquire(lastSeenMutexHandle, portMAX_DELAY);
                cTime = GlobalState.lastSeenTime.time;
                cDate = GlobalState.lastSeenTime.date;
                osMutexRelease(lastSeenMutexHandle);

                sendval[3] = (cDate.WeekDay << (8*1)) | cDate.Month;
                sendval[2] = (cDate.Date << (8*1)) | cDate.Year;
                sendval[1] = (cTime.Hours << (8*1)) | cTime.Minutes;
                sendval[0] = (cTime.Seconds << (8*1)) | cTime.TimeFormat;
            	numBytes = 18;
            	break;

            case TX_TEMP_HUMD:
            case TX_LUX_WHITELUX:
            	sendval[8] = (cDate.WeekDay << (8*1)) | cDate.Month;
            	sendval[7] = (cDate.Date << (8*1)) | cDate.Year;
            	sendval[6] = (cTime.Hours << (8*1)) | cTime.Minutes;
            	sendval[5] = (cTime.Seconds << (8*1)) | cTime.TimeFormat;
            	sendval[4] = TX_TEMP_HUMD;
            	numBytes = 18;

            	osMutexAcquire(conditionMutexHandle, portMAX_DELAY);
            	memcpy(&(sendval[2]), &GlobalState.lastConditions.temp, 4);
            	memcpy(sendval, &GlobalState.lastConditions.humd, 4);
            	memcpy(&(lightval[2]), &GlobalState.lastConditions.lux, 4);
            	memcpy(lightval, &GlobalState.lastConditions.whiteLux, 4);
            	osMutexRelease(conditionMutexHandle);
            	break;

                //CONSTRUCT SECOND PACKET FOR CONDITIONS
                //sendval[4] = TX_LUX_WHITELUX
                //memcpy(sendval, lightval, 8);
          }


          //try to send queued data if we have a queue
          uint8_t dataSuccessFlag = 1;
          uint8_t failed_attempts = 0;
		  const uint8_t MAX_ATTEMPTS = 50;

          if (!P2P_Server_App_Context.Connected) { dataSuccessFlag = 0;}

          while(DataQueue && dataSuccessFlag){//we have data queued and have not failed to send data

        	  //try to send data at front of list
        	  if (P2PS_STM_App_Update_Int8(P2P_NOTIFY_CHAR_UUID, (uint8_t *)DataQueue->packet, DataQueue->numBytes) == BLE_STATUS_SUCCESS){

        		  //if successful, move dataQueue to next, which is NULL for last element, and free memory
        		  UnsentQueueAddress_t addressJustSent = DataQueue;
        		  DataQueue = DataQueue->next;

        		  vPortFree(addressJustSent->packet);
        		  vPortFree(addressJustSent);

        		  failed_attempts = 0;
        	  } else {  //if unsuccessful MAX_ATTEMPTS in a row, dataSuccessFlag = 0
        		  failed_attempts +=1;

        		  if (failed_attempts >= MAX_ATTEMPTS){ dataSuccessFlag = 0; }
        		  else { osDelay(2); }
        	  }
          }

        //if we haven't had a data failure with the queue, try to send current data packet
        if (dataSuccessFlag){
        	if (P2PS_STM_App_Update_Int8(P2P_NOTIFY_CHAR_UUID, (uint8_t *)&sendval, numBytes) != BLE_STATUS_SUCCESS){
        		//unsuccessful packet send means we flag it
        		dataSuccessFlag = 0;

        	} else if (sendData.sendType == TX_TEMP_HUMD){ //if first packet was successful and we're transmitting conditions

        		//construct second packet for conditions
                sendval[4] = TX_LUX_WHITELUX;
                memcpy(sendval, lightval, 8);

                //and send it
        		if (P2PS_STM_App_Update_Int8(P2P_NOTIFY_CHAR_UUID, (uint8_t *)&sendval, numBytes) != BLE_STATUS_SUCCESS){
        		        dataSuccessFlag = 0;
        		}
        	}

        }

        //if we had a data send failure, add current packet to the queue dynamically
        if (!dataSuccessFlag){

        	//malloc the packet data
        	uint16_t *newPacketAddress = pvPortMalloc(numBytes);
        	//copy the packet data in from sendval
        	memcpy(newPacketAddress, sendval, numBytes);

        	//malloc the queue item that points to that data
        	UnsentQueue_t *newQueueItemAddress = pvPortMalloc(sizeof(UnsentQueue_t));
        	//correctly fill the new Queue Item
        	newQueueItemAddress->packet = newPacketAddress;
        	newQueueItemAddress->numBytes = numBytes;
        	newQueueItemAddress->next = NULL;

        	//if DataQueue is empty, simply set the queue address to this one.
        	if (!DataQueue){ DataQueue = newQueueItemAddress; }
        	else {//otherwise traverse until we get null
        		UnsentQueueAddress_t current_node = DataQueue;
        		while (current_node->next){ //while the pointer to the next is not null
        			current_node = current_node->next; //update current_node to next
        		}
        		current_node->next = newQueueItemAddress;
        	}

        	if (sendData.sendType == TX_TEMP_HUMD && sendval[4] == TX_TEMP_HUMD){
        		//if we're sending conditions and we didn't successfully move to second packet,
        		//add second packet to data queue as well
        		sendval[4] = TX_LUX_WHITELUX;
        		memcpy(sendval, lightval, 8);

        		//malloc the packet data
				uint16_t *newPacketAddress = pvPortMalloc(numBytes);
				//copy the packet data in from sendval
				memcpy(newPacketAddress, sendval, numBytes);

				//malloc the queue item that points to that data
				UnsentQueue_t *newQueueItemAddress = pvPortMalloc(sizeof(UnsentQueue_t));
				//correctly fill the new Queue Item
				newQueueItemAddress->packet = newPacketAddress;
				newQueueItemAddress->numBytes = numBytes;
				newQueueItemAddress->next = NULL;

				UnsentQueueAddress_t current_node = DataQueue;

				while (current_node->next){ //while the pointer to the next is not null
					current_node = current_node->next; //update current_node to next
				}

				current_node->next = newQueueItemAddress;

        	}
        }
    }
  }
  /* USER CODE END startBLETX */
}

/* USER CODE BEGIN Header_startBLERX */
/**
* @brief Function implementing the bleRX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startBLERX */
void startBLERX(void *argument)
{
  /* USER CODE BEGIN startBLERX */
  /* Infinite loop */

  P2PS_STM_Data_t rxData;

  for(;;)
  {

	if (osMessageQueueGet(bleRXqueueHandle, &rxData, NULL, osWaitForever) == osOK){

		if (rxData.pPayload[0] == 0x00) { // timestamp update starts with 0x00
			memcpy(&P2P_Server_App_Context.OTATimestamp, &(rxData.pPayload[1]), 8);
    	    P2P_Server_App_Context.OTA12HrFormat = rxData.pPayload[9];
    		P2P_Server_App_Context.OTADaylightSavings = rxData.pPayload[10];

    	    RTC_TimeTypeDef sTime = {0};
    		RTC_DateTypeDef sDate = {0};

    		uint8_t timestampvals[8];
    		memcpy(timestampvals, &(P2P_Server_App_Context.OTATimestamp), 8);

    		uint8_t AMPM = timestampvals[0];

    		sTime.Hours      = timestampvals[4];
    		sTime.Minutes    = timestampvals[5];
    		sTime.Seconds    = timestampvals[6];
    		sTime.SubSeconds = 0x0;
    		sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;

    		if (P2P_Server_App_Context.OTADaylightSavings){ sTime.DayLightSaving = RTC_DAYLIGHTSAVING_ADD1H; }

    		sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    		sDate.WeekDay = timestampvals[0];
    		sDate.Month   = timestampvals[1];
    		sDate.Date    = timestampvals[2];
    		sDate.Year    = timestampvals[3];

    		RTC_TimeTypeDef cTime;
   	        RTC_DateTypeDef cDate;

    		osMutexAcquire(rtcMutexHandle, portMAX_DELAY);
    		HAL_RTC_GetTime(&hrtc, &cTime, RTC_FORMAT_BCD);
    		HAL_RTC_GetDate(&hrtc, &cDate, RTC_FORMAT_BCD);
    		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {Error_Handler();}
    		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK) {Error_Handler();}
    	    osMutexRelease(rtcMutexHandle);

    	    //calculate the seconds off between the two.
    	    uint8_t prev_hrs = RTC_Bcd2ToByte(cTime.Hours);
    	    uint8_t prev_min = RTC_Bcd2ToByte(cTime.Minutes);
    	    uint8_t prev_sec = RTC_Bcd2ToByte(cTime.Seconds);

    	    uint8_t new_hrs = RTC_Bcd2ToByte(sTime.Hours);
    	    uint8_t new_min = RTC_Bcd2ToByte(sTime.Minutes);
    	    uint8_t new_sec = RTC_Bcd2ToByte(sTime.Seconds);

    	    int32_t new_totalsec = (60*60*new_hrs + 60*new_min + new_sec); //86400 sec in day
    	    int32_t prev_totalsec = (60*60*prev_hrs + 60*prev_min + prev_sec);

    	    int32_t forward_diff;
    	    int32_t backward_diff;

    	    int16_t signed_sec_difference; //cant hold more than 9 hours difference

    	    if (new_totalsec > prev_totalsec) {
    	    	forward_diff  = new_totalsec - prev_totalsec;
    	    	backward_diff = 86400 - forward_diff;
    	    } else {
    	    	backward_diff = prev_totalsec - new_totalsec;
    	    	forward_diff  = 86400 - backward_diff;
    	    }

    	    if (backward_diff < forward_diff){
    	    	signed_sec_difference= -1 * backward_diff;
    	    }else {
    	    	signed_sec_difference = forward_diff;
    	    }

    	    BLETX_Queue_t bleSendUpdate = {TX_TIMESTAMP_UPDATE, 0x00};
			bleSendUpdate.data = (uint16_t)signed_sec_difference;
			osMessageQueuePut(bleTXqueueHandle, &bleSendUpdate, 0, 0);
		}

		else if (rxData.pPayload[0] == 0x01) {//change time bounds
			//startHR, endHR in BCD
			GlobalState.timeBound.startHR_BCD = rxData.pPayload[1];
			GlobalState.timeBound.endHR_BCD  = rxData.pPayload[2];

			GlobalState.paused = 1;
			const BLETX_Queue_t bleSendPause = {TX_BEGIN_PAUSE, 0x0000};
			osMessageQueuePut(bleTXqueueHandle, &bleSendPause, 0, 0);
		}

		else if (rxData.pPayload[0] == 0x02) {//pause or unpause watch

			if (rxData.pPayload[1]) { //pause things

				GlobalState.paused = 1;
				osMutexAcquire(modeMutexHandle, portMAX_DELAY);
				GlobalState.programMode = MODE_RESTING;
				osMutexRelease(modeMutexHandle);
	    		const BLETX_Queue_t bleSendPause = {TX_BEGIN_PAUSE, 0x0000};
	     		osMessageQueuePut(bleTXqueueHandle, &bleSendPause, 0, 0);

			} else { //unpause things

				osMutexAcquire(modeMutexHandle, portMAX_DELAY);
				GlobalState.programMode = MODE_SHOW_TIME;
				osMutexRelease(modeMutexHandle);
				GlobalState.paused = 0;
			}

		}

	}
  }
  /* USER CODE END startBLERX */
}


void PeriphClock_Config(void)
{
  #if (CFG_USB_INTERFACE_ENABLE != 0)
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };
	RCC_CRSInitTypeDef RCC_CRSInitStruct = { 0 };

	/**
   * This prevents the CPU2 to disable the HSI48 oscillator when
   * it does not use anymore the RNG IP
   */
  LL_HSEM_1StepLock( HSEM, 5 );

  LL_RCC_HSI48_Enable();

	while(!LL_RCC_HSI48_IsReady());

	/* Select HSI48 as USB clock source */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	/*Configure the clock recovery system (CRS)**********************************/

	/* Enable CRS Clock */
	__HAL_RCC_CRS_CLK_ENABLE();

	/* Default Synchro Signal division factor (not divided) */
	RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;

	/* Set the SYNCSRC[1:0] bits according to CRS_Source value */
	RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;

	/* HSI48 is synchronized with USB SOF at 1KHz rate */
	RCC_CRSInitStruct.ReloadValue = RCC_CRS_RELOADVALUE_DEFAULT;
	RCC_CRSInitStruct.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;

	RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;

	/* Set the TRIM[5:0] to the default value*/
	RCC_CRSInitStruct.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;

	/* Start automatic synchronization */
	HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
#endif

	return;
}
/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

static void Config_HSE(void)
{
    OTP_ID0_t * p_otp;

  /**
   * Read HSE_Tuning from OTP
   */
  p_otp = (OTP_ID0_t *) OTP_Read(0);
  if (p_otp)
  {
    LL_RCC_HSE_SetCapacitorTuning(p_otp->hse_tuning);
  }

  return;
}  

static void Reset_Device( void )
{
#if ( CFG_HW_RESET_BY_FW == 1 )
	Reset_BackupDomain();

	Reset_IPCC();
#endif

	return;
}

static void Reset_IPCC( void )
{
	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_IPCC);

	LL_C1_IPCC_ClearFlag_CHx(
			IPCC,
			LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
			| LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

	LL_C2_IPCC_ClearFlag_CHx(
			IPCC,
			LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
			| LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

	LL_C1_IPCC_DisableTransmitChannel(
			IPCC,
			LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
			| LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

	LL_C2_IPCC_DisableTransmitChannel(
			IPCC,
			LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
			| LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

	LL_C1_IPCC_DisableReceiveChannel(
			IPCC,
			LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
			| LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

	LL_C2_IPCC_DisableReceiveChannel(
			IPCC,
			LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
			| LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

	return;
}

static void Reset_BackupDomain( void )
{
	if ((LL_RCC_IsActiveFlag_PINRST() != FALSE) && (LL_RCC_IsActiveFlag_SFTRST() == FALSE))
	{
		HAL_PWR_EnableBkUpAccess(); /**< Enable access to the RTC registers */

		/**
		 *  Write twice the value to flush the APB-AHB bridge
		 *  This bit shall be written in the register before writing the next one
		 */
		HAL_PWR_EnableBkUpAccess();

		__HAL_RCC_BACKUPRESET_FORCE();
		__HAL_RCC_BACKUPRESET_RELEASE();
	}

	return;
}

static void Init_Exti( void )
{
  /**< Disable all wakeup interrupt on CPU1  except IPCC(36), HSEM(38) */
  LL_EXTI_DisableIT_0_31(~0);
  LL_EXTI_DisableIT_32_63( (~0) & (~(LL_EXTI_LINE_36 | LL_EXTI_LINE_38)) );

  return;
}

/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/
void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /* Add a freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
    wait += HAL_GetTickFreq();
  }

  while ((HAL_GetTick() - tickstart) < wait)
  {
    /************************************************************************************
     * ENTER SLEEP MODE
     ***********************************************************************************/
    LL_LPM_EnableSleep( ); /**< Clear SLEEPDEEP bit of Cortex System Control Register */

    /**
     * This option is used to ensure that store operations are completed
     */
  #if defined ( __CC_ARM)
    __force_stores();
  #endif

    __WFI( );
  }
}

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/*
//ISSUES WITH MALLOC AND FREE AND FREERTOS, need to use included
//NEWLIB uses malloc to print non-integers with sprintf; this functionality is broken
//with freertos without work

void *malloc( size_t xBytes )
{
     return pvPortMalloc( xBytes );
}

void free( void *pvBuffer )
{
     vPortFree( pvBuffer );
}
*/

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
