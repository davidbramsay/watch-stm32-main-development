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
  .stack_size = 512 * 4
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
/* Definitions for alert */
osThreadId_t alertHandle;
const osThreadAttr_t alert_attributes = {
  .name = "alert",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 8
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
/* Definitions for timeEstimateMutex */
osMutexId_t timeEstimateMutexHandle;
const osMutexAttr_t timeEstimateMutex_attributes = {
  .name = "timeEstimateMutex"
};
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
void startUIControl(void *argument);
void startESMMain(void *argument);
void startButtonPress(void *argument);
void startAlert(void *argument);
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
    timeEstimateMutexHandle = osMutexNew(&timeEstimateMutex_attributes);

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

        /* creation of alert */
        alertHandle = osThreadNew(startAlert, NULL, &alert_attributes);

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
	GlobalState.timeBound.minInterval = 15;   //15min min interval
	GlobalState.timeBound.maxInterval = 90;   //90min max interval

	RTC_TimeTypeDef tempTime;
	RTC_DateTypeDef tempDate;
	HAL_RTC_GetTime(&hrtc, &tempTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &tempDate, RTC_FORMAT_BCD);

	//shallow structs so no issues with assignment
	GlobalState.lastSeenTime.time = tempTime;
	GlobalState.lastSeenTime.date = tempDate;

	GlobalState.timeEstimateSample.time = tempTime;
	GlobalState.timeEstimateSample.date = tempDate;

	GlobalState.lastConditions.lux = 0.0;
	GlobalState.lastConditions.whiteLux = 0.0;
	GlobalState.lastConditions.temp = 0.0;
	GlobalState.lastConditions.humd = 0.0;

	GlobalState.programMode = MODE_RESTING;

	GlobalState.surveyState.surveyID = SURVEY_NONE;
	char temp_string[10] = "  DRAMSAY.";
	strncpy(GlobalState.surveyState.screenText, temp_string, strlen(temp_string)+1);
	GlobalState.surveyState.screenTextLength = strlen(temp_string);
	memset(GlobalState.surveyState.optionArray, 0, sizeof(GlobalState.surveyState.optionArray));
	GlobalState.surveyState.optionArrayLength = 0;

	GlobalState.currentInterval = 0;
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
  /* USER CODE BEGIN 5 */
  HAL_GPIO_WritePin(OLED_RESET_GPIO_Port, OLED_RESET_Pin, GPIO_PIN_SET);

  uint8_t oled_buf[WIDTH * HEIGHT / 8];

  er_oled_begin();
  er_oled_clear(oled_buf);
  er_oled_string(0, 10, "  DRAMSAY.", 12, 1, oled_buf);
  er_oled_string(0, 28, "resenv | mit", 12, 1, oled_buf);
  er_oled_display(oled_buf);

  osDelay(3000);

  er_oled_clear(oled_buf);
  er_oled_display(oled_buf);

  int16_t current_minute = -1;
  int16_t last_minute = -1;

  uint8_t hrs, mins;
  char time[5];

  uint8_t touch_end_count = 0;

  RTC_TimeTypeDef cTime;
  RTC_DateTypeDef cDate;

  BLETX_Queue_t bleSendData;

  //init peripheral (not turbo mode, poll every 250ms, if touch sample at 40Hz until no touch)
  if (setup_iqs263() == HAL_ERROR) {
	  strncpy(errorCondition, "ERR:IQS263ST", sizeof(errorCondition));
	  GlobalState.programMode = MODE_ERROR;
  }

    /* Infinite loop */
    for(;;)
    {

	 current_minute = iqs263_get_min_if_pressed(); //returns -1 if no press
     if (current_minute != -1) { //touch!

       if (!touch_end_count){ //START TOUCH EVENT!

    	   touch_end_count = 1;

    	   er_oled_clear(oled_buf);

    	   switch (GlobalState.programMode) {
    	   	   case MODE_ESM_TIME_ESTIMATE:
    	   	       xTaskNotifyGive(esmMainHandle);
           		   er_oled_string(0, 0, "GUESS TIME:", 12, 1, oled_buf);
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
    	   		   er_oled_string(0, 0, "GUESS TIME:", 12, 1, oled_buf);
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
       }


  	   if (last_minute != current_minute) { //UPDATE TOUCH VALUE!
  		   //update touch stuff!
           //
           //map 10-50 to options for ESM_SURVEY
           //
           //for time guessing:
           //
           //if we have current_min 10>cm>=0 and last>50 add hr
           //if we have current_min <=59 and 10>last>=0 subtract hr

           switch (GlobalState.programMode){
                case MODE_ESM_SURVEY:
                    //clear bottom
                    er_oled_clear_bottom_third(oled_buf);

                    //divide 10-50 min into option steps roughly
                    uint8_t step = 50 / (GlobalState.surveyState.optionArrayLength+1);
                    for (int i=0; i<GlobalState.surveyState.optionArrayLength; i++){
                        //map minute to option
                        if(current_minute > (11 + i*step) &&
                           current_minute < (11 + (i+1)*step)){
                            er_oled_string(0, 28, GlobalState.surveyState.optionArray[GlobalState.surveyState.optionArrayLength-1-i], 12, 1, oled_buf);
                        }
                    }
                    er_oled_display(oled_buf);

                    break;
                case MODE_TIME_ESTIMATE:
                case MODE_ESM_TIME_ESTIMATE:

                    //hours wrap
                    if (current_minute < 15 &&
                        current_minute >= 0 &&
                        last_minute > 45){
                        hrs = (hrs+1)%24;

                    } else if (current_minute > 45 &&
                               last_minute < 15){
                        if (hrs==0) {hrs = 23};
                        else {hrs -= 1;}
                    }

                    //display time on bottom two thirds
                    sprintf (time, "%02d%02d", hrs, current_minute);
                    er_oled_time_twothird(time, oled_buf);

                    break;
           }

  		   last_minute = current_minute;

  	   }


  	   //optional
  	   osDelay(25);


     } else if (touch_end_count > 0){

  	   touch_end_count += 1;//increment touching_end_count

  	   if (touch_end_count >= TOUCH_END_TIMEOUT){  //FINISHED/CONFIRMED TOUCH VALUE!

  		   touch_end_count = 0;
  		   if (GlobalState.programMode == MODE_ESM_TIME_ESTIMATE ||
  			   GlobalState.programMode == MODE_ESM_SURVEY){
  			   	   xTaskNotifyGive(esmMainHandle);
  		   }
  		   //DO THINGS WITH CONFIRMED TOUCH == LAST_MINUTE
  		   char out_text[10];
  		   sprintf(out_text, "FINAL: %d", last_minute);
  		   er_oled_clear(oled_buf);
  		   er_oled_string(0, 28, out_text, 12, 1, oled_buf);
  		   er_oled_display(oled_buf);
  		   if (GlobalState.programMode == MODE_TIME_ESTIMATE){
  			   osMutexAcquire(modeMutexHandle, portMAX_DELAY);
  			   GlobalState.programMode = MODE_RESTING;
  			   osMutexRelease(modeMutexHandle);
  		   }
  		   //uint16_t touchval = 0x5000 | last_minute;
  		   //osMessageQueuePut(bleTXqueueHandle, &touchval, 0, 0);

  		   last_minute = -1;

  	   }

  	   osDelay(25);


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

	       osMutexAcquire(modeMutexHandle, portMAX_DELAY);
	       GlobalState.programMode = MODE_RESTING;
	       osMutexRelease(modeMutexHandle);

	       //TODO: PICK NEW INTERVAL
	       /////////////////////////

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

		   osMutexAcquire(modeMutexHandle, portMAX_DELAY);
		   GlobalState.programMode = MODE_RESTING;
		   osMutexRelease(modeMutexHandle);

		   //TODO: PICK NEW INTERVAL
		   /////////////////////////

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
    	  er_oled_string(0, 0, "GUESS TIME:", 12, 1, oled_buf);
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
		   break;

       default:
    	   osDelay(20);
    	   break;
       }

     }
    }



  /* USER CODE END startUIControl */
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

  /* Infinite loop */
  for(;;)
  {
    //check state, get mode, call timer if necessary

    //xTaskNotifyGive(startAlertHandle); to alert user with flash and vibration
    osDelay(10000);

    //at random interval, bounded by timebounds ONLY WHEN IN MODE_RESTING
    //(MODE_CANCEL/MODE_TIME_ESTIMATE can happen when user is looking at time;
    //we want to wait for these to complete and use the updated time for estimating
    //intervals

    //if >timebound interval selection + lasttimeseen
	//if between hours of timebound
    //if mode is mode_resting
    //
    if (GlobalState.programMode == MODE_RESTING){

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
    		xTaskNotifyGive(alertHandle);
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
        }

    	//SECOND SCREEN FOR ESM
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
    			strncpy(GlobalState.surveyState.screenText, "  FOCUS?", strlen("  FOCUS?") + 1);
    			GlobalState.surveyState.screenTextLength = strlen("  FOCUS?");
    			GlobalState.surveyState.surveyID = SURVEY_FOCUS;
    			//GlobalState.surveyState.optionArray = opts_five;
    			memcpy(GlobalState.surveyState.optionArray, opts_five, sizeof(opts_five));
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

    	//THIRD SCREEN FOR ESM
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
				strncpy(GlobalState.surveyState.screenText, "  TIME CUE", strlen("  TIME CUE") + 1);
				GlobalState.surveyState.screenTextLength = strlen("  TIME CUE");
				GlobalState.surveyState.surveyID = SURVEY_TIMECUE;
				//&(GlobalState.surveyState.optionArray) = &opts_agree;
				memcpy(GlobalState.surveyState.optionArray, opts_agree, sizeof(opts_agree));
				GlobalState.surveyState.optionArrayLength = 2;
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

				//pick a new interval
				//TODO: implement me!!!!!!!!!!!!!!!!!!!!!


			} else {//timed out due to inactivity
				continue_flag = 0;

				osMutexAcquire(modeMutexHandle, portMAX_DELAY);
				GlobalState.programMode = MODE_CLEAR;
				osMutexRelease(modeMutexHandle);
			}
		}

    }


    //osMutexAcquire(ledStateMutexHandle, portMAX_DELAY);
	//osMutexRelease(ledStateMutexHandle);

    //floats are 32-bits, so 4 bytes. 16 bytes for total
   	//osMessageQueuePut(bleRXqueueHandle, &(pNotification->DataTransfered), 0, 0);
    //const BLETX_Queue_t bleSendData = {TX_TEMP_HUMD, 0x0000};
    //osMessageQueuePut(bleTXqueueHandle, &bleSendData, 0, 0);

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

  const BLETX_Queue_t bleSendData = {TX_LAST_SURVEY_INVALID, 0x0000};

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
			  osMessageQueuePut(bleTXqueueHandle, &bleSendData, 0, 0);

			  osMutexAcquire(modeMutexHandle, portMAX_DELAY);
			  GlobalState.programMode = MODE_CANCEL;
			  osMutexRelease(modeMutexHandle);

			  //TODO: pick new interval
			  /////////////////////////
		  }

		}
		if (callingPin == 0b10000 && first_read != buttonState[1]) { //button 2 trigger
		    //set buttonState
		    buttonState[1] = first_read;

		    //do stuff if button pressed
		    if (!first_read){
		    	osMessageQueuePut(bleTXqueueHandle, &bleSendData, 0, 0);

		    	osMutexAcquire(modeMutexHandle, portMAX_DELAY);
		    	GlobalState.programMode = MODE_CANCEL;
		    	osMutexRelease(modeMutexHandle);

		    	//TODO: pick new interval
				/////////////////////////
		    }
		}
		if (callingPin == 0b100000 && first_read != buttonState[2]) { //button 3 trigger
		    //set buttonState
		    buttonState[2] = first_read;

		    //do stuff if button pressed
		    if (!first_read){
		    	osMessageQueuePut(bleTXqueueHandle, &bleSendData, 0, 0);

		    	osMutexAcquire(modeMutexHandle, portMAX_DELAY);
		    	GlobalState.programMode = MODE_CANCEL;
		        osMutexRelease(modeMutexHandle);

		        //TODO: pick new interval
		        /////////////////////////
		    }
		}

	}

  }
  /* USER CODE END startButtonPress */
}

/* USER CODE BEGIN Header_startAlert */
/**
* @brief Function implementing the alert thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startAlert */
void startAlert(void *argument)
{
  /* USER CODE BEGIN startAlert */

  //xTaskNotifyGive(alertHandle); to alert user with flash and vibration

  //HAL_GPIO_WritePin(VIBRATION_GPIO_Port, VIBRATION_Pin, GPIO_PIN_RESET);

  //Init Vibration Motor PWM Parameters
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

  const uint8_t MAX_BRIGHTNESS = 0x33; //max brightness, 0x01-0xFF

  ds_setBrightness(0);
  osDelay(1000);

  uint16_t counter;
  uint8_t LEDDirection, LEDBrightness;

  /* Infinite loop */
  for(;;)
  {

	counter = 0;
	LEDDirection = 0;
	LEDBrightness = 0;

	ulTaskNotifyTake( pdTRUE, portMAX_DELAY);

	//start vibration
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	//flash loop
	ds_fill(0xFFFFFF, 0, 12);

	while (counter++ < MAX_BRIGHTNESS*2) {

		ds_setBrightness(LEDBrightness);
		ds_show();

		//increment color intensity
		if (LEDDirection) {LEDBrightness--;}
		else {LEDBrightness++;}

		//if we hit a limit switch color scaling up or down
		if (LEDBrightness == MAX_BRIGHTNESS) {LEDDirection = 1;}
		if (LEDBrightness == 0x00) {LEDDirection = 0;}

		osDelay(pdMS_TO_TICKS(2)); //2ms delay
	}

	//turn off LEDs
	ds_setBrightness(0);
	ds_fill(0x000000, 0, 12);
	ds_show();

	osDelay(pdMS_TO_TICKS(100)); //100ms delay

    //stop vibration
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);


  }
  /* USER CODE END startAlert */
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
	osDelay(500); //let screen start first

	//poll ambient temp, humidity, visible light, white light
	//every 5sec
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

    const BLETX_Queue_t bleSendData = {TX_TEMP_HUMD, 0x0000};

  	for (;;){

  		lux = veml_Get_Lux();
    	whiteLux = veml_Get_White_Lux();
    	temperature = si7021_measure_temperature(&hi2c1);
    	humidity = si7021_measure_humidity(&hi2c1);

    	osMutexAcquire(conditionMutexHandle, portMAX_DELAY);
    	GlobalState.lastConditions.lux = lux;
    	GlobalState.lastConditions.whiteLux = whiteLux;
    	GlobalState.lastConditions.temp = temperature;
    	GlobalState.lastConditions.humd = humidity;
    	osMutexRelease(conditionMutexHandle);

		osMessageQueuePut(bleTXqueueHandle, &bleSendData, 0, 0);

    	osDelay(9900);
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
  BLETX_Queue_t sendData;

  /* Infinite loop */
  for(;;)
  {
    if (osMessageQueueGet(bleTXqueueHandle, &sendData, NULL, osWaitForever) == osOK){

    	//TX_TIME_SEEN - data in GlobalState.lastSeenTime

    	//TX_TIME_EST - data in GlobalState.timeEstimateSample

    	//TX_TEMP_HUMD - construct both TX_TEMP_HUMD/TX_LUX_WHITELUX
    	//               with data in GlobalState.lastConditions

    	//TX_LAST_SURVEY_INVALID -- just send that timestamped

    	//TX_SURVEY_INITIALIZED -- just send that timestamped

    	//TX_TIMESTAMP_UPDATE -- use data in sendData for error, to send
    	//TX_SURVEY_RESULT -- use data in sendData, first byte is survey/second is answer

    	P2PS_Send_Data(sendData.data);
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

    		osMutexAcquire(rtcMutexHandle, portMAX_DELAY);
    		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {Error_Handler();}
    		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK) {Error_Handler();}
    	    osMutexRelease(rtcMutexHandle);

		}


/*		else if (rxData.pPayload[0] == 0x06) {//screen text update starts with 0x06

			//works great if string sent is <=18 and has a '00' byte (19 + update byte, 20 byte MTU)
			//'00' obviously connotes the end of the string.
			//should edit this to have a char buffer in place (128 byte) that copies over
			// rxData.pPayload[1] for rxData.Length - 1, check last byte.  if 00 stop,
			//otherwise wait and keep filling buffer with next packet.
			//write a complementary send function that adds a hex '00' and chops the full string
			//into 19 byte chunks when sending with 0x01 header from phone.
			strncpy(&(textbuffer[continuing*19]), &(rxData.pPayload[1]), rxData.Length - 1);
			continuing += 1;

			if (rxData.pPayload[rxData.Length-1] == 0x00) { //completed string

				osMutexAcquire(screenTextMutexHandle, portMAX_DELAY);
				strncpy(ScreenState.screenText, textbuffer, 128);
				osMutexRelease(screenTextMutexHandle);
				xTaskNotify(screenUpdateHandle, (uint32_t)SCREEN_TEXT, eSetValueWithOverwrite);

				continuing = 0;

			}

			//osMutexAcquire(screenTextMutexHandle, portMAX_DELAY);
			//strncpy(ScreenState.screenText, &(rxData.pPayload[1]), rxData.Length - 1);
			//osMutexRelease(screenTextMutexHandle);
			//xTaskNotify(screenUpdateHandle, (uint32_t)SCREEN_TEXT, eSetValueWithOverwrite);

		} else if (rxData.pPayload[0] == 0x03) { //vibrate payload with duration to vibrate starts with 0x03

			//uint8_t duration[4] = {0};
	    	//memcpy(&(duration[4-(rxData.Length-1)]), &(rxData.pPayload[1]), rxData.Length-1);
	    	//uint32_t send_duration = duration[0] << 24 | duration[1] << 16 | duration[2] << 8 | duration[3];

	    	uint32_t send_duration = 0;
	    	for (uint8_t i=0; i<rxData.Length-1; i++){
	    		send_duration |= rxData.pPayload[1+i] << ((rxData.Length-2-i)*8);
	    	}

	    	xTaskNotify(vibrateControlHandle, send_duration, eSetValueWithOverwrite);
	    } else if (rxData.pPayload[0] == 0x07) { //led state update
	    	osMutexAcquire(ledStateMutexHandle, portMAX_DELAY);
	    	LedState.currentMode = rxData.pPayload[1];
	    	osMutexRelease(ledStateMutexHandle);
	    }*/
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
