/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"
#include "cmsis_os.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler(void);

#define OLED_RESET_Pin GPIO_PIN_0
#define OLED_RESET_GPIO_Port GPIOB
#define BUTTON_1_Pin GPIO_PIN_3
#define BUTTON_1_GPIO_Port GPIOB
#define BUTTON_1_EXTI_IRQn EXTI3_IRQn
#define BUTTON_2_Pin GPIO_PIN_4
#define BUTTON_2_GPIO_Port GPIOB
#define BUTTON_2_EXTI_IRQn EXTI4_IRQn
#define BUTTON_3_Pin GPIO_PIN_5
#define BUTTON_3_GPIO_Port GPIOB
#define BUTTON_3_EXTI_IRQn EXTI9_5_IRQn

#define SURVEY_NONE     0x0
#define SURVEY_FOCUS    0x1 //focus level during last interval
#define SURVEY_AROUSAL  0x2 //altertness level during last interval
#define SURVEY_VALENCE  0x3 //valence level during last interval
#define SURVEY_COGLOAD  0x4 //cognitive load during last interval
#define SURVEY_TIMECUE  0x5 //event with known clock reference, notification about time, in last interval?
#define SURVEY_CAFFEINE 0x6 //drink caffeine in last interval?
#define SURVEY_EXERCISE 0x7 //exercise in last interval? heavy light no
#define SURVEY_STRESS   0x8 //stress during last interval
#define SURVEY_LOCATE   0x9 //location
#define SURVEY_TSENSE   0xA //thermal sensation
#define SURVEY_TCOMFORT 0xB //thermal comfort

#define INTERVAL_MIN 0 //min interval in min
#define INTERVAL_MAX 1 //max interval in min

//settings for when to randomly ESM
typedef struct {
  uint8_t startHR_BCD; // hour in day to start allowing notifications, BCD format
  uint8_t endHR_BCD;   // hour in day to stop allowing notifications, BCD format
  uint8_t minInterval; //minimum number of mins to pass before new ESM notification
  uint8_t maxInterval; //maximum number of mins to pass before new ESM notification
} ESMTimeBounds_t;

//wrapper for timestamp data
typedef struct {
  RTC_TimeTypeDef time;
  RTC_DateTypeDef date;
} TimeStruct_t;

//wrapper for condition data
typedef struct {
	float lux;
	float whiteLux;
	float temp;
	float humd;
} ConditionSample_t;

//state indicator for program
typedef enum {
	MODE_RESTING, //nothing is going on
	MODE_TIME_ESTIMATE, //user interaction driven event, wants to check time
	MODE_ESM_TIME_ESTIMATE, //random interruption, ask user for time estimate and notify ESM thread at complete
	MODE_ESM_SURVEY, //random interruption, ask user for survey response and notify ESM thread at complete
	MODE_CANCEL, //button press that indicates we need to display current time
	MODE_SHOW_TIME, //show time (like mode cancel) but without 'dismissed' warning
	MODE_ERROR, //something went wrong, display error
	MODE_CLEAR  //timeout, clear screen and go back to rest
} ProgramMode_t;

//screen state information
typedef struct {
	uint8_t surveyID; //survey ID using #defines above
	char screenText[128]; //topline text for survey
	uint8_t screenTextLength; //length of text
	char *optionArray[7];  //array of char arrays to represent touch options, i.e. 'agree'/'disagree' or '1''2''3'
	uint8_t optionArrayLength; //num of options
} Survey_t;

static const char* const opts_five[][8] = {"      1", "      2", "      3", "      4", "      5"};
static const char* const opts_agree[][11] = {"  disagree", "    agree"};
static const char* const opts_valence[][11] = {"vry negative", "  negative", "  neutral", "  positive", "vry positive"};
static const char* const opts_arousal[][11] = {"  very low", "    low", "  average", "    high", " very high"};
static const char* const opts_yes[][11] = {"     no", "    yes"};
static const char* const opts_location[][11] = {"   indoor", "  outdoor"};
static const char* const opts_thermalsense[][11] = {"    cold", "    cool","slghtly cool","   neutral","slghtly warm","    warm","     hot",};
static const char* const opts_thermalcomfort[][11] = {"   cooler", " no change", "   warmer"};

typedef struct {
	ESMTimeBounds_t timeBound; //protected by timeBoundMutex
	TimeStruct_t lastSeenTime;  //protected by lastSeenMutex
	//TimeStruct_t timeEstimateSample; //protected by timeEstimateMutex
	ConditionSample_t lastConditions; //protected by conditionMutex
	ProgramMode_t programMode; //protected by modeMutex
	Survey_t surveyState; //protected by surveyMutex
	uint8_t currentInterval; //current interval to wait for
	uint8_t paused; //can stop all ESM from phone; turned off by default
	uint8_t demo; //whether we're in demo mode, flash the LEDs
} GlobalState_t;

char errorCondition[13];

GlobalState_t GlobalState;

typedef enum {
	TX_LUX_WHITELUX,        //0 x
	TX_TEMP_HUMD,           //1 x
	TX_SURVEY_INITIALIZED,  //2
	TX_TIME_EST,            //3
	TX_TIME_SEEN,           //4
	TX_SURVEY_RESULT,       //5
	TX_PREVIOUS_INVALID,    //6
	TX_TIMESTAMP_UPDATE,    //7
	TX_BEGIN_PAUSE			//8
} SendDataType_t;

//data pass to BLETX thread
typedef struct {
  SendDataType_t sendType;
  uint16_t data;
} BLETX_Queue_t;

//add data to queue if we cant send over BLE,
//dynamic allocation to grow until we cannot malloc anymore
typedef struct {
	uint16_t *packet;
	uint8_t numBytes;
	struct UnsentQueue_t *next;
} UnsentQueue_t;

typedef UnsentQueue_t *UnsentQueueAddress_t;

#define TOUCH_HISTORY_SIZE 10 //size of touch history used for smoothing
#define TOUCH_END_TIMEOUT 40 //loops of 5ms before timeout
#define ALERT_TIMEOUT 6000 //timeout in in MS before rebuzzing ESM if ignored
#define INTERACTION_TIMEOUT 45000 //ms timeout in MS for each question before assume abandoned
//make alert timeout variable so it doesn't help with time estimation


//DATA TO SEND:
// timestamp is 4x16b, so 8 bytes.  This leaves us with 12 to work with per packet.
// the next byte will be used for a SendDataType_t identifier.  This leaves us with 11 for data.

// timestamped lux/whitelux.  these are each floats(32b), so 4 bytes each, so 8 bytes.
// timestamped temp/humd.  these are each floats(32b), so 4 bytes each, so 8 bytes.
// timestamped time_estimates, also as a timestamp. 8 bytes.
// timestamped seen time.  Correct answer for time_estimate, or on cancel/restart.
// timestamped survey_result.  1 byte for survey type, 1 byte for user input.
// timestamped survey initialized. (no data)
// timestamped last_data_invalid. (no data)
// timestamped timestamp update (send previous time before update)

RTC_HandleTypeDef hrtc; //protected by rtcMutexHandle
osMutexId_t rtcMutexHandle;

osMessageQueueId_t bleRXqueueHandle;



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

//we have a UI thread that initializes screen; need to integrate touch
//we have ESMMain, which should manage ESM randomly within timebounds
//we have alert thread working (buzz and flash)
//we have buttonpress working and updating to BLETX
//we have condition thread also updating to BLETX every 10s with new data in global
//struct

//need to rewrite BLETX to take in new data and send
//need to rewrite to catch failures/check connection status and queue dynamically

//need to rewrite UI thread to integrate touch, based off of mode and notification
//need to write ESM to call/notify

//need to rewrite RX to update time bounds, auto-update time and calc error



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
