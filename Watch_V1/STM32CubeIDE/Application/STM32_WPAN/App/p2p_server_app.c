/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    p2p_server_app.c
 * @author  MCD Application Team
 * @brief   peer to peer Server Application
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
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "p2p_server_app.h"
#include "cmsis_os.h"
#include "p2p_stm.h"

/* Private function prototypes -----------------------------------------------*/
void P2PS_APP_Context_Init(void);

/* Functions Definition ------------------------------------------------------*/
void P2PS_STM_App_Notification(P2PS_STM_App_Notification_evt_t *pNotification)
{
  switch(pNotification->P2P_Evt_Opcode)
  {

#if(BLE_CFG_OTA_REBOOT_CHAR != 0)
    case P2PS_STM_BOOT_REQUEST_EVT:
      APP_DBG_MSG("-- P2P APPLICATION SERVER : BOOT REQUESTED\n");
      APP_DBG_MSG(" \n\r");

      *(uint32_t*)SRAM1_BASE = *(uint32_t*)pNotification->DataTransfered.pPayload;
      NVIC_SystemReset();
      break;
#endif

    case P2PS_STM__NOTIFY_ENABLED_EVT:
      P2P_Server_App_Context.Notification_Status = 1;
      APP_DBG_MSG("-- P2P APPLICATION SERVER : NOTIFICATION ENABLED\n");
      APP_DBG_MSG(" \n\r");
      break;

    case P2PS_STM_NOTIFY_DISABLED_EVT:
      P2P_Server_App_Context.Notification_Status = 0;
      APP_DBG_MSG("-- P2P APPLICATION SERVER : NOTIFICATION DISABLED\n");
      APP_DBG_MSG(" \n\r");
      break;

    case P2PS_STM_WRITE_EVT:
		osMessageQueuePut(bleRXqueueHandle, &(pNotification->DataTransfered), 0, 0);
      break;

    default:
      break;
  }
  return;
}

void P2PS_APP_Notification(P2PS_APP_ConnHandle_Not_evt_t *pNotification)
{
  switch(pNotification->P2P_Evt_Opcode)
  {
  case PEER_CONN_HANDLE_EVT :
    break;

    case PEER_DISCON_HANDLE_EVT :
       P2PS_APP_Context_Init();
       break;
    default:
      break;
  }
  return;
}

void P2PS_APP_Init(void)
{
  P2P_Server_App_Context.Notification_Status=0;
  P2PS_APP_Context_Init();
  return;
}

void  P2PS_APP_Context_Init(void)
{
	  //init context on app init and on reconnect events
	  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x01; /* Device1 */
	  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
	  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x01;/* Device1 */
	  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
	  P2P_Server_App_Context.OTATimestamp=0x0000000000000000;
	  P2P_Server_App_Context.OTA12HrFormat=0x00;
	  P2P_Server_App_Context.OTADaylightSavings=0x00;
}

void P2PS_Send_Timestamp(void)
{

   if(P2P_Server_App_Context.Notification_Status){

	APP_DBG_MSG("-- P2P APPLICATION SERVER  : SEND LOCAL TIMESTAMP \n ");
    APP_DBG_MSG(" \n\r");

    RTC_TimeTypeDef cTime;
	RTC_DateTypeDef cDate;

	osMutexAcquire(rtcMutexHandle, portMAX_DELAY);
	HAL_RTC_GetTime(&hrtc, &cTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &cDate, RTC_FORMAT_BCD);
	osMutexRelease(rtcMutexHandle);

	uint64_t sendval = (cDate.WeekDay << (8*3)) | (cDate.Month << (8*2)) | (cDate.Date << (8*1)) | cDate.Year;
	sendval <<= 32;
	sendval |= (cTime.Hours << (8*3)) | (cTime.Minutes << (8*2)) | (cTime.Seconds << (8*1)) | (cTime.TimeFormat);

	P2PS_STM_App_Update_Int8(P2P_NOTIFY_CHAR_UUID, (uint8_t *)&sendval, 8);

   } else {
    APP_DBG_MSG("-- P2P APPLICATION SERVER : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n ");
   }

  return;
}

void P2PS_Send_Data(uint16_t data)
{

   if(P2P_Server_App_Context.Notification_Status){
    APP_DBG_MSG("-- P2P APPLICATION SERVER  : SEND TIMESTAMPED DATA \n ");
    APP_DBG_MSG(" \n\r");

    RTC_TimeTypeDef cTime;
	RTC_DateTypeDef cDate;

	osMutexAcquire(rtcMutexHandle, portMAX_DELAY);
	HAL_RTC_GetTime(&hrtc, &cTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &cDate, RTC_FORMAT_BCD);
	osMutexRelease(rtcMutexHandle);

	uint16_t sendval[5] = {0};

	sendval[4] = (cDate.WeekDay << (8*1)) | cDate.Month;
	sendval[3] = (cDate.Date << (8*1)) | cDate.Year;

	sendval[2] = (cTime.Hours << (8*1)) | cTime.Minutes;
	sendval[1] = (cTime.Seconds << (8*1)) | cTime.TimeFormat;

	sendval[0] = data;

	P2PS_STM_App_Update_Int8(P2P_NOTIFY_CHAR_UUID, (uint8_t *)&sendval, 10);


/*	//if sending text, send text
	if ((data & 0xFF00) == 0x6300){
		P2PS_STM_App_Update_Int8(P2P_NOTIFY_CHAR_UUID, (uint8_t *)&ScreenState.screenText, sizeof(ScreenState.screenText));
		for (int i=0; i<8; i++){
			//send full 128 byte char array; 8 chunks of 16 bytes
			P2PS_STM_App_Update_Int8(P2P_NOTIFY_CHAR_UUID, (uint8_t *)&ScreenState.screenText[i*8], 16);
		}

	}*/


   } else {
    APP_DBG_MSG("-- P2P APPLICATION SERVER : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n ");
   }

  return;
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
