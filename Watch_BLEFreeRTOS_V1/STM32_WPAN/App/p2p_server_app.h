/* USER CODE BEGIN */
/**
 ******************************************************************************
 * File Name          : App/p2p_server_app.h
 * Description        : Header for p2p_server_app.c module
 ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#ifndef __P2P_SERVER_APP_H
#define __P2P_SERVER_APP_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  PEER_CONN_HANDLE_EVT,
  PEER_DISCON_HANDLE_EVT,
} P2PS_APP__Opcode_Notification_evt_t;


typedef struct{
   uint8_t             Device_Led_Selection;
   uint8_t             Led1;
}P2P_LedCharValue_t;

typedef struct{
   uint8_t             Device_Button_Selection;
   uint8_t             ButtonStatus;
}P2P_ButtonCharValue_t;

typedef struct
{
  P2PS_APP__Opcode_Notification_evt_t   P2P_Evt_Opcode;
  uint16_t                              ConnectionHandle;
}P2PS_APP_ConnHandle_Not_evt_t;

typedef struct
 {
   uint8_t               Notification_Status; /* used to chek if P2P Server is enabled to Notify */
   P2P_LedCharValue_t    LedControl;
   P2P_ButtonCharValue_t ButtonControl;
   uint16_t              ConnectionHandle;
   uint64_t				 OTATimestamp;
   uint8_t				 OTA12HrFormat;
   uint8_t               OTADaylightSavings;
 } P2P_Server_App_Context_t;

PLACE_IN_SECTION("BLE_APP_CONTEXT") static P2P_Server_App_Context_t P2P_Server_App_Context;

/* Exported functions ---------------------------------------------*/
void P2PS_APP_Init( void );
void P2PS_APP_Notification( P2PS_APP_ConnHandle_Not_evt_t *pNotification );
void P2PS_Send_Timestamp(void);
void P2PS_Send_Data(uint16_t data);

#ifdef __cplusplus
}
#endif

#endif /*__P2P_SERVER_APP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
