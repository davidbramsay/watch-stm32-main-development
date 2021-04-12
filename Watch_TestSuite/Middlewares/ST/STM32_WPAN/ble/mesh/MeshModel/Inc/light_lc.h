/**
******************************************************************************
* @file    light_control.h
* @author  BLE Mesh Team
* @brief   Header file for the user application file 
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIGHT_LC_H
#define __LIGHT_LC_H

/* Includes ------------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables  -------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define LC_PROPERTY_TABLE_COUNT                  18
#define LIGHT_LC_DEFAULT_TRANSITION_RES_MS       100 /* Recommended value is 100 milliseconds */

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  MOBLEUINT8 LC_modeState;
}Light_LC_ModeParam_t; 

typedef struct
{
  MOBLEUINT8 LC_OMState;
}Light_LC_OccupancyModeParam_t; 

typedef struct
{
  MOBLEUINT8 LC_OnOffState;
  MOBLEUINT8 Tid;
  MOBLEUINT8 Transition_Time;
  MOBLEUINT8 Delay_Time;
}Light_LC_OnOffParam_t; 

/**
  * LC callback structure
 */
typedef struct
{
  void (*LightLCs_ModeGet_cb)(MODEL_MessageHeader_t *pmsgParam);
  void (*LightLCs_ModeSet_cb)(MOBLEUINT8 const* pRxData,
                              MODEL_MessageHeader_t *pmsgParam);
  void (*LightLCs_ModeSetUnack_cb)(MOBLEUINT8 const* pRxData,
                                   MODEL_MessageHeader_t *pmsgParam);
  void (*LightLCs_ModeStatus_cb)(MOBLEUINT8 const*, 
                                 MOBLEUINT32,
                                 uint16_t, 
                                 uint8_t);
  void (*LightLCs_OmGet_cb)(MODEL_MessageHeader_t *pmsgParam);
  void (*LightLCs_OmSet_cb)(MOBLEUINT8 const* pRxData,
                            MODEL_MessageHeader_t *pmsgParam);
  void (*LightLCs_OmSetUnack_cb)(MOBLEUINT8 const* pRxData,
                                      MODEL_MessageHeader_t *pmsgParam);
  void (*LightLCs_OmStatus_cb)(MOBLEUINT8 const*, 
                               MOBLEUINT32,
                               uint16_t, 
                               uint8_t);
  void (*LightLCs_OnOffGet_cb)(MODEL_MessageHeader_t *pmsgParam);
  void (*LightLC_OnOffSet_cb)(MOBLEUINT8 const* pRxData,
                                      MODEL_MessageHeader_t *pmsgParam);
  void (*LightLCs_OnOffSetUnack_cb)(MOBLEUINT8 const* pRxData,
                                    MODEL_MessageHeader_t *pmsgParam);
  void (*LightLCs_OnOffStatus_cb)(MOBLEUINT8 const *, 
                                  MOBLEUINT32,
                                  uint16_t, 
                                  uint8_t);
  void (*LightLCs_PropertyGet_cb)(MOBLEUINT8 const* pRxData,
                                    MODEL_MessageHeader_t *pmsgParam);
  void (*LightLCs_PropertySet_cb)(MOBLEUINT8 const* pRxData,
                                       MODEL_MessageHeader_t *pmsgParam);
  void (*LightLCs_PropertySetUnack_cb)(MOBLEUINT8 const* pRxData,
                                      MODEL_MessageHeader_t *pmsgParam);
  void (*LightLCs_PropertyStatus_cb)(MOBLEUINT8 const*, 
                                     MOBLEUINT32,
                                     uint16_t, 
                                     uint8_t);
} light_lc_cb_t;

extern const light_lc_cb_t AppliLightLc_cb;


/* Exported functions ------------------------------------------------------- */
MOBLE_RESULT LightLcServer_GetOpcodeTableCb(const MODEL_OpcodeTableParam_t **data, 
                                            MOBLEUINT16 *length);
MOBLE_RESULT LightLcServer_GetStatusRequestCb(MODEL_MessageHeader_t *pmsgParam, 
                                              MOBLEUINT16 opcode, 
                                              MOBLEUINT8 *pResponsedata, 
                                              MOBLEUINT32 *plength, 
                                              MOBLEUINT8 const *pRxData,
                                              MOBLEUINT32 dataLength,
                                              MOBLEBOOL response);
MOBLE_RESULT LightLcServer_ProcessMessageCb(MODEL_MessageHeader_t *pmsgParam, 
                                            MOBLEUINT16 opcode, 
                                            MOBLEUINT8 const *pRxData, 
                                            MOBLEUINT32 dataLength, 
                                            MOBLEBOOL response);
void Light_LC_NvmParams_Get(MOBLEUINT8 elementIndex, 
                            MOBLEUINT8* lightLcNvmParamsBuff, 
                            MOBLEUINT8* lcNvmParamsSize);
void Light_LC_OnPowerUp(MOBLEUINT8 elementIndex,
                        MOBLEUINT8 const *pModelValue_Load,
                        MOBLEUINT8 genericOnPowerUp,
                        MOBLEUINT16 lightDefault,
                        MOBLEUINT16 lightLast,
                        MOBLEUINT16 lightActualLKV);
MOBLE_RESULT Light_LC_SetTransitionTimeZero(MOBLEUINT8 elementIndex);
MOBLE_RESULT Light_LC_SensorPropertyUpdate(MOBLEUINT8 lcsElementIndex,
                                           MOBLEUINT16 sensorPropertyId,
                                           MOBLEUINT32 value);
void Light_LC_LigtnessLinearUnsolicitedChange(MOBLEUINT8 elementIndex);
MOBLEUINT32 Light_LC_SleepDurationMs_Get(void);
MOBLE_RESULT Light_LC_Send(MOBLEUINT8 elementIndex);
MOBLE_RESULT Light_LC_LcOnOffUpdate(MOBLEUINT8 elementIndex, 
                                    MOBLEUINT8 onOff,
                                    MOBLEUINT8 optionalParams,
                                    MOBLEUINT32 delayMsOnOffMsg,
                                    MOBLEUINT32 stepResMsOnOffMsg,
                                    MOBLEUINT32 trTimeMsOnOffMsg);
void Light_LC_Process(void);
MOBLE_RESULT Light_LCs_Init(void* lcsBuff, 
                            MOBLEUINT8 lcsElementIndex, 
                            const light_lc_cb_t* lcs_cb, 
                            MOBLEUINT16 sizeBuff);
MOBLE_RESULT ExtractLcServerElementIndex(MOBLEUINT8* pLcsElementIndex,
                                         MOBLEUINT8 noOfElements,
                                         MOBLEUINT8 lcServer,
                                         MOBLEUINT8 lcSetupServer,
                                         MOBLEUINT8 genericOnOffServer,
                                         MOBLEUINT8 genericPowerOnOffServer,
                                         MOBLEUINT8 lightLightnessServer);

#ifdef ENABLE_LIGHT_MODEL_CLIENT_LC  
MOBLE_RESULT LightLC_Client_Mode_Status(MOBLEUINT8 const *pLCMode_status, 
                                        MOBLEUINT32 plength, 
                                        MOBLEUINT16 dstPeer,
                                        MOBLEUINT8 elementIndex);
MOBLE_RESULT LightLC_Client_OM_Status(MOBLEUINT8 const *pLCOccupancyMode_status,
                                      MOBLEUINT32 plength, 
                                      MOBLEUINT16 dstPeer, 
                                      MOBLEUINT8 elementIndex);
MOBLE_RESULT LightLC_Client_OnOff_Status(MOBLEUINT8 const *pLCOnOff_status, 
                                         MOBLEUINT32 plength, 
                                         MOBLEUINT16 dstPeer, 
                                         MOBLEUINT8 elementIndex);
MOBLE_RESULT LightLC_Client_Property_Status(MOBLEUINT8 const *pLCProperty_status,
                                            MOBLEUINT32 plength, 
                                            MOBLEUINT16 dstPeer, 
                                            MOBLEUINT8 elementIndex);
#endif /* #ifdef ENABLE_LIGHT_MODEL_CLIENT_LC */

#endif /* __LIGHT_LC_H */

/******************* (C) COPYRIGHT 2020 STMicroelectronics *****END OF FILE****/

