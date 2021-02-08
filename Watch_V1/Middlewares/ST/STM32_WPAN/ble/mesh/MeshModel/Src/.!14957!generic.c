/**
******************************************************************************
* @file    generic.c
* @author  BLE Mesh Team
* @brief   Generic model middleware file
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
/* Includes ------------------------------------------------------------------*/
#include "hal_common.h"
#include "ble_mesh.h"
#include "mesh_cfg.h"
#include "generic.h"
#include "light.h"
#include "light_lc.h"
#include "common.h"
#include "models_if.h"
#include <string.h>
#include "compiler.h"
#include <stdint.h>
#include "math.h"


/** @addtogroup MODEL_GENERIC
*  @{
*/

/** @addtogroup Generic_Model_Callbacks
*  @{
*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static Generic_TemporaryStatus_t Generic_TemporaryStatus[APPLICATION_NUMBER_OF_ELEMENTS];
static Generic_TimeParam_t Generic_TimeParam[APPLICATION_NUMBER_OF_ELEMENTS];

/* initialize the array with minimum level value and other parameter as zero */
static Generic_LevelStatus_t Generic_LevelStatus[APPLICATION_NUMBER_OF_ELEMENTS] = 
{
  {
    LEVEL_MIN_VALID_RANGE,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00
  }
};

static Generic_OnOffStatus_t Generic_OnOffStatus[APPLICATION_NUMBER_OF_ELEMENTS];
 

#ifdef ENABLE_GENERIC_MODEL_SERVER_DEFAULT_TRANSITION_TIME
/* Initialize the default transition value 0x00 
   Default Transition Step Resolution is 100 milliseconds
   Generic Default Transition Time is immediate.
*/
Generic_DefaultTransitionParam_t Generic_DefaultTransitionParam = {0x00};
#endif
static Generic_ModelFlag_t Generic_ModelFlag[APPLICATION_NUMBER_OF_ELEMENTS];

extern MOBLEUINT16 CommandStatus; /* Current on/off status, shared with Vendor model, used to publish status  */
MOBLEUINT8 GenericUpdateFlag = 0;
MOBLEUINT8 OptionalParam = 0;

extern Model_Tid_t Model_Tid;
/*Variables used for the publishing of binded data */
extern Model_Binding_Var_t Model_Binding_Var;

const MODEL_OpcodeTableParam_t Generic_Opcodes_Table[] = 
{
  /* Generic OnOff Server */
/* model_id                                         opcode,                                    reliable,    min_payload_size, max_payload_size, response_opcode,                        min_response_size, max_response_size
    Here in this array, Handler is not defined; */
#ifdef ENABLE_GENERIC_MODEL_SERVER_ONOFF  
  {GENERIC_ONOFF_SERVER_MODEL_ID,                   GENERIC_ON_OFF_GET,                        MOBLE_TRUE,  0,                0,                GENERIC_ON_OFF_STATUS,                  1,                 3},
  {GENERIC_ONOFF_SERVER_MODEL_ID,                   GENERIC_ON_OFF_SET_ACK,                    MOBLE_TRUE,  2,                4,                GENERIC_ON_OFF_STATUS,                  1,                 3},  
  {GENERIC_ONOFF_SERVER_MODEL_ID,                   GENERIC_ON_OFF_SET_UNACK,                  MOBLE_FALSE, 2,                4,                GENERIC_ON_OFF_STATUS,                  1,                 3}, 
  {GENERIC_ONOFF_SERVER_MODEL_ID,                   GENERIC_ON_OFF_STATUS,                     MOBLE_FALSE, 1,                3,                0,                                      1,                 3},
#endif                                                                                                                                                                                                           
#ifdef ENABLE_GENERIC_MODEL_SERVER_LEVEL                                                                                                                                                                         
  /* Generic Level Server */                                                                                                                                                                                     
  {GENERIC_LEVEL_SERVER_MODEL_ID,                   GENERIC_LEVEL_GET,                         MOBLE_TRUE,   0,                0,               GENERIC_LEVEL_STATUS,                   2,                 5}, 
  {GENERIC_LEVEL_SERVER_MODEL_ID,                   GENERIC_LEVEL_SET_ACK,                     MOBLE_TRUE,   3,                5,               GENERIC_LEVEL_STATUS,                   2,                 5},
  {GENERIC_LEVEL_SERVER_MODEL_ID,                   GENERIC_LEVEL_SET_UNACK,                   MOBLE_FALSE,  3,                5,               GENERIC_LEVEL_STATUS,                   2,                 5}, 
  {GENERIC_LEVEL_SERVER_MODEL_ID,                   GENERIC_DELTA_SET,                         MOBLE_TRUE,   5,                7,               GENERIC_LEVEL_STATUS,                   2,                 5},
  {GENERIC_LEVEL_SERVER_MODEL_ID,                   GENERIC_DELTA_SET_UNACK,                   MOBLE_FALSE,  5,                7,               GENERIC_LEVEL_STATUS,                   2,                 5},
  {GENERIC_LEVEL_SERVER_MODEL_ID,                   GENERIC_MOVE_SET,                          MOBLE_TRUE,   3,                5,               GENERIC_LEVEL_STATUS,                   2,                 5},
  {GENERIC_LEVEL_SERVER_MODEL_ID,                   GENERIC_MOVE_SET_UNACK,                    MOBLE_FALSE,  3,                5,               GENERIC_LEVEL_STATUS,                   2,                 5},
  {GENERIC_LEVEL_SERVER_MODEL_ID,                   GENERIC_LEVEL_STATUS,                      MOBLE_FALSE,  2,                5,               0,                                      2,                 5},
#endif                                                                                                                                                                                                           
#ifdef ENABLE_GENERIC_MODEL_SERVER_POWER_ONOFF                                                                                                                                                                   
  {GENERIC_POWER_ONOFF_SETUP_SERVER_MODEL_ID,       GENERIC_POWER_ON_OFF_SET,                  MOBLE_TRUE,   1,                1,               GENERIC_POWER_ON_OFF_STATUS,            1,                 1},
  {GENERIC_POWER_ONOFF_SETUP_SERVER_MODEL_ID,       GENERIC_POWER_ON_OFF_SET_UNACK,            MOBLE_FALSE,  1,                1,               0,                                      1,                 1},
  {GENERIC_SERVER_POWER_ONOFF_MODEL_ID,             GENERIC_POWER_ON_OFF_GET ,                 MOBLE_TRUE,   0,                0,               GENERIC_POWER_ON_OFF_STATUS,            1,                 1},
  {GENERIC_SERVER_POWER_ONOFF_MODEL_ID,             GENERIC_POWER_ON_OFF_STATUS ,              MOBLE_FALSE,  1,                1,               0,                                      1,                 1},
#endif                                                                                                                                                                                                           
#ifdef ENABLE_GENERIC_MODEL_SERVER_DEFAULT_TRANSITION_TIME                                                                                                                                                       
  /* Generic Default Transition Time Server Model  */                                                                                                                                                            
  {GENERIC_DEFAULT_TRANSITION_TIME_SERVER_MODEL_ID, GENERIC_DEFAULT_TRANSITION_TIME_GET,       MOBLE_TRUE,  0,                0,                GENERIC_DEFAULT_TRANSITION_TIME_STATUS, 1,                 1}, 
  {GENERIC_DEFAULT_TRANSITION_TIME_SERVER_MODEL_ID, GENERIC_DEFAULT_TRANSITION_TIME_SET,       MOBLE_TRUE,  1,                1,                GENERIC_DEFAULT_TRANSITION_TIME_STATUS, 1,                 1},
  {GENERIC_DEFAULT_TRANSITION_TIME_SERVER_MODEL_ID, GENERIC_DEFAULT_TRANSITION_TIME_SET_UNACK, MOBLE_FALSE, 1,                1,                GENERIC_DEFAULT_TRANSITION_TIME_STATUS, 1 ,                1}, 
  {GENERIC_DEFAULT_TRANSITION_TIME_SERVER_MODEL_ID, GENERIC_DEFAULT_TRANSITION_TIME_STATUS,    MOBLE_FALSE, 1,                1,                0,                                      1,                 1},
#endif                                                                                                                                                                                                           
  
  {0}
};

/* Private function prototypes -----------------------------------------------*/
WEAK_FUNCTION (MOBLE_RESULT Appli_Generic_OnOff_Set(Generic_OnOffStatus_t* pGeneric_OnOffParam, 
                                                    MOBLEUINT8 OptionalValid, 
                                                    uint16_t dstPeer, 
                                                    uint8_t elementIndex));
WEAK_FUNCTION (MOBLE_RESULT Appli_Generic_Level_Set(Generic_LevelStatus_t* plevelParam, 
                                                    MOBLEUINT8 OptionalValid, 
                                                    uint16_t dstPeer, 
                                                    uint8_t elementIndex));
WEAK_FUNCTION (MOBLE_RESULT Appli_Generic_Delta_Set(Generic_LevelStatus_t* pdeltalevelParam, 
                                                    MOBLEUINT8 OptionalValid, 
                                                    uint16_t dstPeer, 
                                                    uint8_t elementIndex));
WEAK_FUNCTION (MOBLE_RESULT Appli_Generic_Move_Set(Generic_LevelStatus_t* pdeltaMoveParam, 
                                                   MOBLEUINT8 OptionalValid, 
                                                   uint16_t dstPeer, 
                                                   uint8_t elementIndex));
WEAK_FUNCTION (MOBLE_RESULT Appli_Generic_PowerOnOff_Set(Generic_PowerOnOffParam_t* pPowerOnOffParam, 
                                                         MOBLEUINT8 OptionalValid, 
                                                         uint16_t dstPeer, 
                                                         uint8_t elementIndex));
WEAK_FUNCTION (void Appli_Generic_Restore_PowerOn_Value(MOBLEUINT8 restoreValue, 
                                                        uint16_t dstPeer, 
                                                        uint8_t elementIndex));
WEAK_FUNCTION (MOBLE_RESULT Appli_Generic_DefaultTransitionTime_Set(Generic_DefaultTransitionParam_t* pDefaultTimeParam, 
                                                                    MOBLEUINT8 OptionalValid, 
                                                                    uint16_t dstPeer, 
                                                                    uint8_t elementIndex));
WEAK_FUNCTION (MOBLE_RESULT Appli_Generic_OnOff_Status(MOBLEUINT8 const *pOnOff_status, 
                                                       MOBLEUINT32 plength, 
                                                       uint16_t dstPeer, 
                                                       uint8_t elementIndex));
WEAK_FUNCTION (MOBLE_RESULT Appli_Generic_Level_Status(MOBLEUINT8 const *plevel_status, 
                                                       MOBLEUINT32 plength, 
                                                       uint16_t dstPeer, 
                                                       uint8_t elementIndex));
WEAK_FUNCTION (MOBLE_RESULT Appli_Generic_PowerOnOff_Status(MOBLEUINT8 const *powerOnOff_status , 
                                                            MOBLEUINT32 plength, 
                                                            uint16_t dstPeer, 
                                                            uint8_t elementIndex));
WEAK_FUNCTION (MOBLE_RESULT Appli_Generic_DefaultTransitionTime_Status(MOBLEUINT8 const *pTransition_status , 
                                                                       MOBLEUINT32 plength, 
                                                                       uint16_t dstPeer, 
                                                                       uint8_t elementIndex));
WEAK_FUNCTION (MOBLE_RESULT Appli_Generic_GetOnOffStatus(MOBLEUINT8* pOnOff_Status, 
                                                         uint16_t dstPeer, 
                                                         uint8_t elementIndex));
WEAK_FUNCTION (MOBLE_RESULT Appli_Generic_GetOnOffValue(MOBLEUINT8* pOnOff_Value, 
                                                        uint16_t dstPeer, 
                                                        uint8_t elementIndex));
WEAK_FUNCTION (MOBLE_RESULT Appli_Generic_GetLevelStatus(MOBLEUINT8* pLevel_Status, 
                                                         uint16_t dstPeer, 
                                                         uint8_t elementIndex));
WEAK_FUNCTION (MOBLE_RESULT Appli_Generic_GetPowerOnOffStatus(MOBLEUINT8* pPower_Status, 
                                                              uint16_t dstPeer, 
                                                              uint8_t elementIndex));
WEAK_FUNCTION (MOBLE_RESULT Appli_Generic_GetDefaultTransitionStatus(MOBLEUINT8* pTransition_Status, 
                                                                     uint16_t dstPeer, 
                                                                     uint8_t elementIndex));
WEAK_FUNCTION(MOBLE_RESULT Appli_GenericClient_Level_Set_Unack(void));

/* Private functions ---------------------------------------------------------*/

/**
* @brief  Generic_OnOff_Set: This function is called for both Acknowledged and 
*         unacknowledged message
* @param  pOnOff_param: Pointer to the parameters received for message
* @param  length: Length of the parameters received for message
* @param  *pmsgParam: Pointer to structure of message header for parameters:
*          elementIndex, src, dst addresses, TTL, RSSI, NetKey & AppKey Offset
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Generic_OnOff_Set(MOBLEUINT8 const *pOnOff_param, 
                               MOBLEUINT32 length,
                               MODEL_MessageHeader_t *pmsgParam)  
{
  
  /* 3.2.1.2 Generic OnOff Set 
  OnOff: 1B The target value of the Generic OnOff state 
  TID :  1B Transaction Identifier
  Transition Time: 1B Format as defined in Section 3.1.3. (Optional)
  Delay: 1B Message execution delay in 5 millisecond steps (C.1)
  */
  
  TRACE_M(TF_GENERIC_M, "Generic_OnOff_Set callback received \r\n");  
  
  Generic_OnOffParam_t Generic_OnOffParam; 
  Generic_OnOffParam.TargetOnOffState = pOnOff_param[0];
  Generic_OnOffParam.Generic_TID = pOnOff_param[1];
  CommandStatus = pOnOff_param[0];

  Generic_OnOffParam.Transition_Time = 0;

  /*  
  Checking for optional parameters
  length > 2 , 4 values  received(OnOff status, TID, Trasmisition time(optional),
  Delay(optional)),length < 2 OnOff status and TID   
  */
  
  if((length > 2) && (pOnOff_param[2] !=0))
  {
    /* Transition_Time & Delay_Time Present */
    Generic_OnOffParam.Transition_Time = pOnOff_param[2];
    Generic_OnOffParam.Delay_Time = pOnOff_param[3];
      
    /* 3.1.1.1 Binary state transitions: Because binary states cannot support transitions, 
       when changing to 0x01 (On), the Generic OnOff state shall change immediately 
       when the transition starts, and when changing to 0x00, the state shall
       change when the transition finishes.*/
    if(pOnOff_param[0] == 1)
    {
      Generic_OnOffStatus[pmsgParam->elementIndex].Present_OnOff_State = pOnOff_param[0];
    }
    Generic_OnOffStatus[pmsgParam->elementIndex].Target_OnOff = Generic_OnOffParam.TargetOnOffState;
    Generic_OnOffStatus[pmsgParam->elementIndex].RemainingTime = Generic_OnOffParam.Transition_Time;   
    
    Generic_TemporaryStatus[pmsgParam->elementIndex].RemainingTime = Generic_OnOffStatus[pmsgParam->elementIndex].RemainingTime;
    /* Function to calculate time parameters, step resolution
      step size for transition state machine
    */
    Generic_GetStepValue(pOnOff_param[2], pmsgParam->elementIndex);   
    /*option parameter flag, enable to send optional parameters in status.*/         
    Generic_ModelFlag[pmsgParam->elementIndex].GenericOptionalParam = 1;
    /*Flag to enable the on Off transition state machine */
    Generic_ModelFlag[pmsgParam->elementIndex].GenericTransitionFlag = GENERIC_ON_OFF_TRANSITION_START;
    /* flag is used for the application to get the information about the transition 
       time parameter is included or not in the received message.
    */
    OptionalParam = IN_TRANSITION;
  } 
  else
  {
   Generic_OnOffStatus[pmsgParam->elementIndex].Present_OnOff_State = Generic_OnOffParam.TargetOnOffState;
   
    /* when default transition time enabled,and the target time is not given by client
       the transition time will be used from default value
      */
#ifdef ENABLE_GENERIC_MODEL_SERVER_DEFAULT_TRANSITION_TIME     
    Generic_OnOffDefaultTransitionValue(pmsgParam->elementIndex);
/* EME BEGIN: management of Present_OnOff_Value and Target_OnOff on immediate transition */
    if(Generic_TimeParam[pmsgParam->elementIndex].StepValue == 0)
    {
      /* Immediate transition */
      if(Generic_OnOffStatus[pmsgParam->elementIndex].Present_OnOff_State > 0)
      {
        Generic_OnOffStatus[pmsgParam->elementIndex].Present_OnOff_Value = PWM_TIME_PERIOD;
        Generic_OnOffStatus[pmsgParam->elementIndex].Target_OnOff = PWM_TIME_PERIOD;
      }
      else
      {
        Generic_OnOffStatus[pmsgParam->elementIndex].Present_OnOff_Value = 0;
        Generic_OnOffStatus[pmsgParam->elementIndex].Target_OnOff = 0;
      }
      Generic_OnOffStatus[pmsgParam->elementIndex].RemainingTime = Generic_OnOffParam.Transition_Time;   
    }
/* EME END: management of Present_OnOff_Value and Target_OnOff on immediate transition */
#else
    /* When no optional parameter received, target value will be set as present
       value in application.
    */  
    OptionalParam = NO_TRANSITION;
    Generic_ModelFlag[pmsgParam->elementIndex].GenericTransitionFlag = GENERIC_TRANSITION_STOP;
#endif       
    
    Generic_OnOffStatus[pmsgParam->elementIndex].Present_OnOff_State = Generic_OnOffParam.TargetOnOffState;
  }
  
  /* Application Callback */
  (GenericAppli_cb.OnOff_Set_cb)(&Generic_OnOffStatus[pmsgParam->elementIndex], OptionalParam, pmsgParam->dst_peer, pmsgParam->elementIndex);
#ifdef ENABLE_MODEL_BINDING    

#ifdef ENABLE_LIGHT_MODEL_SERVER_LIGHTNESS  
  /* Binding of data b/w Generic on off and Light lightness Actual model */
  GenericOnOff_LightActualBinding(&Generic_OnOffStatus[pmsgParam->elementIndex],pmsgParam->elementIndex);
#endif

#ifdef ENABLE_LIGHT_MODEL_SERVER_LC 
  //todo
  Binding_GenericOnOff_LightLcLightOnOff(pmsgParam->elementIndex,
                                         Generic_OnOffParam.TargetOnOffState,
                                          0, 0, 0);//todo transition parameters
#endif
  
#endif
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Generic_OnOff_Status
* @param  pOnoff_status: Pointer to the status message, which needs to be updated
* @param  plength: Pointer to the Length of the Status message
* @param  dstPeer: *pmsgParam Pointer to structure of message header for parameters:
*                  elementIndex, src, dst addresses, TTL, RSSI, NetKey & AppKey Offset

* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Generic_OnOff_Status(MOBLEUINT8* pOnOff_status, 
                                  MOBLEUINT32 *plength,
                                  MODEL_MessageHeader_t *pmsgParam) 
{
  /* 
  Following is the status message:
  Present OnOff The present value of the Generic OnOff state. 
  Target OnOff The target value of the Generic OnOff state (optional).
  Remaining Time is transition time. 
  */
  
  TRACE_M(TF_GENERIC_M, ">>>\r\n");
  TRACE_M(TF_SERIAL_CTRL,"#8201! \n\r");
  /* 
  Default value of GenericOptionalParam=0, 
  GenericOptionalParam set equal to 1 in Generic_OnOff_Set for Generic_OnOff_Status 
  */
  if((Generic_ModelFlag[pmsgParam->elementIndex].GenericOptionalParam == 1) || (Generic_TimeParam[pmsgParam->elementIndex].StepValue != 0))
  {   
    /*  
    When optional parameter received present value,targert value, remaing time be sent in status message
    length of received data is equal to 4B
    */
    TRACE_M(TF_GENERIC_M, "Generic_OnOff_Status sent with Transition \r\n"); 

    *(pOnOff_status+1) = Generic_OnOffStatus[pmsgParam->elementIndex].Target_OnOff;
    *(pOnOff_status+2) = Generic_OnOffStatus[pmsgParam->elementIndex].RemainingTime;
       *plength = 3; 
    Generic_ModelFlag[pmsgParam->elementIndex].GenericOptionalParam = 0;
  }
  else
  { /* When no optional parameter received, target value will
       be sent in status message.
       length of received data is equal to 2B
    */
    TRACE_M(TF_GENERIC_M, "Generic_OnOff_Status sent without Transition \r\n"); 
    TRACE_M(TF_GENERIC_M, "%d \r\n",*pOnOff_status);
    *plength = 1;
  }

  *pOnOff_status = Generic_OnOffStatus[pmsgParam->elementIndex].Present_OnOff_State; 
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Generic_Level_Set: This function is called for both Acknowledged and 
*         unacknowledged message
* @param  plevel_param: Pointer to the parameters received for message
* @param  length: Length of the parameters received for message
* @param  *pmsgParam Pointer to structure of message header for parameters:
*         elementIndex, src, dst addresses, TTL, RSSI, NetKey & AppKey Offset
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Generic_Level_Set(const MOBLEUINT8* plevel_param, 
                               MOBLEUINT32 length,
                               MODEL_MessageHeader_t *pmsgParam) 
{
  /*
  3.2.2.2 Generic Level Set
  Level: 2B The target value of the Generic Level state
  TID :  1B Transaction Identifier
  Transition Time: 1B Format as defined in Section 3.1.3. (Optional)
  Delay:1B Message execution delay in 5 milliseconds steps (C.1)
  */
  
  TRACE_M(TF_GENERIC_M, "Generic_Level_Set callback received \r\n");
  
  Generic_LevelParam_t Generic_LevelParam;
  MOBLEINT16 setValue;
  
  Generic_LevelParam.TargetLevel = CopyU8LittleEndienArrayToU16word((MOBLEUINT8*)plevel_param);
  Generic_LevelParam.Generic_TID = plevel_param[2]; 
  
  setValue =  Generic_LevelParam.TargetLevel;
  /* Check for Optional Parameters. 
     length > 3  plevel_param has level,TID,Transition Time, Delay      
     length < 3  plevel_param has level,TID
  */ 
  if((length > 3) && (plevel_param[3] !=0))
  {
     Generic_LevelParam.Transition_Time = plevel_param[3];
     Generic_LevelParam.Delay_Time = plevel_param[4];
     /* Copy the data into status message which needs to be update in 
       application message.
     */
    Generic_LevelStatus[pmsgParam->elementIndex].Target_Level16 = setValue;
    Generic_LevelStatus[pmsgParam->elementIndex].RemainingTime = Generic_LevelParam.Transition_Time;
    /* copy status parameters in Temporary parameters for transition 
       process.
    */
    Generic_TemporaryStatus[pmsgParam->elementIndex].TargetValue16 = Generic_LevelStatus[pmsgParam->elementIndex].Target_Level16;
    Generic_TemporaryStatus[pmsgParam->elementIndex].RemainingTime = Generic_LevelStatus[pmsgParam->elementIndex].RemainingTime;
    /* Function to calculate time parameters, step resolution
      step size for transition state machine
    */
    Generic_GetStepValue(plevel_param[3], pmsgParam->elementIndex); 
    /*option parameter flag, enable to sent all required parameter in status.*/
    Generic_ModelFlag[pmsgParam->elementIndex].GenericOptionalParam = 1;
    /*transition process enable flag. */
    Generic_ModelFlag[pmsgParam->elementIndex].GenericTransitionFlag = GENERIC_LEVEL_TRANSITION_START;
  }
  else
  {     
#ifdef ENABLE_GENERIC_MODEL_SERVER_DEFAULT_TRANSITION_TIME
    
    Generic_LevelDefaultTransitionValue(pmsgParam->elementIndex, setValue);

#else
     /* When no optional parameter received, target value will
         be set as present value in application.
     */
    Generic_ModelFlag[pmsgParam->elementIndex].GenericTransitionFlag = GENERIC_TRANSITION_STOP;
    Generic_LevelStatus[pmsgParam->elementIndex].Present_Level16= setValue;
#endif    
  }  
  Generic_LevelStatus[pmsgParam->elementIndex].Last_Present_Level16 = Generic_LevelStatus[pmsgParam->elementIndex].Present_Level16;
 
  /* Application Callback */
  (GenericAppli_cb.Level_Set_cb)(&Generic_LevelStatus[pmsgParam->elementIndex], 0, pmsgParam->dst_peer, pmsgParam->elementIndex);
#ifdef ENABLE_MODEL_BINDING       

#ifdef ENABLE_LIGHT_MODEL_SERVER_LIGHTNESS  
  /* Binding of Generic level with light lightnes actual */
  GenericLevel_LightBinding(&Generic_LevelStatus[pmsgParam->elementIndex],BINDING_GENERIC_LEVEL_SET, pmsgParam->elementIndex);
#endif 
        
#endif   /* ENABLE_MODEL_BINDING */
  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Generic_Delta_Set: This function is called for both Acknowledged 
*         and unacknowledged message
* @param  plevel_param: Pointer to the parameters received for message
* @param  length: Length of the parameters received for message
* @param  *pmsgParam Pointer to structure of message header for parameters:
*          elementIndex, src, dst addresses, TTL, RSSI, NetKey & AppKey Offset
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Generic_Delta_Set(const MOBLEUINT8* plevel_param, MOBLEUINT32 length,\
                               MODEL_MessageHeader_t *pmsgParam) 
{
  /*
  3.2.2.4 Generic Delta Set
  Delta Level: 4B The Delta change of the Generic Level state
  TID:   1B Transaction Identifier
  Transition Time: 1B Format as defined in Section 3.1.3. (Optional)
  Delay: 1B Message execution delay in 5 milliseconds steps (C.1)
  */
  
  TRACE_M(TF_GENERIC_M, "Generic_Delta_Set callback received \r\n");
  
  Generic_DeltaLevelParam_t Generic_DeltaLevelParam ;
  MOBLEUINT32 delta;
  
  /* Copy the 4Bytes data to local variable */
  delta = (plevel_param[3] << 24);
  delta |= (plevel_param[2] << 16);
  delta |= (plevel_param[1] << 8);
  delta |= (plevel_param[0]);
  Generic_DeltaLevelParam.TargetDeltaLevel32 = delta;
  
  TRACE_M(TF_GENERIC_M,"Generic Delta value %.2lx \r\n",
          Generic_DeltaLevelParam.TargetDeltaLevel32);
  
  Generic_DeltaLevelParam.Generic_TID = plevel_param[4];
  /* 
     Check for Optional Parameters 
     length > 5  Delata level,TID, Trasition Time(optional),Delay(optional)
     length < 5  Delta level,TID
  */    
  if((length > 5) && (plevel_param[5] !=0))
  {
    Generic_DeltaLevelParam.Transition_Time = plevel_param[5];
    Generic_DeltaLevelParam.Delay_Time = plevel_param[6];
    /* Copy the data into status message which needs to be update in 
     application message.
    */ 
    Generic_LevelStatus[pmsgParam->elementIndex].Target_Level16 = Generic_LevelStatus[pmsgParam->elementIndex].Present_Level16 + 
                                                Generic_DeltaLevelParam.TargetDeltaLevel32;
    Generic_LevelStatus[pmsgParam->elementIndex].RemainingTime = Generic_DeltaLevelParam.Transition_Time;
       
    /* copy status parameters in Temporary parameters for transition 
     process.
    */
    Generic_TemporaryStatus[pmsgParam->elementIndex].TargetValue16 = Generic_LevelStatus[pmsgParam->elementIndex].Target_Level16;
    Generic_TemporaryStatus[pmsgParam->elementIndex].RemainingTime = Generic_LevelStatus[pmsgParam->elementIndex].RemainingTime;
    /* Function to calculate time parameters, step resolution
    step size for transition state machine.
    */
    Generic_GetStepValue(plevel_param[5], pmsgParam->elementIndex);   
       
    /*option parameter flag, enable to sent all required parameter in status.*/ 
    Generic_ModelFlag[pmsgParam->elementIndex].GenericOptionalParam = 1;
       
    /*transition process enable flag. */
    Generic_ModelFlag[pmsgParam->elementIndex].GenericTransitionFlag = GENERIC_LEVEL_TRANSITION_START;
  }
  else
  {   
		
    if(Generic_LevelStatus[pmsgParam->elementIndex].Last_Level_TID == Generic_DeltaLevelParam.Generic_TID)
    {
        if(Generic_DeltaLevelParam.TargetDeltaLevel32 > Generic_LevelStatus[pmsgParam->elementIndex].Last_delta_level)
      {
          Generic_LevelStatus[pmsgParam->elementIndex].Present_Level16 += (Generic_DeltaLevelParam.TargetDeltaLevel32  
          - Generic_LevelStatus[pmsgParam->elementIndex].Last_delta_level);     
      }
      else
      {
          Generic_LevelStatus[pmsgParam->elementIndex].Present_Level16 -= (Generic_LevelStatus[pmsgParam->elementIndex].Last_delta_level
            -Generic_DeltaLevelParam.TargetDeltaLevel32) ;           
      }       
    }
    else
    {    /*If TID value is different from the last TID, then new transaction has been started*/
        Generic_LevelStatus[pmsgParam->elementIndex].Present_Level16 += Generic_DeltaLevelParam.TargetDeltaLevel32;
    }
    
#ifdef ENABLE_GENERIC_MODEL_SERVER_DEFAULT_TRANSITION_TIME
    
    Generic_DeltaDefaultTransitionValue(pmsgParam->elementIndex, delta);
    
#else
    /* When no optional parameter received, target value will
    be set as present value in application.
    */
    Generic_ModelFlag[pmsgParam->elementIndex].GenericTransitionFlag = GENERIC_TRANSITION_STOP;
#endif   
  }
  
  TRACE_M(TF_GENERIC_M, "Generic Level value %.2x \r\n",
          Generic_LevelStatus[pmsgParam->elementIndex].Present_Level16);
  
  Generic_LevelStatus[pmsgParam->elementIndex].Last_delta_level = Generic_DeltaLevelParam.TargetDeltaLevel32;
  Generic_LevelStatus[pmsgParam->elementIndex].Last_Level_TID = Generic_DeltaLevelParam.Generic_TID; 
      
  /* Application Callback */
  (GenericAppli_cb.Level_Set_cb)(&Generic_LevelStatus[pmsgParam->elementIndex], 0, pmsgParam->dst_peer, pmsgParam->elementIndex);
#ifdef ENABLE_MODEL_BINDING    
#ifdef ENABLE_LIGHT_MODEL_SERVER_LIGHTNESS  
  /* Binding of Generic level with light lightnes actual */
  GenericLevel_LightBinding(&Generic_LevelStatus[pmsgParam->elementIndex],BINDING_GENERIC_LEVEL_SET,pmsgParam->elementIndex);
#endif  
#endif  
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Generic_Move_Set: This function is called for both 
*         Acknowledged and unacknowledged message
* @param  plevel_param: Pointer to the parameters received for message
* @param  length: Length of the parameters received for message
* @param  *pmsgParam Pointer to structure of message header for parameters:
*          elementIndex, src, dst addresses, TTL, RSSI, NetKey & AppKey Offset
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Generic_Move_Set(const MOBLEUINT8* plevel_param, 
                              MOBLEUINT32 length,
                              MODEL_MessageHeader_t *pmsgParam) 
{
  /*
  3.2.2.6 Generic Level Move Set
  Level: 2B The target value of the Generic Level state
  TID:   1B Transaction Identifier
  Transition Time: 1B Format as defined in Section 3.1.3. (Optional)
  Delay: 1B Message execution delay in 5 milliseconds steps (C.1)
  */
  TRACE_M(TF_GENERIC_M, "Generic_Move_Set callback received \r\n");
  
  Generic_LevelMoveParam_t  Generic_LevelMoveParam;
  
  Generic_LevelMoveParam.TargetMoveLevel16  = (plevel_param[1] << 8);
  Generic_LevelMoveParam.TargetMoveLevel16 |= (plevel_param[0]);
  Generic_LevelMoveParam.Generic_TID = plevel_param[2];
  
  /* Check for Optional Parameters 
  
    length > 3 plevel_param has level,TID,Trasition Time,Delay
    length < 3 plevel_param has level,TID
  */     
  if((length > 3) && (plevel_param[3] !=0))
  {
    Generic_LevelMoveParam.Transition_Time = plevel_param[3];
    Generic_LevelMoveParam.Delay_Time = plevel_param[4];
    
    /* Copy the data into status message which needs to be update in 
      application message.
    */ 
    /* The assumption here is that the move command starts transition with each 
       step of size delta, which terminates only at max or min value */
    if (Generic_LevelMoveParam.TargetMoveLevel16 <= 0x7FFF)
    {
        Generic_LevelStatus[pmsgParam->elementIndex].Target_Level16 = 0x7FFF;
    }
    else
    {
        Generic_LevelStatus[pmsgParam->elementIndex].Target_Level16 = 0x8000;
    }

    Generic_LevelStatus[pmsgParam->elementIndex].RemainingTime = UNDEFSTEPVAL;
    
    /* Function to calculate time parameters, step resolution
      step size for transition state machine.
    */
    Generic_GetStepValue(plevel_param[3], pmsgParam->elementIndex);   
    /* option parameter flag, enable to sent all required parameter in status.*/
    Generic_ModelFlag[pmsgParam->elementIndex].GenericOptionalParam = 1;  
    /* transition process enable flag. */
    Generic_ModelFlag[pmsgParam->elementIndex].GenericTransitionFlag = GENERIC_LEVEL_TRANSITION_START;
  }
  else
  {   
    Generic_ModelFlag[pmsgParam->elementIndex].GenericTransitionFlag = GENERIC_TRANSITION_STOP;		
    Generic_TimeParam[pmsgParam->elementIndex].StepValue = 0;
    if(Generic_LevelStatus[pmsgParam->elementIndex].Last_Level_TID == Generic_LevelMoveParam.Generic_TID)
    {
      Generic_LevelStatus[pmsgParam->elementIndex].Present_Level16 =  Generic_LevelStatus[pmsgParam->elementIndex].Last_Present_Level16 
        + Generic_LevelMoveParam.TargetMoveLevel16;
    }
    else
    {
      
    }
    }
    
  Generic_LevelStatus[pmsgParam->elementIndex].Last_Level_TID = Generic_LevelMoveParam.Generic_TID; 
 
  /* Application Callback */
  (GenericAppli_cb.LevelDeltaMove_Set_cb)(&Generic_LevelStatus[pmsgParam->elementIndex], 0, pmsgParam->dst_peer, pmsgParam->elementIndex);
     
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Generic_Level_Status
* @param  plevel_status: Pointer to the status message, which needs to be updated
* @param  plength: Pointer to the Length of the Status message
* @param  *pmsgParam Pointer to structure of message header for parameters:
*          elementIndex, src, dst addresses, TTL, RSSI, NetKey & AppKey Offset
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Generic_Level_Status(MOBLEUINT8* plevel_status, 
                                  MOBLEUINT32 *plength,
                                  MODEL_MessageHeader_t *pmsgParam) 
{
  /* 
  3.2.2.8 Generic Level Status
  Following is the status message:
  Present Level: 2B The present value of the Generic Level state. 
  Target Level: 2B The target value of the Generic Level state (Optional). 
  Remaining Time: 1B Format as defined in Section 3.1.3 (C.1).
  
  */
  
  TRACE_M(TF_GENERIC_M, "Generic_Level_Status callback received \r\n");
  TRACE_M(TF_SERIAL_CTRL,"#8205! \n\r");
  
  /* checking the transition is in process.
  checking for remaining time is not equal to zero.
  */
  
  if((Generic_ModelFlag[pmsgParam->elementIndex].GenericOptionalParam ==1) || (Generic_TimeParam[pmsgParam->elementIndex].StepValue != 0))
  {
    TRACE_M(TF_GENERIC_M, "Generic_Level_Status sent with Transition \r\n"); 

    *(plevel_status+2) = Generic_LevelStatus[pmsgParam->elementIndex].Target_Level16;
    *(plevel_status+3) = Generic_LevelStatus[pmsgParam->elementIndex].Target_Level16 >> 8;
    *(plevel_status+4) = Generic_LevelStatus[pmsgParam->elementIndex].RemainingTime;
    *plength = 5;
    Generic_ModelFlag[pmsgParam->elementIndex].GenericOptionalParam = 0;    
  }
  else
  {
    TRACE_M(TF_GENERIC_M, "Generic_Level_Status sent without Transition \r\n"); 
    *plength = 2;             
  }
     
  *(plevel_status) = Generic_LevelStatus[pmsgParam->elementIndex].Present_Level16;
  *(plevel_status+1) = Generic_LevelStatus[pmsgParam->elementIndex].Present_Level16 >> 8;
  TRACE_M(TF_GENERIC_M, "%d \r\n", Generic_LevelStatus[pmsgParam->elementIndex].Present_Level16); 
  return MOBLE_RESULT_SUCCESS;   
}

/**
* @brief  Generic_PowerOnOff_Set: This function is called for both 
*         Acknowledged and unacknowledged message
* @param  powerOnOff_param: Pointer to the parameters received for message
* @param  length: Length of the parameters received for message
* @param  *pmsgParam Pointer to structure of message header for parameters:
*          elementIndex, src, dst addresses, TTL, RSSI, NetKey & AppKey Offset
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Generic_PowerOnOff_Set(const MOBLEUINT8 *powerOnOff_param , MOBLEUINT32 length,\
                                   MODEL_MessageHeader_t *pmsgParam) 
{
  /* 
  3.2.4.2 Generic Power On Off Time
  Following is the set message:
  powerOnOff_param:1B parameter is received to set the power on off model.  
  */
  TRACE_M(TF_GENERIC_M, "Generic_PowerOnOff_Set callback received \r\n");
  TRACE_M(TF_GENERIC_M,"Generic_PowerOnOff_Set is %d\r\n", powerOnOff_param[0]);
  Generic_PowerOnOffParam_t Generic_PowerOnOffParam;
  Generic_PowerOnOffParam.PowerOnOffState = powerOnOff_param[0];
  
  /* Application Callback */
  (GenericAppli_cb.GenericPowerOnOff_cb)(&Generic_PowerOnOffParam, length, pmsgParam->dst_peer,\
                                                     pmsgParam->elementIndex);
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief  Generic_PowerOnOff_Status
* @param  powerOnOff_status: Pointer to the status message, which needs to be updated
* @param  plength: Pointer to the Length of the Status message
* @param  *pmsgParam Pointer to structure of message header for parameters:
*          elementIndex, src, dst addresses, TTL, RSSI, NetKey & AppKey Offset
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Generic_PowerOnOff_Status(MOBLEUINT8 *powerOnOff_status , 
                                       MOBLEUINT32 *plength,
                                       MODEL_MessageHeader_t *pmsgParam) 
{  
  /* 
  3.2.4.4 Generic OnPowerUp Status
  Following is the status message:
  powerOnOff_status: 1B is the status parameter of the Power on off model. 
  */  
  MOBLEUINT8 Generic_GetBuff[2] ;
  TRACE_M(TF_GENERIC_M, "Generic_PowerOnOff_Status callback received \r\n");
  TRACE_M(TF_SERIAL_CTRL,"#8211! \n\r");
  
  /* Function call back to get the values from application*/
  (Appli_GenericState_cb.GetPowerOnOffStatus_cb)(Generic_GetBuff, pmsgParam->dst_peer,\
                                                        pmsgParam->elementIndex);
   
  *(powerOnOff_status) = Generic_GetBuff[0];
  *plength = 1;
 
  return MOBLE_RESULT_SUCCESS;
}


#ifdef ENABLE_GENERIC_MODEL_SERVER_DEFAULT_TRANSITION_TIME
/**
* @brief  Generic_DefaultTransitionTime_Set: This function is called for both 
*         Acknowledged and unacknowledged message
* @param  defaultTransition_param: Pointer to the parameters received for message
* @param  length: Length of the parameters received for message
* @param  *pmsgParam Pointer to structure of message header for parameters:
*          elementIndex, src, dst addresses, TTL, RSSI, NetKey & AppKey Offset
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Generic_DefaultTransitionTime_Set(const MOBLEUINT8 *defaultTransition_param,
                                               MOBLEUINT32 length, 
                                               MODEL_MessageHeader_t *pmsgParam)  
{
  /* 
  3.2.3.2 Generic Default Transition Time Set
  Following is the set message:
  defaultTime_param:1B parameter is received to set the Default transition time model.  
  */
  TRACE_M(TF_GENERIC_M, "Generic_DefaultTransitionTime_Set callback received \r\n"); 
  TRACE_M(TF_GENERIC_M,"Generic_DefaultTransitionTime is = %.2x \r\n", defaultTransition_param[0]); 
  Generic_DefaultTransitionParam.DefaultTransitionTime = defaultTransition_param[0];

  /* Application Callback */
  (GenericAppli_cb.GenericDefaultTransition_cb)(&Generic_DefaultTransitionParam,
                                length, pmsgParam->dst_peer, pmsgParam->elementIndex);
  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  Generic_DefaultTransitionTime_Status
* @param  pTransition_status: Pointer to the status message, which needs to be updated
* @param  plength: Pointer to the Length of the Status message
* @param  *pmsgParam Pointer to structure of message header for parameters:
*          elementIndex, src, dst addresses, TTL, RSSI, NetKey & AppKey Offset
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Generic_DefaultTransitionTime_Status(MOBLEUINT8 *pTransition_status,\
                          MOBLEUINT32 *plength, MODEL_MessageHeader_t *pmsgParam)
{  
  /* 
  3.2.3.4 Generic Default Transition Time Status
  Following is the status message:
  powerOnOff_status: 1B is the status parameter of the Default transition time model. 
  */  
  MOBLEUINT8 Generic_GetBuff[2] ;
  TRACE_M(TF_GENERIC_M, "Generic_DefaultTransitionTime_Status callback received \r\n");
  TRACE_M(TF_SERIAL_CTRL,"#820D! \n\r");
  
  /* Function call back to get the values from application*/
  (Appli_GenericState_cb.GetDefaultTransitionStatus_cb)(Generic_GetBuff, pmsgParam->dst_peer,\
                                                         pmsgParam->elementIndex);
   
  *(pTransition_status) = Generic_GetBuff[0];
  *plength = 1;
 
   TRACE_M(TF_GENERIC_M,"Generic_DefaultTransitionTime_Status = %.2x \r\n", Generic_GetBuff[0]);
   
 
  return MOBLE_RESULT_SUCCESS;
}
#endif


/**
* @brief   GenericModelServer_GetOpcodeTableCb: This function is call-back 
*          from the library to send Model Opcode Table info to library
* @param  MODEL_OpcodeTableParam_t:  Pointer to the Generic Model opcode array 
* @param  length: Pointer to the Length of Generic Model opcode array
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT GenericModelServer_GetOpcodeTableCb(const MODEL_OpcodeTableParam_t **data, 
                                                 MOBLEUINT16 *length)
{
  *data = Generic_Opcodes_Table;
  *length = sizeof(Generic_Opcodes_Table)/sizeof(Generic_Opcodes_Table[0]);

  return MOBLE_RESULT_SUCCESS;
}


/**
* @brief  GenericModelServer_GetStatusRequestCb : This function is call-back 
*         from the library to send response to the message from peer
* @param  *pmsgParam Pointer to structure of message header for parameters:
*          elementIndex, src, dst addresses, TTL, RSSI, NetKey & AppKey Offset
* @param  opcode: Received opcode of the Status message callback
* @param  pResponsedata: Pointer to the buffer to be updated with status
* @param  plength: Pointer to the Length of the data, to be updated by application
* @param  pRxData: Pointer to the data received in packet.
* @param  dataLength: length of the data in packet.
* @param  response: Value to indicate wheather message is acknowledged meassage or not.
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT GenericModelServer_GetStatusRequestCb(MODEL_MessageHeader_t *pmsgParam,
                                                   MOBLEUINT16 opcode, 
                                                   MOBLEUINT8 *pResponsedata, 
                                                   MOBLEUINT32 *plength, 
                                                   MOBLEUINT8 const *pRxData,
                                                   MOBLEUINT32 dataLength,
                                                   MOBLEBOOL response)

{
  TRACE_M(TF_GENERIC_M, "response status enable \n\r");
  switch(opcode)
  {
#ifdef ENABLE_GENERIC_MODEL_SERVER_ONOFF      
    case GENERIC_ON_OFF_STATUS:
    {
      Generic_OnOff_Status(pResponsedata, plength, pmsgParam);       
      break;
    }
#endif 
    
#ifdef ENABLE_GENERIC_MODEL_SERVER_LEVEL      
    case GENERIC_LEVEL_STATUS:
    {
      Generic_Level_Status(pResponsedata, plength, pmsgParam);     
      break;
    }
#endif      
    
#ifdef ENABLE_GENERIC_MODEL_SERVER_POWER_ONOFF    
    case GENERIC_POWER_ON_OFF_STATUS:
    {
      Generic_PowerOnOff_Status(pResponsedata, plength, pmsgParam);
      break;
    }
#endif
    
#ifdef ENABLE_GENERIC_MODEL_SERVER_DEFAULT_TRANSITION_TIME    
    case GENERIC_DEFAULT_TRANSITION_TIME_STATUS:
    {
      Generic_DefaultTransitionTime_Status(pResponsedata, plength, pmsgParam);
    }
#endif    
    default:
    {
      break;
    }
  }
  return MOBLE_RESULT_SUCCESS;    
}


/**
* @brief  GenericModelServer_ProcessMessageCb: This is a callback function from
*         the library whenever a Generic Model message is received
* @param  *pmsgParam Pointer to structure of message header for parameters:
*          elementIndex, src, dst addresses, TTL, RSSI, NetKey & AppKey Offset
* @param  opcode: Received opcode of the Status message callback
* @param  pRxData: Pointer to the data received in packet.
* @param  dataLength: length of the data in packet.
* @param  response: Value to indicate whether message is acknowledged message or not.
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT GenericModelServer_ProcessMessageCb(MODEL_MessageHeader_t *pmsgParam,
                                                 MOBLEUINT16 opcode, 
                                                 MOBLEUINT8 const *pRxData, 
                                                 MOBLEUINT32 dataLength, 
                                                 MOBLEBOOL response)
{

  MOBLE_RESULT result = MOBLE_RESULT_SUCCESS;
  MOBLE_ADDRESS publishAddress;
  MOBLEUINT8 modelStateChangeFlag = MOBLE_FALSE; 
  
  TRACE_M(TF_GENERIC_M, "elementIndex %.2x dst_peer %.2X peer_add %.2X opcode %.2X response %.2X\r\n",
          pmsgParam->elementIndex, pmsgParam->dst_peer, pmsgParam->peer_addr, opcode, response);   
                                                      
  switch(opcode)
  {
#ifdef ENABLE_GENERIC_MODEL_SERVER_ONOFF
    
    case GENERIC_ON_OFF_SET_ACK:
    case GENERIC_ON_OFF_SET_UNACK:
    {
      result = Chk_ParamValidity(pRxData[0], 1); 
