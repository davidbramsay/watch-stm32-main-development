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
#include "stm32wbxx_nucleo.h"
#include "cmsis_os.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler(void);
void   MX_LPUART1_UART_Init(void);
void   MX_USART1_UART_Init(void);

osMutexId_t rtcMutexHandle;
osMessageQueueId_t bleRXqueueHandle;

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

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
