/*
 * veml7070.h

// Adapted with reference to adafruit library found here:
// https://github.com/adafruit/Adafruit_VEML7700/blob/master/Adafruit_VEML7700.cpp
// by David Ramsay (dramsay9)
// for STM32WB w/FreeRTOS

 *  Created on: Feb 3, 2021
 */

#ifndef INC_VEML7700_H_
#define INC_VEML7700_H_

#include "stdio.h"
#include "stm32wbxx_hal.h"


#define VEML_ADDR (0x10 << 1)

// normal operation, poll once every 5 seconds
// powersave mode 4 + gain 2x + integration time 800ms = 4800ms refresh time, 8uA, 0.0036 lx/bit
// powersave mode 4 + gain 1x + integration time 800ms = 4800ms refresh time, 8uA, 0.0072 lx/bit

// fast operation
// powersave mode off (PSM_EN=0) + gain 2x + integration time 25ms-100ms, >=48uA

typedef enum {
	VEML_5S_POLLING,
	VEML_100MS_POLLING,
	VEML_25MS_POLLING
} VEML7700_Mode_t;

HAL_StatusTypeDef veml_Setup(I2C_HandleTypeDef i2cHandle, VEML7700_Mode_t Mode);
HAL_StatusTypeDef veml_SetMode(VEML7700_Mode_t Mode);

HAL_StatusTypeDef veml_AutoGain_On();
HAL_StatusTypeDef veml_AutoGain_Off();

HAL_StatusTypeDef veml_PowerSaveMode_On();
HAL_StatusTypeDef veml_PowerSaveMode_Off();

HAL_StatusTypeDef veml_Set_Gain(uint8_t Gain);
HAL_StatusTypeDef veml_Set_IntegrationTime(uint8_t integrationTime);

float veml_Get_Lux();
float veml_Get_White_Lux();

//the following #defines are from Adafruit's library here:
//https://github.com/adafruit/Adafruit_VEML7700/blob/master/Adafruit_VEML7700.h

#define VEML7700_ALS_CONFIG 0x00        ///< Light configuration register
#define VEML7700_ALS_THREHOLD_HIGH 0x01 ///< Light high threshold for irq
#define VEML7700_ALS_THREHOLD_LOW 0x02  ///< Light low threshold for irq
#define VEML7700_ALS_POWER_SAVE 0x03    ///< Power save regiester
#define VEML7700_ALS_DATA 0x04          ///< The light data output
#define VEML7700_WHITE_DATA 0x05        ///< The white light data output
#define VEML7700_INTERRUPTSTATUS 0x06   ///< What IRQ (if any)

#define VEML7700_INTERRUPT_HIGH 0x4000 ///< Interrupt status for high threshold
#define VEML7700_INTERRUPT_LOW 0x8000  ///< Interrupt status for low threshold

#define VEML7700_GAIN_1 0x00   ///< ALS gain 1x
#define VEML7700_GAIN_2 0x01   ///< ALS gain 2x
#define VEML7700_GAIN_1_8 0x02 ///< ALS gain 1/8x
#define VEML7700_GAIN_1_4 0x03 ///< ALS gain 1/4x

#define VEML7700_IT_100MS 0x00 ///< ALS intetgration time 100ms
#define VEML7700_IT_200MS 0x01 ///< ALS intetgration time 200ms
#define VEML7700_IT_400MS 0x02 ///< ALS intetgration time 400ms
#define VEML7700_IT_800MS 0x03 ///< ALS intetgration time 800ms
#define VEML7700_IT_50MS 0x08  ///< ALS intetgration time 50ms
#define VEML7700_IT_25MS 0x0C  ///< ALS intetgration time 25ms

#define VEML7700_PERS_1 0x00 ///< ALS irq persisance 1 sample
#define VEML7700_PERS_2 0x01 ///< ALS irq persisance 2 samples
#define VEML7700_PERS_4 0x02 ///< ALS irq persisance 4 samples
#define VEML7700_PERS_8 0x03 ///< ALS irq persisance 8 samples

#define VEML7700_POWERSAVE_MODE1 0x00 ///< Power saving mode 1
#define VEML7700_POWERSAVE_MODE2 0x01 ///< Power saving mode 2
#define VEML7700_POWERSAVE_MODE3 0x02 ///< Power saving mode 3
#define VEML7700_POWERSAVE_MODE4 0x03 ///< Power saving mode 4



#endif /* INC_VEML7700_H_ */
