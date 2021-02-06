// Adapted with reference to adafruit library found here:
// https://github.com/adafruit/Adafruit_VEML7700/blob/master/Adafruit_VEML7700.cpp
// by David Ramsay (dramsay9)
// for STM32WB w/FreeRTOS


#include "veml7700.h"
#include "string.h"
#include "stdlib.h"


typedef struct {
  I2C_HandleTypeDef i2cHandle;
  uint8_t          	 powerSaveMode;    //default of VEML7700_POWERSAVE_MODE4 (1/2/3)
  uint8_t			 powerSaveEnable;  //default 0x01 (on)
  uint8_t            gain;            // default to VEML7700_GAIN_2 - highest res (1x, 1/8, 1/4)
  uint8_t            integrationTime; // default to VEML7700_IT_800MS (25/50/100/200/400)
  uint8_t			 autoGain;        //default to 0, not tested
} VEML7700_State_t;

VEML7700_State_t VEML_State;

void I2C_Write_16b(uint8_t addr, uint16_t val)
{
  uint8_t Data[3];
  Data[2] = addr;
  Data[1] = val & 0x00FF;
  Data[0] = (val >> 8);
  HAL_I2C_Master_Transmit(&VEML_State.i2cHandle, VEML_ADDR, Data, 3, HAL_MAX_DELAY);
}



HAL_StatusTypeDef veml_PushState(){ //helper to push power/gain/it to VEML7700

	//main config register; gain and integration time
	HAL_StatusTypeDef resp = HAL_ERROR;
	uint8_t out_data[2] = {0x00, 0x00};
	out_data[1] |= (VEML_State.gain << 3);
	out_data[1] |= ((VEML_State.integrationTime & 0x0C) >> 2);
	out_data[0] |= ((VEML_State.integrationTime & 0x03) << 6);
	out_data[0] |= (VEML7700_PERS_1 << 4);


	//out_data |= (VEML_State.gain << 11);
	//out_data |= (VEML_State.integrationTime << 6);
	//out_data |= (VEML7700_PERS_1 << 4);

	while (resp == HAL_ERROR){
	  resp = HAL_I2C_Mem_Write(&(VEML_State.i2cHandle),
	  		  	  	  	  	   VEML_ADDR,
	  						   VEML7700_ALS_CONFIG, 1,
	  						   out_data, 2,
	  						   HAL_MAX_DELAY);
	  //I2C_Write_16b(VEML7700_ALS_CONFIG, out_data);
	}

	//power save config register
	resp = HAL_ERROR;
	out_data[0] = 0x00;
	out_data[1] = 0x00;

	out_data[0] |= (VEML_State.powerSaveMode << 1);
	out_data[0] |= VEML_State.powerSaveEnable;


	//out_data |= (VEML_State.powerSaveMode << 1);
	//out_data |= VEML_State.powerSaveEnable;

	while (resp == HAL_ERROR){
	  resp = HAL_I2C_Mem_Write(&(VEML_State.i2cHandle),
			  	  	  	  	   VEML_ADDR,
							   VEML7700_ALS_POWER_SAVE, 1,
							   out_data, 2,
							   HAL_MAX_DELAY);
	  //I2C_Write_16b(VEML7700_ALS_CONFIG, out_data);
	}

	return HAL_OK;
}

HAL_StatusTypeDef veml_Setup(I2C_HandleTypeDef i2cHandle, VEML7700_Mode_t Mode){

	VEML_State.i2cHandle = i2cHandle;
	VEML_State.autoGain = 0;

	switch (Mode){
		case VEML_5S_POLLING:
			VEML_State.powerSaveMode = VEML7700_POWERSAVE_MODE4;
			VEML_State.powerSaveEnable = 0x01;
			VEML_State.gain = VEML7700_GAIN_2;
			VEML_State.integrationTime = VEML7700_IT_800MS;
			break;

		case VEML_100MS_POLLING:
			VEML_State.powerSaveMode = VEML7700_POWERSAVE_MODE1;
			VEML_State.powerSaveEnable = 0x00;
			VEML_State.gain = VEML7700_GAIN_2;
			VEML_State.integrationTime = VEML7700_IT_100MS;
			break;

		case VEML_25MS_POLLING:
			VEML_State.powerSaveMode = VEML7700_POWERSAVE_MODE1;
			VEML_State.powerSaveEnable = 0x00;
			VEML_State.gain = VEML7700_GAIN_2;
			VEML_State.integrationTime = VEML7700_IT_25MS;
			break;
	}

	return veml_PushState();

}

//just call init function but without reassigning i2c handle
HAL_StatusTypeDef veml_SetMode(VEML7700_Mode_t Mode){
	return veml_Setup(VEML_State.i2cHandle, Mode);
}

HAL_StatusTypeDef veml_AutoGain_On(){
	VEML_State.autoGain = 0x01;
	return HAL_OK;
}

HAL_StatusTypeDef veml_AutoGain_Off(){
	VEML_State.autoGain = 0x00;
	return HAL_OK;
}


HAL_StatusTypeDef veml_PowerSaveMode_On(){
	VEML_State.powerSaveEnable = 0x01;
	return veml_PushState();
}

HAL_StatusTypeDef veml_PowerSaveMode_Off(){
	VEML_State.powerSaveEnable = 0x00;
	return veml_PushState();
}


HAL_StatusTypeDef veml_Set_Gain(uint8_t Gain){
	VEML_State.gain = Gain;
	return veml_PushState();
}

HAL_StatusTypeDef veml_Set_IntegrationTime(uint8_t integrationTime){
	VEML_State.integrationTime = integrationTime;
	return veml_PushState();
}

void autoGain(uint16_t raw_data){

	if (VEML_State.autoGain){ //if we are autoGaining

		//if raw value is > ~90% of 0xFFFF and we're not at min gain
		if (VEML_State.gain != VEML7700_GAIN_1_8 && raw_data > 0xE665) {
			switch (VEML_State.gain){
				case VEML7700_GAIN_2:
					VEML_State.gain = VEML7700_GAIN_1;
					break;
				case VEML7700_GAIN_1:
					VEML_State.gain = VEML7700_GAIN_1_4;
				    break;
				case VEML7700_GAIN_1_4:
					VEML_State.gain = VEML7700_GAIN_1_8;
					break;
			}
			veml_PushState();
		}

		//if raw value is < ~45% of 0xFFFF and we're not at max gain
		if (VEML_State.gain != VEML7700_GAIN_2 && raw_data < 0x7332) {
			switch (VEML_State.gain){
				case VEML7700_GAIN_1_8:
					VEML_State.gain = VEML7700_GAIN_1_4;
					break;
				case VEML7700_GAIN_1_4:
					VEML_State.gain = VEML7700_GAIN_1;
				    break;
				case VEML7700_GAIN_1:
					VEML_State.gain = VEML7700_GAIN_2;
					break;
			}
			veml_PushState();
		}
	}
}

float veml_norm_data(uint16_t raw_data){

	float lux = (float)raw_data;

	switch (VEML_State.gain){
		case VEML7700_GAIN_2:
			lux /= 2.0;
			break;
		case VEML7700_GAIN_1_4:
		    lux *= 4;
		    break;
		case VEML7700_GAIN_1_8:
		    lux *= 8;
		    break;
	}

	switch (VEML_State.integrationTime){
	  	case VEML7700_IT_25MS:
	  		lux *= 4;
	  		break;
	  	case VEML7700_IT_50MS:
	  		lux *= 2;
	  		break;
	  	case VEML7700_IT_200MS:
	  		lux /= 2.0;
	  		break;
	  	case VEML7700_IT_400MS:
	  		lux /= 4.0;
	  		break;
	  	case VEML7700_IT_800MS:
	  		lux /= 8.0;
	  		break;
	}

	return lux;
}

float veml_Get_Lux(){

	const uint16_t max_tries = 300;
	uint16_t current_tries = 0;

	uint8_t buffer[2];

	HAL_StatusTypeDef resp = HAL_ERROR;

	while (resp == HAL_ERROR && current_tries < max_tries){
	  resp = HAL_I2C_Mem_Read(&(VEML_State.i2cHandle),
			  	  	  	  	  VEML_ADDR,
							  VEML7700_ALS_DATA, 1,
							  buffer, 2,
							  HAL_MAX_DELAY);

	  //resp = HAL_I2C_Master_Receive(&VEML_State.i2cHandle,
	  //	  	  	  VEML_ADDR,&buffer, 2,
	//			  HAL_MAX_DELAY);
	  current_tries++;
	}

	if (current_tries >= max_tries) {
	  return -1;
	}

	uint16_t data = (buffer[1] << 8) | buffer[0];

	autoGain(data);

	float lux = (veml_norm_data(data) * 0.0576);

	if (VEML_State.gain == VEML7700_GAIN_1_8 && VEML_State.integrationTime == VEML7700_IT_25MS){
		lux = 6.0135e-13 * pow(lux, 4) - 9.3924e-9 * pow(lux, 3) + 8.1488e-5 * pow(lux, 2) + 1.0023 * lux;
	}

	return lux;
}

float veml_Get_White_Lux(){

	const uint16_t max_tries = 300;
	uint16_t current_tries = 0;

	uint8_t buffer[2];

	HAL_StatusTypeDef resp = HAL_ERROR;

	while (resp == HAL_ERROR && current_tries < max_tries){
	  resp = HAL_I2C_Mem_Read(&(VEML_State.i2cHandle),
			  	  	  	  	  VEML_ADDR,
							  VEML7700_WHITE_DATA, 1,
							  buffer, 2,
							  HAL_MAX_DELAY);
	  current_tries++;
	}

	if (current_tries >= max_tries) {
	  return -1;
	}

	uint16_t data = (buffer[1] << 8) | buffer[0];

	autoGain(data);

	float lux = (veml_norm_data(data) * 0.0576);

	if (VEML_State.gain == VEML7700_GAIN_1_8 && VEML_State.integrationTime == VEML7700_IT_25MS){
		lux = 6.0135e-13 * pow(lux, 4) - 9.3924e-9 * pow(lux, 3) + 8.1488e-5 * pow(lux, 2) + 1.0023 * lux;
	}

	return lux;

}
