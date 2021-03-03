//written by David Ramsay for STM32

#include "iqs263.h"
#include <math.h>
//#include "stm32wbxx_hal_i2c.h"
//#include "stm32wbxx_hal.h"





int _angle_correction(int measured_angle){
  //given a measured angle, get back a warped angle interpolated from measured corrections
  int i=0;
  while(measured_angle > MAP_ANGLE_MEASURED[i+1]) i++;

  int low_m = MAP_ANGLE_MEASURED[i];
  int high_m = MAP_ANGLE_MEASURED[i+1];
  int low_r = MAP_ANGLE_REAL[i];
  int high_r = MAP_ANGLE_REAL[i+1];

  int scaled = (int)((high_r-low_r)*(measured_angle-low_m)/(float)(high_m-low_m));
  return (low_r + scaled);
}


int _get_pad_angle(int p1, int p2){
    //return angle from 0 to 120 degrees between two pads given their two values
    if (p1 == 0) return 120;
    if (p2 == 0) return 0;
    return round(((float)(p2)/(float)(p1+p2))*120.0);
}


int _get_angle(int* c) {
  //c[0] is proximity
  //60 min on clock, so 6 deg resolution
  //3 pads.  take top 2 values, map linearly to 120 degree slice.
  int angle = ANGLE_OFFSET;
  int warped_angle;

  if ( ((c[1]==0) + (c[2]==0) + (c[3]==0)) > 1){//if more than one value is zero
	  if (c[1]) angle += 0;
	  else if (c[2]) angle += 120;
	  else if (c[3]) angle += 240;
	  else return -1;
  }

  else if (c[1] > c[3] && c[2] > c[3]) { //first third, between 1 and 2
    int pos1 = c[1]-c[3];
    int pos2 = c[2]-c[3];
    angle += _get_pad_angle(pos1, pos2);
  }

  else if (c[2] > c[1] && c[3] > c[1]) { //second third, between 2 and 3
    int pos1 = c[2]-c[1];
    int pos2 = c[3]-c[1];
    angle += _get_pad_angle(pos1, pos2) + 120;
  }

  else { //third third, between 3 and 1
    int pos1 = c[3]-c[2];
    int pos2 = c[1]-c[2];
    angle += _get_pad_angle(pos1, pos2) + 240;
  }

  angle %= 360;
  warped_angle = _angle_correction(angle);

  return warped_angle;
}


int _get_min(int* c) {
	int angle = _get_angle(c);
	if (angle == -1) return -1;
	return angle/6;
}


int _get_min_if_pressed(int* c){
	//return -1 if not pressed, otherwise return a logical minute value
	if (c[0] <= IQS_TOUCH_THRESH) return -1;
	else return _get_min(c);
}


void iqs263_poll_raw(int* coords){

  //uint16_t coords[4];

  uint8_t coords_raw[8];

  iqs263_read(0x06, coords_raw, 8);

  for (int i=0; i<4; i++){
	  coords[i] = (coords_raw[2*i+1] << 8) | (coords_raw[2*i] & 0xFF);
  }
  //c[0] is proximity

}


HAL_StatusTypeDef setup_iqs263() {

	uint8_t c;
	HAL_StatusTypeDef resp = HAL_ERROR;
	uint8_t out_data[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

	//check product num
	while (resp == HAL_ERROR){
		resp = HAL_I2C_Mem_Read(&IQS_I2C_PORT, IQS_ADDR, 0x00, sizeof(uint8_t), &c, sizeof(uint8_t), HAL_MAX_DELAY);
	}

	if (c != 0x3C) {
	  //ERROR - should read product code 0x3C
	  return HAL_ERROR;
	}

	HAL_Delay(100);


	//now write and read 0x0E to address 0x0D
	out_data[0] = 0x0E;

	resp = HAL_ERROR;
	while (resp == HAL_ERROR){
		  resp = HAL_I2C_Mem_Write(&IQS_I2C_PORT, IQS_ADDR, 0x0D, 1, out_data, 1, HAL_MAX_DELAY);
	}
	HAL_Delay(100);

	/*
	resp = HAL_ERROR;
	while (resp == HAL_ERROR){
		resp = HAL_I2C_Mem_Read(&IQS_I2C_PORT, IQS_ADDR, 0x0D, 1, &c, 1, HAL_MAX_DELAY);
	}

	HAL_Delay(50);
	*/

	out_data[0]=0x00;
	//time average filter coef in bits 5:4 (00 is slowest, 11 is fastest), counts filtering for noise in bits 1:0 (00 is no filter, 11 is slowest).
	out_data[1]=0b00001001;
	//lets go into Low Power mode if we have a prolonged state.	Wake on  movement on CH3
	out_data[2]=0x00;
	//out_data[2]=0b10001000;
	//lets disable turbo and only sample at fixed period 40Hz (given 2MHz clock)
	//out_data[3]=0x00;
	out_data[3]=0b00000110;
	out_data[4]=0x00;

	resp = HAL_ERROR;
	while (resp == HAL_ERROR){
		  resp = HAL_I2C_Mem_Write(&IQS_I2C_PORT, IQS_ADDR, 0x09, 1, out_data, 5, HAL_MAX_DELAY);
	}

	/*
	uint8_t readback[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
	resp = HAL_ERROR;
	while (resp == HAL_ERROR){
	    resp = HAL_I2C_Mem_Read(&IQS_I2C_PORT, IQS_ADDR, 0x09, 1, readback, 5, HAL_MAX_DELAY);
	}
	*/

    return HAL_OK;
}


HAL_StatusTypeDef iqs263_write(uint8_t addr, uint8_t* cmd, uint8_t size) {


  const uint16_t max_tries = 300;
  uint16_t current_tries = 0;

  HAL_StatusTypeDef resp = HAL_ERROR;

  while (resp == HAL_ERROR && current_tries < max_tries){
    resp = HAL_I2C_Mem_Write(&IQS_I2C_PORT, IQS_ADDR, addr, 1, cmd, size, HAL_MAX_DELAY);
    current_tries++;
  }

  if (current_tries >= max_tries) {
	  return HAL_ERROR;
  }

  return HAL_OK;

}

HAL_StatusTypeDef iqs263_read(uint8_t addr, uint8_t* buf, uint8_t size) {


  const uint16_t max_tries = 300;
  uint16_t current_tries = 0;

  HAL_StatusTypeDef resp = HAL_ERROR;

  while (resp == HAL_ERROR && current_tries < max_tries){
    resp = HAL_I2C_Mem_Read(&IQS_I2C_PORT, IQS_ADDR, addr, 1, buf, size, HAL_MAX_DELAY);
    current_tries++;
  }

  if (current_tries >= max_tries) {
	  return HAL_ERROR;
  }

  return HAL_OK;

}


int iqs263_get_angle(){
	int* coords[4] = {0x0000, 0x0000, 0x0000, 0x0000};
	iqs263_poll_raw(coords);
	return _get_angle(coords);

}


int iqs263_get_min(){
	int* coords[4] = {0x0000, 0x0000, 0x0000, 0x0000};
	iqs263_poll_raw(coords);
	return _get_min(coords);
}


int iqs263_get_min_if_pressed(){
	int* coords[4] = {0x0000, 0x0000, 0x0000, 0x0000};
	iqs263_poll_raw(coords);
	return _get_min_if_pressed(coords);
}

