//written by David Ramsay for STM32


#ifndef _iqs263_H_
#define _iqs263_H_

#include "stdio.h"
#include "stm32wbxx_hal.h"

// EDIT THIS TO BE THE I2C INTERFACE YOU'RE USING
extern I2C_HandleTypeDef hi2c1;
#define IQS_I2C_PORT hi2c1
#define IQS_ADDR (0x44 << 1)

#define ANGLE_OFFSET 353

#define IQS_TOUCH_THRESH 5 //touch sensitivity threshold (10/20), larger is more

static const int MAP_ANGLE_REAL[] = {0, 45, 90, 135, 180, 225, 270, 315, 360};
//static const int MAP_ANGLE_MEASURED[] = {0, 40, 103, 136, 179, 233, 272, 312, 360};
static const int MAP_ANGLE_MEASURED[] = {0, 45, 90, 135, 180, 225, 270, 315, 360};


HAL_StatusTypeDef setup_iqs263();
int iqs263_get_angle();
int iqs263_get_min();
int iqs263_get_min_if_pressed();

void iqs263_poll_raw(int* coords);
HAL_StatusTypeDef iqs263_write(uint8_t addr, uint8_t* cmd, uint8_t size);
HAL_StatusTypeDef iqs263_read(uint8_t addr, uint8_t* buf, uint8_t size);

#endif
