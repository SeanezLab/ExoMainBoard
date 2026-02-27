/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "structs.h"
/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN Private defines */
#define FOLLOWER_7BIT_ADDR 0X12
#define FOLLOWER_ADDR_HAL (FOLLOWER_7BIT_ADDR << 1)
#define I2C_RX_BYTES 100

extern uint8_t i2c_rx_buf[];
extern volatile uint8_t i2c_rx_done;
extern volatile uint32_t i2c_rx_err;
extern volatile HAL_StatusTypeDef i2c_rx_status;

/* USER CODE END Private defines */

void MX_I2C2_Init(void);

/* USER CODE BEGIN Prototypes */
uint8_t check_i2c_dma(void);
HAL_StatusTypeDef start_i2c_rx_dma(uint16_t nbytes);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

