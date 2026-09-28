/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
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
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -65.0f
#define V_MAX 65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 100.0f
#define I_MIN -40.0f
#define I_MAX 40.0f
#define KT 0.2454f // Motor Constant
#define GR 6.0 // Gear Ratio

/* Driver command protocol. Change CAN_COMMAND_MODE to select the packet type. */
#define CAN_POSITION_CONTROL_MODE       0U
#define CAN_SLOW_VELOCITY_CONTROL_MODE  1U
#define CAN_COMMAND_MODE                CAN_SLOW_VELOCITY_CONTROL_MODE

#define SLOW_VELOCITY_MIN -0.08726646f // -5 degrees/s, radians/s
#define SLOW_VELOCITY_MAX  0.08726646f //  5 degrees/s, radians/s
#define SLOW_VELOCITY_KP_MIN 0.0f
#define SLOW_VELOCITY_KP_MAX 500.0f
#define SLOW_VELOCITY_KD_MIN 0.0f
#define SLOW_VELOCITY_KD_MAX 100.0f

/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */

typedef struct{
	uint8_t id;
	uint8_t data[6];
	FDCAN_RxHeaderTypeDef rx_header;
	FDCAN_FilterTypeDef filter;
}CANRxMessage;

typedef struct{
	uint8_t id;
	uint8_t data[8];
	FDCAN_TxHeaderTypeDef tx_header;
}CANTxMessage;

void can_rx_init(CANRxMessage* msg);
void can_tx_init(CANTxMessage* msg, uint32_t motor_id);
void can_pack_tx(CANTxMessage* msg, float* p_des, float* v_des, float* kp, float* kd, float* t_ff);
void can_unpack_rx(float* rx_reply);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

