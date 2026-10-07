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
#include "can_protocol.h"

/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */

#define P_MIN -6.28f
#define P_MAX 6.28f
#define V_MIN -65.0f
#define V_MAX 65.0f
#define KP_MIN 0.0f
#define KP_MAX 2000.0f
#define KD_MIN 0.0f
#define KD_MAX 100.0f
#define I_MIN -40.0f
#define I_MAX 40.0f
#define KT 0.2454f // Motor Constant
#define GR 6.0 // Gear Ratio

/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */

typedef struct{
	uint8_t id;
	uint8_t data[64]; // HAL copies by DLC before we can reject an unexpected frame
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
// Latest samples stay in driver coordinates; the existing control state keeps its sign convention.
typedef struct{
	CANStateReply state;
	CANCharacterizationReply characterization;
	CANAbsEncoderReply abs_encoder_reply;
	uint32_t state_count;
	uint32_t characterization_count;
	uint32_t abs_encoder_count;
	uint32_t last_reply_ms; // HAL tick when the latest valid reply was applied
	CANReplyMode last_reply_mode;
}CANMotorTelemetry;

extern volatile CANMotorTelemetry m1_can_telemetry;
extern volatile CANMotorTelemetry m2_can_telemetry;

bool can_pack_tx(CANTxMessage* msg, const CANCommandData* command, CANRequestMode mode);
bool can_pack_query(CANTxMessage* msg, CANRequestMode mode);
bool can_pack_special(CANTxMessage* msg, CANSpecialCommand command);
bool can_unpack_state(const CANRxMessage* msg, CANStateReply* reply);
bool can_unpack_characterization(const CANRxMessage* msg, CANCharacterizationReply* reply);
bool can_unpack_abs_encoder(const CANRxMessage* msg, CANAbsEncoderReply* reply);
void can_unpack_rx(const CANRxMessage* msg);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

