/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */
#include "structs.h"
#include "math_helpers.h"
#include "data_tx_arrays.h"
#include <string.h>

volatile CANMotorTelemetry m1_can_telemetry;
volatile CANMotorTelemetry m2_can_telemetry;
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PB8-BOOT0     ------> FDCAN1_RX
    PB9     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = CAN1_RX_Pin|CAN1_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PB8-BOOT0     ------> FDCAN1_RX
    PB9     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, CAN1_RX_Pin|CAN1_TX_Pin);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

// Initializes the CAN reception structure, and sets the rx filter configuration
void can_rx_init(CANRxMessage* msg)
{
	msg->filter.IdType = FDCAN_STANDARD_ID;
	msg->filter.FilterIndex = 0;
	msg->filter.FilterType = FDCAN_FILTER_RANGE;
	msg->filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	msg->filter.FilterID1 = 0x000;
	msg->filter.FilterID2 = 0x7FF;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &(msg->filter)) != HAL_OK)
	{
		Error_Handler();
	}
}

void can_tx_init(CANTxMessage* msg, uint32_t motor_id)
{
	msg->id = motor_id;
	msg->tx_header.IdType = FDCAN_STANDARD_ID;
	msg->tx_header.DataLength = FDCAN_DLC_BYTES_8;
	msg->tx_header.TxFrameType = FDCAN_DATA_FRAME;
	msg->tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	msg->tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	msg->tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	msg->tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	msg->tx_header.MessageMarker = 0;
	msg->tx_header.Identifier = motor_id;
}

// Commands have mode 1 or 3 and use 9 bits for kd. Limits must match the driver's settings
bool can_pack_tx(CANTxMessage* msg, const CANCommandData* command, CANRequestMode mode)
{
	if (msg->tx_header.Identifier > CAN_MOTOR_ID_MAX ||
		(mode != CAN_COMMAND && mode != CAN_COMMAND_CHARACTERIZATION) ||
		!isfinite(command->p_des) || !isfinite(command->v_des) ||
		!isfinite(command->kp) || !isfinite(command->kd) || !isfinite(command->t_ff))
	{
		return false;
	}
	CANCommandFields fields =
	{
		.position = float_to_uint(fminf(fmaxf(P_MIN, command->p_des), P_MAX), P_MIN, P_MAX, 16),
		.velocity = float_to_uint(fminf(fmaxf(V_MIN, command->v_des), V_MAX), V_MIN, V_MAX, 12),
		.kp = float_to_uint(fminf(fmaxf(KP_MIN, command->kp), KP_MAX), KP_MIN, KP_MAX, 12),
		.kd = float_to_uint(fminf(fmaxf(KD_MIN, command->kd), KD_MAX), KD_MIN, KD_MAX, 9),
		.torque = float_to_uint(fminf(fmaxf(I_MIN*KT*GR, command->t_ff), I_MAX*KT*GR), I_MIN*KT*GR, I_MAX*KT*GR, 12)
	};
	can_pack_command_fields(msg->data, &fields, mode);
	msg->tx_header.DataLength = FDCAN_DLC_BYTES_8;
	return true;
}

bool can_pack_query(CANTxMessage* msg, CANRequestMode mode)
{
	if (msg->tx_header.Identifier > CAN_MOTOR_ID_MAX ||
		(mode != CAN_QUERY_STATE && mode != CAN_QUERY_CHARACTERIZATION))
	{
		return false;
	}
	memset(msg->data, 0, sizeof(msg->data));
	msg->data[0] = mode << CAN_MODE_SHIFT;
	msg->tx_header.DataLength = FDCAN_DLC_BYTES_1;
	return true;
}

bool can_pack_special(CANTxMessage* msg, CANSpecialCommand command)
{
	if (msg->tx_header.Identifier > CAN_MOTOR_ID_MAX ||
		command < CAN_SPECIAL_ENABLE || command > CAN_SPECIAL_QUERY)
	{
		return false;
	}
	memset(msg->data, 0xFF, sizeof(msg->data));
	msg->data[7] = command;
	msg->tx_header.DataLength = FDCAN_DLC_BYTES_8;
	return true;
}

bool can_unpack_state(const CANRxMessage* msg, CANStateReply* reply)
{
	if (msg->rx_header.DataLength != FDCAN_DLC_BYTES_7 ||
		(msg->data[0] >> CAN_MODE_SHIFT) != CAN_REPLY_STATE)
	{
		return false;
	}
	int p_int = (msg->data[1] << 8) | msg->data[2];
	int v_int = (msg->data[3] << 4) | (msg->data[4] >> 4);
	int t_int = ((msg->data[4] & 0xF) << 8) | msg->data[5];
	reply->id = msg->data[0] & CAN_MOTOR_ID_MAX;
	reply->position = uint_to_float(p_int, P_MIN, P_MAX, 16);
	reply->velocity = uint_to_float(v_int, V_MIN, V_MAX, 12);
	reply->torque = uint_to_float(t_int, I_MIN*KT*GR, I_MAX*KT*GR, 12);
	reply->bus_voltage = uint_to_float(msg->data[6], 0.0f, 40.0f, 8);
	return true;
}

bool can_unpack_characterization(const CANRxMessage* msg, CANCharacterizationReply* reply)
{
	if (msg->rx_header.DataLength != FDCAN_DLC_BYTES_6 ||
		(msg->data[0] >> CAN_MODE_SHIFT) != CAN_REPLY_CHARACTERIZATION)
	{
		return false;
	}
	uint16_t p_int = (msg->data[1] << 8) | msg->data[2];
	int i_int = (msg->data[3] << 4) | (msg->data[4] >> 4);
	int i_des_int = ((msg->data[4] & 0xF) << 8) | msg->data[5];
	reply->id = msg->data[0] & CAN_MOTOR_ID_MAX;
	reply->position = can_decode_characterization_position(p_int);
	reply->i_q = uint_to_float(i_int, CAN_CHARACTERIZATION_I_MIN, CAN_CHARACTERIZATION_I_MAX, 12);
	reply->i_q_des = uint_to_float(i_des_int, CAN_CHARACTERIZATION_I_MIN, CAN_CHARACTERIZATION_I_MAX, 12);
	return true;
}

// Preserve the main board's existing direction convention and trajectory updates.
static void can_apply_state_reply(const CANStateReply* reply)
{
	float p = reply->position;
	float v = reply->velocity;
	if (reply->id == 1)
	{
		p = -p;
		v = -v;
		memcpy(m1_pos, &p, sizeof(float));
		memcpy(m1_vel, &v, sizeof(float));
		memcpy(m1_ic, &(reply->torque), sizeof(float));
		m1_traj.theta_d_measured = p - m1_traj.theta_current;
		m1_traj.theta_current = p;
	}
	else if (reply->id == 2)
	{
		memcpy(m2_pos, &p, sizeof(float));
		memcpy(m2_vel, &v, sizeof(float));
		memcpy(m2_ic, &(reply->torque), sizeof(float));
		m2_traj.theta_d_measured = p - m2_traj.theta_current;
		m2_traj.theta_current = p;
	}
}

static void can_apply_characterization_reply(const CANCharacterizationReply* reply)
{

	static float last_position = 0;
	float dt = 1.0f/5000.0f;
	float p = reply->position;
	float v = (p - last_position) / dt;
	last_position = p;
	float torque_measured  = reply->i_q * KT * GR;
	float torque_desired = reply->i_q_des *KT * GR;

	if (reply->id == 1)
	{
		p = -p;
		v = -v;
		memcpy(m1_pos, &p, sizeof(float));
		memcpy(m1_vel, &v, sizeof(float));
		memcpy(m1_ic, &torque_measured, sizeof(float));
		memcpy(m1_ic_des, &torque_desired, sizeof(float));
		m1_traj.theta_d_measured = p - m1_traj.theta_current;
		m1_traj.theta_current = p;
	}
	else if (reply->id == 2)
	{
		memcpy(m2_pos, &p, sizeof(float));
		memcpy(m2_vel, &v, sizeof(float));
		memcpy(m2_ic, &torque_measured, sizeof(float));
		memcpy(m2_ic_des, &torque_desired, sizeof(float));
		m2_traj.theta_d_measured = p - m2_traj.theta_current;
		m2_traj.theta_current = p;
	}
}

void can_unpack_rx(const CANRxMessage* msg)
{
	if (msg->rx_header.IdType != FDCAN_STANDARD_ID || msg->rx_header.RxFrameType != FDCAN_DATA_FRAME ||
		msg->rx_header.FDFormat != FDCAN_CLASSIC_CAN || msg->rx_header.DataLength == FDCAN_DLC_BYTES_0 ||
		msg->rx_header.DataLength > FDCAN_DLC_BYTES_8)
	{
		return;
	}
	uint8_t id = msg->data[0] & CAN_MOTOR_ID_MAX;
	volatile CANMotorTelemetry* telemetry;
	if (id == 1){telemetry = &m1_can_telemetry;}
	else if (id == 2){telemetry = &m2_can_telemetry;}
	else{return;}

	switch (msg->data[0] >> CAN_MODE_SHIFT)
	{
		case CAN_REPLY_STATE:
		{
			CANStateReply reply;
			if (!can_unpack_state(msg, &reply)){return;}
			telemetry->state = reply;
			can_apply_state_reply(&reply);
			telemetry->last_reply_mode = CAN_REPLY_STATE;
			telemetry->state_count++;
			data_tx_history_capture();
			break;
		}
		case CAN_REPLY_CHARACTERIZATION:
		{
			CANCharacterizationReply reply;
			if (!can_unpack_characterization(msg, &reply)){return;}
			telemetry->characterization = reply;
			can_apply_characterization_reply(&reply);
			telemetry->last_reply_mode = CAN_REPLY_CHARACTERIZATION;
			telemetry->characterization_count++;
			data_tx_history_capture();
			break;
		}
		default:
			break;
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0){return;}
	while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0))
	{
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &(m_rx.rx_header), m_rx.data) == HAL_OK)
		{
			can_unpack_rx(&m_rx);
		}
	}
}

/* USER CODE END 1 */
