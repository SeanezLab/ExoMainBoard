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

/* USER CODE BEGIN 0 */
#include "fdcan.h"
#include "math_helpers.h"
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
  hfdcan1.Init.NominalTimeSeg1 = 3;
  hfdcan1.Init.NominalTimeSeg2 = 13;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
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

  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

// Helper function to convert from integers to enums for HAL
static uint32_t CAN_BytesToDlc(uint8_t len)
{
  switch (len)
  {
    case 0: return FDCAN_DLC_BYTES_0;
    case 1: return FDCAN_DLC_BYTES_1;
    case 2: return FDCAN_DLC_BYTES_2;
    case 3: return FDCAN_DLC_BYTES_3;
    case 4: return FDCAN_DLC_BYTES_4;
    case 5: return FDCAN_DLC_BYTES_5;
    case 6: return FDCAN_DLC_BYTES_6;
    case 7: return FDCAN_DLC_BYTES_7;
    case 8: return FDCAN_DLC_BYTES_8;
    default: return 0xFFFFFFFFu; // invalid
  }
}

// Initializes the CAN reception structure, and sets the rx filter configuration
void can_rx_init(CANRxMessage* msg)
{
	msg->filter.IdType = FDCAN_STANDARD_ID; // Selects the standard 11 bit ID table
	msg->filter.FilterIndex = 0; // Selects the filter index
	msg->filter.FilterType = FDCAN_FILTER_RANGE; // Accept the ID if it is between Filter ID1 and Filter ID2
	msg->filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // Tells the hardware what to do if the filter matches
	msg->filter.FilterID1 = 0x000; // Lower bound ID range
	msg->filter.FilterID2 = 0x7FF; // Upper bound ID range
	// Apply filter configuration, handle errors if needed
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &(msg->filter)) != HAL_OK)
	{
	  Error_Handler();
	}
}
// Initializes the CAN transmission structure, and sets the header attributes
void can_tx_init(CANTxMessage* msg, uint32_t motor_id)
{
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
// Send
void can_pack_tx(CANTxMessage* msg, float* p_des, float* v_des, float* kp_des, float* kd_des, float* t_ff_des)
{
	// Limit the data to be within bounds
    float p_bnd = fminf(fmaxf(P_MIN, *p_des), P_MAX);
    float v_bnd = fminf(fmaxf(V_MIN, *v_des), V_MAX);
    float kp_bnd = fminf(fmaxf(KP_MIN, *kp_des), KP_MAX);
    float kd_bnd = fminf(fmaxf(KD_MIN, *kd_des), KD_MAX);
    float t_bnd = fminf(fmaxf(I_MIN, *t_ff_des), I_MAX);
	// Compress a float to an uint of given bits
	int p_int = float_to_uint(p_bnd, P_MIN, P_MAX, 16);
	int v_int = float_to_uint(v_bnd, V_MIN, V_MAX, 12);
	int kp_int = float_to_uint(kp_bnd, KP_MIN, KP_MAX, 12);
	int kd_int = float_to_uint(kd_bnd, KP_MIN, KP_MAX, 12);
	int t_int = float_to_uint(t_bnd, I_MIN, I_MAX, 12);
	// Pack the commands into the CAN buffer
	msg->data[0] = p_int>>8;
	msg->data[1] = p_int&0xFF;
	msg->data[2] = v_int>>4;
	msg->data[3] = ((v_int&0xF)<<4)|(kp_int>>8);
	msg->data[4] = kp_int&0xFF;
	msg->data[5] = kd_int>>4;
	msg->data[6] = ((kd_int&0xF)<<4)|(t_int>>8);
	msg->data[7] = t_int&0xff;
}

void unpack_reply(CANRxMessage msg)
{
    /// unpack ints from can buffer ///
    int id = msg.data[0];
    int p_int = (msg.data[1]<<8)|msg.data[2];
    int v_int = (msg.data[3]<<4)|(msg.data[4]>>4);
    int i_int = ((msg.data[4]&0xF)<<8)|msg.data[5];
    /// convert ints to floats ///
    float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    float i = uint_to_float(i_int, -I_MAX, I_MAX, 12);

    //printf("%d  %.3f   %.3f   %.3f\n\r", id, p, v, i);

}

/* USER CODE END 1 */
