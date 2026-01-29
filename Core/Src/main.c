/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>


#include "structs.h"
#include "data_tx_arrays.h"
#include "crc.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Define interrupt flags
volatile bool got_bt_msg = false;
volatile uint16_t bt_msg_size = 0;

// Define Bluetooth buffer structures
rdg_buf_struct* bt_dma_reader;
uint8_t bt_rx_dma_buffer[BT_RX_DMA_SIZE]; // (Written to by DMA)

// Motor CAN Structs
// Motor 1 (Proximal Joint)
CANTxMessage m1_tx;
// Motor 2 (Distal Joint)
CANTxMessage m2_tx;
// Once receiver for all messages.
CANRxMessage m_rx;

// Load Cell Structs




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	// Initialize the buffers with a given size
	bt_dma_reader = rdg_buf_init(BT_RX_DMA_SIZE);
	volatile uint16_t bt_idx = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */

  // --- Enable Receiver Timeout (RTO) on USART1 ---
  // Choose a timeout value in bit-times / baud clocks.
  // For a start, pick something like "20 char times".
  // Exact scaling depends on the reference manual, but this shape is right:
  huart1.Instance->RTOR = 200;                 // timeout value (tune later)
  SET_BIT(huart1.Instance->CR2, USART_CR2_RTOEN);   // enable RTO
  SET_BIT(huart1.Instance->CR1, USART_CR1_RTOIE);   // enable RTO interrupt


  // Start the UART of the BT RX line
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, bt_rx_dma_buffer, BT_RX_DMA_SIZE);
  // Turn OFF DMA half-transfer + transfer-complete interrupts
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_TC);

  // Try receiver timeout interrupt on huart1
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RTO);

  // Initialize CAN communication structures
  // Motor 1
  can_tx_init(&m1_tx, 1);
  // Motor 2
  can_tx_init(&m2_tx, 2);
  // One receiver for all messages
  can_rx_init(&m_rx);

  // Start the CAN bus
  HAL_FDCAN_Start(&hfdcan1);

  // Activate notifications
  HAL_FDCAN_ActivateNotification(
      &hfdcan1,
      FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
      0
  );

  FDCAN_ProtocolStatusTypeDef ps;
  m1_tx.data[0] = 0xff;
  m1_tx.data[1] = 0xff;
  m1_tx.data[2] = 0xff;
  m1_tx.data[3] = 0xff;
  m1_tx.data[7] = 0xff;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // First query all our sensors (Motor drivers, Vibrotactile Stimulator, Load Cells, etc.)

	  if (got_bt_msg == true)
	  {
		  dma_to_rdg_buf(bt_dma_reader, bt_rx_dma_buffer, bt_msg_size);
		  crc_uart_rcv_data(bt_dma_reader, bt_msg_size);
		  flush_buffer(bt_dma_reader);
		  got_bt_msg = false;
	  }

	  // Transmit states

	  compile_data_sources(21,
			  vibro_z_axis, vibro_gpio, vibro_fft, vibro_state,
			  exo_busy, exo_fsm, exo_debug,
			  m1_pos, m1_vel, m1_accel, m1_ic, m1_tau, m1_kd, m1_ki,
			  m2_pos, m2_vel, m2_accel, m2_ic, m2_tau, m2_kd, m2_ki);

	  crc_uart_send_data(compiled_payload, &huart1);

	  HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &(m1_tx.tx_header), m1_tx.data);
	  HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
	  if (st != HAL_OK)
	  {
		  uint32_t free = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
		  HAL_GPIO_WritePin(Debug_GPIO_Port, Debug_Pin, GPIO_PIN_SET);
		  Error_Handler();
	  }
	  HAL_Delay(1000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// Interrupt handler for the bt_rx_dma_buffer
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == USART1) {
        // Drop a flag to print that data
//    	got_bt_msg = true;
//    	bt_msg_size = size;
        }

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
