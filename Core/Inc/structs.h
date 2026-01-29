/*
 * structs.h
 *
 *  Created on: Dec 10, 2025
 *      Author: k.rodolfo
 */

#ifndef INC_STRUCTS_H_
#define INC_STRUCTS_H_

#include "circular_reading_buffer.h"
#include "ring_buffer.h"
#include "fdcan.h"
#include "cmd_array.h"

// Global Structs ///
// Bluetooth UART reading buffer
extern rdg_buf_struct* bt_dma_reader;

// Motor CAN Structs
// Motor 1 (Proximal Joint)
extern CANTxMessage m1_tx;
// Motor 2 (Distal Joint)
extern CANTxMessage m2_tx;
// One receiver for all messages.
extern CANRxMessage m_rx;

// Command Handling Structs
extern MotorCommand m1_cmd;
extern MotorCommand m2_cmd;
extern VibroCommand vibro_cmd;


#endif /* INC_STRUCTS_H_ */
