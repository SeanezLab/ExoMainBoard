/*
 * data_arrays.h
 *
 *  Created on: Jan 7, 2026
 *      Author: k.rodolfo
 */

#ifndef INC_DATA_TX_ARRAYS_H_
#define INC_DATA_TX_ARRAYS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdint.h>

// Existing UART payload size. Each queued entry becomes one ordinary CRC packet.
#define DATA_TX_SAMPLE_BYTES 40U
#define DATA_TX_HISTORY_CAPACITY 256U

typedef struct{
	uint8_t payload[DATA_TX_SAMPLE_BYTES];
}DataTxSample;

typedef struct{
	DataTxSample samples[DATA_TX_HISTORY_CAPACITY];
	volatile uint16_t write_index;
	volatile uint16_t read_index;
	volatile uint16_t count;
	volatile uint16_t high_water_mark;
	volatile uint32_t captured; // Capture attempts, including dropped samples
	volatile uint32_t dropped; // Queue full: keep old samples, drop the new one
	volatile uint32_t submitted; // Samples accepted by UART DMA
	volatile uint32_t tx_failures; // DMA failed to start; samples remain queued
}DataTxHistory;

extern DataTxHistory data_tx_history;
void data_tx_history_capture(void); // Save the current arrays after a valid CAN reply
void data_tx_history_drain(void); // Start a non-blocking batch from the com loop

// Holds the data array variable for better readability


// Exoskeleton main control board data
extern uint8_t exo_busy[];
extern uint8_t exo_fsm[];
extern uint8_t exo_debug[];
// Motor drive 1 data (knee)
extern uint8_t m1_pos[];
extern uint8_t m1_des[];
extern uint8_t m1_vel[];
extern uint8_t m1_accel[];
extern uint8_t m1_ic[];
extern uint8_t m1_tau[];
extern uint8_t m1_kp[];
extern uint8_t m1_kd[];
extern uint8_t m1_mode[];
extern uint8_t m1_traj_status[];
// Motor drive 2 data (ankle)
extern uint8_t m2_pos[];
extern uint8_t m2_des[];
extern uint8_t m2_vel[];
extern uint8_t m2_accel[];
extern uint8_t m2_ic[];
extern uint8_t m2_tau[];
extern uint8_t m2_kp[];
extern uint8_t m2_kd[];
extern uint8_t m2_mode[];
extern uint8_t m2_traj_status[];
// Debugging transmits
extern uint8_t frame[];
extern uint8_t debug[];
extern uint8_t tx_dropped[];
extern uint8_t high_water_mark[];
extern uint8_t failures[];




#ifdef __cplusplus
}
#endif


#endif /* INC_DATA_TX_ARRAYS_H_ */
