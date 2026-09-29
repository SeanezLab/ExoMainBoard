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
#include <stdbool.h>

#define DATA_TX_HISTORY_CAPACITY 256U
#define DATA_TX_BATCH_CAPACITY 32U // Samples per buffer; two buffers alternate on UART

// Layout arrays and source data must outlive the packet. Allocate once at startup.
typedef struct{
	uint16_t field_count;
	uint16_t payload_bytes;
	uint16_t packet_bytes;
	const uint16_t* length_key;
	const uint8_t* const* data_sources;
	uint8_t* tx_buffer; // Separate DMA staging, owned by the same allocation
	uint8_t compiled_payload[];
}TxPacket;

typedef struct{
	uint8_t* samples;
	uint16_t sample_bytes;
	volatile uint16_t write_index;
	volatile uint16_t read_index;
	volatile uint16_t count; // Samples still in history, excluding prepared/transmitting batches
	volatile uint16_t high_water_mark;
	volatile uint32_t captured; // Capture attempts, including dropped samples
	volatile uint32_t dropped; // Queue full or invalid packet: discard the new sample
	volatile uint32_t submitted; // Samples accepted by UART DMA
	volatile uint32_t tx_failures; // DMA failed to start; the prepared batch is kept for retry
}DataTxHistory;

extern DataTxHistory data_tx_history;
TxPacket* tx_packet_init(uint16_t field_count, const uint16_t* length_key,
	const uint8_t* const* data_sources);
bool data_tx_arrays_init(void); // Initialize the telemetry packet and its history before CAN starts
void data_tx_history_capture(void); // Save the current arrays after a valid CAN reply
void data_tx_history_drain(void); // Main-loop only: submit ready data and prepare one spare batch

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
extern uint8_t m1_ic_des[];
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
extern uint8_t m2_ic_des[];
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
