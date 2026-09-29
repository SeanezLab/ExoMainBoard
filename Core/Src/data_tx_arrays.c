/*
 * data_arrays.c
 *
 *  Created on: Jan 7, 2026
 *      Author: k.rodolfo
 */


#include "data_tx_arrays.h"
#include "crc.h"
#include "usart.h"

// Holds the data array variable for better readability



// Exoskeleton main control board data
uint8_t exo_busy[1] = {0};
uint8_t exo_fsm[1] = {0};
uint8_t exo_debug[1] = {0};
// Motor drive 1 data (knee)
uint8_t m1_pos[4] = {0};
uint8_t m1_des[4] = {0};
uint8_t m1_vel[4] = {0};
uint8_t m1_accel[4] = {0};
uint8_t m1_ic[4] = {0};
uint8_t m1_tau[4] = {0};
uint8_t m1_kp[4] = {0};
uint8_t m1_kd[4] = {0};
uint8_t m1_mode[1] = {0};
uint8_t m1_traj_status[1] = {0};
// Motor drive 2 data (ankle)
uint8_t m2_pos[4] = {0};
uint8_t m2_des[4] = {0};
uint8_t m2_vel[4] = {0};
uint8_t m2_accel[4] = {0};
uint8_t m2_ic[4] = {0};
uint8_t m2_tau[4] = {0};
uint8_t m2_kp[4] = {0};
uint8_t m2_kd[4] = {0};
uint8_t m2_mode[1] = {0};
uint8_t m2_traj_status[1] = {0};
// Debugging transmits
uint8_t frame[1] = {0};
uint8_t debug[10] = {0};
uint8_t tx_dropped[4] = {0};
uint8_t high_water_mark[4] = {0};
uint8_t failures[4] = {0};

DataTxHistory data_tx_history = {0};

typedef struct{
	uint8_t packets[DATA_TX_HISTORY_CAPACITY * PKT_BYTES];
	uint16_t count;
}DataTxBatch;

// DMA owns this buffer until the selected UART's TX-complete flag is set.
static DataTxBatch data_tx_batch;

_Static_assert(DATA_TX_HISTORY_CAPACITY > 0 &&
	DATA_TX_HISTORY_CAPACITY * PKT_BYTES <= UINT16_MAX, "History batch must fit one UART DMA transfer");

void data_tx_history_capture(void)
{
	uint32_t irq_state = __get_PRIMASK();
	__disable_irq();
	uint8_t sequence = (uint8_t)data_tx_history.captured++;
	if (data_tx_history.count == DATA_TX_HISTORY_CAPACITY)
	{
		data_tx_history.dropped++;
		__set_PRIMASK(irq_state);
		return;
	}

	// Capture the same fields and direction conventions as the existing stream.
	exo_fsm[0] = state.state;
	frame[0] = sequence;
	compile_data_sources(19,
		exo_busy, exo_fsm, exo_debug,
		m1_pos, m1_des, m1_vel, m1_ic, m1_mode, m1_traj_status,
		m2_pos, m2_des, m2_vel, m2_ic, m2_mode, m2_traj_status,
		frame);
	memcpy(data_tx_history.samples[data_tx_history.write_index].payload,
		compiled_payload, DATA_TX_SAMPLE_BYTES);
	data_tx_history.write_index = (data_tx_history.write_index + 1U) % DATA_TX_HISTORY_CAPACITY;
	data_tx_history.count++;
	if (data_tx_history.count > data_tx_history.high_water_mark)
	{
		data_tx_history.high_water_mark = data_tx_history.count;
	}
	__set_PRIMASK(irq_state);
}

void data_tx_history_drain(void)
{
	// Do not overwrite the batch while DMA is still reading it.
	if ((UART_PORT == 1 && huart1_tx_complete == 0) ||
		(UART_PORT == 2 && huart2_tx_complete == 0))
	{
		return;
	}
	data_tx_batch.count = data_tx_history.count;
	if (data_tx_batch.count == 0){return;}

	uint16_t read_index = data_tx_history.read_index;
	for (uint16_t i = 0; i < data_tx_batch.count; i++)
	{
		// The producer never overwrites unread entries. New arrivals wait for the next batch.
		crc_pack_data(&data_tx_batch.packets[i * PKT_BYTES],
			data_tx_history.samples[read_index].payload);
		read_index = (read_index + 1U) % DATA_TX_HISTORY_CAPACITY;
	}

	// Keep the queue's count update atomic with respect to CAN reception.
	uint32_t irq_state = __get_PRIMASK();
	__disable_irq();
	bool started = false;
	if (UART_PORT == 1)
	{
		started = huart1_try_send(data_tx_batch.packets, data_tx_batch.count * PKT_BYTES);
	}
	else if (UART_PORT == 2)
	{
		started = huart2_try_send(data_tx_batch.packets, data_tx_batch.count * PKT_BYTES);
	}
	if (started)
	{
		data_tx_history.read_index = read_index;
		data_tx_history.count -= data_tx_batch.count;
		data_tx_history.submitted += data_tx_batch.count;
	}
	else
	{
		data_tx_history.tx_failures++;
	}
	__set_PRIMASK(irq_state);
}


