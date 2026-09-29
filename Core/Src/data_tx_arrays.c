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
uint8_t high_water_mark[2] = {0};
uint8_t failures[4] = {0};

DataTxHistory data_tx_history = {0};


typedef struct{
	uint8_t* packets;
	uint16_t packet_bytes;
	uint16_t count;
}DataTxBatch;

// DMA owns this buffer until the selected UART's TX-complete flag is set.
static DataTxBatch data_tx_batch;

_Static_assert(DATA_TX_HISTORY_CAPACITY > 0 && DATA_TX_HISTORY_CAPACITY <= UINT16_MAX,
	"History capacity must fit its indices");

TxPacket* tx_packet_init(uint16_t field_count, const uint16_t* length_key,
	const uint8_t* const* data_sources)
{
	if (field_count == 0 || length_key == NULL || data_sources == NULL)
	{
		return NULL;
	}
	uint32_t payload_bytes = 0;
	for (uint16_t i = 0; i < field_count; i++)
	{
		if (length_key[i] == 0 || data_sources[i] == NULL)
		{
			return NULL;
		}
		payload_bytes += length_key[i];
		if (payload_bytes + CRC_PACKET_OVERHEAD_BYTES > UINT16_MAX)
		{
			return NULL;
		}
	}

	uint16_t packet_bytes = payload_bytes + CRC_PACKET_OVERHEAD_BYTES;
	TxPacket* packet = malloc(sizeof(TxPacket) + payload_bytes + packet_bytes);
	if (packet == NULL)
	{
		return NULL;
	}
	packet->field_count = field_count;
	packet->payload_bytes = payload_bytes;
	packet->packet_bytes = packet_bytes;
	packet->length_key = length_key;
	packet->data_sources = data_sources;
	packet->tx_buffer = packet->compiled_payload + payload_bytes;
	memset(packet->compiled_payload, 0, payload_bytes + packet_bytes);
	return packet;
}

bool data_tx_arrays_init(void)
{
	if (data_tx_packet != NULL){return true;}

	// Field order is the existing 40-byte telemetry payload. Keep these tables together.
	static const uint8_t* const data_sources[] = {
		exo_busy, exo_fsm, exo_debug,
		m1_pos, m1_des, m1_vel, m1_ic, m1_mode, m1_traj_status,
		m2_pos, m2_des, m2_vel, m2_ic, m2_mode, m2_traj_status,
		frame, tx_dropped, high_water_mark, failures
	};
	// Create the length key
	static const uint16_t length_key[] = {
		1, 1, 1,
		4, 4, 4, 4, 1, 1,
		4, 4, 4, 4, 1, 1,
		1, 4, 2, 4
	};
	// Check to ensure that the data packet is valid at compile time.
	_Static_assert(sizeof(data_sources) / sizeof(data_sources[0]) ==
		sizeof(length_key) / sizeof(length_key[0]), "Each source must have a length");
	// Initialize the transmission packet
	TxPacket* packet = tx_packet_init(sizeof(length_key) / sizeof(length_key[0]),
		length_key, data_sources);
	if (packet == NULL)// If we cannot create a packet, return
	{
		return false;
	}
	// Initialize the data history
	uint32_t batch_bytes = (uint32_t)DATA_TX_HISTORY_CAPACITY * packet->packet_bytes; // Gets the number of bytes in a batch based on how large a packet is
	if (batch_bytes > UINT16_MAX) // The UART transmit take a uint16_t byte count, confirms that we can send this w/out wrapping
	{
		free(packet);
		return false;
	}
	// Allocates the byte arrays that will hold the history.
	uint8_t* samples = malloc((size_t)DATA_TX_HISTORY_CAPACITY * packet->payload_bytes); //Holds number of payload bytes
	uint8_t* packets = malloc(batch_bytes);// Holds all the total bytes for the packets
	if (samples == NULL || packets == NULL)
	{
		free(samples);
		free(packets);
		free(packet);
		return false;
	}
	data_tx_history.samples = samples;
	data_tx_history.sample_bytes = packet->payload_bytes;
	data_tx_batch.packets = packets;
	data_tx_batch.packet_bytes = packet->packet_bytes;
	data_tx_packet = packet;
	return true;
}

// Captures the state of the Exo. Called every reception from the Motor Drivers. I don't like that this is tied to driver communication. Change to calling on a clock.
// Also perhaps it should be called State Capture.
void data_tx_history_capture(void)
{
	if (data_tx_packet == NULL)
	{
		return;
	}
	// Prevent interuupts during capture
	uint32_t irq_state = __get_PRIMASK();
	__disable_irq();
	// Recalls the sequence of packets captures
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
	uint32_t dropped_count = data_tx_history.dropped;
	uint16_t high_water_count = data_tx_history.high_water_mark;
	uint32_t failure_count = data_tx_history.tx_failures;
	memcpy(tx_dropped, &dropped_count, sizeof(dropped_count));
	memcpy(high_water_mark, &high_water_count, sizeof(high_water_count));
	memcpy(failures, &failure_count, sizeof(failure_count));

	// Early return if the payload and history byte amounts aren't consistent. A bit redundant but in theory you could have changed payload bytes after packet init.
	if (data_tx_packet->payload_bytes != data_tx_history.sample_bytes ||
		!compile_data_sources(data_tx_packet))// Also returns on a failure to compile the packets
	{
		data_tx_history.dropped++;
		__set_PRIMASK(irq_state);
		return;
	}
	// Copies all the data to the history buffer (indexed by the write idx)
	memcpy(&data_tx_history.samples[data_tx_history.write_index * data_tx_history.sample_bytes],
		data_tx_packet->compiled_payload, data_tx_history.sample_bytes);
	// Keeps track of the write location
	data_tx_history.write_index = (data_tx_history.write_index + 1U) % DATA_TX_HISTORY_CAPACITY;
	data_tx_history.count++; // Keeps track of the number of packets in the history
	// Keeps track of the highest the buffer gets for tuning writes
	if (data_tx_history.count > data_tx_history.high_water_mark)
	{
		data_tx_history.high_water_mark = data_tx_history.count;
	}
	__set_PRIMASK(irq_state);
}

void data_tx_history_drain(void)
{
	if (data_tx_packet == NULL){return;}
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
		if (!crc_pack_data(&data_tx_batch.packets[i * data_tx_batch.packet_bytes],//This is the area of the packet array to place this packet (destination)
			data_tx_batch.packet_bytes, // This is how many bytes to place into this area of memory (destination size)
			&data_tx_history.samples[read_index * data_tx_history.sample_bytes], // This is where to read the data from (source)
			data_tx_history.sample_bytes))// This is how much data to read from the source (source size)
		{
			return;
		}
		read_index = (read_index + 1U) % DATA_TX_HISTORY_CAPACITY; // Updates the read index
	}

	// Keep the queue's count update atomic with respect to CAN reception.
	// Disables interrupts while starting the DMA transfer
	uint32_t irq_state = __get_PRIMASK();
	__disable_irq();
	bool started = false;
	if (UART_PORT == 1)
	{
		started = huart1_try_send(data_tx_batch.packets, data_tx_batch.count * data_tx_batch.packet_bytes);
	}
	else if (UART_PORT == 2)
	{
		started = huart2_try_send(data_tx_batch.packets, data_tx_batch.count * data_tx_batch.packet_bytes);
	}
	if (started) // If data is successfully started, we decrement our history by the packets we sent
	{
		data_tx_history.read_index = read_index;
		data_tx_history.count -= data_tx_batch.count;
		data_tx_history.submitted += data_tx_batch.count;
	}
	else
	{
		data_tx_history.tx_failures++; // If we are unable to send, we note it as a failure and do not update the history so we can retry.
	}
	__set_PRIMASK(irq_state);
}


