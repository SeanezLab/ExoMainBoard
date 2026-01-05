/*
 * crc.c
 *
 *  Created on: Dec 29, 2025
 *      Author: rdkee
 */


#include "crc.h"


// Calculate the total amount of dynamic packet constants
uint16_t payload_bytes = calc_payload_bytes();
uint16_t pkt_bytes = (HEADER_BYTES + payload_bytes + CRC_BYTES);


// helper function to calculate payload byte length

static uint16_t calc_payload_bytes()
{
	for (uint8_t i = 0; i < payload_data_fields; i++)
	{
		pkt_bytes += payload_length_key[i];
	}
}

// helper function for implementing CRC packets
static uint16_t crc16_ccitt(const uint8_t* buf, uint16_t len)
{
	uint16_t crc = 0xFFFF;
	for (uint16_t i = 0; i < len; i++)
	{
		crc ^= (uint16_t)buf[i] << 8;
		for (uint8_t j = 0; j < 8; j++)
		{
			crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
		}
	}
	return crc;
}


// Package data and send over UART
// Packet the predefined data payload (Non-generic)
static void crc_uart_send_data(const float* src,
		uint16_t* payload_length_key,
		uint16_t payload_data_fields,
		UART_HandleTypeDef* huart)
{


	uint8_t pkt[PKT_BYTES];

	// 1. Header (preamble)
	pkt[0] = 0x55;
	pkt[1] = 0xAA;

    // 2. 2-byte payload length (little endian)
    pkt[2] = (uint8_t)(PAYLOAD_BYTES & 0xFF);        // LSB
    pkt[3] = (uint8_t)((PAYLOAD_BYTES >> 8) & 0xFF); // MSB

    // 3. Build the payload: accel_data (float), trig_data (uint8_t), state_data, fft_data
    size_t write_index = 4; // Create a write index, starting at 4
    // Check the size of the input data to ensure correctness
    if (accel_size + trig_size + state_size + fft_size == PAYLOAD_BYTES)
    {
		memcpy(&pkt[write_index], accel_data, accel_size);
		write_index += accel_size;

		memcpy(&pkt[write_index], trig_data, trig_size);
		write_index += trig_size;

		memcpy(&pkt[write_index], state_data, state_size);
		write_index += state_size;

		memcpy(&pkt[write_index], fft_data, fft_size);
		write_index += fft_size;
    } else
    {
    	uint8_t error_array[PAYLOAD_BYTES] = {0};
    	memcpy(&pkt[write_index], error_array, (size_t)PAYLOAD_BYTES);
    }


    // 4. CRC over length + payload
    //    Starts from pkt[2], length = LEN_FIELD_BYTES + PAYLOAD_BYTES
    uint16_t crc = crc16_ccitt(&pkt[2], LEN_FIELD_BYTES + PAYLOAD_BYTES);
    pkt[4 + PAYLOAD_BYTES]     = (uint8_t)(crc & 0xFF);
    pkt[4 + PAYLOAD_BYTES + 1] = (uint8_t)(crc >> 8);

    // 5. Transmit over UART
    HAL_UART_Transmit(&huart2, pkt, PKT_BYTES, HAL_MAX_DELAY);
}
