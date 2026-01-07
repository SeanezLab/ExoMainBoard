/*
 * crc.c
 *
 *  Created on: Dec 29, 2025
 *      Author: rdkee
 */


#include "crc.h"

// Fill in Below for each new protocol ///////////////////////////////////////////////////////////////////////
char *payload_entries[] = {"vibro_z_axis", "vibro_gpio","vibro_fft","vibro_state",\
						"exo_busy","exo_fsm","exp_debug",\
						"m1_pos","m1_vel","m1_accel","m1_ic","m1_tau","m1_kd","m1_ki",\
						"m2_pos","m2_vel","m2_accel","m2_ic","m2_tau","m2_kd","m2_ki"};

// Length of each entry, in bytes
uint16_t payload_length_key[] = {100, 50, 128, 5,\
								1, 1, 1,\
								4, 4, 4, 4, 4, 4, 4,\
								4, 4, 4, 4, 4, 4, 4};

// End of Fill out //////////////////////////////////////////////////////////////////////////////////////////

uint8_t compiled_payload[PAYLOAD_BYTES] = {0};


//// helper function to calculate payload byte length
//static uint16_t calc_payload_bytes(void)
//{
//	uint16_t payload_bytes = 0;
//	for (uint16_t i = 0; i < PAYLOAD_DATA_FIELDS; i++)
//	{
//		payload_bytes += payload_length_key[i];
//	}
//	return payload_bytes;
//}

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
// Helper function to compile data from different memory locations into one contiguous source for sending
void compile_data_sources(uint8_t input_count, ...)
{
	va_list args;
	va_start(args, input_count);
	uint16_t write_idx = 0;

	// Check the input argument number. NOTE, IT IS IMPORTANT THAT YOU PASS AS MANY ARGUMENTS AS THERE ARE DATAFIELDS!
	// Otherwise, you're reading random memory
	if (input_count != PAYLOAD_DATA_FIELDS)
	{
		va_end(args);
		return; // Don't change the payload array. Echoing the same data will be the error state.
	}

	for (uint16_t i = 0; i < PAYLOAD_DATA_FIELDS; i++)
	{
		uint8_t* src = va_arg(args, uint8_t*);
		uint16_t len = payload_length_key[i];
		// Check for null pointer
		if (src == NULL)
		{
			va_end(args);
			return;
		}
		// Check bounds
		if (write_idx + len > PAYLOAD_BYTES)
		{
			va_end(args);
			return;
		}
		memcpy(&compiled_payload[write_idx], src, len);
		write_idx += payload_length_key[i];
	}
	va_end(args);
	return;
}

// Package data and send over UART
// Packet the predefined data payload (Non-generic)
void crc_uart_send_data(const uint8_t* src,
		UART_HandleTypeDef* huart)
{

	uint8_t pkt[PKT_BYTES];

	// 1. Header (preamble)
	pkt[0] = 0x55;
	pkt[1] = 0xAA;

    // 2. 2-byte payload length (little endian)
    pkt[2] = (uint8_t)(PAYLOAD_BYTES & 0xFF);        // LSB
    pkt[3] = (uint8_t)((PAYLOAD_BYTES >> 8) & 0xFF); // MSB

    // 3. Build the payload: Copy the data from the compiled array to the delivery packet.
    memcpy(&pkt[4], compiled_payload, PAYLOAD_BYTES);

    // 4. CRC over length + payload
    //    Starts from pkt[2], length = LEN_FIELD_BYTES + PAYLOAD_BYTES
    uint16_t crc = crc16_ccitt(&pkt[2], LEN_FIELD_BYTES + PAYLOAD_BYTES);
    pkt[4 + PAYLOAD_BYTES]     = (uint8_t)(crc & 0xFF);
    pkt[4 + PAYLOAD_BYTES + 1] = (uint8_t)(crc >> 8);

    // 5. Transmit over UART
    HAL_UART_Transmit(huart, pkt, PKT_BYTES, HAL_MAX_DELAY);
}
