/*
 * crc.h
 *
 *  Created on: Dec 29, 2025
 *      Author: rdkee
 */

#ifndef INC_CRC_H_
#define INC_CRC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <main.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "structs.h"
#include "data_tx_arrays.h"

// Two byte protocol
#define LEN_FIELD_BYTES 2
#define HEADER_BYTES 2 // This was 3?!
#define FOOTER_BYTES 2
#define CRC_BYTES 2
#define RX_BUF_LEN 50 // Modify this field if you expect to receive packets >50 bytes.


extern uint8_t rx_buffer[];
#define CRC_PACKET_OVERHEAD_BYTES (HEADER_BYTES + LEN_FIELD_BYTES + CRC_BYTES + FOOTER_BYTES)



bool compile_data_sources(TxPacket* packet); // Compile the sources described by this packet
bool crc_pack_data(uint8_t* pkt, uint16_t capacity, const uint8_t* src, uint16_t payload_bytes);
bool crc_uart_send_data(TxPacket* packet); // Keep packet allocated until DMA completes
void crc_uart_rcv_data(rdg_buf_struct* rdg_struct, uint16_t length); // Receive and handle crc_packeted data over UART



#ifdef __cplusplus
}
#endif

#endif /* INC_CRC_H_ */
