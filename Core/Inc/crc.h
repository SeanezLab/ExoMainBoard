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
#include <stdint.h>
#include <string.h>
#include <stdarg.h>

// Two byte protocol
#define LEN_FIELD_BYTES 2
#define HEADER_BYTES 3
#define CRC_BYTES 2

// Fill in Below for each new protocol ///////////////////////////////////////////////////////////////////////
// Array that tell the CRC packager how many bytes are used for each data field
uint16_t payload_length_key[] = {100, 50, 128, 5,\
								4, 4, 4, 4, 4, 4, 4, 1, 1,\
								4, 4, 4, 4, 4, 4, 4, 1, 1};
// Human readable list of each entry (Not necessary for the CRC packager but helps me remember)
char *payload_entries[] = {"vibro-Z-axis", "vibro-gpio","vibro-fft","vibro-state",\
						"exo1-pos","exo1-vel","exo1-accel","exo1-ic","exo1-tau","exo1-kd","exo1-ki",'exo1-busy',"exo1-fsm",\
						"exo2-pos","exo2-vel","exo2-accel","exo2-ic","exo2-tau","exo2-kd","exo2-ki",'exo2-busy',"exo2-fsm"};
// End of Fill out //////////////////////////////////////////////////////////////////////////////////////////


static uint16_t calc_payload_bytes(void); // helper function for calculating payload bytes
static uint16_t crc16_ccitt(const uint8_t* buf, uint16_t len); // helper function for implementing CRC packets
static void crc_uart_send_data(const uint8_t* src, UART_HandleTypeDef* huart); // Send crc-packeted data over UART



#ifdef __cplusplus
}
#endif

#endif /* INC_CRC_H_ */
