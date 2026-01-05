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

// VIBROTACTILE STIMULATOR PACKET DEFINES START
// Initialize buffers and associated parameters for data transmission
#define TIME_SIZE 256 // in data points: ~77.57 ms of data at 3300 hz
#define FFT_SIZE 256 // in data points: 1 second of data at 330 hz.
#define STATE_SIZE 5 // in bytes: 4 float states, tSCS delay (int8_t), target frequency (int8_t), pwm duty cycle (int8_t), z-axis threshold (uint16_t)


// Packet defines
#define FFT_SIZE
#define SAMPLES_PER_PACKET 50
#define SAMPLES_PER_FFT FFT_SIZE/2
// Z accel + GPIO sync + FFT magnitude + State
#define PAYLOAD_BYTES_VIBRO ((SAMPLES_PER_PACKET * sizeof(int16_t)) +\
		(SAMPLES_PER_PACKET * sizeof(uint8_t)) +\
		(SAMPLES_PER_FFT * sizeof(int8_t)) +\
		STATE_SIZE)

// VIBROTACTILE STIMULATOR PACKET DEFINES END

// EXOSKELETON PACKET DEFINES START
#define MOTOR_COUNT 2 // Number of motors connected. TODO Should be kept in a config header.
#define POS_BYTES 4 // in bytes: 1 float
#define VEL_BYTES 4
#define ACCEL_BYTES 4
#define IC_BYTES 4
#define TAU_BYTES 4
#define KD_BYTES 4
#define KI_BYTES 4
#define BUSY_BYTES 1
#define FSM_BYTES 1
// Total number of Bytes
#define PAYLOAD_BYTES_EXO MOTOR_COUNT *\
		(POS_BYTES +\
		VEL_BYTES +\
		ACCEL_BYTES +\
		IC_BYTES +\
		TAU_BYTES +\
		KD_BYTES +\
		KI_BYTES +\
		BUSY_BYTES +\
		FSM_BYTES +\
		)

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


// Tells the CRC packager how many data entries are expected.
uint16_t payload_data_fields = sizeof(payload_length_key) / sizeof(payload_length_key[0]);


// Two byte protocol
#define LEN_FIELD_BYTES 2
#define HEADER_BYTES 3
#define CRC_BYTES 2
#define PKT_BYTES (3 + LEN_FIELD_BYTES + PAYLOAD_BYTES_VIBRO + PAYLOAD_BYTES_EXO + 2)  // header + len + data + CRC


static uint16_t crc16_ccitt(const uint8_t* buf, uint16_t len); // helper function for implementing CRC packets
static void crc_uart_send_data(const float* src, uint16_t* payload_length_key, uint16_t payload_data_fields, UART_HandleTypeDef* huart); // Send crc-packeted data over UART



#ifdef __cplusplus
}
#endif

#endif /* INC_CRC_H_ */
