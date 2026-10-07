/* CAN wire format. Keep this file identical in motorcontrol and ExoMainBoard. */
#ifndef INC_CAN_PROTOCOL_H_
#define INC_CAN_PROTOCOL_H_

#include <stdbool.h>
#include <stdint.h>

#define CAN_MODE_SHIFT 5U
#define CAN_MOTOR_ID_MAX 31U
#define CAN_QUERY_BYTES 1U
#define CAN_COMMAND_BYTES 8U
#define CAN_STATE_BYTES 7U
#define CAN_CHARACTERIZATION_BYTES 6U
#define CAN_ENCODER_BYTES 8U
#define CAN_ENCODER_COUNT_MIN (-8388608)
#define CAN_ENCODER_COUNT_MAX 8388607
#define CAN_CHARACTERIZATION_P_STEP (3.14159265358979323846f*2.0f / 32768.0f)
#define CAN_CHARACTERIZATION_I_MIN -40.0f
#define CAN_CHARACTERIZATION_I_MAX 40.0f

typedef enum{
	CAN_QUERY_STATE = 0,
	CAN_COMMAND = 1,
	CAN_QUERY_CHARACTERIZATION = 2,
	CAN_COMMAND_CHARACTERIZATION = 3,
	CAN_QUERY_ENCODER = 4,
	CAN_COMMAND_ENCODER = 5,
	CAN_SPECIAL_COMMAND = 7 // Existing FF ... FC/FD/FE special commands
}CANRequestMode;

typedef enum{
	CAN_REPLY_STATE = 0,
	CAN_REPLY_CHARACTERIZATION = 1,
	CAN_REPLY_ABS_ENCODER = 2
}CANReplyMode;

typedef enum{
	CAN_SPECIAL_ENABLE = 0xFC,
	CAN_SPECIAL_DISABLE = 0xFD,
	CAN_SPECIAL_ZERO = 0xFE,
	CAN_SPECIAL_QUERY = 0xFF // Accept the old query as well
}CANSpecialCommand;

typedef struct{
	float p_des, v_des, kp, kd, t_ff;
}CANCommandData;

typedef struct{
	uint16_t position, velocity, kp, kd, torque;
}CANCommandFields;

typedef struct{
	uint8_t id;
	float position, velocity, torque, bus_voltage;
}CANStateReply;

typedef struct{
	uint8_t id;
	float position, i_q, i_q_des; // Driver coordinates; position in rad, currents in A
}CANCharacterizationReply;

// One eight-byte frame: position:16, iq_des:12, reserved:4, signed linearized_count:24.
typedef struct{
	uint8_t id;
	float position, i_q_des; // Same units/ranges and 12-bit current scale as characterization
	int32_t linearized_count; // Sign-extended 24-bit encoder.count, before M_ZERO and wrapping
}CANAbsEncoderReply;

static inline bool can_request_length_valid(uint8_t mode, uint32_t length){
	switch(mode){
		case CAN_QUERY_STATE:
		case CAN_QUERY_CHARACTERIZATION:
		case CAN_QUERY_ENCODER:
			return length == CAN_QUERY_BYTES;
		case CAN_COMMAND:
		case CAN_COMMAND_CHARACTERIZATION:
		case CAN_COMMAND_ENCODER:
		case CAN_SPECIAL_COMMAND:
			return length == CAN_COMMAND_BYTES;
		default:
			return false;
	}
}

// [mode:3][position:16][velocity:12][kp:12][kd:9][torque:12], MSB first.
static inline void can_pack_command_fields(uint8_t *data, const CANCommandFields *fields, CANRequestMode mode){
	data[0] = (mode << CAN_MODE_SHIFT) | (fields->position >> 11);
	data[1] = fields->position >> 3;
	data[2] = ((fields->position & 0x7) << 5) | (fields->velocity >> 7);
	data[3] = ((fields->velocity & 0x7F) << 1) | (fields->kp >> 11);
	data[4] = fields->kp >> 3;
	data[5] = ((fields->kp & 0x7) << 5) | (fields->kd >> 4);
	data[6] = ((fields->kd & 0xF) << 4) | (fields->torque >> 8);
	data[7] = fields->torque;
}

static inline void can_unpack_command_fields(const uint8_t *data, CANCommandFields *fields){
	fields->position = ((data[0] & 0x1F) << 11) | (data[1] << 3) | (data[2] >> 5);
	fields->velocity = ((data[2] & 0x1F) << 7) | (data[3] >> 1);
	fields->kp = ((data[3] & 0x1) << 11) | (data[4] << 3) | (data[5] >> 5);
	fields->kd = ((data[5] & 0x1F) << 4) | (data[6] >> 4);
	fields->torque = ((data[6] & 0xF) << 8) | data[7];
}

// The position is signed, with exact zero. Saturate at the ends, do not wrap.
// Callers check that the position is finite before encoding it.
static inline uint16_t can_encode_characterization_position(float position){
	float counts = position / CAN_CHARACTERIZATION_P_STEP;
	if(counts <= -32768.0f){return 0x8000;}
	if(counts >= 32767.0f){return 0x7FFF;}
	int32_t rounded = (int32_t)(counts + (counts >= 0.0f ? 0.5f : -0.5f));
	return (uint16_t)rounded;
}

static inline float can_decode_characterization_position(uint16_t position){
	int32_t counts = position;
	if(counts >= 32768){counts -= 65536;}
	return (float)counts * CAN_CHARACTERIZATION_P_STEP;
}

#endif /* INC_CAN_PROTOCOL_H_ */
