/*
 * cmd_array.h
 *
 * The purpose of this file is to hold all the commands sent to the exoskeleton main board. During the main control loop, commands will be
 * taken from this array and executed. This is not a queue. If the state of the exoskeleton matches the command arrays, nothing should change.
 * I will adapt this array if this does not hold.
 *
 *  Created on: Jan 28, 2026
 *      Author: k.rodolfo
 */

#ifndef INC_CMD_ARRAY_H_
#define INC_CMD_ARRAY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "fdcan.h"

// Motor 1 control  constants (Do not change unless you've validated the tuning!)
#define DES_M1_V .01f
#define DES_M1_KP 20.0f
#define DES_M1_KD .75f
#define DES_M1_TFF 0.0f

// Exoskeleton desired state/mode
extern float des_mode;

// Motor command structure
typedef struct{
	uint8_t motor_id;
	float des_pos;
	float des_mode;
	float des_v;
	float des_kp;
	float des_kd;
	float des_tff;
	bool new_pos;
	bool new_sp_cmd; // New special command, 0:exit motor mode, 1:enter motor mode, 2:zero position
	bool rdy_to_snd;
	bool new_cont;
}MotorCommand;

// Vibrotactile command structure
typedef struct{
	float des_tscs_delay;
	float des_targ_freq;
	float des_duty_cycle;
	float des_vibro_zthresh;
	bool new_delay;
	bool new_freq;
	bool new_duty_cycle;
	bool new_zthresh;
}VibroCommand;

void motor_cmd_init(MotorCommand* m_cmd, uint8_t motor_id);
void vibro_cmd_init(VibroCommand* vibro_cmd);
void handle_m_cmd(MotorCommand* m_cmd, CANTxMessage* m_tx);


#ifdef __cplusplus
}
#endif

#endif /* INC_CMD_ARRAY_H_ */
