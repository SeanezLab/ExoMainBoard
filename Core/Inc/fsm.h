/*
 * fsm.h
 *
 *  Created on: Aug 8, 2026
 *      Author: k.rodolfo
 */


#ifndef INC_FSM_H_
#define INC_FSM_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>



#define COMMAND_MODE 0
#define TRANSPARENCY_MODE 1
#define CONFIG_MODE 2

#define COMMAND_CMD 'c'
#define TRANSPARENCY_TGL 't'
#define CONFIG_CMD 'cg'



typedef struct{
	uint8_t state;
	uint8_t next_state;
	uint8_t state_change;
	uint8_t ready;
	char cmd_buff[8];
	char bytecount;
	char cmd_id;
}FSMStruct;

void run_fsm(FSMStruct* fsmstate);
void update_fsm(FSMStruct * fsmstate, char fsm_input);
void fsm_enter_state(FSMStruct * fsmstate);
void fsm_exit_state(FSMStruct * fsmstate);
void enter_command_state(void);
void enter_transparency_state(void);
void enter_config_state(void);
void process_user_input(FSMStruct * fsmstate);
void run_motor_loop(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_FSM_H_ */
