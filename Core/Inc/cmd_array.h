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

// Exoskeleton desired state/mode
extern float des_mode;

// Motor commands
extern float des_m1_pos;
extern float des_m2_pos;

// Vibrotactile commands
extern float des_tscs_delay;
extern float des_targ_freq;
extern float des_vibro_zthresh;







#ifdef __cplusplus
}
#endif

#endif /* INC_CMD_ARRAY_H_ */
