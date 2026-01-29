/*
 * cmd_array.c
 *
 * The purpose of this file is to hold all the commands sent to the exoskeleton main board. During the main control loop, commands will be
 * taken from this array and executed. This is not a queue. If the state of the exoskeleton matches the command arrays, nothing should change.
 * I will adapt this array if this does not hold.
 *
 *  Created on: Jan 28, 2026
 *      Author: k.rodolfo
 */


#include "cmd_array.h"

// Exoskeleton desired state/mode
float des_mode = 0;

// Motor commands
float des_m1_pos = 0;
float des_m2_pos = 0;

// Vibrotactile commands
float des_tscs_delay = 0;
float des_targ_freq = 0;
float des_vibro_zthresh = 0;
