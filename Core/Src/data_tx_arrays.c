/*
 * data_arrays.c
 *
 *  Created on: Jan 7, 2026
 *      Author: k.rodolfo
 */


#include "data_tx_arrays.h"

// Holds the data array variable for better readability



// Vibrotactile stimulator data
uint8_t vibro_z_axis[100] = {0};
uint8_t vibro_gpio[50] = {0};
uint8_t vibro_fft[128] = {0};
// 1st byte, tSCS delay (ms), Uint8; 2nd byte, Target Frequency (hz), Uint8; 3rd byte, Duty Cycle (%), Uint8; 4th and 5th bytes, Z-axis threshold (Gs), Int16.
uint8_t vibro_state[5] = {0};
// Exoskeleton main control board data
uint8_t exo_busy[1] = {0};
uint8_t exo_fsm[1] = {0};
uint8_t exo_debug[1] = {0};
// Motor drive 1 data (knee)
uint8_t m1_pos[4] = {0};
uint8_t m1_vel[4] = {0};
uint8_t m1_accel[4] = {0};
uint8_t m1_ic[4] = {0};
uint8_t m1_tau[4] = {0};
uint8_t m1_kd[4] = {0};
uint8_t m1_ki[4] = {0};
// Motor drive 2 data (ankle)
uint8_t m2_pos[4] = {0};
uint8_t m2_vel[4] = {0};
uint8_t m2_accel[4] = {0};
uint8_t m2_ic[4] = {0};
uint8_t m2_tau[4] = {0};
uint8_t m2_kd[4] = {0};
uint8_t m2_ki[4] = {0};

