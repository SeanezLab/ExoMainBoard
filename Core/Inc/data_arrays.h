/*
 * data_arrays.h
 *
 *  Created on: Jan 7, 2026
 *      Author: k.rodolfo
 */

#ifndef INC_DATA_ARRAYS_H_
#define INC_DATA_ARRAYS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdint.h>

// Holds the data array variable for better readability

// Vibrotactile stimulator data
extern uint8_t vibro_z_axis[];
extern uint8_t vibro_gpio[];
extern uint8_t vibro_fft[];
extern uint8_t vibro_state[];
// Exoskeleton main control board data
extern uint8_t exo_busy[];
extern uint8_t exo_fsm[];
extern uint8_t exo_debug[];
// Motor drive 1 data (knee)
extern uint8_t m1_pos[];
extern uint8_t m1_vel[];
extern uint8_t m1_accel[];
extern uint8_t m1_ic[];
extern uint8_t m1_tau[];
extern uint8_t m1_kd[];
extern uint8_t m1_ki[];
// Motor drive 2 data (ankle)
extern uint8_t m2_pos[];
extern uint8_t m2_vel[];
extern uint8_t m2_accel[];
extern uint8_t m2_ic[];
extern uint8_t m2_tau[];
extern uint8_t m2_kd[];
extern uint8_t m2_ki[];


#ifdef __cplusplus
}
#endif


#endif /* INC_DATA_ARRAYS_H_ */
