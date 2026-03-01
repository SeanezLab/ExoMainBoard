/*
 * test_signals.h
 *
 *  Created on: Feb 28, 2026
 *      Author: k.rodolfo
 */

#ifndef INC_TEST_SIGNALS_H_
#define INC_TEST_SIGNALS_H_

#ifdef __cplusplus
extern "C" {
#endif

extern float frame_counter;
void increment_cos_counter(void);
void increment_frame_counter(void);
float update_cos_signal(void);


#ifdef __cplusplus
}
#endif

#endif /* INC_TEST_SIGNALS_H_ */
