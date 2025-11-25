/*
 * ring_buffer.h
 *
 *  Created on: Nov 24, 2025
 *      Author: k.rodolfo
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdint.h>

typedef struct{
	uint16_t buf_size;
	volatile uint16_t rb_head;
	volatile uint16_t rb_tail;
	uint8_t rb_buffer[];
}rb_struct;

inline uint16_t rb_count(rb_struct* buffer);
inline uint16_t rb_space(rb_struct* buffer);
inline void rb_push(rb_struct* buffer, uint8_t byte);
inline uint16_t rb_pop_chunk(rb_struct* buffer, uint8_t* dst, uint16_t max_len);
rb_struct* rb_init(uint16_t size);





#endif /* INC_RING_BUFFER_H_ */
