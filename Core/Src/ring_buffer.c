/*
 * ring_buffer.c
 *
 *  Created on: Nov 24, 2025
 *      Author: k.rodolfo
 */

#include "ring_buffer.h"

inline uint16_t rb_count(rb_struct* buffer){
	//  Gets the total count of data in the buffer
	return (uint16_t)(buffer->rb_head - buffer-> rb_tail);
}

inline uint16_t rb_space(rb_struct* buffer){
	// Gets the remaining space in the buffer. 1 is subtracted as a slot is used to distinguish full and empty
	return (uint16_t)(buffer->buf_size - 1u - rb_count(buffer));
}

inline void rb_push(rb_struct* buffer, uint8_t byte){
	// Checks if there is a space for a byte, then writes it if so. Does nothing if there is no space
	if (rb_space(buffer)){
		buffer->rb_buffer[buffer->rb_head++ % buffer->buf_size] = byte;
	}
}

inline uint16_t rb_pop_chunk(rb_struct* buffer, uint8_t* dst, uint16_t max_len){
	// Extracts a chunk of data from the input buffer, and copies it to a destination buffer.
	uint16_t count = rb_count(buffer);
	if (!count){
		return 0;
	}
	if (count > max_len){
		count = max_len;
	}
	for (uint16_t i = 0; i < count; i++){
		dst[i] = buffer->rb_buffer[buffer->rb_tail++ % buffer->buf_size];
	}
	return 1;
}

rb_struct* rb_init(uint16_t size){
	rb_struct* rb = malloc(sizeof(rb_struct) + size);
	rb->buf_size = size;
	rb->rb_head = 0;
	rb->rb_tail = 0;
	return rb;
}


