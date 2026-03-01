/*
 * test_signal.c
 *
 *  Created on: Feb 28, 2026
 *      Author: k.rodolfo
 */

#include "test_signals.h"
#include "math.h"

#define PI 3.14F

float cos_counter = 0;
float frame_counter = 0;


void increment_cos_counter(void)
{
	if (cos_counter >= 2*PI)
	{
		cos_counter = 0;
	}
	else
	{
		cos_counter += .1F;
	}
}

void increment_frame_counter(void)
{
	if (frame_counter >= 65535)
	{
		frame_counter = 0;
	}
	else
	{
		frame_counter += 1;
	}
}


float update_cos_signal(void)
{
	float cos_out = cosf(cos_counter);
	increment_cos_counter();
	return cos_out;
}
