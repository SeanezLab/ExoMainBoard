/*
 * math_helpers.h
 *
 *  Created on: Jan 20, 2026
 *      Author: k.rodolfo
 */

#ifndef INC_MATH_HELPERS_H_
#define INC_MATH_HELPERS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);


#ifdef __cplusplus
}
#endif

#endif /* INC_MATH_HELPERS_H_ */
