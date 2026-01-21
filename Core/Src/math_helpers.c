/*
 * math_helpers.c
 *
 *  Created on: Jan 20, 2026
 *      Author: Ben Katz, yoinked by Rodolfo Keesey from https://os.mbed.com/users/benkatz/code/CanMasterTest/
 */

float fmaxf(float x, float y)
	{
    /// Returns maximum of x, y ///
    return (((x)>(y))?(x):(y));
    }

float fminf(float x, float y)
	{
    /// Returns minimum of x, y ///
    return (((x)<(y))?(x):(y));
    }

int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits via linear normalization ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }


float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }




