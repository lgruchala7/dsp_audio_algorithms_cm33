/*
 * algorithm_testbench.c
 *
 *  Created on: 26 cze 2023
 *      Author: Łukasz
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "fsl_debug_console.h"

#include "arm_math.h"
#include "fsl_powerquad.h"

#include "drc_algorithms_cm33.h"
#include "algorithm_testbench.h"

#include <time.h>
#include <math.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FREQ_COUNT	3

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
static void init_arr_with_rand_u16(uint16_t * arr, size_t arr_size)
{
	srand(time(NULL));
    for (int i = 0 ; i < arr_size ; i++)
    {
    	arr[i] = (uint16_t)(rand() % UINT16_MAX);
    }
}

static void init_arr_with_rand_float(float * arr, size_t arr_size)
{
	srand(time(NULL));
    for (int i = 0 ; i < arr_size ; i++)
    {
    	arr[i] = (float)(rand() % UINT8_MAX);
    }
}

static void generate_sine_wave_u16(uint16_t * input_vec, uint32_t vec_len, uint32_t fs)
{
	float offset = (UINT16_MAX / 2);
	float freq[FREQ_COUNT] = {10000.0f, 7000.0f, 2000.0f};
	float amp[FREQ_COUNT] = {1.0f, 1.5f, 0.2f};
	float rad[FREQ_COUNT] = {0.0f};
	float comp[FREQ_COUNT] = {0.0f};
	float temp_comp_sum = 0.0f;
	float amp_sum = 0.0f;

	for (uint32_t i = 0U; i < FREQ_COUNT; i++)
	{
		amp_sum += amp[i];
	}

	for (uint32_t i = 0U, sample_num = 0U; i < vec_len; i += 2, sample_num++)
	{
		/* convert to radians */
		for (uint32_t j = 0U; j < FREQ_COUNT; j++)
		{
			rad[j] = (2 * PI * freq[j] * sample_num / fs);
		}

		/* calculate component signals values */
		for (uint32_t j = 0U; j < FREQ_COUNT; j++)
		{
			comp[j] = (offset * amp[j] * arm_sin_f32(rad[j]));
		}

		temp_comp_sum = 0.0f;
		/* sum component signals */
		for (uint32_t j = 0U; j < FREQ_COUNT; j++)
		{
			temp_comp_sum += comp[j];
		}
		temp_comp_sum /= amp_sum;
		input_vec[i] = (uint16_t)(offset + temp_comp_sum);

//		rad[0] = (2 * PI * freq[0] * i / fs);
//		rad[1] = (2 * PI * freq[1] * i / fs);
//		comp[0] = (offset * amp[0] * arm_sin_f32(rad[0]));
//		comp[1] = (offset * amp[1] * arm_sin_f32(rad[1]));
//		input_vec[i] += (uint16_t)(offset + ((comp[0] + comp[1]) / FREQ_COUNT));
	}
}

void measure_algorithm_time_u16(void (*algorithm_func)(uint16_t *, uint16_t *, size_t), uint16_t * src_buffer, uint16_t * dst_buffer, size_t buffer_size, uint32_t iterations)
{
	volatile unsigned long previous_time = 0UL;
    unsigned long exec_time_ticks = 0UL;
    unsigned long exec_time_ticks_sum = 0UL;
    unsigned long max_time_ticks = 0UL;

    // write random values to buffer
    init_arr_with_rand_u16(src_buffer, buffer_size);

	previous_time = ctimer_ticks;
	for (int i = 0; i < iterations; i++)
	{
		algorithm_func(src_buffer, dst_buffer, buffer_size);
	}
	exec_time_ticks_sum = (ctimer_ticks - previous_time);

    // execute algorithm 'iterations' times
//    for (int i = 0; i < iterations; i++)
//    {
//		previous_time = ctimer_ticks;
//		algorithm_func(src_buffer, dst_buffer, buffer_size);
//		exec_time_ticks = (ctimer_ticks - previous_time);
//		exec_time_ticks_sum += exec_time_ticks;
//		if (exec_time_ticks > max_time_ticks)
//		{
//			max_time_ticks = exec_time_ticks;
//		}
//	}

    // print calculated values
    PRINTF("Average time: %.3f ms\r\n", (TICKS_TO_MS(exec_time_ticks_sum) / (float)iterations));
}

void test_pq_math(float * arr, uint32_t iterations)
{
	volatile unsigned long previous_time = 0UL;
    unsigned long exec_time_ticks = 0UL;
    unsigned long exec_time_ticks_sum = 0UL;

	float x1 = 3.0f;
	float x2 = 9.0f;
	volatile float result;

	init_arr_with_rand_float(arr, iterations);

	previous_time = ctimer_ticks;
	for (int i = 0; i < iterations; i++)
	{
		PQ_DivF32(&x1, &arr[i], (float *)&result);
	}
	exec_time_ticks_sum = (ctimer_ticks - previous_time);

	PRINTF("[PQ] Average time x1/x2: %.3f us\r\n", (TICKS_TO_US(exec_time_ticks_sum) / (float)iterations));

	previous_time = ctimer_ticks;
	for (int i = 0; i < iterations; i++)
	{
		result = x1 / arr[i];
	}
	exec_time_ticks_sum = (ctimer_ticks - previous_time);
	PRINTF("[CM-33] Average time x1/x2: %.3f us\r\n", (TICKS_TO_US(exec_time_ticks_sum) / (float)iterations));


	previous_time = ctimer_ticks;
	for (int i = 0; i < iterations; i++)
	{
		PQ_LnF32(&arr[i], (float *)&result);
	}
	exec_time_ticks_sum = (ctimer_ticks - previous_time);
	PRINTF("[PQ] Average time ln: %.3f us\r\n", (TICKS_TO_US(exec_time_ticks_sum) / (float)iterations));

	previous_time = ctimer_ticks;
	for (int i = 0; i < iterations; i++)
	{
		result = logf(arr[i]);
	}
	exec_time_ticks_sum = (ctimer_ticks - previous_time);
	PRINTF("[CM-33] Average time ln: %.3f us\r\n", (TICKS_TO_US(exec_time_ticks_sum) / (float)iterations));
}

void print_buffer_data_u16(uint16_t * data, size_t data_size)
{
	for (size_t i = 0; i < data_size; i++)
	{
		PRINTF("0x%0X, ", data[i]);
		if (i%20 == 19)
		{
			PRINTF("\r\n");
		}
	}
	PRINTF("\r\n\n");
}

void test_algorithm(void (*algorithm_func)(uint16_t *, uint16_t *, size_t), uint16_t * src_buffer, uint16_t * dst_buffer, size_t buffer_size, uint32_t fs)
{
    generate_sine_wave_u16(src_buffer, buffer_size, fs);
    print_buffer_data_u16(src_buffer, buffer_size);
    algorithm_func(src_buffer, dst_buffer, buffer_size);
    print_buffer_data_u16(dst_buffer, buffer_size);

}
