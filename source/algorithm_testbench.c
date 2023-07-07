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
#include "fsl_device_registers.h"

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
static void init_arr_with_rand_16(int16_t * arr, size_t arr_size)
{
	srand(time(NULL));
    for (int i = 0 ; i < arr_size ; i++)
    {
    	arr[i] = (int16_t)((rand() % INT16_MAX) - INT16_MAX);
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

static void generate_sine_wave_16(int16_t * input_vec, uint32_t vec_len, uint32_t fs, float max_amplitude, float * freq, float * amp, int freq_count)
{
//	float freq[FREQ_COUNT] 	= {9000.0f, 70000.0f, 1500.0f};
//	float amp[FREQ_COUNT] 	= {0.0f, 0.0f, 1.0f};
	float * rad 	= (float *)malloc(sizeof(float) * freq_count);
	float * comp = (float *)malloc(sizeof(float) * freq_count);
	float temp_comp_sum 	= 0.0f;
	float amp_sum 			= 0.0f;

	if (max_amplitude > (float)INT16_MAX)
	{
		PRINTF("\r\nChange amplitude!!!\r\n");
	}

	for (uint32_t i = 0U; i < freq_count; i++)
	{
		amp_sum += amp[i];
	}

	for (uint32_t i = 0U, sample_num = 0U; i < vec_len; i += 2, sample_num++)
	{
		/* convert to radians */
		for (uint32_t j = 0U; j < freq_count; j++)
		{
			rad[j] = (2 * PI * freq[j] * sample_num / fs);
		}

		/* calculate component signals values */
		for (uint32_t j = 0U; j < freq_count; j++)
		{
			comp[j] = (max_amplitude * amp[j] * arm_sin_f32(rad[j]));
		}

		temp_comp_sum = 0.0f;
		/* sum component signals */
		for (uint32_t j = 0U; j < freq_count; j++)
		{
			temp_comp_sum += comp[j];
		}
		input_vec[i] = (int16_t)(temp_comp_sum / amp_sum);
	}

	free(rad);
	free(comp);
}

void measure_algorithm_time_16(void (*algorithm_func)(int16_t *, int16_t *, size_t), int16_t * src_buffer, int16_t * dst_buffer, size_t buffer_size, uint32_t iterations)
{
	volatile unsigned long previous_time = 0UL;
    unsigned long exec_time_ticks = 0UL;
    unsigned long exec_time_ticks_sum = 0UL;

    // write random values to buffer
    init_arr_with_rand_16(src_buffer, buffer_size);

	previous_time = MSDK_GetCpuCycleCount();
    // execute algorithm 'iterations' times
	for (int i = 0; i < iterations; i++)
	{
		algorithm_func(src_buffer, dst_buffer, buffer_size);
	}
	exec_time_ticks_sum = (MSDK_GetCpuCycleCount() - previous_time);

    PRINTF("Average time: %.3f ms\r\n", (CYCLES_TO_MS(exec_time_ticks_sum)) / (float)iterations);
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

	previous_time = MSDK_GetCpuCycleCount();
	for (int i = 0; i < iterations; i++)
	{
		PQ_DivF32(&x1, &arr[i], (float *)&result);
	}
	exec_time_ticks_sum = (MSDK_GetCpuCycleCount() - previous_time);

	PRINTF("[PQ] Average time x1/x2: %.3f us\r\n", (CYCLES_TO_US(exec_time_ticks_sum) / (float)iterations));

	previous_time = MSDK_GetCpuCycleCount();
	for (int i = 0; i < iterations; i++)
	{
		result = x1 / arr[i];
	}
	exec_time_ticks_sum = (MSDK_GetCpuCycleCount() - previous_time);
	PRINTF("[CM-33] Average time x1/x2: %.3f us\r\n", (CYCLES_TO_US(exec_time_ticks_sum) / (float)iterations));


	previous_time = MSDK_GetCpuCycleCount();
	for (int i = 0; i < iterations; i++)
	{
		PQ_LnF32(&arr[i], (float *)&result);
	}
	exec_time_ticks_sum = (MSDK_GetCpuCycleCount() - previous_time);
	PRINTF("[PQ] Average time ln: %.3f us\r\n", (CYCLES_TO_US(exec_time_ticks_sum) / (float)iterations));

	previous_time = MSDK_GetCpuCycleCount();
	for (int i = 0; i < iterations; i++)
	{
		result = logf(arr[i]);
	}
	exec_time_ticks_sum = (MSDK_GetCpuCycleCount() - previous_time);
	PRINTF("[CM-33] Average time ln: %.3f us\r\n", (CYCLES_TO_US(exec_time_ticks_sum) / (float)iterations));
}

void print_buffer_data_16(int16_t * data, size_t data_size)
{
	for (size_t i = 0; i < data_size; i++)
	{
//		PRINTF("0x%0X, ", data[i]);
		PRINTF("%d, ", data[i]);
		if (i%20 == 19)
		{
			PRINTF("\r\n");
		}
	}
	PRINTF("\r\n\n");
}

void write_buffer_data_to_file_16(int16_t * data, size_t data_size)
{
	PRINTF("$");
	for (size_t i = 0; i < data_size; i++)
	{
		PRINTF("%d, ", data[i]);
		if (i%20 == 19)
		{
			PRINTF("\r\n");
		}
	}
}

void test_algorithm(void (*algorithm_func)(int16_t *, int16_t *, size_t), int16_t * src_buffer, int16_t * dst_buffer,
		size_t buffer_size, uint32_t fs, float * freq, float * amp, int freq_count)
{
	generate_sine_wave_16(&src_buffer[0], buffer_size/5, fs, (float)INT16_MAX * 0.15, freq, amp, freq_count);
	generate_sine_wave_16(&src_buffer[1000],buffer_size/5, fs, (float)INT16_MAX * 0.40, freq, amp, freq_count);
	generate_sine_wave_16(&src_buffer[2000], buffer_size/5, fs, (float)INT16_MAX * 0.65, freq, amp, freq_count);
	generate_sine_wave_16(&src_buffer[3000], buffer_size/5, fs, (float)INT16_MAX * 0.8, freq, amp, freq_count);
	generate_sine_wave_16(&src_buffer[4000], buffer_size/5, fs, (float)INT16_MAX * 0.95, freq, amp, freq_count);

	write_buffer_data_to_file_16(src_buffer, buffer_size);
	algorithm_func(src_buffer, dst_buffer, buffer_size);
	write_buffer_data_to_file_16(dst_buffer, buffer_size);
}
