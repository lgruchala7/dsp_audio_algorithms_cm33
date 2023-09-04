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
#include "fsl_powerquad.h"

#include "drc_algorithms.h"
#include "algorithm_testbench.h"
#include "filters_cfg.h"
#include "main_cm33.h"

#include <time.h>
#include <math.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/


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

static void init_arr_with_rand_f32(float * arr, size_t arr_size)
{
	srand(time(NULL));
    for (int i = 0 ; i < arr_size ; i++)
    {
    	arr[i] = (float)(rand() % UINT8_MAX);
    }
}

void generate_sine_wave_16(int16_t * input_vec, uint32_t vec_len, uint32_t fs, float max_amplitude, float * freq, float * amp, int freq_cnt)
{
	float * rad 	= (float *)malloc(sizeof(float) * freq_cnt);
	float * comp = (float *)malloc(sizeof(float) * freq_cnt);
	float temp_comp_sum 	= 0.0f;
	float amp_sum 			= 0.0f;

	if (max_amplitude > (float)INT16_MAX)
	{
		PRINTF("\r\nChange amplitude!!!\r\n");
	}

	for (uint32_t i = 0U; i < freq_cnt; i++)
	{
		amp_sum += amp[i];
	}

	for (uint32_t i = 0U, sample_num = 0U; i < vec_len; i += 2, sample_num++)
	{
		/* convert to radians */
		for (uint32_t j = 0U; j < freq_cnt; j++)
		{
			rad[j] = (2 * PI * freq[j] * sample_num / fs);
		}

		/* calculate component signals values */
		for (uint32_t j = 0U; j < freq_cnt; j++)
		{
			comp[j] = (max_amplitude * amp[j] * arm_sin_f32(rad[j]));
		}

		temp_comp_sum = 0.0f;
		/* sum component signals */
		for (uint32_t j = 0U; j < freq_cnt; j++)
		{
			temp_comp_sum += comp[j];
		}
		input_vec[i] = (int16_t)(temp_comp_sum / amp_sum);
	}

	free(rad);
	free(comp);
}

void generate_sine_wave_f32(float32_t * input_vec, uint32_t vec_len, uint32_t fs, float max_amplitude, float * freq, float * amp, int freq_cnt)
{
	float * rad 	= (float *)malloc(sizeof(float) * freq_cnt);
	float * comp = (float *)malloc(sizeof(float) * freq_cnt);
	float temp_comp_sum 	= 0.0f;
	float amp_sum 			= 0.0f;

	if (max_amplitude > (float)INT16_MAX)
	{
		PRINTF("\r\nChange amplitude!!!\r\n");
	}

	for (uint32_t i = 0U; i < freq_cnt; i++)
	{
		amp_sum += amp[i];
	}

	for (uint32_t i = 0U; i < vec_len; i++)
	{
		/* convert to radians */
		for (uint32_t j = 0U; j < freq_cnt; j++)
		{
			rad[j] = (2 * PI * freq[j] * i / fs);
		}

		/* calculate component signals values */
		for (uint32_t j = 0U; j < freq_cnt; j++)
		{
			comp[j] = (max_amplitude * amp[j] * arm_sin_f32(rad[j]));
		}

		temp_comp_sum = 0.0f;
		/* sum component signals */
		for (uint32_t j = 0U; j < freq_cnt; j++)
		{
			temp_comp_sum += comp[j];
		}
		input_vec[i] = (float32_t)(temp_comp_sum / amp_sum);
	}

	free(rad);
	free(comp);
}

void measure_algorithm_time_f32(void (*algorithm_func)(float32_t *, float32_t *, size_t), float32_t * src_buffer, float32_t * dst_buffer, size_t bytes, uint32_t iterations)
{
	volatile unsigned long previous_time = 0UL;
    unsigned long exec_time_ticks = 0UL;
    unsigned long exec_time_ticks_sum = 0UL;
    size_t buffer_len = (bytes / sizeof(int16_t));
    // write random values to buffer
    init_arr_with_rand_f32(src_buffer, buffer_len);

	previous_time = MSDK_GetCpuCycleCount();
    // execute algorithm 'iterations' times
	for (int i = 0; i < iterations; i++)
	{
		algorithm_func(src_buffer, dst_buffer, buffer_len);
	}
	exec_time_ticks_sum = (MSDK_GetCpuCycleCount() - previous_time);

    PRINTF("Average time: %.3f ms\r\n", (CYCLES_TO_MS(exec_time_ticks_sum)) / (float)iterations);
}

void test_pq_math(float * arr, uint32_t arr_size)
{
	volatile unsigned long previous_time = 0UL;
    unsigned long exec_time_ticks = 0UL;
    unsigned long exec_time_ticks_sum = 0UL;

	float x1 = 3.0f;
	float x2 = 9.0f;
	volatile float result;

	init_arr_with_rand_f32(arr, arr_size);

	previous_time = MSDK_GetCpuCycleCount();
	for (int i = 0; i < arr_size; i++)
	{
		PQ_DivF32(&x1, &arr[i], (float *)&result);
	}
	exec_time_ticks_sum = (MSDK_GetCpuCycleCount() - previous_time);

	PRINTF("[PQ] Average time x1/x2: %.3f us\r\n", (CYCLES_TO_US(exec_time_ticks_sum) / (float)arr_size));

	previous_time = MSDK_GetCpuCycleCount();
	for (int i = 0; i < arr_size; i++)
	{
		result = x1 / arr[i];
	}
	exec_time_ticks_sum = (MSDK_GetCpuCycleCount() - previous_time);
	PRINTF("[CM-33] Average time x1/x2: %.3f us\r\n", (CYCLES_TO_US(exec_time_ticks_sum) / (float)arr_size));


	previous_time = MSDK_GetCpuCycleCount();
	for (int i = 0; i < arr_size; i++)
	{
		PQ_LnF32(&arr[i], (float *)&result);
	}
	exec_time_ticks_sum = (MSDK_GetCpuCycleCount() - previous_time);
	PRINTF("[PQ] Average time ln: %.3f us\r\n", (CYCLES_TO_US(exec_time_ticks_sum) / (float)arr_size));

	previous_time = MSDK_GetCpuCycleCount();
	for (int i = 0; i < arr_size; i++)
	{
		result = logf(arr[i]);
	}
	exec_time_ticks_sum = (MSDK_GetCpuCycleCount() - previous_time);
	PRINTF("[CM-33] Average time ln: %.3f us\r\n", (CYCLES_TO_US(exec_time_ticks_sum) / (float)arr_size));
}

void test_cmsis_dsp_f32(float32_t * src_arr, float32_t * dst_arr, uint32_t arr_len, uint32_t fs, uint32_t iterations)
{
	arm_fir_instance_f32 fir_instance;
	float32_t fir_state[FIR_BLOCK_SIZE + FIR_COEFF_COUNT - 1];
	arm_status status;
	float32_t  *input_arr, *output_arr;
	uint32_t numBlocks = arr_len / FIR_BLOCK_SIZE;

	float32_t freq[] 	= {9000.0f, 7000.0f, 1500.0f};
	float32_t amp[] 	= {0.2f, 0.0f, 1.0f};
	int freq_cnt = sizeof(freq) / sizeof(freq[0]);

	volatile unsigned long previous_time = 0UL;
    unsigned long exec_time_ticks = 0UL;
    unsigned long exec_time_ticks_sum = 0UL;

	generate_sine_wave_f32(&src_arr[0], arr_len, fs, (float)(INT16_MAX * 0.5), freq, amp, freq_cnt);

	/* Initialize input and output buffer pointers */
	input_arr = &src_arr[0];
	output_arr = &dst_arr[0];
	/* Call FIR init function to initialize the instance structure. */
	arm_fir_init_f32(&fir_instance, FIR_COEFF_COUNT, (float32_t *)&fir_coeff_f32[0], &fir_state[0], FIR_BLOCK_SIZE);

//	print_buffer_data_f32(src_arr, arr_size);

	previous_time = MSDK_GetCpuCycleCount();
	for (uint32_t i = 0; i < iterations; i++)
	{
		for(uint32_t j = 0; j < numBlocks; j++)
		{
			arm_fir_f32(&fir_instance, input_arr + (j * FIR_BLOCK_SIZE), output_arr + (j * FIR_BLOCK_SIZE), FIR_BLOCK_SIZE);
		}
	}
	exec_time_ticks_sum = (MSDK_GetCpuCycleCount() - previous_time);

//	print_buffer_data_f32(dst_arr, arr_size);
	PRINTF("[CM-33] Average time arm_fir_f32: %.3f ms\r\n", CYCLES_TO_MS(exec_time_ticks_sum) / (float)iterations);
}

void print_buffer_data_16(int16_t * buffer, size_t buffer_len)
{
	for (size_t i = 0; i < buffer_len; i++)
	{
//		PRINTF("0x%0X, ", data[i]);
		PRINTF("%d, ", buffer[i]);
		if (i%20 == 19)
		{
			PRINTF("\r\n");
		}
	}
	PRINTF("\r\n\n");
}

void print_buffer_data_32(int32_t * buffer, size_t buffer_len)
{
	for (size_t i = 0; i < buffer_len; i++)
	{
//		PRINTF("0x%0X, ", data[i]);
		PRINTF("%d, ", buffer[i]);
		if (i%20 == 19)
		{
			PRINTF("\r\n");
		}
	}
	PRINTF("\r\n\n");
}

void print_buffer_data_f32(float32_t * buffer, size_t buffer_len)
{
	for (size_t i = 0; i < buffer_len; i++)
	{
		PRINTF("%.3f, ", buffer[i]);
		if (i%20 == 19)
		{
			PRINTF("\r\n");
		}
	}
	PRINTF("\r\n\n");
}

void write_buffer_data_to_file_16(int16_t * buffer, size_t buffer_len)
{
	PRINTF("$");
	for (size_t i = 0; i < buffer_len; i++)
	{
		PRINTF("%d, ", buffer[i]);
		if (i%20 == 19)
		{
			PRINTF("\r\n");
		}
	}
}

void test_drc_algorithm(void (*algorithm_func)(float32_t *, float32_t *, size_t), float32_t * src_buffer, float32_t * dst_buffer,
		size_t buffer_len, uint32_t fs)
{
	float freq[] 	= {9000.0f, 7000.0f, 1500.0f};
	float amp[] 	= {0.1f, 0.2f, 2.0f};
	int freq_cnt = sizeof(freq) / sizeof(freq[0]);

	generate_sine_wave_f32(&src_buffer[0], buffer_len/5, fs, (float)INT16_MAX * 0.15, freq, amp, freq_cnt);
	generate_sine_wave_f32(&src_buffer[1000], buffer_len/5, fs, (float)INT16_MAX * 0.40, freq, amp, freq_cnt);
	generate_sine_wave_f32(&src_buffer[2000], buffer_len/5, fs, (float)INT16_MAX * 0.65, freq, amp, freq_cnt);
	generate_sine_wave_f32(&src_buffer[3000], buffer_len/5, fs, (float)INT16_MAX * 0.8, freq, amp, freq_cnt);
	generate_sine_wave_f32(&src_buffer[4000], buffer_len/5, fs, (float)INT16_MAX * 0.95, freq, amp, freq_cnt);

	print_buffer_data_f32(src_buffer, buffer_len);
	algorithm_func(src_buffer, dst_buffer, buffer_len);
	print_buffer_data_f32(dst_buffer, buffer_len);
}

void test_hifi4_fir_q31(float32_t * src_buffer_f32, q31_t * src_buffer_q31, float32_t * dst_buffer_f32, q31_t * dst_buffer_q31,
		size_t buffer_len, uint32_t fs)
{
	uint32_t previous_time;
	float freq[] 	= {9000.0f, 7000.0f, 1500.0f};
	float amp[] 	= {0.1f, 0.2f, 2.0f};
	int freq_cnt = sizeof(freq) / sizeof(freq[0]);

	SEMA42_Lock(APP_SEMA42, SEMA42_GATE, PROC_NUM);

	MU_SetFlags(APP_MU, SEMA42_LOCK_FLAG);

	generate_sine_wave_f32((float32_t *)&src_buffer_f32[0], buffer_len/5, fs, 0.15f, freq, amp, freq_cnt);
	generate_sine_wave_f32((float32_t *)&src_buffer_f32[1*buffer_len/5], buffer_len/5, fs, 0.40f, freq, amp, freq_cnt);
	generate_sine_wave_f32((float32_t *)&src_buffer_f32[2*buffer_len/5], buffer_len/5, fs, 0.65f, freq, amp, freq_cnt);
	generate_sine_wave_f32((float32_t *)&src_buffer_f32[3*buffer_len/5], buffer_len/5, fs, 0.8f, freq, amp, freq_cnt);
	generate_sine_wave_f32((float32_t *)&src_buffer_f32[4*buffer_len/5], buffer_len/5, fs, 0.40f, freq, amp, freq_cnt);

	arm_float_to_q31((float32_t *)src_buffer_f32, (q31_t *)src_buffer_q31, buffer_len);
	print_buffer_data_f32((float32_t *)src_buffer_f32, buffer_len);

	SEMA42_Unlock(APP_SEMA42, SEMA42_GATE);

	previous_time = MSDK_GetCpuCycleCount();
	while (SEMA42_DSP_LOCK_FLAG != MU_GetFlags(APP_MU))
	{
	}

	SEMA42_Lock(APP_SEMA42, SEMA42_GATE, (uint8_t)1U);
	PRINTF("Execution time: %.3f ms\r\n", CYCLES_TO_MS(MSDK_GetCpuCycleCount() - previous_time));
	arm_q31_to_float((q31_t *)dst_buffer_q31, (float32_t *)dst_buffer_f32, buffer_len);
	print_buffer_data_f32((float32_t *)dst_buffer_f32, buffer_len);
}

void test_hifi4_fir_f32(float32_t * src_buffer_f32, float32_t * dst_buffer_f32, size_t buffer_len, uint32_t fs)
{
	uint32_t previous_time;
	float freq[] 	= {9000.0f, 7000.0f, 1500.0f};
	float amp[] 	= {0.1f, 0.2f, 2.0f};
	int freq_cnt = sizeof(freq) / sizeof(freq[0]);

	SEMA42_Lock(APP_SEMA42, SEMA42_GATE, PROC_NUM);

	MU_SetFlags(APP_MU, SEMA42_LOCK_FLAG);

	generate_sine_wave_f32((float32_t *)&src_buffer_f32[0], buffer_len/5, fs, 0.15f, freq, amp, freq_cnt);
	generate_sine_wave_f32((float32_t *)&src_buffer_f32[1*buffer_len/5], buffer_len/5, fs, 0.40f, freq, amp, freq_cnt);
	generate_sine_wave_f32((float32_t *)&src_buffer_f32[2*buffer_len/5], buffer_len/5, fs, 0.65f, freq, amp, freq_cnt);
	generate_sine_wave_f32((float32_t *)&src_buffer_f32[3*buffer_len/5], buffer_len/5, fs, 0.8f, freq, amp, freq_cnt);
	generate_sine_wave_f32((float32_t *)&src_buffer_f32[4*buffer_len/5], buffer_len/5, fs, 0.40f, freq, amp, freq_cnt);

	print_buffer_data_f32((float32_t *)src_buffer_f32, buffer_len);

	SEMA42_Unlock(APP_SEMA42, SEMA42_GATE);

	previous_time = MSDK_GetCpuCycleCount();
	while (SEMA42_DSP_LOCK_FLAG != MU_GetFlags(APP_MU))
	{
	}

	SEMA42_Lock(APP_SEMA42, SEMA42_GATE, (uint8_t)1U);
	PRINTF("Execution time: %.3f ms\r\n", CYCLES_TO_MS(MSDK_GetCpuCycleCount() - previous_time));
	print_buffer_data_f32((float32_t *)dst_buffer_f32, buffer_len);
}
