/*
 * algorithm_testbench.h
 *
 *  Created on: 26 cze 2023
 *      Author: Łukasz
 */

#ifndef ALGORITHM_TESTBENCH_H_
#define ALGORITHM_TESTBENCH_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "arm_math.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CYCLES_TO_MS(c)					((c) * 1000.0 / (SystemCoreClock))
#define CYCLES_TO_US(c)					((c) * 1000000.0 / (SystemCoreClock))

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void measure_algorithm_time_f32(void (*algorithm_func)(float32_t *, float32_t *, size_t), float32_t * src_buffer, float32_t * dst_buffer, size_t bytes, uint32_t iterations);
void test_pq_math(float * arr, uint32_t iterations);
void print_buffer_data_16(int16_t * buffer, size_t buffer_len);
void print_buffer_data_32(int32_t * buffer, size_t buffer_len);
void test_drc_algorithm(void (*algorithm_func)(float32_t *, float32_t *, size_t), float32_t * src_buffer, float32_t * dst_buffer, size_t buffer_len, uint32_t fs);
void print_buffer_data_f32(float32_t * buffer, size_t buffer_len);
void test_cmsis_dsp_f32(float32_t * src_arr, float32_t * dst_arr, uint32_t arr_len, uint32_t fs, uint32_t iterations);
void generate_sine_wave_f32(float32_t * input_vec, uint32_t vec_len, uint32_t fs, float max_amplitude, float * freq, float * amp, int freq_cnt);
void generate_sine_wave_16(int16_t * input_vec, uint32_t vec_len, uint32_t fs, float max_amplitude, float * freq, float * amp, int freq_cnt);
void test_hifi4_fir_q31(float32_t * src_buffer_f32, q31_t * src_buffer_q31, float32_t * dst_buffer_f32, q31_t * dst_buffer_q31, size_t buffer_len, uint32_t fs);
void test_hifi4_fir_f32(float32_t * src_buffer_f32, float32_t * dst_buffer_f32, size_t buffer_len, uint32_t fs);

/*******************************************************************************
 * Exported variables
 ******************************************************************************/

#endif /* ALGORITHM_TESTBENCH_H_ */
