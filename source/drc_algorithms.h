/*
 * drc_algorithms_cm33.h
 *
 *  Created on: 22 may 2023
 *      Author: Łukasz
 */

#ifndef DRC_ALGORITHMS_H_
#define DRC_ALGORITHMS_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "main_cm33.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void calculate_coefficients(void);
void check_coefficients(float32_t log2_LT, float32_t  log2_CT, float32_t  log2_ET, float32_t  log2_NT, float32_t  log2_NT_MUTE);
void limiter_f32(float32_t * src_signal_arr, float32_t * dst_signal_arr, size_t signal_arr_count);
void limiter_32(int32_t * src_signal_arr, int32_t * dst_signal_arr, size_t signal_arr_count);
void compressor_expander_ngate_32(int32_t * src_signal_arr, int32_t * dst_signal_arr, size_t signal_arr_count);
void compressor_expander_ngate_f32(float32_t * src_signal_arr, float32_t * dst_signal_arr, size_t signal_arr_count);
void drc_process_f32(float32_t * src_signal_arr, float32_t * dst_signal_arr, size_t signal_arr_count);
#ifdef Q31_USED
void compressor_expander_ngate_q31(q31_t * src_signal_arr, q31_t * dst_signal_arr, size_t signal_arr_count, q31_t * x_rms_log2_array);
void limiter_q31(q31_t * src_signal_arr, q31_t * dst_signal_arr, size_t signal_arr_count);
void drc_process_q31(q31_t * src_signal_arr, q31_t * dst_signal_arr, size_t signal_arr_count);

arm_status arm_divide_q31(q31_t numerator, q31_t denominator, q31_t *quotient, int16_t *shift);
#endif

#endif /* DRC_ALGORITHMS_H_ */
