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
#if Q31_USED
void compressor_expander_ngate(q31_t * src_signal_arr, q31_t * dst_signal_arr, size_t signal_arr_count, q31_t * x_rms_log2_array);
void limiter(q31_t * src_signal_arr, q31_t * dst_signal_arr, size_t signal_arr_count);
void drc_full(q31_t * src_signal_arr, q31_t * dst_signal_arr, size_t signal_arr_count, q31_t * x_rms_log2_array);
void drc_full_stereo_balanced(q31_t * src_signal_arr, q31_t * dst_signal_arr, q31_t * x_peak_log2_array);
arm_status arm_divide_q31(q31_t numerator, q31_t denominator, q31_t *quotient, int16_t *shift);
#else
void limiter(float32_t * src_signal_arr, float32_t * dst_signal_arr, size_t signal_arr_count);
void compressor_expander_ngate(float32_t * src_signal_arr, float32_t * dst_signal_arr, size_t signal_arr_count);
void drc_full(float32_t * src_signal_arr, float32_t * dst_signal_arr, size_t signal_arr_count, float32_t * x_rms_log2_array);
void drc_full_stereo_balanced(float32_t * src_signal_arr, float32_t * dst_signal_arr, float32_t * x_peak_log2_array);
#endif

#endif /* DRC_ALGORITHMS_H_ */
