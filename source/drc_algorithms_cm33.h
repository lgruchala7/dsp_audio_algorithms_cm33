/*
 * drc_algorithms_cm33.h
 *
 *  Created on: 22 may 2023
 *      Author: Łukasz
 */

#ifndef DRC_ALGORITHMS_CM33_H_
#define DRC_ALGORITHMS_CM33_H_

void limiter_16(int16_t * src_signal_arr, int16_t * dst_signal_arr, size_t signal_arr_count);
void compressor_expander_ngate_16(int16_t * src_signal_arr, int16_t * dst_signal_arr, size_t signal_arr_count);
void fir_filter_16(int16_t * src_signal_arr, int16_t * dst_signal_arr, size_t signal_arr_count);
void calculate_coefficients(void);
void check_coeff_validity(void);

#endif /* DRC_ALGORITHMS_CM33_H_ */
