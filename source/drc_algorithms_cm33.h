/*
 * drc_algorithms_cm33.h
 *
 *  Created on: 22 may 2023
 *      Author: Łukasz
 */

#ifndef DRC_ALGORITHMS_CM33_H_
#define DRC_ALGORITHMS_CM33_H_

void limiter(uint8_t * signal_arr, size_t signal_arr_count);
void limiter_8(uint8_t * signal_arr, size_t signal_arr_count);
void limiter_16(uint16_t * signal_arr, size_t signal_arr_count);
void compressor_expander_ngate_8(uint8_t * signal_arr, size_t signal_arr_count);
void compressor_expander_ngate_16(uint16_t * signal_arr, size_t signal_arr_count);
void calculate_coefficients(void);

#endif /* DRC_ALGORITHMS_CM33_H_ */
