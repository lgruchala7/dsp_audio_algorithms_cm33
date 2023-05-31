/*
 * drc_algorithms_cm33.h
 *
 *  Created on: 22 may 2023
 *      Author: Łukasz
 */

#ifndef DRC_ALGORITHMS_CM33_H_
#define DRC_ALGORITHMS_CM33_H_

void limiter(uint8_t * music_arr, size_t music_arr_count);
void compressor_expander_ngate(uint8_t * music_arr, size_t music_arr_count);
void calculate_coefficients(void);

#endif /* DRC_ALGORITHMS_CM33_H_ */
