/*
 * drc_algorithms_cm33.h
 *
 *  Created on: 22 may 2023
 *      Author: Łukasz
 */

#include "drc_algorithms_cm33_conf.h"

#ifndef DRC_ALGORITHMS_CM33_H_
#define DRC_ALGORITHMS_CM33_H_

uint32_t limiter(uint8_t * music_arr, size_t music_arr_count);
uint32_t compressor_expander_ngate(uint8_t * music_arr, size_t music_arr_count);
void calculate_coefficients(void);

#endif /* DRC_ALGORITHMS_CM33_H_ */
