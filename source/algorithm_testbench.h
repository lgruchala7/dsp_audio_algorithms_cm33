/*
 * algorithm_testbench.h
 *
 *  Created on: 26 cze 2023
 *      Author: Łukasz
 */

#ifndef ALGORITHM_TESTBENCH_H_
#define ALGORITHM_TESTBENCH_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MATCH_VAL_MS					0.1
#define CTIMER_TICKS_TO_MS(t)			((t) * (MATCH_VAL_MS))
#define CTIMER_TICKS_TO_US(t)			((t) * (MATCH_VAL_MS) * 1000.0)
#define CYCLES_TO_MS(c)					((c) * 1000.0 / (SystemCoreClock))
#define CYCLES_TO_US(c)					((c) * 1000000.0 / (SystemCoreClock))

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void measure_algorithm_time_u16(void (*algorithm_func)(uint16_t *, uint16_t *, size_t), uint16_t * src_buffer, uint16_t * dst_buffer, size_t buffer_size, uint32_t iterations);
void test_pq_math(float * arr, uint32_t iterations);
void print_buffer_data_u16(uint16_t * data, size_t data_size);
void test_algorithm(void (*algorithm_func)(uint16_t *, uint16_t *, size_t), uint16_t * src_buffer, uint16_t * dst_buffer, size_t buffer_size, uint32_t fs);
/*******************************************************************************
 * Exported variables
 ******************************************************************************/
extern volatile unsigned long ctimer_ticks;

#endif /* ALGORITHM_TESTBENCH_H_ */
