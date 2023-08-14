/*
 * filters_cfg.h
 *
 *  Created on: 11 sie 2023
 *      Author: Łukasz
 */

#ifndef FILTERS_CFG_H_
#define FILTERS_CFG_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "main_cm33.h"
#include "arm_math.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FIR_ORDER			32U
#define FIR_COEFF_COUNT		(FIR_ORDER + 1U)
#define FIR_BLOCK_SIZE		16U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void init_fir_filters(void);
#ifndef Q31_USED
void fir_process_batch(float32_t * src_buffer, float32_t * dst_buffer, size_t buffer_size);
#else
void fir_process_batch(q31_t * src_buffer, q31_t * dst_buffer, size_t buffer_size);
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern const float32_t fir_filter_coeff_f32[FIR_COEFF_COUNT];
#ifndef Q31_USED
extern arm_fir_instance_f32 fir_instance_f32_1;
extern arm_fir_instance_f32 fir_instance_f32_2;
extern float32_t fir_state_f32_1[FIR_BLOCK_SIZE + FIR_COEFF_COUNT - 1];
extern float32_t fir_state_f32_2[FIR_BLOCK_SIZE + FIR_COEFF_COUNT - 1];
#else
extern arm_fir_instance_q31 fir_instance_q31_1;
extern arm_fir_instance_q31 fir_instance_q31_2;
extern q31_t fir_state_q31_1[FIR_BLOCK_SIZE + FIR_COEFF_COUNT - 1];
extern q31_t fir_state_q31_2[FIR_BLOCK_SIZE + FIR_COEFF_COUNT - 1];
extern q31_t fir_filter_coeff_q31[FIR_COEFF_COUNT];
#endif

#endif /* FILTERS_CFG_H_ */
