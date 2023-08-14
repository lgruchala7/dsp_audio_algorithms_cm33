/*
 * drc_algorithms_cm33_cfg.h
 *
 *  Created on: 22 maj 2023
 *      Author: Łukasz
 */

#ifndef DRC_ALGORITHMS_CM33_CFG_H_
#define DRC_ALGORITHMS_CM33_CFG_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "arm_math_types.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PQ_USED
//#define _16bit

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* attack time */
#define t_at_ms 	1.0f
/* attack time ctrl_factor */
#define t_at_cf_ms 	0.1f
/* release time */
#define t_re_ms 	50.0f
/* release time ctrl_factor */
#define t_re_cf_ms 	5.0f
/* averaging time */
#define t_tav_ms 	3.33f
/* sampling frequency in Hz*/
#define fs_hz 		48000.0f
/* limiter slope */
#define LS 			1.0f
/* compressor slope */
#define CS 			0.9f
/* expander slope */
#define ES 			-0.9f
/* noise gate slope */
#define NS 			-10.0f

#ifdef _16bit
#define MAX_AMPL	INT16_MAX
#else
#define MAX_AMPL	INT32_MAX
#endif
#define HALF_SQRT_2	0.707f

/* limiter threshold */
#define LT 	((float32_t)(MAX_AMPL * 0.8f * HALF_SQRT_2))
/* compressor threshold */
#define CT 	((float32_t)(MAX_AMPL * 0.6f * HALF_SQRT_2))
/* expander threshold */
#define ET 	((float32_t)(MAX_AMPL * 0.4f * HALF_SQRT_2))
/* noise gate threshold */
#define NT 	((float32_t)(MAX_AMPL * 0.2f * HALF_SQRT_2))

#endif /* DRC_ALGORITHMS_CM33_CFG_H_ */