/*
 * drc_algorithms_cm33_conf.h
 *
 *  Created on: 22 maj 2023
 *      Author: Łukasz
 */

#ifndef DRC_ALGORITHMS_CM33_CONF_H_
#define DRC_ALGORITHMS_CM33_CONF_H_

#define _16_bit
#define PQ_USED

/* attack time */
const float t_at_ms 	= 0.1f;
/* release time */
const float t_re_ms 	= 5.0f;
/* averaging time */
const float t_tav_ms 	= 0.1f;
/* sampling frequency in Hz*/
const float fs_hz 		= 44100.0f;
/* limiter slope */
const float LS 			= 1.0f;
/* compressor slope */
const float CS 			= 0.9f;
/* expander slope */
const float ES 			= -0.9f;
/* noise gate slope */
const float NS 			= -10.0f;

#ifdef _16_bit
/* limiter threshold */
const float LT = 51400.0f;
/* compressor threshold */
const float CT = 38550.0f;
/* expander threshold */
const float ET = 25700.0f;
/* noise gate threshold */
const float NT = 12850.0f;
#else
const float LT = 200.0f;
const float CT = 150.0f;
const float ET = 100.0f;
const float NT = 50.0f;
#endif


#endif /* DRC_ALGORITHMS_CM33_CONF_H_ */
