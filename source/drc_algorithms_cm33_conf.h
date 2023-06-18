/*
 * drc_algorithms_cm33_conf.h
 *
 *  Created on: 22 maj 2023
 *      Author: Łukasz
 */

#ifndef DRC_ALGORITHMS_CM33_CONF_H_
#define DRC_ALGORITHMS_CM33_CONF_H_

#define _16_bit

/* attack time */
#define t_at_ms		0.1f
/* release time */
#define t_re_ms 	5.0f
/* averaging time */
#define t_tav_ms	0.1f
/* sampling frequency in Hz*/
#define fs_hz		44100.0f
/* limiter threshold */
#ifdef _16_bit
#define LT			51400.0f
#else
#define LT			200.0f
#endif
/* limiter slope */
#define LS			1.0f
/* compressor threshold */
#define CT			38550.0f
//#define CT			150.0f
/* compressor slope */
#define CS			0.9f
/* expander threshold */
#ifdef _16_bit
#define ET			25700.0f
#else
#define ET			100.0f
#endif
/* expander slope */
#define ES			-0.9f
/* noise gate threshold */
#ifdef _16_bit
#define NT			12850.0f
#else
#define NT			50.0f
#endif
/* noise gate slope */
#define NS			-10.0f

#endif /* DRC_ALGORITHMS_CM33_CONF_H_ */
