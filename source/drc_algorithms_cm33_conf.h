/*
 * drc_algorithms_cm33_conf.h
 *
 *  Created on: 22 maj 2023
 *      Author: Łukasz
 */

#ifndef DRC_ALGORITHMS_CM33_CONF_H_
#define DRC_ALGORITHMS_CM33_CONF_H_

#define t_at_ms		0.1
#define t_re_ms 	5.0
#define t_tav_ms	0.1
#define fs_hz		44100.0
#define AT 			(1.0 - exp(-2.2 * 1000.0 / (t_at_ms * fs_hz)))
#define RT 			(1.0 - exp(-2.2 * 1000.0 / (t_re_ms * fs_hz)))
#define TAV			(1.0 - exp((-2.2 * 1000.0) / (t_tav_ms * fs_hz)))
#define LT			200.0
#define LS			1.0
#define CT			150.0
#define CS			0.9
#define ET			100.0
#define ES			-0.9
#define NT			50.0
#define NS			-10.0

#endif /* DRC_ALGORITHMS_CM33_CONF_H_ */
