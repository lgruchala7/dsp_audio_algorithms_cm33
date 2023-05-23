/*
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    drc_algorithms_cm33.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MIMXRT685S_cm33.h"
#include "fsl_debug_console.h"
#include <math.h>

#include "drc_algorithms_cm33.h"

#define LOG2(x) 	(logf(x) / logf(2.0f))

#define OK		0U
#define NOT_OK 	1U

static float AT;
static float RT;
static float TAV;
static float log_2_LT;

void calculate_coefficients(void)
{
	AT = (1.0 - exp(-2.2 * 1000.0 / (t_at_ms * fs_hz)));
	RT = (1.0 - exp(-2.2 * 1000.0 / (t_re_ms * fs_hz)));
	TAV = (1.0 - exp((-2.2 * 1000.0) / (t_tav_ms * fs_hz)));
	log_2_LT = LOG2(LT);
}

uint32_t limiter(uint8_t * signal_arr, size_t signal_arr_count)
{
	float x_peak = 0.0f;
	float x_peak_log = 0.0f;
	float ctrl_factor = 0.0f;
	float ctrl_factor_old = 0.0f;
	float ctrl_factor_smooth = 0.0f;
	float k;
	uint32_t ret_val = NOT_OK;

    for (uint32_t i = 0U; i < signal_arr_count; i++)
    {
    	if ((float)signal_arr[i] > x_peak)
    	{
    		x_peak = (1.0 - AT) * x_peak + AT * signal_arr[i];
    	}
    	else
    	{
    		x_peak = (1.0 - RT) * x_peak;
    	}

    	x_peak_log = LOG2(x_peak);

    	if (x_peak_log > log_2_LT)
    	{
    		ctrl_factor = pow(2.0, -LS * (x_peak_log - log_2_LT));
    	}
    	else
    	{
    		ctrl_factor = 1.0;
    	}

    	if (ctrl_factor < ctrl_factor_old)
    	{
    		k = RT;
    	}
    	else
    	{
    		k = AT;
    	}

    	ctrl_factor_smooth = (1 - k) * ctrl_factor_smooth + k * ctrl_factor;
    	ctrl_factor_old = ctrl_factor;

    	signal_arr[i] = (uint8_t)(signal_arr[i] * ctrl_factor_smooth);
    }

	ret_val = OK;

    return ret_val;
}

uint32_t compressor_expander_ngate(uint8_t * signal_arr, size_t signal_arr_count)
{
	float x_rms_2 = 0.0f;
	float x_rms_log;
	float ctrl_factor = 0.0f;
	float ctrl_factor_old = 0.0f;
	float ctrl_factor_smooth = 0.0f;
	float k;
	uint32_t ret_val = NOT_OK;


    for (uint32_t i = 0U; i < signal_arr_count; i++)
    {
    	x_rms_2 = ((1.0 - TAV) * x_rms_2 + TAV * signal_arr[i] * signal_arr[i]);
    	x_rms_log = 0.5 * LOG2(x_rms_2);

    	if (x_rms_log > LOG2(CT))
    	{
    		ctrl_factor = pow(2.0, -CS * (x_rms_log - LOG2(CT)));
    	}
    	else if (x_rms_log <= LOG2(CT) && x_rms_log >= LOG2(ET))
    	{
    		ctrl_factor = 1.0;
    	}
    	else if (x_rms_log < LOG2(ET) && x_rms_log >= LOG2(NT))
    	{
    		ctrl_factor = pow(2.0, -ES * (x_rms_log - LOG2(ET)));
    	}
    	else
    	{
    		ctrl_factor = pow(2.0, -NS * (x_rms_log - LOG2(NT)) + ES * (LOG2(ET) - LOG2(NT)));
    	}

    	if (ctrl_factor < ctrl_factor_old)
    	{
    		k = RT;
    	}
    	else
    	{
    		k = AT;
    	}

    	ctrl_factor_smooth = (1 - k) * ctrl_factor_smooth + k * ctrl_factor;
    	ctrl_factor_old = ctrl_factor;

    }

	ret_val = OK;

	return ret_val;
}
