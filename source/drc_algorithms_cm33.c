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
 * @brief   DSP algorithms implementations.
 */

#include "fsl_debug_console.h"
#include <math.h>

#include "drc_algorithms_cm33.h"
#include "drc_algorithms_cm33_conf.h"

#include "fsl_powerquad.h"
#include "fsl_power.h"

#define OK		0U
#define NOT_OK 	1U

static float AT = 0.0f;
static float RT = 0.0f;
static float TAV = 0.0f;
static float log2_LT = 0.0f;
static float log2_CT = 0.0f;
static float log2_ET = 0.0f;
static float log2_NT = 0.0f;
static float diff_ET_NT = 0.0f;
static float ES_times_diff_ET_NT = 0.0f;
static float one_minus_AT = 0.0f;
static float one_minus_RT = 0.0f;
static float one_minus_TAV = 0.0f;

const static float LN_OF_2 = 0.693147f;
const static uint16_t const_comp = (UINT16_MAX / 2);

#ifdef PQ_USED
#define LOG2(p_src, p_dst) 	do { \
									PQ_LnF32(p_src, p_dst); \
									PQ_DivF32(p_dst, (float *)&LN_OF_2, p_dst); \
								} while (0)
#else
#define LOG2(x) 				(logf(x) / LN_OF_2)

#endif

void check_coeff_validity(void)
{
	if (NT >= ET || ET >= CT || ES >= 0.0 || NS >= 0.0)
	{
		PRINTF("Wrong coefficient value defined\r\n");
		exit(1);
	}
}

void calculate_coefficients(void)
{
	AT = (float)(1.0 - exp(-2.2 * 1000.0 / (t_at_ms * fs_hz)));
	RT = (float)(1.0 - exp(-2.2 * 1000.0 / (t_re_ms * fs_hz)));
	TAV = (float)(1.0 - exp((-2.2 * 1000.0) / (t_tav_ms * fs_hz)));
#ifdef PQ_USED
	// casting to suppress discarded qualifier warning
	LOG2((float *)&LT, &log2_LT);
	LOG2((float *)&CT, &log2_CT);
	LOG2((float *)&ET, &log2_ET);
	LOG2((float *)&NT, &log2_NT);
#else
	log2_LT = LOG2(LT);
	log2_CT = LOG2(CT);
	log2_ET = LOG2(ET);
	log2_NT = LOG2(NT);
#endif
	one_minus_AT = (1.0f - AT);
	one_minus_RT = (1.0f - RT);
	one_minus_TAV = (1.0f - TAV);
	diff_ET_NT = (log2_ET - log2_NT);
	ES_times_diff_ET_NT = (ES * diff_ET_NT);
}

void limiter_u16(uint16_t * src_signal_arr, uint16_t * dst_signal_arr, size_t signal_arr_count)
{
	float x_peak = 0.0f;
	float x_peak_log = 0.0f;
	float ctrl_factor = 0.0f;
	float ctrl_factor_sqrt = 0.0f;
	float ctrl_factor_old = 0.0f;
	float ctrl_factor_smooth = 0.0f;
	float k = 0.0f;

	/* left channel */
    for (uint32_t i = 0U; i < signal_arr_count; i += 2)
    {
    	if ((float)src_signal_arr[i] > x_peak)
    	{
    		x_peak = one_minus_AT * x_peak + AT * src_signal_arr[i];
    	}
    	else
    	{
    		x_peak = one_minus_RT * x_peak;
    	}

#ifdef PQ_USED
    	LOG2(&x_peak, &x_peak_log);
#else
    	x_peak_log = LOG2(x_peak);
#endif

    	if (x_peak_log > log2_LT)
    	{
    		ctrl_factor_sqrt = -LS * (x_peak_log - log2_LT);
    		ctrl_factor = ctrl_factor_sqrt * ctrl_factor_sqrt;
    	}
    	else
    	{
    		ctrl_factor = 1.0f;
    	}

    	if (ctrl_factor < ctrl_factor_old)
    	{
    		k = RT;
    	}
    	else
    	{
    		k = AT;
    	}

    	ctrl_factor_smooth = (1.0f - k) * ctrl_factor_smooth + k * ctrl_factor;
    	ctrl_factor_old = ctrl_factor;

    	dst_signal_arr[i] = (uint16_t)(src_signal_arr[i] * ctrl_factor_smooth);
    }

    x_peak = 0.0f;
    ctrl_factor_old = 0.0f;

    /* right channel */
    for (uint32_t i = 1U; i < signal_arr_count; i += 2)
    {
    	if ((float)src_signal_arr[i] > x_peak)
    	{
    		x_peak = one_minus_AT * x_peak + AT * src_signal_arr[i];
    	}
    	else
    	{
    		x_peak = one_minus_RT * x_peak;
    	}

#ifdef PQ_USED
    	LOG2(&x_peak, &x_peak_log);
#else
    	x_peak_log = LOG2(x_peak);
#endif

    	if (x_peak_log > log2_LT)
    	{
    		ctrl_factor_sqrt = -LS * (x_peak_log - log2_LT);
    		ctrl_factor = ctrl_factor_sqrt * ctrl_factor_sqrt;
    	}
    	else
    	{
    		ctrl_factor = 1.0f;
    	}

    	if (ctrl_factor < ctrl_factor_old)
    	{
    		k = RT;
    	}
    	else
    	{
    		k = AT;
    	}

    	ctrl_factor_smooth = (1.0f - k) * ctrl_factor_smooth + k * ctrl_factor;
    	ctrl_factor_old = ctrl_factor;

    	dst_signal_arr[i] = (uint16_t)(src_signal_arr[i] * ctrl_factor_smooth);
    }
}

void compressor_expander_ngate_u16(uint16_t * src_signal_arr, uint16_t * dst_signal_arr, size_t signal_arr_count)
{
	float x_rms_pow2 = 0.0f;
	float x_rms_log = 0.0f;
	float ctrl_factor = 0.0f;
	float ctrl_factor_sqrt = 0.0f;
	float ctrl_factor_old = 0.0f;
	float ctrl_factor_smooth = 0.0f;
	float k = 0.0f;

	/* left channel */
    for (uint32_t i = 0U; i < signal_arr_count; i += 2U)
    {
    	x_rms_pow2 = (one_minus_TAV * x_rms_pow2 + TAV * src_signal_arr[i] * src_signal_arr[i]);

#ifdef PQ_USED
    	LOG2(&x_rms_pow2, &x_rms_log);
    	x_rms_log *= 0.5f;
#else
    	x_rms_log = 0.5f * LOG2(x_rms_pow2);
#endif

    	if (x_rms_log > log2_CT)
    	{
    		ctrl_factor_sqrt = -CS * (x_rms_log - log2_CT);
    		ctrl_factor = ctrl_factor_sqrt * ctrl_factor_sqrt;
    	}
    	else if (x_rms_log >= log2_ET)
    	{
    		ctrl_factor = 1.0f;
    	}
    	else if (x_rms_log >= log2_NT)
    	{
    		ctrl_factor_sqrt = -ES * (x_rms_log - log2_ET);
    		ctrl_factor = ctrl_factor_sqrt * ctrl_factor_sqrt;
    	}
    	else
    	{
    		ctrl_factor_sqrt = -NS * (x_rms_log - log2_NT) + ES_times_diff_ET_NT;
    		ctrl_factor = ctrl_factor_sqrt * ctrl_factor_sqrt;
    	}

    	if (ctrl_factor < ctrl_factor_old)
    	{
    		k = RT;
    	}
    	else
    	{
    		k = AT;
    	}

    	ctrl_factor_smooth = (1.0f - k) * ctrl_factor_smooth + k * ctrl_factor;
    	ctrl_factor_old = ctrl_factor;

    	dst_signal_arr[i] = (uint16_t)(src_signal_arr[i] * ctrl_factor_smooth);
    }

	x_rms_pow2 = 0.0f;
	ctrl_factor_old = 0.0f;
	ctrl_factor_smooth = 0.0f;

	/* right channel */
    for (uint32_t i = 1U; i < signal_arr_count; i += 2U)
    {
    	x_rms_pow2 = (one_minus_TAV * x_rms_pow2 + TAV * src_signal_arr[i] * src_signal_arr[i]);

#ifdef PQ_USED
    	LOG2(&x_rms_pow2, &x_rms_log);
    	x_rms_log *= 0.5f;
#else
    	x_rms_log = 0.5f * LOG2(x_rms_pow2);
#endif

    	if (x_rms_log > log2_CT)
    	{
    		ctrl_factor_sqrt = -CS * (x_rms_log - log2_CT);
    		ctrl_factor = ctrl_factor_sqrt * ctrl_factor_sqrt;
    	}
    	else if (x_rms_log >= log2_ET)
    	{
    		ctrl_factor = 1.0f;
    	}
    	else if (x_rms_log >= log2_NT)
    	{
    		ctrl_factor_sqrt = -ES * (x_rms_log - log2_ET);
    		ctrl_factor = ctrl_factor_sqrt * ctrl_factor_sqrt;
    	}
    	else
    	{
    		ctrl_factor_sqrt = -NS * (x_rms_log - log2_NT) + ES_times_diff_ET_NT;
    		ctrl_factor = ctrl_factor_sqrt * ctrl_factor_sqrt;
    	}

    	if (ctrl_factor < ctrl_factor_old)
    	{
    		k = RT;
    	}
    	else
    	{
    		k = AT;
    	}

    	ctrl_factor_smooth = (1.0f - k) * ctrl_factor_smooth + k * ctrl_factor;
    	ctrl_factor_old = ctrl_factor;

    	dst_signal_arr[i] = (uint16_t)(src_signal_arr[i] * ctrl_factor_smooth);
    }
}

void fir_filter_u16(uint16_t * src_signal_arr, uint16_t * dst_signal_arr, size_t signal_arr_count)
{
	static uint16_t prev_samples[FIR_ORDER * 2] = { [0 ... ((FIR_ORDER * 2) - 1)] = const_comp };
	uint32_t old_samples = FIR_ORDER;
	float temp_sum = 0.0f;

	/* left channel */
	for (uint32_t i = 0U; i < signal_arr_count; i += 2U)
	{
    	if (i == 40U)
    	{
    		__NOP();
    	}
		for (uint32_t j = 0; j < FIR_COEFF_COUNT; j++)
		{
			if (j < old_samples)
			{
				temp_sum += prev_samples[i + (2 * j)] * fir_filter_coeff[j];
			}
			else
			{
				temp_sum += src_signal_arr[i - (2 * (FIR_ORDER - j))] * fir_filter_coeff[j];
			}
		}
		dst_signal_arr[i] = (uint16_t)temp_sum;
		temp_sum = 0.0f;
		(old_samples > 0) ? old_samples-- : 0u;
	}

	old_samples = FIR_ORDER;

	/* right channel */
	for (uint32_t i = 1U; i < signal_arr_count; i += 2U)
	{
		for (int32_t j = 0; j < FIR_COEFF_COUNT; j++)
		{
			if (j < old_samples)
			{
				temp_sum += prev_samples[i + (2 * j)] * fir_filter_coeff[j];
			}
			else
			{
				temp_sum += src_signal_arr[i - (2 * (FIR_ORDER - j))] * fir_filter_coeff[j];
			}
		}
		dst_signal_arr[i] = (uint16_t)temp_sum;
		temp_sum = 0.0f;
		(old_samples > 0) ? old_samples-- : 0u;
	}

	memcpy(&prev_samples[0], &src_signal_arr[signal_arr_count - (2 * FIR_ORDER)], (2 * FIR_ORDER * sizeof(src_signal_arr[0])));
}
