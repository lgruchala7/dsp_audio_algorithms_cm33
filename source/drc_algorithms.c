/**
 * @file    drc_algorithms_cm33.c
 * @brief   DRC algorithms implementations.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_debug_console.h"
#include <math.h>

#include "fsl_powerquad.h"

#include "drc_algorithms.h"
#include "drc_algorithms_cfg.h"
#include "main_cm33.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#if Q31_USED
/* LUT constants */
#define TABLE_SIZE      	4096
#define SCALE_FACTOR	   	8
#define SCALE_FACTOR_SHIFT  3 /* log2(SCALE_FACTOR) */
#define LUT_MIN_LN         	0.006738f
#define LUT_MIN_LN_Q31     	0x00dcca71
#define LUT_MAX_LN         	1.0f
#define LUT_RANGE_LN       	0x7f23358f
#define LUT_RANGE_LN_F32   	(LUT_MAX_LN - LUT_MIN_LN)
#define LUT_MIN_EXP         -1.0f
#define LUT_MIN_EXP_Q31     0x80000000
#define LUT_MAX_EXP        	0.0f
#define LUT_RANGE_EXP      	0x80000000
#define LUT_RANGE_EXP_F32 	(LUT_MAX_EXP - LUT_MIN_EXP)
/* Q31 operations */
#define Q31_MUL(x, y)							((__SSAT((((q63_t) (x) * (y)) >> 32), 31)) << 1U)
#define Q31_ADD(x, y)							(__QADD((x), (y)))
#define Q31_SUB(x, y)							(__QSUB((x), (y)))
#define Q31_DIV(num, den, q_ptr, sh_ptr)		(arm_divide_q31((num), (den), (q_ptr), (sh_ptr)))
#define Q31_LOG2(src, result_ptr, shift_ptr) 	Q31_DIV(q31_ln(src), LN_OF_2_Q31, (result_ptr), (shift_ptr))
#define Q31_POW2(src, dst_ptr)					(*dst_ptr = q31_exp((src)))
#endif
#if PQ_USED
#define LOG2(src_ptr, dst_ptr) 	do { \
									PQ_LnF32((src_ptr), (dst_ptr)); \
									PQ_DivF32((dst_ptr), (float32_t *)&LN_OF_2, (dst_ptr)); \
								} while (0)
#else
#define LOG2(src_ptr, dst_ptr)	(*(dst_ptr) = (logf(*(src_ptr)) / LN_OF_2))
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
const static float32_t LN_OF_2 = 0.693147f;
#if !(Q31_USED)
static float32_t AT;
static float32_t RT;
static float32_t RT_ctrl_factor;
static float32_t AT_ctrl_factor;
static float32_t TAV;
static float32_t log2_LT;
static float32_t log2_CT;
static float32_t log2_ET;
static float32_t log2_NT;
static float32_t log2_NT_MUTE;
static float32_t CS_times_diff_CT_LT;
static float32_t ES_times_diff_ET_NT;
static float32_t one_minus_AT;
static float32_t one_minus_RT;
static float32_t one_minus_TAV;
#else
const static q31_t LN_OF_2_Q31 = (q31_t)0x58b90bfc;
static q31_t AT;
static q31_t RT;
static q31_t RT_ctrl_factor;
static q31_t AT_ctrl_factor;
static q31_t TAV;
static float32_t log2_LT_scaled_f32;
static float32_t log2_CT_scaled_f32;
static float32_t log2_ET_scaled_f32;
static float32_t log2_NT_scaled_f32;
static float32_t log2_NT_MUTE_scaled_f32;
static q31_t log2_LT_scaled;
static q31_t log2_CT_scaled;
static q31_t log2_ET_scaled;
static q31_t log2_NT_scaled;
static q31_t log2_NT_MUTE_scaled;
static q31_t pow2_ES_times_diff_ET_NT;
static q31_t pow2_CS_times_diff_CT_LT;
static q31_t one_minus_AT;
static q31_t one_minus_RT;
static q31_t one_minus_TAV;
#endif

#if Q31_USED
// Pre-calculated Q1.31 fixed-point natural logarithm values
static q31_t ln_lookup_table[TABLE_SIZE];
static q31_t exp_lookup_table[TABLE_SIZE];
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void check_coefficients(float32_t log2_LT, float32_t  log2_CT, float32_t  log2_ET, float32_t  log2_NT, float32_t  log2_NT_MUTE);
#if Q31_USED
static inline void peak_log2_calculate(q31_t src, q31_t * dst_ptr, uint32_t channel);
static inline void rms_log2_calculate(q31_t src, q31_t * dst_ptr, uint32_t channel);
static inline q31_t q31_ln(q31_t x_q31);
static inline q31_t q31_exp(q31_t x_q31);
#else
static inline void peak_log2_calculate(float32_t src, float32_t * dst_ptr, uint32_t channel);
static inline void rms_log2_calculate(float32_t src, float32_t * dst_ptr, uint32_t channel);
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
#if Q31_USED
static void initialize_ln_lut(void)
{
	/* fill LUT of ln(x) for range (LUT_MIN_LN, 1) / SCALE_FACTOR */
    for (int i = 0; i < TABLE_SIZE; i++)
    {
    	float32_t x = LUT_MIN_LN + ((float32_t)i / (float32_t)TABLE_SIZE) * LUT_RANGE_LN_F32;
    	ln_lookup_table[i] = (q31_t)((logf(x) / (float32_t)SCALE_FACTOR) * Q31_ONE_F32);
    }
}

static void initialize_exp_lut(void)
{
	/* fill LUT of ln(x) for range (-1, 0) */
    for (int i = 0; i < TABLE_SIZE; i++)
    {
    	float32_t x = LUT_MIN_EXP + ((float32_t)i / (float32_t)TABLE_SIZE) * LUT_RANGE_EXP_F32;
    	exp_lookup_table[i] = (q31_t)(expf(x) * Q31_ONE_F32);
    }
}
#endif

#if Q31_USED
#if PQ_USED
// Calculate natural logarithm using lookup table and interpolation
static inline q31_t q31_ln(q31_t x_q31)
{
	if (x_q31 < LUT_MIN_LN_Q31)
	{
		return Q31_MIN_ONE;
	}
	q31_t input_val = (q31_t)(x_q31 >> 15);
	q31_t result = PQ_LnFixed(input_val) << 15;

	return result;
}
#else
static inline q31_t q31_ln(q31_t x_q31)
{
    if (x_q31 <= 0)
    {
        // Handle invalid input (<= 0)
        return 0;
    }

    // Calculate index into the lookup table
    int index = (int)((q63_t)(x_q31 - LUT_MIN_LN_Q31) * TABLE_SIZE / LUT_RANGE_LN);

    if (x_q31 < LUT_MIN_LN_Q31)
    {
        return ln_lookup_table[0];
    }
    else if (index >= (TABLE_SIZE - 1))
    {
        return ln_lookup_table[TABLE_SIZE - 1];
    }

    // Linear interpolation
    q31_t y0 = ln_lookup_table[index];
    q31_t y1 = ln_lookup_table[index + 1];
    q31_t diff = x_q31 - (q31_t)((q63_t)index * LUT_RANGE_LN / TABLE_SIZE + LUT_MIN_LN_Q31);
    q31_t result = y0 + (q31_t)((y1 - y0) * (q63_t)diff * TABLE_SIZE / LUT_RANGE_LN);

    return result;
}
#endif
#endif

#if Q31_USED
#if PQ_USED
// Calculate exp() using lookup table and interpolation
static inline q31_t q31_exp(q31_t x_q31)
{

	q31_t input_val = (q31_t)(x_q31 >> 15);
	q31_t result = PQ_EtoxFixed(input_val) << 15;

	return result;
}
#else
static inline q31_t q31_exp(q31_t x_q31)
{

    // Calculate index into the lookup table
    int index = (int)((q63_t)(x_q31 - LUT_MIN_EXP_Q31) * TABLE_SIZE / LUT_RANGE_EXP);

    if (x_q31 < LUT_MIN_EXP_Q31)
    {
        return exp_lookup_table[0];
    }
    else if (index >= (TABLE_SIZE - 1))
    {
        return exp_lookup_table[TABLE_SIZE - 1];
    }

    // Linear interpolation
    q31_t y0 = exp_lookup_table[index];
    q31_t y1 = exp_lookup_table[index + 1];
    q31_t diff = x_q31 - (q31_t)((q63_t)index * LUT_RANGE_EXP / TABLE_SIZE + LUT_MIN_EXP_Q31);
    q31_t result = y0 + (q31_t)((y1 - y0) * (q63_t)diff * TABLE_SIZE / LUT_RANGE_EXP);

    return result;
}
#endif
#endif

static void check_coefficients(float32_t log2_LT, float32_t  log2_CT, float32_t  log2_ET, float32_t  log2_NT, float32_t  log2_NT_MUTE)
{
	if (NT >= ET || ET >= CT || CT >= LT)
	{
		PRINTF("Wrong threshold value(s) defined\r\n");
		exit(1);
	}
	#if !(Q31_USED)
	if ((LS != 1.0f) || (CS >= 1.0f) || (CS <= 0.0f) || (ES <= -1.0f) || (ES >= 0.0f) || (NS >= -1.0f) ||
		(ES_times_diff_ET_NT >= 0.0f))
	{
		PRINTF("Wrong slope value(s) defined\r\n");
		exit(1);
	}
	#else
	if (((uint32_t)LS_INV != (uint32_t)Q31_ONE) ||
		IS_IN_RANGE_INCL((uint32_t)CS, (uint32_t)Q31_ONE, (uint32_t)Q31_ZERO) ||
		IS_IN_RANGE_INCL((uint32_t)ES, (uint32_t)Q31_ZERO, (uint32_t)Q31_ONE) ||
		IS_IN_RANGE_INCL((uint32_t)NS_INV, (uint32_t)Q31_ZERO, (uint32_t)Q31_ONE) ||
		IS_IN_RANGE_INCL((uint32_t)pow2_ES_times_diff_ET_NT, (uint32_t)Q31_ONE, (uint32_t)Q31_ZERO) ||
		IS_IN_RANGE_INCL((uint32_t)pow2_CS_times_diff_CT_LT, (uint32_t)Q31_ONE, (uint32_t)Q31_ZERO))
	{
		PRINTF("Wrong slope value(s) defined\r\n");
		exit(1);
	}
	if ((((log2_LT - 1.0f) / LS_INV) >= SCALE_FACTOR) ||
		((CS * (log2_CT - log2_LT)) >= SCALE_FACTOR) ||
		((ES * (log2_ET - log2_NT)) >= SCALE_FACTOR) ||
		(((log2_NT - log2_NT_MUTE) / NS_INV) >= SCALE_FACTOR))
	{
		PRINTF("Wrong ranges value(s) defined\r\n");
	}
	#endif
}

void calculate_coefficients(void)
{
	#if !(Q31_USED)
	float32_t diff_ET_NT, diff_CT_LT;

	AT = (float32_t)(1.0 - exp(-2.2 * 1000.0 / (t_at_ms * fs_hz)));
	RT = (float32_t)(1.0 - exp(-2.2 * 1000.0 / (t_re_ms * fs_hz)));
	TAV = (float32_t)(1.0 - exp((-2.2 * 1000.0) / (t_tav_ms * fs_hz)));
	AT_ctrl_factor = (float32_t)(1.0 - exp(-2.2 * 1000.0 / ((t_at_cf_ms) * fs_hz)));
	RT_ctrl_factor = (float32_t)(1.0 - exp(-2.2 * 1000.0 / ((t_re_cf_ms) * fs_hz)));

	float32_t lt = LT, ct = CT, et = ET, nt = NT, nt_mute = NT_MUTE;

	LOG2(&lt, &log2_LT);
	LOG2(&ct, &log2_CT);
	LOG2(&et, &log2_ET);
	LOG2(&nt, &log2_NT);
	LOG2(&nt_mute, &log2_NT_MUTE);

	one_minus_AT = (1.0f - AT);
	one_minus_RT = (1.0f - RT);
	one_minus_TAV = (1.0f - TAV);
	diff_ET_NT = (log2_ET - log2_NT);
	diff_CT_LT = (log2_CT - log2_LT);
	ES_times_diff_ET_NT = (ES * diff_ET_NT);
	CS_times_diff_CT_LT = (CS * diff_CT_LT);
	#else
	float32_t AT_f32, RT_f32, TAV_f32, AT_ctrl_factor_f32, RT_ctrl_factor_f32;
	q31_t ES_times_diff_ET_NT, CS_times_diff_CT_LT;

	AT_f32 = (float32_t)(1.0 - expf(-2.2f * 1000.0f / (t_at_ms * fs_hz)));
	RT_f32 = (float32_t)(1.0 - expf(-2.2f * 1000.0f / (t_re_ms * fs_hz)));
	TAV_f32 = (float32_t)(1.0 - expf((-2.2f * 1000.0f) / (t_tav_ms * fs_hz)));
	AT_ctrl_factor_f32 = (float32_t)(1.0 - expf(-2.2f * 1000.0f / ((t_at_ms) * fs_hz)));
	RT_ctrl_factor_f32 = (float32_t)(1.0 - expf(-2.2 * 1000.0f / ((t_re_ms) * fs_hz)));

	arm_float_to_q31(&AT_f32, &AT, 1);
	arm_float_to_q31(&RT_f32, &RT, 1);
	arm_float_to_q31(&TAV_f32, &TAV, 1);
	arm_float_to_q31(&AT_ctrl_factor_f32, &AT_ctrl_factor, 1);
	arm_float_to_q31(&RT_ctrl_factor_f32, &RT_ctrl_factor, 1);

	q31_t lt = LT, ct = CT, et = ET, nt = NT, nt_mute = NT_MUTE;
	float32_t lt_f32, ct_f32, et_f32, nt_f32, nt_mute_f32;
	float32_t log2_LT_f32, log2_CT_f32, log2_ET_f32, log2_NT_f32, log2_NT_MUTE_f32;

	arm_q31_to_float(&lt, &lt_f32, 1);
	arm_q31_to_float(&ct, &ct_f32, 1);
	arm_q31_to_float(&et, &et_f32, 1);
	arm_q31_to_float(&nt, &nt_f32, 1);
	arm_q31_to_float(&nt_mute, &nt_mute_f32, 1);

	LOG2(&lt_f32, &log2_LT_f32);
	LOG2(&ct_f32, &log2_CT_f32);
	LOG2(&et_f32, &log2_ET_f32);
	LOG2(&nt_f32, &log2_NT_f32);
	LOG2(&nt_mute_f32, &log2_NT_MUTE_f32);

	arm_scale_f32(&log2_LT_f32, (1.0f / (float32_t)SCALE_FACTOR), &log2_LT_scaled_f32, 1);
	arm_float_to_q31(&log2_LT_scaled_f32, &log2_LT_scaled, 1);

	arm_scale_f32(&log2_CT_f32, (1.0f / (float32_t)SCALE_FACTOR), &log2_CT_scaled_f32, 1);
	arm_float_to_q31(&log2_CT_scaled_f32, &log2_CT_scaled, 1);

	arm_scale_f32(&log2_ET_f32, (1.0f / (float32_t)SCALE_FACTOR), &log2_ET_scaled_f32, 1);
	arm_float_to_q31(&log2_ET_scaled_f32, &log2_ET_scaled, 1);

	arm_scale_f32(&log2_NT_f32, (1.0f / (float32_t)SCALE_FACTOR), &log2_NT_scaled_f32, 1);
	arm_float_to_q31(&log2_NT_scaled_f32, &log2_NT_scaled, 1);

	arm_scale_f32(&log2_NT_MUTE_f32, (1.0f / (float32_t)SCALE_FACTOR), &log2_NT_MUTE_scaled_f32, 1);
	arm_float_to_q31(&log2_NT_MUTE_scaled_f32, &log2_NT_MUTE_scaled, 1);

	one_minus_AT 	= Q31_SUB(Q31_ONE, AT);
	one_minus_RT	= Q31_SUB(Q31_ONE, RT);
	one_minus_TAV	= Q31_SUB(Q31_ONE, TAV);

	float32_t diff_ET_NT_f32 = log2_ET_f32 - log2_NT_f32;
	float32_t ES_times_diff_ET_NT_f32 = ES_F32 * diff_ET_NT_f32;
	float32_t pow2_ES_times_diff_ET_NT_f32 = powf(2.0f, ES_times_diff_ET_NT_f32);
	arm_float_to_q31(&pow2_ES_times_diff_ET_NT_f32, &pow2_ES_times_diff_ET_NT, 1);

	float32_t diff_CT_LT_f32 = log2_CT_f32 - log2_LT_f32;
	float32_t CS_times_diff_CT_LT_f32 = CS_F32 * diff_CT_LT_f32;
	float32_t pow2_CS_times_diff_CT_LT_f32 = powf(2.0f, CS_times_diff_CT_LT_f32);
	arm_float_to_q31(&pow2_CS_times_diff_CT_LT_f32, &pow2_CS_times_diff_CT_LT, 1);

	check_coefficients(log2_LT_f32, log2_CT_f32, log2_ET_f32, log2_NT_f32, log2_NT_MUTE_f32);

	initialize_ln_lut();
	initialize_exp_lut();
	#endif /* Q31_USED */
}

#if !(Q31_USED)

#endif

#if Q31_USED
static inline void peak_log2_calculate(q31_t src, q31_t * dst_ptr, uint32_t channel)
{
	static q31_t peak[CHANNEL_CNT] = {Q31_ZERO};
	q31_t abs_src;
	q31_t peak_log2_temp;
	int16_t shift;

	arm_abs_q31(&src, &abs_src, 1);

	if (abs_src > peak[channel])
	{
		peak[channel] = Q31_ADD(Q31_MUL(one_minus_AT, peak[channel]), Q31_MUL(AT, abs_src));
	}
	else
	{
		peak[channel] = Q31_MUL(one_minus_RT, peak[channel]);
	}

	Q31_LOG2(peak[channel], &peak_log2_temp, &shift);
	if (shift != 0)
	{
		arm_shift_q31(&peak_log2_temp, (int8_t)(shift), dst_ptr, 1);
	}
	else
	{
		*dst_ptr = peak_log2_temp;
	}
}

#else

static inline void peak_log2_calculate(float32_t src, float32_t * dst_ptr, uint32_t channel)
{
	static float32_t peak[CHANNEL_CNT] = {0.0f};
	float32_t peak_abs;

	float32_t abs_src = fabsf(src);

	if (abs_src > peak[channel])
	{
		peak[channel] = (one_minus_AT * peak[channel]) +  (AT * abs_src);
	}
	else
	{
		peak[channel] = one_minus_RT * peak[channel];
	}

	if (peak[channel] < 0.0f)
	{
		peak_abs = fabsf(peak[channel]);
	}
	else
	{
		peak_abs = peak[channel];
	}

	LOG2(&peak_abs, dst_ptr);
}
#endif

#if Q31_USED
static inline void rms_log2_calculate(q31_t src, q31_t * dst_ptr, uint32_t channel)
{
	static q31_t rms_pow2[CHANNEL_CNT] = {Q31_ZERO};
	q31_t rms_log2_temp;
	int16_t shift;

	rms_pow2[channel] = Q31_ADD(Q31_MUL(one_minus_TAV, rms_pow2[channel]), Q31_MUL(TAV, Q31_MUL(src, src)));
	Q31_LOG2(rms_pow2[channel], &rms_log2_temp, &shift);
	if (shift != 1)
	{
		arm_shift_q31(&rms_log2_temp, (int8_t)(shift - 1), dst_ptr, 1);
	}
	else
	{
		*dst_ptr = rms_log2_temp;
	}
}

#else

static inline void rms_log2_calculate(float32_t src, float32_t * dst_ptr, uint32_t channel)
{
	static float32_t rms_pow2[CHANNEL_CNT] = {0.0f};

	rms_pow2[channel] = (one_minus_TAV * rms_pow2[channel]) + (TAV * (src * src));
	LOG2(&rms_pow2[channel], dst_ptr);
	*dst_ptr = (*dst_ptr) * 0.5f;
}
#endif /* Q31_USED */

#if Q31_USED
void limiter(q31_t * src_signal_arr, q31_t * dst_signal_arr, size_t signal_arr_count)
{
	static q31_t ctrl_factor_old[CHANNEL_CNT] = {[0 ... (CHANNEL_CNT - 1)] = 1.0f};
	static q31_t ctrl_factor_smooth[CHANNEL_CNT] = {[0 ... (CHANNEL_CNT - 1)] = 1.0f};
	q31_t x_peak_log2;
	q31_t ctrl_factor;
	q31_t ctrl_factor_exp;
	q31_t k;
	int16_t shift;

	for (uint32_t channel = 0; channel < CHANNEL_CNT; ++channel)
	{
		for (uint32_t i = (uint32_t)(channel * signal_arr_count / 2); i < (uint32_t)((signal_arr_count / 2) * (channel + 1)); ++i)
		{
			q31_t sample = src_signal_arr[i];
			peak_log2_calculate(sample, &x_peak_log2, channel);

			if (x_peak_log2 > log2_LT_scaled)
			{
				Q31_DIV(Q31_SUB(log2_LT_scaled, x_peak_log2), LS_INV, &ctrl_factor_exp, &shift);
				assert(shift == 0U);
				Q31_POW2(ctrl_factor_exp, &ctrl_factor);

				q31_t temp = ctrl_factor;
				for (int i = 0; i < SCALE_FACTOR; i++)
				{
					ctrl_factor = Q31_MUL(ctrl_factor, temp);
				}
			}
			else
			{
				ctrl_factor = Q31_ONE;
			}

			if (ctrl_factor < ctrl_factor_old[channel])
			{
				k = RT_ctrl_factor;
			}
			else
			{
				k = AT_ctrl_factor;
			}

			ctrl_factor_smooth[channel] = Q31_ADD(Q31_MUL(Q31_SUB(Q31_ONE, k), ctrl_factor_smooth[channel]), Q31_MUL(k, ctrl_factor));
			ctrl_factor_old[channel] = ctrl_factor;
			dst_signal_arr[i] = Q31_MUL(sample, ctrl_factor_smooth[channel]);
		}
	}
}

#else

void limiter(float32_t * src_signal_arr, float32_t * dst_signal_arr, size_t signal_arr_count)
{
	static float32_t ctrl_factor_old[CHANNEL_CNT] = {[0 ... (CHANNEL_CNT - 1)] = 1.0f};
	static float32_t ctrl_factor_smooth[CHANNEL_CNT] = {[0 ... (CHANNEL_CNT - 1)] = 1.0f};
	float32_t x_peak_log2;
	float32_t ctrl_factor;
	float32_t ctrl_factor_exp;
	float32_t k;

	for (uint32_t channel = 0; channel < CHANNEL_CNT; ++channel)
	{
		for (uint32_t i = (uint32_t)(channel * signal_arr_count / 2); i < (uint32_t)((signal_arr_count / 2) * (channel + 1)); ++i)
		{
			float32_t sample = src_signal_arr[i];
			peak_log2_calculate(sample, &x_peak_log2, channel);

			if (x_peak_log2 > log2_LT)
			{
				ctrl_factor_exp = LS * (log2_LT - x_peak_log2);
				ctrl_factor = powf(2.0f, ctrl_factor_exp);
			}
			else
			{
				ctrl_factor = 1.0f;
			}

			if (ctrl_factor < ctrl_factor_old[channel])
			{
				k = RT_ctrl_factor;
			}
			else
			{
				k = AT_ctrl_factor;
			}

			ctrl_factor_smooth[channel] = (1.0f - k) * ctrl_factor_smooth[channel] + k * ctrl_factor;
			ctrl_factor_old[channel] = ctrl_factor;
			dst_signal_arr[i] = sample * ctrl_factor_smooth[channel];
		}
	}
}
#endif /* Q31_USED */


#if Q31_USED
void compressor_expander_ngate(q31_t * src_signal_arr, q31_t * dst_signal_arr, size_t signal_arr_count, q31_t * x_rms_log2_array)
{
	static q31_t ctrl_factor_old[CHANNEL_CNT] = {[0 ... (CHANNEL_CNT - 1)] = Q31_ONE};
	static q31_t ctrl_factor_smooth[CHANNEL_CNT] = {[0 ... (CHANNEL_CNT - 1)] = Q31_ONE};
	q31_t x_rms_log2;
	q31_t ctrl_factor;
	q31_t ctrl_factor_exp;
	q31_t k;
	int16_t shift;

	for (uint32_t channel = 0; channel < CHANNEL_CNT; ++channel)
	{
		for (uint32_t i = (uint32_t)(channel * signal_arr_count / 2); i < (uint32_t)((signal_arr_count / 2) * (channel + 1)); ++i)
		{
			rms_log2_calculate(src_signal_arr[i], &x_rms_log2, channel);

			x_rms_log2_array[i] = x_rms_log2;

			if (x_rms_log2 > log2_CT_scaled)
			{
				ctrl_factor_exp = Q31_MUL(CS, Q31_SUB(log2_CT_scaled, x_rms_log2));
				Q31_POW2(ctrl_factor_exp, &ctrl_factor);

				q31_t temp = ctrl_factor;
				for (int i = 0; i < SCALE_FACTOR; i++)
				{
					ctrl_factor = Q31_MUL(ctrl_factor, temp);
				}
			}
			else if (x_rms_log2 < log2_NT_MUTE_scaled)
			{
				ctrl_factor = Q31_ZERO;
			}
			else if (x_rms_log2 < log2_NT_scaled)
			{
				Q31_DIV(Q31_SUB(log2_NT_scaled, x_rms_log2), NS_INV, &ctrl_factor_exp, &shift);
				assert(shift == 0U);
				Q31_POW2(ctrl_factor_exp, &ctrl_factor);

				q31_t temp = ctrl_factor;
				for (int i = 0; i < SCALE_FACTOR; i++)
				{
					ctrl_factor = Q31_MUL(ctrl_factor, temp);
				}
				ctrl_factor = Q31_MUL(ctrl_factor, pow2_ES_times_diff_ET_NT);
			}
			else if (x_rms_log2 < log2_ET_scaled)
			{
				ctrl_factor_exp = Q31_MUL(ES, Q31_SUB(log2_ET_scaled, x_rms_log2));
				Q31_POW2(ctrl_factor_exp, &ctrl_factor);

				q31_t temp = ctrl_factor;
				for (int i = 0; i < SCALE_FACTOR; i++)
				{
					ctrl_factor = Q31_MUL(ctrl_factor, temp);
				}
			}
			else
			{
				ctrl_factor = Q31_ONE;
			}

			if (ctrl_factor < ctrl_factor_old[channel])
			{
				k = RT_ctrl_factor;
			}
			else
			{
				k = AT_ctrl_factor;
			}

			ctrl_factor_smooth[channel] = Q31_ADD(Q31_MUL(Q31_SUB(Q31_ONE, k), ctrl_factor_smooth[channel]), Q31_MUL(k, ctrl_factor));
			ctrl_factor_old[channel] = ctrl_factor;
			dst_signal_arr[i] = Q31_MUL(src_signal_arr[i], ctrl_factor_smooth[channel]);
		}
	}
}

#else

void compressor_expander_ngate(float32_t * src_signal_arr, float32_t * dst_signal_arr, size_t signal_arr_count)
{
	static float32_t ctrl_factor_old[CHANNEL_CNT] = {[0 ... (CHANNEL_CNT - 1)] = 1.0f};
	static float32_t ctrl_factor_smooth[CHANNEL_CNT] = {[0 ... (CHANNEL_CNT - 1)] = 1.0f};
	float32_t x_rms_log2;
	float32_t ctrl_factor;
	float32_t ctrl_factor_exp;
	float32_t k;

	for (uint32_t channel = 0; channel < CHANNEL_CNT; ++channel)
	{
		for (uint32_t i = (uint32_t)(channel * signal_arr_count / 2); i < (uint32_t)((signal_arr_count / 2) * (channel + 1)); ++i)
		{
			float32_t sample = src_signal_arr[i];
			rms_log2_calculate(sample, &x_rms_log2, channel);

			if (x_rms_log2 > log2_CT)
			{
				ctrl_factor_exp = CS * (log2_CT - x_rms_log2);
				ctrl_factor = powf(2.0f, ctrl_factor_exp);
			}
			else if (x_rms_log2 < log2_NT_MUTE)
			{
				ctrl_factor = 0.0f;;
			}
			else if (x_rms_log2 < log2_NT)
			{
				ctrl_factor_exp = NS * (log2_NT - x_rms_log2) + ES_times_diff_ET_NT;
				ctrl_factor = powf(2.0f, ctrl_factor_exp);
			}
			else if (x_rms_log2 < log2_ET)
			{
				ctrl_factor_exp = ES * (log2_ET - x_rms_log2);
				ctrl_factor = powf(2.0f, ctrl_factor_exp);
			}
			else
			{
				ctrl_factor = 1.0f;
			}

			if (ctrl_factor < ctrl_factor_old[channel])
			{
				k = RT_ctrl_factor;
			}
			else
			{
				k = AT_ctrl_factor;
			}

			ctrl_factor_smooth[channel] = (1.0f - k) * ctrl_factor_smooth[channel] + k * ctrl_factor;
			ctrl_factor_old[channel] = ctrl_factor;
			dst_signal_arr[i] = sample * ctrl_factor_smooth[channel];
		}
	}
}
#endif /* Q31_USED */

#if Q31_USED
void drc_full(q31_t * src_signal_arr, q31_t * dst_signal_arr, size_t signal_arr_count, q31_t * x_peak_log2_array)
{
	static q31_t ctrl_factor_old[CHANNEL_CNT] = {[0 ... (CHANNEL_CNT - 1)] = 1.0f};
	static q31_t ctrl_factor_smooth[CHANNEL_CNT] = {[0 ... (CHANNEL_CNT - 1)] = 1.0f};
	q31_t x_rms_log2;
	q31_t x_peak_log2;
	q31_t ctrl_factor;
	q31_t ctrl_factor_exp;
	q31_t k;
	int16_t shift;

	for (uint32_t channel = 0; channel < CHANNEL_CNT; ++channel)
	{
		for (uint32_t i = (uint32_t)(channel * signal_arr_count / 2); i < (uint32_t)((signal_arr_count / 2) * (channel + 1)); ++i)
		{
			q31_t sample = src_signal_arr[i];
			peak_log2_calculate(sample, &x_peak_log2, channel);
			rms_log2_calculate(sample, &x_rms_log2, channel);

			#if DEBUG
			x_peak_log2_array[i] = x_peak_log2;
			#endif

			if (x_peak_log2 > log2_LT_scaled)
			{
				Q31_DIV(Q31_SUB(log2_LT_scaled, x_peak_log2), LS_INV, &ctrl_factor_exp, &shift);
				assert(shift == 0U);
				Q31_POW2(ctrl_factor_exp, &ctrl_factor);

				q31_t temp = ctrl_factor;
				for (int i = 0; i < SCALE_FACTOR; i++)
				{
					ctrl_factor = Q31_MUL(ctrl_factor, temp);
				}
				ctrl_factor = Q31_MUL(ctrl_factor, pow2_CS_times_diff_CT_LT);
			}
			else if (x_rms_log2 > log2_CT_scaled)
			{
				ctrl_factor_exp = Q31_MUL(CS, Q31_SUB(log2_CT_scaled, x_rms_log2));
				Q31_POW2(ctrl_factor_exp, &ctrl_factor);

				q31_t temp = ctrl_factor;
				for (int i = 0; i < SCALE_FACTOR; i++)
				{
					ctrl_factor = Q31_MUL(ctrl_factor, temp);
				}
			}
			else if (x_rms_log2 < log2_NT_MUTE_scaled)
			{
				ctrl_factor = Q31_ZERO;
			}
			else if (x_rms_log2 < log2_NT_scaled)
			{
				Q31_DIV(Q31_SUB(log2_NT_scaled, x_rms_log2), NS_INV, &ctrl_factor_exp, &shift);
				assert(shift == 0U);
				Q31_POW2(ctrl_factor_exp, &ctrl_factor);

				q31_t temp = ctrl_factor;
				for (int i = 0; i < SCALE_FACTOR; i++)
				{
					ctrl_factor = Q31_MUL(ctrl_factor, temp);
				}
				ctrl_factor = Q31_MUL(ctrl_factor, pow2_ES_times_diff_ET_NT);
			}
			else if (x_rms_log2 < log2_ET_scaled)
			{
				ctrl_factor_exp = Q31_MUL(ES, Q31_SUB(log2_ET_scaled, x_rms_log2));
				Q31_POW2(ctrl_factor_exp, &ctrl_factor);

				q31_t temp = ctrl_factor;
				for (int i = 0; i < SCALE_FACTOR; i++)
				{
					ctrl_factor = Q31_MUL(ctrl_factor, temp);
				}
			}
			else
			{
				ctrl_factor = Q31_ONE;
			}

			if (ctrl_factor < ctrl_factor_old[channel])
			{
				k = RT_ctrl_factor;
			}
			else
			{
				k = AT_ctrl_factor;
			}

			ctrl_factor_smooth[channel] = Q31_ADD(Q31_MUL(Q31_SUB(Q31_ONE, k), ctrl_factor_smooth[channel]), Q31_MUL(k, ctrl_factor));
			ctrl_factor_old[channel] = ctrl_factor;
			dst_signal_arr[i] = Q31_MUL(sample, ctrl_factor_smooth[channel]);
		}
	}
}

#else

void drc_full(float32_t * src_signal_arr, float32_t * dst_signal_arr, size_t signal_arr_count, float32_t * x_peak_log2_array)
{
	static float32_t ctrl_factor_old[CHANNEL_CNT] = {[0 ... (CHANNEL_CNT - 1)] = 1.0f};
	static float32_t ctrl_factor_smooth[CHANNEL_CNT] = {[0 ... (CHANNEL_CNT - 1)] = 1.0f};
	float32_t x_rms_log2;
	float32_t x_peak_log2;
	float32_t ctrl_factor;
	float32_t ctrl_factor_exp;
	float32_t k;

	for (uint32_t channel = 0; channel < CHANNEL_CNT; ++channel)
	{
		for (uint32_t i = (uint32_t)(channel * signal_arr_count / 2); i < (uint32_t)((signal_arr_count / 2) * (channel + 1)); ++i)
		{
			float32_t sample = src_signal_arr[i];
			peak_log2_calculate(sample, &x_peak_log2, channel);
			rms_log2_calculate(sample, &x_rms_log2, channel);

			#if DEBUG
			x_peak_log2_array[i] = x_peak_log2;
			#endif

			if (x_peak_log2 > log2_LT)
			{
				ctrl_factor_exp = LS * (log2_LT - x_peak_log2) + CS_times_diff_CT_LT;
				ctrl_factor = powf(2.0f, ctrl_factor_exp);
			}
			else if (x_rms_log2 > log2_CT)
			{
				ctrl_factor_exp = CS * (log2_CT - x_rms_log2);
				ctrl_factor = powf(2.0f, ctrl_factor_exp);
			}
			else if (x_rms_log2 < log2_NT_MUTE)
			{
				ctrl_factor = 0.0f;
			}
			else if (x_rms_log2 < log2_NT)
			{
				ctrl_factor_exp = NS * (log2_NT - x_rms_log2) + ES_times_diff_ET_NT;
				ctrl_factor = powf(2.0f, ctrl_factor_exp);
			}
			else if (x_rms_log2 < log2_ET)
			{
				ctrl_factor_exp = ES * (log2_ET - x_rms_log2);
				ctrl_factor = powf(2.0f, ctrl_factor_exp);
			}
			else
			{
				ctrl_factor = 1.0f;
			}

			if (ctrl_factor < ctrl_factor_old[channel])
			{
				k = RT_ctrl_factor;
			}
			else
			{
				k = AT_ctrl_factor;
			}

			ctrl_factor_smooth[channel] = (1.0f - k) * ctrl_factor_smooth[channel] + k * ctrl_factor;
			ctrl_factor_old[channel] = ctrl_factor;
			dst_signal_arr[i] = sample * ctrl_factor_smooth[channel];
		}
	}
}
#endif /* Q31_USED */

#if Q31_USED
void drc_full_stereo_balanced(q31_t * src_signal_arr, q31_t * dst_signal_arr, q31_t * x_peak_log2_array)
{
	static q31_t ctrl_factor_old = 1.0f;
	static q31_t ctrl_factor_smooth = 1.0f;
	q31_t x_rms_log2;
	q31_t x_peak_log2;
	q31_t ctrl_factor;
	q31_t ctrl_factor_exp;
	q31_t k;
	int16_t shift;
	int channel_offset = BUFFER_SIZE / 2;


	const pq_prescale_t prescale = {
		.inputPrescale  = (-16),
		.outputPrescale = (16 - SCALE_FACTOR_SHIFT),
		.outputSaturate = 0,
	};
	PQ_SetCoprocessorScaler(POWERQUAD, &prescale);

	for (int i = 0; i < channel_offset; ++i)
	{
		q31_t avg_sample = (q31_t)(((q63_t)src_signal_arr[i] + (q63_t)src_signal_arr[i + channel_offset]) >> 1); // >>1 ??
		peak_log2_calculate(avg_sample, &x_peak_log2, 0U);
		rms_log2_calculate(avg_sample, &x_rms_log2, 0U);

		#if DEBUG
		x_peak_log2_array[i] = x_peak_log2;
		x_peak_log2_array[i + channel_offset] = x_peak_log2;
		#endif

		if (x_peak_log2 > log2_LT_scaled)
		{
			Q31_DIV(Q31_SUB(log2_LT_scaled, x_peak_log2), LS_INV, &ctrl_factor_exp, &shift);
			assert(shift == 0U);
			q31_t x_times_ln2 = Q31_MUL(ctrl_factor_exp, 0x58b90bfc);
			ctrl_factor = (PQ_EtoxFixed(x_times_ln2 >> 15) << (15 + SCALE_FACTOR_SHIFT));


			q31_t temp = ctrl_factor;
			for (int i = 0; i < SCALE_FACTOR; i++)
			{
				ctrl_factor = Q31_MUL(ctrl_factor, temp);
			}
			ctrl_factor = Q31_MUL(ctrl_factor, pow2_CS_times_diff_CT_LT);
		}
		else if (x_rms_log2 > log2_CT_scaled)
		{
			ctrl_factor_exp = Q31_MUL(CS, Q31_SUB(log2_CT_scaled, x_rms_log2));
			q31_t x_times_ln2 = Q31_MUL(ctrl_factor_exp, 0x58b90bfc);
			ctrl_factor = (PQ_EtoxFixed(x_times_ln2 >> 15) << (15 + SCALE_FACTOR_SHIFT));

			q31_t temp = ctrl_factor;
			for (int i = 0; i < SCALE_FACTOR; i++)
			{
				ctrl_factor = Q31_MUL(ctrl_factor, temp);
			}
		}
		else if (x_rms_log2 < log2_NT_MUTE_scaled)
		{
			ctrl_factor = Q31_ZERO;
		}
		else if (x_rms_log2 < log2_NT_scaled)
		{
			Q31_DIV(Q31_SUB(log2_NT_scaled, x_rms_log2), NS_INV, &ctrl_factor_exp, &shift);
			assert(shift == 0U);
			q31_t x_times_ln2 = Q31_MUL(ctrl_factor_exp, 0x58b90bfc);
			ctrl_factor = (PQ_EtoxFixed(x_times_ln2 >> 15) << (15 + SCALE_FACTOR_SHIFT));

			q31_t temp = ctrl_factor;
			for (int i = 0; i < SCALE_FACTOR; i++)
			{
				ctrl_factor = Q31_MUL(ctrl_factor, temp);
			}
			ctrl_factor = Q31_MUL(ctrl_factor, pow2_ES_times_diff_ET_NT);
		}
		else if (x_rms_log2 < log2_ET_scaled)
		{
			ctrl_factor_exp = Q31_MUL(ES, Q31_SUB(log2_ET_scaled, x_rms_log2));
			q31_t x_times_ln2 = Q31_MUL(ctrl_factor_exp, 0x58b90bfc);
			ctrl_factor = (PQ_EtoxFixed(x_times_ln2 >> 15) << (15 + SCALE_FACTOR_SHIFT));

			q31_t temp = ctrl_factor;
			for (int i = 0; i < SCALE_FACTOR; i++)
			{
				ctrl_factor = Q31_MUL(ctrl_factor, temp);
			}
		}
		else
		{
			ctrl_factor = Q31_ONE;
		}

		if (ctrl_factor < ctrl_factor_old)
		{
			k = RT_ctrl_factor;
		}
		else
		{
			k = AT_ctrl_factor;
		}

		/* smooth control factor */
		ctrl_factor_smooth = Q31_ADD(Q31_MUL(Q31_SUB(Q31_ONE, k), ctrl_factor_smooth), Q31_MUL(k, ctrl_factor));
		ctrl_factor_old = ctrl_factor;
		/* calculate output sample */
		dst_signal_arr[i] = Q31_MUL(src_signal_arr[i], ctrl_factor_smooth);
		dst_signal_arr[i + channel_offset] = Q31_MUL(src_signal_arr[i + channel_offset], ctrl_factor_smooth);
	}
}

#else

#if PQ_USED
void drc_full_stereo_balanced(float32_t * src_signal_arr, float32_t * dst_signal_arr, float32_t * x_peak_log2_array)
{
	static float32_t ctrl_factor_old = 1.0f;
	static float32_t ctrl_factor_smooth = 1.0f;
	float32_t x_rms_log2;
	float32_t x_peak_log2;
	float32_t ctrl_factor;
	float32_t ctrl_factor_exp;
	float32_t k;
	float32_t avg_sample;
	const static int channel_offset = BUFFER_SIZE / CHANNEL_CNT;

	for (int i = 0; i < channel_offset; ++i)
	{
		float32_t num = src_signal_arr[i] + src_signal_arr[i + channel_offset];
		float32_t den = (float32_t)CHANNEL_CNT;
		PQ_DivF32(&num, &den, &avg_sample);

		peak_log2_calculate(avg_sample, &x_peak_log2, 0U);
		rms_log2_calculate(avg_sample, &x_rms_log2, 0U);

		#if DEBUG
		x_peak_log2_array[i] = x_peak_log2;
		x_peak_log2_array[i + channel_offset] = x_peak_log2;
		#endif

		if (x_peak_log2 > log2_LT)
		{
			ctrl_factor_exp = LS * (log2_LT - x_peak_log2) + CS_times_diff_CT_LT;
			float32_t x_times_ln2 = ctrl_factor_exp * 0.693147f;
			PQ_EtoxF32(&x_times_ln2, &ctrl_factor);
		}
		else if (x_rms_log2 > log2_CT)
		{
			ctrl_factor_exp = CS * (log2_CT - x_rms_log2);
			float32_t x_times_ln2 = ctrl_factor_exp * 0.693147f;
			PQ_EtoxF32(&x_times_ln2, &ctrl_factor);
		}
		else if (x_rms_log2 < log2_NT_MUTE)
		{
			ctrl_factor = 0.0f;
		}
		else if (x_rms_log2 < log2_NT)
		{
			ctrl_factor_exp = NS * (log2_NT - x_rms_log2) + ES_times_diff_ET_NT;
			float32_t x_times_ln2 = ctrl_factor_exp * 0.693147f;
			PQ_EtoxF32(&x_times_ln2, &ctrl_factor);
		}
		else if (x_rms_log2 < log2_ET)
		{
			ctrl_factor_exp = ES * (log2_ET - x_rms_log2);
			float32_t x_times_ln2 = ctrl_factor_exp * 0.693147f;
			PQ_EtoxF32(&x_times_ln2, &ctrl_factor);
		}
		else
		{
			ctrl_factor = 1.0f;
		}

		/* determine if control factor (in/de)creasing */
		if (ctrl_factor < ctrl_factor_old)
		{
			k = RT_ctrl_factor;
		}
		else
		{
			k = AT_ctrl_factor;
		}

		/* smooth control factor */
		ctrl_factor_smooth = (1.0f - k) * ctrl_factor_smooth + k * ctrl_factor;
		ctrl_factor_old = ctrl_factor;
		/* calculate output sample */
		dst_signal_arr[i] = src_signal_arr[i] * ctrl_factor_smooth;
		dst_signal_arr[i + channel_offset] = src_signal_arr[i + channel_offset] * ctrl_factor_smooth;
	}
}

#else

void drc_full_stereo_balanced(float32_t * src_signal_arr, float32_t * dst_signal_arr, float32_t * x_peak_log2_array)
{
	static float32_t ctrl_factor_old = 1.0f;
	static float32_t ctrl_factor_smooth = 1.0f;
	float32_t x_rms_log2;
	float32_t x_peak_log2;
	float32_t ctrl_factor;
	float32_t ctrl_factor_exp;
	float32_t k;
	const static int channel_offset = BUFFER_SIZE / CHANNEL_CNT;

	for (int i = 0; i < channel_offset; ++i)
	{
		float32_t avg_sample = ((src_signal_arr[i] + src_signal_arr[i + channel_offset]) / (float32_t)CHANNEL_CNT);

		peak_log2_calculate(avg_sample, &x_peak_log2, 0U);
		rms_log2_calculate(avg_sample, &x_rms_log2, 0U);

		#if DEBUG
		x_peak_log2_array[i] = x_peak_log2;
		x_peak_log2_array[i + channel_offset] = x_peak_log2;
		#endif

		if (x_peak_log2 > log2_LT)
		{
			ctrl_factor_exp = LS * (log2_LT - x_peak_log2) + CS_times_diff_CT_LT;
			ctrl_factor = powf(2.0f, ctrl_factor_exp);
		}
		else if (x_rms_log2 > log2_CT)
		{
			ctrl_factor_exp = CS * (log2_CT - x_rms_log2);
			ctrl_factor = powf(2.0f, ctrl_factor_exp);
		}
		else if (x_rms_log2 < log2_NT_MUTE)
		{
			ctrl_factor = 0.0f;
		}
		else if (x_rms_log2 < log2_NT)
		{
			ctrl_factor_exp = NS * (log2_NT - x_rms_log2) + ES_times_diff_ET_NT;
			ctrl_factor = powf(2.0f, ctrl_factor_exp);
		}
		else if (x_rms_log2 < log2_ET)
		{
			ctrl_factor_exp = ES * (log2_ET - x_rms_log2);
			ctrl_factor = powf(2.0f, ctrl_factor_exp);
		}
		else
		{
			ctrl_factor = 1.0f;
		}

		/* determine if control factor (in/de)creasing */
		if (ctrl_factor < ctrl_factor_old)
		{
			k = RT_ctrl_factor;
		}
		else
		{
			k = AT_ctrl_factor;
		}

		/* smooth control factor */
		ctrl_factor_smooth = (1.0f - k) * ctrl_factor_smooth + k * ctrl_factor;
		ctrl_factor_old = ctrl_factor;
		/* calculate output sample */
		dst_signal_arr[i] = src_signal_arr[i] * ctrl_factor_smooth;
		dst_signal_arr[i + channel_offset] = src_signal_arr[i + channel_offset] * ctrl_factor_smooth;
	}
}
#endif /* PQ_USED */
#endif /* Q31_USED */
