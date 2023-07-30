/*
 * algorithm_testbench.c
 *
 *  Created on: 26 cze 2023
 *      Author: Łukasz
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "fsl_powerquad.h"

#include "drc_algorithms_cm33.h"
#include "algorithm_testbench.h"
#include "main_cm33.h"

#include <time.h>
#include <math.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FIR_ORDER		256U
#define FIR_COEFF_COUNT	(FIR_ORDER + 1U)
#define BLOCK_SIZE		32U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*******************************************************************************
 * Variables
 ******************************************************************************/
static float32_t fir_state[BLOCK_SIZE + FIR_COEFF_COUNT - 1];
static uint32_t block_size = BLOCK_SIZE;
static const float fir_filter_coeff[FIR_COEFF_COUNT] = {
	/* cutoff 6kHz, order 32 */
//	-0.0f, -0.00133619341280010f, -0.00262120541617210f, -0.00273376615706914f, 0.0f,
//	0.00583381302308364f, 0.0116127053037436f, 0.0112869709658527f, 0.0f, -0.0202965559831879f,
//	-0.0380763811518282f, -0.0358973374642139f, 0.0f, 0.0693732235110302f, 0.153944232060998f,
//	0.223615729382988f, 0.250589530675151f, 0.223615729382988f, 0.153944232060998f, 0.0693732235110302f,
//	0.0f, -0.0358973374642139f, -0.0380763811518282f, -0.0202965559831879f, 0.0f,
//	0.0112869709658527f, 0.0116127053037436f, 0.00583381302308364f, 0.0f, -0.00273376615706914f,
//	-0.00262120541617210f, -0.00133619341280010f, -0.0f,
	/* cutoff 6kHz, order 256 */
		0.0f, -0.000142095463926298f, -0.000203598484145306f, -0.000146364768649777f, 0.0f, 0.000152796255334117f, 0.000221823870954137f, 0.000161487217690435f, 0.0f, -0.000172536476232354f, -0.000253111426525135f, -0.000186044545777425f, 0.0f, 0.000202113832835479f, 0.000298599184600296f, 0.000220848866463926f, 0.0f, -0.000242356566655504f, -0.000359470896364611f, -0.000266746554889788f, 0.0f, 0.000294131512141478f, 0.000436968307791359f, 0.000324627590413839f, 0.0f, -0.000358354884777625f, -0.000532407533612327f, -0.000395437973970450f, 0.0f, 0.000436006538883024f, 0.000647200648234585f, 0.000480196069766373f, 0.0f, -0.000528148674789083f, -0.000782883983192874f, -0.000580014004716558f, 0.0f, 0.000635950311052743f, 0.000941155138348767f, 0.000696125658073407f, 0.0f, -0.000760719312910043f, -0.00112392144914783f, -0.000829923342363559f, 0.0f, 0.000903944450630234f, 0.00133336371102959f, 0.000983006098849127f, 0.0f, -0.00106735095564239f, -0.00157202051065915f, -0.00115724373801470f, 0.0f, 0.00125297451462210f, 0.00184290081687561f, 0.00135486255918409f, 0.0f, -0.00146326086157541f, -0.00214963597346358f, -0.00157856142905363f, 0.0f, 0.00170120154169663f, 0.00249668762992459f, 0.00183167116649932f, 0.0f, -0.00197052178649341f, -0.00288963667437853f, -0.00211837696848963f, 0.0f, 0.00227594508079392f, 0.00333559205593175f, 0.00244403468771380f, 0.0f, -0.00262357330092913f, -0.00384378142477216f, -0.00281563037311102f, 0.0f, 0.00302144569694921f, 0.00442642515110156f, 0.00324246475256718f, 0.0f, -0.00348038307159067f, -0.00510006593656665f, -0.00373720242153848f, 0.0f, 0.00401530265745672f, 0.00588765744666140f, 0.00431753458898457f, 0.0f, -0.00464734140520535f, -0.00682197091934482f, -0.00500891946123627f, 0.0f, 0.00540743416951147f, 0.00795140479312829f, 0.00584931437171450f, 0.0f, -0.00634266063239365f, -0.00935043979531675f, -0.00689782244903868f, 0.0f, 0.00752823196298245f, 0.0111397418830013f, 0.00825163780957785f, 0.0f, -0.00909198266861255f, -0.0135281916709367f, -0.0100823536463049f, 0.0f, 0.0112697959941345f, 0.0169109477103547f, 0.0127235246748694f, 0.0f, -0.0145497050974079f, -0.0221356107387184f, -0.0169198775073432f, 0.0f, 0.0201303896832348f, 0.0314071709056526f, 0.0247410730777657f, 0.0f, -0.0319516492901427f, -0.0528127241883479f, -0.0448815117930890f, 0.0f, 0.0749686340967730f, 0.159142724192092f, 0.225155374452901f, 0.250119395289229f, 0.225155374452901f, 0.159142724192092f, 0.0749686340967730f, 0.0f, -0.0448815117930890f, -0.0528127241883479f, -0.0319516492901427f, 0.0f, 0.0247410730777657f, 0.0314071709056526f, 0.0201303896832348f, 0.0f, -0.0169198775073432f, -0.0221356107387184f, -0.0145497050974079f, 0.0f, 0.0127235246748694f, 0.0169109477103547f, 0.0112697959941345f, 0.0f, -0.0100823536463049f, -0.0135281916709367f, -0.00909198266861255f, 0.0f, 0.00825163780957785f, 0.0111397418830013f, 0.00752823196298245f, 0.0f, -0.00689782244903868f, -0.00935043979531675f, -0.00634266063239365f, 0.0f, 0.00584931437171450f, 0.00795140479312829f, 0.00540743416951147f, 0.0f, -0.00500891946123627f, -0.00682197091934482f, -0.00464734140520535f, 0.0f, 0.00431753458898457f, 0.00588765744666140f, 0.00401530265745672f, 0.0f, -0.00373720242153848f, -0.00510006593656665f, -0.00348038307159067f, 0.0f, 0.00324246475256718f, 0.00442642515110156f, 0.00302144569694921f, 0.0f, -0.00281563037311102f, -0.00384378142477216f, -0.00262357330092913f, 0.0f, 0.00244403468771380f, 0.00333559205593175f, 0.00227594508079392f, 0.0f, -0.00211837696848963f, -0.00288963667437853f, -0.00197052178649341f, 0.0f, 0.00183167116649932f, 0.00249668762992459f, 0.00170120154169663f, 0.0f, -0.00157856142905363f, -0.00214963597346358f, -0.00146326086157541f, 0.0f, 0.00135486255918409f, 0.00184290081687561f, 0.00125297451462210f, 0.0f, -0.00115724373801470f, -0.00157202051065915f, -0.00106735095564239f, 0.0f, 0.000983006098849127f, 0.00133336371102959f, 0.000903944450630234f, 0.0f, -0.000829923342363559f, -0.00112392144914783f, -0.000760719312910043f, 0.0f, 0.000696125658073407f, 0.000941155138348767f, 0.000635950311052743f, 0.0f, -0.000580014004716558f, -0.000782883983192874f, -0.000528148674789083f, 0.0f, 0.000480196069766373f, 0.000647200648234585f, 0.000436006538883024f, 0.0f, -0.000395437973970450f, -0.000532407533612327f, -0.000358354884777625f, 0.0f, 0.000324627590413839f, 0.000436968307791359f, 0.000294131512141478f, 0.0f, -0.000266746554889788f, -0.000359470896364611f, -0.000242356566655504f, 0.0f, 0.000220848866463926f, 0.000298599184600296f, 0.000202113832835479f, 0.0f, -0.000186044545777425f, -0.000253111426525135f, -0.000172536476232354f, 0.0f, 0.000161487217690435f, 0.000221823870954137f, 0.000152796255334117f, 0.0f, -0.000146364768649777f, -0.000203598484145306f, -0.000142095463926298f, 0.0f
};
/*******************************************************************************
 * Code
 ******************************************************************************/
static void init_arr_with_rand_16(int16_t * arr, size_t arr_size)
{
	srand(time(NULL));
    for (int i = 0 ; i < arr_size ; i++)
    {
    	arr[i] = (int16_t)((rand() % INT16_MAX) - INT16_MAX);
    }
}

static void init_arr_with_rand_f32(float * arr, size_t arr_size)
{
	srand(time(NULL));
    for (int i = 0 ; i < arr_size ; i++)
    {
    	arr[i] = (float)(rand() % UINT8_MAX);
    }
}

void generate_sine_wave_16(int16_t * input_vec, uint32_t vec_len, uint32_t fs, float max_amplitude, float * freq, float * amp, int freq_cnt)
{
	float * rad 	= (float *)malloc(sizeof(float) * freq_cnt);
	float * comp = (float *)malloc(sizeof(float) * freq_cnt);
	float temp_comp_sum 	= 0.0f;
	float amp_sum 			= 0.0f;

	if (max_amplitude > (float)INT16_MAX)
	{
		PRINTF("\r\nChange amplitude!!!\r\n");
	}

	for (uint32_t i = 0U; i < freq_cnt; i++)
	{
		amp_sum += amp[i];
	}

	for (uint32_t i = 0U, sample_num = 0U; i < vec_len; i += 2, sample_num++)
	{
		/* convert to radians */
		for (uint32_t j = 0U; j < freq_cnt; j++)
		{
			rad[j] = (2 * PI * freq[j] * sample_num / fs);
		}

		/* calculate component signals values */
		for (uint32_t j = 0U; j < freq_cnt; j++)
		{
			comp[j] = (max_amplitude * amp[j] * arm_sin_f32(rad[j]));
		}

		temp_comp_sum = 0.0f;
		/* sum component signals */
		for (uint32_t j = 0U; j < freq_cnt; j++)
		{
			temp_comp_sum += comp[j];
		}
		input_vec[i] = (int16_t)(temp_comp_sum / amp_sum);
	}

	free(rad);
	free(comp);
}

void generate_sine_wave_f32(float32_t * input_vec, uint32_t vec_len, uint32_t fs, float max_amplitude, float * freq, float * amp, int freq_cnt)
{
	float * rad 	= (float *)malloc(sizeof(float) * freq_cnt);
	float * comp = (float *)malloc(sizeof(float) * freq_cnt);
	float temp_comp_sum 	= 0.0f;
	float amp_sum 			= 0.0f;

	if (max_amplitude > (float)INT16_MAX)
	{
		PRINTF("\r\nChange amplitude!!!\r\n");
	}

	for (uint32_t i = 0U; i < freq_cnt; i++)
	{
		amp_sum += amp[i];
	}

	for (uint32_t i = 0U; i < vec_len; i++)
	{
		/* convert to radians */
		for (uint32_t j = 0U; j < freq_cnt; j++)
		{
			rad[j] = (2 * PI * freq[j] * i / fs);
		}

		/* calculate component signals values */
		for (uint32_t j = 0U; j < freq_cnt; j++)
		{
			comp[j] = (max_amplitude * amp[j] * arm_sin_f32(rad[j]));
		}

		temp_comp_sum = 0.0f;
		/* sum component signals */
		for (uint32_t j = 0U; j < freq_cnt; j++)
		{
			temp_comp_sum += comp[j];
		}
		input_vec[i] = (float32_t)(temp_comp_sum / amp_sum);
	}

	free(rad);
	free(comp);
}

void measure_algorithm_time_16(void (*algorithm_func)(int16_t *, int16_t *, size_t), int16_t * src_buffer, int16_t * dst_buffer, size_t bytes, uint32_t iterations)
{
	volatile unsigned long previous_time = 0UL;
    unsigned long exec_time_ticks = 0UL;
    unsigned long exec_time_ticks_sum = 0UL;
    size_t buffer_size = (bytes / sizeof(int16_t));
    // write random values to buffer
    init_arr_with_rand_16(src_buffer, buffer_size);

	previous_time = MSDK_GetCpuCycleCount();
    // execute algorithm 'iterations' times
	for (int i = 0; i < iterations; i++)
	{
		algorithm_func(src_buffer, dst_buffer, buffer_size);
	}
	exec_time_ticks_sum = (MSDK_GetCpuCycleCount() - previous_time);

    PRINTF("Average time: %.3f ms\r\n", (CYCLES_TO_MS(exec_time_ticks_sum)) / (float)iterations);
}

void test_pq_math(float * arr, uint32_t arr_size)
{
	volatile unsigned long previous_time = 0UL;
    unsigned long exec_time_ticks = 0UL;
    unsigned long exec_time_ticks_sum = 0UL;

	float x1 = 3.0f;
	float x2 = 9.0f;
	volatile float result;

	init_arr_with_rand_f32(arr, arr_size);

	previous_time = MSDK_GetCpuCycleCount();
	for (int i = 0; i < arr_size; i++)
	{
		PQ_DivF32(&x1, &arr[i], (float *)&result);
	}
	exec_time_ticks_sum = (MSDK_GetCpuCycleCount() - previous_time);

	PRINTF("[PQ] Average time x1/x2: %.3f us\r\n", (CYCLES_TO_US(exec_time_ticks_sum) / (float)arr_size));

	previous_time = MSDK_GetCpuCycleCount();
	for (int i = 0; i < arr_size; i++)
	{
		result = x1 / arr[i];
	}
	exec_time_ticks_sum = (MSDK_GetCpuCycleCount() - previous_time);
	PRINTF("[CM-33] Average time x1/x2: %.3f us\r\n", (CYCLES_TO_US(exec_time_ticks_sum) / (float)arr_size));


	previous_time = MSDK_GetCpuCycleCount();
	for (int i = 0; i < arr_size; i++)
	{
		PQ_LnF32(&arr[i], (float *)&result);
	}
	exec_time_ticks_sum = (MSDK_GetCpuCycleCount() - previous_time);
	PRINTF("[PQ] Average time ln: %.3f us\r\n", (CYCLES_TO_US(exec_time_ticks_sum) / (float)arr_size));

	previous_time = MSDK_GetCpuCycleCount();
	for (int i = 0; i < arr_size; i++)
	{
		result = logf(arr[i]);
	}
	exec_time_ticks_sum = (MSDK_GetCpuCycleCount() - previous_time);
	PRINTF("[CM-33] Average time ln: %.3f us\r\n", (CYCLES_TO_US(exec_time_ticks_sum) / (float)arr_size));
}

void test_cmsis_dsp(float32_t * src_arr, float32_t * dst_arr, uint32_t arr_size, uint32_t fs, uint32_t iterations)
{
	arm_fir_instance_f32 fir_instance;
	arm_status status;
	float32_t  *input_arr, *output_arr;
	uint32_t numBlocks = arr_size / BLOCK_SIZE;

	float32_t freq[] 	= {9000.0f, 7000.0f, 1500.0f};
	float32_t amp[] 	= {0.2f, 0.0f, 1.0f};
	int freq_cnt = sizeof(freq) / sizeof(freq[0]);

	volatile unsigned long previous_time = 0UL;
    unsigned long exec_time_ticks = 0UL;
    unsigned long exec_time_ticks_sum = 0UL;

	generate_sine_wave_f32(&src_arr[0], arr_size, fs, (float)(INT16_MAX * 0.5), freq, amp, freq_cnt);

	/* Initialize input and output buffer pointers */
	input_arr = &src_arr[0];
	output_arr = &dst_arr[0];
	/* Call FIR init function to initialize the instance structure. */
	arm_fir_init_f32(&fir_instance, FIR_COEFF_COUNT, (float32_t *)&fir_filter_coeff[0], &fir_state[0], block_size);

//	print_buffer_data_f32(src_arr, arr_size);

	previous_time = MSDK_GetCpuCycleCount();
	for (uint32_t i = 0; i < iterations; i++)
	{
		for(uint32_t j = 0; j < numBlocks; j++)
		{
			arm_fir_f32(&fir_instance, input_arr + (j * block_size), output_arr + (j * block_size), block_size);
		}
	}
	exec_time_ticks_sum = (MSDK_GetCpuCycleCount() - previous_time);

//	print_buffer_data_f32(dst_arr, arr_size);
	PRINTF("[CM-33] Average time arm_fir_f32: %.3f ms\r\n", CYCLES_TO_MS(exec_time_ticks_sum) / (float)iterations);
}

void print_buffer_data_16(int16_t * data, size_t data_size)
{
	for (size_t i = 0; i < data_size; i++)
	{
//		PRINTF("0x%0X, ", data[i]);
		PRINTF("%d, ", data[i]);
		if (i%20 == 19)
		{
			PRINTF("\r\n");
		}
	}
	PRINTF("\r\n\n");
}

void print_buffer_data_f32(float32_t * buffer, size_t buffer_size)
{
	for (size_t i = 0; i < buffer_size; i++)
	{
		PRINTF("%.3f, ", buffer[i]);
		if (i%20 == 19)
		{
			PRINTF("\r\n");
		}
	}
	PRINTF("\r\n\n");
}

void write_buffer_data_to_file_16(int16_t * buffer, size_t buffer_size)
{
	PRINTF("$");
	for (size_t i = 0; i < buffer_size; i++)
	{
		PRINTF("%d, ", buffer[i]);
		if (i%20 == 19)
		{
			PRINTF("\r\n");
		}
	}
}

void test_drc_algorithm(void (*algorithm_func)(int16_t *, int16_t *, size_t), int16_t * src_buffer, int16_t * dst_buffer,
		size_t buffer_size, uint32_t fs)
{
	float freq[] 	= {9000.0f, 7000.0f, 1500.0f};
	float amp[] 	= {0.1f, 0.2f, 2.0f};
	int freq_cnt = sizeof(freq) / sizeof(freq[0]);

	generate_sine_wave_16(&src_buffer[0], buffer_size/5, fs, (float)INT16_MAX * 0.15, freq, amp, freq_cnt);
	generate_sine_wave_16(&src_buffer[1000], buffer_size/5, fs, (float)INT16_MAX * 0.40, freq, amp, freq_cnt);
	generate_sine_wave_16(&src_buffer[2000], buffer_size/5, fs, (float)INT16_MAX * 0.65, freq, amp, freq_cnt);
	generate_sine_wave_16(&src_buffer[3000], buffer_size/5, fs, (float)INT16_MAX * 0.8, freq, amp, freq_cnt);
	generate_sine_wave_16(&src_buffer[4000], buffer_size/5, fs, (float)INT16_MAX * 0.95, freq, amp, freq_cnt);

	write_buffer_data_to_file_16(src_buffer, buffer_size);
	algorithm_func(src_buffer, dst_buffer, buffer_size);
	write_buffer_data_to_file_16(dst_buffer, buffer_size);
}

void test_hifi4_fir_q31(float32_t * src_buffer_f32, q31_t * src_buffer_q31, float32_t * dst_buffer_f32, q31_t * dst_buffer_q31,
		size_t buffer_size, uint32_t fs)
{
	uint32_t previous_time;
	float freq[] 	= {9000.0f, 7000.0f, 1500.0f};
	float amp[] 	= {0.1f, 0.2f, 2.0f};
	int freq_cnt = sizeof(freq) / sizeof(freq[0]);

	SEMA42_Lock(APP_SEMA42, SEMA42_GATE, PROC_NUM);

	MU_SetFlags(APP_MU, SEMA42_LOCK_FLAG);

	generate_sine_wave_f32((float32_t *)&src_buffer_f32[0], buffer_size/5, fs, 0.15f, freq, amp, freq_cnt);
	generate_sine_wave_f32((float32_t *)&src_buffer_f32[1*buffer_size/5], buffer_size/5, fs, 0.40f, freq, amp, freq_cnt);
	generate_sine_wave_f32((float32_t *)&src_buffer_f32[2*buffer_size/5], buffer_size/5, fs, 0.65f, freq, amp, freq_cnt);
	generate_sine_wave_f32((float32_t *)&src_buffer_f32[3*buffer_size/5], buffer_size/5, fs, 0.8f, freq, amp, freq_cnt);
	generate_sine_wave_f32((float32_t *)&src_buffer_f32[4*buffer_size/5], buffer_size/5, fs, 0.40f, freq, amp, freq_cnt);

	arm_float_to_q31((float32_t *)src_buffer_f32, (q31_t *)src_buffer_q31, buffer_size);
	print_buffer_data_f32((float32_t *)src_buffer_f32, buffer_size);

	SEMA42_Unlock(APP_SEMA42, SEMA42_GATE);

	previous_time = MSDK_GetCpuCycleCount();
	while (SEMA42_DSP_LOCK_FLAG != MU_GetFlags(APP_MU))
	{
	}

	SEMA42_Lock(APP_SEMA42, SEMA42_GATE, (uint8_t)1U);
	PRINTF("Execution time: %.3f ms\r\n", CYCLES_TO_MS(MSDK_GetCpuCycleCount() - previous_time));
	arm_q31_to_float((q31_t *)dst_buffer_q31, (float32_t *)dst_buffer_f32, buffer_size);
	print_buffer_data_f32((float32_t *)dst_buffer_f32, buffer_size);
}

void test_hifi4_fir_f32(float32_t * src_buffer_f32, float32_t * dst_buffer_f32, size_t buffer_size, uint32_t fs)
{
	uint32_t previous_time;
	float freq[] 	= {9000.0f, 7000.0f, 1500.0f};
	float amp[] 	= {0.1f, 0.2f, 2.0f};
	int freq_cnt = sizeof(freq) / sizeof(freq[0]);

	SEMA42_Lock(APP_SEMA42, SEMA42_GATE, PROC_NUM);

	MU_SetFlags(APP_MU, SEMA42_LOCK_FLAG);

	generate_sine_wave_f32((float32_t *)&src_buffer_f32[0], buffer_size/5, fs, 0.15f, freq, amp, freq_cnt);
	generate_sine_wave_f32((float32_t *)&src_buffer_f32[1*buffer_size/5], buffer_size/5, fs, 0.40f, freq, amp, freq_cnt);
	generate_sine_wave_f32((float32_t *)&src_buffer_f32[2*buffer_size/5], buffer_size/5, fs, 0.65f, freq, amp, freq_cnt);
	generate_sine_wave_f32((float32_t *)&src_buffer_f32[3*buffer_size/5], buffer_size/5, fs, 0.8f, freq, amp, freq_cnt);
	generate_sine_wave_f32((float32_t *)&src_buffer_f32[4*buffer_size/5], buffer_size/5, fs, 0.40f, freq, amp, freq_cnt);

	print_buffer_data_f32((float32_t *)src_buffer_f32, buffer_size);

	SEMA42_Unlock(APP_SEMA42, SEMA42_GATE);

	previous_time = MSDK_GetCpuCycleCount();
	while (SEMA42_DSP_LOCK_FLAG != MU_GetFlags(APP_MU))
	{
	}

	SEMA42_Lock(APP_SEMA42, SEMA42_GATE, (uint8_t)1U);
	PRINTF("Execution time: %.3f ms\r\n", CYCLES_TO_MS(MSDK_GetCpuCycleCount() - previous_time));
	print_buffer_data_f32((float32_t *)dst_buffer_f32, buffer_size);
}
