/*
 * filters_cfg.c
 *
 *  Created on: 11 sie 2023
 *      Author: Łukasz
 */
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "filters_cfg.h"
//#include "fsl_powerquad.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint32_t fir_num_blocks = (BUFFER_SIZE / FIR_BLOCK_SIZE / 2);

const float32_t fir_filter_coeff_f32[FIR_COEFF_COUNT] = {
	/* cutoff 15kHz, order 32 */
	0.0f,	-0.00174341419120027f,	0.00185091709012007f,	0.00147746406869799f,	-0.00570147477422282f,	0.00315288456652385f,	0.00820010311920392f,	-0.0147268091348366f,	0.0f,	0.0264821719629854f,	-0.0268869521514896f,	-0.0194007179905114f,	0.0689231568626912f,	-0.0374927624304152f,	-0.108704952419648f,	0.291765273086423f,	0.625610224671355f,	0.291765273086423f,	-0.108704952419648f,	-0.0374927624304152f,	0.0689231568626912f,	-0.0194007179905114f,	-0.0268869521514896f,	0.0264821719629854f,	0.0f,	-0.0147268091348366f,	0.00820010311920392f,	0.00315288456652385f,	-0.00570147477422282f,	0.00147746406869799f,	0.00185091709012007f,	-0.00174341419120027f,	0.0f
	/* cutoff 6kHz, order 256 */
//	0.0f, -0.000142095463926298f, -0.000203598484145306f, -0.000146364768649777f, 0.0f, 0.000152796255334117f, 0.000221823870954137f, 0.000161487217690435f, 0.0f, -0.000172536476232354f, -0.000253111426525135f, -0.000186044545777425f, 0.0f, 0.000202113832835479f, 0.000298599184600296f, 0.000220848866463926f, 0.0f, -0.000242356566655504f, -0.000359470896364611f, -0.000266746554889788f, 0.0f, 0.000294131512141478f, 0.000436968307791359f, 0.000324627590413839f, 0.0f, -0.000358354884777625f, -0.000532407533612327f, -0.000395437973970450f, 0.0f, 0.000436006538883024f, 0.000647200648234585f, 0.000480196069766373f, 0.0f, -0.000528148674789083f, -0.000782883983192874f, -0.000580014004716558f, 0.0f, 0.000635950311052743f, 0.000941155138348767f, 0.000696125658073407f, 0.0f, -0.000760719312910043f, -0.00112392144914783f, -0.000829923342363559f, 0.0f, 0.000903944450630234f, 0.00133336371102959f, 0.000983006098849127f, 0.0f, -0.00106735095564239f, -0.00157202051065915f, -0.00115724373801470f, 0.0f, 0.00125297451462210f, 0.00184290081687561f, 0.00135486255918409f, 0.0f, -0.00146326086157541f, -0.00214963597346358f, -0.00157856142905363f, 0.0f, 0.00170120154169663f, 0.00249668762992459f, 0.00183167116649932f, 0.0f, -0.00197052178649341f, -0.00288963667437853f, -0.00211837696848963f, 0.0f, 0.00227594508079392f, 0.00333559205593175f, 0.00244403468771380f, 0.0f, -0.00262357330092913f, -0.00384378142477216f, -0.00281563037311102f, 0.0f, 0.00302144569694921f, 0.00442642515110156f, 0.00324246475256718f, 0.0f, -0.00348038307159067f, -0.00510006593656665f, -0.00373720242153848f, 0.0f, 0.00401530265745672f, 0.00588765744666140f, 0.00431753458898457f, 0.0f, -0.00464734140520535f, -0.00682197091934482f, -0.00500891946123627f, 0.0f, 0.00540743416951147f, 0.00795140479312829f, 0.00584931437171450f, 0.0f, -0.00634266063239365f, -0.00935043979531675f, -0.00689782244903868f, 0.0f, 0.00752823196298245f, 0.0111397418830013f, 0.00825163780957785f, 0.0f, -0.00909198266861255f, -0.0135281916709367f, -0.0100823536463049f, 0.0f, 0.0112697959941345f, 0.0169109477103547f, 0.0127235246748694f, 0.0f, -0.0145497050974079f, -0.0221356107387184f, -0.0169198775073432f, 0.0f, 0.0201303896832348f, 0.0314071709056526f, 0.0247410730777657f, 0.0f, -0.0319516492901427f, -0.0528127241883479f, -0.0448815117930890f, 0.0f, 0.0749686340967730f, 0.159142724192092f, 0.225155374452901f, 0.250119395289229f, 0.225155374452901f, 0.159142724192092f, 0.0749686340967730f, 0.0f, -0.0448815117930890f, -0.0528127241883479f, -0.0319516492901427f, 0.0f, 0.0247410730777657f, 0.0314071709056526f, 0.0201303896832348f, 0.0f, -0.0169198775073432f, -0.0221356107387184f, -0.0145497050974079f, 0.0f, 0.0127235246748694f, 0.0169109477103547f, 0.0112697959941345f, 0.0f, -0.0100823536463049f, -0.0135281916709367f, -0.00909198266861255f, 0.0f, 0.00825163780957785f, 0.0111397418830013f, 0.00752823196298245f, 0.0f, -0.00689782244903868f, -0.00935043979531675f, -0.00634266063239365f, 0.0f, 0.00584931437171450f, 0.00795140479312829f, 0.00540743416951147f, 0.0f, -0.00500891946123627f, -0.00682197091934482f, -0.00464734140520535f, 0.0f, 0.00431753458898457f, 0.00588765744666140f, 0.00401530265745672f, 0.0f, -0.00373720242153848f, -0.00510006593656665f, -0.00348038307159067f, 0.0f, 0.00324246475256718f, 0.00442642515110156f, 0.00302144569694921f, 0.0f, -0.00281563037311102f, -0.00384378142477216f, -0.00262357330092913f, 0.0f, 0.00244403468771380f, 0.00333559205593175f, 0.00227594508079392f, 0.0f, -0.00211837696848963f, -0.00288963667437853f, -0.00197052178649341f, 0.0f, 0.00183167116649932f, 0.00249668762992459f, 0.00170120154169663f, 0.0f, -0.00157856142905363f, -0.00214963597346358f, -0.00146326086157541f, 0.0f, 0.00135486255918409f, 0.00184290081687561f, 0.00125297451462210f, 0.0f, -0.00115724373801470f, -0.00157202051065915f, -0.00106735095564239f, 0.0f, 0.000983006098849127f, 0.00133336371102959f, 0.000903944450630234f, 0.0f, -0.000829923342363559f, -0.00112392144914783f, -0.000760719312910043f, 0.0f, 0.000696125658073407f, 0.000941155138348767f, 0.000635950311052743f, 0.0f, -0.000580014004716558f, -0.000782883983192874f, -0.000528148674789083f, 0.0f, 0.000480196069766373f, 0.000647200648234585f, 0.000436006538883024f, 0.0f, -0.000395437973970450f, -0.000532407533612327f, -0.000358354884777625f, 0.0f, 0.000324627590413839f, 0.000436968307791359f, 0.000294131512141478f, 0.0f, -0.000266746554889788f, -0.000359470896364611f, -0.000242356566655504f, 0.0f, 0.000220848866463926f, 0.000298599184600296f, 0.000202113832835479f, 0.0f, -0.000186044545777425f, -0.000253111426525135f, -0.000172536476232354f, 0.0f, 0.000161487217690435f, 0.000221823870954137f, 0.000152796255334117f, 0.0f, -0.000146364768649777f, -0.000203598484145306f, -0.000142095463926298f, 0.0f
};
#ifndef Q31_USED
arm_fir_instance_f32 fir_instance_f32_1;
arm_fir_instance_f32 fir_instance_f32_2;
float32_t fir_state_f32_1[FIR_BLOCK_SIZE + FIR_COEFF_COUNT - 1];
float32_t fir_state_f32_2[FIR_BLOCK_SIZE + FIR_COEFF_COUNT - 1];
#else
arm_fir_instance_q31 fir_instance_q31_1;
arm_fir_instance_q31 fir_instance_q31_2;
q31_t fir_state_q31_1[FIR_BLOCK_SIZE + FIR_COEFF_COUNT - 1];
q31_t fir_state_q31_2[FIR_BLOCK_SIZE + FIR_COEFF_COUNT - 1];
q31_t fir_filter_coeff_q31[FIR_COEFF_COUNT];
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
void init_fir_filters(void)
{
	#ifndef Q31_USED
	arm_fir_init_f32(&fir_instance_f32_1, FIR_COEFF_COUNT, (float32_t *)&fir_filter_coeff_f32[0], &fir_state_f32_1[0], FIR_BLOCK_SIZE);
	arm_fir_init_f32(&fir_instance_f32_2, FIR_COEFF_COUNT, (float32_t *)&fir_filter_coeff_f32[0], &fir_state_f32_2[0], FIR_BLOCK_SIZE);
	#else
	arm_float_to_q31(fir_filter_coeff_f32, fir_filter_coeff_q31, FIR_COEFF_COUNT);
	arm_fir_init_q31(&fir_instance_q31_1, FIR_COEFF_COUNT, (q31_t *)&fir_filter_coeff_q31[0], &fir_state_q31_1[0], FIR_BLOCK_SIZE);
	arm_fir_init_q31(&fir_instance_q31_2, FIR_COEFF_COUNT, (q31_t *)&fir_filter_coeff_q31[0], &fir_state_q31_2[0], FIR_BLOCK_SIZE);
	#endif
}

#ifndef Q31_USED
void fir_process_batch(float32_t * src_buffer, float32_t * dst_buffer, size_t buffer_size)
{
	for (uint32_t j = 0; j < fir_num_blocks; ++j)
	{
		arm_fir_f32(&fir_instance_f32_1, &src_buffer[0] + (j * FIR_BLOCK_SIZE), &dst_buffer[0] + (j * FIR_BLOCK_SIZE), FIR_BLOCK_SIZE);
		arm_fir_f32(&fir_instance_f32_2, &src_buffer[buffer_size/2] + (j * FIR_BLOCK_SIZE), &dst_buffer[buffer_size/2] + (j * FIR_BLOCK_SIZE), FIR_BLOCK_SIZE);
	}
}
#else
void fir_process_batch(q31_t * src_buffer, q31_t * dst_buffer, size_t buffer_size)
{
	for (uint32_t j = 0; j < fir_num_blocks; ++j)
	{
		arm_fir_q31(&fir_instance_q31_1, &src_buffer[0] + (j * FIR_BLOCK_SIZE), &dst_buffer[0] + (j * FIR_BLOCK_SIZE), FIR_BLOCK_SIZE);
		arm_fir_q31(&fir_instance_q31_2, &src_buffer[BUFFER_SIZE/2] + (j * FIR_BLOCK_SIZE), &dst_buffer[buffer_size/2] + (j * FIR_BLOCK_SIZE), FIR_BLOCK_SIZE);
	}
}
#endif
