/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "dsp_support.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_dma.h"
#include "fsl_i2c.h"
#include "fsl_i2s.h"
#include "fsl_i2s_dma.h"
#include "fsl_codec_common.h"
#include "fsl_codec_adapter.h"
#include "fsl_cs42448.h"
#include "fsl_powerquad.h"
#include "fsl_power.h"

#include "drc_algorithms.h"
#include "drc_algorithms_cfg.h"
#include "algorithm_testbench.h"
#include "filters_cfg.h"
#include "main_cm33.h"

#include <time.h>
#include <stdlib.h>
#include <math.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define I2S_MASTER_CLOCK_FREQUENCY 	CLOCK_GetMclkClkFreq()
#define I2S_CLOCK_DIVIDER          	8U /* I2S_DIV = audio_pll_clk / (WS * Word count * Word length) = 24.576 MHz / (48 kHz * 2 * 32)*/
#define AUDIO_SAMPLE_RATE          	48000U
#define AUDIO_PROTOCOL             	kCODEC_BusI2S
#define AUDIO_BIT_WIDTH				24U
#define DMA_ENGINE                 	(DMA0)
#define DMA_DESCRIPTOR_NUM      	2U
#define I2S_TX                     	(I2S3)
#define I2S_RX                     	(I2S1)
#define I2S_TX_CHANNEL             	7U
#define I2S_RX_CHANNEL             	2U
#define I2S_TX_MODE                	kI2S_MasterSlaveNormalMaster
#define I2S_RX_MODE                	kI2S_MasterSlaveNormalMaster
#define I2S_TRANSFER_CNT			2U
#define CODEC_I2C_BASEADDR         	(I2C2)
#define CODEC_VOLUME               	100U

#define TEST_ARR_SIZE				800U

#define APP_MU_IRQHandler_0			MU_A_IRQHandler

#define GPIO_DEBUG_PORT				0U
#define GPIO_DEBUG_PIN_TX			28U
#define GPIO_DEBUG_PIN_RX			27U

#define PQ_SET_Q31_CONFIG                                                                               \
    POWERQUAD->OUTFORMAT = ((uint32_t)(-31) << 8U) | ((uint32_t)kPQ_32Bit << 4U) | (uint32_t)kPQ_Float; \
    POWERQUAD->INAFORMAT = ((uint32_t)(0) << 8U) | ((uint32_t)kPQ_32Bit << 4U) | (uint32_t)kPQ_Float;   \
    POWERQUAD->INBFORMAT = ((uint32_t)(0) << 8U) | ((uint32_t)kPQ_32Bit << 4U) | (uint32_t)kPQ_Float;   \
    POWERQUAD->TMPFORMAT = ((uint32_t)(0) << 8U) | ((uint32_t)kPQ_Float << 4U) | (uint32_t)kPQ_Float;   \
    POWERQUAD->TMPBASE   = 0xE0000000U

/*******************************************************************************
 * Type definitions
 ******************************************************************************/
typedef struct {
	volatile bool is_hifi4_processing;
	volatile unsigned long start_time;
    unsigned long exec_time_sum;
} hifi4_ctrl_t;

enum {
	SRC_BUFFER_1_RCV,
	SRC_BUFFER_2_RCV,
	DST_BUFFER_1_RCV,
	DST_BUFFER_2_RCV,
	RUN,
	STAGES_MAX,
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void TxCallback(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData);
static void RxCallback(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData);

/*******************************************************************************
 * Variables
 ******************************************************************************/
cs42448_config_t cs42448Config = {
    .DACMode      = kCS42448_ModeSlave,
    .ADCMode      = kCS42448_ModeSlave,
    .reset        = NULL,
    .master       = false,
    .i2cConfig    = {.codecI2CInstance = BOARD_CODEC_I2C_INSTANCE},
    .format       = {.sampleRate = AUDIO_SAMPLE_RATE, .bitWidth = AUDIO_BIT_WIDTH},
    .bus          = kCS42448_BusI2S,
    .slaveAddress = CS42448_I2C_ADDR,
};

//pq_config_t pq_config = {
//	.inputAFormat = ((uint32_t)(-31) << 8U) | ((uint32_t)kPQ_32Bit << 4U) | (uint32_t)kPQ_Float,
//	.inputAPrescale = (int8_t)0,
//	.inputBFormat = ((uint32_t)(0) << 8U) | ((uint32_t)kPQ_32Bit << 4U) | (uint32_t)kPQ_Float,
//	.inputBPrescale = (int8_t)0,
//	.outputFormat = ((uint32_t)(-31) << 8U) | ((uint32_t)kPQ_32Bit << 4U) | (uint32_t)kPQ_Float,
//	.outputPrescale = (int8_t)0,
//	.tmpFormat = ((uint32_t)(0) << 8U) | ((uint32_t)kPQ_Float << 4U) | (uint32_t)kPQ_Float,
//	.tmpPrescale = (int8_t)0,
//	.machineFormat = kPQ_Float,
//	.tmpBase = 0xE0000000U,
//};

codec_config_t boardCodecConfig = {.codecDevType = kCODEC_CS42448, .codecDevConfig = &cs42448Config};

static dma_descriptor_t txDmaDescriptors[DMA_DESCRIPTOR_NUM] __attribute__((aligned(FSL_FEATURE_DMA_LINK_DESCRIPTOR_ALIGN_SIZE)));
static dma_descriptor_t rxDmaDescriptors[DMA_DESCRIPTOR_NUM] __attribute__((aligned(FSL_FEATURE_DMA_LINK_DESCRIPTOR_ALIGN_SIZE)));

#if HIFI4_USED
volatile int32_t * src_buffer_32_1 = NULL;
volatile int32_t * src_buffer_32_2 = NULL;
volatile int32_t * dst_buffer_32_1 = NULL;
volatile int32_t * dst_buffer_32_2 = NULL;
#else
volatile int32_t src_buffer_32_1[BUFFER_SIZE] __attribute__((aligned(4)));
volatile int32_t src_buffer_32_2[BUFFER_SIZE] __attribute__((aligned(4)));
volatile int32_t dst_buffer_32_1[BUFFER_SIZE] __attribute__((aligned(4)));
volatile int32_t dst_buffer_32_2[BUFFER_SIZE] __attribute__((aligned(4)));
#if Q31_USED
volatile q31_t src_buffer_q31_1[BUFFER_SIZE] __attribute__((aligned(4)));
volatile q31_t src_buffer_q31_2[BUFFER_SIZE] __attribute__((aligned(4)));
volatile q31_t dst_buffer_q31_1[BUFFER_SIZE] __attribute__((aligned(4)));
volatile q31_t dst_buffer_q31_2[BUFFER_SIZE] __attribute__((aligned(4)));
#else
volatile float32_t src_buffer_f32_1[BUFFER_SIZE] __attribute__((aligned(4)));
volatile float32_t src_buffer_f32_2[BUFFER_SIZE] __attribute__((aligned(4)));
volatile float32_t dst_buffer_f32_1[BUFFER_SIZE] __attribute__((aligned(4)));
volatile float32_t dst_buffer_f32_2[BUFFER_SIZE] __attribute__((aligned(4)));
#endif /* Q31_USED */
#endif/* HIFI4_USED */

static dma_handle_t dmaTxHandle;
static dma_handle_t dmaRxHandle;
static i2s_config_t txConfig;
static i2s_config_t rxConfig;
static i2s_dma_handle_t txHandle;
static i2s_dma_handle_t rxHandle;
static i2s_transfer_t rxTransfer[I2S_TRANSFER_CNT];
static i2s_transfer_t txTransfer[I2S_TRANSFER_CNT];
static codec_handle_t codecHandle;
extern codec_config_t boardCodecConfig;

volatile int16_t src_test_arr_16[TEST_ARR_SIZE] = {0};
volatile int16_t dst_test_arr_16[TEST_ARR_SIZE] = {0};
float32_t src_test_arr_f32[TEST_ARR_SIZE] = {0.0f};
#if Q31_USED
q31_t src_test_arr_q31[TEST_ARR_SIZE] = {(q31_t)0};
volatile float32_t dst_test_arr_f32[TEST_ARR_SIZE] = {0.0f};
volatile q31_t * dst_test_arr_q31 = NULL;
#else
volatile float32_t * dst_test_arr_f32 = NULL;
#endif

int volatile program_stage = SRC_BUFFER_1_RCV;
hifi4_ctrl_t hifi4_ctrl;
float32_t scale_down_factor = (1.0f / (float32_t)INT32_MAX);
float32_t scale_up_factor = (float32_t)INT32_MAX;

/*******************************************************************************
 * Code
 ******************************************************************************/
static void test_hifi4(void)
{
    /* Semaphore init */
    SEMA42_Init(APP_SEMA42);
    SEMA42_ResetAllGates(APP_SEMA42);

    /* MUA init */
    MU_Init(APP_MU);

    /* Copy DSP image to RAM and start DSP core. */
    BOARD_DSP_Init();
    hifi4_ctrl.is_hifi4_processing = false;

    MU_SetFlags(APP_MU, BOOT_FLAG);
    while (BOOT_FLAG != MU_GetFlags(APP_MU))
    {
    }
    MU_EnableInterrupts(APP_MU, (kMU_Rx0FullInterruptEnable));

    while (!hifi4_ctrl.is_hifi4_processing)
    {
    }

#if !(Q31_USED)
    test_hifi4_fir_f32(src_test_arr_f32, (float32_t *)dst_test_arr_f32, TEST_ARR_SIZE, AUDIO_SAMPLE_RATE);
#else
    test_hifi4_fir_q31(src_test_arr_f32, src_test_arr_q31, (float32_t *)dst_test_arr_f32, (q31_t *)dst_test_arr_q31,
    		TEST_ARR_SIZE, AUDIO_SAMPLE_RATE);
#endif
}

#if HIFI4_USED
static void init_hifi4(void)
{
    /* Semaphore init */
    SEMA42_Init(APP_SEMA42);
    SEMA42_ResetAllGates(APP_SEMA42);

    /* MUA init */
    MU_Init(APP_MU);

    /* Copy DSP image to RAM and start DSP core. */
    BOARD_DSP_Init();
    hifi4_ctrl.is_hifi4_processing = false;

    MU_SetFlags(APP_MU, BOOT_FLAG);

    while (BOOT_FLAG != MU_GetFlags(APP_MU))
    {
    }

    MU_EnableInterrupts(APP_MU, kMU_Rx0FullInterruptEnable);

    while (!hifi4_ctrl.is_hifi4_processing)
    {
    }

    SEMA42_Lock(APP_SEMA42, SEMA42_GATE, PROC_NUM);
}
#endif

static void start_digital_loopback(void)
{
    PRINTF("Setup digital loopback\r\n");

    rxTransfer[0].data     = (uint8_t *)src_buffer_32_1;
    rxTransfer[0].dataSize = BUFFER_SIZE * sizeof(int32_t);

    rxTransfer[1].data     = (uint8_t *)src_buffer_32_2;
    rxTransfer[1].dataSize = BUFFER_SIZE * sizeof(int32_t);

    txTransfer[0].data     = (uint8_t *)dst_buffer_32_1;
    txTransfer[0].dataSize = BUFFER_SIZE * sizeof(int32_t);

    txTransfer[1].data     = (uint8_t *)dst_buffer_32_2;
    txTransfer[1].dataSize = BUFFER_SIZE * sizeof(int32_t);

    I2S_RxTransferCreateHandleDMA(I2S_RX, &rxHandle, &dmaRxHandle, RxCallback, (void *)&rxTransfer);
    I2S_TxTransferCreateHandleDMA(I2S_TX, &txHandle, &dmaTxHandle, TxCallback, (void *)&txTransfer);

    I2S_TransferInstallLoopDMADescriptorMemory(&rxHandle, rxDmaDescriptors, DMA_DESCRIPTOR_NUM);
    I2S_TransferInstallLoopDMADescriptorMemory(&txHandle, txDmaDescriptors, DMA_DESCRIPTOR_NUM);

    if (I2S_TransferSendLoopDMA(I2S_TX, &txHandle, &txTransfer[0], I2S_TRANSFER_CNT) != kStatus_Success)
    {
        assert(false);
    }

    if (I2S_TransferReceiveLoopDMA(I2S_RX, &rxHandle, &rxTransfer[0], I2S_TRANSFER_CNT) != kStatus_Success)
    {
        assert(false);
    }
}

static void sleep_ms(float time_ms)
{
	uint32_t prev_time_ticks = MSDK_GetCpuCycleCount();

	while (CYCLES_TO_MS(MSDK_GetCpuCycleCount() - prev_time_ticks) < time_ms)
	{
		__NOP();
	}
}

static void TxCallback(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData)
{
	(void)userData;
	(void)completionStatus;
	(void)handle;
	(void)base;
	GPIO_PinWrite(GPIO, 0U, GPIO_DEBUG_PIN_TX, 1U);
	sleep_ms(0.05f);
	GPIO_PinWrite(GPIO, 0U, GPIO_DEBUG_PIN_TX, 0U);
}

#if !(HIFI4_USED)
static void RxCallback(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData)
{
	(void)userData;
	(void)completionStatus;
	(void)base;
	(void)handle;
	static bool is_intA = false;
	static int call_cnt = 0;

	GPIO_PinWrite(GPIO, 0U, GPIO_DEBUG_PIN_RX, 1U);

	#if DEBUG == 1
	if (call_cnt == 101)
	{
		I2S_TransferAbortDMA(I2S_RX, &rxHandle);
		PRINTF("$");
	}
	#endif

	if (!is_intA)
	{
		#if !(Q31_USED)
		for (int i = 0, j = 0, k = (BUFFER_SIZE/2); i < BUFFER_SIZE; i += 2, ++j, ++k)
		{
			src_buffer_f32_1[j] = (float32_t)src_buffer_32_1[i];
			src_buffer_f32_1[k] = (float32_t)src_buffer_32_1[i+1];
		}
		#else
		for (int i = 0, j = 0, k = (BUFFER_SIZE/2); i < BUFFER_SIZE; i += 2, ++j, ++k)
		{
			src_buffer_q31_1[j] = (q31_t)src_buffer_32_1[i];
			src_buffer_q31_1[k] = (q31_t)src_buffer_32_1[i+1];
		}
		#endif

		#if Q31_USED
		fir_process_batch((q31_t *)src_buffer_q31_1, (q31_t *)dst_buffer_q31_1);
//		iir_df1_process_batch((q31_t *)src_buffer_q31_1, (q31_t *)dst_buffer_q31_1);
		drc_full_stereo_balanced((q31_t *)src_buffer_q31_1, (q31_t *)dst_buffer_q31_1);
//		iir_df2_process_batch((q31_t *)src_buffer_q31_1, (q31_t *)dst_buffer_q31_1);
		#else
//		fir_process_batch((float32_t *)src_buffer_f32_1, (float32_t *)dst_buffer_f32_1);
//		iir_df1_process_batch((float32_t *)src_buffer_f32_1, (float32_t *)dst_buffer_f32_1);
//		iir_df2T_process_batch((float32_t *)src_buffer_f32_1, (float32_t *)dst_buffer_f32_1);
//		iir_df2_process_batch((float32_t *)src_buffer_f32_1, (float32_t *)dst_buffer_f32_1);
		drc_full_stereo_balanced((float32_t *)src_buffer_f32_1, (float32_t *)dst_buffer_f32_1);
		#endif

		#if !(Q31_USED)
		for (int i = 0, j = 0, k = (BUFFER_SIZE/2); i < BUFFER_SIZE; i += 2, ++j, ++k)
		{
			dst_buffer_32_1[i] = (int32_t)dst_buffer_f32_1[j];
			dst_buffer_32_1[i+1] = (int32_t)dst_buffer_f32_1[k];
		}
		#else
		for (int i = 0, j = 0, k = (BUFFER_SIZE/2); i < BUFFER_SIZE; i += 2, ++j, ++k)
		{
			dst_buffer_32_1[i] = (int32_t)dst_buffer_q31_1[j];
			dst_buffer_32_1[i+1] = (int32_t)dst_buffer_q31_1[k];
		}
		#endif
	}
	else
	{
		#if !(Q31_USED)
		for (int i = 0, j = 0, k = (BUFFER_SIZE/2); i < BUFFER_SIZE; i += 2, ++j, ++k)
		{
			src_buffer_f32_2[j] = (float32_t)src_buffer_32_2[i];
			src_buffer_f32_2[k] = (float32_t)src_buffer_32_2[i+1];
		}
		#else
		for (int i = 0, j = 0, k = (BUFFER_SIZE/2); i < BUFFER_SIZE; i += 2, ++j, ++k)
		{
			src_buffer_q31_2[j] = (q31_t)src_buffer_32_2[i];
			src_buffer_q31_2[k] = (q31_t)src_buffer_32_2[i+1];
		}
		#endif

		#if !(Q31_USED)
//		fir_process_batch((float32_t *)src_buffer_f32_2, (float32_t *)dst_buffer_f32_2);
//		iir_df1_process_batch((float32_t *)src_buffer_f32_2, (float32_t *)dst_buffer_f32_2);
//		iir_df2T_process_batch((float32_t *)src_buffer_f32_2, (float32_t *)dst_buffer_f32_2);
//		iir_df2_process_batch((float32_t *)src_buffer_f32_2, (float32_t *)dst_buffer_f32_2);
		drc_full_stereo_balanced((float32_t *)src_buffer_f32_2, (float32_t *)dst_buffer_f32_2);
		#else
		fir_process_batch((q31_t *)src_buffer_q31_2, (q31_t *)dst_buffer_q31_2);
//		iir_df1_process_batch((q31_t *)src_buffer_q31_2, (q31_t *)dst_buffer_q31_2);
		drc_full_stereo_balanced((q31_t *)src_buffer_q31_2, (q31_t *)dst_buffer_q31_2);
//		iir_df2_process_batch((q31_t *)src_buffer_q31_2, (q31_t *)dst_buffer_q31_2);
		#endif

		#if !(Q31_USED)
		for (int i = 0, j = 0, k = (BUFFER_SIZE/2); i < BUFFER_SIZE; i += 2, ++j, ++k)
		{
			dst_buffer_32_2[i] = (int32_t)dst_buffer_f32_2[j];
			dst_buffer_32_2[i+1] = (int32_t)dst_buffer_f32_2[k];
		}
		#else
		for (int i = 0, j = 0, k = (BUFFER_SIZE/2); i < BUFFER_SIZE; i += 2, ++j, ++k)
		{
			dst_buffer_32_2[i] = (int32_t)dst_buffer_q31_2[j];
			dst_buffer_32_2[i+1] = (int32_t)dst_buffer_q31_2[k];
		}
		#endif
	}

	#if DEBUG == 1
	if (call_cnt == 101)
	{
		for (int i = 0; i < BUFFER_SIZE; i++)
		{
			PRINTF("%d, ", src_buffer_32_1[i]);
			if (i%20 == 18)
			{
				PRINTF("\r\n");
			}
		}
		for (int i = 0; i < BUFFER_SIZE; i++)
		{
			PRINTF("%d, ", src_buffer_32_2[i]);
			if (i%20 == 18)
			{
				PRINTF("\r\n");
			}
		}
		PRINTF("$");
		for (int i = 0; i < BUFFER_SIZE; i++)
		{
			PRINTF("%d, ", dst_buffer_32_1[i]);
			if (i%20 == 18)
			{
				PRINTF("\r\n");
			}
		}
		for (int i = 0; i < BUFFER_SIZE; i++)
		{
			PRINTF("%d, ", dst_buffer_32_2[i]);
			if (i%20 == 18)
			{
				PRINTF("\r\n");
			}
		}
		PRINTF("$");
		for (int i = 0; i < BUFFER_SIZE; i++)
		{
			PRINTF("%.3f, ", x_peak_log_f32_1[i]);
			if (i%20 == 18)
			{
				PRINTF("\r\n");
			}
		}
		for (int i = 0; i < BUFFER_SIZE; i++)
		{
			PRINTF("%.3f, ", x_peak_log_f32_2[i]);
			if (i%20 == 18)
			{
				PRINTF("\r\n");
			}
		}
//		PRINTF("\nsrc_buffer_32_1\r\n");
//		print_buffer_data_32(src_buffer_32_1, BUFFER_SIZE);
//		PRINTF("\nsrc_buffer_32_2\r\n");
//		print_buffer_data_32(src_buffer_32_2, BUFFER_SIZE);
//		PRINTF("\ndst_buffer_32_1\r\n");
//		print_buffer_data_32(dst_buffer_32_1, BUFFER_SIZE);
//		PRINTF("\ndst_buffer_32_2\r\n");
//		print_buffer_data_32(dst_buffer_32_2, BUFFER_SIZE);
//		PRINTF("\nx_peak_log_1\r\n");
//		print_buffer_data_f32(x_peak_log_1_f32, BUFFER_SIZE);
//		PRINTF("\nx_peak_log_2\r\n");
//		print_buffer_data_f32(x_peak_log_2_f32, BUFFER_SIZE);
	}
	#endif /* DEBUG */

	call_cnt++;
	is_intA = (is_intA == true ? false : true);

	GPIO_PinWrite(GPIO, 0U, GPIO_DEBUG_PIN_RX, 0U);
}
#else
static void RxCallback(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData)
{
	(void)completionStatus;
	(void)base;
	(void)handle;
	i2s_transfer_t *transfer = (i2s_transfer_t *)userData;
	static bool is_intA = false;

	GPIO_PinWrite(GPIO, 0U, GPIO_DEBUG_PIN_RX, 1U);

	if (!is_intA)
	{
		assert(transfer->dataSize == (BUFFER_SIZE * (sizeof(src_buffer_32_1[0]) / sizeof(transfer->data[0]))));
		assert((void *)&transfer->data[0] == (void *)&src_buffer_32_1[0]);

		MU_SetFlags(APP_MU, SEMA42_UNLOCK_FLAG);
		SEMA42_Unlock(APP_SEMA42, SEMA42_GATE);

		while (SEMA42_DSP_UNLOCK_FLAG != MU_GetFlags(APP_MU))
		{
		}

		MU_SetFlags(APP_MU, SEMA42_LOCK_FLAG);
		SEMA42_Lock(APP_SEMA42, SEMA42_GATE, PROC_NUM);
	}
	else
	{
		assert(transfer->dataSize == (BUFFER_SIZE * (sizeof(src_buffer_32_2[0]) / sizeof(transfer->data[0]))));
		assert((void *)&transfer->data[0] == (void *)&src_buffer_32_1[0]);

		MU_SetFlags(APP_MU, SEMA42_UNLOCK_FLAG);
		SEMA42_Unlock(APP_SEMA42, SEMA42_GATE);

		while (SEMA42_DSP_UNLOCK_FLAG != MU_GetFlags(APP_MU))
		{
		}

		MU_SetFlags(APP_MU, SEMA42_LOCK_FLAG);
		SEMA42_Lock(APP_SEMA42, SEMA42_GATE, PROC_NUM);
	}

	is_intA = (is_intA == true ? false : true);

	GPIO_PinWrite(GPIO, 0U, GPIO_DEBUG_PIN_RX, 0U);
}
#endif

static void configure_codec(void)
{
    cs42448Config.i2cConfig.codecI2CSourceClock = CLOCK_GetFlexCommClkFreq(2);
    cs42448Config.format.mclk_HZ                = CLOCK_GetMclkClkFreq();

    /* protocol: i2s
     * sampleRate: 48K
     * bitWidth: 24
     */
    if (CODEC_Init(&codecHandle, &boardCodecConfig) != kStatus_Success)
    {
        PRINTF("codec_Init failed!\r\n");
        assert(false);
    }
    /* Initial volume kept low for hearing safety.
     * Adjust it to your needs, 0-100, 0 for mute, 100 for maximum volume.
     */
    if (CODEC_SetVolume(&codecHandle, kCODEC_PlayChannelHeadphoneLeft | kCODEC_PlayChannelHeadphoneRight,
                        CODEC_VOLUME) != kStatus_Success)
    {
        assert(false);
    }
}

static void configure_i2s(void)
{
    /*
     * masterSlave = kI2S_MasterSlaveNormalMaster;
     * mode = kI2S_ModeI2sClassic;
     * rightLow = false;
     * leftJust = false;
     * pdmData = false;
     * sckPol = false;
     * wsPol = false;
     * divider = 1;
     * oneChannel = false;
     * dataLength = 32;
     * frameLength = 64;
     * position = 0;
     * watermark = 4;
     * txEmptyZero = true;
     * pack48 = false;
     */
    I2S_TxGetDefaultConfig(&txConfig);
    txConfig.divider     = I2S_CLOCK_DIVIDER;
    txConfig.masterSlave = I2S_TX_MODE;
    txConfig.dataLength = 32U;
    txConfig.frameLength = 64U;

    /*
     * masterSlave = kI2S_MasterSlaveNormalSlave;
     * mode = kI2S_ModeI2sClassic;
     * rightLow = false;
     * leftJust = false;
     * pdmData = false;
     * sckPol = false;
     * wsPol = false;
     * divider = 1;
     * oneChannel = false;
     * dataLength = 32;
     * frameLength = 64;
     * position = 0;
     * watermark = 4;
     * txEmptyZero = false;
     * pack48 = true;
     */
    I2S_RxGetDefaultConfig(&rxConfig);
    rxConfig.divider     = I2S_CLOCK_DIVIDER;
    rxConfig.masterSlave = I2S_RX_MODE;
    rxConfig.dataLength = 32U;
    rxConfig.frameLength = 64U;

    I2S_TxInit(I2S_TX, &txConfig);
    I2S_RxInit(I2S_RX, &rxConfig);
}

static void configure_dma(void)
{
	DMA_Init(DMA_ENGINE);

	DMA_EnableChannel(DMA_ENGINE, I2S_TX_CHANNEL);
	DMA_EnableChannel(DMA_ENGINE, I2S_RX_CHANNEL);
	DMA_SetChannelPriority(DMA_ENGINE, I2S_TX_CHANNEL, kDMA_ChannelPriority3);
	DMA_SetChannelPriority(DMA_ENGINE, I2S_RX_CHANNEL, kDMA_ChannelPriority2);
	DMA_CreateHandle(&dmaTxHandle, DMA_ENGINE, I2S_TX_CHANNEL);
	DMA_CreateHandle(&dmaRxHandle, DMA_ENGINE, I2S_RX_CHANNEL);
}

static void configure_gpio(void)
{
	uint8_t p0_27_state = 0U;
	uint8_t p0_28_state = 0U;

    gpio_pin_config_t pin_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0,
    };

    GPIO_PortInit(GPIO, GPIO_DEBUG_PORT);
    GPIO_PinInit(GPIO, GPIO_DEBUG_PORT, GPIO_DEBUG_PIN_RX, &pin_config);
    GPIO_PinInit(GPIO, GPIO_DEBUG_PORT, GPIO_DEBUG_PIN_TX, &pin_config);
	GPIO_PinWrite(GPIO, GPIO_DEBUG_PORT, GPIO_DEBUG_PIN_RX, p0_27_state);
	GPIO_PinWrite(GPIO, GPIO_DEBUG_PORT, GPIO_DEBUG_PIN_TX, p0_28_state);
}

static void configure_clocks(void)
{
    CLOCK_EnableClock(kCLOCK_InputMux);

    /* I2C */
    CLOCK_AttachClk(kFFRO_to_FLEXCOMM2);

    /* attach AUDIO PLL clock to FLEXCOMM1 (I2S1) */
    CLOCK_AttachClk(kAUDIO_PLL_to_FLEXCOMM1);
    /* attach AUDIO PLL clock to FLEXCOMM3 (I2S3) */
    CLOCK_AttachClk(kAUDIO_PLL_to_FLEXCOMM3);

    /* attach AUDIO PLL clock to MCLK */
    CLOCK_AttachClk(kAUDIO_PLL_to_MCLK_CLK);
    CLOCK_SetClkDiv(kCLOCK_DivMclkClk, 1);
    SYSCTL1->MCLKPINDIR = SYSCTL1_MCLKPINDIR_MCLKPINDIR_MASK;
}

#if HIFI4_USED
void APP_MU_IRQHandler_0(void)
{
    uint32_t flag = MU_GetStatusFlags(APP_MU);

    if ((flag & kMU_Rx0FullFlag) == kMU_Rx0FullFlag)
    {
    	MU_ClearStatusFlags(APP_MU, kMU_Rx0FullFlag);
    	switch (program_stage)
    	{
			case SRC_BUFFER_1_RCV:
			{
				src_buffer_32_1 = (int32_t *)MU_ReceiveMsgNonBlocking(APP_MU, CHN_MU_REG_NUM);
				program_stage = SRC_BUFFER_2_RCV;
				break;
			}
			case SRC_BUFFER_2_RCV:
			{
				src_buffer_32_2 = (int32_t *)MU_ReceiveMsgNonBlocking(APP_MU, CHN_MU_REG_NUM);
				program_stage = DST_BUFFER_1_RCV;
				break;
			}
    		case DST_BUFFER_1_RCV:
    		{
    			dst_buffer_32_1 = (int32_t *)MU_ReceiveMsgNonBlocking(APP_MU, CHN_MU_REG_NUM);
				program_stage = DST_BUFFER_2_RCV;
    			break;
			}
    		case DST_BUFFER_2_RCV:
    		{
    			MU_DisableInterrupts(APP_MU, kMU_Rx0FullInterruptEnable);
    			dst_buffer_32_2 = (int32_t *)MU_ReceiveMsgNonBlocking(APP_MU, CHN_MU_REG_NUM);
				hifi4_ctrl.is_hifi4_processing = true;
				program_stage = RUN;
    			break;
			}
			default:
			{
				PRINTF("Program flow error - unexpected interrupt\r\nExiting with code -2\r\n");
				exit(-2);
				break;
			}
		}
    }
}
#endif

/*!
 * @brief Main function
 */
int main(void)
{
	#if PQ_USED
    /* Power up PQ RAM. */
    SYSCTL0->PDRUNCFG1_CLR = SYSCTL0_PDRUNCFG1_PQ_SRAM_APD_MASK | SYSCTL0_PDRUNCFG1_PQ_SRAM_PPD_MASK;
    /* Apply power setting. */
    POWER_ApplyPD();
    /* Initialize POWERQUAD. */
    PQ_Init(POWERQUAD);

	#if Q31_USED
    /* Set PQ config to Q31 format */
    PQ_SET_Q31_CONFIG;
//    PQ_SetFormat(POWERQUAD, kPQ_CP_PQ, kPQ_Float);
	#else
    PQ_SetFormat(POWERQUAD, kPQ_CP_PQ, kPQ_Float);
	#endif
	#endif

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    /* Clear MUA reset */
    RESET_PeripheralReset(kMU_RST_SHIFT_RSTn);
    /* Clear SEMA42 reset */
    RESET_PeripheralReset(kSEMA_RST_SHIFT_RSTn);
    EnableIRQ(MU_A_IRQn);
    /* Clear DMA Controller 0 reset */
    RESET_PeripheralReset(kDMAC0_RST_SHIFT_RSTn);
    /* Clear ADC0 reset */
    RESET_PeripheralReset(kADC0_RST_SHIFT_RSTn);


    PRINTF("\nConfigure clocks\r\n");
    configure_clocks();

    PRINTF("Configure codec\r\n");
    configure_codec();

    PRINTF("Configure I2S\r\n");
    configure_i2s();

    PRINTF("Configure DMA\r\n");
    configure_dma();

    PRINTF("Configure GPIO\r\n");
    configure_gpio();

    MSDK_EnableCpuCycleCounter();

    calculate_coefficients();

	#if HIFI4_USED
    init_hifi4();
	#else
    init_fir_filter();
    #if PQ_USED
    init_iir_df2_filter();
	#else
    init_iir_df1_filter();
	#if Q31_USED
	#else
    init_iir_df2T_filter();
    #endif /* Q31_USED */
    #endif /* PQ_USED */
    #endif /* HIFI4_USED */

    start_digital_loopback();

    while (1)
    {
    	__NOP();
    }
}
