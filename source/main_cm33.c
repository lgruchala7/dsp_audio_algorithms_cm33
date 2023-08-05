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

#include "drc_algorithms_cm33.h"
#include "drc_algorithms_cm33_conf.h"
#include "algorithm_testbench.h"
#include "main_cm33.h"

#include <stdbool.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define I2S_MASTER_CLOCK_FREQUENCY 	CLOCK_GetMclkClkFreq()
#define AUDIO_SAMPLE_RATE          	(48000U)
#define AUDIO_PROTOCOL             	kCODEC_BusI2S
#define I2S_TX                     	(I2S3)
#define I2S_RX                     	(I2S1)
#define DMA_ENGINE                 	(DMA0)
#define I2S_TX_CHANNEL             	(7)
#define I2S_RX_CHANNEL             	(2)
#define I2S_CLOCK_DIVIDER          	(16)
#define I2S_TX_MODE                	kI2S_MasterSlaveNormalMaster
#define I2S_RX_MODE                	kI2S_MasterSlaveNormalMaster
#define CODEC_I2C_BASEADDR         	I2C2
//#define CODEC_I2C_INSTANCE         2U
#define CODEC_VOLUME               	100U
#define BUFFER_SIZE					160U
#define DMA_DESCRIPTOR_NUM      	2U
#define I2S_TRANSFER_NUM			2U

#define TEST_ARR_SIZE		800U
#define ITER_COUNT			100

#define BOOT_FLAG 				0x01U
#define APP_MU_IRQHandler_0	MU_A_IRQHandler

/*******************************************************************************
 * Type definitions
 ******************************************************************************/
typedef struct {
	volatile bool is_hifi4_processing;
	volatile unsigned long start_time;
    unsigned long exec_time_sum;
} hifi4_ctrl_t;

enum {
	SRC_BUFFER_1_SEND,
	SRC_BUFFER_2_SEND,
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
    .format       = {.sampleRate = AUDIO_SAMPLE_RATE, .bitWidth = 16U},
    .bus          = kCS42448_BusI2S,
    .slaveAddress = CS42448_I2C_ADDR,
};

codec_config_t boardCodecConfig = {.codecDevType = kCODEC_CS42448, .codecDevConfig = &cs42448Config};

hifi4_ctrl_t hifi4_ctrl;

SDK_ALIGN(static dma_descriptor_t txDmaDescriptors[DMA_DESCRIPTOR_NUM], FSL_FEATURE_DMA_LINK_DESCRIPTOR_ALIGN_SIZE);
SDK_ALIGN(static dma_descriptor_t rxDmaDescriptors[DMA_DESCRIPTOR_NUM], FSL_FEATURE_DMA_LINK_DESCRIPTOR_ALIGN_SIZE);
static int16_t src_buffer_16_1[BUFFER_SIZE] __attribute__((aligned(4)));
static int16_t src_buffer_16_2[BUFFER_SIZE] __attribute__((aligned(4)));
static int16_t dst_buffer_16_1[BUFFER_SIZE] __attribute__((aligned(4)));
static int16_t dst_buffer_16_2[BUFFER_SIZE] __attribute__((aligned(4)));
static float32_t src_buffer_f32_1[BUFFER_SIZE] __attribute__((aligned(4)));
static float32_t src_buffer_f32_2[BUFFER_SIZE] __attribute__((aligned(4)));
static float32_t * dst_buffer_f32_1 = NULL;
static float32_t * dst_buffer_f32_2 = NULL;
static float32_t src_buffer_q31_1[BUFFER_SIZE] __attribute__((aligned(4)));
static float32_t src_buffer_q31_2[BUFFER_SIZE] __attribute__((aligned(4)));
static float32_t * dst_buffer_q31_1 = NULL;
static float32_t * dst_buffer_q31_2 = NULL;
static dma_handle_t dmaTxHandle;
static dma_handle_t dmaRxHandle;
static i2s_config_t txConfig;
static i2s_config_t rxConfig;
static i2s_dma_handle_t txHandle;
static i2s_dma_handle_t rxHandle;
static i2s_transfer_t rxTransfer[I2S_TRANSFER_NUM];
static i2s_transfer_t txTransfer[I2S_TRANSFER_NUM];
static codec_handle_t codecHandle;
extern codec_config_t boardCodecConfig;

static float test_arr[TEST_ARR_SIZE] = {0.0f};
volatile int16_t src_test_arr_16[TEST_ARR_SIZE] = {0};
volatile int16_t dst_test_arr_16[TEST_ARR_SIZE] = {0};
float32_t src_test_arr_f32[TEST_ARR_SIZE] = {0.0f};
#ifdef Q31
q31_t src_test_arr_q31[TEST_ARR_SIZE] = {(q31_t)0};
volatile float32_t dst_test_arr_f32[TEST_ARR_SIZE] = {0.0f};
volatile q31_t * dst_test_arr_q31 = NULL;
#else
volatile float32_t * dst_test_arr_f32 = NULL;
#endif
int volatile program_stage = SRC_BUFFER_1_SEND;

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
    MU_EnableInterrupts(APP_MU, (kMU_Tx0EmptyInterruptEnable | kMU_Rx0FullInterruptEnable));

    while (!hifi4_ctrl.is_hifi4_processing)
    {
    }

#ifndef Q31
    test_hifi4_fir_f32(src_test_arr_f32, (float32_t *)dst_test_arr_f32, TEST_ARR_SIZE, AUDIO_SAMPLE_RATE);
#else
    test_hifi4_fir_q31(src_test_arr_f32, src_test_arr_q31, (float32_t *)dst_test_arr_f32, (q31_t *)dst_test_arr_q31,
    		TEST_ARR_SIZE, AUDIO_SAMPLE_RATE);
#endif
}

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
    MU_EnableInterrupts(APP_MU, (kMU_Tx0EmptyInterruptEnable | kMU_Rx0FullInterruptEnable));

    while (!hifi4_ctrl.is_hifi4_processing)
    {
    }
}

static void start_digital_loopback(void)
{
    bool intA = true;
    i2s_dma_handle_t * handle;
    dma_handle_t * dma_handle;
    i2s_transfer_t *currentTransfer;

    PRINTF("Setup digital loopback\r\n");

    rxTransfer[0].data     = (uint8_t *)&src_buffer_16_1[0];
    rxTransfer[0].dataSize = sizeof(src_buffer_16_1);

    rxTransfer[1].data     = (uint8_t *)&src_buffer_16_2[0];
    rxTransfer[1].dataSize = sizeof(src_buffer_16_2);

    txTransfer[0].data     = (uint8_t *)&dst_buffer_16_1[0];
    txTransfer[0].dataSize = sizeof(dst_buffer_16_1);

    txTransfer[1].data     = (uint8_t *)&dst_buffer_16_2[0];
    txTransfer[1].dataSize = sizeof(dst_buffer_16_2);

    I2S_RxTransferCreateHandleDMA(I2S_RX, &rxHandle, &dmaRxHandle, RxCallback, (void *)&rxTransfer);
    I2S_TxTransferCreateHandleDMA(I2S_TX, &txHandle, &dmaTxHandle, TxCallback, (void *)&txTransfer);

    I2S_TransferInstallLoopDMADescriptorMemory(&rxHandle, rxDmaDescriptors, DMA_DESCRIPTOR_NUM);
    I2S_TransferInstallLoopDMADescriptorMemory(&txHandle, txDmaDescriptors, DMA_DESCRIPTOR_NUM);


    if (I2S_TransferSendLoopDMA(I2S_TX, &txHandle, &txTransfer[0], 2U) != kStatus_Success)
    {
        assert(false);
    }

    if (I2S_TransferReceiveLoopDMA(I2S_RX, &rxHandle, &rxTransfer[0], 2U) != kStatus_Success)
    {
        assert(false);
    }
}

static void TxCallback(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData)
{
	I2S_TransferAbortDMA(I2S_TX, &txHandle);
}

static void RxCallback(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData)
{
	i2s_transfer_t *transfer = (i2s_transfer_t *)userData;
	static int call_cnt = 0;
	static bool is_intA = false;

	if (call_cnt == 1000)
	{
		SEMA42_Lock(APP_SEMA42, SEMA42_GATE, PROC_NUM);
		MU_SetFlags(APP_MU, SEMA42_LOCK_FLAG);

		assert(transfer->dataSize == (BUFFER_SIZE * (sizeof(src_buffer_16_1[0]) / sizeof(transfer->data[0]))));
		assert((void *)&transfer->data[0] == (void *)&src_buffer_16_1[0]);

		for (int i = 0, j = 0, k = (BUFFER_SIZE/2); i < BUFFER_SIZE; i += 2, ++j, ++k)
		{
			src_buffer_f32_1[j] = (float32_t)src_buffer_16_1[i];
			src_buffer_f32_1[k] = (float32_t)src_buffer_16_1[i+1];
		}

		MU_SetFlags(APP_MU, SEMA42_UNLOCK_FLAG);
		SEMA42_Unlock(APP_SEMA42, SEMA42_GATE);
		while (SEMA42_DSP_LOCK_FLAG != MU_GetFlags(APP_MU))
		{
		}
		SEMA42_Lock(APP_SEMA42, SEMA42_GATE, PROC_NUM);

		for (int i = 0, j = 0, k = (BUFFER_SIZE/2); i < BUFFER_SIZE; i += 2, ++j, ++k)
		{
			dst_buffer_16_1[i] = (int16_t)dst_buffer_f32_1[j];
			dst_buffer_16_1[i+1] = (int16_t)dst_buffer_f32_1[k];
		}
	}
	else if (call_cnt == 1001)
	{
		I2S_TransferAbortDMA(I2S_RX, &rxHandle);

		SEMA42_Lock(APP_SEMA42, SEMA42_GATE, PROC_NUM);
		MU_SetFlags(APP_MU, SEMA42_LOCK_FLAG);

		assert(transfer->dataSize == (BUFFER_SIZE * (sizeof(src_buffer_16_2[0]) / sizeof(transfer->data[0]))));
		assert((void *)&transfer->data[0] == (void *)&src_buffer_16_1[0]);

		for (int i = 0, j = 0, k = (BUFFER_SIZE/2); i < BUFFER_SIZE; i += 2, ++j, ++k)
		{
			src_buffer_f32_2[j] = (float32_t)src_buffer_16_2[i];
			src_buffer_f32_2[k] = (float32_t)src_buffer_16_2[i+1];
		}

		PRINTF("\r\nsrc_buffer_16_1:\r\n");
		print_buffer_data_16(src_buffer_16_1, BUFFER_SIZE);
		PRINTF("\r\nsrc_buffer_f32_1:\r\n");
		print_buffer_data_f32((float32_t *)src_buffer_f32_1, BUFFER_SIZE);
		PRINTF("\r\nsrc_buffer_16_2:\r\n");
		print_buffer_data_16(src_buffer_16_2, BUFFER_SIZE);
		PRINTF("\r\nsrc_buffer_f32_2:\r\n");
		print_buffer_data_f32((float32_t *)src_buffer_f32_2, BUFFER_SIZE);

		MU_SetFlags(APP_MU, SEMA42_UNLOCK_FLAG);
		SEMA42_Unlock(APP_SEMA42, SEMA42_GATE);
		while (SEMA42_DSP_LOCK_FLAG != MU_GetFlags(APP_MU))
		{
		}
		SEMA42_Lock(APP_SEMA42, SEMA42_GATE, PROC_NUM);

		for (int i = 0, j = 0, k = (BUFFER_SIZE/2); i < BUFFER_SIZE; i += 2, ++j, ++k)
		{
			dst_buffer_16_2[i] = (int16_t)dst_buffer_f32_2[j];
			dst_buffer_16_2[i+1] = (int16_t)dst_buffer_f32_2[k];
		}

		PRINTF("\r\ndst_buffer_16_1:\r\n");
		print_buffer_data_16(dst_buffer_16_1, BUFFER_SIZE);
		PRINTF("\r\ndst_buffer_f32_1:\r\n");
		print_buffer_data_f32((float32_t *)dst_buffer_f32_1, BUFFER_SIZE);
		PRINTF("\r\ndst_buffer_16_2:\r\n");
		print_buffer_data_16(dst_buffer_16_2, BUFFER_SIZE);
		PRINTF("\r\ndst_buffer_f32_2:\r\n");
		print_buffer_data_f32((float32_t *)dst_buffer_f32_2, BUFFER_SIZE);
	}

	call_cnt++;
	is_intA = (is_intA == true ? false : true);

//	if (is_intA)
//	{
//		assert(transfer->dataSize == (BUFFER_SIZE * (sizeof(src_buffer_16_1[0]) / sizeof(transfer->data[0]))));
//		assert((void *)&transfer->data[0] == (void *)&src_buffer_16_1[0]);
//
//		for (int i = 0; i < BUFFER_SIZE; ++i)
//		{
//			src_buffer_f32_1[i] = (float32_t)src_buffer_16_1[i];
//		}
//
////		PRINTF("\r\nsrc_buffer_16_1:\r\n");
////		print_buffer_data_16(src_buffer_16_1, BUFFER_SIZE);
////		PRINTF("\r\nsrc_buffer_f32_1:\r\n");
////		print_buffer_data_f32((float32_t *)src_buffer_f32_1, BUFFER_SIZE);
//
//		MU_SetFlags(APP_MU, SEMA42_UNLOCK_FLAG);
//		SEMA42_Unlock(APP_SEMA42, SEMA42_GATE);
//		while (SEMA42_DSP_LOCK_FLAG != MU_GetFlags(APP_MU))
//		{
//		}
//		SEMA42_Lock(APP_SEMA42, SEMA42_GATE, PROC_NUM);
//
//		for (int i = 0; i < BUFFER_SIZE; ++i)
//		{
//			dst_buffer_16_1[i] = (int16_t)dst_buffer_f32_1[i];
//		}
//
////		PRINTF("\r\ndst_buffer_16_1:\r\n");
////		print_buffer_data_16(dst_buffer_16_1, BUFFER_SIZE);
////		PRINTF("\r\ndst_buffer_f32_1:\r\n");
////		print_buffer_data_f32((float32_t *)dst_buffer_f32_1, BUFFER_SIZE);
//	}
//	else
//	{
//		i2s_transfer_t *transfer = (i2s_transfer_t *)userData;
//		assert(transfer->dataSize == (BUFFER_SIZE * (sizeof(src_buffer_16_2[0]) / sizeof(transfer->data[0]))));
//		assert((void *)&transfer->data[0] == (void *)&src_buffer_16_2[0]);
//
//		for (int i = 0; i < BUFFER_SIZE; ++i)
//		{
//			src_buffer_f32_2[i] = (float32_t)src_buffer_16_2[i];
//		}
//
//		PRINTF("\r\nsrc_buffer_16_2:\r\n");
//		print_buffer_data_16(src_buffer_16_2, BUFFER_SIZE);
//		PRINTF("\r\nsrc_buffer_f32_2:\r\n");
//		print_buffer_data_f32((float32_t *)src_buffer_f32_2, BUFFER_SIZE);
//
//		MU_SetFlags(APP_MU, SEMA42_UNLOCK_FLAG);
//		SEMA42_Unlock(APP_SEMA42, SEMA42_GATE);
//		while (SEMA42_DSP_LOCK_FLAG != MU_GetFlags(APP_MU))
//		{
//		}
//		SEMA42_Lock(APP_SEMA42, SEMA42_GATE, PROC_NUM);
//
//		for (int i = 0; i < BUFFER_SIZE; ++i)
//		{
//			dst_buffer_16_2[i] = (int16_t)dst_buffer_f32_2[i];
//		}
//
//		PRINTF("\r\ndst_buffer_16_2:\r\n");
//		print_buffer_data_16(dst_buffer_16_2, BUFFER_SIZE);
//		PRINTF("\r\ndst_buffer_f32_2:\r\n");
//		print_buffer_data_f32((float32_t *)dst_buffer_f32_2, BUFFER_SIZE);
//	}
}

static void configure_codec(void)
{
    cs42448Config.i2cConfig.codecI2CSourceClock = CLOCK_GetFlexCommClkFreq(2);
    cs42448Config.format.mclk_HZ                = CLOCK_GetMclkClkFreq();

    /* protocol: i2s
     * sampleRate: 48K
     * bitWidth: 16
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
     * dataLength = 16;
     * frameLength = 32;
     * position = 0;
     * watermark = 4;
     * txEmptyZero = true;
     * pack48 = false;
     */
    I2S_TxGetDefaultConfig(&txConfig);
    txConfig.divider     = I2S_CLOCK_DIVIDER;
    txConfig.masterSlave = I2S_TX_MODE;

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
     * dataLength = 16;
     * frameLength = 32;
     * position = 0;
     * watermark = 4;
     * txEmptyZero = false;
     * pack48 = true;
     */
    I2S_RxGetDefaultConfig(&rxConfig);
    rxConfig.divider     = I2S_CLOCK_DIVIDER;
    rxConfig.masterSlave = I2S_RX_MODE;

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

void APP_MU_IRQHandler_test(void)
{
//    uint32_t flag = MU_GetStatusFlags(APP_MU);
//
//    if (((flag & kMU_Tx0EmptyFlag) == kMU_Tx0EmptyFlag) && (program_stage == SRC_BUFFER_SEND))
//    {
//    	MU_ClearStatusFlags(APP_MU, kMU_Tx0EmptyFlag);
//		MU_DisableInterrupts(APP_MU, kMU_Tx0EmptyInterruptEnable);
//#ifndef Q31
//		MU_SendMsgNonBlocking(APP_MU, CHN_MU_REG_NUM, (uint32_t)src_test_arr_f32);
//#else
//		MU_SendMsgNonBlocking(APP_MU, CHN_MU_REG_NUM, (uint32_t)src_test_arr_q31);
//#endif
//		program_stage = DST_BUFFER_RCV;
//    }
//    else if (((flag & kMU_Rx0FullFlag) == kMU_Rx0FullFlag) && (program_stage == DST_BUFFER_RCV))
//    {
//#ifndef Q31
//    	dst_test_arr_f32 = (float32_t *)MU_ReceiveMsgNonBlocking(APP_MU, CHN_MU_REG_NUM);
//#else
//    	dst_test_arr_q31 = (q31_t *)MU_ReceiveMsgNonBlocking(APP_MU, CHN_MU_REG_NUM);
//#endif
//    	hifi4_ctrl.is_hifi4_processing = true;
//    	MU_ClearStatusFlags(APP_MU, kMU_Rx0FullFlag);
//        MU_DisableInterrupts(APP_MU, kMU_Rx0FullInterruptEnable);
//		program_stage = RUN;
//    }
//    else
//    {
//    	PRINTF("Unexpected interrupt\r\n");
//    }
}

void APP_MU_IRQHandler_0(void)
{
    uint32_t flag = MU_GetStatusFlags(APP_MU);

    if ((flag & kMU_Tx0EmptyFlag) == kMU_Tx0EmptyFlag && (program_stage < DST_BUFFER_1_RCV))
    {
		MU_ClearStatusFlags(APP_MU, kMU_Tx0EmptyFlag);
    	switch (program_stage)
    	{
    		case SRC_BUFFER_1_SEND:
    		{
				#ifndef Q31
    			MU_SendMsgNonBlocking(APP_MU, CHN_MU_REG_NUM, (uint32_t)src_buffer_f32_1);
				#else
    			MU_SendMsgNonBlocking(APP_MU, CHN_MU_REG_NUM, (uint32_t)src_buffer_q31_1);
				#endif
    			program_stage = SRC_BUFFER_2_SEND;
    			break;
    		}
    		case SRC_BUFFER_2_SEND:
    		{
    			MU_DisableInterrupts(APP_MU, kMU_Tx0EmptyInterruptEnable);
				#ifndef Q31
    			MU_SendMsgNonBlocking(APP_MU, CHN_MU_REG_NUM, (uint32_t)src_buffer_f32_2);
				#else
    			MU_SendMsgNonBlocking(APP_MU, CHN_MU_REG_NUM, (uint32_t)src_buffer_q31_2);
				#endif
    			program_stage = DST_BUFFER_1_RCV;
    			break;
    		}
    		default:
    		{
    			PRINTF("Program flow error - unexpected interrupt\r\nExiting with code -1\r\n");
    			exit(-1);
    		}
    	}
    }
    else if ((flag & kMU_Rx0FullFlag) == kMU_Rx0FullFlag && (program_stage > SRC_BUFFER_2_SEND))
    {
    	MU_ClearStatusFlags(APP_MU, kMU_Rx0FullFlag);
    	switch (program_stage)
    	{
    		case DST_BUFFER_1_RCV:
    		{
				#ifndef Q31
				dst_buffer_f32_1 = (float32_t *)MU_ReceiveMsgNonBlocking(APP_MU, CHN_MU_REG_NUM);
				#else
				dst_buffer_q31_1 = (q31_t *)MU_ReceiveMsgNonBlocking(APP_MU, CHN_MU_REG_NUM);
				#endif
				program_stage = DST_BUFFER_2_RCV;
    			break;
			}
    		case DST_BUFFER_2_RCV:
    		{
    			MU_DisableInterrupts(APP_MU, kMU_Rx0FullInterruptEnable);
				#ifndef Q31
				dst_buffer_f32_2 = (float32_t *)MU_ReceiveMsgNonBlocking(APP_MU, CHN_MU_REG_NUM);
				#else
				dst_buffer_q31_2 = (q31_t *)MU_ReceiveMsgNonBlocking(APP_MU, CHN_MU_REG_NUM);
				#endif
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

/*!
 * @brief Main function
 */
int main(void)
{
	check_coefficients();

	#ifdef PQ_USED
    /* Power up PQ RAM. */
    SYSCTL0->PDRUNCFG1_CLR = SYSCTL0_PDRUNCFG1_PQ_SRAM_APD_MASK | SYSCTL0_PDRUNCFG1_PQ_SRAM_PPD_MASK;

    /* Apply power setting. */
    POWER_ApplyPD();
    PQ_Init(POWERQUAD);
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


    PRINTF("\nConfigure clocks\r\n");
    configure_clocks();

    PRINTF("Configure codec\r\n");
    configure_codec();

    PRINTF("Configure I2S\r\n");
    configure_i2s();

    PRINTF("Configure DMA\r\n");
    configure_dma();

    MSDK_EnableCpuCycleCounter();

    calculate_coefficients();

    /*test algorithms and measure execution time */
//    PRINTF("\r\nlimiter_16:\r\n");
//    measure_algorithm_time_16(limiter_16, (int16_t *)src_test_arr_16, (int16_t *)dst_test_arr_16, sizeof(src_test_arr_16), ITER_COUNT);
//    test_drc_algorithm(limiter_16, (int16_t *)src_test_arr_16, (int16_t *)dst_test_arr_16, TEST_ARR_SIZE, AUDIO_SAMPLE_RATE);
//    PRINTF("\r\ncompressor_expander_ngate_16:\r\n");
//    measure_algorithm_time_16(compressor_expander_ngate_16, (int16_t *)src_test_arr_16, (int16_t *)dst_test_arr_16, sizeof(src_test_arr_16), ITER_COUNT);
//    PRINTF("\r\nfir_filter_16:\r\n");
//    measure_algorithm_time_16(fir_filter_16, (int16_t *)src_buffer_16_1, (int16_t *)dst_buffer_16_1, BUFFER_SIZE, ITER_COUNT);

//	test_cmsis_dsp((float32_t *)src_test_arr_f32, (float32_t *)dst_test_arr_f32, TEST_ARR_SIZE, AUDIO_SAMPLE_RATE, ITER_COUNT);
//	PRINTF("\r\n");

//    test_hifi4();

    /* check if algorithms work correctly */
//	PRINTF("\r\nCPU frequency: %d Hz\r\n", SystemCoreClock);
//	PRINTF("\r\nDSP frequency: %u Hz\r\n", CLOCK_GetDspMainClkFreq());

    init_hifi4();
    start_digital_loopback();

    while (1)
    {
    	__NOP();
    }
}
