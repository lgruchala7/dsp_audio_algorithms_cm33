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

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_dma.h"
#include "fsl_i2c.h"
#include "fsl_i2s.h"
#include "fsl_i2s_dma.h"
#include "fsl_codec_common.h"
#include "music.h"
#include <stdbool.h>
#include "fsl_codec_adapter.h"
#include "fsl_cs42448.h"

#include "fsl_ctimer.h"

#include "drc_algorithms_cm33.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_I2S_MASTER_CLOCK_FREQUENCY CLOCK_GetMclkClkFreq()
#define DEMO_AUDIO_SAMPLE_RATE          (48000)
#define DEMO_AUDIO_PROTOCOL             kCODEC_BusI2S
#define DEMO_I2S_TX                     (I2S3)
#define DEMO_I2S_RX                     (I2S1)
#define DEMO_DMA                        (DMA0)
#define DEMO_I2S_TX_CHANNEL             (7)
#define DEMO_I2S_RX_CHANNEL             (2)
#define DEMO_I2S_CLOCK_DIVIDER          (16)
#define DEMO_I2S_TX_MODE                kI2S_MasterSlaveNormalMaster
#define DEMO_I2S_RX_MODE                kI2S_MasterSlaveNormalMaster
#define DEMO_CODEC_I2C_BASEADDR         I2C2
#define DEMO_CODEC_I2C_INSTANCE         2U
#define DEMO_CODEC_VOLUME               100U
#define S_BUFFER_SIZE					400

#define CTIMER          				CTIMER2         /* Timer 2 */
#define CTIMER_MAT0_OUT 				kCTIMER_Match_0 /* Match output 0 */
#define CTIMER_MAT1_OUT 				kCTIMER_Match_1 /* Match output 1 */
#define CTIMER_CLK_FREQ 				CLOCK_GetCtimerClkFreq(2)

#define MATCH_VAL_MS					1U
#define MS_TO_CTIMER_CLK_TICKS(ms)		(CTIMER_CLK_FREQ * ms / 1000)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void StartDigitalLoopback(void);

static void TxCallback_1(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData);

static void RxCallback_1(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData);

static void TxCallback_2(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData);

static void RxCallback_2(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData);

void ctimer_match0_callback(uint32_t flags);

static void setup_ctimer(ctimer_config_t * config, CTIMER_Type * base, ctimer_match_config_t * matchConfig,
		ctimer_match_t matchChannel);

/*******************************************************************************
 * Variables
 ******************************************************************************/
cs42448_config_t cs42448Config = {
    .DACMode      = kCS42448_ModeSlave,
    .ADCMode      = kCS42448_ModeSlave,
    .reset        = NULL,
    .master       = false,
    .i2cConfig    = {.codecI2CInstance = BOARD_CODEC_I2C_INSTANCE},
    .format       = {.sampleRate = DEMO_AUDIO_SAMPLE_RATE, .bitWidth = 16U},
    .bus          = kCS42448_BusI2S,
    .slaveAddress = CS42448_I2C_ADDR,
};

codec_config_t boardCodecConfig = {.codecDevType = kCODEC_CS42448, .codecDevConfig = &cs42448Config};

uint8_t * signal_ptr;
uint32_t signal_arr_len;

__ALIGN_BEGIN static uint8_t s_Buffer_1[S_BUFFER_SIZE] __ALIGN_END; /* 100 samples => time about 2 ms */
__ALIGN_BEGIN static uint8_t s_Buffer_2[S_BUFFER_SIZE] __ALIGN_END; /* 100 samples => time about 2 ms */
static dma_handle_t s_DmaTxHandle;
static dma_handle_t s_DmaRxHandle;
static i2s_config_t s_TxConfig;
static i2s_config_t s_RxConfig;
static i2s_dma_handle_t s_TxHandle_1;
static i2s_dma_handle_t s_RxHandle_1;
static i2s_dma_handle_t s_TxHandle_2;
static i2s_dma_handle_t s_RxHandle_2;
static i2s_transfer_t s_TxTransfer_1;
static i2s_transfer_t s_RxTransfer_1;
static i2s_transfer_t s_TxTransfer_2;
static i2s_transfer_t s_RxTransfer_2;
extern codec_config_t boardCodecConfig;
codec_handle_t codecHandle;

static volatile unsigned long time_in_ms = 0UL;
static volatile unsigned long previous_time = 0UL;
/* Match Configuration for Channel 0 */
static ctimer_match_config_t matchConfig0;
ctimer_config_t ctimer_config;
ctimer_callback_t ctimer_callback = ctimer_match0_callback;

/*******************************************************************************
 * Code
 ******************************************************************************/

void ctimer_match0_callback(uint32_t flags)
{
	time_in_ms++;
}

/*!
 * @brief Main function
 */
int main(void)
{

    if (NT >= ET || ET >= CT || ES >= 0.0 || NS >= 0.0)
    {
    	PRINTF("Wrong coefficient value defined\r\n");
    	exit(1);
    }

    CLOCK_AttachClk(kSFRO_to_CTIMER2);

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

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

    cs42448Config.i2cConfig.codecI2CSourceClock = CLOCK_GetFlexCommClkFreq(2);
    cs42448Config.format.mclk_HZ                = CLOCK_GetMclkClkFreq();

    PRINTF("Configure codec\r\n");

    /* protocol: i2s
     * sampleRate: 48K
     * bitwidth:16
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
                        DEMO_CODEC_VOLUME) != kStatus_Success)
    {
        assert(false);
    }

    PRINTF("Configure I2S\r\n");

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
    I2S_TxGetDefaultConfig(&s_TxConfig);
    s_TxConfig.divider     = DEMO_I2S_CLOCK_DIVIDER;
    s_TxConfig.masterSlave = DEMO_I2S_TX_MODE;

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
    I2S_RxGetDefaultConfig(&s_RxConfig);
    s_RxConfig.divider     = DEMO_I2S_CLOCK_DIVIDER;
    s_RxConfig.masterSlave = DEMO_I2S_RX_MODE;

    I2S_TxInit(DEMO_I2S_TX, &s_TxConfig);
    I2S_RxInit(DEMO_I2S_RX, &s_RxConfig);

    DMA_Init(DEMO_DMA);

    DMA_EnableChannel(DEMO_DMA, DEMO_I2S_TX_CHANNEL);
    DMA_EnableChannel(DEMO_DMA, DEMO_I2S_RX_CHANNEL);
    DMA_SetChannelPriority(DEMO_DMA, DEMO_I2S_TX_CHANNEL, kDMA_ChannelPriority3);
    DMA_SetChannelPriority(DEMO_DMA, DEMO_I2S_RX_CHANNEL, kDMA_ChannelPriority2);
    DMA_CreateHandle(&s_DmaTxHandle, DEMO_DMA, DEMO_I2S_TX_CHANNEL);
    DMA_CreateHandle(&s_DmaRxHandle, DEMO_DMA, DEMO_I2S_RX_CHANNEL);


    matchConfig0.enableCounterReset = true;
    matchConfig0.enableCounterStop  = false;
    matchConfig0.matchValue         = MS_TO_CTIMER_CLK_TICKS(MATCH_VAL_MS),
    matchConfig0.outControl         = kCTIMER_Output_Toggle;
    matchConfig0.outPinInitState    = false;
    matchConfig0.enableInterrupt    = true;

//    setup_ctimer(&ctimer_config, CTIMER, &matchConfig0, kCTIMER_Match_0);

//    unsigned long exec_time = (unsigned long)0u;
//    unsigned long max_time = (unsigned long)0u;


//    for(int i = 0 ; i < 1000 ; i++ ) {
//		previous_time = time_in_ms;
//		limiter(s_Buffer, S_BUFFER_SIZE);
//		exec_time = (time_in_ms - previous_time);
//		if (exec_time > max_time)
//		{
//			max_time = (time_in_ms - previous_time);
//		}
//	}
//    PRINTF("Max time: %u ms\r\n", max_time);

    StartDigitalLoopback();



    while (1)
    {
    	__NOP();
    }
}

static void setup_ctimer(ctimer_config_t * config, CTIMER_Type * base,
		ctimer_match_config_t * matchConfig, ctimer_match_t matchChannel)
{
//	CLOCK_AttachClk(clock_attach_id);
    CTIMER_GetDefaultConfig(config);
    CTIMER_Init(base, config);

    CTIMER_RegisterCallBack(base, &ctimer_callback, kCTIMER_SingleCallback);
    CTIMER_SetupMatch(base, matchChannel, matchConfig);
    CTIMER_StartTimer(base);
}

static void StartDigitalLoopback(void)
{
    PRINTF("Setup digital loopback\r\n");

    s_TxTransfer_1.data     = &s_Buffer_1[0];
    s_TxTransfer_1.dataSize = sizeof(s_Buffer_1);

    s_RxTransfer_1.data     = &s_Buffer_1[0];
    s_RxTransfer_1.dataSize = sizeof(s_Buffer_1);

    s_TxTransfer_2.data     = &s_Buffer_2[0];
    s_TxTransfer_2.dataSize = sizeof(s_Buffer_2);

    s_RxTransfer_2.data     = &s_Buffer_2[0];
    s_RxTransfer_2.dataSize = sizeof(s_Buffer_2);

    I2S_TxTransferCreateHandleDMA(DEMO_I2S_TX, &s_TxHandle_1, &s_DmaTxHandle, TxCallback_1, (void *)&s_TxTransfer_1);
    I2S_RxTransferCreateHandleDMA(DEMO_I2S_RX, &s_RxHandle_1, &s_DmaRxHandle, RxCallback_1, (void *)&s_RxTransfer_1);

    I2S_TxTransferCreateHandleDMA(DEMO_I2S_TX, &s_TxHandle_2, &s_DmaTxHandle, TxCallback_2, (void *)&s_TxTransfer_2);
    I2S_RxTransferCreateHandleDMA(DEMO_I2S_RX, &s_RxHandle_2, &s_DmaRxHandle, RxCallback_2, (void *)&s_RxTransfer_2);

    /* need to queue two receive buffers so when the first one is filled,
     * the other is immediatelly starts to be filled */
    I2S_RxTransferReceiveDMA(DEMO_I2S_RX, &s_RxHandle_1, s_RxTransfer_1);
    I2S_RxTransferReceiveDMA(DEMO_I2S_RX, &s_RxHandle_2, s_RxTransfer_2);

    /* need to queue two transmit buffers so when the first one
     * finishes transfer, the other immediatelly starts */
    I2S_TxTransferSendDMA(DEMO_I2S_TX, &s_TxHandle_1, s_TxTransfer_1);
    I2S_TxTransferSendDMA(DEMO_I2S_TX, &s_TxHandle_2, s_TxTransfer_2);

}

static void TxCallback_1(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData)
{
    /* Enqueue the same original buffer all over again */
    i2s_transfer_t *transfer = (i2s_transfer_t *)userData;
    I2S_TxTransferSendDMA(base, handle, *transfer);
}

static void TxCallback_2(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData)
{
    /* Enqueue the same original buffer all over again */
    i2s_transfer_t *transfer = (i2s_transfer_t *)userData;
    I2S_TxTransferSendDMA(base, handle, *transfer);
}

static inline void print_buffer_data8(uint8_t * data, size_t data_bytes)
{
	for (size_t i = 0; i < (data_bytes / sizeof(uint8_t)); i++)
	{
		PRINTF("0x%0X, ", data[i]);
		if (i%20 == 19)
		{
			PRINTF("\r\n");
		}
	}
	PRINTF("\r\n\n");
}

static inline void print_buffer_data16(uint16_t * data, size_t data_bytes)
{
	for (size_t i = 0; i < (data_bytes / sizeof(uint16_t)); i++)
	{
		PRINTF("0x%0X, ", data[i]);
		if (i%20 == 19)
		{
			PRINTF("\r\n");
		}
	}
	PRINTF("\r\n\n");
}

static inline void stereo_to_mono(uint16_t * data, size_t data_len)
{
	for (size_t i = 0; i < data_len - 1; i += 2)
	{
		data[i+1] = data[i];
	}
}

static void RxCallback_1(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData)
{
	/* Enqueue the same original buffer all over again */
    i2s_transfer_t *transfer = (i2s_transfer_t *)userData;
//		print_buffer_data8(transfer->data, transfer->dataSize);
	//    stereo_to_mono(transfer->data, transfer->dataSize);
//		print_buffer_data16((uint16_t *)transfer->data, transfer->dataSize);
	//    limiter(transfer->data, transfer->dataSize);
	//    print_buffer(transfer->data, transfer->dataSize);
	I2S_RxTransferReceiveDMA(base, handle, *transfer);
}

static void RxCallback_2(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData)
{
	/* Enqueue the same original buffer all over again */
    i2s_transfer_t *transfer = (i2s_transfer_t *)userData;
//		print_buffer_data8(transfer->data, transfer->dataSize);
	//    stereo_to_mono(transfer->data, transfer->dataSize);
//		print_buffer_data16((uint16_t *)transfer->data, transfer->dataSize);
	//    limiter(transfer->data, transfer->dataSize);
	//    print_buffer(transfer->data, transfer->dataSize);
	I2S_RxTransferReceiveDMA(base, handle, *transfer);
}
