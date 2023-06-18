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
#include "drc_algorithms_cm33_conf.h"

#include "fsl_powerquad.h"
#include "fsl_power.h"

#include <time.h>
#include <stdlib.h>
#include <math.h>
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
//#define DEMO_CODEC_I2C_INSTANCE         2U
#define DEMO_CODEC_VOLUME               100U
#define S_BUFFER_SIZE					400
#define DMA_DESCRIPTOR_NUM      		2U
#define I2S_TRANSFER_NUM      			2U

#define CTIMER          				CTIMER2         /* Timer 2 */
#define CTIMER_MAT0_OUT 				kCTIMER_Match_0 /* Match output 0 */
#define CTIMER_CLK_FREQ 				CLOCK_GetCtimerClkFreq(2)

#define MATCH_VAL_MS					0.1
#define MS_TO_CTIMER_CLK_TICKS(ms)		(uint32_t)(CTIMER_CLK_FREQ * ms / 1000)
#define TICKS_TO_MS(t)					((t) * (MATCH_VAL_MS))
#define TICKS_TO_US(t)					((t) * (MATCH_VAL_MS) * 1000.0)

#define DEMO_POWERQUAD 					POWERQUAD

#define TEST_ARR_SIZE						10000
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void start_digital_loopback(void);

static void TxCallback(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData);

static void RxCallback(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData);

static void TxCallback_2(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData);

static void RxCallback_2(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData);

void ctimer_match0_callback(uint32_t flags);

static void setup_ctimer(ctimer_config_t * config, CTIMER_Type * base, ctimer_match_config_t * matchConfig,
		ctimer_match_t matchChannel);

/* auxiliary functions */
static void print_buffer_data_8(uint8_t * data, size_t data_bytes);
static void print_buffer_data_16(uint16_t * data, size_t data_bytes);
static void measure_algorithm_time_8(void (*algorithm_func)(uint8_t *, size_t), uint8_t * buffer, size_t buffer_size, uint32_t iterations);
static void measure_algorithm_time_16(void (*algorithm_func)(uint16_t *, size_t), uint16_t * buffer, size_t buffer_size, uint32_t iterations);
static void init_arr_with_rand_8(uint8_t * arr, size_t arr_size);
static void init_arr_with_rand_16(uint16_t * arr, size_t arr_size);
static void test_pq_math(uint32_t iterations);
static void init_arr_with_rand_float(float * arr, size_t arr_size);

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

DMA_ALLOCATE_LINK_DESCRIPTORS(s_txDmaDescriptors, DMA_DESCRIPTOR_NUM);
DMA_ALLOCATE_LINK_DESCRIPTORS(s_rxDmaDescriptors, DMA_DESCRIPTOR_NUM);
__ALIGN_BEGIN static uint8_t s_Buffer_1[S_BUFFER_SIZE] __ALIGN_END; /* 100 samples => time about 2 ms */
__ALIGN_BEGIN static uint8_t s_Buffer_2[S_BUFFER_SIZE] __ALIGN_END; /* 100 samples => time about 2 ms */
static dma_handle_t s_DmaTxHandle;
static dma_handle_t s_DmaRxHandle;
static i2s_config_t s_TxConfig;
static i2s_config_t s_RxConfig;
static i2s_dma_handle_t s_TxHandle;
static i2s_dma_handle_t s_RxHandle;
static i2s_transfer_t s_RxTransfer[I2S_TRANSFER_NUM];
static i2s_transfer_t s_TxTransfer[I2S_TRANSFER_NUM];
extern codec_config_t boardCodecConfig;
codec_handle_t codecHandle;

static volatile unsigned long ctimer_ticks = 0UL;
/* Match Configuration for Channel 0 */
static ctimer_match_config_t matchConfig0;
ctimer_config_t ctimer_config;
ctimer_callback_t ctimer_callback = ctimer_match0_callback;

static float test_arr [TEST_ARR_SIZE] = {0};
/*******************************************************************************
 * Code
 ******************************************************************************/

void ctimer_match0_callback(uint32_t flags)
{
	ctimer_ticks++;
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

    /* Power up PQ RAM. */
    SYSCTL0->PDRUNCFG1_CLR = SYSCTL0_PDRUNCFG1_PQ_SRAM_APD_MASK | SYSCTL0_PDRUNCFG1_PQ_SRAM_PPD_MASK;
    /* Apply power setting. */
    POWER_ApplyPD();
    PQ_Init(DEMO_POWERQUAD);

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

    PRINTF("\nConfigure codec\r\n");

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

    setup_ctimer(&ctimer_config, CTIMER, &matchConfig0, CTIMER_MAT0_OUT);

    calculate_coefficients();

    /* measure algorithms execution time */
//    PRINTF("limiter_16:\r\n");
//    measure_algorithm_time_16(limiter_16, (uint16_t *)s_Buffer_1, (S_BUFFER_SIZE / 2), 1000);
//    PRINTF("limiter_8:\r\n");
//    measure_algorithm_time_8(limiter_8, (uint8_t *)s_Buffer_1, (S_BUFFER_SIZE), 1000);
//    PRINTF("compressor_expander_ngate_16:\r\n");
//    measure_algorithm_time_16(compressor_expander_ngate_16, (uint16_t *)s_Buffer_1, (S_BUFFER_SIZE / 2), 1000);
//    PRINTF("compressor_expander_ngate_8:\r\n");
//    measure_algorithm_time_8(compressor_expander_ngate_8, (uint8_t *)s_Buffer_1, (S_BUFFER_SIZE), 1000);

//    start_digital_loopback();

    /* check if algorithms work correctly */
//    init_arr_with_rand_16((uint16_t *)s_Buffer_1, (S_BUFFER_SIZE / 2));
//    print_buffer_data_16((uint16_t *)s_Buffer_1, (S_BUFFER_SIZE / 2));
//    compressor_expander_ngate_16((uint16_t *)s_Buffer_1, (S_BUFFER_SIZE / 2));
//    print_buffer_data_16((uint16_t *)s_Buffer_1, (S_BUFFER_SIZE / 2));

	for (int var = 0; var < 5; ++var) {
		test_pq_math(TEST_ARR_SIZE);
		PRINTF("\r\n");
	}

    while (1)
    {
    	__NOP();
    }
}

static void setup_ctimer(ctimer_config_t * config, CTIMER_Type * base,
		ctimer_match_config_t * matchConfig, ctimer_match_t matchChannel)
{
//	CLOCK_AttachClk(kSFRO_to_CTIMER2);
//	PRINTF("CTIMER2 Clock Frequency: %u\r\n", CTIMER_CLK_FREQ);
//	PRINTF("CTIMER2 Match Value: %u\r\n", MS_TO_CTIMER_CLK_TICKS(MATCH_VAL_MS));
    CTIMER_GetDefaultConfig(config);
    CTIMER_Init(base, config);

    CTIMER_RegisterCallBack(base, &ctimer_callback, kCTIMER_SingleCallback);
    CTIMER_SetupMatch(base, matchChannel, matchConfig);
    CTIMER_StartTimer(base);
}

static void start_digital_loopback(void)
{
    uint32_t *srcAddr = NULL, *destAddr = NULL, srcInc = 4UL, destInc = 4UL;
    bool intA = true;
    i2s_dma_handle_t * handle;
    dma_handle_t * dma_handle;
    i2s_transfer_t *currentTransfer;

    PRINTF("Setup digital loopback\r\n");

    s_RxTransfer[0].data     = &s_Buffer_1[0];
    s_RxTransfer[0].dataSize = sizeof(s_Buffer_1);

    s_RxTransfer[1].data     = &s_Buffer_2[0];
    s_RxTransfer[1].dataSize = sizeof(s_Buffer_2);

    s_TxTransfer[0].data     = &s_Buffer_1[0];
    s_TxTransfer[0].dataSize = sizeof(s_Buffer_1);

    s_TxTransfer[1].data     = &s_Buffer_2[0];
    s_TxTransfer[1].dataSize = sizeof(s_Buffer_2);

    I2S_RxTransferCreateHandleDMA(DEMO_I2S_RX, &s_RxHandle, &s_DmaRxHandle, RxCallback, (void *)&s_RxTransfer[0]);
    I2S_TxTransferCreateHandleDMA(DEMO_I2S_TX, &s_TxHandle, &s_DmaTxHandle, TxCallback, (void *)&s_TxTransfer[0]);

    I2S_TransferInstallLoopDMADescriptorMemory(&s_RxHandle, s_rxDmaDescriptors, DMA_DESCRIPTOR_NUM);
    I2S_TransferInstallLoopDMADescriptorMemory(&s_TxHandle, s_txDmaDescriptors, DMA_DESCRIPTOR_NUM);


    if (I2S_TransferSendLoopDMA(DEMO_I2S_TX, &s_TxHandle, &s_TxTransfer[0], 2U) != kStatus_Success)
    {
        assert(false);
    }

    if (I2S_TransferReceiveLoopDMA(DEMO_I2S_RX, &s_RxHandle, &s_RxTransfer[0], 2U) != kStatus_Success)
    {
        assert(false);
    }
}

static void TxCallback(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData)
{
	__NOP();
}

static void print_buffer_data_8(uint8_t * data, size_t data_size)
{
	for (size_t i = 0; i < data_size; i++)
	{
		PRINTF("0x%0X, ", data[i]);
		if (i%20 == 19)
		{
			PRINTF("\r\n");
		}
	}
	PRINTF("\r\n\n");
}

static void print_buffer_data_16(uint16_t * data, size_t data_size)
{
	for (size_t i = 0; i < data_size; i++)
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

static void RxCallback(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData)
{
	static unsigned long last_time =  0UL;
	static int counter = 0;
	static bool isIntA = true;
	static unsigned long ticks_sum = 0UL;

	if (last_time == 0UL)
	{
		last_time = ctimer_ticks;
	}

	if (isIntA)
	{
		isIntA = false;
		counter++;
//		last_time = ctimer_ticks;
	}
	else
	{
		isIntA = true;
//		ticks_sum += (ctimer_ticks - last_time);
	}

    i2s_transfer_t *transfer = (i2s_transfer_t *)userData;
    compressor_expander_ngate_16((uint16_t *)transfer->data, transfer->dataSize);
    if (counter == 1000 && !isIntA)
    {
    	PRINTF("Time between interrupts for transfer 1: %.3f\r\n", TICKS_TO_MS(ctimer_ticks - last_time)/1000.0);
    }
}

static void measure_algorithm_time_16(void (*algorithm_func)(uint16_t *, size_t), uint16_t * buffer, size_t buffer_size, uint32_t iterations)
{
	volatile unsigned long previous_time = 0UL;
    unsigned long exec_time_ticks = 0UL;
    unsigned long exec_time_ticks_sum = 0UL;
    unsigned long max_time_ticks = 0UL;

    // write random values to buffer
    init_arr_with_rand_16(buffer, buffer_size);

    // execute algorithm 'iterations' times
    for (int i = 0; i < iterations; i++)
    {
		previous_time = ctimer_ticks;
		algorithm_func(buffer, buffer_size);
		exec_time_ticks = (ctimer_ticks - previous_time);
		exec_time_ticks_sum += exec_time_ticks;
		if (exec_time_ticks > max_time_ticks)
		{
			max_time_ticks = exec_time_ticks;
		}
	}

    // print calculated values
    PRINTF("Max time: %.3f ms\r\n", TICKS_TO_MS(max_time_ticks));
    PRINTF("Average time: %.3f ms\r\n", (TICKS_TO_MS(exec_time_ticks_sum) / (float)iterations));
}

static void measure_algorithm_time_8(void (*algorithm_func)(uint8_t *, size_t), uint8_t * buffer, size_t buffer_size, uint32_t iterations)
{
	volatile unsigned long previous_time = 0UL;
    unsigned long exec_time_ticks = 0UL;
    unsigned long exec_time_ticks_sum = 0UL;
    unsigned long max_time_ticks = 0UL;

    // write random values to buffer
    init_arr_with_rand_8(buffer, buffer_size);

    // execute algorithm 'iterations' times
    for (int i = 0; i < iterations; i++)
    {
		previous_time = ctimer_ticks;
		algorithm_func(buffer, buffer_size);
		exec_time_ticks = (ctimer_ticks - previous_time);
		exec_time_ticks_sum += exec_time_ticks;
		if (exec_time_ticks > max_time_ticks)
		{
			max_time_ticks = exec_time_ticks;
		}
	}

    // print calculated values
    PRINTF("Max time: %.3f ms\r\n", TICKS_TO_MS(max_time_ticks));
    PRINTF("Average time: %.3f ms\r\n", (TICKS_TO_MS(exec_time_ticks_sum) / (float)iterations));
}

static void init_arr_with_rand_8(uint8_t * arr, size_t arr_size)
{
    srand(time(NULL));
    for (int i = 0 ; i < arr_size ; i++)
    {
    	arr[i] = (uint8_t)(rand() % UINT8_MAX);
    }
}

static void init_arr_with_rand_16(uint16_t * arr, size_t arr_size)
{
	srand(time(NULL));
    for (int i = 0 ; i < arr_size ; i++)
    {
    	arr[i] = (uint16_t)(rand() % UINT16_MAX);
    }
}

static void init_arr_with_rand_float(float * arr, size_t arr_size)
{
	srand(time(NULL));
    for (int i = 0 ; i < arr_size ; i++)
    {
    	arr[i] = (float)(rand() % UINT8_MAX);
    }
}

static void test_pq_math(uint32_t iterations)
{
	volatile unsigned long previous_time = 0UL;
    unsigned long exec_time_ticks = 0UL;
    unsigned long exec_time_ticks_sum = 0UL;

	float x1 = 3.0f;
	float x2 = 9.0f;
	volatile float result;

	init_arr_with_rand_float(test_arr, iterations);

	previous_time = ctimer_ticks;
	for (int i = 0; i < iterations; i++)
	{
		PQ_DivF32(&x1, &test_arr[i], &result);
	}
	exec_time_ticks_sum = (ctimer_ticks - previous_time);

	PRINTF("[PQ] Average time x1/x2: %.3f us\r\n", (TICKS_TO_US(exec_time_ticks_sum) / (float)iterations));

	previous_time = ctimer_ticks;
	for (int i = 0; i < iterations; i++)
	{
		result = x1 / test_arr[i];
	}
	exec_time_ticks_sum = (ctimer_ticks - previous_time);
	PRINTF("[CM-33] Average time x1/x2: %.3f us\r\n", (TICKS_TO_US(exec_time_ticks_sum) / (float)iterations));


	previous_time = ctimer_ticks;
	for (int i = 0; i < iterations; i++)
	{
		PQ_LnF32(&test_arr[i], &result);
	}
	exec_time_ticks_sum = (ctimer_ticks - previous_time);
	PRINTF("[PQ] Average time ln: %.3f us\r\n", (TICKS_TO_US(exec_time_ticks_sum) / (float)iterations));

	previous_time = ctimer_ticks;
	for (int i = 0; i < iterations; i++)
	{
		result = logf(test_arr[i]);
	}
	exec_time_ticks_sum = (ctimer_ticks - previous_time);
	PRINTF("[CM-33] Average time ln: %.3f us\r\n", (TICKS_TO_US(exec_time_ticks_sum) / (float)iterations));
}
