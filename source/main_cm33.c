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
#include "fsl_codec_adapter.h"
#include "fsl_cs42448.h"
#include "fsl_ctimer.h"
#include "fsl_powerquad.h"
#include "fsl_power.h"
#include "fsl_mu.h"

#include "drc_algorithms_cm33.h"
#include "algorithm_testbench.h"

#include <stdbool.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_I2S_MASTER_CLOCK_FREQUENCY CLOCK_GetMclkClkFreq()
#define DEMO_AUDIO_SAMPLE_RATE          (48000U)
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
#define BUFFER_SIZE					400
#define DMA_DESCRIPTOR_NUM      		2U
#define I2S_TRANSFER_NUM      			2U

#define CTIMER          				CTIMER2         /* Timer 2 */
#define CTIMER_MAT0_OUT 				kCTIMER_Match_0 /* Match output 0 */
#define CTIMER_CLK_FREQ 				CLOCK_GetCtimerClkFreq(2)
#define MS_TO_CTIMER_CLK_TICKS(ms)		(uint32_t)(CTIMER_CLK_FREQ * ms / 1000)

#define DEMO_POWERQUAD 					POWERQUAD

#define TEST_ARR_SIZE		1000
#define ITER_COUNT			TEST_ARR_SIZE

#define APP_MU            	MUA
#define APP_MU_IRQHandler 	MU_A_IRQHandler
/* Flag indicates Core Boot Up*/
#define BOOT_FLAG 			0x01U
/* Channel transmit and receive register */
#define CHN_MU_REG_NUM 		0U
/* How many message is used to test message sending */
#define MSG_LENGTH 			1U

/*******************************************************************************
 * Type definitions
 ******************************************************************************/
typedef struct {
	volatile bool is_hifi4_processing;
	volatile unsigned long start_time;
    unsigned long exec_time_sum;
} hifi4_ctrl_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void TxCallback(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData);
static void RxCallback(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData);

void ctimer_match0_callback(uint32_t flags);

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

hifi4_ctrl_t hifi4_ctrl;
static uint32_t msgSend[MSG_LENGTH];
static uint32_t msgRecv[MSG_LENGTH];
volatile uint32_t curSend = 0;
volatile uint32_t curRecv = 0;

DMA_ALLOCATE_LINK_DESCRIPTORS(txDmaDescriptors, DMA_DESCRIPTOR_NUM);
DMA_ALLOCATE_LINK_DESCRIPTORS(rxDmaDescriptors, DMA_DESCRIPTOR_NUM);
__ALIGN_BEGIN static uint8_t src_buffer_1[BUFFER_SIZE] __ALIGN_END; /* 100 samples => time about 2 ms */
__ALIGN_BEGIN static uint8_t src_buffer_2[BUFFER_SIZE] __ALIGN_END;
__ALIGN_BEGIN static uint8_t dst_buffer_1[BUFFER_SIZE] __ALIGN_END;
__ALIGN_BEGIN static uint8_t dst_buffer_2[BUFFER_SIZE] __ALIGN_END;
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

volatile unsigned long ctimer_ticks = 0UL;
/* Match Configuration for Channel 0 */
static ctimer_match_config_t matchConfig0;
ctimer_config_t ctimer_config;
ctimer_callback_t ctimer_callback = ctimer_match0_callback;

static float test_arr [TEST_ARR_SIZE] = {0.0f};
static volatile uint16_t src_test_arr_16[TEST_ARR_SIZE] = {0};
static volatile uint16_t dst_test_arr_16[TEST_ARR_SIZE] = {0};

/*******************************************************************************
 * Code
 ******************************************************************************/
static void init_hifi4_operation(void)
{
    /* MUA init */
    MU_Init(APP_MU);

    /* Copy DSP image to RAM and start DSP core. */
    BOARD_DSP_Init();

    /* Wait DSP core is Boot Up */
    while (BOOT_FLAG != MU_GetFlags(APP_MU))
    {
    }

    /* Enable transmit and receive interrupt */
    MU_EnableInterrupts(APP_MU, (kMU_Tx0EmptyInterruptEnable | kMU_Rx0FullInterruptEnable));

    hifi4_ctrl.is_hifi4_processing = true;
    while (hifi4_ctrl.is_hifi4_processing)
    {
    }

    PRINTF("\r\n[HiFi4] Average time: %.3f ms\r\n", (TICKS_TO_MS(hifi4_ctrl.exec_time_sum) / (float)ITER_COUNT));
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

    rxTransfer[0].data     = &src_buffer_1[0];
    rxTransfer[0].dataSize = sizeof(src_buffer_1);

    rxTransfer[1].data     = &src_buffer_2[0];
    rxTransfer[1].dataSize = sizeof(src_buffer_2);

    txTransfer[0].data     = &dst_buffer_1[0];
    txTransfer[0].dataSize = sizeof(dst_buffer_1);

    txTransfer[1].data     = &dst_buffer_2[0];
    txTransfer[1].dataSize = sizeof(dst_buffer_2);

    I2S_RxTransferCreateHandleDMA(DEMO_I2S_RX, &rxHandle, &dmaRxHandle, RxCallback, (void *)&rxTransfer[0]);
    I2S_TxTransferCreateHandleDMA(DEMO_I2S_TX, &txHandle, &dmaTxHandle, TxCallback, (void *)&txTransfer[0]);

    I2S_TransferInstallLoopDMADescriptorMemory(&rxHandle, rxDmaDescriptors, DMA_DESCRIPTOR_NUM);
    I2S_TransferInstallLoopDMADescriptorMemory(&txHandle, txDmaDescriptors, DMA_DESCRIPTOR_NUM);


    if (I2S_TransferSendLoopDMA(DEMO_I2S_TX, &txHandle, &txTransfer[0], 2U) != kStatus_Success)
    {
        assert(false);
    }

    if (I2S_TransferReceiveLoopDMA(DEMO_I2S_RX, &rxHandle, &rxTransfer[0], 2U) != kStatus_Success)
    {
        assert(false);
    }
}

static void TxCallback(I2S_Type *base, i2s_dma_handle_t *handle, status_t completionStatus, void *userData)
{
	__NOP();
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
	}
	else
	{
		isIntA = true;
	}

    i2s_transfer_t *transfer = (i2s_transfer_t *)userData;
//    compressor_expander_ngate_u16((uint16_t *)transfer->data, transfer->dataSize);
    if (counter == 1000 && !isIntA)
    {
    	PRINTF("Time between interrupts for transfer 1: %.3f\r\n", TICKS_TO_MS(ctimer_ticks - last_time)/1000.0);
    }
}

static void configure_codec(void)
{
    cs42448Config.i2cConfig.codecI2CSourceClock = CLOCK_GetFlexCommClkFreq(2);
    cs42448Config.format.mclk_HZ                = CLOCK_GetMclkClkFreq();

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
    txConfig.divider     = DEMO_I2S_CLOCK_DIVIDER;
    txConfig.masterSlave = DEMO_I2S_TX_MODE;

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
    rxConfig.divider     = DEMO_I2S_CLOCK_DIVIDER;
    rxConfig.masterSlave = DEMO_I2S_RX_MODE;

    I2S_TxInit(DEMO_I2S_TX, &txConfig);
    I2S_RxInit(DEMO_I2S_RX, &rxConfig);
}

static void configure_dma(void)
{
	DMA_Init(DEMO_DMA);

	DMA_EnableChannel(DEMO_DMA, DEMO_I2S_TX_CHANNEL);
	DMA_EnableChannel(DEMO_DMA, DEMO_I2S_RX_CHANNEL);
	DMA_SetChannelPriority(DEMO_DMA, DEMO_I2S_TX_CHANNEL, kDMA_ChannelPriority3);
	DMA_SetChannelPriority(DEMO_DMA, DEMO_I2S_RX_CHANNEL, kDMA_ChannelPriority2);
	DMA_CreateHandle(&dmaTxHandle, DEMO_DMA, DEMO_I2S_TX_CHANNEL);
	DMA_CreateHandle(&dmaRxHandle, DEMO_DMA, DEMO_I2S_RX_CHANNEL);
}

static void configure_ctimer(void)
{
	matchConfig0.enableCounterReset = true;
	matchConfig0.enableCounterStop  = false;
	matchConfig0.matchValue         = MS_TO_CTIMER_CLK_TICKS(MATCH_VAL_MS);
	matchConfig0.outControl         = kCTIMER_Output_Toggle;
	matchConfig0.outPinInitState    = false;
	matchConfig0.enableInterrupt    = true;

	setup_ctimer(&ctimer_config, CTIMER, &matchConfig0, CTIMER_MAT0_OUT);
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

void ctimer_match0_callback(uint32_t flags)
{
	ctimer_ticks++;
}

void APP_MU_IRQHandler(void)
{
    uint32_t flag = 0;

    flag = MU_GetStatusFlags(APP_MU);
    if ((flag & kMU_Tx0EmptyFlag) == kMU_Tx0EmptyFlag)
    {
        if (curSend < MSG_LENGTH)
        {
            MU_SendMsgNonBlocking(APP_MU, CHN_MU_REG_NUM, msgSend[curSend++]);
            hifi4_ctrl.start_time = ctimer_ticks;
        }
        else
        {
            MU_DisableInterrupts(APP_MU, kMU_Tx0EmptyInterruptEnable);
        }
    }
    if ((flag & kMU_Rx0FullFlag) == kMU_Rx0FullFlag)
    {
        if (curRecv < MSG_LENGTH)
        {
            msgRecv[curRecv++] = MU_ReceiveMsgNonBlocking(APP_MU, CHN_MU_REG_NUM);
            hifi4_ctrl.exec_time_sum = ctimer_ticks - hifi4_ctrl.start_time;
            hifi4_ctrl.is_hifi4_processing = false;
        }
        else
        {
            MU_DisableInterrupts(APP_MU, kMU_Rx0FullInterruptEnable);
        }
    }
    SDK_ISR_EXIT_BARRIER;
}

/*!
 * @brief Main function
 */
int main(void)
{

	check_coeff_validity();

    /* Power up PQ RAM. */
    SYSCTL0->PDRUNCFG1_CLR = SYSCTL0_PDRUNCFG1_PQ_SRAM_APD_MASK | SYSCTL0_PDRUNCFG1_PQ_SRAM_PPD_MASK;

    /* Apply power setting. */
    POWER_ApplyPD();
    PQ_Init(DEMO_POWERQUAD);

    CLOCK_AttachClk(kSFRO_to_CTIMER2);

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    /* Clear MUA reset */
    RESET_PeripheralReset(kMU_RST_SHIFT_RSTn);
    EnableIRQ(MU_A_IRQn);

    PRINTF("\nConfigure clocks\r\n");
    configure_clocks();

    PRINTF("Configure codec\r\n");
    configure_codec();

    PRINTF("Configure I2S\r\n");
    configure_i2s();

    PRINTF("Configure DMA\r\n");
    configure_dma();

    PRINTF("Configure CTIMER\r\n");
    configure_ctimer();

    calculate_coefficients();

    /* measure algorithms execution time */
    PRINTF("\r\nlimiter_u16:\r\n");
    measure_algorithm_time_u16(limiter_u16, (uint16_t *)src_buffer_1, (uint16_t *)dst_buffer_1, (BUFFER_SIZE / 2), ITER_COUNT);
    PRINTF("\r\ncompressor_expander_ngate_u16:\r\n");
    measure_algorithm_time_u16(compressor_expander_ngate_u16, (uint16_t *)src_buffer_1, (uint16_t *)dst_buffer_1, (BUFFER_SIZE / 2), ITER_COUNT);

//    start_digital_loopback();

    /* check if algorithms work correctly */
//    init_arr_with_rand_16((uint16_t *)src_buffer_1, (BUFFER_SIZE / 2));
//    print_buffer_data_u16((uint16_t *)src_buffer_1, (BUFFER_SIZE / 2));
//    compressor_expander_ngate_u16((uint16_t *)src_buffer_1, (uint16_t *)dst_buffer_1, (BUFFER_SIZE / 2));
//    print_buffer_data_u16((uint16_t *)dst_buffer_1, (BUFFER_SIZE / 2));

//	for (int var = 0; var < 5; ++var)
//	{
//		test_pq_math(TEST_ARR_SIZE);
//		PRINTF("\r\n");
//	}

//    init_hifi4_operation();

//    test_algorithm(fir_filter_u16, (uint16_t *)src_test_arr_16, (uint16_t *)dst_test_arr_16, TEST_ARR_SIZE, DEMO_AUDIO_SAMPLE_RATE);
    PRINTF("\r\nfir_filter_u16:\r\n");
    measure_algorithm_time_u16(fir_filter_u16, (uint16_t *)src_buffer_1, (uint16_t *)dst_buffer_1, (BUFFER_SIZE / 2), ITER_COUNT);

    while (1)
    {
    	__NOP();
    }
}
