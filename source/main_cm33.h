/*
 * main_cm33.h
 *
 *  Created on: 30 lip 2023
 *      Author: Łukasz
 */

#ifndef MAIN_CM33_H_
#define MAIN_CM33_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_device_registers.h"
#include "fsl_mu.h"
#include "fsl_sema42.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
//#define Q31

#define APP_MU            		MUA
#define APP_SEMA42          	SEMA42
#define SEMA42_GATE 			0U
/* Flag indicates Core Boot Up*/
#define SEMA42_LOCK_FLAG 		0x02U
#define SEMA42_UNLOCK_FLAG		0x03U
#define SEMA42_DSP_LOCK_FLAG	0x04U
/* Channel transmit and receive register */
#define CHN_MU_REG_NUM 			0U
#define PROC_NUM				1U

#endif /* MAIN_CM33_H_ */
