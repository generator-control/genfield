/******************************************************************************
* File Name          : GenfieldUpdates.h
* Date First Issued  : 01/06/2020
* Description        : Update outputs in Genfield function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#ifndef __GENFIELDUPDATES
#define __GENFIELDUPDATES

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#include "adc_idx_v_struct.h"
#include "GenfieldTask.h"


/* *************************************************************************/
void GenfieldUpdates(struct GENFIELDFUNCTION* pcf);
/* @brief	: Update outputs based on bits set
 * *************************************************************************/
#endif

