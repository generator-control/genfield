/******************************************************************************
* File Name          : genfield_hv.h
* Date First Issued  : 01/06/2020
* Description        : hv uart conversion and recalibration
*******************************************************************************/

#ifndef __GENFIELDHV
#define __GENFIELDHV

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "GenfieldTask.h"
#include "CanTask.h"

/* *************************************************************************/
void genfield_hv_uartline(struct GENFIELDFUNCTION* pcf);
/* @brief	: Get & convert ascii line to binary readings
 * @return	: readings stored in genfield function struct
 * *************************************************************************/
void genfield_hv_calibrate(struct GENFIELDFUNCTION* pcf);
/* @brief	: Apply calibration to raw readings
 * @return	: readings stored in genfield function struct
 * *************************************************************************/

#endif
