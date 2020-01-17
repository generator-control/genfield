/******************************************************************************
* File Name          : GenfieldEvents.h
* Date First Issued  : 01/06/2020
* Description        : Events in Genfield function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#ifndef __GENFIELDEVENTS
#define __GENFIELDEVENTS

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#include "adc_idx_v_struct.h"

void GenfieldEvents_00(struct GENFIELDFUNCTION* pcf);
void GenfieldEvents_01(struct GENFIELDFUNCTION* pcf);
void GenfieldEvents_02(struct GENFIELDFUNCTION* pcf);
void GenfieldEvents_03(struct GENFIELDFUNCTION* pcf);
void GenfieldEvents_04(struct GENFIELDFUNCTION* pcf);
void GenfieldEvents_05(struct GENFIELDFUNCTION* pcf);
void GenfieldEvents_06(struct GENFIELDFUNCTION* pcf);
void GenfieldEvents_07(struct GENFIELDFUNCTION* pcf);
void GenfieldEvents_08(struct GENFIELDFUNCTION* pcf);

#endif

