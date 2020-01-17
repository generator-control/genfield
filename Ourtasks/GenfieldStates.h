/******************************************************************************
* File Name          : GenfieldStates.h
* Date First Issued  : 01/06/2020
* Description        : States in Genfield function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#ifndef __GENFIELDSTATES
#define __GENFIELDSTATES

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#include "adc_idx_v_struct.h"

/* Substate states for CONNECTING main state */
enum connecting_state
{
/* Two genfield mode */
	CONNECT_C1,  // Genfield #1 closure delay
	CONNECT_C2,  // Minimum pre-charge duration delay
	CONNECT_C3,  // Additional pre-charge delay, voltage test
	CONNECT_C4,  // Genfield #2 closure delay
/* One genfield mode: #1 is genfield, #2 pre-chg relay */
	CONNECT_C1B, // Genfield #1 closure delay
	CONNECT_C2B, // Minimum pre-charge duration delay
	CONNECT_C3B, // Additional pre-charge delay, voltage test
	CONNECT_C4B, // Genfield #2 closure delay
};

void GenfieldStates_otosettling_init(struct GENFIELDFUNCTION* pcf);
void GenfieldStates_disconnecting(struct GENFIELDFUNCTION* pcf);
void GenfieldStates_disconnected(struct GENFIELDFUNCTION* pcf);
void GenfieldStates_connecting(struct GENFIELDFUNCTION* pcf);
void GenfieldStates_connected(struct GENFIELDFUNCTION* pcf);
void GenfieldStates_faulting(struct GENFIELDFUNCTION* pcf);
void GenfieldStates_faulted(struct GENFIELDFUNCTION* pcf);
void GenfieldStates_reset(struct GENFIELDFUNCTION* pcf);
void transition_faulting(struct GENFIELDFUNCTION* pcf, uint8_t fc);

#endif

