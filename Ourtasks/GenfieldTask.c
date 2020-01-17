/******************************************************************************
* File Name          : GenfieldTask.c
* Date First Issued  : 01/06/2020
* Description        : Genfield function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"
#include "ADCTask.h"
#include "adctask.h"
#include "morse.h"
#include "SerialTaskReceive.h"
#include "GenfieldTask.h"
#include "genfield_idx_v_struct.h"
#include "GenfieldEvents.h"
#include "GenfieldStates.h"
#include "GenfieldUpdates.h"
#include "genfield_func_init.h"

/* From 'main.c' */
extern UART_HandleTypeDef huart3;

#define SWTIM1 500

osThreadId GenfieldTaskHandle;

struct GENFIELDFUNCTION genfieldfunction;

/* *************************************************************************
 * void swtim1_callback(TimerHandle_t tm);
 * @brief	: Software timer 1 timeout callback
 * *************************************************************************/
static void swtim1_callback(TimerHandle_t tm)
{
	xTaskNotify(GenfieldTaskHandle, CNCTBIT04, eSetBits);
	return;
}
/* *************************************************************************
 * void swtim2_callback(TimerHandle_t tm);
 * @brief	: Software timer 2 timeout callback
 * *************************************************************************/
static void swtim2_callback(TimerHandle_t tm)
{
	xTaskNotify(GenfieldTaskHandle, CNCTBIT05, eSetBits);
	return;
}
/* *************************************************************************
 * void swtim3_callback(TimerHandle_t tm);
 * @brief	: Software timer 3 timeout callback
 * *************************************************************************/
static void swtim3_callback(TimerHandle_t tm)
{
	xTaskNotify(GenfieldTaskHandle, CNCTBIT03, eSetBits);
	return;
}
/* *************************************************************************
 * osThreadId xGenfieldTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: GenfieldTaskHandle
 * *************************************************************************/
osThreadId xGenfieldTaskCreate(uint32_t taskpriority)
{
 	osThreadDef(GenfieldTask, StartGenfieldTask, osPriorityNormal, 0, 128);
	GenfieldTaskHandle = osThreadCreate(osThread(GenfieldTask), NULL);
	vTaskPrioritySet( GenfieldTaskHandle, taskpriority );
	return GenfieldTaskHandle;
}
/* *************************************************************************
 * void StartGenfieldTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
void StartGenfieldTask(void const * argument)
{
	/* Convenience pointer. */
	struct GENFIELDFUNCTION* pcf = &genfieldfunction;

	/* A notification copies the internal notification word to this. */
	uint32_t noteval = 0;    // Receives notification word upon an API notify
	uint32_t noteuse = 0xffffffff;

	/* Init struct with working params */
	genfield_idx_v_struct_hardcode_params(&genfieldfunction.lc);

	/* Initialize working struc for GenfieldTask. */
	extern struct ADCFUNCTION adc1;
	genfield_func_init_init(pcf, &adc1);

	/* CAN hardware filter: restrict incoming to necessary CAN msgs. */
	genfield_func_init_canfilter(pcf);
      
	/* Start command/keep-alive timer */
	BaseType_t bret = xTimerReset(pcf->swtimer1, 10);
	if (bret != pdPASS) {morse_trap(44);}

	/* Upon startup allow some sensor readings to settle. */
	pcf->state = OTOSETTLING;

if (pcf->evstat != 0) morse_trap(46); // Debugging check

  /* Infinite loop */
  for(;;)
  {
		/* Wait for notifications */
		xTaskNotifyWait(0,0xffffffff, &noteval, portMAX_DELAY);
//		noteused = 0;	// Accumulate bits in 'noteval' processed.
  /* ========= Events =============================== */
// NOTE: this could be made into a loop that shifts 'noteval' bits
// and calls from table of addresses.  This would have an advantage
// if the high rate bits are shifted out first since a test for
// no bits left could end the testing early.
		// Check notification and deal with it if set.
		noteuse = 0;
		if ((noteval & CNCTBIT00) != 0)
		{ // ADC readings ready
			GenfieldEvents_00(pcf);
			noteuse |= CNCTBIT00;
		}
		if ((noteval & CNCTBIT01) != 0)
		{ // uart RX line ready
			GenfieldEvents_01(pcf);
			noteuse |= CNCTBIT01;
		}
		if ((noteval & CNCTBIT02) != 0)
		{ // (spare)
			noteuse |= CNCTBIT02;
		}
		if ((noteval & CNCTBIT03) != 0)
		{ // 
			GenfieldEvents_03(pcf);			
			noteuse |= CNCTBIT03;
		}
		if ((noteval & CNCTBIT04) != 0)
		{ // 
			GenfieldEvents_04(pcf);
			noteuse |= CNCTBIT04;
		}
		if ((noteval & CNCTBIT05) != 0)
		{ // 
			GenfieldEvents_05(pcf);
			noteuse |= CNCTBIT05;
		}
		if ((noteval & CNCTBIT06) != 0) 
		{ // CAN: cid_cmd_i 
			GenfieldEvents_06(pcf);
			noteuse |= CNCTBIT06;
		}
		if ((noteval & CNCTBIT07) != 0) 
		{ // CAN: cid_keepalive_i received
			GenfieldEvents_07(pcf);
			noteuse |= CNCTBIT07;
		}
		if ((noteval & CNCTBIT08) != 0) 
		{ // CAN: cid_gps_sync 
			GenfieldEvents_08(pcf);
			noteuse |= CNCTBIT08;
		}
  /* ========= States =============================== */

		switch (pcf->state)
		{
		case DISCONNECTED:
			GenfieldStates_disconnected(pcf);
			break;
		case CONNECTING:
			GenfieldStates_connecting(pcf);
			break;
		case CONNECTED:
			GenfieldStates_connected(pcf);
			break;
		case FAULTING:
			GenfieldStates_faulting(pcf);
			break;
		case FAULTED:
			GenfieldStates_faulted(pcf);
			break;
		case RESETTING:
			GenfieldStates_reset(pcf);
			break;
		case DISCONNECTING:
			GenfieldStates_disconnecting(pcf);
			break;
		case OTOSETTLING:
			GenfieldStates_otosettling_init(pcf);
			break;
		}
  /* ========= Update outputs ======================= */
		GenfieldUpdates(pcf);
  }
}

