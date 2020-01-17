/******************************************************************************
* File Name          : GenfieldEvents.c
* Date First Issued  : 01/06/2020
* Description        : Events in Genfield function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"
#include "ADCTask.h"
#include "adctask.h"

#include "genfield_idx_v_struct.h"
#include "GenfieldTask.h"
#include "morse.h"

#include "SerialTaskReceive.h"
#include "GenfieldTask.h"
#include "can_iface.h"
#include "genfield_hv.h"
#include "genfield_cmd_msg.h"
#include "genfield_msgs.h"
#include "genfield_hv.h"
#include "MailboxTask.h"

/* *************************************************************************
 * void GenfieldEvents_00(struct GENFIELDFUNCTION* pcf);
 * @brief	: ADC readings available
 * *************************************************************************/
void GenfieldEvents_00(struct GENFIELDFUNCTION* pcf)
{
	pcf->evstat |= CNCTEVADC; // Show new readings ready
	return;
}

/* *************************************************************************
 * void GenfieldEvents_01(struct GENFIELDFUNCTION* pcf);
 * @brief	: HV sensors usart RX line ready
 * *************************************************************************/
uint32_t dbgCE1;
void GenfieldEvents_01(struct GENFIELDFUNCTION* pcf)
{
dbgCE1 += 1;
	genfield_hv_uartline(pcf);  // Extract readings from received line
	genfield_hv_calibrate(pcf); // Calibrate raw ADC ticks to scale int volts
	
	xTimerReset(pcf->swtimer3,1); // Reset keep-alive timer
	pcf->evstat &= ~CNCTEVTIMER3;	// Clear timeout bit 
	pcf->evstat |= CNCTEVHV;      // Show new HV readings available
	pcf->hvuartctr += 1;		// Running count of lines received
	return;
}
/* *************************************************************************
 * void GenfieldEvents_02(struct GENFIELDFUNCTION* pcf);
 * @brief	: (spare)
 * *************************************************************************/
void GenfieldEvents_02(struct GENFIELDFUNCTION* pcf)
{
}
/* *************************************************************************
 * void GenfieldEvents_03(struct GENFIELDFUNCTION* pcf);
 * @brief	: TIMER3: uart RX keep alive failed
 * *************************************************************************/
void GenfieldEvents_03(struct GENFIELDFUNCTION* pcf)
{  // Readings failed to come in before timer timed out.
	pcf->evstat |= CNCTEVTIMER3;	// Set timeout bit 

	pcf->evstat &= ~CNCTEVHV;      // Show new HV readings NOT available

	/* Show uart RX timer timed out, i.e. no readings. */
	pcf->outstat |= CNCTOUTUART3;
	return;
}
/* *************************************************************************
 * void GenfieldEvents_04(struct GENFIELDFUNCTION* pcf);
 * @brief	: TIMER1: Command Keep Alive failed (loss of command control)
 * *************************************************************************/
uint32_t dbgev04;

void GenfieldEvents_04(struct GENFIELDFUNCTION* pcf)
{
dbgev04 += 1;
	pcf->evstat |= CNCTEVTIMER1;	// Set to show that TIMER1 timed out

	/* Update connect command and reset status */
// Let GenfieldStates go to faulting state
//	pcf->evstat &= ~(CNCTEVCMDCN | CNCTEVCMDRS);

	/* Send status msg as a status heartbeat. */
//	genfield_msg_ka(pcf);
	pcf->outstat |=  CNCTOUT05KA;  // Output status bit: Show keep-alive

	/* Send with CAN id for heartbeat. */
	genfield_msg1(pcf, 0); // Send battery string voltage and current
	genfield_msg2(pcf, 0); // Send DMOC+ and DMOC- voltages

	return;
}
/* *************************************************************************
 * void GenfieldEvents_05(struct GENFIELDFUNCTION* pcf);
 * @brief	: TIMER2: delay ended
 * *************************************************************************/
void GenfieldEvents_05(struct GENFIELDFUNCTION* pcf)
{
	pcf->evstat |= CNCTEVTIMER2;	// Set timeout bit 	
	return;
}
/* *************************************************************************
 * void GenfieldEvents_06(struct GENFIELDFUNCTION* pcf);
 * @brief	: CAN: cid_cmd_i (function/diagnostic command/poll)
 * *************************************************************************/
void GenfieldEvents_06(struct GENFIELDFUNCTION* pcf)
{
	genfield_cmd_msg_i(pcf); // Build and send CAN msg with data requested
	return;
}
/* *************************************************************************
 * void GenfieldEvents_07(struct GENFIELDFUNCTION* pcf);
 * @brief	: CAN: cid_keepalive_i 
 * *************************************************************************/
uint8_t dbgevcmd;

void GenfieldEvents_07(struct GENFIELDFUNCTION* pcf)
{
	BaseType_t bret = xTimerReset(pcf->swtimer1, 10);
	if (bret != pdPASS) {morse_trap(44);}

	pcf->outstat |=  CNCTOUT05KA;  // Output status bit: Show keep-alive
	pcf->evstat  &= ~CNCTEVTIMER1; // Reset timer1 keep-alive timed-out bit

	/* Incoming command byte with command bits */
	uint8_t cmd = pcf->pmbx_cid_keepalive_i->ncan.can.cd.uc[0];
dbgevcmd = cmd;

	/* Update connect request status bits */
	if ( (cmd & CMDCONNECT) != 0) // Command to connect
	{ // Here, request to connect
		pcf->evstat |= CNCTEVCMDCN;		
	}
	else
	{
		pcf->evstat &= ~CNCTEVCMDCN;		
	}
	/* Update reset status */
	if ( (cmd & CMDRESET ) != 0) // Command to reset
	{ // Here, request to reset
		pcf->evstat |= CNCTEVCMDRS;		
	}
	else
	{
		pcf->evstat &= ~CNCTEVCMDRS;		
	}
	return;
}	
/* *************************************************************************
 * void GenfieldEvents_08(struct GENFIELDFUNCTION* pcf);
 * @brief	: CAN: cid_gps_sync: send response CAN msgs
 * *************************************************************************/
uint32_t dbggpsflag;

void GenfieldEvents_08(struct GENFIELDFUNCTION* pcf)
{

/* Testing: use incoming gps msg to time defaultTask loop. */
struct CANRCVBUF* pcan = &pcf->pmbx_cid_gps_sync->ncan.can;
if (pcan->id == 0x00400000)
{
	if (pcan->cd.uc[0] == 0)
      dbggpsflag += 1;
}
	/* Send with regular polled CAN ID */
	genfield_msg1(pcf, 1); // Send battery string voltage and current
	genfield_msg2(pcf, 1); // Send DMOC+ and DMOC- voltages
	return;
}
	
