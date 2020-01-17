/******************************************************************************
* File Name          : genfield_hv.c
* Date First Issued  : 01/06/2020
* Description        : hv uart conversion and recalibration
*******************************************************************************/

#include "genfield_hv.h"
#include "SerialTaskReceive.h"
#include "hexbin.h"

#include <string.h>
#include "morse.h"


/* *************************************************************************
 * void genfield_hv_uartline(struct GENFIELDFUNCTION* pcf);
 * @brief	: Get & convert ascii line to binary readings
 * @return	: readings stored in genfield function struct
 * *************************************************************************/
/*
Expect 13 hex chars: AAaaBBbbCCcc<CR> and,
place binary values in uint16_t array.
*/
/* From GenfieldTask.h for convenience 
struct CNCNTHV
{
	struct CNTCTCALSI ical; // Scale integer calibration values
   float fhv;     // Calibrated: Float
	uint32_t ihv;  // Calibrated: Scaled
	uint16_t hv;   // Raw reading as received from uart
};
*/
void genfield_hv_uartline(struct GENFIELDFUNCTION* pcf)
{
	int i;
	uint8_t* pline;	// Pointer to line buffer
	do
	{
		/* Get pointer of next completed line. */
		pline = (uint8_t*)xSerialTaskReceiveGetline(pcf->prbcb3);
		if (pline != NULL)
		{ // Here, a line is ready.
			if (*(pline+12) != '\n') return; // Not correct line

			for (i = 0; i < NUMHV; i++)
			{ 
				/* Table lookup ASCII to binary: 4 asci -> uint16_t */
				pcf->hv[i].hv  = \
                (hxbn[*(pline+0)] <<  4) | \
                (hxbn[*(pline+1)] <<  0) | \
				    (hxbn[*(pline+2)] << 12) | \
                (hxbn[*(pline+3)] <<  8);
				pline += 4;
			}
		}
	} while (pline != NULL); // Catchup jic we got behind
	return;
}
/* *************************************************************************
 * void genfield_hv_calibrate(struct GENFIELDFUNCTION* pcf);
 * @brief	: Apply calibration to raw readings
 * @return	: readings stored in genfield function struct
 * *************************************************************************/
void genfield_hv_calibrate(struct GENFIELDFUNCTION* pcf)
{
	int i;
	for (i = 0; i < NUMHV; i++)
	{
		/* Volts = Volts per ADC ct * Raw ADC ticks (16b) from uart */
		pcf->hv[i].hvc = pcf->hv[i].hvcal * pcf->hv[i].hv;
	}
	return;
}
