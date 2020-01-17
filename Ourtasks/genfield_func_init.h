/******************************************************************************
* File Name          : genfield_func_init.h
* Date First Issued  : 01/06/2020
* Description        : hv uart conversion and recalibration
*******************************************************************************/

#ifndef __GENFIELDFUNCINIT
#define __GENFIELDFUNCINIT

#include "iir_filter_lx.h"
#include "adcparams.h"

/* *************************************************************************/
void genfield_func_init_init(struct GENFIELDFUNCTION* p, struct ADCFUNCTION* padc);
/*	@brief	: Initialize working struct for ContactorTask
 * @param	: p    = pointer to ContactorTask
 * @param	: padc = pointer to ADC working struct
 * *************************************************************************/
void genfield_func_init_canfilter(struct GENFIELDFUNCTION* p);
/*	@brief	: Setup CAN hardware filter with CAN addresses to receive
 * @param	: p    = pointer to ContactorTask
 * *************************************************************************/

#endif

