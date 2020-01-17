/******************************************************************************
* File Name          : genfield_cmd_msg.c
* Date First Issued  : 01/06/2020
* Description        : cid_cmd_msg_i: Function command
*******************************************************************************/

#ifndef __GENFIELDCMDMSG
#define __GENFIELDCMDMSG

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "adc_idx_v_struct.h"
#include "adcparams.h"
#include "GenfieldTask.h"
#include "CanTask.h"


/* *************************************************************************/
void contactor_cmd_msg_i(struct GENFIELDFUNCTION* pcf);
/*	@brief	: Given the Mailbox pointer (within GENFIELDFUNCTION) handle request
 * *************************************************************************/

#endif
