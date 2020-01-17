/******************************************************************************
* File Name          : GenfieldUpdates.c
* Date First Issued  : 01/06/2020
* Description        : Update outputs in Genfield function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"
#include "ADCTask.h"
#include "adctask.h"

#include "GenfieldTask.h"
#include "genfield_idx_v_struct.h"
#include "CanTask.h"
#include "genfield_msgs.h"

#include "morse.h"

/* From 'main.c' */
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* *************************************************************************
 * void GenfieldUpdates(struct GENFIELDFUNCTION* pcf);
 * @brief	: Update outputs based on bits set
 * *************************************************************************/
void GenfieldUpdates(struct GENFIELDFUNCTION* pcf)
{
	/* Reset new ADC readings flag. */
	if ((pcf->evstat & CNCTEVADC) != 0)
	{
			pcf->evstat &= ~CNCTEVADC;
	}

	/* Queue keep-alive status CAN msg */
	if ((pcf->outstat & CNCTOUT05KA) != 0)
	{
		pcf->outstat &= ~CNCTOUT05KA;	
		genfield_msg_ka(pcf);
	}

	/* Genfield #1 energization */
	if ( ((pcf->outstat & (CNCTOUT00K1 | CNCTOUT06KAw)) ^ (pcf->outstat_prev & (CNCTOUT00K1 | CNCTOUT06KAw))) != 0)
	{ // Either/both energize or pwm coil requested
		if ((pcf->outstat & CNCTOUT00K1) != 0)
		{ // Here, genfield off-->on
			if ((pcf->outstat & CNCTOUT06KAw) == 0)
			{ // Genfield ON, PWM OFF
				pcf->sConfigOCn.Pulse = (htim4.Init.Period+2); // Max+1 PWM period
				pcf->outstat_prev |= CNCTOUT00K1;
			}
			else
			{ // Genfield was on, switch to pwm
				pcf->sConfigOCn.Pulse = pcf->ipwmpct1; // PWM period for #1
				pcf->outstat_prev |= CNCTOUT06KAw;
			}
			HAL_TIM_PWM_ConfigChannel(&htim4, &pcf->sConfigOCn, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);  
		}
		else
		{ // Genfield OFF; 
			pcf->sConfigOCn.Pulse = 0; // Period = 0 is OFF
			HAL_TIM_PWM_ConfigChannel(&htim4, &pcf->sConfigOCn, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);  
			pcf->outstat_prev &= ~(CNCTOUT00K1 | CNCTOUT06KAw); // Reset requests
		}
	}

	/* Genfield #2 energization */
	if ( ((pcf->outstat & (CNCTOUT01K2 | CNCTOUT07KAw)) ^ (pcf->outstat_prev & (CNCTOUT01K2 | CNCTOUT07KAw))) != 0)
	{ // Either/both energize or pwm coil requested
		if ((pcf->outstat & CNCTOUT01K2) != 0)
		{  // Here, genfield off-->on
			if ((pcf->outstat & CNCTOUT07KAw) == 0)
			{ // Genfield ON, PWM OFF
				pcf->sConfigOCn.Pulse = (htim3.Init.Period+2); // Max+1 PWM period
				pcf->outstat_prev |= CNCTOUT01K2;
			}
			else
			{ // Genfield was on, switch to pwm
				pcf->sConfigOCn.Pulse = pcf->ipwmpct2; // PWM period for #2
				pcf->outstat_prev |= CNCTOUT07KAw;
			}
			HAL_TIM_PWM_ConfigChannel(&htim3, &pcf->sConfigOCn, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  
		}
		else
		{ // Genfield OFF; 
			pcf->sConfigOCn.Pulse = 0; // Period = 0
			HAL_TIM_PWM_ConfigChannel(&htim3, &pcf->sConfigOCn, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  
			pcf->outstat_prev &= ~(CNCTOUT01K2 | CNCTOUT07KAw);
		}
	}

	/* DMOC FET enabling. */
	if ((pcf->outstat & CNCTOUT04EN) != 0)
	{ // Turn on FET for DMOC hardware enable
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
	}
	else
	{ // Turn FET off.
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
	}

	return;
}

