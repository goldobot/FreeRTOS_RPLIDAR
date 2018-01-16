/*
 * RPLIDAR.c
 *
 *  Created on: Jan 16, 2018
 *      Author: Nefastor
 */

#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include "RPLIDAR.h"

// Tache du RPLIDAR
void StartLIDARTask(void const * argument)
{
	// Initialisation

	// Boucle infinie
	for(;;)
	{
		osDelay(1);
	}
}
