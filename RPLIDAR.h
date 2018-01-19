/*
 * RPLIDAR.h
 *
 *  Created on: Jan 16, 2018
 *      Author: Nefastor
 */

#ifndef RPLIDAR_H_
#define RPLIDAR_H_

void StartLIDARTask(void const * argument);	  // Tache du RPLIDAR

void decodage_express_scan (uint8_t* f);		// Fonction interne

#endif /* RPLIDAR_H_ */
