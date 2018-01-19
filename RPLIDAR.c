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

/******* TO DO ***************************

- Contrôle de la broche "motor control" du LIDAR si celle-ci n'est pas juste connectée au 3.3V

Notes sur le timing :

- Une trame de donnée fait 84 octets. A 115.2 Kbps cela représente un peu moins de 6 ms. Cette tâche doit
  donc être exécutée au moins à la fréquence de 6ms, et le traîtement d'une trame doit prendre moins de 6ms.

******************************************/

// Variables externes
extern UART_HandleTypeDef huart3;

// Variables globales
static uint8_t rplidar_tx_buffer[] = {0xA5, 0x82, 0x05, 0, 0, 0, 0, 0, 0x22};	// La trame de démarrage du mode Express Scan, fournie par la doc Slamtec
static uint8_t rplidar_rx_buffer[84*2];	// Les trames "express scan" font 84 octets, on fait du double-buffering
static uint8_t rplidar_status = 0;	// 0 : défaut, 1 : LIDAR démarré
static uint8_t rplidar_data_trigger = 0;	// Quand le DMA passe cette valeur à un, c'est qu'une trame de données est prête à être traitée
static int offset_dma = 0;	// offset pour la réception  / 0 ou 84 selon le buffer contenant la trame la plus récente
static int offset = 84;		// offset pour le traitement / 0 ou 84 selon le buffer contenant la trame la plus récente

// Variables globales temporaires (STM Studio / visualisation)
//static uint8_t rplidar_test_1;

// Structures de données pour le décodage de trames:
union header
{
	uint8_t bytes[4];
    struct
	{
		uint8_t sync1: 4;
		uint8_t cksum_lo: 4;
		uint8_t sync2: 4;
		uint8_t cksum_hi: 4;
		uint8_t angle_hi: 8;
		uint8_t S:1;
		uint8_t angle_lo: 7;
    };
};

// Structure d'une "cabin"
struct cabin
{

};

// Structure d'une trame de données complete
struct trame_express
{
		uint8_t sync1: 4;
		uint8_t cksum_lo: 4;
		uint8_t sync2: 4;
		uint8_t cksum_hi: 4;
		uint8_t angle_hi: 8;
		uint8_t S:1;
		uint8_t angle_lo: 7;
		struct cabin cab[5];
};

// Tache du RPLIDAR
// Son but est de lancer le mode "express scan" puis de traiter chaque trame complète dès qu'elle est reçue
void StartRPLIDARTask(void const * argument)
{
	// Initialisation
	HAL_UART_Transmit_DMA(&huart3, rplidar_tx_buffer, 9);	// Envoi de la trame de démarrage du LIDAR
	HAL_UART_Receive_DMA(&huart3, rplidar_tx_buffer, 7);	// Lecture du descripteur de réponse renvoyé par le LIDAR
	// Note : on utilise bien le buffer d'émission pour la réception dans ce cas, ce n'est pas une typo
	// La transmission et la réception vont prendre 16 octets à 115.2 Kbps = 1.1 ms

	// On attend maintenant la fin de la réception du descripteur de réponse:
	while (rplidar_status == 0)
		osDelay(2);		// Le temps de traitement du RPLIDAR n'est pas connu, à mesurer !

	// Le LIDAR va maintenant envoyer des trames de données à répétition, qu'il faut traiter:
	for(;;)
	{
		// y a-t'il une nouvelle trame dans le buffer de réception de l'UART ? Sinon, on attend :
		while (rplidar_data_trigger == 0)
			osDelay(1);

		// remise à zéro du trigger:
		rplidar_data_trigger = 0;

		// traitement de la trame par une fonction dédiée
		decodage_express_scan (rplidar_rx_buffer + offset);

		// mise à jour de l'offset pour la prochaine trame:
		offset = 84 - offset;	// bascule entre les deux buffers

	}
}

// Callback UART3 DMA : cette fonction est déclarée par la HAL comme "weak", on la redéfinit ici
// Attention ! Elle est utilisée par TOUTES les UART (possible complication dans le futur...)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Le test suivant vérifie quelle UART vient de terminer une réception en mode DMA :
	if (huart->Instance == USART3)	// RPLIDAR
	{
		// Un transfert DMA complet (réception de trame RPLIDAR) a eu lieu : traitons-le
		// On attend deux trames possibles : descripteur de réponse, ou réponse. Notre variable état nous dit laquelle:
		if (rplidar_status == 0)
		{
			// Le RPLIDAR n'est pas encore actif, on a donc affaire à un descripteur de réponse.
			if (rplidar_tx_buffer[6] == 82) 	// On vérifie...
			{
				rplidar_status = 1;				// ... et on transitionne à l'état actif si c'est bien le cas

				// lancement du premier transfert DMA pour les données:
				HAL_UART_Receive_DMA(&huart3, rplidar_rx_buffer + offset_dma, 84);
				offset_dma = 84 - offset_dma;	// bascule entre les deux buffers
				return;
			}
			else
			{
				// Là, on a un pépin... on a reçu une trame mais pas la bonne... on gèrera ça dans une version ultérieure
				return;		// pour le moment, on se barre...
			}
		}

		// Si on arrive ici, c'est que le LIDAR est actif et qu'on vient de recevoir une trame de données

		if (rplidar_data_trigger == 1)
		{
			// problème : la précédente trame reçue n'a pas encore été traitée !
			return;		// on gèrera ce cas dans une version ultérieure
		}

		rplidar_data_trigger = 1;	// Indiquer à la tâche qu'elle a du travail
		// Il y a certainement un meilleur mécanisme (wait sur signal) mais on verra ça plus tard

		// Mise à jour de l'offset d'indice:
		offset_dma = 84 - offset_dma;	// bascule entre les deux buffers

		// On lance le transfer DMA suivant:
		HAL_UART_Receive_DMA(&huart3, rplidar_rx_buffer + offset_dma, 84);
	}
}

void decodage_express_scan (uint8_t* t)
{
	// t[] est une table de 84 octets représentant une trame de données du LIDAR (express scan)
	// celle-ci est composée d'un entête de quatre octets suivie de cinq blocs de données de même structure, contenant
	// deux échantillons de distance chacun.

	struct trame_express *f = t;	// On va mapper f sur une structure pour faciliter le décodage

	// Vérifier la synchro:
	if ((f->sync1 == 0xA) && (f->sync2 == 0x5))
	{
		// Bonne synchro ! Récupérons le checksum
		uint8_t cksum = (f->cksum_hi << 4) | f->cksum_lo;
		// Verification du checksum par rapport au reste de la trame:
		int k;
		uint8_t sum = 0;
		for (k=0; k<84; k++)
			sum ^= t[k];	// XOR de tous les octets de la trame

		if (sum == cksum)
		{
			// checksum valide, on continue avec l'angle de départ

		}

	}
}
