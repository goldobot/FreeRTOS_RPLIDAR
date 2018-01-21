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

- Contr�le de la broche "motor control" du LIDAR si celle-ci n'est pas juste connect�e au 3.3V

Notes sur le timing :

- Une trame de donn�e fait 84 octets. A 115.2 Kbps cela repr�sente un peu moins de 6 ms. Cette t�che doit
  donc �tre ex�cut�e au moins � la fr�quence de 6ms, et le tra�tement d'une trame doit prendre moins de 6ms.

- Connections du LIDAR � la Nucl�o F303RE:
  - PB10 : USART3_TX, va vers RPLIDAR RX (vert)
  - PB11 : USART3_RX, va vers RPLIDAR TX (jaune)
  - Alim LIDAR : fil rouge va au 5V, fil noir va � la masse
  - PWM Moteur (fil bleu) li� au 3.3V pour l'instant


******************************************/

// Variables externes
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim8;

// Variables globales
static uint8_t rplidar_reset_cmd[] = {0xA5, 0x40};
static uint8_t rplidar_tx_buffer[] = {0xA5, 0x82, 0x05, 0, 0, 0, 0, 0, 0x22};	// La trame de d�marrage du mode Express Scan, fournie par la doc Slamtec
static uint8_t rplidar_rx_buffer[84*2];	// Les trames "express scan" font 84 octets, on fait du double-buffering
static uint8_t rplidar_status = 0;	// 0 : d�faut, 1 : LIDAR d�marr�
static uint8_t rplidar_data_trigger = 0;	// Quand le DMA passe cette valeur � un, c'est qu'une trame de donn�es est pr�te � �tre trait�e
static int offset_dma = 0;	// offset pour la r�ception  / 0 ou 84 selon le buffer contenant la trame la plus r�cente
static int offset = 84;		// offset pour le traitement / 0 ou 84 selon le buffer contenant la trame la plus r�cente

// Variables globales temporaires (STM Studio / visualisation)
static volatile uint8_t rplidar_test_1 = 0;
static volatile uint8_t rplidar_test_2 = 0;
static volatile uint8_t rplidar_test_3 = 0;
static volatile int16_t rplidar_start_angle = 0;

// Structures de donn�es pour le d�codage de trames:
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
	uint8_t dat[5];
};

// Structure d'une trame de donn�es complete
struct trame_express
{
	uint8_t sync1: 4;
	uint8_t cksum_lo: 4;
	uint8_t sync2: 4;
	uint8_t cksum_hi: 4;
	uint8_t angle_hi: 8;
	uint8_t S:1;
	uint8_t angle_lo: 7;
	struct cabin cab[16];
};

// Tache du RPLIDAR
// Son but est de lancer le mode "express scan" puis de traiter chaque trame compl�te d�s qu'elle est re�ue
void StartRPLIDARTask(void const * argument)
{
	// D�marrage PWM
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);	// Timer 8 canal 3 pour la g�n�ration de PWM (r�gl� � 25 KHz / 60% via CubeMX)

	// Reset RPLIDAR
	//HAL_UART_Transmit_DMA(&huart3, rplidar_reset_cmd, 2);	// non-DMA, pour ne pas d�clencher la callback
	osDelay(50);		// La doc du LIDAR dit d'attendre 2 ms, mais on n'est pas � �a pr�s
	// Initialisation express scan
	HAL_UART_Transmit_DMA(&huart3, rplidar_tx_buffer, 9);	// Envoi de la trame de d�marrage du LIDAR
	HAL_UART_Receive_DMA(&huart3, rplidar_tx_buffer, 7);	// Lecture du descripteur de r�ponse renvoy� par le LIDAR
	// Note : on utilise bien le buffer d'�mission pour la r�ception dans ce cas, ce n'est pas une typo
	// La transmission et la r�ception vont prendre 16 octets � 115.2 Kbps = 1.1 ms

	// On attend maintenant la fin de la r�ception du descripteur de r�ponse:
	while (rplidar_status == 0)
	{
		//rplidar_test_1 = 1;
		osDelay(2);		// Le temps de traitement du RPLIDAR n'est pas connu, � mesurer !
	}
	// Le LIDAR va maintenant envoyer des trames de donn�es � r�p�tition, qu'il faut traiter:
	for(;;)
	{
		// y a-t'il une nouvelle trame dans le buffer de r�ception de l'UART ? Sinon, on attend :
		while (rplidar_data_trigger == 0)
			osDelay(1);

		//rplidar_test_1 = 10;
		//rplidar_test_2++;
		//rplidar_test_3 = offset;

		// remise � z�ro du trigger:
		rplidar_data_trigger = 0;

		// traitement de la trame par une fonction d�di�e
		decodage_express_scan (rplidar_rx_buffer + offset);

		// mise � jour de l'offset pour la prochaine trame:
		offset = 84 - offset;	// bascule entre les deux buffers

	}
}

// Callback UART3 DMA : cette fonction est d�clar�e par la HAL comme "weak", on la red�finit ici
// Attention ! Elle est utilis�e par TOUTES les UART (possible complication dans le futur...)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// rplidar_test_2++;	// Weird, we're never coming here again after the first reponse from the LIDAR...

	// Le test suivant v�rifie quelle UART vient de terminer une r�ception en mode DMA :
	if (huart->Instance == USART3)	// RPLIDAR
	{
		rplidar_test_1 = 2;
		//rplidar_test_2++;
		// Un transfert DMA complet (r�ception de trame RPLIDAR) a eu lieu : traitons-le
		// On attend deux trames possibles : descripteur de r�ponse, ou r�ponse. Notre variable �tat nous dit laquelle:
		if (rplidar_status == 0)
		{
			// Le RPLIDAR n'est pas encore actif, on a donc affaire � un descripteur de r�ponse.
			if (rplidar_tx_buffer[6] == 0x82) 	// On v�rifie...
			{
				// We're never getting here...
				rplidar_test_1 = 3;
				rplidar_status = 1;				// ... et on transitionne � l'�tat actif si c'est bien le cas

				// lancement du premier transfert DMA pour les donn�es:
				HAL_UART_Receive_DMA(&huart3, rplidar_rx_buffer + offset_dma, 84);
				offset_dma = 84 - offset_dma;	// bascule entre les deux buffers
				return;
			}
			else
			{
				// L�, on a un p�pin... on a re�u une trame mais pas la bonne... on g�rera �a dans une version ult�rieure
				//rplidar_test_1 = 4; // J'arrive ici... bizarre parce que j'ai bien re�u la trame.
				rplidar_test_2 = rplidar_tx_buffer[6];	// what do we see exactly ?
				return;		// pour le moment, on se barre...
			}
		}

		rplidar_test_1 = 5;

		// Si on arrive ici, c'est que le LIDAR est actif et qu'on vient de recevoir une trame de donn�es

		if (rplidar_data_trigger == 1)
		{
			// probl�me : la pr�c�dente trame re�ue n'a pas encore �t� trait�e !
			rplidar_test_1 = 6;
			return;		// on g�rera ce cas dans une version ult�rieure
		}

		rplidar_test_1 = 7;

		rplidar_data_trigger = 1;	// Indiquer � la t�che qu'elle a du travail
		// Il y a certainement un meilleur m�canisme (wait sur signal) mais on verra �a plus tard

		// Mise � jour de l'offset d'indice:
		offset_dma = 84 - offset_dma;	// bascule entre les deux buffers

		// On lance le transfer DMA suivant:
		HAL_UART_Receive_DMA(&huart3, rplidar_rx_buffer + offset_dma, 84);
	}
}

void decodage_express_scan (uint8_t* t)
{
	// t[] est une table de 84 octets repr�sentant une trame de donn�es du LIDAR (express scan)
	// celle-ci est compos�e d'un ent�te de quatre octets suivie de cinq blocs de donn�es de m�me structure, contenant
	// deux �chantillons de distance chacun.

	struct trame_express *f = t;	// On va mapper f sur une structure pour faciliter le d�codage

	//rplidar_test_3 = f->sync1;

	// Et flute... les unions �a marche pas... va falloir tout convertir � la main !
	uint8_t sync  = (t[0] & 0xF0) | (t[1] >> 4);
	uint8_t cksum = (t[0] & 0x0F) | (t[1] << 4);
	uint8_t	S = t[3] >> 7;	// Ce bit ne passe jamais � 1... �a pourrait �tre d� � la vitesse de rotation ?
	//uint16_t start_angle_q6 = (((uint16_t) t[3] & 0x7F) << 8) | (uint16_t) t[2];	// virgule fixe, 6 bits apr�s la virgule
	// uint16_t start_angle_q6 = (t[3] << 8) | t[2];	// virgule fixe, 6 bits apr�s la virgule
	uint16_t start_angle_q6 = (t[3] * 256) + t[2];	// par curiosit�
	rplidar_start_angle = start_angle_q6;
	rplidar_test_3 += S;

	// V�rifier la synchro:
	// if ((f->sync1 == 0xA) && (f->sync2 == 0x5))
	if (sync == 0xA5)
	{
		// Bonne synchro ! R�cup�rons le checksum
		//uint8_t cksum = (f->cksum_hi << 4) | f->cksum_lo;
		// Verification du checksum par rapport au reste de la trame:
		int k;
		uint8_t sum = 0;
		for (k=2; k<84; k++)	// Les deux premiers octets contiennent le checksum et ne sont logiquement pas compris dans le calcul.
			sum ^= t[k];	// XOR de tous les octets de la trame

		if (sum == cksum)
		{
			// checksum valide, on continue avec l'angle de d�part
			rplidar_test_2++;
		}
		else
		{
			// checksum invalide, on g�rera plus tard
		}

	}
}
