/*
	FILE IMPLEMENTAZIONI LIDARDRIVER.CPP
	Autore:		Andrea Visonà

	Vengono implementate le funzioni della libreria LidarDriver.h
*/

#include "../include/LidarDriver.h"
#include <vector>  // per operazioni su vector

namespace lidar_driver {
	/* Funzione clear_buffer():
		1. reimposta gli indici di posizione e la dimensione
		2. rialloco il vettore del buffer

		Osservazione:
		- non effettuo controlli se il buffer è vuoto, perché anche se dimension = 0, non è detto
		  che tutti i vettori 
	 */
	void LidarDriver::clear_buffer() {
		// Reimposta le variabili dell'oggetto
		elPiNovo = elPiVecio = dimension = 0;

		// rialloco il vettore del buffer come nel costruttore
		std::vector<std::vector<double>>(BUFFER_DIM).swap(secia);
	}
}
