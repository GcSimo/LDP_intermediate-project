/*
	FILE IMPLEMENTAZIONI LIDARDRIVER.CPP
	Autore:		Giovanni Bordignon

	Vengono implementate le funzioni della libreria LidarDriver.h
*/

#include "../include/LidarDriver.h"
#include <vector>  // per operazioni su vector

namespace lidar_driver {
	/* Funzione get_last():
		- La funzione restituisce l'ultimo vettore inserito, in caso il buffer sia vuoto viene
		  lanciata l'eccezione "NoGheSonVettoriError".

		- La funzione non è richiesta dalle specifiche, ma viene usata nella helper function
		  dell'overloading dell'operatore <<
	*/
	std::vector<double> LidarDriver::get_last() const {
		if (dimension == 0)
			throw NoGheSonVettoriError();
		
		return secia[elPiNovo];
	}

	/* Overloading dell'operatore <<
		Con un try - catch viene gestito il caso in cui il buffer sia vuoto:
		- la funzione get_last lancia infatti l'eccezione "NoGheSonVettori", che, recepita dalla presente
		  funzione, permette di inviare immediatamente allo stream di output una generica stampa di un array
		  vuoto
		- nel caso l'eccezione non si presenti, sintomo che il buffer non è vuoto, l'ultimo vettore inserito
		  viene restituito dalla funzione get_last() e stampato con un'opportuna formattazione.
	*/
	std::ostream &operator<<(std::ostream& os, const LidarDriver& ld) {
		try {
			std::vector<double> temp = ld.get_last(); // <- qui si potrebbe lanciare l'eccezione
			std::string s = "{ ";
			for (int i = 0; i < temp.size(); i++) {
				s += std::to_string(temp[i]);
				if (i < temp.size() - 1)
					s += ", ";
			}
			s += " }\n";

			return os << s;
		}
		catch (LidarDriver::NoGheSonVettoriError) {
			return os << "{ }\n";
		}
	}
}
