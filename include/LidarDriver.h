/*
    FILE HEADER LIDARDRIVER.H
    Versione:   3.0
    Data:       21/11/2024
    Autore:     Giovanni Bordignon

    Osservazioni (1.0 - G. Bordignon):
        1. BUFFER_DIM dichiarata costante e statica. OK?
        2. Indice "coa", da trovare nome per eventuale secondo indice.
        3. Per ora dichiarato solo il costruttore di default e copia, serve altro?.
        4. Valutare se tornare il vettore sovrascritto dalla funzione new_scan();
        5. (Funzioni const)

    Osservazioni (3.0 - G. Bordignon)
        1. Verificare i punti 1 (IMPORTANTE, valida anche per MAX_LETTURE) - 3 - 4 delle Osservazioni della versione 1.0;
        2. Valutare eventuale cambio nome degli indici agli elementi.
*/

#ifndef LIDARDRIVER_H
#define LIDARDRIVER_H

#include <iostream>
#include <vector>

namespace lidar_driver {
	class LidarDriver {
		public:
			LidarDriver();
			LidarDriver(const LidarDriver &);
			void new_scan(std::vector<double>);
			std::vector<double> get_scan();
			void clear_buffer();
			double get_distance(double);

			class NoGheSonVettoriError {}; // Eccezione "NoGheSonVettori" ("NoCiSonoVettori")
			class NullVettorError {};
		private:
			static constexpr int BUFFER_DIM{10};
			static constexpr int DIM_LETTURE{181};
			std::vector<double> *secia; // BUFFER ("secia" = secchio)
			int elPiNovo;   // Indice all'ultimo vettore inserito ("elPiNovo" = ilPiùNuovo)
			int elPiVecio;  // Indice al vettore da più tempo presente nel buffer ("elPiVecio" = ilPiùVecchio)
			int dimension;  // Dimensione attuale del buffer
	};

	std::ostream &operator<<(std::ostream &, LidarDriver &);
};

#endif // LIDARDRIVER_H