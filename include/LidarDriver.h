/*
	FILE HEADER LIDARDRIVER.H
	Autore:		Giovanni Bordignon

	La classe si compone di un buffer dove vengono salvate le scansioni dello strumento e di alcune
	funzioni per maneggiare i dati memorizzati.

	Note sulla implementazione del buffer:
	 - il buffer è implementato come vettore (o coda) circolare di dimensione costate di BUFFER_DIM
	   con indice del primo elemento, indice dell'ultimo elemento e una variabile che tiene traccia
	   della dimensione occupata
	 - secia     -> vettore del tipo std::vector<std::vector<double>>
	 - elPiNovo  -> indice dell'ultimo vettore inserito
	 - elPiVecio -> indice dell'elemento nel vettore da più tempo
	 - dimension -> dimensione occupata nel buffer

	Costanti private della classe:
	- int BUFFER_DIM = 10         -> dimensione massima del buffer
	- int MIN_ANGLE = 0           -> angolo minimo da cui parte la scansione
	- int MAX_ANGLE = 180         -> angolo massimo in cui termina la scansione
	- double MIN_RESOLUTION = 0.1 -> risoluzione minima accettata
	- double MAX_RESOLUTION = 1   -> risoluzione massima accettata

	Variabili rpivate della classe:
	- std::vector<std::vector<double>> secia ->
	- int elPiNovo      -> indice dell'ultimo vettore inserito
	- int elPiVecio     -> indice dell'elemento nel vettore da più tempo
	- int dimension     -> dimensione occupata nel buffer
	- int dimScansioni  -> dimensione dei vettori delle scansioni
	- double resolusion -> risoluzione angolare dello strumento

	Nota sui costruttori-operatori di copia e di move:
	1. non serve implementare il costruttore e l'operatore di copia perché è sufficiente la shallow copy
	   membro a membro: non abbiamo puntatori da gestire e il vettore secia applica la copia membro a membro
	   sugli elementi che contiene come definita dalla classe std::vector
	2. serve, invece, il costruttore di move e l'operatore di move per risparmiare dati e tempo: meglio non
	   copiare i vettori delle scansioni se si può evitare facendo una move
	
	Costruttori:
	- LidarDriver(double)         -> costruttore che riceve come parametro la risoluzione dello strumento
	- LidarDriver(LidarDriver &&) -> costruttore di move
	
	Funzioni membro:
	- void new_scan(std::vector<double>)   -> inserisce nel buffer la scansione passata come parametro
	- std::vector<double> get_scan()       -> restituisce e rimuove dal buffer la scansione più vecchia
	- std::vector<double> get_last() const -> restituisce senza rimuovere l'ultima scansione inserita
	- void clear_buffer()                  -> svuota il buffer da tutte le scansioni
	- double get_distance(double) const    -> restituisce la misura effettuata nell'ultima scansione per
	                                          uno specifico angolo passato come parametro

	Overloading operatori
	void operator=(LidarDriver &&)                                -> overloading operatore di move
	std::ostream &operator<<(std::ostream &, const LidarDriver &) -> overloading operatore per output stream

	Classi per lancio di eccezioni
	- class NoGheSonVettoriError{}        -> classe lanciata in caso si tenta di leggere/accedere/rimuovere
	                                         delle scansioni quando il buffer è vuoto
	- class ResolusionForaDaiRangeError{} -> classe lanciata se la risoluzione passata al costruttore non è valida
	- class AngoloForaDaiRangeError{}     -> classe lanciata se l'angolo passato a get_distance non è valido
*/

#ifndef LIDARDRIVER_H
#define LIDARDRIVER_H

#include <ostream>
#include <vector>

namespace lidar_driver {
	class LidarDriver {
		public:
			// costruttori e distruttori
			LidarDriver(double);
			LidarDriver(LidarDriver &&);

			// member function
			void new_scan(std::vector<double>);
			std::vector<double> get_scan();
			std::vector<double> get_last() const;
			void clear_buffer();
			double get_distance(double) const;

			// overloading operatori
			void operator=(LidarDriver &&);
			LidarDriver& operator=(LidarDriver&);

			// classi per lancio di errori
			class NoGheSonVettoriError{}; // Eccezione "NoGheSonVettori" ("NoCiSonoVettori")
			class ResolusionForaDaiRangeError{};
			class AngoloForaDaiRangeError{};

		private:
			// costanti private
			static constexpr int BUFFER_DIM{10};
			static constexpr int MIN_ANGLE{0};
			static constexpr int MAX_ANGLE{180};
			static constexpr double MIN_RESOLUTION{0.1};
			static constexpr double MAX_RESOLUTION{1};

			// variabili private
			std::vector<std::vector<double>> secia;	// BUFFER ("secia" = secchio)
			int elPiNovo;		// Indice all'ultimo vettore inserito ("elPiNovo" = ilPiùNuovo)
			int elPiVecio;		// Indice al vettore da più tempo presente nel buffer ("elPiVecio" = ilPiùVecchio)
			int dimension;		// Dimensione utilizzata del buffer
			int dimScansioni;	// Dimensione dei vettori delle scansioni
			double resolusion;	// Risoluzione angolare dello strumento
	};

	// overloading operatore output
	std::ostream &operator<<(std::ostream &, const LidarDriver &);
}

#endif // LIDARDRIVER_H
