# LDP-intermediate-project
Progetto intermedio del corso di Laboratorio di Programmazione

## TO-DO List
1. nella funzione get_scan si può restituire il vettore scansione senza farne la copia, usando la funzione move
2. nella funzione clear_buffer:
   1. renderla eseguibile anche nel caso in cui dimension sia 0, ma ci possono ancora essere dati memorizzati nel vettore e gli indici non è detto che siano entrambi sul primo elemento del vettore
   2. verificare se ha senso usare la versione2 al posto del ciclo for
3. sistemare costruttori di copy
4. rivedere funzione new_scan per bug

## Descrizione Classe LidarDriver
La classe memorizza un certo numero di scansioni, effettuate da un Lidar, in un buffer. Le scansioni sono ottenute dalla misurazione della distanza per vari angoli consecutivi da 0° a 180°. La risoluzione angolare, ovvero la distanza tra due misurazioni consecutive, dipende dal tipo di Lidar ed è compresa tra 0.1° e 1°.

## Specifiche classe LidarDriver
- riceve in input la risoluzione angolare del Lidar
- memorizza le scansioni in un buffer di dimensione costante BUFFER_DIM
- se il buffer è pieno, la nuova scansione sovrascrive quella più vecchia
- le scansioni sono std::vector<double>

## Dettagli implementativi
i dettagli implementativi sono contenuti nel file ``include/LidarDriver.h``
<!--  dettagli implementativi non aggiornati

| Variabili private | descrizione |
| - | - |
| secia | std::vector<*std::vector<double>> vettore di puntatori a vettori double, ovvero vettore di puntatori alle varie scansioni, con dimensione fissata di BUFFER_DIM |
| elPiNovo | indice dell'ultimo elemento inserito |
| elPiVecio | indice del primo elemento inserito |
| dimension | numero di elementi nel buffer |
| resolusion | risoluzione angolare dello strumento |

| Costanti | descrizione |
| - | - |
| BUFFER_DIM | dimensione massima del buffer |
| MIN_ANGLE | angolo minimo scansione |
| MAX_ANGLE | angolo massimo scansione |
| MIN_RESOLUTION | risoluzione minima |
| MAX_RESOLUTION | risoluzione massima |

| Classi eccezioni | descrizione |
| - | - |
| NoGheSonVettoriError | eccezione lanciata nel caso di operazioni invalide su buffer vuoto |
| NullVettorError | eccezione lanciata quando si ricevono parametri nullptr/null |
| ResolusionForaDaiRangeError | eccezione lanciata quando la risoluzione angolare è al di fuori dei range specificati nella consegna |

| Costruttori | descrizione |
| - | - |
| LidarDriver(double) | specifica la risoluzione angolare dello strumento |
| LidarDriver(const LidarDriver &) | costruttore di copia |
| LidarDriver(LidarDriver &&) | costruttore di move |
| ~LidarDriver() | distruttore |

| Member function pubbliche | descrizione |
| - | - |
| void new_scan(vector<double>) | aggiunge una nuova scansione al buffer |
| vector<double> get_scan() | restituisce la più vecchia scansione inserita e la rimuove dal buffer |
| void clear_buffer() | elimina tutte le scansioni nel buffer |
| double get_distance(double) const | restituisce la lettura corrispondente all'angolo specificato come parametro (o al suo angolo più vicino) presente nell'ultima scansione inserita nel buffer senza rimuoverla |
| operator<< const | stampa l'ultima scansione inserita nel buffer senza rimuoverla |
-->
