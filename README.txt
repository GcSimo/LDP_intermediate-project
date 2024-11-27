Gruppo ~BankAccount();

Autori dei vari files
Files scritti da Giovanni Bordignon:
- include/LidarDriver.h
- src/LidarDriver_pt1.cpp

Files scritti da Giacomo Simonetto:
- src/LidarDriver_pt2.cpp

Files scritti da Andrea Visonà:
- src/main.cpp
- src/LidarDriver_pt3.cpp

Suddivisione dei file:
- Abbiamo spezzettato il file src/LidarDriver.cpp in tre parti: la prima contiene le funzioni
  implementate da Giovanni, la seconda contiene le funzioni implementate da Giacomo e la terza
  contiene le funzioni implementate da Andrea. In questo modo c'è un solo autore per ogni file,
  come richiesto dalle specifiche.

- Ci rendiamo conto che si potrebbero avere problemi di compilazione, dato che il file con le
  implementazioni delle funzioni della libreria è stato spezzettato e rinominato. Per ovviare a
  questo problema abbiamo anche incluso il file src/LidarDriver.cpp tutto unito insieme.
