/*
	FILE TESTER MAIN.CPP
	Autore:		Andrea Visonà

	Vengono testate le funzioni della classe LidarDriver
*/

#include <iostream>
#include "../include/LidarDriver.h"
using namespace std;
using namespace lidar_driver;

int main() {
	// creo un oggetto LidarDriver con risoluzione superiore o inferiore di (0.1-1)
	try { LidarDriver lidar(2); LidarDriver lida(0.01); }
	catch(LidarDriver::ResolusionForaDaiRangeError) {
		cout<<"<<errore (voluto) di risoluzione>>"<<endl;
	}
	// creo un oggetto LidarDriver con risoluzione 1 quindi 181 valori
	LidarDriver LD(1);

	// proviamo a ottenere un elemento sebbene sia impossibile ora
	try {
		vector<double> a = LD.get_last();
		cout<<a.size();
	}catch(LidarDriver::NoGheSonVettoriError){
		cout<<"<<errore (voluto) al fatto che non ci siano vettori da restituire>>"<<endl;
	}
	
	// riempio il buffer con 10 vettori con rispettivamente il loro indice come valore di tutti i 180 elementi
	for(int j=0;j<10;j++){
		vector<double> v;
		// il for è fino a 200 per dimostrare che tronca alla cifra che gli interessa e basta
		for(int i=0;i<200;i++){
			// il primo valore sarà la posizione rispettiva del vettore nel buffer
			// il secondo sarà invece la posizione nel rispettivo vettore, così volontariamente capirò subito
			// se qualcosa non va appena vedo dei valori non aspettati
			v.push_back(i+j*1000);
		}
		LD.new_scan(v);
	}
	
	cout<<"quale sara' il vettore attuale? mi aspetto che mi stampi l'ultimo, quindi con 9000 come migliaia : "<<endl;
	cout<<LD<<endl;
	// l'operator << viene eseguito e stampa tutti i valori dell'ultimo aggiunto
	
	// ora controllo cosa succede se aggiungo un vettore con meno elementi e se, come mi aspetto, sovrascriva
	// il primo elemento aggiunto, ovvero quello con 0 come migliaia
	vector<double> v;
	for(int i=0;i<40;i++){
		v.push_back(10000+i);
	}
	LD.new_scan(v);
	cout<<"mi aspetto che i primi 40 elementi siano dei valori da 10000 a 10039 e gli altri 0 : "<<endl;
	cout<<LD<<endl;

	// ora proviamo a togliere qualcosa
	cout<<" inizio controllando l'ultimo vettore immesso, di cui abbiamo i dati direttamente sopra nel terminale"<<endl<<endl;
	vector<double> x = LD.get_last();
	cout<<" primo valore dell'ultimo vettore : "<<x[0]<<" e ultimo valore dell'ultimo vettore : "<<x[x.size()-1]<<endl;
	
	cout<<" ora controllo il vettore piu vecchio e lo scarto con il metodo get_scan() e successivamente la richiamo"<<endl;
	// per assicurarmi di aver tolto il vettore
	vector<double> y = LD.get_last();
	cout<<" primo valore dell'ultimo vettore : "<<y[0]<<" e ultimo valore dell'ultimo vettore : "<<y[y.size()-1]<<endl;
	cout<<" la richiamo e notero' che si tratta del vettore aggiunto subito dopo quello appena eliminato"<<endl;
	vector<double> z = LD.get_scan();
	cout<<" primo valore dell'ultimo vettore : "<<z[0]<<" e ultimo valore dell'ultimo vettore : "<<z[z.size()-1]<<endl<<endl;
	
	// ora controllo che l'assegnamento sia corretto
	LidarDriver LD2(1);
	LD2 = LD;
	// ora sono uguali, quindi per vedere cosa succede devo togliere a uno dei due un vettore
	
	LD.get_scan(); // perdo l'ultimo vettore
	vector<double> vld = LD.get_scan();
	vector<double> vld2 = LD2.get_scan();
	cout<<"primo valore del vettore piu vecchio di LD : "<<vld[0]<<" ultimo valore del vettore piu vecchio di LD : "<<vld[vld.size()-1]<<endl;
	cout<<"primo valore del vettore piu vecchio di LD2 : "<<vld2[0]<<" ultimo valore del vettore piu vecchio di LD2 : "<<vld2[vld2.size()-1]<<endl;
	cout<<endl<<"come si puo notare da qua sopra, il primo a cui era gia stato tolto un vettore, ora sta stampando i valori del 4o vettore, mentre LD2 e' ancora al 3o"<<endl<<endl;
	// gia che ci sono, continuo a togliere vettori finché posso a uno dei due e poi confronto
	
	try{
		while(true){
			LD.get_scan();
		}
	}catch(LidarDriver::NoGheSonVettoriError){
		cout<<"come mi aspettavo, non si puo' togliere un vettore che nemmeno c'e', anche perché sono appena uscito da un loop infinito se non fosse per l'errore lanciato dal metodo get_scan()"<<endl;
	}
	// ora provo a svuotare anche l'altro vettore
	LD2.clear_buffer();
	// per valutare se ha funzionato... proviamo di nuovo con l'errore
	try{
		LD2.get_scan();
	}catch(LidarDriver::NoGheSonVettoriError){
		cout<<"il vettore e' stato effettivamente svuotato con successo"<<endl<<endl;
	
	}

	// ora testo la funzione get_distance
	// creo un buffer di un lidar con risoluzione 1 e due vettori da 181 elementi
	LidarDriver ld1(1);
	vector<double> v1(181);
	vector<double> v2(181);
	// popolo un vettore con valori da 1 a 181 e lo inserisco nel buffer
	for (int i = 0; i < 181; i++)
		v1[i] = i;
	ld1.new_scan(v1);
	
	// vado a pescare i valori per ogni angolo da 0 a 180 e li salvo nel secondo vettore
	// i valori degli angoli sono esatti e non serve approssimare
	for (int i = 0; i < 181; i++) {
		v2[i] = ld1.get_distance(i);
	}

	// ora verifico che i vettori di inserimento e rimozione siano uguali
	if(v1 == v2)
		cout<<"distanza angoli esatti -> corretta"<<endl;
	else
		cout<<"distanza angoli esatti -> sbagliata"<<endl;

	// ora testo la funzione per angoli in cui si va incontro ad una approssimazione
	// vado a pescare i valori per ogni angolo da 0 a 180 con un certo errore e li salvo nel secondo vettore
	// i valori degli angoli non sono esatti e nella funzione si va incontro ad una approssimazione
	for (int i = 0; i < 181; i++) {
		if (i < 100) // errore per i primi 100 angoli
			v2[i] = ld1.get_distance(i+0.1);
		else // cambio errore tanto per variare un po'
			v2[i] = ld1.get_distance(i-0.1);
	}
	// ora verifico che i vettori di inserimento e rimozione siano uguali
	if(v1 == v2)
		cout<<"distanza angoli con approssimazione -> corretta"<<endl;
	else
		cout<<"distanza angoli con approssimazione -> sbagliata"<<endl;
	
	// verifico anche il caso in cui gli angoli siano fuori dalla portata o buffer vuoto
	try {
		ld1.get_distance(-1);
	} catch (LidarDriver::AngoloForaDaiRangeError) {
		cout << "<<errore voluto - eccezione lanciata correttamente per angoli troppo piccoli>>" << endl;
	}
	try {
		ld1.get_distance(182);
	} catch (LidarDriver::AngoloForaDaiRangeError) {
		cout << "<<errore voluto - eccezione lanciata correttamente per angoli troppo grandi>>" << endl;
	}
	try {
		ld1.clear_buffer();
		ld1.get_distance(50);
	} catch (LidarDriver::NoGheSonVettoriError) {
		cout << "<<errore voluto - eccezione lanciata correttamente se si vuole leggere una misura in un buffer vuoto>>" << endl;
	}

	return 0;
}
