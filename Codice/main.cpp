/**
* @file modelv0.8.cpp
* @brief
*/

#include <cstdio>
#include <iostream>
#include <vector>
#include <fstream>
#include <time.h>
#include "cpxmacro.h"
#include <chrono>
#include <random>

#define DISCONNECTED_COST CPX_INFBOUND

using namespace std;
using namespace std::chrono;

struct solution_t
{
	vector<double> yPositions;
	double objValue;
};

//test coordinate discrete su griglia
struct nodesCoordinates_t
{
	int x;
	int y;
	int id;
};

// error status and message buffer
int status;
char errmsg[BUF_SIZE];

const int NAME_SIZE = 512;
char name[NAME_SIZE];

//Global data

//numero di utenti
int n = 0;
// numero di droni
int d = 0;
//numero di posizioni potenziali dei droni
int P = 0;


int totalNodes = n + d;
int totalPotentialNodes = n + P;

//numero di commodities, ovvero numero di coppie distinte sorgente-destinazione
int K = n*(n - 1); 

const int s = 10; //numero massimo connessioni sostenibili da un drone

const int bigM = 1000; // TODO: bigM deve essere maggiore della capacità massima dei link

const int threshold = 100; //soglia oltre la quale un costo viene considerato infinito



//indici iniziali per la memorizzazione delle variabili di CPLEX
int f_index = 0;
int y_index = 0;
int x_index = 0;
int w_index = 0;

//int t[n][n];
//matrice di traffico
vector< vector<int> > t(n, vector<int>(n, -1)); //matrice di traffico

vector<double> deployCost(P, 50); //costo deployment drone

//int graph[n+d][n+d];
//matrice di adiacenza del grafo
//vector< vector<int> > graph(totalPotentialNodes, vector<int>(totalPotentialNodes, 0));


//double c[n+d][n+d][K];
//matrice dei costi
vector< vector< vector<double> > > c(totalPotentialNodes, vector< vector<double> >(totalPotentialNodes, vector<double>(K, DISCONNECTED_COST)));

//matrice di capacità degli archi
vector< vector<double> > u(totalPotentialNodes, vector<double>(totalPotentialNodes));

//matrice bilanciamento flussi
vector< vector<double> > b(totalPotentialNodes, vector<double>(K, 0));


const double zero = 0.0;
const double uno = 1.0;


vector<nodesCoordinates_t> createNodesStruct(int size)
{
	vector<nodesCoordinates_t> *nodi = new vector<nodesCoordinates_t>(size);
	return nodi;
}

void setNodesCoordinates(vector<nodesCoordinates_t> &nodes, int RowSize)
{
	int rows=0;
	for(int i= 0; i<nodes.size();i++ )
	{
		if(i%8 < )
	}
}

bool areOutOfSight(int i, int j)
{
	int k = 0;
	bool flag = true;
	while (k < K && flag == true)
	{
		if (c[i][j][k] <= threshold)
			flag = false;
		k++;
	}
	return flag;
}


double getdRand(double inf, double sup)
{
	unsigned seed1 = system_clock::now().time_since_epoch().count();

	std::mt19937 generator(seed1);   // mt19937 is a standard mersenne_twister_engine
	std::uniform_real_distribution<double> distribution(inf, sup);
	return distribution(generator);
}

//generatore pseudo-casuale di numeri int nel range inf - sup
/*int getRand(int inf, int sup)
{
	return (inf + rand() % (sup - inf));
}*/

int getRand(int inf, int sup)
{
	// obtain a seed from the system clock:
	unsigned seed1 = system_clock::now().time_since_epoch().count();

	std::mt19937 generator(seed1);   // mt19937 is a standard mersenne_twister_engine
	std::uniform_int_distribution<int> distribution(inf, sup);
	return distribution(generator);
}

void randomizeTraffic(int minVal, int maxVal)
{
	for (unsigned int i = 0; i < t.size(); i++)
	{
		for (unsigned int j = 0; j < t[0].size(); j++)
		{
			if (i != j)
				t[i][j] = getRand(minVal, maxVal);
		}
	}
}

void randomizeCosts(double minCVal, double maxCVal, double minUVal, double maxUVal)
{
	for(unsigned int i=0; i < c.size(); i++)
	{
		for(unsigned int j=0; j< c[0].size(); j++)
		{
			for(unsigned int k=0; k< c[0][0].size(); k++)
			{
				if(i!=j)
				{
					c[i][j][k] = getdRand(minCVal,maxCVal);
					if(c[i][j][k] > threshold)
					{
						u[i][j]=0;
					}
					else
					{
						u[i][j] = getdRand(minUVal,maxUVal);
					}
				}
			}
		}
	}
}

void randomizeDeployCost(double minVal, double maxVal)
{
	for(unsigned int i=0; i<deployCost.size();i++)
	{
		deployCost[i]=getdRand(minVal,maxVal);
	}
}


void createRandomInstance(int users, int drones, int positions)
{
	int result=0;
	n=users;
	d=drones;
	P=positions;  
	result= initDataStructures(users,drones,positions);
	if(result==0)
	{
		randomizeTraffic(tInf,tSup);
		randomizeCosts(cInf,cSup);
	}
	else
	{

	}
}

//discount factor relativo al costo di trasmissione tra hubs
void applyDiscount(double alpha)
{
	for(int i=n;i<totalPotentialNodes; i++)
	{
		for(int j=n;j<totalPotentialNodes;j++)
		{
			for(int k=0;k<K;k++)
			{
				if(i!=j)
				{
					c[i][j][k]*=alpha;
				}
			}
		}
	}
}

void setupLP(CEnv env, Prob lp)
{
	//allocazione mappe
	vector< vector< vector<int> > > fMap(totalPotentialNodes, vector< vector<int> >(totalPotentialNodes, vector<int>(K, -1)));
	vector< vector<int> > xMap(totalPotentialNodes, vector<int>(totalPotentialNodes, -1));

	vector< vector<int> > wMap(totalPotentialNodes, vector<int>(totalPotentialNodes, -1));

	int baseDroneIndex = n;

	//aggiunta delle variabili f_i_j_k, una per volta, con i!=j
	//nota: la variabile f_i_j_k e' distinta dalla variabile f_j_i_k
	int fMapIndex = 0;
	for (int k = 0; k < K; k++)
	{
		for (int i = 0; i < totalPotentialNodes; i++)
		{
			for (int j = 0; j < totalPotentialNodes; j++)
			{
				
				//if (i != j) //c'è un arco tra i nodi i e j, quindi creo la corrispondente variabile di flusso fijk
				if (i != j && (c[i][j][k] <= threshold && u[i][j]>0) && !(i < n && j < n)) //***
				{
					char ftype = 'I';
					double lb = 0.0;
					double ub = CPX_INFBOUND;

					snprintf(name, NAME_SIZE, "f_%d_%d_%d", i, j, k); //scrive il nome della variabile f_i_j_k sulla stringa name[]
					char* fname = (char*)(&name[0]);
					CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &c[i][j][k], &lb, &ub, &ftype, &fname);   //costruisce la singola variabile 

					//costruzione della map
					fMap[i][j][k] = fMapIndex;
					fMapIndex++;
				}
			}
		}
	}

	printf("Sono state create %d variabili f_i_j_k\n", CPXgetnumcols(env, lp)); //numero totale delle vars create	

	//numerazione nodi: da 0 a n sono gli utenti, da n a n+d sono le posizioni potenziali
	//inserimento delle variabili y_i

	y_index = CPXgetnumcols(env, lp);

	for (int i = 0; i < P; i++)
	{
		char ytype = 'B';
		double lb = 0.0;
		double ub = 1.0;

		snprintf(name, NAME_SIZE, "y_%d", baseDroneIndex + i); //scrive il nome della variabile y_i sulla stringa name[]
		char* yname = (char*)(&name[0]);
		CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &deployCost[i], &lb, &ub, &ytype, &yname);


	}
	printf("Sono state create %d variabili y_i\n", CPXgetnumcols(env, lp) - y_index); //numero totale delle vars create	

	//aggiunta delle variabili x_i_j

	//si divide in due step la creazione delle variabili x_i_j
	//variabili relative agli "utenti"
	x_index = CPXgetnumcols(env, lp);
	int xMapIndex = x_index;

	for (int i = 0; i < n; i++)
	{
		//esclude la creazione delle variabili con i==j e quelle che esprimono link tra due utenti
		for (int j = baseDroneIndex; j < totalPotentialNodes; j++)
		{
			if (!areOutOfSight(i, j) && u[i][j]>0) //***
			{
				char xtype = 'B';
				double lb = 0.0;
				double ub = 1.0;

				snprintf(name, NAME_SIZE, "xu_%d_%d", i, j); //scrive il nome della variabile x_i_j sulla stringa name[]
				char* xname = (char*)(&name[0]);
				CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &zero, &lb, &ub, &xtype, &xname);   //costruisce la singola variabile

				//costruzione della mappa
				xMap[i][j] = xMapIndex;
				xMapIndex++;
			}
		}
	}

	//variabili relative ai "droni"
	for (int i = baseDroneIndex; i < totalPotentialNodes; i++)
	{
		for (int j = 0; j < totalPotentialNodes; j++)
		{
			//esclude le variabili x_i_j con i==j
			//if(i!=j)
			if (i != j && (!areOutOfSight(i, j) && u[i][j]>0)) //***
			{
				char xtype = 'B';
				double lb = 0.0;
				double ub = 1.0;

				snprintf(name, NAME_SIZE, "xd_%d_%d", i, j); //scrive il nome della variabile x_i_j sulla stringa name[]
				char* xname = (char*)(&name[0]);
				CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &zero, &lb, &ub, &xtype, &xname);   //costruisce la singola variabile 

				//costruzione della mappa
				xMap[i][j] = xMapIndex;
				xMapIndex++;
			}
		}
	}

	printf("Sono state create %d variabili x_i_j\n", CPXgetnumcols(env, lp) - x_index); //numero totale delle vars create	

	//aggiunta variabili  w_i_j
	int w_index = CPXgetnumcols(env, lp);
	int wMapIndex = w_index;
	for (int i = 0; i < totalPotentialNodes; i++)
	{
		char wtype = 'I'; //***
		double lb = 0.0;
		double ub = CPX_INFBOUND;
		for (int j = 0; j < totalPotentialNodes; j++)
		{
			//if (i != j && !(i < n && j < n) && u[i][j] != 0)
			if (i != j && !(i < n && j < n) && u[i][j] > 0) //***
			{
				snprintf(name, NAME_SIZE, "w_%d_%d", i, j); //scrive il nome della variabile w_i_j sulla stringa name[]
				char* wname = (char*)(&name[0]);
				CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &zero, &lb, &ub, &wtype, &wname);   //costruisce la singola variabile 

				//costruzione della mappa
				wMap[i][j] = wMapIndex;
				wMapIndex++;
			}
		}
	}

	printf("Sono state create %d variabili w_i_j\n", CPXgetnumcols(env, lp) - w_index); //numero totale delle vars create	

	
	//vincoli

	char sense;
	int matbeg;
	vector<int> idx;
	vector<double> coef;

	int rowNumber = 0;

	// 1. capacità dei link
	sense = 'L';
	matbeg = 0;
	for (int i = 0; i < totalPotentialNodes; i++)
	{
		for (int j = 0; j < totalPotentialNodes; j++)
		{
			//if (i != v && !(v < n && i < n)) //condizione più generale, include le fijv di nodi tra cui non ci sono link
			//( !areOutOfSight(i,j) && u[i][j]>0 )
			//if (i != j && graph[i][j] != 0)
			//if (i != j && (!areOutOfSight(i, j) && u[i][j]>0))
			//if (i != j && graph[i][j] != 0)
			if(i != j) //***
			{
				for (int k = 0; k < K; k++)
				{
					if (fMap[i][j][k] != -1) //***se la variabile esiste o è stata trascurata
					{
						idx.push_back(fMap[i][j][k]);
						coef.push_back(1.0);
					}
				}

				//add -w_i_j
				if (idx.size() > 0) //***
				{
					idx.push_back(wMap[i][j]);
					coef.push_back(-1.0);

					snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
					char* rowname = (char*)(&name[0]);
					rowNumber++;

					CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, coef.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
					idx.clear();
					coef.clear();
				}
			}
		}
	}
	cout << "Vincoli (1) creati\n" << endl;
	
	// 2. conservazione flusso
	sense = 'E';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int v = 0; v < totalPotentialNodes; v++)
	{
		for (int k = 0; k < K; k++)
		{
			for (int i = 0; i < totalPotentialNodes; i++)
			{
				//if (i != v && graph[i][v] != 0)
				//if (i != v && !(v < n && i < n)) //condizione più generale, include le fijk di nodi tra cui non ci sono link
				if(i!=v && fMap[i][v][k]!=-1) //*** a meno della presenza di nodi isolati, ci sarà per ogni vincolo almeno una coppia di f tc fmap[][][]!=-1
				{
					idx.push_back(fMap[i][v][k]);
					coef.push_back(1.0);
				}
			}

			for (int j = 0; j < totalPotentialNodes; j++)
			{
				//if (v != j && graph[v][j] != 0)
				if (v != j && fMap[v][j][k]!=-1) //***condizione più generale, include le fijv di nodi tra cui non ci sono link
				{
					idx.push_back(fMap[v][j][k]);
					coef.push_back(-1.0);
				}
			}

			snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;

			
			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &b[v][k], &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
			idx.clear();
			coef.clear();
			
		}
	}
	cout << "Vincoli (2) creati\n" << endl;
	
	// 3. capacita' effettiva
	sense = 'E';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int i = 0; i < totalPotentialNodes; i++)
	{
		for (int j = baseDroneIndex; j < totalPotentialNodes; j++)
		{
			if (i != j && wMap[i][j] != -1)//***
			{
				//add +w_i_j
				idx.push_back(wMap[i][j]);
				coef.push_back(1.0);

				//add -u_i_j*y_j
				idx.push_back(y_index + (j - baseDroneIndex));
				coef.push_back(u[i][j] * (-1));

				snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
				char* rowname = (char*)(&name[0]);
				rowNumber++;

				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
				idx.clear();
				coef.clear();
			}
		}
	}

	// 3.a 
	coef.clear();
	idx.clear();
	for (int j = baseDroneIndex; j < totalPotentialNodes; j++)
	{
		for (int i = 0; i < n; i++)
		{
			if (i != j && wMap[j][i] != -1) //***
			{
				//add +w_i_j
				idx.push_back(wMap[j][i]);
				coef.push_back(1.0);

				//add -u_i_j*y_j
				idx.push_back(y_index + (j - baseDroneIndex));
				coef.push_back(u[j][i] * (-1));

				snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
				char* rowname = (char*)(&name[0]);
				rowNumber++;

				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
				idx.clear();
				coef.clear();
			}
		}
	}
	cout << "Vincoli (3) creati\n" << endl;
	CHECKED_CPX_CALL(CPXwriteprob, env, lp, "vartest.lp", NULL);
	// 4. legame tra variabili u_i_j e x_i_j
	/*sense = 'L';
	matbeg = 0;
	for (int i = 0; i < totalPotentialNodes; i++)
	{
	int idx = 0;
	double coef = 0;
	for (int j = 0; j < totalPotentialNodes; j++)
	{
	if (i != j && graph[i][j] != 0)
	{
	idx = xMap[i][j];
	coef = 1.0;
	CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, 1, &u[i][j], &sense, &matbeg, &idx, &coef, NULL, NULL);
	}
	}
	}*/

	// 4. legame tra variabili x_i_j e y_j
	sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int j = n; j < P + n; j++)
	{
		for (int i = 0; i < totalPotentialNodes; i++)
		{
			//if (i != j && graph[i][j] != 0)
			if (i != j && xMap[i][j] != -1) //***
			{
				//add x_i_j
				idx.push_back(xMap[i][j]);
				coef.push_back(1.0);

				//add -y_j
				idx.push_back(y_index + (j - n));
				coef.push_back(-1.0);

				snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
				char* rowname = (char*)(&name[0]);
				rowNumber++;

				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
				idx.clear();
				coef.clear();
			}
		}
	}

	cout << "Vincoli (4) creati\n" << endl;

	// 6. ogni nodo deve essere in grado di connettersi ad almeno un drone
	/*sense = 'G';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int i = 0; i < n+d; i++)
	{
	for (int v = baseDroneIndex; v < n+d; v++)
	{
	if (i != v)
	{
	idx.push_back(xMap[i][v]);
	coef.push_back(1.0);
	}
	}
	CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &uno, &sense, &matbeg, &idx[0], &coef[0], NULL, NULL);
	idx.clear();
	coef.clear();
	}*/


	// 5. un drone non puo' mantenere piu' di s potenziali connessioni simultanee
	sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int v = baseDroneIndex; v < totalPotentialNodes; v++)
	{
		double temp_s = (double) s;
		for (int i = 0; i < totalPotentialNodes; i++)
		{
			//if (i != v)
			if(xMap[i][v] != -1) //***
			{
				idx.push_back(xMap[i][v]);
				coef.push_back(1.0);
			}
		}
		
		if (idx.size() > 0) //***
		{

			snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;

			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &temp_s, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
			idx.clear();
			coef.clear();
		}
	}
	cout << "Vincoli (5) creati\n" << endl;


	// 6. non posizionare piu' di d droni
	double temp_d = (double)d;
	sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int i = 0; i < P; i++)
	{
		idx.push_back(y_index + i);
		coef.push_back(1.0);
	}
	
	snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
	char* rowname = (char*)(&name[0]);
	rowNumber++;

	CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &temp_d, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
	idx.clear();
	coef.clear();

	cout << "Vincoli (6) creati\n" << endl;

	// 7. legame tra variabili c_i_j_k e f_i_j_k
	//REWORKED
	sense = 'E';
	for (int i = 0; i < totalPotentialNodes; i++)
	{
		for (int j = 0; j < totalPotentialNodes; j++)
		{
			//TODO: nota, se si creano solo le vars fijk tc graph[i][j]!=0, questo vincolo perde significato e non genera nuove righe
			if (i != j && !(j < n && i < n)) //esclude i link i cui estremi sono entrambi nodi utenti 
			{
				for (int k = 0; k < K; k++)
				{
					if (c[i][j][k] > threshold && fMap[i][j][k]!=-1) //***
					{
						double coef = 1.0;
						int idx = fMap[i][j][k];

						snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
						char* rowname = (char*)(&name[0]);
						rowNumber++;

						CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, 1, &zero, &sense, &matbeg, &idx, &coef, NULL, &rowname);
					}
				}
			}
		}
	}
	cout << "Vincoli (7) creati\n" << endl;

	//remove duplicate
	/*for (int i = baseDroneIndex; i < n+d; i++)
	{
	for (int j = 0; j < totalPotentialNodes; j++)
	{
	if (i != j)
	{
	for (int k = 0; k < K; k++)
	{
	if (c[i][j][k] > threshold)
	{
	double coef = 1.0;
	int idx = fMap[i][j][k];
	CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, 1, &zero, &sense, &matbeg, &idx, &coef, NULL, NULL);

	}
	}
	}
	}
	}*/



	/*sense = 'L';
	matbeg = 0;
	for (int i = 0; i < n; i++)
	{
	for (int j = n; j < totalPotentialNodes; j++)
	{
	if (i != j)
	{
	double coef = c[i][j][0] - threshold;
	int idx = xMap[i][j];
	CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, 1, &zero, &sense, &matbeg, &idx, &coef, NULL, NULL);

	}
	}
	}

	for (int i = baseDroneIndex; i < totalPotentialNodes; i++)
	{
	for (int j = 0; j < totalPotentialNodes; j++)
	{
	if (i != j)
	{
	double coef = c[i][j][0] - threshold;
	int idx = xMap[i][j];
	CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, 1, &zero, &sense, &matbeg, &idx, &coef, NULL, NULL);

	}
	}
	}*/
	/*
	// 11. in ogni posizione potenziale puo' esserci solo un drone
	sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int i = 0; i < P; i++)
	{
	for (int v = 0; v < d; v++)
	{
	idx.push_back(y_index + (d*i) + v);
	coef.push_back(1.0);
	}
	CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &uno, &sense, &matbeg, &idx[0], &coef[0], NULL, NULL);
	idx.clear();
	coef.clear();
	}

	// 12. un drone può essere dislocato in una sola posizione potenziale
	sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int v = 0; v < d; v++)
	{
	for (int i = 0; i < P; i++)
	{
	idx.push_back(y_index + (d*i) + v);
	coef.push_back(1.0);
	}
	CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &uno, &sense, &matbeg, &idx[0], &coef[0], NULL, NULL);
	idx.clear();
	coef.clear();
	}
	*/
	// 8. legame tra le variabili x_i_j e f_i_j_k
	//REWORKED
	sense = 'L';
	matbeg = 0;
	idx.clear();
	coef.clear();
	for (int i = 0; i < totalPotentialNodes; i++)
	{
		for (int j = 0; j < totalPotentialNodes; j++)
		{
			//if (i != j) //***
			//{
				//if (!(j < n && i < n)) //esclude i link i cui estremi sono entrambi nodi utenti //***
				//{
					//cout << i << " " << j << " passa" << endl;
					for (int k = 0; k < K; k++)
					{
						if (fMap[i][j][k] != -1) //***
						{


							idx.push_back(fMap[i][j][k]);
							coef.push_back(1.0);

							// f_i_j_k - M * x_i_j
							idx.push_back(xMap[i][j]);
							coef.push_back(-bigM);

							snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
							char* rowname = (char*)(&name[0]);
							rowNumber++;

							CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
							idx.clear();
							coef.clear();
						}
					}
				//}
			//}
		}
	}
	cout << "Vincoli (8) creati\n" << endl;
}

void saveInstance(const char *filename)
{
	ofstream file;
	file.open(filename, ios::out);
	if (file.is_open())
	{
		file << n << endl;
		file << d << endl;
		file << P << endl;
		for (int i = 0; i < P; i++)
		{
			file << deployCost[i] << " ";
		}
		file << endl;

		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				file << t[i][j] << " ";
			}
			file << endl;
		}

		for (int i = 0; i < totalPotentialNodes; i++)
		{
			for (int j = 0; j < totalPotentialNodes; j++)
			{
				file << u[i][j] << " ";
			}
			file << endl;
		}

		for (int i = 0; i < totalPotentialNodes; i++)
		{
			for (int j = 0; j < totalPotentialNodes; j++)
			{
				for (int k = 0; k < K; k++)
				{
					file << c[i][j][k] << " ";
				}
				file << endl;
			}
		}

		file.close();
	}
	else
		cout << "Impossibile aprire il file: " << filename << endl;
}

int initDataStructures(int n, int d, int P)
{
	int success = 0;
	totalNodes = n + d;
	totalPotentialNodes = n + P;
	K = n*(n - 1);

	try
	{
		//matrice nxn, init=-1
		t.resize(n);
		for (int i = 0; i < n; i++)
		{
			t[i].resize(n, -1);
		}

		//vettore P, init=-1
		deployCost.resize(P, -1);

		//matrice 3D (n+P)x(n+P)x(K), init=DISCONNECTED_COST
		c.resize(totalPotentialNodes);
		for (int i = 0; i < totalPotentialNodes; i++)
		{
			c[i].resize(totalPotentialNodes);
			for (int j = 0; j < totalPotentialNodes; j++)
			{
				c[i][j].resize(K, DISCONNECTED_COST);
			}
		}

		//matrice (n+P)x(n+P), init=-1
		u.resize(totalPotentialNodes);
		for (int i = 0; i < totalPotentialNodes; i++)
		{
			u[i].resize(totalPotentialNodes, -1);
		}

		//matrice (n+P)x(K), init=0
		b.resize(totalPotentialNodes);
		for (int i = 0; i < totalPotentialNodes; i++)
		{
			b[i].resize(K, 0);
		}
	}
	catch (exception &e)
	{
		cout << "initDataSructures(): An exception occurred: " << e.what() << endl;
		success = 1;
	}
	return success;
}

int loadInstance(const char *filename) //TODO: controlli su possibile istanza corrotta (getline)
{
	int result = 0;
	ifstream file;

	n = 0, d = 0, P = 0;

	file.open(filename, ios::in);
	if (file.is_open())
	{
		try
		{
			file >> n >> d >> P;
			if (n > 0 && d > 0 && P > 0)
			{
				if (initDataStructures(n, d, P) == 0)
				{
					for (int i = 0; i < P; i++)
					{
						file >> deployCost[i];
					}

					for (int i = 0; i < n; i++)
					{
						for (int j = 0; j < n; j++)
						{
							file >> t[i][j];
						}
					}

					for (int i = 0; i < totalPotentialNodes; i++)
					{
						for (int j = 0; j < totalPotentialNodes; j++)
						{
							file >> u[i][j];
						}
					}

					for (int i = 0; i < totalPotentialNodes; i++)
					{
						for (int j = 0; j < totalPotentialNodes; j++)
						{
							for (int k = 0; k < K; k++)
							{
								file >> c[i][j][k];
							}
						}
					}
				}
				else
				{
					cout << "loadInstance(): allocazione strutture fallita." << endl;
					result = 1;
				}
			}
			else
			{
				cout << "loadInstance(): Parametri dell'istanza " << filename << " errati." << endl;
				result = 1;
			}
		}
		catch (exception &e)
		{
			cout << "loadInstance(): errore nella lettura del file " << filename << ": " << e.what() << endl;
			result = 1;
		}
		file.close();
	}
	else
	{
		cout << "loadInstance(): Impossibile aprire il file " << filename << endl;
		result = 1;
	}
	return result;
}

//costruisce la matrice b partendo dalla matrice di traffico 
//la matrice b e' pre-inizializzata al valore 0
void convertIntoBMatrix()
{
	int k = 0; //commodity counter
	for (unsigned int i = 0; i < t.size(); i++)
	{
		for (unsigned int j = 0; j < t.size(); j++)
		{
			if (i != j)
			{
				//copia i valori dalla matrice t alla b. I valori vengono resi negativi perche' il traffico e'"uscente" dai nodi
				b[i][k] = t[i][j] * (-1);

				//il traffico uscente dal nodo i al nodo j sara' visto come traffico entrante dal nodo j
				b[j][k] = t[i][j];
				k++;
			}
		}
	}
}


//test sui file delle soluzioni
int testSolutionFile(const char *reference, const char *test)
{
	ifstream refFile, tstFile;
	int buf = 128, rowCount = 1, result = 0;
	refFile.open(reference, ios::in);
	tstFile.open(test, ios::in);
	if (refFile.is_open() && tstFile.is_open())
	{
		cout << "Verifica file:" << endl;
		char str1[buf], str2[buf];
		while (!refFile.eof() && !tstFile.eof())
		{
			refFile.getline(str1, buf, '\n');
			tstFile.getline(str2, buf, '\n');
			if (strcmp(str1, str2) != 0)
			{
				cout << "diff in row " << rowCount << ": " << str1 << " " << str2 << endl;
				result = 1;
			}
			rowCount++;
		}
		refFile.close();
		tstFile.close();
	}
	else
	{
		cout << "Impossibile aprire i file: " << reference << " e/o " << test << endl;
		result = -1;
	}
	if (result == 0)
		cout << "File identici" << endl;
	return result;
}


//generatore pseudo-casuale di numeri double nel range inf - sup
/*double getdRand(double inf, double sup)
{
	double drand01 = ((double)rand()) / ((double)RAND_MAX); //drand(0,1)
	return (drand01 * (sup - inf) + inf); //drand(inf,sup)
}*/

/*
string getStatus(int code)
{
	switch (code)
	{
	case -1:
		return "CPX_CONFLICT_EXCLUDED";
		break;
	case 1:
		return "CPX_CONFLICT_POSSIBLE_LB";
		break;
	case 2:
		return "CPX_CONFLICT_POSSIBLE_UB";
		break;
	case 3:
		return "CPX_CONFLICT_MEMBER";
		break;
	case 4:
		return "CPX_CONFLICT_LB";
		break;
	case 5:
		return "CPX_CONFLICT_UB";
		break;
	default:
		return "unknown";
		break;
	}
}*/

void saveSolution(CEnv env, Prob lp, int yBegin, solution_t &solution)
{
	int status = 0;
	double objval=0;
	CHECKED_CPX_CALL(CPXgetobjval, env, lp, &objval);

	solution.objValue = objval;
	
	solution.yPositions.resize(P);

	CHECKED_CPX_CALL(CPXgetx, env, lp, &(solution.yPositions[0]), yBegin, yBegin + P);
	
}

void printSolution(solution_t solution)
{
	int count = 0;
	cout << endl << "Soluzione istanza: " << endl;
	cout << "Valore f. obiettivo: " << solution.objValue << endl;
	for (unsigned int i = 0; i < solution.yPositions.size(); i++)
	{
		if (solution.yPositions[i] == 1)
			count++;
	}
	cout << "Droni impiegati/droni totali: " << count << "/" << d << endl;

	for (unsigned int i = 0; i < solution.yPositions.size(); i++)
	{
		cout << "y_" << i + n << ": " << (int)solution.yPositions[i] << endl;
	}
	cout << endl;

}

void printVarsValue(CEnv env, Prob lp)
{
	char **cur_colname = new char *[1];
	int cur_storespace = 16;
	char *cur_colnamestore = new char[cur_storespace];
	int *surplus = new int[1];

	vector<double> varVals;

	//print solution (var values)
	int n = CPXgetnumcols(env, lp);

	varVals.resize(n);

	cout << "Visualizzazione delle variabili: " << endl;
	CHECKED_CPX_CALL(CPXgetx, env, lp, &varVals[0], 0, n - 1);
	for (int i = 0; i < n; i++)
	{
		if (varVals[i] != 0)
		{
			status = CPXgetcolname(env, lp, cur_colname, cur_colnamestore, cur_storespace, surplus, i, i);
			if (status == 0)
				cout << cur_colnamestore << " : " << varVals[i] << endl;
			else
				cout << "Var in position " << i << " : " << varVals[i] << endl;
		}
	}

	delete[] surplus;
	delete[] cur_colnamestore;
	delete[] cur_colname;
}



void printSimplifiedSolFile(CEnv env, Prob lp, const char* solution)
{
	char **cur_colname = new char *[1];
	int cur_storespace = 16;
	char *cur_colnamestore = new char[cur_storespace];
	int *surplus = new int[1];
	double objval = 0;
	vector<double> varVals;

	//print unit test file
	ofstream unitFile;
	unitFile.open(solution, ios::out);
	if (unitFile.is_open()) //TODO: eliminare condizione
	{
		cout << "File " << solution << " creato." << endl;

		// print objval
		CHECKED_CPX_CALL(CPXgetobjval, env, lp, &objval);
		unitFile << "Objval: " << objval << endl;

		//print solution (var values)
		int n = CPXgetnumcols(env, lp);

		varVals.resize(n);

		CHECKED_CPX_CALL(CPXgetx, env, lp, &varVals[0], 0, n - 1);
		/// status = CPXgetx (env, lp, x, 0, CPXgetnumcols(env, lp)-1);
		for (int i = 0; i < n; i++)
		{
			if (varVals[i] != 0)
			{
				//
				/// to get variable name, use the RATHER TRICKY "CPXgetcolname"
				status = CPXgetcolname(env, lp, cur_colname, cur_colnamestore, cur_storespace, surplus, i, i);
				if (status == 0)
				{
					unitFile << cur_colnamestore << " : " << varVals[i] << endl;
				}
				else
				{
					unitFile << "Var in position " << i << " : " << varVals[i] << endl;
				}
			}
		}

		delete[] surplus;
		delete[] cur_colnamestore;
		delete[] cur_colname;
		
	}
	unitFile.close();
}

void setCompleteGraph(double stdCost, int stdCap)
{
	for(int i=n; i< totalPotentialNodes; i++)
	{
		for(int j=n; j< totalPotentialNodes; j++)
		{
			if(i!=j)
			{
				for(int k=0;k<K ;k++)
				{
					c[i][j][k]= stdCost;
				}
				u[i][j]=stdCap;
			}
		} 
	} 
}

int getSolutionFlowMatrix(CEnv env, Prob lp, vector< vector< vector<int> > > &flow)
{
	char **cur_colname = new char *[1];
	int cur_storespace = 16;
	char *cur_colnamestore = new char[cur_storespace];
	int *surplus = new int[1];
	vector<double> varVals;
	char *tokens;

	int indexes[3];
	int status = 0;
	//print solution (var values)
	int n = CPXgetnumcols(env, lp);

	varVals.resize(n);

	CHECKED_CPX_CALL(CPXgetx, env, lp, &varVals[0], 0, n - 1);
	int v = 0;
	while(v < n && status==0)
	{
		/// to get variable name, use the RATHER TRICKY "CPXgetcolname"
		status = CPXgetcolname(env, lp, cur_colname, cur_colnamestore, cur_storespace, surplus, v, v);
		if (status == 0)
		{
			if(cur_colnamestore[0]!='f')
			{
				status=1;
			}
			else
			{
				int i = 0;
				//cout<< "stringa: " << cur_colnamestore<<endl;
				tokens = strtok(cur_colnamestore, "f_");
				if(tokens!=NULL)
					indexes[i] = atoi(tokens);
				//cout << "token trovato: " << tokens << endl;
				
				while (tokens != NULL)
				{
					tokens = strtok(NULL, "f_");
					if(tokens!=NULL)
					{
						i++;
						//cout << "token trovato: " << tokens << endl;
						indexes[i] = atoi(tokens);
						
					}
				}
				//cout<< "indici:\n"<< indexes[0] <<" "<< indexes[1] <<" "<< indexes[2]<<endl;


				if (indexes[0] >= 0 && indexes[0] < totalPotentialNodes && indexes[1] >= 0 && indexes[1] < totalPotentialNodes && indexes[2] >= 0 && indexes[2] < K)
				{
					flow[indexes[0]][indexes[1]][indexes[2]] = varVals[v];
				}
				else
				{
					status = 1;
				}
			}
		}
		
		v++;
	}
	return status;
}

void setCostsByRadius(int xPos, int yPos, int xMax, int yMax, int radius, float cost)
{
	int yStart, yEnd, xStart, xEnd;
	
	if(xPos-radius <0)
		xStart= xPos;
	else
		xStart= xPos-radius;

	if(yPos-radius <0)
		yStart= yPos;
	else
		yStart= yPos-radius;

	if(xPos+radius >xMax)
		xEnd= xPos;
	else
		xEnd= xPos+radius;

	if(yPos+radius >yMax)
		yEnd= yPos;
	else
		yEnd= yPos+radius;

	for(int y= yStart; y<= yEnd ; y++)
	{
		for(int x=xStart; x<= xEnd;x++)
		{
			for(int k=0;k<K;k++)
			{
				c[x][y][k]= cost;
			}
		}

	}
}

void printNetworkUsage(vector< vector< vector<int> > > &flow)
{
	for(unsigned int i=0;i<flow.size();i++)
	{
		for(unsigned int j=0;j<flow[0].size();j++)
		{
			int acc=0;
			for(unsigned int k=0;k<flow[0][0].size();k++)
			{
				if(flow[i][j][k]>0)
					acc+=flow[i][j][k];
			}
			if(acc>0)
				cout << "Flusso sul link ("<<i<<","<<j<<"): "<<acc<<"/"<<u[i][j]<<endl;
		}
	}
}

void printNodesIOFlow()
{
	int accIN = 0, accOUT = 0;
	for (int i = 0; i < n; i++)
	{
		accIN = 0, accOUT = 0;
		cout << i << ")";
		for (int j = 0; j < n; j++)
		{
			if (i != j)
			{
				accOUT += t[i][j];
				accIN += t[j][i];
			}
		}
		cout << "Out= " << accOUT << " In= " << accIN << " Tot= " << accOUT + accIN << endl;
	}
}
 
int main(int argc, char const *argv[])
{
	string lpFile("flow2.lp"), solution("solution2.txt"), instance("test2.txt"), solFile("flow2.sol"), clpFile("conflict.clp");

	solution_t instSolution;

	//dichiarazione dei timepoint/epoch (strutture dati in cui porre i tempi di inizio e fine dell'esecuzione di ogni istanza) 
	high_resolution_clock::time_point start, end;

	srand(time(NULL)); /* seed random number generator */

	loadInstance(instance.c_str());

	

	cout << "Instance " << instance << " loaded." << endl;
	cout << "Users: " << n << endl;
	cout << "Drones: " << d << endl;
	cout << "Potential drones positions: " << P << endl << endl;
	convertIntoBMatrix();
	//buildAdjMatrix(threshold);

	printf("\nb matrix:\n");
	for (int i = 0; i < totalPotentialNodes; i++)
	{
		for (int j = 0; j < K; j++)
		{
			printf("%.0f  ", b[i][j]);
		}
		printf("\n");
	}
/*
	for (int k = 0; k < K; k++)
	{
		c[0][6][k] = 10;
		c[6][0][k] = 10;

		c[0][12][k] = 10;
		c[12][0][k] = 10;

		c[1][6][k] = 10;
		c[6][1][k] = 10;

		c[1][12][k] = 10;
		c[12][1][k] = 10;

		c[1][18][k] = 10;
		c[18][1][k] = 10;
		
		c[2][24][k] = 10;
		c[24][2][k] = 10;

		c[2][25][k] = 10;
		c[25][2][k] = 10;
		
		c[2][26][k] = 10;
		c[26][2][k] = 10;
		
		c[3][28][k] = 10;
		c[28][3][k] = 10;

		c[3][29][k] = 10;
		c[29][3][k] = 10;

		c[4][29][k] = 10;
		c[29][4][k] = 10;
		
		c[4][23][k] = 10;
		c[23][4][k] = 10;
		
		c[4][17][k] = 10;
		c[17][4][k] = 10;
		
		c[4][11][k] = 10;
		c[11][4][k] = 10;
		
		c[5][11][k] = 10;
		c[11][5][k] = 10;
		
	}

		u[0][6]= 300;
		u[6][0]= 300;

		u[0][12] = 300;
		u[12][0] = 300;

		u[1][6]= 130;
		u[6][1]= 130;

		u[1][12] = 300;
		u[12][1] = 300;

		u[1][18] = 300;
		u[18][1] = 300;
		
		u[2][24] = 300;
		u[24][2] = 300;

		u[2][25] = 300;
		u[25][2] = 300;
		
		u[2][26] = 300;
		u[26][2] = 300;
		
		u[3][28] = 300;
		u[28][3] = 300;

		u[3][29] = 300;
		u[29][3] = 300;

		u[4][29] = 300;
		u[29][4] = 300;
		
		u[4][23] = 300;
		u[23][4] = 300;
		
		u[4][17] = 300;
		u[17][4] = 300;
		
		u[4][11] = 300;
		u[11][4] = 300;
		
		u[5][11] = 300;
		u[11][5] = 300;
*/

	//setCompleteGraph(10.0,300);
	//randomizeTraffic(0,50);

	saveInstance(instance.c_str());
	printNodesIOFlow();
	try
	{
		// init
		DECL_ENV(env);
		DECL_PROB(env, lp);

		CPXsetintparam(env, CPX_PARAM_CONFLICTDISPLAY, 2);

		// setup LP
		setupLP(env, lp);
		CHECKED_CPX_CALL(CPXwriteprob, env, lp, lpFile.c_str(), NULL); 
		// optimize

		start = high_resolution_clock::now(); //timestamp di inizio
			CHECKED_CPX_CALL(CPXmipopt, env, lp);
		end = high_resolution_clock::now(); //timestamp di fine

		std::cout << "Istanza eseguita in "
			<< duration_cast<milliseconds>(end - start).count() //casting per la conversione in millisecondi; (end-start).count() calcola la differenza di tempo e la restituisce come valore numerico
			<< "ms" << " (" << duration_cast<seconds>(end - start).count() << "s c.ca).\n";
		cout << "--------------------------------------------" << endl;

		status = CPXgetstat(env, lp);

		cout << "status code: " << status << endl;

		if (status == 103) //CPXMIP_INFEASIBLE 
		{
			// infeasibility detected
			int confnumrows = 0, confnumcols = 0;

			cout << "CPXMIP_INFEASIBLE: infeasibility detected." << endl;

			status = CPXrefineconflict(env, lp, &confnumrows, &confnumcols);
			cout << "Number of conflicting rows: " << confnumrows << endl;
			cout << "Number of conflicting columns: " << confnumcols << endl;

			status = CPXclpwrite(env, lp, clpFile.c_str());
			if (status == 0)
				cout << "Conflict file " << clpFile << " created." << endl;
			else
				cout << "Failed to create " << clpFile << " conflict file." << endl;

		}

		saveSolution(env, lp, y_index, instSolution);
		printSolution(instSolution);

		printSimplifiedSolFile(env, lp, solution.c_str());
		printVarsValue(env,lp);

		vector< vector< vector<int> > > flow(totalPotentialNodes, vector< vector<int> >(totalPotentialNodes, vector<int>(K, -1)));
		getSolutionFlowMatrix(env,lp,flow);
		printNetworkUsage(flow);
		testSolutionFile("refIST3.txt", solution.c_str());

		//vector<double> y(P, 0);
		//status = CPXgetx(env, lp, y, y_index, y_index+P);

		CHECKED_CPX_CALL(CPXsolwrite, env, lp, solFile.c_str());
		// free
		CPXfreeprob(env, &lp);
		CPXcloseCPLEX(&env);
	}
	catch (exception& e)
	{
		cout << ">>>EXCEPTION: " << e.what() << endl;
	}
	return 0;
}
