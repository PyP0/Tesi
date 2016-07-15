/**
* @file modelv0.4.cpp
* @brief
*/

#include <cstdio>
#include <iostream>
#include <vector>
#include <fstream>
#include "cpxmacro.h"

#define DISCONNECTED_COST CPX_INFBOUND

using namespace std;

// error status and messagge buffer
int status;
char errmsg[BUF_SIZE];

// data

int n = 4; //numero di utenti
int d = 4; // numero di droni
int P = 4; //numero di posizioni potenziali dei droni
int K = n*(n - 1); //numero di commodities, ovvero numero di coppie distinte sorgente-destinazione
int s = 10; //numero massimo connessioni sostenibili da un drone

const int bigM = 1000; // TODO: bigM deve essere maggiore della capacità massima dei link
const int threshold = 100;

//int t[n][n];
//matrice di traffico
vector< vector<int> > t(n, vector<int>(n,-1)); //matrice di traffico


//int graph[n+d][n+d];
//matrice di adiacenza del grafo
vector< vector<int> > graph(n+d, vector<int>(n+d, 0)); 


//double c[n+d][n+d][K];
//matrice dei costi
vector< vector< vector<double> > > c(n + d, vector< vector<double> >(n + d, vector<double>(K, DISCONNECTED_COST )));

//matrice di capacità degli archi
vector< vector<double> > u(n + d, vector<double>(n + d));

//matrice bilanciamento flussi
vector< vector<double> > b(n+d, vector<double>(K, 0));

const int NAME_SIZE = 512;
char name[NAME_SIZE];


const double zero = 0.0;
const double uno = 1.0;



void setupLP(CEnv env, Prob lp, int *numVars)
{
	//allocazione mappe
	vector< vector< vector<int> > > fMap(n+d, vector< vector<int> >(n+d, vector<int>(K, -1)));
	vector< vector<int> > xMap(n + d, vector<int>(n + d, -1));
	vector< vector<int> > yMap(P, vector<int>(d, -1));

	int baseDroneIndex = n;

	//aggiunta delle variabili f_i_j_k, una per volta, con i!=j
	//nota: la variabile f_i_j_k e' distinta dalla variabile f_j_i_k
	int fMapIndex = 0;
	for (int k = 0; k < K; k++)
	{
		for (int i = 0; i < n+d; i++)
		{
			for (int j = 0; j < n+d; j++)
			{
				if (i != j && graph[i][j] != 0) //c'è un arco tra i nodi i e j, quindi creo la corrispondente variabile di flusso fijk
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

	//inserimento delle variabili y_i_v
	int y_index= CPXgetnumcols(env, lp);
	int yMapIndex = y_index;
	for (int i = 0; i < P; i++)
	{
		for (int v = 0; v < d; v++)
		{
			char ytype = 'B';
			double lb = 0.0;
			double ub = 1.0;

			snprintf(name, NAME_SIZE, "y_%d_%d", baseDroneIndex+i, v); //scrive il nome della variabile y_i_v sulla stringa name[]
			char* yname = (char*)(&name[0]);
			CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &zero, &lb, &ub, &ytype, &yname);  

			yMap[i][v] = yMapIndex;
			yMapIndex++;
		}
	}
	printf("Sono state create %d variabili y_i_v\n", CPXgetnumcols(env, lp)-y_index); //numero totale delle vars create	

	//aggiunta delle variabili xiv
		
	//si divide in due step la creazione delle variabili x_i_j

	//variabili relative agli "utenti"
	int x_index = CPXgetnumcols(env, lp);
	int xMapIndex = x_index;

	for (int i = 0; i < n; i++)
	{
		//esclude la creazione delle variabili con i==j e quelle che esprimono link tra due utenti
		for (int j = baseDroneIndex; j < n + d; j++) 
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

	//variabili relative ai "droni"
	for (int i = baseDroneIndex; i < n + d; i++)
	{
		for (int j = 0; j < n + d; j++)
		{
			//esclude le variabili x_i_j con i==j
			if (i != j)
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

	printf("Sono state create %d variabili x_i_j\n", CPXgetnumcols(env, lp)-x_index); //numero totale delle vars create	

	//vincoli
	
	char sense;
	int matbeg;
	vector<int> idx;
	vector<double> coef;

	// 1. capacità dei link
	sense = 'L';
	matbeg = 0;
	for (int i = 0; i < n+d; i++)
	{
		for (int j = 0; j < n+d; j++)
		{
			if (i != j && graph[i][j] != 0)
			{
				for (int k = 0; k < K; k++)
				{
					idx.push_back(fMap[i][j][k]);
					coef.push_back(1.0);
				}

				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, coef.size(), &u[i][j], &sense, &matbeg, &idx[0], &coef[0], NULL, NULL);
				idx.clear();
				coef.clear();
			}
		}
	}

	// 2. conservazione flusso
	sense = 'E';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int v = 0; v < n+d; v++)
	{
		for (int k = 0; k < K; k++)
		{
			for (int i = 0; i < n+d; i++)
			{
				if (i != v && graph[i][v] != 0)
				{
					idx.push_back(fMap[i][v][k]);
					coef.push_back(1.0);
				}
			}

			for (int j = 0; j < n+d; j++)
			{
				if (v != j && graph[v][j] != 0)
				{
					idx.push_back(fMap[v][j][k]);
					coef.push_back(-1.0);
				}
			}
			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &b[v][k], &sense, &matbeg, &idx[0], &coef[0], NULL, NULL);
			idx.clear();
			coef.clear();
		}
	}

	// 3. capacita' effettiva

	// 4. legame tra variabili u_i_j e x_i_j
	sense = 'L';
	matbeg = 0;
	for (int i = 0; i < n + d; i++)
	{
		int idx = 0;
		double coef = 0;
		for (int j = 0; j < n + d; j++)
		{
			if (i != j && graph[i][j] != 0)
			{
				idx = xMap[i][j];
				coef = 1.0;
				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, 1, &u[i][j], &sense, &matbeg, &idx, &coef, NULL, NULL);
			}
		}
	}

	// 5. legame tra variabili x_i_j e y_j_v
	sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int j = n; j < P+n; j++)
	{
		for (int i = 0; i < n + d; i++)
		{
				if (i != j && graph[i][j] != 0)
				{
					idx.push_back(xMap[i][j]);
					coef.push_back(1.0);
					for (int v = 0; v < d; v++)
					{
						idx.push_back(y_index + ((j-n)*d) + v);
						coef.push_back(-1.0);
					}
					CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, NULL);
					idx.clear();
					coef.clear();
				}
				
		}
		
	}

	// 6. ogni nodo deve essere in grado di connettersi ad almeno un drone
	sense = 'G';
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
	}


	// 7. un drone non puo' mantenere piu' di s potenziali connessioni simultanee
	sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int v = baseDroneIndex; v < n+d; v++)
	{
		for (int i = 0; i < n+d; i++)
		{
			if (i != v)
			{
				idx.push_back(xMap[i][v]);
				coef.push_back(1.0);
			}
		}
		double temp_s = s;
		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &temp_s, &sense, &matbeg, &idx[0], &coef[0], NULL, NULL);
		idx.clear();
		coef.clear();
	}

	// 9. non posizionare piu' di d droni
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
	}
	double temp_d = d;
	CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &temp_d, &sense, &matbeg, &idx[0], &coef[0], NULL, NULL);
	idx.clear();
	coef.clear();

	// 10. legame tra variabili c_i_j_k e x_i_j
	sense = 'L';
	matbeg = 0;
	for (int i = 0; i < n; i++)
	{
		for (int j = n; j < n + d; j++)
		{
			if (i != j)
			{
				double coef = c[i][j][0] - threshold;
				int idx = xMap[i][j];
				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, 1, &zero, &sense, &matbeg, &idx, &coef, NULL, NULL);

			}
		}
	}

	for (int i = baseDroneIndex; i < n + d; i++)
	{
		for (int j = 0; j < n + d; j++)
		{
			if (i != j)
			{
				double coef = c[i][j][0] - threshold;
				int idx = xMap[i][j];
				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, 1, &zero, &sense, &matbeg, &idx, &coef, NULL, NULL);

			}
		}
	}

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

	// 13. legame tra le variabili x_i_j e f_i_j_k
	sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();

	for (int i = 0; i < n + d; i++)
	{
		for (int j = 0; j < n + d; j++)
		{
			if (i != j && graph[i][j] != 0)
			{
				for (int k = 0; k < K; k++)
				{
					idx.push_back(fMap[i][j][k]);
					coef.push_back(1.0);
				}
				// sum(f_i_j_k) - M * x_i_j
				idx.push_back(xMap[i][j]);
				coef.push_back(-bigM);
				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, NULL);
				idx.clear();
				coef.clear();
			}
		}
	}

	CHECKED_CPX_CALL(CPXwriteprob, env, lp, "flow.lp", NULL);

}

void saveInstance(char *filename)
{
	ofstream file;
	file.open(filename, ios::out);
	if (file.is_open())
	{
		file << n << endl;
		file << d << endl;
		file << P << endl;
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				file << t[i][j] << " ";
			}
			file << endl;
		}

		for (int i = 0; i < n + d; i++)
		{
			for (int j = 0; j < n + d; j++)
			{
				file << u[i][j] << " ";
			}
			file << endl;
		}

		for (int i = 0; i < n + d; i++)
		{
			for (int j = 0; j < n + d; j++)
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

void loadInstance(char *filename)
{
	ifstream file;
	file.open(filename, ios::in);
	if (file.is_open())
	{
		file >> n >> d >> P;
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				file >> t[i][j];
			}
		}

		for (int i = 0; i < n+d; i++)
		{
			for (int j = 0; j < n+d; j++)
			{
				file >> u[i][j];
			}
		}

		for (int i = 0; i < n+d; i++)
		{
			for (int j = 0; j < n+d; j++)
			{
				for (int k = 0; k < K; k++)
				{
					file >> c[i][j][k];
				}
			}
		}

		file.close();
	}
	else
		cout << "Impossibile aprire il file: " << filename << endl;
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

//partendo dalla matrice dei costi c si costruisce la matrice delle adiacenze graph
//ogni costo > threshold viene interpretato come infinito, cioe' come assenza dell'arco corrispondente
void buildAdjMatrix(double threshold)
{
	int count = 0;
	for (int i = 0; i < n + d; i++)
	{
		for (int j = 0; j < n + d; j++)
		{
			if (i != j)
			{
				for (int k = 0; k<K; k++)
				{
					if (c[i][j][k] > threshold)
					{
						count++;
					}
				}
				//se per almeno una commodity il costo non è INF, allora l'arco (i,j) esiste
				if (count != K)
				{
					graph[i][j] = 1;
				}
				count = 0;
			}
		}
	}
}


int main(int argc, char const *argv[])
{
	double objval = 0.0;
	int numVars = 0;

	//leaky init 
	/*for (int i = 0; i < n+d; i++)
	{
		for (int j = 0; j < n+d; j++)
		{
			u[i][j] = 20;
		}
	}

	//IST 1
	for (int k = 0; k < K; k++)
	{
		c[0][5][k] = 2;
		c[5][0][k] = 2;
		
		c[1][6][k] = 2;
		c[6][1][k] = 2;

		c[1][6][k] = 2;
		c[6][1][k] = 2;

		c[3][4][k] = 2;
		c[4][3][k] = 2;

		c[2][7][k] = 2;
		c[7][2][k] = 2;


		c[5][4][k] = 2;
		c[4][5][k] = 2;

		c[1][6][k] = 2;
		c[6][1][k] = 2;

		c[5][6][k] = 2;
		c[6][5][k] = 2;

		c[6][7][k] = 2;
		c[7][6][k] = 2;


	}

	t[0][1] = 6;
	t[0][2] = 10;
	t[0][3] = 2;

	t[1][0] = 1;
	t[1][2] = 0;
	t[1][3] = 4;

	t[2][0] = 0;
	t[2][1] = 5;
	t[2][3] = 0;

	t[3][0] = 1;
	t[3][1] = 0;
	t[3][2] = 0;*/

	loadInstance("test.txt");
	convertIntoBMatrix();
	buildAdjMatrix(threshold);
	
	printf("\nb matrix:\n");
	for (int i = 0; i < n + d; i++)
	{
		for (int j = 0; j < K; j++)
		{
			printf("%.0f ", b[i][j]);
		}
		printf("\n");
	}

	printf("\nArchi del grafo: \n");
	for (int i = 0; i < n + d; i++)
	{
		for (int j = 0; j < n + d; j++)
		{
			if (graph[i][j] != 0)
			{
				printf("arc from %d to %d \n", i, j);
			}
		}
	}

	//saveInstance("test.txt");
	try
	{
		// init
		DECL_ENV(env);
		DECL_PROB(env, lp);

		// setup LP
		setupLP(env, lp, &numVars);

		// optimize
		CHECKED_CPX_CALL(CPXmipopt, env, lp);

		// print objval
		CHECKED_CPX_CALL(CPXgetobjval, env, lp, &objval);
		cout << "Objval: " << objval << std::endl;

		//print solution (var values)
		int n = CPXgetnumcols(env, lp);

		vector<double> varVals;
		varVals.resize(n);
		char **cur_colname = new char *[1] ;
		int cur_storespace=16;
		char *cur_colnamestore = new char[cur_storespace];
		int *surplus = new int[1];



		CHECKED_CPX_CALL(CPXgetx, env, lp, &varVals[0], 0, n - 1);
		/// status = CPXgetx (env, lp, x, 0, CPXgetnumcols(env, lp)-1);
		for (int i = 0; i < n; i++)
		{
			if (varVals[i] != 0)
			{
				//
				/// to get variable name, use the RATHER TRICKY "CPXgetcolname"
				status = CPXgetcolname (env, lp, cur_colname, cur_colnamestore, cur_storespace, surplus, i, i);
				if(status ==0)
					cout << cur_colnamestore << " : " << varVals[i] << std::endl;
				else
					cout << "Var in position " << i << " : " << varVals[i] << std::endl;
			}
		}

		delete[] surplus;
		delete[] cur_colnamestore;
		delete[] cur_colname;
		CHECKED_CPX_CALL(CPXsolwrite, env, lp, "flow.sol");
		// free
		CPXfreeprob(env, &lp);
		CPXcloseCPLEX(&env);
	}
	catch (std::exception& e)
	{
		cout << ">>>EXCEPTION: " << e.what() << std::endl;
	}
	return 0;
}
