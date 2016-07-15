/**
* @file modelv0.9.1.cpp
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
	double *yPositions;
	double objValue;
};

//test coordinate discrete su griglia
struct nodesCoordinates_t
{
	int x;
	int y;
	int id;
};

struct instance_t
{
	int n;
	int d;
	int P;
	int totalNodes;
	int totalPotentialNodes;
	int K;
	int s;
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

int droneTXCapacity= 20000;
int droneRXCapacity= 10000;

//indici iniziali per la memorizzazione delle variabili di CPLEX
int f_index = 0;
int y_index = 0;
int x_index = 0;
int w_index = 0;
int z_index = 0;

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


//vector<nodesCoordinates_t> *createNodesStruct(int);
bool areOutOfSight(int,int);

double getdRand(double,double);
int getRand(int,int);
void randomizeTraffic(int,int);
void randomizeCosts(double,double,double, double);
void randomizeDeployCost(double, double);
void createRandomInstance(int,int,int,int,int,double,double,double,double,double,double,const char *);
void setCompleteGraph(double, int);
void applyDiscount(double);
void setCostsByRadius(int, int, int, int, int, float);
double getDistanceCoef(int, int);
bool isIntersection(double,double,double,double,double,double);
double getInterferenceFactor(int, int);


void setupLP(CEnv, Prob);

int saveInstance(const char *);
int initDataStructures(int, int, int);
int loadInstance(const char *);
void convertIntoBMatrix();


int testSolutionFile(const char *,const char *);
solution_t *saveSolution(CEnv, Prob, int);
void printSolution(solution_t);
void printVarsValue(CEnv, Prob);
void printSimplifiedSolFile(CEnv,Prob, const char*);
int getSolutionFlowMatrix(CEnv, Prob, vector< vector< vector<int> > > &);

void printNetworkUsage(vector< vector< vector<int> > > &);
void printNodesIOFlow();


vector< nodesCoordinates_t > createGrid(double,double,double);

//-----------------------------------------------------------------------------

/*vector<nodesCoordinates_t> *createNodesStruct(int size)
{
	vector<nodesCoordinates_t> *nodi = new vector<nodesCoordinates_t>(size);
	return nodi;
}*/

//alloca e popola un vettore per ospitare le coordinate di tutti i punti 
vector< nodesCoordinates_t > createGrid(double length, double height, double step)
{
	if(length >0 && height > 0 && step >0)
	{
		double xCoord=0.0, yCoord=0.0;

		vector< nodesCoordinates_t > grid(length*height); 
		int rows=0;
		int index=0;
		for(int a=0; a<height; a++)
		{
			xCoord=0.0;
			
			for(int b=0; b< length; b++)
			{
				grid[index].x = xCoord;
				xCoord+=step;

				grid[index].y = yCoord;
				index++;
			}
			rows++;
			yCoord+=step;
		}
		return grid;
	}
	else
	{
		cerr << __FUNCTION__ << "(): Uno o piu' parametri sono errati.";
		//return empty vector
		return vector< nodesCoordinates_t >();
	}
}

void printGrid(vector< nodesCoordinates_t > grid)
{
	if(grid.size() > 0)
	{
		for(unsigned int i=0; i<grid.size();i++)
		{
			cout << i << ": " << "(" << grid[i].x << ", " << grid[i].y << ")" << endl; 
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): Struttura dati vuota.";
	}
}


double getDistanceCoef(int i, int j)
{
	return 1.0; //TODO
}

double getInterferenceFactor(int i, int j)
{
	return 1.0; //TODO
}

double getDistance(int px, int py, int qx, int qy)
{
	return sqrt(pow(sqrt(px - qx), 2) + pow(sqrt(px - qx), 2));
}

bool isInRange(int px, int py, int centerx, int centery, int radius)
{
	if (abs(getDistance(px, py, centerx, centery)) <= radius)
		return true;
	else
		return false;
}

bool isIntersection(double x0, double y0, double r0, double x1, double y1, double r1)
{
	//(R0-R1)^2 <= (x0-x1)^2+(y0-y1)^2 <= (R0+R1)^2
	double diffCoords = pow(x0-x1,2) + pow(y0-y1,2);
	if (diffCoords >= pow(r0-r1,2) || diffCoords <= pow(r0+r1,2))
		return true;
	else
		return false;
}

bool areOutOfSight(int i, int j)
{
	int k = 0;
	bool flag = true;
	if (c.size() > 0) //c e' gia' stato allocato?
	{
		while (k < K && flag == true)
		{
			if (c[i][j][k] <= threshold)
				flag = false;
			k++;
		}
	}
	else
	{
		flag = false;
		cerr << __FUNCTION__ << "(): Struttura dati non allocata." << endl;
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
	if (t.size() > 0)
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
	else
	{
		cerr << __FUNCTION__ << "(): Struttura dati non allocata." << endl;
	}
}

void randomizeCosts(double minCVal, double maxCVal, double minUVal, double maxUVal)
{
	if (c.size() > 0 && u.size() > 0)
	{
		for (unsigned int i = 0; i < c.size(); i++)
		{
			for (unsigned int j = 0; j < c[0].size(); j++)
			{
				for (unsigned int k = 0; k< c[0][0].size(); k++)
				{
					if (i != j)
					{
						c[i][j][k] = getdRand(minCVal, maxCVal);
						if (c[i][j][k] > threshold)
						{
							u[i][j] = 0;
						}
						else
						{
							u[i][j] = getdRand(minUVal, maxUVal);
						}
					}
				}
			}
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): Strutture dati non allocate." << endl;
	}
}

void randomizeDeployCost(double minVal, double maxVal)
{
	if (deployCost.size() > 0)
	{
		for (unsigned int i = 0; i < deployCost.size(); i++)
		{
			deployCost[i] = getdRand(minVal, maxVal);
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): Strutture dati non allocate." << endl;
	}

}

void createRandomInstance(int users, int drones, int positions, int tInf, int tSup, double cInf, double cSup, double uInf, double uSup, double dInf, double dSup, const char *filename)
{
	int result=0;
	n=users;
	d=drones;
	P=positions;  
	result= initDataStructures(users,drones,positions);
	if(result==0)
	{
		randomizeTraffic(tInf,tSup);
		randomizeCosts(cInf,cSup,uInf,uSup);
		randomizeDeployCost(dInf,dSup);
		saveInstance(filename); 
	}
	else
	{
		cerr << __FUNCTION__ << "(): Impossibile creare la nuova istanza." << endl;
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

int saveInstance(const char *filename)
{
	int status = 0;
	if (n > 0 && d > 0 && P>0)
	{
		ofstream file;
		file.open(filename, ios::out);
		if (file.is_open())
		{
			file << n << endl;
			file << d << endl;
			file << P << endl;
			for (unsigned int i = 0; i < deployCost.size(); i++)
			{
				file << deployCost[i] << " ";
			}
			file << endl;

			for (unsigned int i = 0; i < t.size(); i++)
			{
				for (unsigned int j = 0; j < t[0].size(); j++)
				{
					file << t[i][j] << " ";
				}
				file << endl;
			}

			for (unsigned int i = 0; i < u.size(); i++)
			{
				for (unsigned int j = 0; j < u[0].size(); j++)
				{
					file << u[i][j] << " ";
				}
				file << endl;
			}

			for (unsigned int i = 0; i < c.size(); i++)
			{
				for (unsigned int j = 0; j < c[0].size(); j++)
				{
					for (unsigned int k = 0; k < c[0][0].size(); k++)
					{
						file << c[i][j][k] << " ";
					}
					file << endl;
				}
			}

			file.close();
		}
		else
		{
			cerr << __FUNCTION__ << "(): Impossibile aprire il file: " << filename << endl;
			status = 1;
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): Impossibile salvare l'istanza vuota. " << endl;
		status = 1;
	}
	return status;

}

int initDataStructures(int n, int d, int P)
{
	int success = 0;
	if (n > 0 && d > 0 && P > 0)
	{
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
		catch (exception& e)
		{
			cerr << __FUNCTION__ << "(): An exception occurred: " << e.what() << endl;
			success = 1;
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): Impossibile allocare strutture con dimensione 0." << endl;
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
					for (unsigned int i = 0; i < deployCost.size(); i++)
					{
						file >> deployCost[i];
					}

					for (unsigned int i = 0; i < t.size(); i++)
					{
						for (unsigned int j = 0; j < t[0].size(); j++)
						{
							file >> t[i][j];
						}
					}

					for (unsigned int i = 0; i < u.size(); i++)
					{
						for (unsigned int j = 0; j < u[0].size(); j++)
						{
							file >> u[i][j];
						}
					}

					for (unsigned int i = 0; i < c.size(); i++)
					{
						for (unsigned int j = 0; j < c[0].size(); j++)
						{
							for (unsigned int k = 0; k < c[0][0].size(); k++)
							{
								file >> c[i][j][k];
							}
						}
					}
				}
				else
				{
					cerr << __FUNCTION__ << "(): Allocazione strutture fallita." << endl;
					result = 1;
				}
			}
			else
			{
				cerr << __FUNCTION__ << "(): Parametri dell'istanza " << filename << " errati." << endl;
				result = 1;
			}
		}
		catch (exception &e)
		{
			cerr << __FUNCTION__ << "(): An exception has occurred: " << filename << ": " << e.what() << endl;
			result = 1;
		}
		file.close();
	}
	else
	{
		cerr << __FUNCTION__ << "(): Impossibile aprire il file " << filename << endl;
		result = 1;
	}
	return result;
}

//costruisce la matrice b partendo dalla matrice di traffico 
//la matrice b e' pre-inizializzata al valore 0
void convertIntoBMatrix()
{
	if (t.size() > 0 && b.size() > 0)
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
	else
	{
		cerr << __FUNCTION__ << "(): Strutture dati non allocate." << endl;
	}
}


//test sui file delle soluzioni
int testSolutionFile(const char *reference, const char *test)
{
	ifstream refFile, tstFile;
	const int buf = 128;
	int rowCount = 1, result = 0;
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
		cerr << __FUNCTION__ << "(): Impossibile aprire i file: " << reference << " e/o " << test << endl;
		result = -1;
	}
	if (result == 0)
		cout << "File identici" << endl;
	return result;
}

solution_t *saveSolution(CEnv env, Prob lp, int yBegin)
{
	try
	{
		int status = 0;
		double objval = 0;

		solution_t *solution= new solution_t; 

		double *solutionArray = new double[P];


		CHECKED_CPX_CALL(CPXgetobjval, env, lp, &objval);
	
		solution->objValue = objval;
	
		status = CPXgetx(env, lp, solutionArray, yBegin, yBegin + P-1);
		if (status == 0)
		{
			solution->yPositions = solutionArray;
			return solution;
		}
		else
		{
			solution->yPositions = NULL;
			return NULL;
		}
	}
	catch(exception &e)
	{
		cout<< __FUNCTION__ << " An exception has occurred: " << e.what() << endl; 
		return NULL;
	}
}

void printSolution(solution_t *solution)
{
	if(solution==NULL)
	{
		cerr << __FUNCTION__ << "(): Soluzione non allocata."<<endl;
	}
	else
	{
		int count = 0;
		cout << endl << "Soluzione istanza: " << endl;
		cout << "Valore f. obiettivo: " << solution->objValue << endl;
		for (int i = 0; i < P; i++)
		{
			if (solution->yPositions[i] == 1)
				count++;
		}
		cout << "Droni impiegati/droni totali: " << count << "/" << d << endl;

		for (unsigned int i = 0; i < P; i++)
		{
			cout << "y_" << i + n << ": " << (int)solution->yPositions[i] << endl;
		}
		cout << endl;
	}

}

void printVarsValue(CEnv env, Prob lp)
{
	try
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
	catch (exception &e)
	{
		cout << __FUNCTION__ << " An exception has occurred: " << e.what() << endl;
	}
}

void printSimplifiedSolFile(CEnv env, Prob lp, const char* solution)
{
	try
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
		if (unitFile.is_open())
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
	catch (exception &e)
	{
		cerr << __FUNCTION__ << " An exception has occurred: " << e.what() << endl;
	}
}

void setCompleteGraph(double stdCost, int stdCap)
{
	if (c.size() > 0 && u.size() > 0)
	{
		for (unsigned int i = n; i < c.size(); i++)
		{
			for (unsigned int j = n; j < c[0].size(); j++)
			{
				if (i != j)
				{
					for (unsigned int k = 0; k < c[0][0].size(); k++)
					{
						c[i][j][k] = stdCost;
					}
					u[i][j] = stdCap;
				}
			}
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): Strutture dati non allocate." << endl;
	}
}

int getSolutionFlowMatrix(CEnv env, Prob lp, vector< vector< vector<int> > > &flow)
{
	int status = 0;
	try
	{
		char **cur_colname = new char *[1];
		int cur_storespace = 16;
		char *cur_colnamestore = new char[cur_storespace];
		int *surplus = new int[1];
		vector<double> varVals;
		char *tokens;

		int indexes[3];
		
		//print solution (var values)
		int n = CPXgetnumcols(env, lp);

		varVals.resize(n);

		CHECKED_CPX_CALL(CPXgetx, env, lp, &varVals[0], 0, n - 1);
		int v = 0;
		while (v < n && status == 0)
		{
			/// to get variable name, use the RATHER TRICKY "CPXgetcolname"
			status = CPXgetcolname(env, lp, cur_colname, cur_colnamestore, cur_storespace, surplus, v, v);
			if (status == 0)
			{
				if (cur_colnamestore[0] != 'f')
				{
					status = 1;
				}
				else
				{
					int i = 0;
					//cout<< "stringa: " << cur_colnamestore<<endl;
					tokens = strtok(cur_colnamestore, "f_");
					if (tokens != NULL)
						indexes[i] = atoi(tokens);
					//cout << "token trovato: " << tokens << endl;

					while (tokens != NULL)
					{
						tokens = strtok(NULL, "f_");
						if (tokens != NULL)
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
		delete[] surplus;
		delete[] cur_colnamestore;
		delete[] cur_colname;
		return status;
	}
	catch (exception &e)
	{
		cerr << __FUNCTION__ << " An exception has occurred: " << e.what() << endl;
		status = 1;
		return status;
	}
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
	if (flow.size() > 0)
	{
		for (unsigned int i = 0; i < flow.size(); i++)
		{
			for (unsigned int j = 0; j < flow[0].size(); j++)
			{
				int acc = 0;
				for (unsigned int k = 0; k < flow[0][0].size(); k++)
				{
					if (flow[i][j][k]>0)
						acc += flow[i][j][k];
				}
				if (acc > 0)
					cout << "Flusso sul link (" << i << "," << j << "): " << acc << "/" << u[i][j] << endl;
			}
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): Strutture dati non allocate." << endl;
	}
}

void printNodesIOFlow()
{
	if (t.size() > 0)
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
	else
	{
		cerr << __FUNCTION__ << "(): Strutture dati non allocate." << endl;
	}
}
 
void setupLP(CEnv env, Prob lp)
{
	//allocazione mappe
	vector< vector< vector<int> > > fMap(totalPotentialNodes, vector< vector<int> >(totalPotentialNodes, vector<int>(K, -1)));
	
	vector< vector<int> > xMap(totalPotentialNodes, vector<int>(totalPotentialNodes, -1));

	vector< vector<int> > wMap(totalPotentialNodes, vector<int>(totalPotentialNodes, -1));

	vector< vector<int> > zMap(P, vector<int>(P, -1));

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

	cout << "Sono state create " << CPXgetnumcols(env, lp) << " variabili f_i_j_k" << endl; //numero totale delle vars create	

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
	cout << "Sono state create " << CPXgetnumcols(env, lp)- y_index << " variabili y_i" << endl; //numero totale delle vars create	

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

	cout << "Sono state create " << CPXgetnumcols(env, lp) - x_index << " variabili x_i_j" << endl; //numero totale delle vars create	

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

	cout << "Sono state create " << CPXgetnumcols(env, lp) - w_index << " variabili w_i_j" << endl;  //numero totale delle vars create	

	//aggiunta variabili z_i_j
	int z_index = CPXgetnumcols(env,lp);
	int zMapIndex = z_index;
	for (int i = n; i < totalPotentialNodes; i++)
	{
		char ztype = 'B'; //***
		double lb = 0.0;
		double ub = 1.0;
		for(int j = n; j < totalPotentialNodes; j++)
		{
			if(i!=j)
			{
				snprintf(name, NAME_SIZE, "z_%d_%d", i, j); //scrive il nome della variabile z_i_j sulla stringa name[]
				char* zname = (char*)(&name[0]);
				CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &zero, &lb, &ub, &ztype, &zname);   //costruisce la singola variabile 

				//costruzione della mappa
				zMap[i-n][j-n] = zMapIndex; //TODO: sistemare gli indici
				zMapIndex++;
			}
		}
	} 

	cout << "Sono state create " << CPXgetnumcols(env, lp) - z_index << " variabili z_i_j" << endl;  //numero totale delle vars create

	//vincoli

	char sense;
	int matbeg;
	vector<int> idx;
	vector<double> coef;

	int rowNumber = 0;

	// 1. capacità dei link
	/*sense = 'L';
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
			if (i != j) //***
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
	cout << "Vincoli (1) creati\n" << endl;*/

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
				if (i != v && fMap[i][v][k] != -1) //*** a meno della presenza di nodi isolati, ci sarà per ogni vincolo almeno una coppia di f tc fmap[][][]!=-1
				{
					idx.push_back(fMap[i][v][k]);
					coef.push_back(1.0);
				}
			}

			for (int j = 0; j < totalPotentialNodes; j++)
			{
				//if (v != j && graph[v][j] != 0)
				if (v != j && fMap[v][j][k] != -1) //***condizione più generale, include le fijv di nodi tra cui non ci sono link
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
	/*sense = 'L';
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
	cout << "Vincoli (3) creati\n" << endl;*/

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

	// 5. un drone non puo' mantenere piu' di s potenziali connessioni simultanee
	sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int v = baseDroneIndex; v < totalPotentialNodes; v++)
	{
		double temp_s = (double)s;
		for (int i = 0; i < totalPotentialNodes; i++)
		{
			//if (i != v)
			if (xMap[i][v] != -1) //***
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
					if (c[i][j][k] > threshold && fMap[i][j][k] != -1) //***
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

	// 9. vincoli sulla capacita' massima dei droni: Utx e Urx
	/*sense = 'L';
	matbeg = 0;
	
	for (int i = 0; i < totalPotentialNodes; i++)
	{
		
		vector<int> idxTx, idxRx;
		vector<double> coefTx, coefRx;
		double temp_uTx=(double)droneTXCapacity;
		double temp_uRx=(double)droneRXCapacity;
		//sum of w_i_j
		for (int j = 0; j < totalPotentialNodes; j++)
		{

			//Utx: trasmissione, solo archi diretti
			if (i != j && wMap[i][j] != -1)//***
			{
				idxTx.push_back(wMap[i][j]);
				coefTx.push_back(1.0);
			}

			//Urx: ricezione, solo archi inversi
			if (i != j && wMap[j][i] != -1)//***
			{
				idxRx.push_back(wMap[j][i]);
				coefRx.push_back(1.0);
			}
		}

		if(idxTx.size() > 0)
		{ 
			snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;

			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idxTx.size(), &temp_uTx, &sense, &matbeg, &idxTx[0], &coefTx[0], NULL, &rowname);
		}

		if(idxRx.size() > 0)
		{
			rowNumber++;
			snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);

			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idxRx.size(), &temp_uRx, &sense, &matbeg, &idxRx[0], &coefRx[0], NULL, &rowname);
		}
		idxTx.clear();
		idxRx.clear();
		coefTx.clear();
		coefRx.clear();
	}
	cout << "Vincoli (9) creati\n" << endl;*/

	//10. gestione interferenza

	//TX 
	//10.a
	sense = 'L';
	matbeg = 0;
	//coef.clear();
	//idx.clear();
	for (int i = n; i < totalPotentialNodes; i++)
	{
		vector< int > idxTx, idxRx;
		vector< double > coefTx, coefRx; 
		//first sum A*f
		for (int j = 0; j < totalPotentialNodes; j++)
		{
			for (int k = 0; k < K; k++)
			{
				if(i != j && fMap[i][j][k] != -1)
				{
					idxTx.push_back(fMap[i][j][k]);
					coefTx.push_back(getDistanceCoef(i, j));

					idxRx.push_back(fMap[j][i][k]);
					coefRx.push_back(getDistanceCoef(j, i));
				}
			}
		}

		

		//second sum z*B
		for(int l=n; l< totalPotentialNodes; l++)
		{
			if(l != i)
			{
				idxTx.push_back(zMap[i-n][l-n]); //TODO: gestire meglio gli indici
				coefTx.push_back(getInterferenceFactor(i,l));

				idxRx.push_back(zMap[l-n][i-n]); //TODO: gestire meglio gli indici
				coefRx.push_back(getInterferenceFactor(l,i));
			}
		}

		//third sum

		double sumBTx = 0;
		double sumBRx = 0;
		for(int v=0; v< n; v++)
		{
			sumBTx+= getInterferenceFactor(i,v);

			sumBRx+= getInterferenceFactor(v,i);
		}

		//idx.push_back(y_index + i - n);
		//coef.push_back(sumB);
		
		if(idxTx.size() > 0)
		{
			//add Utx*yi
			idxTx.push_back(y_index + i - n);
			coefTx.push_back(-droneTXCapacity+sumBTx);

			
			//add Utx
			//idx.push_back(y_index + i - n);
			//coef.push_back(- droneTXCapacity);

			snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;


			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idxTx.size(), &zero, &sense, &matbeg, &idxTx[0], &coefTx[0], NULL, &rowname);
			idxTx.clear();
			coefTx.clear();
		}

		if(idxRx.size() > 0)
		{
			idxRx.push_back(y_index + i - n);
			coefRx.push_back(-droneTXCapacity+sumBRx);

			snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
			rowname = (char*)(&name[0]);
			rowNumber++;


			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idxRx.size(), &zero, &sense, &matbeg, &idxRx[0], &coefRx[0], NULL, &rowname);
			idxRx.clear();
			coefRx.clear();

		}
	}

	//10.b
	sense = 'L';
	matbeg = 0;
	
	for (int i = 0; i < n; i++)
	{
		double temp_uTx = droneTXCapacity;
		double temp_uRx = droneRXCapacity;

		vector< int > idxTx, idxRx;
		vector< double > coefTx, coefRx; 

		//first sum
		for (int j = n; j < totalPotentialNodes; j++)
		{
			for (int k = 0; k < K; k++)
			{
				if(i != j && fMap[i][j][k] != -1)
				{
					idxTx.push_back(fMap[i][j][k]);
					coefTx.push_back(getDistanceCoef(i, j));

					idxRx.push_back(fMap[j][i][k]);
					coefRx.push_back(getDistanceCoef(j, i));

				}
			}
		}

		//second sum 
		for(int l=n; l< totalPotentialNodes; l++)
		{
			if(l != i)
			{
				idxTx.push_back(y_index + l - n);
				coefTx.push_back(getInterferenceFactor(i,l));

				idxRx.push_back(y_index + l - n);
				coefRx.push_back(getInterferenceFactor(l,i));
			}
		}

		//third sum
		//double sumB = 0;
		//for(int v=0; v< n; v++)
		//{
		//	sumB+= getInterferenceFactor(i,v);
		//}
		
		if(idxTx.size() > 0)
		{
			//add Utx
			//idxTx.push_back(y_index + i - n);
			//coef.push_back(- droneTXCapacity);

			snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;


			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idxTx.size(), &temp_uTx, &sense, &matbeg, &idxTx[0], &coefTx[0], NULL, &rowname);
			idxTx.clear();
			coefTx.clear();
		}

		if(idxRx.size() > 0)
		{
			//add Utx
			//idx.push_back(y_index + i - n);
			//coef.push_back(- droneTXCapacity);

			snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;


			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idxRx.size(), &temp_uRx, &sense, &matbeg, &idxRx[0], &coefRx[0], NULL, &rowname);
			idxTx.clear();
			coefTx.clear();
		}

		
	}
	cout << "Vincoli (10) creati\n" << endl;

	//11. ausiliario
	sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();

	for(int i=n; i<totalPotentialNodes; i++)
	{
		for(int l=n; l<totalPotentialNodes; l++)
		{
			if(i!=l)
			{
				idx.push_back(y_index + i - n);
				coef.push_back(1.0);

				idx.push_back(y_index + l - n);
				coef.push_back(1.0);

				idx.push_back(zMap[i-n][l-n]); //TODO: sistema indici
				coef.push_back(-1.0);

				snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
				char* rowname = (char*)(&name[0]);
				rowNumber++;


				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &uno, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
				idx.clear();
				coef.clear();

				//Auxiliary constraint 1: zl - xi <= 0
				idx.push_back(zMap[i-n][l-n]); //TODO: sistema indici
				coef.push_back(1.0);
				idx.push_back(y_index + i - n);
				coef.push_back(-1.0);

				snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
				rowname = (char*)(&name[0]);
				rowNumber++;


				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
				idx.clear();
				coef.clear();

				//Auxiliary constraint 2: zl - xl <= 0
				idx.push_back(zMap[i-n][l-n]); //TODO: sistema indici
				coef.push_back(1.0);
				idx.push_back(y_index + l - n);
				coef.push_back(-1.0);

				snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
				rowname = (char*)(&name[0]);
				rowNumber++;


				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
				idx.clear();
				coef.clear();

			}
		}
		
	}
}

int main(int argc, char const *argv[])
{
	string lpFile("flow4.lp"), solution("solution4.txt"), instance("test4.txt"), solFile("flow4.sol"), clpFile("conflict.clp");
	solution_t *instSolution=NULL;
	//dichiarazione dei timepoint/epoch (strutture dati in cui porre i tempi di inizio e fine dell'esecuzione di ogni istanza) 
	high_resolution_clock::time_point start, end;

	srand(time(NULL)); /* seed random number generator */

	vector<nodesCoordinates_t> grid=createGrid(6,4,1.0);
	printGrid(grid);

	if (loadInstance(instance.c_str()) != 0)
	{
		cerr << __FUNCTION__ <<" Impossibile caricare l'istanza: " << instance << endl;
	}
	else
	{
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

		//saveInstance(instance.c_str());
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

			cout << "Status code: " << status << endl;

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
			
			instSolution = saveSolution(env, lp, y_index);
			if (instSolution == NULL)
			{
				cerr << __FUNCTION__ << "(): Impossibile salvare la soluzione dell'istanza risolta." << endl;
			}
			else
			{
				printSolution(instSolution);
			}
			
			printSimplifiedSolFile(env, lp, solution.c_str());
			
			printVarsValue(env, lp);
			
			vector< vector< vector<int> > > flow(totalPotentialNodes, vector< vector<int> >(totalPotentialNodes, vector<int>(K, -1)));
			getSolutionFlowMatrix(env, lp, flow);
			printNetworkUsage(flow);
			//testSolutionFile("refIST3.txt", solution.c_str());

			//vector<double> y(P, 0);
			//status = CPXgetx(env, lp, y, y_index, y_index+P);

			CHECKED_CPX_CALL(CPXsolwrite, env, lp, solFile.c_str());
			// free
			CPXfreeprob(env, &lp);
			CPXcloseCPLEX(&env);
			if (instSolution != NULL)
			{
				delete[] instSolution->yPositions;
				delete instSolution;
			}

		}
		catch (exception& e)
		{
			cerr << __FUNCTION__ << "(): An exception has occurred: " << e.what() << endl;
		}
		return 0;
	}
}
