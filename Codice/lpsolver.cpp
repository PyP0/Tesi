#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <chrono>
#include <ctime>
#include <algorithm>
#include <numeric>
#include "cpxmacro.h"
#include "lpsolver.h"
#include "utility.h"
#include "getRSS.h"
#include "instance.h"


using namespace std;

// error status and message buffer
int status;
char errmsg[BUF_SIZE];

const int NAME_SIZE = 512;
char name[NAME_SIZE];

//indici iniziali per la memorizzazione delle variabili di CPLEX
int f_index = 0;
int y_index = 0;
int x_index = 0;
int z_index = 0;
int s_index = 0;
int supera_index = 0;

int numfVars = 0;
int numyVars = 0;
int numxVars = 0;
int numzVars = 0;
int numsVars = 0;
int numsuperaVars = 0;

//vettori di supporto per lo switch continuo/intero
vector <char> ynewType;
vector <char> xnewType;
vector <char> znewType;
vector <char> superanewType;
	
vector <int> yIDX;
vector <int> xIDX;
vector <int> zIDX;
vector <int> superaIDX;
vector <int> fIDX;
vector <int> sIDX;

int bigM = 1000000; //default value

int specialGrid = 0; // BTNGRIDS it's a miserable workaround, no time to implement nicely, apologies



static bool areOutOfSight(int i, int j) //TODO: controllarne effettiva funzionalita'
{
	if(getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y) <= getNodeRadius())
		return false;
	else
		return true;
	/*int k = 0; NOCOST
	bool flag = true;
	if (isCEmpty() == false) //c e' gia' stato allocato?
	{
		while (k < getCommsNum() && flag == true)
		{
			if (getCost(i,j,k) <= getThreshold())
				flag = false;
			k++;
		}
	}
	else
	{
		flag = false;
		cerr << __FUNCTION__ << "(): Struttura dati non allocata." << endl;
	}
	return flag;*/
}

void setSpecialGrid(int g)
{
	specialGrid = g;
}

int gety_index()
{
	return y_index;
}

int getz_index()
{
	return z_index;
}

int gets_index()
{
	return s_index;
}

int getsupera_index()
{
	return supera_index;
}

static void setupLP(CEnv env, Prob lp, int contRelax[])
{

	const double zero = 0.0;
	//const double uno = 1.0;

	cout << "Current memory usage, pre-model: " << getCurrentRSS( ) / 1024 << "KB" << endl;
	//allocazione mappe
	vector< vector< vector<int> > > fMap(getTotalPotentialNodes(), vector< vector<int> >(getTotalPotentialNodes(), vector<int>(getCommsNum(), -1)));

	vector< vector<int> > xMap(getTotalPotentialNodes(), vector<int>(getTotalPotentialNodes(), -1));

	vector< vector<int> > zMap(getTotalPotentialNodes(), vector<int>(getTotalPotentialNodes(), -1));

	

	//aggiunta delle variabili f_i_j_k, una per volta, con i!=j
	//nota: la variabile f_i_j_k e' distinta dalla variabile f_j_i_k
	int fMapIndex = 0;
	f_index = 0;
	char ftype; 
	
	if( (contRelax[0] & 16) != 16) //rilassamento continuo?
		ftype = 'C';
	else
	{
		ftype = 'C'; 
		cout << "Rilassamento continuo su variabili f_i_j_k" << endl;
	}
	
	for (int k = 0; k < getCommsNum(); k++)
	{
		for (int i = 0; i < getTotalPotentialNodes(); i++)
		{
			for (int j = 0; j < getTotalPotentialNodes(); j++)
			{

				//if (i != j) //c'è un arco tra i nodi i e j, quindi creo la corrispondente variabile di flusso fijk
				//if (i != j && (getCost(i,j,k) <= getThreshold()) && !(i < getUsrsNum() && j < getUsrsNum())) //*** NOCOST
				
				
				if (i != j && (getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y) <= getNodeRadius()) && !(i < getUsrsNum() && j < getUsrsNum()))
				{		
					double lb = 0.0;
					double ub = CPX_INFBOUND;
					
					//double cost = getCost(i,j,k); NOCOST
					
					snprintf(name, NAME_SIZE, "f_%d_%d_%d", i, j, k); //scrive il nome della variabile f_i_j_k sulla stringa name[]
					char* fname = (char*)(&name[0]);
					//CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &cost, &lb, &ub, &ftype, &fname);   //costruisce la singola variabile 
					CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &zero, &lb, &ub, &ftype, &fname);   //costruisce la singola variabile 

					//costruzione della map
					fMap[i][j][k] = fMapIndex;
					fMapIndex++;
				}
			}
		}
	}
	
	numfVars = CPXgetnumcols(env, lp);
	cout << "Sono state create " << numfVars << " variabili f_i_j_k" << endl; //numero totale delle vars create	

	//numerazione nodi: da 0 a n sono gli utenti, da n a n+d sono le posizioni potenziali
	//inserimento delle variabili y_i

	y_index = CPXgetnumcols(env, lp);

	//for (int i = 0; i < getTotalPotentialNodes() - getUsrsNum(); i++) NEWGRID
	for (int i = 0; i < getTotalPotentialNodes() - getUsrsNum(); i++)
	{
		char ytype; 
		if( (contRelax[0] & 8) != 8) //rilassamento continuo?
			ytype = 'B';
		else
		{
			ytype = 'C'; 
			if( i == 0) //debug
				cout << "Rilassamento continuo su variabili y_i" << endl;
		}
		
		double lb = 0.0;
		double ub = 1.0;

		snprintf(name, NAME_SIZE, "y_%d", getUsrsNum() + i); //scrive il nome della variabile y_i sulla stringa name[]
		char* yname = (char*)(&name[0]);
		CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &deployCost[i], &lb, &ub, &ytype, &yname);


	}
	numyVars = CPXgetnumcols(env, lp) - y_index;
	cout << "Sono state create " << numyVars << " variabili y_i" << endl; //numero totale delle vars create	

	//aggiunta delle variabili x_i_j

	//si divide in due step la creazione delle variabili x_i_j
	//variabili relative agli "utenti"

	x_index = CPXgetnumcols(env, lp);
	int xMapIndex = x_index;

	bool printFlag = false; 

	for (int i = 0; i < getUsrsNum(); i++)
	{
		//esclude la creazione delle variabili con i==j e quelle che esprimono link tra due utenti
		for (int j = getUsrsNum(); j < getTotalPotentialNodes(); j++)
		{
			if (!areOutOfSight(i, j)) //***
			//if (!areOutOfSight(i, j)) //***
			{
				char xtype; 
				if( (contRelax[0] & 4) != 4) //rilassamento continuo?
					xtype = 'B';
				else
				{
					xtype = 'C'; 
					if( printFlag == false) //debug
					{
						cout << "Rilassamento continuo su variabili x_i_j" << endl;
						printFlag = true;
					}
				}
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
	for (int i = getUsrsNum(); i < getTotalPotentialNodes(); i++)
	{
		for (int j = 0; j < getTotalPotentialNodes(); j++)
		{
			//esclude le variabili x_i_j con i==j
			//if(i!=j)
			if (i != j && (!areOutOfSight(i, j) )) //***
			//if (i != j && (!areOutOfSight(i, j))) //***
			{
				char xtype; 
				if( (contRelax[0] & 4) != 4) //rilassamento continuo?
					xtype = 'B';
				else
				{
					xtype = 'C';
				}
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
	
	numxVars = CPXgetnumcols(env, lp) - x_index;
	cout << "Sono state create " << numxVars << " variabili x_i_j" << endl; //numero totale delle vars create	

	
	//aggiunta variabili z_i
	z_index = CPXgetnumcols(env, lp);
	
	//int zMapIndex = z_index;
	for (int i = 0; i < getTotalPotentialNodes(); i++)
	{
		char ztype; 
		if( (contRelax[0] & 2) != 2) //rilassamento continuo?
			ztype = 'B';
		else
		{
			ztype = 'C'; 
			if( i == 0) //debug
				cout << "Rilassamento continuo su variabili z_i" << endl;
		}

		double lb = 0.0;
		double ub = 1.0;
		
		snprintf(name, NAME_SIZE, "z_%d", i); //scrive il nome della variabile z_i sulla stringa name[]
		char* zname = (char*)(&name[0]);
		
		CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &zero, &lb, &ub, &ztype, &zname);   //costruisce la singola variabile 

		//costruzione della mappa
		//zMap[i] = zMapIndex; //TODO: sistemare gli indici
		//zMapIndex++;
		
		
	}

	numzVars = CPXgetnumcols(env, lp) - z_index;
	cout << "Sono state create " << numzVars  << " variabili z_i" << endl;  //numero totale delle vars create


	//aggiunta variabili s_i
	s_index = CPXgetnumcols(env, lp);
	for (int i = 0; i < getTotalPotentialNodes(); i++)
	{
		char stype = 'C'; 

		double lb = 0.0;
		double ub = CPX_INFBOUND;
		
		snprintf(name, NAME_SIZE, "s_%d", i); //scrive il nome della variabile s_i sulla stringa name[]
		char* sname = (char*)(&name[0]);
		
		CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &zero, &lb, &ub, &stype, &sname);   //costruisce la singola variabile 
	}

	numsVars = CPXgetnumcols(env, lp) - s_index;
	cout << "Sono state create " << numsVars << " variabili s_i" << endl;  //numero totale delle vars create


	//aggiunta variabili supera_i
	supera_index = CPXgetnumcols(env, lp);
	for (int i = 0; i < getTotalPotentialNodes(); i++)
	{  
		char sptype; 
		if( (contRelax[0] & 1) != 1) //rilassamento continuo?
			sptype = 'B';
		else
		{
			sptype = 'C'; 
			if( i == 0) //debug
				cout << "Rilassamento continuo su variabili sp_i" << endl;
		}
		
		double lb = 0.0; 
		double ub = 1.0;
		
		snprintf(name, NAME_SIZE, "sp_%d", i); //scrive il nome della variabile s_i sulla stringa name[]
		char* sname = (char*)(&name[0]);
		
		CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &zero, &lb, &ub, &sptype, &sname);   //costruisce la singola variabile 
	}

	numsuperaVars = CPXgetnumcols(env, lp) - supera_index;
	cout << "Sono state create " << numsuperaVars << " variabili sp_i" << endl;  //numero totale delle vars create

	
	if(contRelax[0] == 31) //cambia tipo del problema
	{
		CPXchgprobtype(env, lp, CPXPROB_LP);
	}

	// 1.0 addition //

	//Reduction factor
	double reductionFactor = getReductionFactor(); 
	
	cout << "Smax parameter: " << reductionFactor << endl;

	//vettori di supporto per lo switch continuo/intero

	yIDX.resize(numyVars); 
	xIDX.resize(numxVars);
	zIDX.resize(numzVars);
	superaIDX.resize(numsuperaVars);
	fIDX.resize(numfVars);
	sIDX.resize(numsVars);
	
	iota(yIDX.begin(), yIDX.end(), y_index);
	iota(xIDX.begin(), xIDX.end(), x_index);
	iota(zIDX.begin(), zIDX.end(), z_index);
	iota(superaIDX.begin(), superaIDX.end(), supera_index);
	iota(fIDX.begin(), fIDX.end(), f_index);
	iota(sIDX.begin(), sIDX.end(), s_index);
	
	//=========================================================================
	//=========================================================================
	//									vincoli
	//=========================================================================
	//=========================================================================


	char sense;
	int matbeg;
	vector<int> idx;
	vector<double> coef;

	int rowNumber = 0;
	double bigM = 0.0;
	
	// 2. conservazione flusso (8)
	sense = 'E';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int v = 0; v < getTotalPotentialNodes(); v++) // V'
	{
		for (int k = 0; k < getCommsNum(); k++) // K
		{
			for (int i = 0; i < getTotalPotentialNodes(); i++) // V'
			{
				if (i != v && fMap[i][v][k] != -1) 
				{
					idx.push_back(fMap[i][v][k]);
					coef.push_back(1.0);
				}
			}

			for (int j = 0; j < getTotalPotentialNodes(); j++) // V'
			{
				
				if (v != j && fMap[v][j][k] != -1) 
				{
					idx.push_back(fMap[v][j][k]);
					coef.push_back(-1.0);
				}
			}

			snprintf(name, NAME_SIZE, "c2_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;


			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &b[v][k], &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
			idx.clear();
			coef.clear();

		}
	}

	// 3 gestione flussi TX 1.5  Sum(Sum(f_ijk)) <= U_tx * y_i 
	sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();

	for(int i = getUsrsNum(); i < getTotalPotentialNodes(); i++)  // P
	{
		// somma f_i_j_k (1)
		for(int j = 0; j < getTotalPotentialNodes(); j++) // V'
		{
			for(int k = 0; k < getCommsNum(); k++)
			{
				if (i != j && fMap[i][j][k] != -1)
				{
					idx.push_back(fMap[i][j][k]);
					coef.push_back(1.0 + (getInterference(j,i)/getReductionFactor()));
				}
			}
		}

		if(coef.size() > 0)
		{
			idx.push_back( y_index + i - getUsrsNum() );
			coef.push_back( -getTXCapacity() );
			//rhs = getTXCapacity() - rhs; 

			snprintf(name, NAME_SIZE, "c3_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;

			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
			idx.clear();
			coef.clear();
		}
	}

	// 4.  Sum(f_jik) <= U_rx * (y_i - s_i)  (1) 1.5
	sense = 'L'; 
	matbeg = 0;
	coef.clear();
	idx.clear();
	for(int i = getUsrsNum(); i < getTotalPotentialNodes(); i++) // P
	{
		// sommatoria f_j_i_k
		for(int j = 0; j < getTotalPotentialNodes(); j++)  // V'
		{
			for (int k = 0; k < getCommsNum(); k++)  // K
			{
				if (i != j && fMap[j][i][k] != -1) // f_j_i_k esiste
				{
					idx.push_back(fMap[j][i][k]);
					coef.push_back(1.0);
				}
			}
		}

		// Urx * y_i
		idx.push_back(y_index + i - getUsrsNum()); //y_i
		coef.push_back(-getRXCapacity());

		// Urx * s_i
		idx.push_back(s_index + i);
		coef.push_back(1.0);

		snprintf(name, NAME_SIZE, "c4_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}	
	//cout << "Vincoli (5) creati\n";


	// 5. zi >= yj  versione 1.5
	matbeg = 0;
	coef.clear();
	idx.clear();
	sense = 'G';

	for(int i = getUsrsNum(); i < getTotalPotentialNodes(); i++)  // P
	{
		for(int j = getUsrsNum(); j < getTotalPotentialNodes(); j++)  // P
		{
			if( i != j )
			{
				double distance = getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y);	
				if(distance <= getEpsilonNodeRadius())
				{
					idx.push_back(z_index + i);
					coef.push_back(1.0);
					
					idx.push_back(y_index + j - getUsrsNum());
					coef.push_back(-1.0);

					snprintf(name, NAME_SIZE, "c5_%d", rowNumber); //numerazione progressiva dei vincoli
					char* rowname = (char*)(&name[0]);
					rowNumber++;

					CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
					idx.clear();
					coef.clear();
				}
			}
		}
	}


	// 6. Sum(A_ji * y_j) + Sum(A_ji) <= S_max + (M  - Smax)*sp_i   1.5

	//bigM = 2 * reductionFactor; //set the big M value for each specific constraint
	sense = 'L'; 
	matbeg = 0;
	coef.clear();
	idx.clear();

	for(int i = getUsrsNum(); i < getTotalPotentialNodes(); i++)  // P
	{
		double rhs = 0;
		///int bigMMultiplyCounter = 0;
		bigM = 0.0;

		// sum Aji * y_j su P
		for(int j = getUsrsNum(); j < getTotalPotentialNodes(); j++)  // P
		{
			double distance = getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y);
			if(i != j && distance > getEpsilonNodeRadius() && distance <= getNodeRadius()) //R_epsilon excluded
			{
				idx.push_back(y_index + j - getUsrsNum());
				coef.push_back(getInterference(j,i));

				///bigMMultiplyCounter++;
				bigM+= getInterference(j,i);
			}
		}

		///bigMMultiplyCounter = min(bigMMultiplyCounter+1, getDrnsNum());

		// sum Aji su V v 1.5

		for(int j = 0; j < getUsrsNum(); j++) // V
		{
			double distance = getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y);
			if(i != j && distance <= getNodeRadius()) //1.5 mod
			{
				rhs-= getInterference(j,i);

				///bigMMultiplyCounter++; 
				bigM+= getInterference(j,i);
			}
		}

		///bigM = bigMMultiplyCounter * reductionFactor; //set the big M value for each specific constraint
		bigM += 100;

		rhs+= reductionFactor;

		// Smax+(M-Smax)(supera_i + 1 - y_i)

		idx.push_back(supera_index + i);
		coef.push_back(-(bigM - reductionFactor));


		snprintf(name, NAME_SIZE, "c6_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();

	}

	// 7. s_i >= S_max (z_i +y_i -1) 1.5

	sense = 'G'; 
	matbeg = 0;
	coef.clear();
	idx.clear();
	for( int i = getUsrsNum(); i < getTotalPotentialNodes(); i++)  // P
	{
		double rhs = 0.0;
		// sconto
		idx.push_back(s_index + i);
		coef.push_back(1.0);

		// fattore riduzione subordinato a area critica (epsilon)
		idx.push_back(z_index + i);
		coef.push_back(-reductionFactor);

		// Smax * y_i
		idx.push_back(y_index + i - getUsrsNum());
		coef.push_back(-reductionFactor);

		//rhs = - Smax
		rhs = -reductionFactor;

		snprintf(name, NAME_SIZE, "c7_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;


		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}
	//cout << "Vincoli (2) creati\n";

	// 8. s_i >= S_max(o_i + y_i -1) 1.5

	sense = 'G'; 
	matbeg = 0;
	coef.clear();
	idx.clear();
	for( int i = getUsrsNum(); i < getTotalPotentialNodes(); i++)  // P
	{
		double rhs = 0.0;
		// sconto
		idx.push_back(s_index + i);
		coef.push_back(1.0);

		// fattore riduzione subordinato a area critica (epsilon)
		idx.push_back(supera_index + i);
		coef.push_back(-reductionFactor);

		// Smax * y_i
		idx.push_back(y_index + i - getUsrsNum());
		coef.push_back(-reductionFactor);

		//rhs = - Smax
		rhs = -reductionFactor;

		snprintf(name, NAME_SIZE, "c8_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;


		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}


	// 9. s_i >= Sum(A_ji * y_j) + Sum(A_ji) - M ( 1 - y_i - z_i - o_i)   1.5

	//bigM = 2 * reductionFactor; //set the big M value for each specific constraint
	sense = 'G'; 
	matbeg = 0;
	coef.clear();
	idx.clear();

	for( int i = getUsrsNum(); i < getTotalPotentialNodes(); i++) // P
	{
		double rhs = 0;
		///int bigMMultiplyCounter = 0;
		bigM = 0.0;

		// sconto_i
		idx.push_back(s_index + i);
		coef.push_back(1.0);

		// sum Aji * y_j su P
		for(int j = getUsrsNum(); j < getTotalPotentialNodes(); j++) // P
		{
			double distance = getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y);
			if(i != j && distance > getEpsilonNodeRadius() && distance <= getNodeRadius()) //R_epsilon excluded
			{
				idx.push_back(y_index + j - getUsrsNum());
				coef.push_back(-getInterference(j,i));

				bigM += getInterference(j,i);
				///bigMMultiplyCounter++;
			}
		}

		///bigMMultiplyCounter = min(bigMMultiplyCounter+1, getDrnsNum());

		// sum Aji su V su range aumentato
		for(int j = 0; j < getUsrsNum(); j++) // V
		{
			double distance = getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y);
			if(i != j && distance <= getNodeRadius()) // 1.5 mod
			{
				rhs+= getInterference(j,i);

				///bigMMultiplyCounter++;
				bigM+= getInterference(j,i);
			}
		}

		bigM+= 100; //final bigM value 
		///bigM = bigMMultiplyCounter * reductionFactor; //set the big M value for each specific constraint
		
		// -M(1 - y_i - z_i - o_i)

		// -M 
		rhs-= bigM;

		// -M * (-y_i)
		idx.push_back(y_index + i - getUsrsNum());
		coef.push_back(- bigM);


		// -M * (+o_i)
		idx.push_back(supera_index + i);
		coef.push_back( bigM);

		// -M * (+z_i)
		idx.push_back(z_index + i);
		coef.push_back( bigM);

		snprintf(name, NAME_SIZE, "c9_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}


	// 2. vers 1.0  Sum(f_ijk) <= M * x_ij (12)
	//bigM = max(getRXCapacity(),getTXCapacity()); 

	//set the big M value for each specific constraint
	/*bigM = 0;
	for(int i = 0; i < getUsrsNum(); i++)
	{
		for(int j = 0; j < getUsrsNum(); j++)
		{
			if(i != j)
			{
				int trf = getRequestedTraffic(i,j);
				if(trf > 0)
					bigM += trf;  //total bigM = sum of total traffic matrix
			}
		}
	}
	bigM += 1;


	sense = 'L';
	matbeg = 0;
	idx.clear();
	coef.clear();
	for (int i = 0; i < getTotalPotentialNodes(); i++)
	{
		for (int j = 0; j < getTotalPotentialNodes(); j++)
		{
			//if (i != j) ***
			//{
			//if (!(j < n && i < n)) //esclude i link i cui estremi sono entrambi nodi utenti //***
			//{
			for (int k = 0; k < getCommsNum(); k++)
			{
				if (i != j && fMap[i][j][k] != -1) //***
				{
					idx.push_back(fMap[i][j][k]);
					coef.push_back(1.0);
				}
			}
			// f_i_j_k - M * x_i_j
			if(coef.size() > 0)
			{
				idx.push_back(xMap[i][j]);
				coef.push_back(-bigM);

				snprintf(name, NAME_SIZE, "c2_%d", rowNumber); //numerazione progressiva dei vincoli
				char* rowname = (char*)(&name[0]);
				rowNumber++;
	
				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
			}
			idx.clear();
			coef.clear();
		}
	}*/

	// 2. Legame tra le variabili f_i_j_k e y_i

	//vers. 1.5
	//2.a
	/*bigM = 0; //max(getRXCapacity(),getTXCapacity()); //set the big M value for each specific constraint
	sense = 'L';
	matbeg = 0;
	idx.clear();
	coef.clear();

	for(int i = 0; i < getUsrsNum(); i++)
	{
		for(int j = 0; j < getUsrsNum(); j++)
		{
			if(i != j)
			{
				int trf = getRequestedTraffic(i,j);
				if(trf > 0)
					bigM += trf;  //total bigM = sum of total traffic matrix
			}
		}
	}
	bigM += 1;

	for (int i = getUsrsNum(); i < getTotalPotentialNodes(); i++) // P
	{
		for (int j = 0; j < getTotalPotentialNodes(); j++) // V'
		{
			//if (!(j < n && i < n)) //esclude i link i cui estremi sono entrambi nodi utenti 
			for (int k = 0; k < getCommsNum(); k++)
			{
				if (i != j && fMap[i][j][k] != -1) 
				{
					//f_i_j_k - My_i
					idx.push_back(fMap[i][j][k]);
					coef.push_back(1.0);


					

					idx.push_back(y_index + i - getUsrsNum());
					coef.push_back(-bigM); // bigM

					snprintf(name, NAME_SIZE, "c2.a_%d", rowNumber); //numerazione progressiva dei vincoli
					char* rowname = (char*)(&name[0]);
					rowNumber++;
	
					CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
					idx.clear();
					coef.clear();
				}
			}
		}
	}*/

	//2.b
	/*for (int i = 0; i < getTotalPotentialNodes(); i++) // V'
	{
		for (int j = getUsrsNum(); j < getTotalPotentialNodes(); j++) // P
		{
			//if (!(j < n && i < n)) //esclude i link i cui estremi sono entrambi nodi utenti 
			for (int k = 0; k < getCommsNum(); k++)
			{
				if (i != j && fMap[i][j][k] != -1) 
				{
					//f_i_j_k - My_j
					idx.push_back(fMap[i][j][k]);
					coef.push_back(1.0);

					idx.push_back(y_index + j - getUsrsNum());
					coef.push_back(-bigM);

					snprintf(name, NAME_SIZE, "c2.b_%d", rowNumber); //numerazione progressiva dei vincoli
					char* rowname = (char*)(&name[0]);
					rowNumber++;
	
					CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
					idx.clear();
					coef.clear();
				}
			}
		}
	}*/

	// 2. ver 1.1 AGGREGATA
	//2.a
	/*bigM = max(getRXCapacity(),getTXCapacity()); //set the big M value for each specific constraint
	sense = 'L';
	matbeg = 0;
	idx.clear();
	coef.clear();

	for (int i = getUsrsNum(); i < getTotalPotentialNodes(); i++) // P
	{
		for (int j = 0; j < getTotalPotentialNodes(); j++) // V'
		{
			for (int k = 0; k < getCommsNum(); k++)
			{
				if (i != j && fMap[i][j][k] != -1) 
				{
					//SUM(f_i_j_k)
					idx.push_back(fMap[i][j][k]);
					coef.push_back(1.0);
				}
			}

			if(coef.size() > 0)
			{
				// M*y_i
				idx.push_back(y_index + i - getUsrsNum());
				coef.push_back(-bigM);

				snprintf(name, NAME_SIZE, "c12_%d", rowNumber); //numerazione progressiva dei vincoli
				char* rowname = (char*)(&name[0]);
				rowNumber++;
		
				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
				idx.clear();
				coef.clear();
			}
		}
	}

	//2.b
	for (int i = 0; i < getTotalPotentialNodes(); i++) // V'
	{
		for (int j = getUsrsNum(); j < getTotalPotentialNodes(); j++) // P
		{
			for (int k = 0; k < getCommsNum(); k++)
			{
				if (i != j && fMap[i][j][k] != -1) 
				{
					//SUM(f_i_j_k)
					idx.push_back(fMap[i][j][k]);
					coef.push_back(1.0);
				}
			}

			if(coef.size() > 0)
			{
				idx.push_back(y_index + j - getUsrsNum());
				coef.push_back(-bigM);

				snprintf(name, NAME_SIZE, "c12a_%d", rowNumber); //numerazione progressiva dei vincoli
				char* rowname = (char*)(&name[0]);
				rowNumber++;
	
				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
				idx.clear();
				coef.clear();
				
			}
		}
	}*/
	//cout << "Vincoli (2) creati\n" << endl;


	// 3. legame tra variabili x_i_j e y_j (9)
	/*sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int j = getUsrsNum(); j < getTotalPotentialNodes(); j++)
	{
		for (int i = 0; i < getTotalPotentialNodes(); i++)
		{
			//if (i != j && graph[i][j] != 0)
			//if (i != j && xMap[i][j] != -1) //***
			if (i != j && xMap[i][j] != -1 && xMap[j][i] != -1) //***
			{
				//add x_i_j
				idx.push_back(xMap[i][j]);
				coef.push_back(1.0);

				//add -y_j
				idx.push_back(y_index + (j - getUsrsNum()));
				coef.push_back(-1.0);

				snprintf(name, NAME_SIZE, "c3_%d", rowNumber); //numerazione progressiva dei vincoli
				char* rowname = (char*)(&name[0]);
				rowNumber++;

				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);

				//add x_j_i
				idx[0] = xMap[j][i];
				

				snprintf(name, NAME_SIZE, "c3a_%d", rowNumber); //numerazione progressiva dei vincoli
				rowname = (char*)(&name[0]);
				rowNumber++;

				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
				idx.clear();
				coef.clear();
			}
		}
	}*/

	//cout << "Vincoli (3) creati\n";

	// 4. non posizionare piu' di d droni (11)
	/*double temp_d = (double)getDrnsNum();
	sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int i = 0; i < getTotalPotentialNodes() - getUsrsNum(); i++)
	{
		idx.push_back(y_index + i);
		coef.push_back(1.0);
	}

	snprintf(name, NAME_SIZE, "c4_%d", rowNumber); //numerazione progressiva dei vincoli
	char* rowname = (char*)(&name[0]);
	rowNumber++;

	CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &temp_d, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
	idx.clear();
	coef.clear();*/

	//cout << "Vincoli (4) creati\n";

	

	// 5.a Sum(f_jik) <= U_rx - s_i

	/*sense = 'L'; 
	matbeg = 0;
	coef.clear();
	idx.clear();
	for(int i = 0; i < getUsrsNum(); i++) // V
	{
		double rhs = getRXCapacity();

		// sommatoria f_j_i_k
		for(int j = getUsrsNum(); j < getTotalPotentialNodes(); j++)  // P
		{
			for (int k = 0; k < getCommsNum(); k++)  // K
			{
				if (i != j && fMap[j][i][k] != -1) // f_j_i_k esiste
				{
					idx.push_back(fMap[j][i][k]);
					coef.push_back(1.0);
				}
			}
		}

		// sconto
		idx.push_back(s_index + i);
		coef.push_back(1.0);

		snprintf(name, NAME_SIZE, "c5.a_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;


		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}*/
	//cout << "Vincoli (5.a) creati\n";

	

	// 7. vers. 1.0 aggregata (6)

	//bigM = 2 * getTotalPotentialNodes(); 
	/*matbeg = 0;
	coef.clear();
	idx.clear();
	for(int i = getUsrsNum(); i < getTotalPotentialNodes(); i++)  // P
	{
		//pre-elaborazione
		int usersEpsilonRadiusCounter = 0;
		for(int l = 0; l < getUsrsNum(); l++) // V
		{
			double distance = getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[l].x, mapGrid[l].y);
			if(i != l && distance <= getEpsilonNodeRadius())
			{
				usersEpsilonRadiusCounter++;
			}
		}

		if(usersEpsilonRadiusCounter == 0) // |D_i_epsilon| = 0
		{
			double rhs = 0;
			sense = 'L'; 
			
			//sum y_j
			for(int j = getUsrsNum(); j < getTotalPotentialNodes(); j++)  // P
			{
				double distance = getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y);
				if(i != j && distance <= getEpsilonNodeRadius())
				{
					idx.push_back(y_index + j - getUsrsNum());
					coef.push_back(1.0);
				}
			}

			if(coef.size() > 0)
			{
				bigM = min( (int)coef.size() + 5, getTotalPotentialNodes() + 5 ); //set the big M value for each specific constraint
				// M(z_i + 1 - y_i)

				idx.push_back(z_index + i);
				coef.push_back(-bigM);

				rhs= bigM;

				idx.push_back(y_index + i - getUsrsNum());
				coef.push_back(bigM);

				snprintf(name, NAME_SIZE, "c7_%d", rowNumber); //numerazione progressiva dei vincoli
				char* rowname = (char*)(&name[0]);
				rowNumber++;

				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
				idx.clear();
				coef.clear();

			}
		}
		else  // |D_i_epsilon| > 0
		{
			// z_i >= y_i 
			sense = 'G'; 
			matbeg = 0;
			coef.clear();
			idx.clear();
			
			idx.push_back(z_index + i);
			coef.push_back(1.0);

			idx.push_back(y_index + i - getUsrsNum());
			coef.push_back(-1.0);

			snprintf(name, NAME_SIZE, "c7_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;

			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
			idx.clear();
			coef.clear();
		}
	}*/
	
	
	//7.a TODO: test
	//bigM = 2 * getTotalPotentialNodes(); //set the big M value for each specific constraint
	/*sense = 'L'; 
	matbeg = 0;
	coef.clear();
	idx.clear();
	for(int i = 0; i < getUsrsNum(); i++) // V
	{
		for(int j = getUsrsNum(); j < getTotalPotentialNodes(); j++) // P
		{
			double distance = getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y);
			if(i != j && distance <= getEpsilonNodeRadius())
			{
				idx.push_back(y_index + j - getUsrsNum());
				coef.push_back(1.0);
			}
		}
		if(coef.size() > 0)
		{
			bigM = min( (int)coef.size() + 5, getTotalPotentialNodes() + 5 ); //set the big M value for each specific constraint 
			idx.push_back(z_index + i);
			coef.push_back(-bigM);
		
			snprintf(name, NAME_SIZE, "c7.a_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;

			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
			idx.clear();
			coef.clear();
		}
	}*/

	


	// 7. versione 1.1 non aggregata
	/*matbeg = 0;
	coef.clear();
	idx.clear();
	sense = 'G';

	for(int i = 0; i < getTotalPotentialNodes(); i++)  // V'
	{

		for(int j = getUsrsNum(); j < getTotalPotentialNodes(); j++)  // P
		{
			if( i != j )
			{
				double distance = getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y);	
				if(distance <= getEpsilonNodeRadius())
				{
					idx.push_back(z_index + i);
					coef.push_back(1.0);
					
					idx.push_back(y_index + j - getUsrsNum());
					coef.push_back(-1.0);

					snprintf(name, NAME_SIZE, "c6_%d", rowNumber); //numerazione progressiva dei vincoli
					char* rowname = (char*)(&name[0]);
					rowNumber++;

					CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
					idx.clear();
					coef.clear();
				}
			}
		}
	}*/
/*
	int criticalAreaCounter = 0;
	for(int i = getUsrsNum(); i < getTotalPotentialNodes(); i++)  // P
	{
		criticalAreaCounter = 0;
		for(int j = 0; j < getUsrsNum(); j++)  // V
		{
			double distance = getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y);	
			
			if(distance <= getEpsilonNodeRadius())
				criticalAreaCounter++;
		}

		if(criticalAreaCounter > 0)
		{
			idx.push_back(z_index + i);
			coef.push_back(1.0);

			snprintf(name, NAME_SIZE, "c6_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;

			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &uno, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
			idx.clear();
			coef.clear();

		}
	}*/

	//cout << "Vincoli (6) creati\n";

	
	//cout << "Vincoli (3) creati\n";


	// 8.a si >= Sum(A_ji * y_j) - M * sp_i

	//bigM = 2 * reductionFactor; //set the big M value for each specific constraint
	/*sense = 'G'; 
	matbeg = 0;
	coef.clear();
	idx.clear();
	
	for( int i = 0; i < getUsrsNum(); i++) // V
	{
		bigM = 0.0;
		///int bigMMultiplyCounter = 0;

		// sconto_i
		idx.push_back(s_index + i);
		coef.push_back(1.0);

		// sum Aji * y_j su P
		for(int j = getUsrsNum(); j < getTotalPotentialNodes(); j++) // P
		{
			double distance = getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y);
			if(i != j && distance > getEpsilonNodeRadius() && distance <= getNodeRadius()) //R_epsilon excluded
			{
				idx.push_back(y_index + j - getUsrsNum());
				coef.push_back(-getInterference(j,i));

				bigM+= getInterference(j,i);
				///bigMMultiplyCounter++;
			}
		}

		///bigMMultiplyCounter = min(bigMMultiplyCounter+1, getDrnsNum());
		///bigM = bigMMultiplyCounter * reductionFactor; //set the big M value for each specific constraint
		
		bigM += 100; //final bigM value 

		// -M(supera_i)
		idx.push_back(supera_index + i);
		coef.push_back(bigM);

		snprintf(name, NAME_SIZE, "c8a_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;


		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}*/
	//cout << "Vincoli (3.a) creati\n";

	
	//cout << "Vincoli (4) creati\n";

	// 9.a Sum(A_ji * y_j) <= S_max + M * sp_i

	//bigM = 2 * reductionFactor; //set the big M value for each specific constraint
	/*sense = 'L'; 
	matbeg = 0;
	coef.clear();
	idx.clear();

	for(int i = 0; i < getUsrsNum(); i++)  // V
	{
		///int bigMMultiplyCounter = 0;
		bigM = 0.0;

		double rhs = 0;

		// sum Aji * y_j su P
		for(int j = getUsrsNum(); j < getTotalPotentialNodes(); j++)  // P
		{
			double distance = getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y);
			if(i != j && distance > getEpsilonNodeRadius() && distance <= getNodeRadius()) //R_epsilon excluded
			{
				idx.push_back(y_index + j - getUsrsNum());
				coef.push_back(getInterference(j,i));

				///bigMMultiplyCounter++;
				bigM+= getInterference(j,i);
			}
		}

		///bigMMultiplyCounter = min(bigMMultiplyCounter+1, getDrnsNum());
		///bigM = bigMMultiplyCounter * reductionFactor; //set the big M value for each specific constraint
		bigM+= 100;

		rhs= reductionFactor;

		// M(supera_i)

		idx.push_back(supera_index + i);
		coef.push_back(-bigM);

		//rhs+= bigM;
		//idx.push_back(y_index + i);
		//coef.push_back( bigM);

		snprintf(name, NAME_SIZE, "c9.a_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();

	}*/
	//cout << "Vincoli (4.a) creati\n";

	// 9.a. s_i >= S_max * sp_i - M(1 - y_i)  (5)

	//bigM = 1.5 * reductionFactor; //set the big M value for each specific constraint
	/*sense = 'G';
	matbeg = 0;
	coef.clear();
	idx.clear();
	
	for(int i = getUsrsNum(); i < getTotalPotentialNodes(); i++) // P
	{
		double rhs = 0;
		
		// sconto_i
		idx.push_back(s_index + i);
		coef.push_back(1.0);

		// Smax (supera_i + y_i - 1)
		idx.push_back(supera_index + i);
		coef.push_back(-reductionFactor);

		idx.push_back(y_index + i - getUsrsNum());
		coef.push_back(-reductionFactor);

		// -M(1-y_i)
		rhs-= reductionFactor;


		snprintf(name, NAME_SIZE, "c10_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}*/

	//cout << "Vincoli (5) creati\n";

	// 10.a   s_i >= S_max * sp_i

	/*sense = 'G';  
	matbeg = 0;
	coef.clear();
	idx.clear();

	for(int i = 0; i < getUsrsNum(); i++)  // V
	{
		// sconto_i
		idx.push_back(s_index + i);
		coef.push_back(1.0);

		// sconto * supera_i
		idx.push_back(supera_index + i);
		coef.push_back(-reductionFactor);

		snprintf(name, NAME_SIZE, "c10.a_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}*/
	//cout << "Vincoli (5.a) creati\n";

	
	//cout << "Vincoli (7) creati\n";

	//11.a gestione flussi TX, senza interferenza 1.1

	/*sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();

	for(int i = 0; i < getUsrsNum(); i++)  // V
	{
		// somma f_i_j_k (1)
		for(int j = getUsrsNum(); j< getTotalPotentialNodes(); j++) // P
		{
			for(int k = 0; k < getCommsNum(); k++)
			{
				if (i != j && fMap[i][j][k] != -1)
				{
					idx.push_back(fMap[i][j][k]);
					coef.push_back(1.0);
				}
			}
		}

		if(coef.size() > 0)
		{
			// termine noto
			double rhs = getTXCapacity();

			snprintf(name, NAME_SIZE, "c11.a_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;

			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
			idx.clear();
			coef.clear();
		}
	}*/

	//11.  Gestione interferenza lato TX   (7)
/*
	bigM = 10 * getTXCapacity(); //set the big M value for each specific constraint //TODO

	sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();

	for(int i = getUsrsNum(); i < getTotalPotentialNodes(); i++)  // P
	{
		// somma f_i_j_k (1)
		for(int j = 0; j< getTotalPotentialNodes(); j++) // V'
		{
			for(int k = 0; k < getCommsNum(); k++)
			{
				if (i != j && fMap[i][j][k] != -1)
				{
					idx.push_back(fMap[i][j][k]);
					coef.push_back(1.0);
				}
			}
		}

		// somma fattori interferenza posizioni per P (2)
		for(int v = getUsrsNum(); v < getTotalPotentialNodes(); v++)  // P
		{
			if(i != v && getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[v].x, mapGrid[v].y) <= getNodeRadius())
			{
				idx.push_back(y_index + v - getUsrsNum());
				coef.push_back(getInterferenceFactor(i,v)); //TODO
			}
		}

		// somma fattori interferenza utenti per V (3)
		double rhs = 0;
		for(int v = 0; v < getUsrsNum(); v++)  //  V
		{
			if(i != v && getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[v].x, mapGrid[v].y) <= getNodeRadius())
			{
				rhs += getInterferenceFactor(i,v); //TODO
			}
		}

		rhs= bigM - rhs;
		idx.push_back( y_index + i - getUsrsNum() );
		coef.push_back( bigM -getTXCapacity() );
		//rhs = getTXCapacity() - rhs; 

		snprintf(name, NAME_SIZE, "c7_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}
	//cout << "Vincoli (7) creati\n";

	//11.a  Sum(Sum(f_ijk)) <= U_tx

	sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();

	for(int i = 0; i < getUsrsNum(); i++)  // V
	{
		// somma f_i_j_k (1)
		for(int j = getUsrsNum(); j< getTotalPotentialNodes(); j++) // P
		{
			for(int k = 0; k < getCommsNum(); k++)
			{
				if (i != j && fMap[i][j][k] != -1)
				{
					idx.push_back(fMap[i][j][k]);
					coef.push_back(1.0);
				}
			}
		}

		// somma fattori interferenza posizioni per P (2)
		for(int v = getUsrsNum(); v < getTotalPotentialNodes(); v++)  // P
		{
			if(i != v && getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[v].x, mapGrid[v].y) <= getNodeRadius())
			{
				idx.push_back(y_index + v - getUsrsNum());
				coef.push_back(getInterferenceFactor(i,v));
			}
		}

		// termine noto
		double rhs = getTXCapacity();

		snprintf(name, NAME_SIZE, "c7_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}*/
	//cout << "Vincoli (7.a) creati\n";



	// 10. un drone non puo' mantenere piu' di s potenziali connessioni simultanee
	/*sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int v = getUsrsNum(); v < getTotalPotentialNodes(); v++)
	{
		double temp_s = (double)getMaxConnections();
		for (int i = 0; i < getTotalPotentialNodes(); i++)
		{
			//if (i != v)
			if (xMap[i][v] != -1) 
			{
				idx.push_back(xMap[i][v]);
				coef.push_back(1.0);
			}
		}

		if (idx.size() > 0) 
		{

			snprintf(name, NAME_SIZE, "c10_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;

			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &temp_s, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
			idx.clear();
			coef.clear();
		}
	}*/
	//cout << "Vincoli (10) creati\n";


	//istanze speciali
	//if(specialGrid == 25) || specialGrid == 41 || specialGrid == 48)

	//btn25  CODE 25
	vector<int> noFlightZones;
	
	if(specialGrid == 25)
		noFlightZones= {0,1,3,4,5,7,8,9,13,17,27,31,35,36,37,39,40,41,43,44};

	//btn41 CODE 41
	if(specialGrid == 41)
		noFlightZones = {0,1,4,5,6,7,8,9,10,11,12,13,14,15,20,21,22,23,24,25,26,28,29,36,37,40,44,60,65,66,67,68,69,70,74,75,76,79,80,81,82,83,84,85,86,88,89};
	
	//btn48 CODE 48 
	if(specialGrid == 48)
		noFlightZones = {3,4,24,26,27,28,29,31,32,34,35,36,37,39,59,60};


	for(int i=0; i< (int) noFlightZones.size(); i++)
	{
		noFlightZones[i]+= getUsrsNum();
	}

	vector <char> lb(1, 'L');
	vector <char> ub(1, 'U');
	vector <double> zeros(1, 0.0);
	 
	for(int i = 0 ; i< (int)noFlightZones.size(); i++)
	{
		int j=0;
		int offset = 0;
		cout << "cerco pos: " << noFlightZones[i] << endl;
		while( mapGrid[j].id != noFlightZones[i])
		{
			cout << "mapGrid[" <<j <<"].id =" << mapGrid[j].id << endl;
			if(mapGrid[j].isUser != true)
			{
				offset++;
			}
			j++;
		}
		cout << "*mapGrid[" <<j <<"].id =" << mapGrid[j].id << endl;
		cout << "scelto offset: " << offset << endl;
		int indY = y_index + offset ;
		cout << "indY = " << indY << endl;

		//fisso y_i
		CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &indY, &lb[0], &zeros[0]);
		CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &indY, &ub[0], &zeros[0]); 
	}
		//fisso s_i
		//int indS = s_index + noFlightZones[i] - getUsrsNum();
		//CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &indS, &lb[0], &zeros[0]);
		//CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &indS, &ub[0], &zeros[0]);
		
		//fisso z_i
		//int indZ = z_index + noFlightZones[i] - getUsrsNum();
		//CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &indZ, &lb[0], &zeros[0]);
		//CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &indZ, &ub[0], &zeros[0]);
		
		//fisso o_i
		//int ind_supera = supera_index + noFlightZones[i] - getUsrsNum();
		//CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &ind_supera, &lb[0], &zeros[0]);
		//CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &ind_supera, &ub[0], &zeros[0]);

		//int index = noFlightZones[i] - getUsrsNum();
		//for (int k = 0; k < getCommsNum(); k++)
		//{
			//for (int i = 0; i < getTotalPotentialNodes(); i++)
			//{

				//CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &fMap[index][i][k], &lb[0], &zeros[0]);
				//CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &fMap[i][index][k], &ub[0], &zeros[0]);
			//}
		//}
	//}
	 
	



	cout << "Max physical memory usage at model creation: " << getPeakRSS( ) << " KB" << endl;
	cout << "Current memory usage, post-model: " << getCurrentRSS( ) / 1024 << "KB" << endl;

	/*clearC();
	cout << "*Max physical memory usage at model creation: " << getPeakRSS( ) << " KB" << endl;
	cout << "*Current memory usage, post-model: " << getCurrentRSS( ) / 1024 << "KB" << endl;*/
	
}

void printConflictFile(string clpFile, CEnv env, Prob lp)
{			
	int confnumrows = 0, confnumcols = 0, status = 0;
	cout << "CPXMIP_INFEASIBLE: infeasibility detected." << endl;

	status = CPXrefineconflict(env, lp, &confnumrows, &confnumcols);
	cout << "Number of conflicting rows: " << confnumrows << endl;
	cout << "Number of conflicting columns: " << confnumcols << endl;

	status = CPXclpwrite(env, lp, clpFile.c_str());
	if (status == 0)
		cout << "Conflict file " << clpFile << " created." << endl;
	else
		cerr << __FUNCTION__ << "(): Failed to create " << clpFile << " conflict file." << endl;
}

/*solution_t *getSolution(CEnv env, Prob lp)
{
	r;
}*/

void printSolution(solution_t *solution)
{
	if (solution == NULL)
	{
		cerr << __FUNCTION__ << "(): Soluzione non allocata." << endl;
	}
	else
	{
		int count = 0;
		cout << endl << "Soluzione istanza: " << endl;
		cout << "Valore f. obiettivo: " << solution->objValue << endl;
		cout << "Tempo totale impiegato (ms): " << solution->execTime << endl;
		cout << "Nodi esplorati: " << solution->openedNodes << endl;
		for (int i = 0; i < getTotalPotentialNodes() - getUsrsNum(); i++)
		{
			if (round(solution->yPositions[i]) == 1)
				count++;
		}
		cout << "Droni impiegati/droni totali: " << count << "/" << getDrnsNum() << endl;

		for (int i = 0; i < getTotalPotentialNodes() - getUsrsNum(); i++)
		{
			cout << "y_" << i + getUsrsNum() << ": " << round(solution->yPositions[i]) << endl;
		}
		cout << endl;
	}
}

void printRelaxedSolution(solution_t *solution)
{
	if (solution == NULL)
	{
		cerr << __FUNCTION__ << "(): Soluzione non allocata." << endl;
	}
	else
	{
		int count = 0;
		cout << endl << "*Soluzione istanza: " << endl;
		cout << "*Valore f. obiettivo: " << solution->objValue << endl;
		cout << "*Tempo totale impiegato (ms): " << solution->execTime << endl;
		cout << "*Nodi esplorati: " << solution->openedNodes << endl;
		for (int i = 0; i < getTotalPotentialNodes() - getUsrsNum(); i++)
		{
			if (solution->yPositions[i] > 0)
				count++;
		}
		cout << "*Droni impiegati/droni totali: " << count << "/" << getDrnsNum() << endl;

		for (int i = 0; i < getTotalPotentialNodes() - getUsrsNum(); i++)
		{
			cout << "*y_" << i + getUsrsNum() << ": " << solution->yPositions[i] << endl;
		}
		cout << endl;
	}
}

solution_t *solveLP(CEnv env, Prob lp, string baseFileName, bool verbose, int contRelax[])
{
	cout << "Current memory usage, before post-model: " << getCurrentRSS( ) / 1024 << " KB" << endl;

	string lpFile = baseFileName + ".lp";
	//dichiarazione dei timepoint/epoch (strutture dati in cui porre i tempi di inizio e fine dell'esecuzione di ogni istanza) 
	std::chrono::high_resolution_clock::time_point start, end;
	solution_t *solution = NULL;
	
	// setupFR LP
	setupLP(env, lp, contRelax);
	cout << "Current memory usage, after post-model: " << getCurrentRSS( ) / 1024 << " KB" << endl;
	
	
	//CHECKED_CPX_CALL(CPXwriteprob, env, lp, lpFile.c_str(), NULL);
	
	// optimize

	cout << endl << "--------------------------------------------" << endl << endl;
	
	start = std::chrono::high_resolution_clock::now(); //debug: output timestamp 
	std::time_t timestamp = std::chrono::system_clock::to_time_t(start);
	cout << "Optimization started at: " << std::ctime(&timestamp) << endl;
	
	if(contRelax[0]!= 31) //MIP  LPOPT
	{
		cout << __FUNCTION__ <<"(): Esecuzione di CPXmipopt." << endl;
		start = std::chrono::high_resolution_clock::now(); //timestamp di inizio
			CHECKED_CPX_CALL(CPXmipopt, env, lp);
		end = std::chrono::high_resolution_clock::now(); //timestamp di fine
	}
	else //LP
	{
		swapToCont(env,lp);
		//cout << "qui" << endl;
		cout << __FUNCTION__ << "(): Esecuzione di CPXlpopt." << endl;
		start = std::chrono::high_resolution_clock::now(); //timestamp di inizio
			CHECKED_CPX_CALL(CPXlpopt, env, lp);
		end = std::chrono::high_resolution_clock::now(); //timestamp di fine
		//cout << " quil" << endl;
	}
	/*start = std::chrono::high_resolution_clock::now(); //timestamp di inizio
		CHECKED_CPX_CALL(CPXmipopt, env, lp);
	end = std::chrono::high_resolution_clock::now();*/ //timestamp di fine
	
	timestamp = std::chrono::system_clock::to_time_t(end); //debug: output timestamp 
	cout << "Optimization finished at: " << std::ctime(&timestamp) << endl;
	

	cout << "Istanza eseguita in "
		<< std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() //casting per la conversione in millisecondi; (end-start).count() calcola la differenza di tempo e la restituisce come valore numerico
		<< "ms" << " (" << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << "s c.ca).\n";

	//istanziazione della struttura dati per la soluzione

	try
	{
		double objval = 0;
		const int size = getTotalPotentialNodes();
		double solutionArray[size]; 

		solution = new solution_t;

		CHECKED_CPX_CALL(CPXgetobjval, env, lp, &objval);

		solution->instName = baseFileName;
		solution->objValue = objval;
		solution->statusCode = CPXgetstat(env, lp);
		solution->execTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		solution->openedNodes = CPXgetnodecnt(env,lp);

		double bestobj = 0;
		CPXgetbestobjval(env, lp, &bestobj);

		solution->bestobj = bestobj;

		double cutoff = 0;
		CPXgetcutoff(env, lp, &cutoff);

		solution->cutoff = cutoff;

		double gap = 0;
		CPXgetmiprelgap(env, lp, &gap);

		solution->gap = gap;

		solution->nodeint = CPXgetnodeint(env, lp);

		int cuts = 0;
		int cuttype = 0;
		CPXgetnumcuts(env, lp, cuttype, &cuts);

		solution->cuts = cuts;


		int status = CPXgetx(env, lp, solutionArray, y_index, y_index + getTotalPotentialNodes() - getUsrsNum() - 1);
		if (status == 0)
		{
			solution->yPositions.resize(size - getUsrsNum());
			for (int i = 0; i < size - getUsrsNum() ; i++)
			{
				solution->yPositions[i] = solutionArray[i];
			}
			//solution->yPositions(solutionArray, solutionArray + sizeof(solutionArray) / sizeof (solutionArray[0]) );
		}
		else
		{
			//return vettore vuoto
			solution->yPositions = vector<double>();
		}
	}
	catch (exception &e)
	{
		cerr << __FUNCTION__ << " An exception has occurred: " << e.what() << endl;
		if(solution != NULL)
			delete solution;
		return NULL;
	}

	interfaceToGraphicModule(env, lp, string(baseFileName +".buf.txt").c_str());
	
	//if(verbose == true)	
		printSimplifiedSolFile(env, lp, string(baseFileName + ".var.txt").c_str());
	return solution;
}

solution_t *solveHeurLP(CEnv env, Prob lp, string baseFileName, bool verbose, int contRelax[])
{
	solution_t *instSolution = NULL;
		try
		{
			// risolve MIP con rilassamento continuo
			instSolution = solveLP(env, lp, baseFileName, verbose, contRelax);  
			//CHECKED_CPX_CALL(CPXsolwrite, env, lp, (baseFileName+ ".sol.relaxed").c_str()); //debug, remove after
			
			if(instSolution != NULL)
			{
				cout << "RELAXATION status code: " << instSolution->statusCode << endl; //debug

				if (instSolution->statusCode == 103) //CPXMIP_INFEASIBLE 
				{
					// infeasibility detected
					printConflictFile((baseFileName + ".clp"), env, lp);
					cerr << __FUNCTION__ << "(): Infeasibility, impossibile procedere con la risoluzione euristica. " << endl;
				}
				else
				{
					vector< double > yValue;
					vector< int > yIndexes;
					
					//discretizzazione variabili y con valore di soglia 0.1
					for(unsigned int l = 0; l < instSolution->yPositions.size(); l++)
					{
						cout << instSolution->yPositions[l] << endl;
						
						double rounded = round(instSolution->yPositions[l] * 10.0) / 10.0; // arrotondamento al secondo decimale
						
						if(rounded >= 0.1) // vero -> c'e' un drone
							yValue.push_back(1.0);
						else
							yValue.push_back(0.0);
						
						yIndexes.push_back(gety_index() + l);
						
						cout << "y" << l+getUsrsNum() << " : " << yValue[l] << " " << yIndexes[l] <<endl;
					}
					
					//delete instSolution;
					//instSolution = NULL;
					
					// ottimizza con MIP START

					/*int beg[] = {0};
					int effortlevel[] = {0};
					status = CPXaddmipstarts (env, lp, 1, yValue.size(), beg, &yIndexes[0], &yValue[0], effortlevel, NULL);
					cout << status << endl;
					CPXwritemipstarts(env, lp, (fileName + ".mst").c_str(), 0, 0);
					contRelax[0]=0;
					instSolution = solveLP(env, lp, instance, true, contRelax);*/

					std::chrono::high_resolution_clock::time_point start, end;
					//alternativa: mantieni problema ma cambia bounds di y
					
					vector <char> lb(yIndexes.size(), 'L');
					vector <char> ub(yIndexes.size(), 'U');
					vector <char> newType(yIndexes.size(), 'B');
					vector <double> lowerBounds(yIndexes.size(), 0.0);
					vector <double> upperBounds(yIndexes.size(), 1.0);
					
					CHECKED_CPX_CALL(CPXchgctype, env, lp, yIndexes.size(), &yIndexes[0], &newType[0]);
					
					CHECKED_CPX_CALL(CPXchgbds, env, lp, yIndexes.size(), &yIndexes[0], &lb[0], &lowerBounds[0]);
					CHECKED_CPX_CALL(CPXchgbds, env, lp, yIndexes.size(), &yIndexes[0], &ub[0], &upperBounds[0]);

					start = std::chrono::high_resolution_clock::now(); //debug: output timestamp 
						std::time_t timestamp = std::chrono::system_clock::to_time_t(start);
					cout << "Optimization started at: " << std::ctime(&timestamp) << endl;
					
					if(contRelax[0] != 31) //LPOPT
					{
						start = std::chrono::high_resolution_clock::now(); //timestamp di inizio
							CHECKED_CPX_CALL(CPXmipopt, env, lp);
						end = std::chrono::high_resolution_clock::now(); //timestamp di fine
					}
					else
					{
						//cout << "qui" << endl;
						start = std::chrono::high_resolution_clock::now(); //timestamp di inizio
							CHECKED_CPX_CALL(CPXlpopt, env, lp);
						end = std::chrono::high_resolution_clock::now(); //timestamp di fine
						//cout << " qui1" << endl;
					}
					/*start = std::chrono::high_resolution_clock::now(); //timestamp di inizio
							CHECKED_CPX_CALL(CPXmipopt, env, lp);
						end = std::chrono::high_resolution_clock::now(); *///timestamp di fine
					
					timestamp = std::chrono::system_clock::to_time_t(end); //debug: output timestamp 
					cout << "Optimization finished at: " << std::ctime(&timestamp) << endl;
					

					cout << "Istanza eseguita in "
						<< std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() //casting per la conversione in millisecondi; (end-start).count() calcola la differenza di tempo e la restituisce come valore numerico
						<< "ms" << " (" << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << "s c.ca).\n";

					double objval = 0;
					const int size = getTotalPotentialNodes();
					double solutionArray[size]; 

					//solution = new solution_t;
					//aggiorna la soluzione
					CHECKED_CPX_CALL(CPXgetobjval, env, lp, &objval);

					instSolution->instName = baseFileName;
					instSolution->objValue = objval;
					instSolution->statusCode = CPXgetstat(env, lp);
					instSolution->execTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
					instSolution->yPositions.clear();

					int status = CPXgetx(env, lp, solutionArray, y_index, y_index + getTotalPotentialNodes() - getUsrsNum() - 1);
					if (status == 0)
					{
						instSolution->yPositions.resize(size - getUsrsNum());
						for (int i = 0; i < size - getUsrsNum() ; i++)
						{
							instSolution->yPositions[i] = solutionArray[i];
						}
						//solution->yPositions(solutionArray, solutionArray + sizeof(solutionArray) / sizeof (solutionArray[0]) );
					}
					else
					{
						//return vettore vuoto
						instSolution->yPositions = vector<double>();
					}					
					
					/*if(instSolution != NULL)
					{
						cout << "Status code: " << instSolution->statusCode << endl;

						if (instSolution->statusCode == 103) //CPXMIP_INFEASIBLE 
						{
							// infeasibility detected
							printConflictFile(clpFile, env, lp);
						}
						else
						{
							//printSolutionData(env, lp, y_index, solution);
							if(verbose == true)
							{
								printSimplifiedSolFile(env, lp, solution.c_str());
								printVarsValue(env, lp);
							
								//printNetworkUsage(env, lp);
								//testSolutionFile("refIST3.txt", solution.c_str());
							}

							if (instSolution == NULL)
								cerr << __FUNCTION__ << "(): Impossibile salvare la soluzione dell'istanza risolta." << endl;
							else
							{
								if(verbose == true)
									printSolution(instSolution);
							}

							//if(verbose == true)
								CHECKED_CPX_CALL(CPXsolwrite, env, lp, solFile.c_str());
						}
					}
					else
						cerr << __FUNCTION__ << "(): Impossibile salvare la soluzione dell'istanza risolta." << endl;*/
					// ???
					
					
					if(verbose == true)
					{
						printSimplifiedSolFile(env, lp, (baseFileName + ".txt").c_str());
						printVarsValue(env, lp);
					}
					//CHECKED_CPX_CALL(CPXsolwrite, env, lp, (baseFileName + ".sol").c_str());
				}
			}
			else
				cerr << __FUNCTION__ << "(): Impossibile salvare la soluzione dell'istanza RELAXED risolta." << endl;
		}
		catch (exception& e)
		{
			cerr << __FUNCTION__ << "(): An exception has occurred: " << e.what() << endl;
			if(instSolution != NULL)
				delete instSolution;
			return NULL;
		}
	return instSolution;
}

solution_t *getSolution(CEnv env, Prob lp, unsigned long int execTime, string instanceName)
{
	solution_t *instSolution = NULL;
	try
	{
		instSolution = new solution_t;
		double objval = 0.0;
		const int size = getTotalPotentialNodes();
		double solutionArray[size];

		CHECKED_CPX_CALL(CPXgetobjval, env, lp, &objval);

		instSolution->instName = instanceName;
		instSolution->objValue = objval;
		instSolution->statusCode = CPXgetstat(env, lp);
		instSolution->execTime = execTime;
		instSolution->yPositions.clear();

		instSolution->openedNodes =  CPXgetnodecnt(env, lp);

		int status = CPXgetx(env, lp, solutionArray, gety_index(), gety_index() + getTotalPotentialNodes() - getUsrsNum() - 1);
		if (status == 0)
		{
			instSolution->yPositions.resize(size - getUsrsNum());
			for (int i = 0; i < size - getUsrsNum() ; i++)
			{
				instSolution->yPositions[i] = solutionArray[i];
			}
			//solution->yPositions(solutionArray, solutionArray + sizeof(solutionArray) / sizeof (solutionArray[0]) );
		}
		else
		{
			//return vettore vuoto
			instSolution->yPositions = vector<double>();
		}	
		return instSolution;
	}
	catch (exception& e)
	{
		cerr << __FUNCTION__ << "(): An exception has occurred: " << e.what() << endl;
		if(instSolution != NULL)
			delete instSolution;
		return NULL;
	}		
}

solution_t *getHeurSolution(CEnv env, Prob lp, unsigned long int execTime, string instanceName, int iterations)
{
	solution_t *instSolution = NULL;
	try
	{
		instSolution = new solution_t;
		double objval = 0.0;
		const int size = getTotalPotentialNodes();
		double solutionArray[size];

		CHECKED_CPX_CALL(CPXgetobjval, env, lp, &objval);

		instSolution->instName = instanceName;
		instSolution->objValue = objval;
		instSolution->statusCode = CPXgetstat(env, lp);
		instSolution->execTime = execTime;
		instSolution->yPositions.clear();

		instSolution->heurIterationCount = iterations;

		int status = CPXgetx(env, lp, solutionArray, gety_index(), gety_index() + getTotalPotentialNodes() - getUsrsNum() - 1);
		if (status == 0)
		{
			instSolution->yPositions.resize(size - getUsrsNum());
			for (int i = 0; i < size - getUsrsNum() ; i++)
			{
				instSolution->yPositions[i] = solutionArray[i];
			}
			//solution->yPositions(solutionArray, solutionArray + sizeof(solutionArray) / sizeof (solutionArray[0]) );
		}
		else
		{
			//return vettore vuoto
			instSolution->yPositions = vector<double>();
		}	
		return instSolution;
	}
	catch (exception& e)
	{
		cerr << __FUNCTION__ << "(): An exception has occurred: " << e.what() << endl;
		if(instSolution != NULL)
			delete instSolution;
		return NULL;
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
		int nCols = CPXgetnumcols(env, lp);

		varVals.resize(nCols);

		cout << "Visualizzazione delle variabili: " << endl;
		CHECKED_CPX_CALL(CPXgetx, env, lp, &varVals[0], 0, nCols - 1);
		for (int i = 0; i < nCols; i++)
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


int interfaceToGraphicModule(CEnv env, Prob lp, string fileName)
{
	ofstream file;
	file.open(fileName,ios::out);
	if(file.is_open())
	{
		const int size = getTotalPotentialNodes() - getUsrsNum();
		double solutionArray[size];		

		int status = CPXgetx(env, lp, solutionArray, y_index, y_index + size - 1);
		if (status == 0)
		{
			//stampa dimensioni griglia
			file << getGridL() << endl;
			file << getGridH() << endl;
			file << getGridStep() << endl;
	
			//stampa dei nodi utente

			file << getUsrsNum() << endl;
			for(int i=0; i< getUsrsNum(); i++)
			{
				if(mapGrid[i].isUser == true)
				{
					file << mapGrid[i].id << " " << "U" << endl;
				}
			}

			//stampa dei nodi drone
			int count = 0;
			for (int i = 0; i < size; i++)
			{
				if(round(solutionArray[i]) == 1)
					count++;
			}

			file << count << endl;
			for (int i = 0; i < size; i++)
			{
				if(round(solutionArray[i]) == 1)
					file << mapGrid[i+getUsrsNum()].id << " " << "D" << endl;
			}

			//stampa degli archi (solo utenti/droni e droni/droni)

			//utenti-droni
			for(int i = 0; i < getUsrsNum(); i++)
			{
				for (int j = getUsrsNum(); j < getTotalPotentialNodes(); j++)
				{
					int yOffset = j - getUsrsNum();
					if (mapGrid[i].id != mapGrid[j].id && isInRange(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y, getNodeRadius()))
					{
						if (round(solutionArray[yOffset]) == 1)
						{
							file << mapGrid[i].id << " " << mapGrid[j].id << endl;
						}
					}
				}
			}

			//droni-droni
			for(int i = getUsrsNum(); i < getTotalPotentialNodes(); i++)
			{
				for (int j = getUsrsNum(); j < getTotalPotentialNodes(); j++)
				{
					int yOffsetS = i - getUsrsNum();
					int yOffsetD = j - getUsrsNum();
					if (mapGrid[i].id != mapGrid[j].id && isInRange(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y, getNodeRadius()))
					{
						if (round(solutionArray[yOffsetS]) == 1 && round(solutionArray[yOffsetD]) == 1)
						{
							file << mapGrid[i].id << " " << mapGrid[j].id << endl;
						}
					}
				}
			}

			//stampa degli archi (tutti)
			/*for(int i = 0; i < getTotalPotentialNodes(); i++)
			{
				for(int j = 0; j < getTotalPotentialNodes(); j++)
				{
					if(i != j)
					{
						if(isInRange(mapGrid[i].x,mapGrid[i].y,mapGrid[j].x, mapGrid[j].y, getNodeRadius()))
							file << mapGrid[i].id << " " << mapGrid[j].id << endl;
					}
				}
			}*/
			
			file.close();
			return 0;
		}
		else
		{
			cerr << __FUNCTION__ << "(): Impossibile accedere alle variabili del problema." << endl;
			return 1;
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): Impossibile creare il file " << fileName << endl;
		return 1;
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
				//
				/// to get variable name, use the RATHER TRICKY "CPXgetcolname"
				status = CPXgetcolname(env, lp, cur_colname, cur_colnamestore, cur_storespace, surplus, i, i);
				if (status == 0)
				{
					double zero = 0.0;
					
					if (round(varVals[i]) == -0.0) //risolve il problema del -0
						unitFile << cur_colnamestore << " : " << zero << endl;
					else
						unitFile << cur_colnamestore << " : " << round(varVals[i]) << endl;
				}
				else
				{
					unitFile << "Var in position " << i << " : " << round(varVals[i]) << endl;
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

void swapToInt(CEnv env, Prob lp)
{
	cout << "Swapping problem configuration from LP to MIP." << endl;
	double lb = -1;
	int status = CPXgetlb(env, lp, &lb, f_index , f_index );
	cout <<"LB f LP: " << lb << endl;

	CPXchgprobtype (env, lp, CPXPROB_MILP);//LPOPT

	lb = -1;
	status = CPXgetlb(env, lp, &lb, f_index , f_index );
	cout <<"LB f MIP: " << lb << endl;
	
	vector <char> ynewType(numyVars, 'B');
	vector <char> xnewType(numxVars, 'B');
	vector <char> znewType(numzVars, 'B');
	vector <char> superanewType(numsuperaVars, 'B');
	
	CHECKED_CPX_CALL(CPXchgctype, env, lp, numyVars, &yIDX[0], &ynewType[0]);
	CHECKED_CPX_CALL(CPXchgctype, env, lp, numxVars, &xIDX[0], &xnewType[0]);
	CHECKED_CPX_CALL(CPXchgctype, env, lp, numzVars, &zIDX[0], &znewType[0]);
	CHECKED_CPX_CALL(CPXchgctype, env, lp, numsuperaVars, &superaIDX[0], &superanewType[0]);
	
	/*vector <double> ones(max(xIDX.size(), fIDX.size()), 1.0); //LPOPT
	vector <double> zeros(max(xIDX.size(), fIDX.size()), 0.0);
	vector <char> lb(max(xIDX.size(), fIDX.size()), 'L');
	vector <char> ub(max(xIDX.size(), fIDX.size()), 'U');
	
	//binaries 
	CHECKED_CPX_CALL(CPXchgbds, env, lp, yIDX.size(), &yIDX[0], &lb[0], &zeros[0]);
	CHECKED_CPX_CALL(CPXchgbds, env, lp, yIDX.size(), &yIDX[0], &ub[0], &ones[0]);
	
	
	CHECKED_CPX_CALL(CPXchgbds, env, lp, xIDX.size(), &xIDX[0], &lb[0], &zeros[0]);
	CHECKED_CPX_CALL(CPXchgbds, env, lp, xIDX.size(), &xIDX[0], &ub[0], &ones[0]);
	
	CHECKED_CPX_CALL(CPXchgbds, env, lp, zIDX.size(), &zIDX[0], &lb[0], &zeros[0]);
	CHECKED_CPX_CALL(CPXchgbds, env, lp, zIDX.size(), &zIDX[0], &ub[0], &ones[0]);
	
	CHECKED_CPX_CALL(CPXchgbds, env, lp, superaIDX.size(), &superaIDX[0], &lb[0], &zeros[0]);
	CHECKED_CPX_CALL(CPXchgbds, env, lp, superaIDX.size(), &superaIDX[0], &ub[0], &ones[0]);
	
	//LB of continuous
	CHECKED_CPX_CALL(CPXchgbds, env, lp, fIDX.size(), &fIDX[0], &lb[0], &zeros[0]);
	
	CHECKED_CPX_CALL(CPXchgbds, env, lp, sIDX.size(), &sIDX[0], &lb[0], &zeros[0]);*/

}

void swapToCont(CEnv env, Prob lp)
{
	/*vector <char> ynewType(numyVars, 'C');
	vector <char> xnewType(numxVars, 'C');
	vector <char> znewType(numzVars, 'C');
	vector <char> superanewType(numsuperaVars, 'C');
			
	CHECKED_CPX_CALL(CPXchgctype, env, lp, numyVars, &yIDX[0], &ynewType[0]);
	CHECKED_CPX_CALL(CPXchgctype, env, lp, numxVars, &xIDX[0], &xnewType[0]);
	CHECKED_CPX_CALL(CPXchgctype, env, lp, numzVars, &zIDX[0], &znewType[0]);
	CHECKED_CPX_CALL(CPXchgctype, env, lp, numsuperaVars, &superaIDX[0], &superanewType[0]);*/
	cout << "Swapping problem configuration from MIP to LP." << endl;
	double lb = -1;
	int status = CPXgetlb(env, lp, &lb, f_index , f_index );
	cout <<"LB f MIP: " << lb << endl;

	CPXchgprobtype (env, lp, CPXPROB_LP) ;//LPOPT 

	lb = -1;
	status = CPXgetlb(env, lp, &lb, f_index , f_index );
	cout <<"LB f LP: " << lb << endl;

	vector <double> zeros(fIDX.size(), 0);
	vector <char> lbl(max(xIDX.size(), fIDX.size()), 'L');
	CHECKED_CPX_CALL(CPXchgbds, env, lp, fIDX.size(), &fIDX[0], &lbl[0], &zeros[0]);

	lb = -1;
	status = CPXgetlb(env, lp, &lb, f_index , f_index );
	cout <<"LB f forced: " << lb << endl;
}

int dronesCount(solution_t *sol, double threshold)
{
	int counter = 0;
	for(int i = 0; i < (int) sol->yPositions.size(); i++)
	{
		if(sol->yPositions[i] >= threshold) //qui le soluzioni sono intere
		{
			counter++;
		}
	}
	return counter;
}

solution_t *copySolution(solution_t *sol)
{
	try
	{
		solution_t *newSol = new solution_t;

		newSol->instName = sol->instName;
		newSol->yPositions = sol->yPositions;
		newSol->objValue = sol->objValue;
		newSol->statusCode = sol->statusCode;
		newSol->execTime = sol->execTime;
		newSol->openedNodes = sol->openedNodes;
		newSol->rootGap = sol->rootGap;
		newSol->heurIterationCount = sol->heurIterationCount;

		return newSol;
	}
	catch (exception& e)
	{
		cerr << __FUNCTION__ << "(): An exception has occurred: " << e.what() << endl;
		
		return NULL;
	}
}
