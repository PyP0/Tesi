#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <chrono>
#include <ctime>
#include <algorithm>
#include "cpxmacro.h"
#include "lpsolver.h"
#include "utility.h"
#include "getRSS.h"

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
int w_index = 0;
int z_index = 0;

int bigM = 1000000; //default value


/*static int getSolutionFlowMatrix(CEnv env, Prob lp, vector< vector< vector<int> > > &flow)
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


					if (indexes[0] >= 0 && indexes[0] < getTotalPotentialNodes() && indexes[1] >= 0 && indexes[1] < getTotalPotentialNodes() && indexes[2] >= 0 && indexes[2] < getCommsNum())
					{
						flow[indexes[0]][indexes[1]][indexes[2]] = (int)varVals[v];
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
}*/

static bool areOutOfSight(int i, int j) //TODO: controllarne effettiva funzionalita'
{
	int k = 0;
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
	return flag;
}

static void setupLP(CEnv env, Prob lp, int contRelax[])
{

	const double zero = 0.0;
	const double uno = 1.0;

	cout << "Current memory usage, pre-model: " << getCurrentRSS( ) / 1024 << "KB" << endl;
	//allocazione mappe
	vector< vector< vector<int> > > fMap(getTotalPotentialNodes(), vector< vector<int> >(getTotalPotentialNodes(), vector<int>(getCommsNum(), -1)));

	vector< vector<int> > xMap(getTotalPotentialNodes(), vector<int>(getTotalPotentialNodes(), -1));

	//vector< vector<int> > wMap(getTotalPotentialNodes(), vector<int>(getTotalPotentialNodes(), -1));

	vector< vector<int> > zMap(getPosNum(), vector<int>(getPosNum(), -1));

	

	//aggiunta delle variabili f_i_j_k, una per volta, con i!=j
	//nota: la variabile f_i_j_k e' distinta dalla variabile f_j_i_k
	int fMapIndex = 0;
	char ftype; 
	
	if( (contRelax[0] & 16) != 16) //rilassamento continuo?
		ftype = 'I';
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
				if (i != j && (getCost(i,j,k) <= getThreshold()) && !(i < getUsrsNum() && j < getUsrsNum())) //***
				{		
					double lb = 0.0;
					double ub = CPX_INFBOUND;
					
					double cost = getCost(i,j,k);
					
					snprintf(name, NAME_SIZE, "f_%d_%d_%d", i, j, k); //scrive il nome della variabile f_i_j_k sulla stringa name[]
					char* fname = (char*)(&name[0]);
					CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &cost, &lb, &ub, &ftype, &fname);   //costruisce la singola variabile 

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

	for (int i = 0; i < getPosNum() - getUsrsNum(); i++)
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
	cout << "Sono state create " << CPXgetnumcols(env, lp) - y_index << " variabili y_i" << endl; //numero totale delle vars create	

	//aggiunta delle variabili x_i_j

	//si divide in due step la creazione delle variabili x_i_j
	//variabili relative agli "utenti"

	x_index = CPXgetnumcols(env, lp);
	int xMapIndex = x_index;

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
					if( i == 0) //debug
						cout << "Rilassamento continuo su variabili x_i_j" << endl;
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
					if( i == 0) //debug
						cout << "Rilassamento continuo su variabili x_i_j" << endl;
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

	cout << "Sono state create " << CPXgetnumcols(env, lp) - x_index << " variabili x_i_j" << endl; //numero totale delle vars create	

	
	//aggiunta variabili z_i
	int z_index = CPXgetnumcols(env, lp);
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

	cout << "Sono state create " << CPXgetnumcols(env, lp) - z_index << " variabili z_i" << endl;  //numero totale delle vars create


	//aggiunta variabili s_i
	int s_index = CPXgetnumcols(env, lp);
	for (int i = 0; i < getTotalPotentialNodes(); i++)
	{
		char stype = 'C'; 

		double lb = 0.0;
		double ub = CPX_INFBOUND;
		
		snprintf(name, NAME_SIZE, "s_%d", i); //scrive il nome della variabile s_i sulla stringa name[]
		char* sname = (char*)(&name[0]);
		
		CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &zero, &lb, &ub, &stype, &sname);   //costruisce la singola variabile 
	}

	cout << "Sono state create " << CPXgetnumcols(env, lp) - s_index << " variabili s_i" << endl;  //numero totale delle vars create


	//aggiunta variabili supera_i
	int supera_index = CPXgetnumcols(env, lp);
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

	cout << "Sono state create " << CPXgetnumcols(env, lp) - supera_index << " variabili sp_i" << endl;  //numero totale delle vars create


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

	// 1.0 addition //

	double e, reductionFactor;
	e = exp(1);
	reductionFactor = getRXCapacity() * ( (2 * e) - 1) / (2 * e);
	
	cout << "Smax parameter: " << reductionFactor << endl;
	

	// 1
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

		// sconto s_i
		idx.push_back(s_index + i);
		coef.push_back(1.0);

		snprintf(name, NAME_SIZE, "c1_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}	
	//cout << "Vincoli (1) creati\n";

	// 1.a

	sense = 'L'; 
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

		snprintf(name, NAME_SIZE, "c1_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;


		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}
	//cout << "Vincoli (1.a) creati\n";

	// 2

	sense = 'G'; 
	matbeg = 0;
	coef.clear();
	idx.clear();
	for( int i = 0; i < getTotalPotentialNodes(); i++)  // V'
	{
		double rhs = 0.0;
		// sconto
		idx.push_back(s_index + i);
		coef.push_back(1.0);

		// fattore riduzione subordinato a area critica (epsilon)
		idx.push_back(z_index + i);
		coef.push_back(-reductionFactor);

		//1.1 TODO: test per relazionare le zi con le yi
		// M(1-y_i)
		/*if( i >= getUsrsNum()) // P
		{
			bigM = 2 * reductionFactor;
			// - M(1-y_i)
			idx.push_back(y_index + i -getUsrsNum());
			coef.push_back(-bigM);
			rhs = - bigM;

		}*/

		snprintf(name, NAME_SIZE, "c2_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;


		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}
	//cout << "Vincoli (2) creati\n";
	
	//1.1 TODO: test
	//2.b
	/*sense = 'L';  
	matbeg = 0;
	coef.clear();
	idx.clear();
	for( int i = 0; i < getTotalPotentialNodes(); i++)  // V'
	{
		// sconto
		idx.push_back(s_index + i);
		coef.push_back(1.0);

		snprintf(name, NAME_SIZE, "c2_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;


		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &reductionFactor, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}*/

	// 3

	bigM = 2 * reductionFactor; //set the big M value for each specific constraint
	sense = 'G'; 
	matbeg = 0;
	coef.clear();
	idx.clear();

	
	for( int i = getUsrsNum(); i < getTotalPotentialNodes(); i++) // P
	{
		double rhs = 0;

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
				coef.push_back(-getDistanceCoef(j,i));
			}
		}

		// sum Aji * y_j su V

		for(int j = 0; j < getUsrsNum(); j++) // V
		{
			double distance = getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y);
			if(i != j && distance > getEpsilonNodeRadius() && distance <= getNodeRadius()) //R_epsilon excluded
			{
				rhs+= getDistanceCoef(j,i);
			}
		}

		// -M(supera_i + 1 - y_i)

		idx.push_back(supera_index + i);
		coef.push_back(bigM);


		rhs-= bigM;

		idx.push_back(y_index + i - getUsrsNum());
		coef.push_back(- bigM);


		snprintf(name, NAME_SIZE, "c3_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}
	//cout << "Vincoli (3) creati\n";

	// 3.a

	bigM = 2 * reductionFactor; //set the big M value for each specific constraint
	sense = 'G'; 
	matbeg = 0;
	coef.clear();
	idx.clear();
	
	for( int i = 0; i < getUsrsNum(); i++) // V
	{
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
				coef.push_back(-getDistanceCoef(j,i));
			}
		}

		// -M(supera_i)

		idx.push_back(supera_index + i);
		coef.push_back(bigM);

		snprintf(name, NAME_SIZE, "c3_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;


		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}
	//cout << "Vincoli (3.a) creati\n";

	// 4 

	bigM = 2 * reductionFactor; //set the big M value for each specific constraint
	sense = 'L'; 
	matbeg = 0;
	coef.clear();
	idx.clear();

	for(int i = getUsrsNum(); i < getTotalPotentialNodes(); i++)  // P
	{
		double rhs = 0;

		// sum Aji * y_j su P
		for(int j = getUsrsNum(); j < getTotalPotentialNodes(); j++)  // P
		{
			double distance = getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y);
			if(i != j && distance > getEpsilonNodeRadius() && distance <= getNodeRadius()) //R_epsilon excluded
			{
				idx.push_back(y_index + j - getUsrsNum());
				coef.push_back(getDistanceCoef(j,i));
			}
		}

		// sum Aji su V

		for(int j = 0; j < getUsrsNum(); j++) // V
		{
			double distance = getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y);
			if(i != j && distance > getEpsilonNodeRadius() && distance <= getNodeRadius()) //R_epsilon excluded
			{
				rhs-= getDistanceCoef(j,i);
			}
		}

		rhs+= reductionFactor;

		// M(supera_i + 1 - y_i)

		idx.push_back(supera_index + i);
		coef.push_back(-bigM);

		rhs+= bigM;

		idx.push_back(y_index + i - getUsrsNum());
		coef.push_back( bigM);


		snprintf(name, NAME_SIZE, "c4_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();

	}
	//cout << "Vincoli (4) creati\n";

	// 4.a 

	bigM = 2 * reductionFactor; //set the big M value for each specific constraint
	sense = 'L'; 
	matbeg = 0;
	coef.clear();
	idx.clear();

	for(int i = 0; i < getUsrsNum(); i++)  // V
	{
		double rhs = 0;

		// sum Aji * y_j su P
		for(int j = getUsrsNum(); j < getTotalPotentialNodes(); j++)  // P
		{
			double distance = getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y);
			if(i != j && distance > getEpsilonNodeRadius() && distance <= getNodeRadius()) //R_epsilon excluded
			{
				idx.push_back(y_index + j - getUsrsNum());
				coef.push_back(getDistanceCoef(j,i));
			}
		}


		rhs= reductionFactor;

		// M(supera_i)

		idx.push_back(supera_index + i);
		coef.push_back(-bigM);

		//rhs+= bigM;
		//idx.push_back(y_index + i);
		//coef.push_back( bigM);

		snprintf(name, NAME_SIZE, "c4_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();

	}
	//cout << "Vincoli (4.a) creati\n";

	// 5 

	bigM = 2 * reductionFactor; //set the big M value for each specific constraint
	sense = 'G';
	matbeg = 0;
	coef.clear();
	idx.clear();
	
	for(int i = getUsrsNum(); i < getTotalPotentialNodes(); i++) // P
	{
		double rhs = 0;
		
		// sconto_i
		idx.push_back(s_index + i);
		coef.push_back(1.0);

		// sconto * supera_i
		idx.push_back(supera_index + i);
		coef.push_back(-reductionFactor);

		// -M(1-y_i)
		rhs-= bigM;

		idx.push_back(y_index + i - getUsrsNum());
		coef.push_back(-bigM);

		snprintf(name, NAME_SIZE, "c5_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}

	//1.1 TODO: test rimozione -M(...)
	/*for(int i = getUsrsNum(); i < getTotalPotentialNodes(); i++) // P
	{
		// sconto_i
		idx.push_back(s_index + i);
		coef.push_back(1.0);

		// sconto * supera_i
		idx.push_back(supera_index + i);
		coef.push_back(-reductionFactor);

		snprintf(name, NAME_SIZE, "c5_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}*/
	//cout << "Vincoli (5) creati\n";

	// 5.a

	sense = 'G';  
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

		snprintf(name, NAME_SIZE, "c5_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}
	//cout << "Vincoli (5.a) creati\n";

	
	//1.1 TODO: test  IF_si >= smax THEN_spi=1-y_i
	// 5.b
	/*bigM = 2 * reductionFactor; //set the big M value for each specific constraint
	sense = 'L'; 
	matbeg = 0;
	coef.clear();
	idx.clear();
	for(int i = 0; i < getTotalPotentialNodes(); i++) // V'
	{
		double epsilon = 0.0001 ; 
		double rhs = reductionFactor - epsilon;
		idx.push_back(s_index + i);
		coef.push_back(1.0);
		
		idx.push_back(supera_index + i);
		coef.push_back(-bigM);
		
		snprintf(name, NAME_SIZE, "c5_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}*/
	
	// 6

	bigM = 2 * getTotalPotentialNodes(); //set the big M value for each specific constraint
	matbeg = 0;
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
				// M(z_i + 1 - y_i)

				idx.push_back(z_index + i);
				coef.push_back(-bigM);

				rhs= bigM;

				idx.push_back(y_index + i - getUsrsNum());
				coef.push_back(bigM);

				snprintf(name, NAME_SIZE, "c6_%d", rowNumber); //numerazione progressiva dei vincoli
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

			snprintf(name, NAME_SIZE, "c6_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;

			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
			idx.clear();
			coef.clear();

			//1.1 TODO: test  zi == 1
			/*sense = 'E'; 
			matbeg = 0;
			coef.clear();
			idx.clear();
			
			idx.push_back(z_index + i);
			coef.push_back(1.0);

			snprintf(name, NAME_SIZE, "c6_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;

			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &uno, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
			idx.clear();
			coef.clear();*/

		}
	}
	//cout << "Vincoli (6) creati\n";
	
	// TODO: test
	bigM = 5*getTotalPotentialNodes(); //set the big M value for each specific constraint
	sense = 'L'; 
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
			idx.push_back(z_index + i);
			coef.push_back(-bigM);
		
			snprintf(name, NAME_SIZE, "c6_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;

			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
			idx.clear();
			coef.clear();
		}
	}
	
	//TODO: Test legame tra yi e spi
	/*sense = 'G'; 
	matbeg = 0;
	coef.clear();
	idx.clear();
	for(int i = getUsrsNum(); i < getTotalPotentialNodes(); i++) // P
	{
		idx.push_back(y_index + i - getUsrsNum());
		coef.push_back(1.0);

		idx.push_back(supera_index + i);
		coef.push_back(-1.0);
		
		snprintf(name, NAME_SIZE, "c6_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
		
	}*/
	
	//TODO: test "if si >= smax then spi =1-y_i
	/*sense = 'G'; 
	matbeg = 0;
	coef.clear();
	idx.clear();
	for(int i = 0; i < getTotalPotentialNodes(); i++) // V'
	{
		bigM = 2*reductionFactor; //TODO
		
		idx.push_back(s_index + i );
		coef.push_back(1.0);

		idx.push_back(supera_index + i);
		coef.push_back(- bigM);
		
		snprintf(name, NAME_SIZE, "cX_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &reductionFactor, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
		
	}*/

	//7. gestione interferenza (TX)

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
				coef.push_back(getInterferenceFactor(i,v));
			}
		}

		// somma fattori interferenza utenti per V (3)
		double rhs = 0;
		for(int v = 0; v < getUsrsNum(); v++)  //  V
		{
			if(i != v && getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[v].x, mapGrid[v].y) <= getNodeRadius())
			{
				rhs += getInterferenceFactor(i,v);
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

	//7.a

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
	}
	//cout << "Vincoli (7.a) creati\n";

	// 8. conservazione flusso
	sense = 'E';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int v = 0; v < getTotalPotentialNodes(); v++)
	{
		for (int k = 0; k < getCommsNum(); k++)
		{
			for (int i = 0; i < getTotalPotentialNodes(); i++)
			{
				//if (i != v && graph[i][v] != 0)
				//if (i != v && !(v < n && i < n)) //condizione più generale, include le fijk di nodi tra cui non ci sono link
				if (i != v && fMap[i][v][k] != -1) //*** a meno della presenza di nodi isolati, ci sarà per ogni vincolo almeno una coppia di f tc fmap[][][]!=-1
				{
					idx.push_back(fMap[i][v][k]);
					coef.push_back(1.0);
				}
			}

			for (int j = 0; j < getTotalPotentialNodes(); j++)
			{
				//if (v != j && graph[v][j] != 0)
				if (v != j && fMap[v][j][k] != -1) //***condizione più generale, include le fijv di nodi tra cui non ci sono link
				{
					idx.push_back(fMap[v][j][k]);
					coef.push_back(-1.0);
				}
			}

			snprintf(name, NAME_SIZE, "c8_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;


			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &b[v][k], &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
			idx.clear();
			coef.clear();

		}
	}
	//cout << "Vincoli (8) creati\n";

	// 9. legame tra variabili x_i_j e y_j
	sense = 'L';
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

				snprintf(name, NAME_SIZE, "c9_%d", rowNumber); //numerazione progressiva dei vincoli
				char* rowname = (char*)(&name[0]);
				rowNumber++;

				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);

				//add x_j_i
				idx[0] = xMap[j][i];
				

				snprintf(name, NAME_SIZE, "c9_%d", rowNumber); //numerazione progressiva dei vincoli
				rowname = (char*)(&name[0]);
				rowNumber++;

				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
				idx.clear();
				coef.clear();
			}
		}
	}

	//cout << "Vincoli (9) creati\n";

	// 10. un drone non puo' mantenere piu' di s potenziali connessioni simultanee
	sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int v = getUsrsNum(); v < getTotalPotentialNodes(); v++)
	{
		double temp_s = (double)getMaxConnections();
		for (int i = 0; i < getTotalPotentialNodes(); i++)
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

			snprintf(name, NAME_SIZE, "c10_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;

			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &temp_s, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
			idx.clear();
			coef.clear();
		}
	}
	//cout << "Vincoli (10) creati\n";


	// 11. non posizionare piu' di d droni
	double temp_d = (double)getDrnsNum();
	sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int i = 0; i < getTotalPotentialNodes() - getUsrsNum(); i++)
	{
		idx.push_back(y_index + i);
		coef.push_back(1.0);
	}

	snprintf(name, NAME_SIZE, "c11_%d", rowNumber); //numerazione progressiva dei vincoli
	char* rowname = (char*)(&name[0]);
	rowNumber++;

	CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &temp_d, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
	idx.clear();
	coef.clear();

	//cout << "Vincoli (11) creati\n";

	// 6. legame tra variabili c_i_j_k e f_i_j_k //TODO: rimuovere
	sense = 'E';
	for (int i = 0; i < getTotalPotentialNodes(); i++)
	{
		for (int j = 0; j < getTotalPotentialNodes(); j++)
		{
			if (i != j && !(j < getUsrsNum() && i < getUsrsNum())) //esclude i link i cui estremi sono entrambi nodi utenti 
			{
				for (int k = 0; k < getCommsNum(); k++)
				{
					if (getCost(i,j,k) > getThreshold() && fMap[i][j][k] != -1) //***
					{
						double coef = 1.0;
						int idx = fMap[i][j][k];

						snprintf(name, NAME_SIZE, "c6a_%d", rowNumber); //numerazione progressiva dei vincoli
						char* rowname = (char*)(&name[0]);
						rowNumber++;

						CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, 1, &zero, &sense, &matbeg, &idx, &coef, NULL, &rowname);
					}
				}
			}
		}
	}
	//cout << "Vincoli (6) creati\n";

	// 12. Legame tra le variabili f_i_j_k e x_i_j

	bigM = max(getRXCapacity(),getTXCapacity()); //set the big M value for each specific constraint
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

				snprintf(name, NAME_SIZE, "c12_%d", rowNumber); //numerazione progressiva dei vincoli
				char* rowname = (char*)(&name[0]);
				rowNumber++;
	
				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
			}
			idx.clear();
			coef.clear();
		}
	}

	//extra: test
	/*sense = 'E';
	matbeg = 0;
	idx.clear();
	coef.clear();

	idx.push_back(y_index + 9);
	coef.push_back(1.0);

	snprintf(name, NAME_SIZE, "cE_%d", rowNumber); //numerazione progressiva dei vincoli
	rowname = (char*)(&name[0]);
	rowNumber++;
	
	CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &uno, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
	idx.clear();
	coef.clear();

	//extra: test
	sense = 'E';
	matbeg = 0;
	idx.clear();
	coef.clear();

	idx.push_back(y_index + 11);
	coef.push_back(1.0);

	snprintf(name, NAME_SIZE, "cE_%d", rowNumber); //numerazione progressiva dei vincoli
	rowname = (char*)(&name[0]);
	rowNumber++;
	
	CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &uno, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
	idx.clear();
	coef.clear();

	//extra: test
	sense = 'E';
	matbeg = 0;
	idx.clear();
	coef.clear();

	idx.push_back(y_index + 15);
	coef.push_back(1.0);

	snprintf(name, NAME_SIZE, "cE_%d", rowNumber); //numerazione progressiva dei vincoli
	rowname = (char*)(&name[0]);
	rowNumber++;
	
	CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &uno, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
	idx.clear();
	coef.clear();

	//extra: test
	sense = 'E';
	matbeg = 0;
	idx.clear();
	coef.clear();

	idx.push_back(y_index + 19);
	coef.push_back(1.0);

	snprintf(name, NAME_SIZE, "cE_%d", rowNumber); //numerazione progressiva dei vincoli
	rowname = (char*)(&name[0]);
	rowNumber++;
	
	CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &uno, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
	idx.clear();
	coef.clear();

	//extra: test
	sense = 'E';
	matbeg = 0;
	idx.clear();
	coef.clear();

	idx.push_back(y_index + 23);
	coef.push_back(1.0);

	snprintf(name, NAME_SIZE, "cE_%d", rowNumber); //numerazione progressiva dei vincoli
	rowname = (char*)(&name[0]);
	rowNumber++;
	
	CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &uno, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
	idx.clear();
	coef.clear();

	//extra: test
	sense = 'E';
	matbeg = 0;
	idx.clear();
	coef.clear();

	idx.push_back(y_index + 25);
	coef.push_back(1.0);

	snprintf(name, NAME_SIZE, "cE_%d", rowNumber); //numerazione progressiva dei vincoli
	rowname = (char*)(&name[0]);
	rowNumber++;
	
	CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &uno, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
	idx.clear();
	coef.clear();*/


	//cout << "Vincoli (12) creati\n" << endl;

	//test MIP start
	/*
	if(CPXsetintparam(env, CPX_PARAM_ADVIND, 1) != 0) 
		cerr << __FUNCTION__ << "(): Impossibile settare MIP start." << endl;

	int beg[] = {0};
	int mcnt = 1;
	int nzcnt = 1;
	int varindices[] = {y_index + 23};
	double values[] = {1.0};
	int effortlevel[] = {CPX_MIPSTART_SOLVEMIP};

	

	if (CPXaddmipstarts(env, lp, mcnt, nzcnt, beg, varindices, values, effortlevel, NULL) != 0)
		cerr << __FUNCTION__ << "(): Errore MIP start" << endl;
	*/

	cout << "Max physical memory usage at model creation: " << getPeakRSS( ) << " KB" << endl;
	cout << "Current memory usage, post-model: " << getCurrentRSS( ) / 1024 << "KB" << endl;

	/*clearC();
	cout << "*Max physical memory usage at model creation: " << getPeakRSS( ) << " KB" << endl;
	cout << "*Current memory usage, post-model: " << getCurrentRSS( ) / 1024 << "KB" << endl;*/
	
}


//si utilizza il tool esterno DOT per creare un'immagine del grafo. Per fare cio' prima si codifica il grafo nel formato dot su un file, e poi si lancia automaticamente il tool
/*int printDOTGraph(string filename)
{
	ofstream file;
	file.open(filename, ios::out);
	if (!file.is_open())
	{
		cerr << __FUNCTION__ << "(): Errore apertura del file " << filename << endl;
		return 1;
	}
	else
	{
		if (isCEmpty() == true)
		{
			cerr << __FUNCTION__ << "(): Grafo vuoto." << endl;
			return 1;
		}
		else //visita della matrice
		{
			file << "digraph G{\n"; //intestazione file .dot
			for (unsigned int i = 0; i< c.size(); i++)
			{
				for (unsigned int j = 0; j< c[0].size(); j++)
				{
					if (i != j && !areOutOfSight(i, j))
					{
						
						//Per ogni arco trovato, si stampa una riga su file che codifica l'arco e i due nodi suoi estremi,
						//nel formato: idNodo1 -> idNodo2. arrowhead="none" permette di rappresentare l'arco come non orientato, mentre
						//l'attributo xlabel permette di etichettare l'arco con una stringa, in questo caso il tipo di vincolo ( = o >= )
						//e il valore wij corrispondente.
						
						file << "\t" << i << " -> " << j << " [xlabel=\"=" << getCost(i,j,0) << "\"];\n";
						//fprintf(file,"\t%d -> %d [arrowhead=\"none\", xlabel=\"=%d\"];\n",i,j,c[i][j][0]);		

					}
				}
			}
		}
		file << "}" << endl;
		return 0;
	}
}*/

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
		for (int i = 0; i < getPosNum() - getUsrsNum(); i++)
		{
			if (round(solution->yPositions[i]) == 1)
				count++;
		}
		cout << "Droni impiegati/droni totali: " << count << "/" << getDrnsNum() << endl;

		for (int i = 0; i < getPosNum() - getUsrsNum(); i++)
		{
			cout << "y_" << i + getUsrsNum() << ": " << round(solution->yPositions[i]) << endl;
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
	
	// setup LP
	setupLP(env, lp, contRelax);
	cout << "Current memory usage, after post-model: " << getCurrentRSS( ) / 1024 << " KB" << endl;
	
	if(verbose == true)	
		CHECKED_CPX_CALL(CPXwriteprob, env, lp, lpFile.c_str(), NULL);
	// optimize

	cout << endl << "--------------------------------------------" << endl << endl;
	
	start = std::chrono::high_resolution_clock::now(); //debug: output timestamp 
	std::time_t timestamp = std::chrono::system_clock::to_time_t(start);
	cout << "Optimization started at: " << std::ctime(&timestamp) << endl;
	
	start = std::chrono::high_resolution_clock::now(); //timestamp di inizio
		CHECKED_CPX_CALL(CPXmipopt, env, lp);
	end = std::chrono::high_resolution_clock::now(); //timestamp di fine
	
	timestamp = std::chrono::system_clock::to_time_t(end); //debug: output timestamp 
	cout << "Optimization finished at: " << std::ctime(&timestamp) << endl;
	

	cout << "Istanza eseguita in "
		<< std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() //casting per la conversione in millisecondi; (end-start).count() calcola la differenza di tempo e la restituisce come valore numerico
		<< "ms" << " (" << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << "s c.ca).\n";

	//istanziazione della struttura dati per la soluzione

	try
	{
		double objval = 0;
		const int size = getPosNum();
		double solutionArray[size]; 

		solution = new solution_t;

		CHECKED_CPX_CALL(CPXgetobjval, env, lp, &objval);

		solution->instName = baseFileName;
		solution->objValue = objval;
		solution->statusCode = CPXgetstat(env, lp);
		solution->execTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

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

