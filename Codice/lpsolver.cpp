#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <chrono>
#include <algorithm>
#include "cpxmacro.h"
#include "lpsolver.h"
#include "utility.h"

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

int bigM = 1000; // TODO: bigM deve essere maggiore della capacità massima dei link

int getBigM()
{
	return bigM;
}

static int estimateBigM() //TODO migliore stima + set variabile bigm
{
	return max(2*getPosNum(),3*getRXCapacity());
}

static int getSolutionFlowMatrix(CEnv env, Prob lp, vector< vector< vector<int> > > &flow)
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
}

static bool areOutOfSight(int i, int j)
{
	int k = 0;
	bool flag = true;
	if (c.size() > 0) //c e' gia' stato allocato?
	{
		while (k < getCommsNum() && flag == true)
		{
			if (c[i][j][k] <= getThreshold())
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

static void setupLP(CEnv env, Prob lp)
{

	const double zero = 0.0;
	const double uno = 1.0;

	//allocazione mappe
	vector< vector< vector<int> > > fMap(getTotalPotentialNodes(), vector< vector<int> >(getTotalPotentialNodes(), vector<int>(getCommsNum(), -1)));

	vector< vector<int> > xMap(getTotalPotentialNodes(), vector<int>(getTotalPotentialNodes(), -1));

	//vector< vector<int> > wMap(getTotalPotentialNodes(), vector<int>(getTotalPotentialNodes(), -1));

	vector< vector<int> > zMap(getPosNum(), vector<int>(getPosNum(), -1));

	

	//aggiunta delle variabili f_i_j_k, una per volta, con i!=j
	//nota: la variabile f_i_j_k e' distinta dalla variabile f_j_i_k
	int fMapIndex = 0;
	for (int k = 0; k < getCommsNum(); k++)
	{
		for (int i = 0; i < getTotalPotentialNodes(); i++)
		{
			for (int j = 0; j < getTotalPotentialNodes(); j++)
			{

				//if (i != j) //c'è un arco tra i nodi i e j, quindi creo la corrispondente variabile di flusso fijk
				if (i != j && (c[i][j][k] <= getThreshold() && u[i][j]>0) && !(i < getUsrsNum() && j < getUsrsNum())) //***
				//if (i != j && (c[i][j][k] <= threshold ) && !(i < n && j < n)) //*** TODO: senza le uij, va rivisto
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

	for (int i = 0; i < getPosNum(); i++)
	{
		char ytype = 'B';
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
			if (!areOutOfSight(i, j) && u[i][j]>0) //***
												   //if (!areOutOfSight(i, j)) //***
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
	for (int i = getUsrsNum(); i < getTotalPotentialNodes(); i++)
	{
		for (int j = 0; j < getTotalPotentialNodes(); j++)
		{
			//esclude le variabili x_i_j con i==j
			//if(i!=j)
			if (i != j && (!areOutOfSight(i, j) && u[i][j]>0)) //***
															   //if (i != j && (!areOutOfSight(i, j))) //***
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
	/*int w_index = CPXgetnumcols(env, lp);
	int wMapIndex = w_index;
	for (int i = 0; i < getTotalPotentialNodes(); i++)
	{
		char wtype = 'I'; //***
		double lb = 0.0;
		double ub = CPX_INFBOUND;
		for (int j = 0; j < getTotalPotentialNodes(); j++)
		{
			//if (i != j && !(i < n && j < n) && u[i][j] != 0)
			if (i != j && !(i < getUsrsNum() && j < getUsrsNum()) && u[i][j] > 0) //***
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
	*/
	//aggiunta variabili z_i
	int z_index = CPXgetnumcols(env, lp);
	//int zMapIndex = z_index;
	for (int i = 0; i < getTotalPotentialNodes(); i++)
	{
		char ztype = 'B'; 
		double lb = 0.0;
		double ub = 1.0;
		
		snprintf(name, NAME_SIZE, "z_%d", i); //scrive il nome della variabile z_i_j sulla stringa name[]
		char* zname = (char*)(&name[0]);
		
		CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &zero, &lb, &ub, &ztype, &zname);   //costruisce la singola variabile 

		//costruzione della mappa
		//zMap[i] = zMapIndex; //TODO: sistemare gli indici
		//zMapIndex++;
		
		
	}

	cout << "Sono state create " << CPXgetnumcols(env, lp) - z_index << " variabili z_i_j" << endl;  //numero totale delle vars create

	//vincoli

	char sense;
	int matbeg;
	vector<int> idx;
	vector<double> coef;

	int rowNumber = 0;

	// 2. conservazione flusso
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

			snprintf(name, NAME_SIZE, "c2_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;


			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &b[v][k], &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
			idx.clear();
			coef.clear();

		}
	}
	cout << "Vincoli (2) creati\n" << endl;

	// 3. legame tra variabili x_i_j e y_j
	sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int j = getUsrsNum(); j < getPosNum() + getUsrsNum(); j++)
	{
		for (int i = 0; i < getTotalPotentialNodes(); i++)
		{
			//if (i != j && graph[i][j] != 0)
			if (i != j && xMap[i][j] != -1) //***
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
				idx.clear();
				coef.clear();
			}
		}
	}

	cout << "Vincoli (3) creati\n" << endl;

	// 4. un drone non puo' mantenere piu' di s potenziali connessioni simultanee
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

			snprintf(name, NAME_SIZE, "c4_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;

			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &temp_s, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
			idx.clear();
			coef.clear();
		}
	}
	cout << "Vincoli (4) creati\n" << endl;


	// 5. non posizionare piu' di d droni
	double temp_d = (double)getDrnsNum();
	sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();
	for (int i = 0; i < getPosNum(); i++)
	{
		idx.push_back(y_index + i);
		coef.push_back(1.0);
	}

	snprintf(name, NAME_SIZE, "c5_%d", rowNumber); //numerazione progressiva dei vincoli
	char* rowname = (char*)(&name[0]);
	rowNumber++;

	CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &temp_d, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
	idx.clear();
	coef.clear();

	cout << "Vincoli (5) creati\n" << endl;

	// 6. legame tra variabili c_i_j_k e f_i_j_k
	//REWORKED
	sense = 'E';
	for (int i = 0; i < getTotalPotentialNodes(); i++)
	{
		for (int j = 0; j < getTotalPotentialNodes(); j++)
		{
			//TODO: nota, se si creano solo le vars fijk tc graph[i][j]!=0, questo vincolo perde significato e non genera nuove righe
			if (i != j && !(j < getUsrsNum() && i < getUsrsNum())) //esclude i link i cui estremi sono entrambi nodi utenti 
			{
				for (int k = 0; k < getCommsNum(); k++)
				{
					if (c[i][j][k] > getThreshold() && fMap[i][j][k] != -1) //***
					{
						double coef = 1.0;
						int idx = fMap[i][j][k];

						snprintf(name, NAME_SIZE, "c6_%d", rowNumber); //numerazione progressiva dei vincoli
						char* rowname = (char*)(&name[0]);
						rowNumber++;

						CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, 1, &zero, &sense, &matbeg, &idx, &coef, NULL, &rowname);
					}
				}
			}
		}
	}
	cout << "Vincoli (6) creati\n" << endl;

	// 7. legame tra le variabili x_i_j e f_i_j_k
	//REWORKED
	sense = 'L';
	matbeg = 0;
	idx.clear();
	coef.clear();
	for (int i = 0; i < getTotalPotentialNodes(); i++)
	{
		for (int j = 0; j < getTotalPotentialNodes(); j++)
		{
			//if (i != j) //***
			//{
			//if (!(j < n && i < n)) //esclude i link i cui estremi sono entrambi nodi utenti //***
			//{
			//cout << i << " " << j << " passa" << endl;
			for (int k = 0; k < getCommsNum(); k++)
			{
				if (fMap[i][j][k] != -1) //***
				{


					idx.push_back(fMap[i][j][k]);
					coef.push_back(1.0);

					// f_i_j_k - M * x_i_j
					idx.push_back(xMap[i][j]);
					coef.push_back(-bigM);

					snprintf(name, NAME_SIZE, "c7_%d", rowNumber); //numerazione progressiva dei vincoli
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
	cout << "Vincoli (7) creati\n" << endl;

	// 0.9.3 version //
	//1.a.1 gestione interferenza (RX)
	 

	//sommatoria f_i_j_k (1)
	sense = 'L'; 
	matbeg = 0;
	coef.clear();
	idx.clear();
	
	double e, reductionFactor;
	for (int i = getUsrsNum(); i < getTotalPotentialNodes(); i++) // P
	{
		double rhs;
		for (int j = 0; j < getTotalPotentialNodes(); j++)  // V'
		{
			for (int k = 0; k < getCommsNum(); k++)  // K
			{
				if (i != j && fMap[j][i][k] != -1)
				{
					idx.push_back(fMap[j][i][k]);
					coef.push_back(1.0);
				}
			}
		}

		//prodotto URX*yi (2) + prodotto bigM*yi (5)
		idx.push_back(y_index + i - getUsrsNum());
		coef.push_back(-getRXCapacity() + getBigM());

		//riduzione capacita' canale prot. slotted ALHOA (3)
		e = exp(1);
		reductionFactor = getRXCapacity() * (e-1) / e; //TODO: check

		idx.push_back(z_index + i);
		coef.push_back(reductionFactor);

		//riduzione capacita' canale mutualmente esclusiva (4)
		double cumulateRHS = 0;
		for(int j = 0; j < getPosNum(); j++)
		{
			double distance = getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y);
			if(i != j && distance >= getEpsilonNodeRadius() && distance <= getNodeRadius())
			{
				cumulateRHS += getDistanceCoef(j,i);
			}
		}

		idx.push_back(z_index + i);
		coef.push_back(- cumulateRHS);

		snprintf(name, NAME_SIZE, "c1_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		rhs= getBigM() -cumulateRHS;
		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();

	}

	// 1.a.2 gestione interferenza (RX)

	//sommatoria f_i_j_k (1)
	sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();
	
	for (int i = 0; i < getUsrsNum(); i++) // V
	{
		double rhs = 0; 
		for (int j = getUsrsNum(); j < getTotalPotentialNodes(); j++)  // P
		{
			for (int k = 0; k < getCommsNum(); k++)  // K
			{
				if (i != j && fMap[j][i][k] != -1)
				{
					idx.push_back(fMap[j][i][k]);
					coef.push_back(1.0);
				}
			}
		} 

		//riduzione capacita' canale prot. slotted ALHOA (2)
		e = exp(1);
		reductionFactor = getRXCapacity() * (e-1) / e; //TODO: check

		idx.push_back(z_index + i);
		coef.push_back(reductionFactor);

		//riduzione capacita' canale mutualmente esclusiva (3)
		double cumulateRHS = 0;
		for(int j = getUsrsNum(); j < getTotalPotentialNodes(); j++)
		{
			double distance = getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y);
			if(i != j && distance >= getEpsilonNodeRadius() && distance <= getNodeRadius())
			{
				cumulateRHS += getDistanceCoef(j,i);
				idx.push_back(z_index + i);
				coef.push_back(-cumulateRHS);
			}
		}

		snprintf(name, NAME_SIZE, "c1_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		rhs = getRXCapacity() - cumulateRHS;

		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();

	}

	//cout << "Vincoli (10) creati\n" << endl;

	//1.b AUX
	matbeg = 0;
	coef.clear();
	idx.clear();

	int rEps = getEpsilonNodeRadius();
	for(int i = 0; i < getTotalPotentialNodes(); i++)
	{
		for(int j = getUsrsNum(); j< getTotalPotentialNodes(); j++)
		{
			if( i != j && getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[j].x, mapGrid[j].y) <= rEps)
			{
				idx.push_back(y_index + j - getUsrsNum());
				coef.push_back(1.0);				
			} 
		}

		if(coef.size() > 0)
		{
			sense = 'G';

			idx.push_back(z_index + i);
			coef.push_back(-1.0);

			snprintf(name, NAME_SIZE, "c1_%d", rowNumber); //numerazione progressiva dei vincoli
			char* rowname = (char*)(&name[0]);
			rowNumber++;

			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);

			//AUX 2
			sense = 'L';

			//cambio solo coeff zi
			coef[1] = -1.0 * getBigM();
			
			snprintf(name, NAME_SIZE, "c1_%d", rowNumber); //numerazione progressiva dei vincoli
			rowname = (char*)(&name[0]);
			rowNumber++;

			CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		}	
		idx.clear();
		coef.clear();
	}

	//1.c gestione interferenza (TX)
	sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();

	for(int i = 0; i < getTotalPotentialNodes(); i++)  // V'
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
		for(int v = getUsrsNum(); v < getTotalPotentialNodes(); v++)
		{
			if(i != v && getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[v].x, mapGrid[v].y) <= getNodeRadius())
			{
				idx.push_back(y_index + v - getUsrsNum());
				coef.push_back(getInterferenceFactor(i,v));
			}
		}

		// somma fattori interferenza utenti per V (3)
		double rhs = 0;
		for(int v = 0; v < getUsrsNum(); v++)
		{
			if(i != v && getDistance(mapGrid[i].x, mapGrid[i].y, mapGrid[v].x, mapGrid[v].y) <= getNodeRadius())
			{
				rhs += getInterferenceFactor(i,v);
			}
		}

		rhs = getTXCapacity() - rhs; 

		snprintf(name, NAME_SIZE, "c1_%d", rowNumber); //numerazione progressiva dei vincoli
		char* rowname = (char*)(&name[0]);
		rowNumber++;

		CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);
		idx.clear();
		coef.clear();
	}


	//OLD
	//TX 
	//10.a
	/*sense = 'L';
	matbeg = 0;
	//coef.clear();
	//idx.clear();
	for (int i = getUsrsNum(); i < getTotalPotentialNodes(); i++)
	{
		vector< int > idxTx, idxRx;
		vector< double > coefTx, coefRx;
		//first sum A*f
		for (int j = 0; j < getTotalPotentialNodes(); j++)
		{
			for (int k = 0; k < getCommsNum(); k++)
			{
				if (i != j && fMap[i][j][k] != -1)
				{
					idxTx.push_back(fMap[i][j][k]);
					coefTx.push_back(getDistanceCoef(i, j));

					idxRx.push_back(fMap[j][i][k]);
					coefRx.push_back(getDistanceCoef(j, i));
				}
			}
		}

		//second sum z*B
		for (int l = getUsrsNum(); l< getTotalPotentialNodes(); l++)
		{
			if (l != i)
			{
				idxTx.push_back(zMap[i - getUsrsNum()][l - getUsrsNum()]); //TODO: gestire meglio gli indici
				coefTx.push_back(getInterferenceFactor(i, l));

				idxRx.push_back(zMap[l - getUsrsNum()][i - getUsrsNum()]); //TODO: gestire meglio gli indici
				coefRx.push_back(getInterferenceFactor(l, i));
			}
		}

		//second sum 
		//for(int l=n; l< totalPotentialNodes; l++)
		//{
		//	if(l!=i)
		//	{
		//		for(int j=0; j< totalPotentialNodes; j++)
		//		{
		//			for(int k=0; k< K; k++)
		//			{
		//				idx.push_back(fMap[l][j][k]);
		//				coef.push_back(getInterferenceFactor(i,l));
		//			}
		//		}
		//	}
		//}	

		//third sum

		double sumBTx = 0;
		double sumBRx = 0;
		for (int v = 0; v< getUsrsNum(); v++)
		{
			sumBTx += getInterferenceFactor(i, v);

			sumBRx += getInterferenceFactor(v, i);
		}

		//idx.push_back(y_index + i - n);
		//coef.push_back(sumB);

		if (idxTx.size() > 0)
		{
			//add Utx*yi
			idxTx.push_back(y_index + i - getUsrsNum());
			coefTx.push_back(-getTXCapacity() + sumBTx);


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

		if (idxRx.size() > 0)
		{
			idxRx.push_back(y_index + i - getUsrsNum());
			coefRx.push_back(-getTXCapacity() + sumBRx);

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

	for (int i = 0; i < getUsrsNum(); i++)
	{
		double temp_uTx = getTXCapacity();
		double temp_uRx = getRXCapacity();

		vector< int > idxTx, idxRx;
		vector< double > coefTx, coefRx;

		//first sum
		for (int j = getUsrsNum(); j < getTotalPotentialNodes(); j++)
		{
			for (int k = 0; k < getCommsNum(); k++)
			{
				if (i != j && fMap[i][j][k] != -1)
				{
					idxTx.push_back(fMap[i][j][k]);
					coefTx.push_back(getDistanceCoef(i, j));

					idxRx.push_back(fMap[j][i][k]);
					coefRx.push_back(getDistanceCoef(j, i));

				}
			}
		}

		//second sum 
		for (int l = getUsrsNum(); l< getTotalPotentialNodes(); l++)
		{
			if (l != i)
			{
				idxTx.push_back(y_index + l - getUsrsNum());
				coefTx.push_back(getInterferenceFactor(i, l));

				idxRx.push_back(y_index + l - getUsrsNum());
				coefRx.push_back(getInterferenceFactor(l, i));
			}
		}

		//second sum
		//for(int l=n; l< totalPotentialNodes; l++)
		//{
		//	if(l!=i)
		//	{
		//		for(int j=0; j< totalPotentialNodes; j++)
		//		{
		//			for(int k=0; k< K; k++)
		//			{
		//				idx.push_back(fMap[l][j][k]);
		//				coef.push_back(getInterferenceFactor(i,l));
		//			}
		//		}
		//	}
		//}

		//third sum
		//double sumB = 0;
		//for(int v=0; v< n; v++)
		//{
		//	sumB+= getInterferenceFactor(i,v);
		//}

		if (idxTx.size() > 0)
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

		if (idxRx.size() > 0)
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


	}*/
	//cout << "Vincoli (10) creati\n" << endl;

	//11. ausiliario
	/*sense = 'L';
	matbeg = 0;
	coef.clear();
	idx.clear();

	for (int i = getUsrsNum(); i<getTotalPotentialNodes(); i++)
	{
		for (int l = getUsrsNum(); l<getTotalPotentialNodes(); l++)
		{
			if (i != l)
			{
				idx.push_back(y_index + i - getUsrsNum());
				coef.push_back(1.0);

				idx.push_back(y_index + l - getUsrsNum());
				coef.push_back(1.0);

				idx.push_back(zMap[i - getUsrsNum()][l - getUsrsNum()]); //TODO: sistema indici
				coef.push_back(-1.0);

				snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
				char* rowname = (char*)(&name[0]);
				rowNumber++;



				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &uno, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);

				idx.clear();
				coef.clear();

				//Auxiliary constraint 1: zl - xi <= 0
				idx.push_back(zMap[i - getUsrsNum()][l - getUsrsNum()]); //TODO: sistema indici
				coef.push_back(1.0);
				idx.push_back(y_index + i - getUsrsNum());
				coef.push_back(-1.0);

				snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
				rowname = (char*)(&name[0]);
				rowNumber++;


				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);

				idx.clear();
				coef.clear();

				//Auxiliary constraint 2: zl - xl <= 0
				idx.push_back(zMap[i - getUsrsNum()][l - getUsrsNum()]); //TODO: sistema indici
				coef.push_back(1.0);
				idx.push_back(y_index + l - getUsrsNum());
				coef.push_back(-1.0);

				snprintf(name, NAME_SIZE, "c_%d", rowNumber); //numerazione progressiva dei vincoli
				rowname = (char*)(&name[0]);
				rowNumber++;


				CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &zero, &sense, &matbeg, &idx[0], &coef[0], NULL, &rowname);

				idx.clear();
				coef.clear();

			}
		}
	}*/
}


//si utilizza il tool esterno DOT per creare un'immagine del grafo. Per fare cio' prima si codifica il grafo nel formato dot su un file, e poi si lancia automaticamente il tool
int printDOTGraph(string filename)
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
		if (c.size() <= 0)
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
						/*
						Per ogni arco trovato, si stampa una riga su file che codifica l'arco e i due nodi suoi estremi,
						nel formato: idNodo1 -> idNodo2. arrowhead="none" permette di rappresentare l'arco come non orientato, mentre
						l'attributo xlabel permette di etichettare l'arco con una stringa, in questo caso il tipo di vincolo ( = o >= )
						e il valore wij corrispondente.
						*/
						file << "\t" << i << " -> " << j << " [xlabel=\"=" << c[i][j][0] << "\"];\n";
						//fprintf(file,"\t%d -> %d [arrowhead=\"none\", xlabel=\"=%d\"];\n",i,j,c[i][j][0]);		

					}
				}
			}
		}
		file << "}" << endl;
		return 0;
	}
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
		for (int i = 0; i < getPosNum(); i++)
		{
			if (round(solution->yPositions[i]) == 1)
				count++;
		}
		cout << "Droni impiegati/droni totali: " << count << "/" << getDrnsNum() << endl;

		for (int i = 0; i < getPosNum(); i++)
		{
			cout << "y_" << i + getUsrsNum() << ": " << round(solution->yPositions[i]) << endl;
		}
		cout << endl;
	}
}

solution_t *solveLP(CEnv env, Prob lp, string baseFileName)
{

	string lpFile = baseFileName + ".lp";
	//dichiarazione dei timepoint/epoch (strutture dati in cui porre i tempi di inizio e fine dell'esecuzione di ogni istanza) 
	std::chrono::high_resolution_clock::time_point start, end;
	solution_t *solution = NULL;
	// setup LP
	setupLP(env, lp);
				
	CHECKED_CPX_CALL(CPXwriteprob, env, lp, lpFile.c_str(), NULL);
	// optimize

	cout<<"Exec" <<endl;
	start = std::chrono::high_resolution_clock::now(); //timestamp di inizio
		CHECKED_CPX_CALL(CPXmipopt, env, lp);
	end = std::chrono::high_resolution_clock::now(); //timestamp di fine
	cout<<"end exec" << endl;


	std::cout << "Istanza eseguita in "
		<< std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() //casting per la conversione in millisecondi; (end-start).count() calcola la differenza di tempo e la restituisce come valore numerico
		<< "ms" << " (" << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << "s c.ca).\n";
	cout << "--------------------------------------------" << endl;

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

		int status = CPXgetx(env, lp, solutionArray, y_index, y_index + getPosNum() - 1);
		if (status == 0)
		{
			solution->yPositions.resize(size);
			for (int i = 0; i < size; i++)
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
		cout << __FUNCTION__ << " An exception has occurred: " << e.what() << endl;
		if(solution != NULL)
			delete solution;
		return NULL;
	}
	printDOTGraph(string(baseFileName+".dot")); //TODO: togli
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


void printNetworkUsage(CEnv env, Prob lp) //TODO: una volta eliminati gli u, bisogna riadattarla con Utx/Urx
{
	vector< vector< vector<int> > > flow(getTotalPotentialNodes(), vector< vector<int> >(getTotalPotentialNodes(), vector<int>(getCommsNum(), -1)));
	getSolutionFlowMatrix(env, lp, flow);
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

