/**
* @file modelv1.2.cpp
* @brief
*/

#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <chrono>
#include <utility>
#include <algorithm>

#include "heuristic.h"
#include "instance.h"
#include "lpsolver.h"
#include "utility.h"
#include "cpxmacro.h"

using namespace std;

double evaluate(solution_t *currentSol)
{
	double objF = 0.0;

	for(unsigned int i = 0; i < currentSol->yPositions.size(); i++)
	{
		objF+= currentSol->yPositions[i] * getDeployCost(i);
	}

	return objF;
}

solution_t *startingSolution(CEnv env, Prob lp, string instanceName, int contRelax[])
{
	// risolve MIP con rilassamento continuo
	bool verbose = false;
	//double threshold = 0.1;

	solution_t *instSolution = NULL;
	//heurSolution_t *firstSolution = NULL;

	instSolution = solveLP(env, lp, instanceName, verbose, contRelax);
	if(instSolution != NULL)
	{
		if (instSolution->statusCode == 103) //CPXMIP_INFEASIBLE 
		{
			// infeasibility detected
			printConflictFile((instanceName + ".clp"), env, lp);
			cerr << __FUNCTION__ << "(): Infeasibility, impossibile procedere con la risoluzione euristica. " << endl;
			delete instSolution;
			return NULL;
		}
		else
		{
			return instSolution;
			/*vector< double > yValue;
			vector< int > yIndexes;
			
			try
			{
				//firstSolution = new heurSolution_t;

				//discretizzazione variabili y con valore di soglia 0.1
				for(unsigned int i = 0; i < instSolution->yPositions.size(); i++)
				{
					//cout << instSolution->yPositions[i] << endl;
					
					double rounded = round(instSolution->yPositions[i] * 10.0) / 10.0; // arrotondamento al secondo decimale
					
					if(rounded >= threshold) // vero -> c'e' un drone
						yValue.push_back(1.0);
					else
						yValue.push_back(0.0);
					
					yIndexes.push_back(gety_index() + i);
					
					//cout << "y" << i + getUsrsNum() << " : " << yValue[i] << " " << yIndexes[i] <<endl;
				}

				firstSolution->instName = instanceName;
				firstSolution->objValue = instSolution->objValue;
				firstSolution->yPositions.assign(yValue.begin(), yValue.end());
				
				//debug
				for(unsigned int i = 0; i < yValue.size(); i++)
				{
					cout << yValue[i] << endl;
				}
				cout << endl;
				for(unsigned int i = 0; i < firstSolution->yPositions.size(); i++)
				{
					cout << firstSolution->yPositions[i] << endl;
				}
				cout << endl;


				delete instSolution;
				return firstSolution; 
				return instSolution;
			}
			catch (exception &e)
			{
				cerr << __FUNCTION__ << " An exception has occurred: " << e.what() << endl;
				if(instSolution != NULL)
					delete instSolution;
				return NULL;
			}*/
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): Nessuna soluzione trovata per il rilassamento continuo dell'istanza " << instanceName << endl;
		return NULL;
	}
}

solution_t *executeHeurInstance(string fileName, bool verbose, int contRelax[])
{
	solution_t *instSolution = NULL;
	
	// init

	DECL_ENV(env);
	DECL_PROB(env, lp);
	
	if ( CPXreadcopyparam(env, "CPLEXparams.prm") != 0)
		cerr << __FUNCTION__ << "(): Impossibile leggere i parametri CPLEX dal file di configurazione, si usera' la configurazione standard." << endl;
	
	string baseFileName (fileName);
	string instance = baseFileName;
	string solution = baseFileName + ".txt";
	string solFile = baseFileName + ".sol";
	string clpFile = baseFileName + ".clp";

	string logFile = baseFileName + ".log";
	CPXFILEptr logF = CPXfopen(logFile.c_str(),"w"); 
	CPXsetlogfile(env,logF);

	if (loadInstance(instance.c_str()) != 0)
	{
		cerr << __FUNCTION__ <<" Impossibile caricare l'istanza: " << instance << endl;
		return NULL;
	}
	else
	{
		cout << "Instance " << instance << " loaded." << endl;
		if(verbose == true)
		{
			
			cout << "Users: " << getUsrsNum() << endl;
			cout << "Drones: " << getDrnsNum() << endl;
			cout << "Potential drones positions: " << getPosNum() << endl << endl;
			//printNodesIOFlow();
			printTrafficMatrix();
		}

		try
		{
			instSolution = solveHeurLP(env, lp, instance, verbose, contRelax);
			if(instSolution != NULL)
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

			// free
			CPXfreeprob(env, &lp);
			CPXcloseCPLEX(&env);
		}
		catch (exception& e)
		{
			cerr << __FUNCTION__ << "(): An exception has occurred: " << e.what() << endl;
			if(instSolution != NULL)
				delete instSolution;
			return NULL;
		}
	}
	CPXfclose(logF);
	return instSolution;
}

static bool pairDecrSort(pair< int,double > i, pair< int,double > j)
{
	return (i.second > j.second);
}

solution_t *solve2(CEnv env, Prob lp, string instance, bool verbose)
{
	int contRelax[] = {31};
	std::chrono::high_resolution_clock::time_point start, end;

	vector< double > firstSolutionArray(getTotalPotentialNodes() - getUsrsNum());
	vector< pair< int,double > > Y(getTotalPotentialNodes() - getUsrsNum()); //insieme delle y_i. Coppia indice variabile - valore
	
	vector <char> lb(1, 'L');
	vector <char> ub(1, 'U');
	vector <double> ones(1, 1.0);
	vector <double> zeros(1, 0.0);
		
	bool stop = false;

	start = std::chrono::high_resolution_clock::now(); //timestamp di inizio

	solution_t *firstSolution = startingSolution(env, lp, instance, contRelax); //kick-start: soluzione iniziale
	if(firstSolution == NULL)
	{
		cerr << __FUNCTION__ << "(): Nessuna soluzione trovata per il rilassamento continuo dell'istanza " << instance << endl;
		return NULL;
	}
	else //starting solution found
	{
		solution_t *solution = NULL;
		int Yiterator = 0; //prossima y_i che verra' scelta
		int Ychosen = -1; // y_i attuale
		
		if(verbose == true)	
		{
			cout << "Soluzione iniziale trovata:" << endl;
			printRelaxedSolution(firstSolution);
		}
		
		int status = CPXgetx(env, lp, &firstSolutionArray[0], gety_index(), gety_index() + getTotalPotentialNodes() - getUsrsNum() - 1);
		if(status != 0)
		{
			cerr << __FUNCTION__ << "(): Nessuna soluzione trovata per il rilassamento continuo dell'istanza " << instance << endl;
			delete firstSolution;
			return NULL;
		}
		
		//riempimento insieme Y
		for(unsigned int i = 0; i < Y.size(); i++)
		{
			Y[i] = make_pair(i,firstSolutionArray[i]);
		}
		//debug

		/*for(unsigned int i = 0; i < Y.size(); i++)
		{
			cout << Y[i].first << " " << Y[i].second << endl;
		}*/
		
		//ordinamento decrescente di Y
		sort(Y.begin(),Y.end(), pairDecrSort);
		
		//debug
		/*cout << endl;
		for(unsigned int i = 0; i < Y.size(); i++)
		{
			cout << Y[i].first << " " << Y[i].second << endl;
		}*/
		
		//scelgo primo valore
		Ychosen = 0;
		Yiterator++;
		
		if(verbose == true)	
			cout << "Ho scelto indice " << Ychosen << ", cioe' var y_" << Y[Ychosen].first << " con valore: " << Y[Ychosen].second << endl;
		
		//setto a 1 la y_i scelta
		while( (stop != true) && (Yiterator <= (int)Y.size()) )
		{
			int YchosenIndex = gety_index() + Y[Ychosen].first;   
			CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &YchosenIndex, &lb[0], &ones[0]);
			CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &YchosenIndex, &ub[0], &ones[0]);
			
			//seconda versione: tutte le altre y a 0.0
			for(unsigned int i = Yiterator; i < Y.size(); i++ )
			{
				YchosenIndex = gety_index() + Y[i].first;
				CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &YchosenIndex, &lb[0], &zeros[0]);
				CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &YchosenIndex, &ub[0], &zeros[0]);
			}
			
			if(verbose == true)	
				CPXwriteprob( env, lp, (instance + "." + to_string(Y[Ychosen].first) + ".cont.lp").c_str(), NULL);
			
			//esegue LP rilassato
			CPXmipopt(env, lp); 
			int optStat = CPXgetstat(env, lp);
			
			if(optStat == 101 || optStat == 102) //soluzione rilassata trovata
			{
				if(verbose == true)	
				{
					cout << "Soluzione rilassata trovata con: ";
					for(int i = 0; i < Yiterator; i++)
					{
						cout << "y_" << Y[i].first << " ";
					}
					cout << endl;
				}
				//esegui LP intero
				if(verbose == true)	
					CPXwriteprob( env, lp, (instance + ".cont.lp").c_str(), NULL);
				
				swapToInt(env, lp);

				if(verbose == true)	
					CPXwriteprob( env, lp, (instance + ".int.lp").c_str(), NULL);
				
				CPXmipopt(env, lp);
				
				int linOptStat = CPXgetstat(env, lp);
				if(linOptStat == 101 || linOptStat == 102) //soluzione intera trovata: stop
				{
					stop = true;
					end = std::chrono::high_resolution_clock::now(); //timestamp di fine
		
					if(verbose == true)	
						{cout << "Soluzione intera trovata con ";
						for(int i = 0; i < Yiterator; i++)
						{
							cout << "y_" << Y[i].first << " ";
						}
						cout << endl;
					}
					
					//aggiorna la soluzione
					solution = getSolution(env, lp, std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(), instance);
					if(solution == NULL)
					{
						cerr << __FUNCTION__ << "(): Impossibile recuperare la soluzione dell'istanza " << instance << endl;
						return NULL;
					}
					if(solution->yPositions.size() == 0)
					{
						cerr << __FUNCTION__ << "(): Impossibile recuperare la soluzione dell'istanza " << instance << endl;
						delete solution;
						return NULL;
					}
					
					if(verbose == true)	
						CPXsolwrite(env, lp, (instance + ".int.sol").c_str());
				}
				else //non c'e' soluzione intera, aggiungi nuova y
				{
					if(verbose == true)	
					{
						cout << "No soluzione intera con ";
						for(int i = 0; i < Yiterator; i++)
						{
							cout << "y_" << Y[i].first << " ";
						}
						cout << endl;
					}
					
					swapToCont(env,lp);
					
					Ychosen = Yiterator;
					Yiterator++;

					if(verbose == true)	
						cout << "Ho scelto indice " << Ychosen << ", cioe' var y_" << Y[Ychosen].first << " con valore: " << Y[Ychosen].second << endl;
				}
			}
			else // non c'e' soluzione rilassata, aggiungi nuova y
			{
				if(verbose == true)	
				{
					cout << "No soluzione rilassata con ";
					for(int i = 0; i < Yiterator; i++)
					{
						cout << "y_" << Y[i].first << " ";
					}
					cout << endl;
				}
				
				Ychosen = Yiterator;
				Yiterator++;

				if(verbose == true)	
					cout << "Ho scelto indice " << Ychosen << ", cioe' var y_" << Y[Ychosen].first << " con valore: " << Y[Ychosen].second << endl;
			}
		}
		if(stop == false)
		{
			cout << __FUNCTION__ << "(): Soluzione non trovata per l'istanza: " << instance << endl;
			return NULL;
		}
		else
		{
			return solution;
		}
	}
}

solution_t *solve(CEnv env, Prob lp, string instance, bool verbose)
{
	int contRelax[] = {31};
	std::chrono::high_resolution_clock::time_point start, end;

	bool stop = false;

	double threshold = 1.0;
	double step = 0.01;
	double accstep = 0.005;
	double stopValue = step;



	vector< int > yIndexes;
	vector< double > solutionArray(getTotalPotentialNodes() - getUsrsNum());
	vector< double > firstSolutionArray(getTotalPotentialNodes() - getUsrsNum());

	vector <char> lb(1, 'L');
	vector <char> ub(1, 'U');
	vector <double> ones(1, 1.0);
	vector <double> zeros(1, 0.0);

	start = std::chrono::high_resolution_clock::now(); //timestamp di inizio

	bool firstFound = false;

	solution_t *firstSolution = startingSolution(env, lp, instance, contRelax); //kick-start: soluzione iniziale
	if(firstSolution == NULL)
	{
		cerr << __FUNCTION__ << "(): Nessuna soluzione trovata per il rilassamento continuo dell'istanza " << instance << endl;
		return NULL;
	}
	else //solution found
	{
		if(verbose == true)
		{
			cout << "Soluzione iniziale trovata:" << endl;
			printRelaxedSolution(firstSolution);
		}

		solution_t *solution = NULL;

		//recupero array y
		int status = CPXgetx(env, lp, &firstSolutionArray[0], gety_index(), gety_index() + getTotalPotentialNodes() - getUsrsNum() - 1);
		if(status != 0)
		{
			cerr << __FUNCTION__ << "(): Nessuna soluzione trovata per il rilassamento continuo dell'istanza " << instance << endl;
			delete firstSolution;
			return NULL;
		}

		while(stop != true)
		{
			//elaboro array y
			if(verbose == true)
			{
				cout << "--------------------------------------------------------------------" <<endl;
				cout << "Threshold: " << threshold << endl;
			}
			for(unsigned int l = 0; l < firstSolutionArray.size(); l++)
			{			
				
				if(firstSolutionArray[l] >= threshold ) // vero -> c'e' un drone
				{
					solutionArray[l] = 1.0;
				}
				else
				{
					solutionArray[l] = 0.0;
				}
				
				//cout << "y" << l + getUsrsNum() << " " << solutionArray[l] << endl;
				yIndexes.push_back(gety_index() + l);
			}
			
			//aggiorno il mip start con il nuovo vettore y
			for(unsigned int i = 0; i< solutionArray.size(); i++)
			{
				
				if(solutionArray[i] == 0.0)
				{
					CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &yIndexes[i], &lb[0], &zeros[0]);
					CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &yIndexes[i], &ub[0], &zeros[0]);
				}
				else
				{
					CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &yIndexes[i], &lb[0], &ones[0]);
					CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &yIndexes[i], &ub[0], &ones[0]);
				}
				//CPXwriteprob( env, lp, (instance + to_string(threshold) + ".lp").c_str(), NULL);
			}	
			
			//ripeto ottimizzazione con mip start
			CPXmipopt(env, lp); 
			//CPXsolwrite(env, lp, (instance + "." + to_string(threshold) + ".sol").c_str());
		
			//check result
			int optStat = CPXgetstat(env, lp);
			
			if(optStat == 101 || optStat == 102) //soluzione rilassata trovata
			{

				if(verbose == true)		
					cout << "Soluzione rilassata trovata con soglia: " << threshold << endl;
				
				int intOptStat = 0;
				
				//esegui LP intero
				if(verbose == true)	
					CPXwriteprob( env, lp, (instance + ".cont.lp").c_str(), NULL);
				
				swapToInt(env, lp);
				
				if(verbose == true)	
					CPXwriteprob( env, lp, (instance + ".int.lp").c_str(), NULL);
				
				CPXmipopt(env, lp); 
				
				intOptStat = CPXgetstat(env, lp); 
				if(intOptStat == 101 || intOptStat == 102) //soluzione intera: stop
				{
					stop = true;
					end = std::chrono::high_resolution_clock::now(); //timestamp di fine
					
					if(verbose == true)	
						cout << "Soluzione intera trovata con soglia: " << threshold << endl;
					
					//aggiorna la soluzione
					/*try
					{
						solution = new solution_t;
					}
					catch (exception& e)
					{
						cerr << __FUNCTION__ << "(): An exception has occurred: " << e.what() << endl;
						delete firstSolution;
						
						return NULL;
					}*/

					solution = getSolution(env, lp, std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(), instance);
					if(solution == NULL)
					{
						cerr << __FUNCTION__ << "(): Impossibile recuperare la soluzione dell'istanza " << instance << endl;
						
						return NULL;
					}

					if(solution->yPositions.size() == 0)
					{
						cerr << __FUNCTION__ << "(): Impossibile recuperare la soluzione dell'istanza " << instance << endl;
						delete solution;
						return NULL;
					}

					if(verbose == true)	
						CPXsolwrite(env, lp, (instance + ".int." + to_string(threshold) + ".sol").c_str());
				}
				else //non c'e' soluzione intera, ricomincia da quella rilassata
				{
					if(verbose == true)	
						cout << "No soluzione intera con soglia: " << threshold << endl;
					
					swapToCont(env,lp);

					if(firstFound == false) //ricerca più approfondita
					{
						
						firstFound = true;
						threshold+= step;
						
						step = accstep;
						
						threshold -= step;
						
					}
					else
					{
						threshold -= step;
						solutionArray = firstSolutionArray; 
					}
					
					if(threshold < stopValue)
					{
						stop = true;	
					}
				}
			}
			else //no solution, riduco la soglia, reset della soluzione
			{
				if(verbose == true)	
					cout << "No soluzione rilassata con soglia: " << threshold << endl;
				
				threshold -= step;
				
				solutionArray = firstSolutionArray; 
				if(threshold < stopValue ) //TODO: double comparison FUBAR, it misses one run
				{
					stop = true;
				}
			}
		}
		CPXsolwrite(env, lp, (instance + ".fin"  + ".sol").c_str());
		//cleanup
		delete firstSolution;
		
		if(solution == NULL)
			cout << "Nessuna soluzione intera trovata per l'istanza: " << instance << endl;

		return solution;
	}
}



/*
solution_t *solve(CEnv env, Prob lp, string instance)
{
	int contRelax[] = {31};
	std::chrono::high_resolution_clock::time_point start, end;
	
	start = std::chrono::high_resolution_clock::now(); //timestamp di inizio
	solution_t *firstSolution = startingSolution(env, lp, instance, contRelax); //kick-start: soluzione iniziale
	if(firstSolution == NULL)
	{
		cerr << __FUNCTION__ << "(): Nessuna soluzione trovata per il rilassamento continuo dell'istanza " << instance << endl;
		return NULL;
	}
	else //solution found
	{
		double currentValue = evaluate(firstSolution);
		double bestValue = currentValue;

		bool stop = false;

		double threshold = 1.0;
		double step = 0.1;
		double stopValue = step;


		solution_t *solution = new solution_t;

		//const int size = getPosNum();
		//double solutionArray[size]; 
		vector< double > solutionArray(getPosNum());
		
		//int status;
		//CPXLPptr originalLP = CPXcloneprob(env,lp, &status);
		
		while(stop != true)
		{
			
			//CHECKED_CPX_CALL(CPXmipopt, env, lp); 
			int optStat = CPXgetstat(env, lp);
			if ( optStat == 103) //CPXMIP_INFEASIBLE 
			{
				// infeasibility detected
				printConflictFile((instance + ".clp"), env, lp);
						
				solution->objValue = -1;
				solution->statusCode = optStat;
				
				cerr << __FUNCTION__ << "(): Conflict, impossibile procedere con la risoluzione euristica. " << endl;
				
				return solution;
			}
			else // 101 or 102 or no solution
			{
				if(optStat == 101 || optStat == 102) //soluzione trovata
				{
					//estraggo array y
					int status = CPXgetx(env, lp, &solutionArray[0], gety_index(), gety_index() + getTotalPotentialNodes() - getUsrsNum() - 1);
					if (status == 0)
					{
						//vector< double > yValue; //forse trascurabile
						vector< int > yIndexes;
						for(unsigned int l = 0; l < solutionArray.size(); l++)
						{			
							double rounded = round(solutionArray[l] * 10.0) / 10.0; // arrotondamento al secondo decimale
							
							if(rounded >= threshold) // vero -> c'e' un drone
								solutionArray[l] = 1.0;
							else
								solutionArray[l] = 0.0;
							
							yIndexes.push_back(gety_index() + l);
						}
						
						solution->yPositions.assign(solutionArray.begin(),solutionArray.end());
						
						solution->objValue = evaluate(solution);
						
						int beg[] = {0};
						int effortlevel[] = {0};
						//status = CPXaddmipstarts (env, lp, 1, solutionArray.size(), beg, &yIndexes[0], &solutionArray[0], effortlevel, NULL);
						status = CPXchgmipstarts (env, lp, 1, 0, solutionArray.size(), beg, &yIndexes[0], &solutionArray[0], effortlevel);	
						CPXwritemipstarts(env, lp, (instance + ".mst").c_str(), 0, 0);
							
										
						//start = std::chrono::high_resolution_clock::now(); //timestamp di inizio
						
						//end = std::chrono::high_resolution_clock::now(); //timestamp di fine

						//cout << "Istanza eseguita in "
						//	<< std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() //casting per la conversione in millisecondi; (end-start).count() calcola la differenza di tempo e la restituisce come valore numerico
						//	<< "ms" << " (" << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << "s c.ca).\n";

						//cout << "y" << l + getUsrsNum() << " : " << yValue[l] << " " << yIndexes[l] <<endl;
						
					}
					else
					{
						cerr << __FUNCTION__ << "(): Errore nel recupero della soluzione dell'istanza. " << endl;
						delete solution;
						return NULL;
					}
					//solution = executeInstance(instance, false, contRelax); 
				}
				else //no solution, riduco la soglia
				{
					threshold -= step;

					if(threshold < stopValue)
					{
						stop = true;
					}
				}
			}
	
		}
		end = std::chrono::high_resolution_clock::now(); //timestamp di fine
		//cleanup
		delete firstSolution;
		return solution;
	}
}*/