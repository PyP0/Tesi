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

bool solutionExists(int code)
{
	if(code != 3 && code != 4 && code != 103 && code != 106 && code != 108 && code != 110 && code != 112 && code != 114 && code != 117 && code !=  119) //soluzione intera trovata: stop
		return true;
	else
		return false;
}

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
	std::chrono::high_resolution_clock::time_point start, end;

	
	start = std::chrono::high_resolution_clock::now(); //timestamp di inizio
		instSolution = solveLP(env, lp, instanceName, verbose, contRelax);
	end = std::chrono::high_resolution_clock::now(); //timestamp di fine

	cout << "Soluzione iniziale trovata in "
		<< std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() //casting per la conversione in millisecondi; (end-start).count() calcola la differenza di tempo e la restituisce come valore numerico
		<< "ms" << " (" << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << "s c.ca).\n";

	//swapToInt(env,lp);

	if(instSolution != NULL)
	{
		if (instSolution->statusCode == 103) //CPXMIP_INFEASIBLE 
		{
			// infeasibility detected
			//printConflictFile((instanceName + ".clp"), env, lp);
			cerr << __FUNCTION__ << "(): Infeasibility, impossibile procedere con la risoluzione euristica. " << endl;
			delete instSolution;
			return NULL;
		}
		else
		{
			//CPXwriteprob( env, lp, (instanceName + "INIT.lp").c_str(), NULL);
			//CHECKED_CPX_CALL(CPXsolwrite, env, lp, (instanceName + "INIT.sol").c_str());
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
			cout << "Potential drones positions: " << getTotalPotentialNodes() << endl << endl;
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
					//printConflictFile(clpFile, env, lp);
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

					if(verbose == true)
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



solution_t *solve2(Env env, Prob lp, string instance, bool verbose)
{
	int contRelax[] = {31};
	std::chrono::high_resolution_clock::time_point start, end;

	std::chrono::high_resolution_clock::time_point startPartial, endPartial;
	std::chrono::high_resolution_clock::time_point Heurstart, Heurend; //TIME

	vector< double > firstSolutionArray(getTotalPotentialNodes() - getUsrsNum());
	vector< pair< int,double > > Y(getTotalPotentialNodes() - getUsrsNum()); //insieme delle y_i. Coppia indice variabile - valore
	
	vector <char> lb(1, 'L');
	vector <char> ub(1, 'U');
	vector <double> ones(1, 1.0);
	vector <double> zeros(1, 0.0);
		
	bool stop = false;

	int iterations = 1;

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
		
		double duration = 0; //TIME
		double remainingTime = 2400000; //40 minuti //TIME

		//setto a 1 la y_i scelta
		while( (stop != true) && (Yiterator <= (int)Y.size()) )
		{
			Heurstart = std::chrono::high_resolution_clock::now(); //TIME

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
			
			//if(verbose == true)	
			//CPXwriteprob( env, lp, (instance + "." + to_string(Y[Ychosen].first) + ".cont.lp").c_str(), NULL);
			
			//esegue LP rilassato
			//CPXmipopt(env, lp); 

			//swaptocont già fatta da soluzione iniziale
			startPartial = std::chrono::high_resolution_clock::now(); //timestamp di inizio
				CPXlpopt(env,lp); //LPOPT
			endPartial = std::chrono::high_resolution_clock::now(); //timestamp di fine

			cout << "***LPOPT iterazione: " << iterations 
			    << " terminato in "
				<< std::chrono::duration_cast<std::chrono::milliseconds>(endPartial - startPartial).count() //casting per la conversione in millisecondi; (end-start).count() calcola la differenza di tempo e la restituisce come valore numerico
				<< "ms" << " (" << std::chrono::duration_cast<std::chrono::seconds>(endPartial - startPartial).count() << "s c.ca).\n";

			int optStat = CPXgetstat(env, lp);
			
			if(solutionExists(optStat) == true) //soluzione rilassata trovata
			{
				//CPXsolwrite(env, lp, (instance + ".lpopt.sol").c_str());
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
				//if(verbose == true)	
				//	CPXwriteprob( env, lp, (instance + ".cont.lp").c_str(), NULL);
				
				swapToInt(env, lp);

				//if(verbose == true)	
				//	CPXwriteprob( env, lp, (instance + ".int.lp").c_str(), NULL);
				
				startPartial = std::chrono::high_resolution_clock::now(); //timestamp di inizio
					CPXmipopt(env, lp); //LPOPT
				endPartial = std::chrono::high_resolution_clock::now(); //timestamp di fine

				cout << "***MIPOPT iterazione: " << iterations 
			   		<< " terminato in "
					<< std::chrono::duration_cast<std::chrono::milliseconds>(endPartial - startPartial).count() //casting per la conversione in millisecondi; (end-start).count() calcola la differenza di tempo e la restituisce come valore numerico
					<< "ms" << " (" << std::chrono::duration_cast<std::chrono::seconds>(endPartial - startPartial).count() << "s c.ca).\n";


				
				
				int linOptStat = CPXgetstat(env, lp);
				if(solutionExists(linOptStat) == true) //soluzione intera trovata: stop
				{
					stop = true;
					end = std::chrono::high_resolution_clock::now(); //timestamp di fine
		
					if(verbose == true)	
					{
						cout << "Soluzione intera trovata con ";
						for(int i = 0; i < Yiterator; i++)
						{
							cout << "y_" << Y[i].first << " ";
						}
						cout << endl;
					}
					
					//aggiorna la soluzione
					solution = getHeurSolution(env, lp, std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(), instance, iterations);
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
					
					//if(verbose == true)	
					//	CPXsolwrite(env, lp, (instance + ".int.sol").c_str());
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

			iterations++;

			Heurend = std::chrono::high_resolution_clock::now(); //TIME
			duration = std::chrono::duration_cast<std::chrono::milliseconds>( Heurend - Heurstart).count();
			cout << "****Sono passati " << duration << " ms. " << endl; //TIME
			remainingTime -= duration; //TIME

			if(remainingTime <= 0) //TIME
				remainingTime = 0.0;
			cout << "****Ne restano " << remainingTime << endl; //TIME

			CPXsetdblparam(env, CPX_PARAM_TILIM, remainingTime/1000.0); //TIME
			double resto = 0; //TIME
			CPXgetdblparam(env, CPX_PARAM_TILIM,&resto); //TIME
			cout <<"Resto: " << resto << endl; //TIME

		}
		if(stop == false)
		{
			cout << __FUNCTION__ << "(): Soluzione non trovata per l'istanza: " << instance << endl;
			return NULL;
		}
		else
		{
			//CPXsolwrite(env, lp, (instance + ".solve2"  + ".sol").c_str());
			return solution;
		}
	}
}



static void assignLPVars(CEnv env, Prob lp, vector< double > solutionArray)
{
	double Smax = getReductionFactor();

	vector <char> lb(1, 'L');
	vector <char> ub(1, 'U');
	vector <double> ones(1, 1.0);
	vector <double> zeros(1, 0.0);

	int tempIndex = 0;

	for(unsigned int i = 0; i < solutionArray.size(); i++)
	{
		cout << i << ") y_" << (i + getUsrsNum()) << " = " << solutionArray[i] << endl;

		if(solutionArray[i] < 0.99) // y_i == 0
		{
			// s_i
			tempIndex = gets_index() + i + getUsrsNum();
			CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &lb[0], &zeros[0]);
			CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &ub[0], &zeros[0]);
			cout << "   " << i <<") s_" << i + getUsrsNum() << " index: " << tempIndex << " -> 0" <<  endl;

			// supera_i
			tempIndex = getsupera_index() + i + getUsrsNum();
			CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &lb[0], &zeros[0]);
			CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &ub[0], &zeros[0]);
			cout << "   " << i <<") o_" << i + getUsrsNum() << " index: " << tempIndex << " -> 0" <<  endl;

			/*for(int j = 0; j < getTotalPotentialNodes(); j++)
			{
				for(int k = 0; k < getCommsNum(); k++)
				{
					tempIndex = getfMap(yIndexes[i], j, k);
					CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &lb[0], &zeros[0]);
					CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &ub[0], &zeros[0]);
				}
			}*/
		}
	}

	// z_j
	for(int j = getUsrsNum(); j < getTotalPotentialNodes() ; j++)
	{
		bool found = false;
		//if(solutionArray[j - getUsrsNum()] >= 0.99)
		//{
			
			for(unsigned int i = 0; i< solutionArray.size() && found != true; i++)
			{
				int Pindex = i + getUsrsNum();

				if( solutionArray[i] > 0.99 && Pindex != j)
				{
					cout << "test pos_" << Pindex << " vs pos_" << j  << " dist = " << getDistance(mapGrid[Pindex].x, mapGrid[Pindex].y, mapGrid[j].x, mapGrid[j].y) << endl;
					double distance = getDistance(mapGrid[Pindex].x, mapGrid[Pindex].y, mapGrid[j].x, mapGrid[j].y);	
					if(distance <= getEpsilonNodeRadius())
					{
						found = true;

						// z_j = 1
						int tempIndex = getz_index() + j ;
						CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &lb[0], &ones[0]);
						CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &ub[0], &ones[0]);
						cout << "   " << i <<") z_" << j << " index: " << tempIndex << " -> 1" <<  endl;
					}
				}
			}
		//}

		if(found == true) // is z_j == 1 ?
		{
			int tempIndex = 0;
			// s_j == Smax
			if(solutionArray[j - getUsrsNum()] >= 0.99) // y_i == 1
			{
				tempIndex = gets_index() + j ;
				
				CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &lb[0], &Smax);
				CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &ub[0], &Smax);
				cout << "   " << j <<") s_" << j  << " index: " << tempIndex << " -> Smax" <<  endl;
			} //se y_i = 0 i corrispondenti s_i gia' settati a 0 in prima parte della funzione

			// o_j == 0?
			tempIndex = getsupera_index() + j ;
				
			CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &lb[0], &zeros[0]);
			CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &ub[0], &zeros[0]);

			cout << "   " << j <<") o_" << j  << " index: " << tempIndex << " -> 0" <<  endl;
		}
		else  // z_j = 0
		{
			int tempIndex = getz_index() + j ;
			CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &lb[0], &zeros[0]);
			CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &ub[0], &zeros[0]);
			cout << "   " << j <<") z_" << j  << " index: " << tempIndex << " -> 0" <<  endl;

			// s_i = sum Aji * y_j + sum Aji
			int Pindex = j ;
			double totalInterference = 0;
			for(int j = getUsrsNum(); j < getTotalPotentialNodes(); j++)  // P
			{
				if(solutionArray[Pindex- getUsrsNum()] > 0.99 && solutionArray[j - getUsrsNum()] > 0.99 ) //non c'e' interferenza se non c'e' il drone
				{
					double distance = getDistance(mapGrid[Pindex].x, mapGrid[Pindex].y, mapGrid[j].x, mapGrid[j].y);
					if(Pindex != j && distance > getEpsilonNodeRadius() && distance <= getNodeRadius()) //R_epsilon excluded
					{
						totalInterference+= getInterference(j,Pindex);
						cout << "   added interference (" << j << " " << Pindex << ") "  <<getInterference(j,Pindex) << " between " << Pindex << " and " << j << " dist " << distance << endl;
					}
				}
			}

			// sum Aji su V v 1.5

			for(int j = 0; j < getUsrsNum(); j++) // V
			{
				if(solutionArray[Pindex- getUsrsNum()] > 0.99 && solutionArray[j - getUsrsNum()] > 0.99) //non c'e' interferenza se non c'e' il drone
				{
					double distance = getDistance(mapGrid[Pindex].x, mapGrid[Pindex].y, mapGrid[j].x, mapGrid[j].y);
					if(Pindex != j && distance <= getNodeRadius()) //1.5 mod
					{
						totalInterference+= getInterference(j,Pindex);
						cout << "   added int between " << Pindex << " and " << j << " dist " << distance << endl;
					}
				}
			}

			// o_i == 1 if totalInterference > Smax; else o_i == 0
			tempIndex = getsupera_index() + j ;
			if(totalInterference > Smax)
			{
				cout << "   " << j <<") o_" << j  << " index: " << tempIndex << " -> 1" <<  endl;
				CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &lb[0], &ones[0]);
				CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &ub[0], &ones[0]);

				if(solutionArray[j - getUsrsNum() ] >= 0.99) // y_i == 1
				{
					tempIndex = gets_index() + j ;
					cout << "   " << j <<") s_" << j  << " index: " << tempIndex << " -> Smax" <<  endl;
					CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &lb[0], &Smax);
					CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &ub[0], &Smax);
				} // else, s_i = 0 in inizio funzione
			}
			else
			{
				cout << "   " << j <<") o_" << j  << " index: " << tempIndex << " -> 0" <<  endl;
				CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &lb[0], &zeros[0]);
				CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &ub[0], &zeros[0]);

				if(solutionArray[j - getUsrsNum()] >= 0.99) // y_i == 1
				{
					tempIndex = gets_index() + j ;
					cout << "   " << j <<") s_" << j  << " index: " << tempIndex << " -> " << totalInterference <<  endl;
					CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &lb[0], &totalInterference);
					CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &ub[0], &totalInterference);
				} // else, s_i = 0 in inizio funzione
			}
		}
	}

	//users side
	for(int i = 0; i < getUsrsNum(); i++) //TODO: to be removed with these variables
	{
		int tempIndex = gets_index() + i ;
		CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &lb[0], &zeros[0]);
		CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &ub[0], &zeros[0]);

		tempIndex = getz_index() + i ;
		CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &lb[0], &zeros[0]);
		CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &ub[0], &zeros[0]);

		tempIndex = getsupera_index() + i ;
		CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &lb[0], &zeros[0]);
		CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &tempIndex, &ub[0], &zeros[0]);
	}

}

solution_t *solve3(Env env, Prob lp, string instance, bool verbose)
{
	
	std::chrono::high_resolution_clock::time_point start, end;
	std::chrono::high_resolution_clock::time_point startPartial, endPartial;

	std::chrono::high_resolution_clock::time_point Heurstart, Heurend; //TIME

	bool stop = false;

	double nUp = getPosNum(), nBottom = 1, n = 0;


	vector< int > yIndexes;
	vector< double > solutionArray(getTotalPotentialNodes() - getUsrsNum());
	vector< double > firstSolutionArray(getTotalPotentialNodes() - getUsrsNum());

	vector <char> lb(1, 'L');
	vector <char> ub(1, 'U');
	vector <double> ones(1, 1.0);
	vector <double> zeros(1, 0.0);

	vector< pair< int,double > > Y(getTotalPotentialNodes() - getUsrsNum()); //insieme delle y_i. Coppia indice variabile - valore
	
	start = std::chrono::high_resolution_clock::now(); //timestamp di inizio

	int iterations = 1;

	int contRelax[] = {31};
	solution_t *firstSolution = startingSolution(env, lp, instance, contRelax); //kick-start: soluzione iniziale
	solution_t *bestSolution = NULL;

	if(firstSolution == NULL)
	{
		cerr << __FUNCTION__ << "(): Nessuna soluzione trovata per il rilassamento continuo dell'istanza " << instance << endl;
		return NULL;
	}
	else //solution found
	{
		solution_t *solution = NULL;

		//recupero array y
		int status = CPXgetx(env, lp, &firstSolutionArray[0], gety_index(), gety_index() + getTotalPotentialNodes() - getUsrsNum() - 1);
		if(status != 0)
		{
			cerr << __FUNCTION__ << "(): Nessuna soluzione trovata per il rilassamento continuo dell'istanza " << instance << endl;
			delete firstSolution;
			return NULL;
		}
 
 		try
 		{
			bestSolution = new solution_t;

			bestSolution->yPositions.assign(getTotalPotentialNodes()-getUsrsNum(), 1.0); //init to worst value possibile	
		}
		catch (exception& e)
		{
			cerr << __FUNCTION__ << "(): An exception has occurred: " << e.what() << endl;
			return NULL;
		}

		//riempimento insieme Y
		for(unsigned int i = 0; i < Y.size(); i++)
		{
			Y[i] = make_pair(i,firstSolutionArray[i]);
		}
		
		//ordinamento decrescente di Y
		sort(Y.begin(),Y.end(), pairDecrSort);

		// vettore di supporto
		vector< double > workingSolution;
		for(int i = 0; i < (int) Y.size(); i++)
		{
			workingSolution.push_back(Y[i].second);
		}

		for(int i=0; i< Y.size(); i++)
		{
			cout << "y_" << Y[i].first + getUsrsNum() << " " << Y[i].second << endl;
		}


		/*for(int i=0; i< firstSolutionArray.size(); i++)
		{
			cout << i << ") " << firstSolutionArray[i] << endl;
		}*/

		swapToInt(env, lp);

		double duration = 0; //TIME
		double remainingTime = 2400000; //40 minuti //TIME
		//std::chrono::minutes timeMax (40); //TIME
		while(nBottom <= nUp)
		{
			Heurstart = std::chrono::high_resolution_clock::now(); //TIME
			

			//vector< double > workingSolution = firstSolutionArray;
			n = floor( (nUp + nBottom) / 2.0 );

			cout << "=============" << endl;
			cout << "n: " << n << endl;
			cout << "nUp: " << nUp << endl;
			cout << "nBottom: " << nBottom << endl;
			cout << "=============" << endl;

			int index = 0;

			//modifico i bound delle variabili y_i
			for(int i = 0; i < (int) Y.size(); i++)
			{
				index = gety_index() + Y[i].first;
				if(i <= n)
				{
					CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &index, &lb[0], &ones[0]);
					CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &index, &ub[0], &ones[0]);
					workingSolution[Y[i].first] = 1.0;
				}
				else
				{
					CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &index, &lb[0], &zeros[0]);
					CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &index, &ub[0], &zeros[0]);
					workingSolution[Y[i].first] = 0.0;
				}
			}

			//altre vars
			//assignLPVars(env,lp,workingSolution);

			
				
			if(verbose == true)	
			//	CPXwriteprob( env, lp, (instance + ". " +  to_string(n) +".lp").c_str(), NULL);
				
			char car;
			//cin >> car;

			startPartial = std::chrono::high_resolution_clock::now(); //timestamp di inizio
				CPXmipopt(env,lp); //LPOPT
			endPartial = std::chrono::high_resolution_clock::now(); //timestamp di fine

			cout << "***MIPOPT iterazione: " << iterations 
			   	<< " terminato in "
				<< std::chrono::duration_cast<std::chrono::milliseconds>(endPartial - startPartial).count() //casting per la conversione in millisecondi; (end-start).count() calcola la differenza di tempo e la restituisce come valore numerico
				<< "ms" << " (" << std::chrono::duration_cast<std::chrono::seconds>(endPartial - startPartial).count() << "s c.ca).\n";
				
			int intOptStat = CPXgetstat(env, lp); 
			cout << "*** code: " << intOptStat << endl;

			if( solutionExists(intOptStat) == true) //soluzione intera
			{
				//CPXsolwrite(env, lp, (instance + "." + to_string(n) + ".sol").c_str());
				if(verbose == true)	
					cout << "Soluzione intera trovata con soglia: " << n << endl;

				end = std::chrono::high_resolution_clock::now(); //timestamp di fine

				solution = getHeurSolution(env, lp, std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(), instance, iterations);
				printSolution(solution);

				
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

				nUp = n-1;
				cout << "Ultima soluzione # droni: " << dronesCount(bestSolution,1.0) << endl;
				if(dronesCount(solution,1.0) <= dronesCount(bestSolution,1.0))
				{
					cout << "Ultima soluzione # droni: " << dronesCount(bestSolution,1.0) << endl;
					cout << "Trovata nuova soluzione con # droni: " << dronesCount(solution,1.0) << endl;
					delete bestSolution;

					bestSolution = copySolution(solution);
				}
			}
			else
			{
				nBottom = n+1;
				if(intOptStat == 103)
				{ //debug
					// infeasibility detected
					//printConflictFile((instance + ".clp"), env, lp);
					cerr << __FUNCTION__ << "(): Infeasibility, impossibile procedere con la risoluzione euristica. " << endl;
				}
			}

			iterations++;


			Heurend = std::chrono::high_resolution_clock::now(); //TIME
			duration = std::chrono::duration_cast<std::chrono::milliseconds>( Heurend - Heurstart).count();
			cout << "****Sono passati " << duration << " ms. " << endl; //TIME
			remainingTime -= duration; //TIME

			if(remainingTime <= 0) //TIME
				remainingTime = 0.0;
			cout << "****Ne restano " << remainingTime << endl; //TIME

			CPXsetdblparam(env, CPX_PARAM_TILIM, remainingTime/1000.0); //TIME
			double resto = 0; //TIME
			CPXgetdblparam(env, CPX_PARAM_TILIM,&resto); //TIME
			cout <<"Resto: " << resto << endl; //TIME

		}

		delete firstSolution;
		
		if(solution == NULL)
		{
			cerr << __FUNCTION__ << "(): Nessuna soluzione intera trovata per l'istanza: " << instance << endl;
			return NULL;
		}

		
				
		return bestSolution;
	}
}



//ULTIMA con lpopt
solution_t *solve(CEnv env, Prob lp, string instance, bool verbose)
{
	int contRelax[] = {31};
	std::chrono::high_resolution_clock::time_point start, end;
	std::chrono::high_resolution_clock::time_point startPartial, endPartial;

	bool stop = false;

	double threshold = 1.0;
	double left, right;

	double stopValue = 0.0000001;


	vector< int > yIndexes;
	vector< double > solutionArray(getTotalPotentialNodes() - getUsrsNum());
	vector< double > firstSolutionArray(getTotalPotentialNodes() - getUsrsNum());

	vector <char> lb(1, 'L');
	vector <char> ub(1, 'U');
	vector <double> ones(1, 1.0);
	vector <double> zeros(1, 0.0);

	start = std::chrono::high_resolution_clock::now(); //timestamp di inizio

	int iterations = 1;

	solution_t *firstSolution = startingSolution(env, lp, instance, contRelax); //kick-start: soluzione iniziale
	solution_t *bestSolution = NULL;

	if(firstSolution == NULL)
	{
		cerr << __FUNCTION__ << "(): Nessuna soluzione trovata per il rilassamento continuo dell'istanza " << instance << endl;
		return NULL;
	}
	else //solution found
	{
		left = 1.0;
		right = 0.0;
		threshold = (left + right) / 2.0;

		cout<< "Left: " << left << endl;
		cout << "Right: " << right << endl;
		cout << "Threshold: " << threshold << endl;

		//if(verbose == true)
		//{
			cout << "Soluzione iniziale trovata:" << endl;
			printRelaxedSolution(firstSolution);
		//}

		solution_t *solution = NULL;

		//recupero array y
		int status = CPXgetx(env, lp, &firstSolutionArray[0], gety_index(), gety_index() + getTotalPotentialNodes() - getUsrsNum() - 1);
		if(status != 0)
		{
			cerr << __FUNCTION__ << "(): Nessuna soluzione trovata per il rilassamento continuo dell'istanza " << instance << endl;
			delete firstSolution;
			return NULL;
		}
 
 		try
 		{
			bestSolution = new solution_t;

			bestSolution->yPositions.assign(getTotalPotentialNodes()-getUsrsNum(), 1.0); //init to worst value possibile
			
		}
		catch (exception& e)
		{
			cerr << __FUNCTION__ << "(): An exception has occurred: " << e.what() << endl;
			return NULL;
		}

		while(threshold >= stopValue && stop == false)
		{
			char car;
			cin >> car;
			//elaboro array y
			//cout << "threshold: " << threshold << endl;
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
			// l'assegnamento di vars y_i determina il valore di altre vars
			double Smax = getReductionFactor();
			for(unsigned int i = 0; i < solutionArray.size(); i++)
			{
				cout << i << ") y_" << (i + getUsrsNum()) << " = " << solutionArray[i] << endl;
				if(solutionArray[i] < 0.99) // y_i == 0
				{
					CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &yIndexes[i], &lb[0], &zeros[0]);
					CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &yIndexes[i], &ub[0], &zeros[0]);

					cout << "   y_" << (yIndexes[i] - gety_index() +getUsrsNum()) << " -> 0" <<  endl;

				}
				else // y_i == 1
				{
					// y_i
					cout << i << ") y_" << ( i + getUsrsNum()) << " = " << solutionArray[i] << endl;
					CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &yIndexes[i], &lb[0], &ones[0]);
					CHECKED_CPX_CALL(CPXchgbds, env, lp, 1, &yIndexes[i], &ub[0], &ones[0]);
				}
			}

			//assignLPVars(env, lp, solutionArray);

			//CPXwriteprob( env, lp, (instance + to_string(threshold) + ".lp").c_str(), NULL);

			
			
			//ripeto ottimizzazione con mip start
			//CPXmipopt(env, lp);  //LPOPT

			//swaptocont già fatta da soluzione iniziale
			startPartial = std::chrono::high_resolution_clock::now(); //timestamp di inizio
				CPXlpopt(env,lp); //LPOPT
			endPartial = std::chrono::high_resolution_clock::now(); //timestamp di fine

			cout << "***LPOPT iterazione: " << iterations 
			    << " terminato in "
				<< std::chrono::duration_cast<std::chrono::milliseconds>(endPartial - startPartial).count() //casting per la conversione in millisecondi; (end-start).count() calcola la differenza di tempo e la restituisce come valore numerico
				<< "ms" << " (" << std::chrono::duration_cast<std::chrono::seconds>(endPartial - startPartial).count() << "s c.ca).\n";

			//CPXsolwrite(env, lp, (instance + "." + to_string(threshold) + ".sol").c_str());
		
			//check result
			int optStat = CPXgetstat(env, lp);
			cout << "**codice: " << optStat << endl;
			if(solutionExists(optStat) == true) //soluzione rilassata trovata
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
				

				startPartial = std::chrono::high_resolution_clock::now(); //timestamp di inizio
					CPXmipopt(env,lp); //LPOPT
				endPartial = std::chrono::high_resolution_clock::now(); //timestamp di fine

				cout << "***MIPOPT iterazione: " << iterations 
			    	<< " terminato in "
					<< std::chrono::duration_cast<std::chrono::milliseconds>(endPartial - startPartial).count() //casting per la conversione in millisecondi; (end-start).count() calcola la differenza di tempo e la restituisce come valore numerico
					<< "ms" << " (" << std::chrono::duration_cast<std::chrono::seconds>(endPartial - startPartial).count() << "s c.ca).\n";

				
				intOptStat = CPXgetstat(env, lp); 
				if( solutionExists(intOptStat) == true) //soluzione intera
				{
					
					//cout << "pippo" << endl;
					if(verbose == true)	
						cout << "Soluzione intera trovata con soglia: " << threshold << endl;

					end = std::chrono::high_resolution_clock::now(); //timestamp di fine

					solution = getHeurSolution(env, lp, std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(), instance, iterations);
					printSolution(solution);

					//cout << "pappo" << endl;
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

					//cout << "peppo" << endl; 
					if(dronesCount(solution,1.0) <= dronesCount(bestSolution,1.0)) //trovata soluzione migliore
					{	
						//cout << "pioppo" << endl;

						cout << "Ultima soluzione # droni: " << dronesCount(bestSolution,1.0) << endl;
						cout << "Trovata nuova soluzione con # droni: " << dronesCount(solution,1.0) << endl;
						delete bestSolution;

						bestSolution = copySolution(solution);

						if(bestSolution == NULL)
						{
							cerr << __FUNCTION__ << "(): Impossibile copiare la soluzione." << endl;
							stop = true; 
						}
						else
						{
							//sposta la ricerca a sinistra
							right = threshold;

							threshold = (left + right) / 2.0;
							cout<< "Left: " << left << endl;
							cout << "Right: " << right << endl;
							cout << "Threshold: " << threshold << endl;
						}
					}
					else // soluzione attuale e' la migliore
					{
						stop = true;
						//end = std::chrono::high_resolution_clock::now(); //timestamp di fine
					}

					if(verbose == true)	
						CPXsolwrite(env, lp, (instance + ".int." + to_string(threshold) + ".sol").c_str());
				}
				else //no soluzione intera, salto a destra
				{
					if(verbose == true)	
						cout << "No soluzione intera con soglia: " << threshold << endl;
					
					swapToCont(env,lp);

					left = threshold;
					threshold = (left + right) / 2.0;

					cout<< "Left: " << left << endl;
					cout << "Right: " << right << endl;
					cout << "Threshold: " << threshold << endl;
				}
			}
			else //no soluzione rilassata, salto a destra 
			{
				if(verbose == true)	
					cout << "No soluzione rilassata con soglia: " << threshold << endl;
				
				//threshold -= step;
				left = threshold;
				threshold = (left + right) / 2.0;

				cout<< "Left: " << left << endl;
				cout << "Right: " << right << endl;
				cout << "Threshold: " << threshold << endl;

				//solutionArray = firstSolutionArray; 
			}

			iterations++;
		}
		//CPXsolwrite(env, lp, (instance + ".solve"  + ".sol").c_str());
		//cleanup
		delete firstSolution;
		
		if(solution == NULL)
		{
			cerr << __FUNCTION__ << "(): Nessuna soluzione intera trovata per l'istanza: " << instance << endl;
			return NULL;
		}

		return bestSolution;
	}
}

//last working
/*solution_t *solve(CEnv env, Prob lp, string instance, bool verbose)
{
	int contRelax[] = {31};
	std::chrono::high_resolution_clock::time_point start, end;
	std::chrono::high_resolution_clock::time_point startPartial, endPartial;

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

	int iterations = 1;

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
			//CPXmipopt(env, lp);  //LPOPT

			//swaptocont già fatta da soluzione iniziale
			startPartial = std::chrono::high_resolution_clock::now(); //timestamp di inizio
				CPXlpopt(env,lp); //LPOPT
			endPartial = std::chrono::high_resolution_clock::now(); //timestamp di fine

			cout << "LPOPT iterazione: " << iterations 
			    << " terminato in "
				<< std::chrono::duration_cast<std::chrono::milliseconds>(endPartial - startPartial).count() //casting per la conversione in millisecondi; (end-start).count() calcola la differenza di tempo e la restituisce come valore numerico
				<< "ms" << " (" << std::chrono::duration_cast<std::chrono::seconds>(endPartial - startPartial).count() << "s c.ca).\n";

			//CPXsolwrite(env, lp, (instance + "." + to_string(threshold) + ".sol").c_str());
		
			//check result
			int optStat = CPXgetstat(env, lp);
			
			if(solutionExists(optStat) == true) //soluzione rilassata trovata
			{

				if(verbose == true)		
					cout << "Soluzione rilassata trovata con soglia: " << threshold << endl;
				
				int intOptStat = 0;
				
				//esegui LP intero
				//if(verbose == true)	
					//CPXwriteprob( env, lp, (instance + ".cont.lp").c_str(), NULL);
				
				swapToInt(env, lp);
				
				//if(verbose == true)	
					//CPXwriteprob( env, lp, (instance + ".int.lp").c_str(), NULL);
				

				startPartial = std::chrono::high_resolution_clock::now(); //timestamp di inizio
					CPXmipopt(env,lp); //LPOPT
				endPartial = std::chrono::high_resolution_clock::now(); //timestamp di fine

				cout << "MIPOPT iterazione: " << iterations 
			    	<< " terminato in "
					<< std::chrono::duration_cast<std::chrono::milliseconds>(endPartial - startPartial).count() //casting per la conversione in millisecondi; (end-start).count() calcola la differenza di tempo e la restituisce come valore numerico
					<< "ms" << " (" << std::chrono::duration_cast<std::chrono::seconds>(endPartial - startPartial).count() << "s c.ca).\n";

				
				intOptStat = CPXgetstat(env, lp); 
				if( solutionExists(intOptStat) == true) //soluzione intera: stop
				{
					stop = true;
					end = std::chrono::high_resolution_clock::now(); //timestamp di fine
					
					if(verbose == true)	
						cout << "Soluzione intera trovata con soglia: " << threshold << endl;

					solution = getHeurSolution(env, lp, std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(), instance, iterations);
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

					//if(verbose == true)	
					//	CPXsolwrite(env, lp, (instance + ".int." + to_string(threshold) + ".sol").c_str());
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

			iterations++;
		}
		CPXsolwrite(env, lp, (instance + ".solve"  + ".sol").c_str());
		//cleanup
		delete firstSolution;
		
		if(solution == NULL)
			cout << "Nessuna soluzione intera trovata per l'istanza: " << instance << endl;

		return solution;
	}
}*/



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

/*
solution_t *solve(CEnv env, Prob lp, string instance, bool verbose)
{
	int contRelax[] = {31};
	std::chrono::high_resolution_clock::time_point start, end;
	std::chrono::high_resolution_clock::time_point startPartial, endPartial;

	bool stop = false;

	double threshold = 1.0;
	double step = 0.01;

	double mid = threshold ;


	double stopValue = step;

	double left, right;


	vector< int > yIndexes;
	vector< double > solutionArray(getTotalPotentialNodes() - getUsrsNum());
	vector< double > firstSolutionArray(getTotalPotentialNodes() - getUsrsNum());

	vector <char> lb(1, 'L');
	vector <char> ub(1, 'U');
	vector <double> ones(1, 1.0);
	vector <double> zeros(1, 0.0);

	start = std::chrono::high_resolution_clock::now(); //timestamp di inizio

	int iterations = 1;

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
				
				if(solutionArray[i] < 0.999)
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
			//CPXmipopt(env, lp);  //LPOPT

			//swaptocont già fatta da soluzione iniziale
			startPartial = std::chrono::high_resolution_clock::now(); //timestamp di inizio
				CPXlpopt(env,lp); //LPOPT
			endPartial = std::chrono::high_resolution_clock::now(); //timestamp di fine

			cout << "***LPOPT iterazione: " << iterations 
			    << " terminato in "
				<< std::chrono::duration_cast<std::chrono::milliseconds>(endPartial - startPartial).count() //casting per la conversione in millisecondi; (end-start).count() calcola la differenza di tempo e la restituisce come valore numerico
				<< "ms" << " (" << std::chrono::duration_cast<std::chrono::seconds>(endPartial - startPartial).count() << "s c.ca).\n";

			//CPXsolwrite(env, lp, (instance + "." + to_string(threshold) + ".sol").c_str());
		
			//check result
			int optStat = CPXgetstat(env, lp);
			//cout << "****codice: " << optStat << endl;
			if(solutionExists(optStat) == true) //soluzione rilassata trovata
			{

				if(verbose == true)		
					cout << "Soluzione rilassata trovata con soglia: " << threshold << endl;
				
				int intOptStat = 0;
				
				//esegui LP intero
				//if(verbose == true)	
					//CPXwriteprob( env, lp, (instance + ".cont.lp").c_str(), NULL);
				
				swapToInt(env, lp);
				
				//if(verbose == true)	
					//CPXwriteprob( env, lp, (instance + ".int.lp").c_str(), NULL);
				

				startPartial = std::chrono::high_resolution_clock::now(); //timestamp di inizio
					CPXmipopt(env,lp); //LPOPT
				endPartial = std::chrono::high_resolution_clock::now(); //timestamp di fine

				cout << "***MIPOPT iterazione: " << iterations 
			    	<< " terminato in "
					<< std::chrono::duration_cast<std::chrono::milliseconds>(endPartial - startPartial).count() //casting per la conversione in millisecondi; (end-start).count() calcola la differenza di tempo e la restituisce come valore numerico
					<< "ms" << " (" << std::chrono::duration_cast<std::chrono::seconds>(endPartial - startPartial).count() << "s c.ca).\n";

				
				intOptStat = CPXgetstat(env, lp); 
				if( solutionExists(intOptStat) == true) //soluzione intera: stop
				{
					stop = true;
					end = std::chrono::high_resolution_clock::now(); //timestamp di fine
					
					if(verbose == true)	
						cout << "Soluzione intera trovata con soglia: " << threshold << endl;

					solution = getHeurSolution(env, lp, std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(), instance, iterations);
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

					//if(verbose == true)	
					//	CPXsolwrite(env, lp, (instance + ".int." + to_string(threshold) + ".sol").c_str());
				}
				else //non c'e' soluzione intera, ricomincia da quella rilassata
				{
					if(verbose == true)	
						cout << "No soluzione intera con soglia: " << threshold << endl;
					
					swapToCont(env,lp);

					
					threshold -= step;
					solutionArray = firstSolutionArray; 
					
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

			iterations++;
		}
		CPXsolwrite(env, lp, (instance + ".solve"  + ".sol").c_str());
		//cleanup
		delete firstSolution;
		
		if(solution == NULL)
			cout << "Nessuna soluzione intera trovata per l'istanza: " << instance << endl;

		return solution;
	}
}*/

//ultima senza lpopt
/*solution_t *solve(CEnv env, Prob lp, string instance, bool verbose)
{
	int contRelax[] = {31};
	std::chrono::high_resolution_clock::time_point start, end;
	std::chrono::high_resolution_clock::time_point startPartial, endPartial;

	bool stop = false;

	double threshold = 1.0;
	double left, right;

	double stopValue = 0.0001;


	vector< int > yIndexes;
	vector< double > solutionArray(getTotalPotentialNodes() - getUsrsNum());
	vector< double > firstSolutionArray(getTotalPotentialNodes() - getUsrsNum());

	vector <char> lb(1, 'L');
	vector <char> ub(1, 'U');
	vector <double> ones(1, 1.0);
	vector <double> zeros(1, 0.0);

	start = std::chrono::high_resolution_clock::now(); //timestamp di inizio

	int iterations = 1;

	solution_t *firstSolution = startingSolution(env, lp, instance, contRelax); //kick-start: soluzione iniziale
	solution_t *bestSolution = NULL;

	if(firstSolution == NULL)
	{
		cerr << __FUNCTION__ << "(): Nessuna soluzione trovata per il rilassamento continuo dell'istanza " << instance << endl;
		return NULL;
	}
	else //solution found
	{
		left = 1.0;
		right = 0.0;
		threshold = (left + right) / 2.0;

		cout<< "Left: " << left << endl;
		cout << "Right: " << right << endl;
		cout << "Threshold: " << threshold << endl;

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
 
 		try
 		{
			bestSolution = new solution_t;

			bestSolution->yPositions.assign(getTotalPotentialNodes()-getUsrsNum(), 1.0); //init to worst value possibile
			
		}
		catch (exception& e)
		{
			cerr << __FUNCTION__ << "(): An exception has occurred: " << e.what() << endl;
			return NULL;
		}

		swapToInt(env, lp); 
		while(threshold >= stopValue && stop == false)
		{
			//elaboro array y
			//cout << "threshold: " << threshold << endl;
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
				
				if(solutionArray[i] < 0.99)
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
			//CPXmipopt(env, lp);  //LPOPT

			//swaptocont già fatta da soluzione iniziale
		//	startPartial = std::chrono::high_resolution_clock::now(); //timestamp di inizio
		//		CPXlpopt(env,lp); //LPOPT
		//	endPartial = std::chrono::high_resolution_clock::now(); //timestamp di fine

		//	cout << "***LPOPT iterazione: " << iterations 
		//	    << " terminato in "
		//		<< std::chrono::duration_cast<std::chrono::milliseconds>(endPartial - startPartial).count() //casting per la conversione in millisecondi; (end-start).count() calcola la differenza di tempo e la restituisce come valore numerico
		//		<< "ms" << " (" << std::chrono::duration_cast<std::chrono::seconds>(endPartial - startPartial).count() << "s c.ca).\n";

		//	//CPXsolwrite(env, lp, (instance + "." + to_string(threshold) + ".sol").c_str());
		
		//	//check result
		//	int optStat = CPXgetstat(env, lp);
		//	cout << "**codice: " << optStat << endl;
		//	if(solutionExists(optStat) == true) //soluzione rilassata trovata
		//	{


		//		if(verbose == true)		
		//			cout << "Soluzione rilassata trovata con soglia: " << threshold << endl;
				
		//		int intOptStat = 0;
				
		//		//esegui LP intero
		//		if(verbose == true)	
		//			CPXwriteprob( env, lp, (instance + ".cont.lp").c_str(), NULL);
				
				
				
				if(verbose == true)	
					CPXwriteprob( env, lp, (instance + ".int.lp").c_str(), NULL);
				

				startPartial = std::chrono::high_resolution_clock::now(); //timestamp di inizio
					CPXmipopt(env,lp); //LPOPT
				endPartial = std::chrono::high_resolution_clock::now(); //timestamp di fine

				cout << "***MIPOPT iterazione: " << iterations 
			    	<< " terminato in "
					<< std::chrono::duration_cast<std::chrono::milliseconds>(endPartial - startPartial).count() //casting per la conversione in millisecondi; (end-start).count() calcola la differenza di tempo e la restituisce come valore numerico
					<< "ms" << " (" << std::chrono::duration_cast<std::chrono::seconds>(endPartial - startPartial).count() << "s c.ca).\n";

				
				int intOptStat = CPXgetstat(env, lp); 
				if( solutionExists(intOptStat) == true) //soluzione intera
				{
					
					//cout << "pippo" << endl;
					if(verbose == true)	
						cout << "Soluzione intera trovata con soglia: " << threshold << endl;

					end = std::chrono::high_resolution_clock::now(); //timestamp di fine

					solution = getHeurSolution(env, lp, std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(), instance, iterations);
					printSolution(solution);

					//cout << "pappo" << endl;
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

					//cout << "peppo" << endl; 
					if(dronesCount(solution,1.0) < dronesCount(bestSolution,1.0)) //trovata soluzione migliore
					{	
						//cout << "pioppo" << endl;

						cout << "Ultima soluzione # droni: " << dronesCount(bestSolution,1.0) << endl;
						cout << "Trovata nuova soluzione con # droni: " << dronesCount(solution,1.0) << endl;
						delete bestSolution;

						bestSolution = copySolution(solution);

						if(bestSolution == NULL)
						{
							cerr << __FUNCTION__ << "(): Impossibile copiare la soluzione." << endl;
							stop = true; 
						}
						else
						{
							//sposta la ricerca a sinistra
							right = threshold;

							threshold = (left + right) / 2.0;
							cout<< "Left: " << left << endl;
							cout << "Right: " << right << endl;
							cout << "Threshold: " << threshold << endl;
						}
					}
					else // soluzione attuale e' la migliore
					{
						stop = true;
						//end = std::chrono::high_resolution_clock::now(); //timestamp di fine
					}

					if(verbose == true)	
						CPXsolwrite(env, lp, (instance + ".int." + to_string(threshold) + ".sol").c_str());
				}
				else //no soluzione intera, salto a destra
				{
					if(verbose == true)	
						cout << "No soluzione intera con soglia: " << threshold << endl;
					
					//swapToCont(env,lp);

					left = threshold;
					threshold = (left + right) / 2.0;

					cout<< "Left: " << left << endl;
					cout << "Right: " << right << endl;
					cout << "Threshold: " << threshold << endl;
				}
			//}
			//else //no soluzione rilassata, salto a destra 
			//{
			//	if(verbose == true)	
			//		cout << "No soluzione rilassata con soglia: " << threshold << endl;
				
				//threshold -= step;
			//	left = threshold;
			//	threshold = (left + right) / 2.0;

			//	cout<< "Left: " << left << endl;
			//	cout << "Right: " << right << endl;
			//	cout << "Threshold: " << threshold << endl;

				//solutionArray = firstSolutionArray; 
			//}

			//iterations++;
		}
		//CPXsolwrite(env, lp, (instance + ".solve"  + ".sol").c_str());
		//cleanup
		delete firstSolution;
		
		if(solution == NULL)
			cout << "Nessuna soluzione intera trovata per l'istanza: " << instance << endl;

		return bestSolution;
	}
}*/