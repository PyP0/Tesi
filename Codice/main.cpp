/**
* @file modelv1.2.cpp
* @brief
*/
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <math.h>
#include <vector>
#include <chrono>
#include "cpxmacro.h"
#include "instance.h"
#include "utility.h"
#include "lpsolver.h"
#include "heuristic.h"
#include "getRSS.h"



using namespace std;

void printHelper()
{
	cout << endl << "Program usage: " <<endl;
	cout <<"ProgramName \t Generates instances. " <<endl; //TODO
	cout <<"ProgramName -a [PATH] \t Executes every instance found into [PATH] or default DIR (if PATH not provided)." <<endl; //TODO
	cout <<"ProgramName [INSTANCE_NAME] \t Execute the specific instance." << endl << endl; //TODO 
}

vector< string > loadFileList(string masterFile)
{
	vector< string> fileList;
	string fileLine;
	ifstream file;
	int count = 0;
	file.open(masterFile,ios::in);
	if(file.is_open())
	{
		while(file >> fileLine)
		{
			cout <<fileLine <<endl;
			fileList.push_back(fileLine);
				count++;
			
		}
		cout << "Sono state caricate " << count << " istanze." << endl;
		file.close();
		return fileList;
	}
	else
	{
		cout << "(): Impossibile aprire il file: " << fileLine << endl;
		return fileList;
	}
}


solution_t *executeInstance(string fileName, bool verbose, int contRelax[])
{
	solution_t *instSolution = NULL;
	
	// init

	DECL_ENV(env);
	DECL_PROB(env, lp);

	/*if(CPXsetintparam(env, CPX_PARAM_CLOCKTYPE, 2) != 0) 
		cerr << __FUNCTION__ << "(): 2 Impossibile settare uno o piu' parametri di CPLEX." << endl;
	
	if(CPXsetintparam(env, CPX_PARAM_CONFLICTDISPLAY, 2) != 0) 
		cerr << __FUNCTION__ << "(): 3 Impossibile settare uno o piu' parametri di CPLEX." << endl;

	if(CPXsetintparam(env, CPX_PARAM_MEMORYEMPHASIS, 1) != 0) 
		cerr << __FUNCTION__ << "(): 4 Impossibile settare uno o piu' parametri di CPLEX." << endl;

	if(CPXsetintparam(env, CPX_PARAM_NODEFILEIND, 3) != 0) 
		cerr << __FUNCTION__ << "(): 5 Impossibile settare uno o piu' parametri di CPLEX." << endl;

	if(CPXsetdblparam(env, CPX_PARAM_TRELIM, 16384.0 ) != 0) //16GB
		cerr << __FUNCTION__ << "(): 6 Impossibile settare uno o piu' parametri di CPLEX." << endl;


	if(CPXsetdblparam(env, CPX_PARAM_TILIM, 1800.0 ) != 0) 
		cerr << __FUNCTION__ << "(): 7 Impossibile settare uno o piu' parametri di CPLEX." << endl;*/


	//CPXwriteparam (env, "CPLEXparams.prm");
	
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
			instSolution = solveLP(env, lp, instance, verbose, contRelax);
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

int executeMaster(vector< string > fileList, string masterSolutionsFile, bool verbose, int contRelax[])
{
	int globalStatus = 0; 
	solution_t *instSolution = NULL;
	ofstream file;
	if(fileList.size() > 0)
	{
		file.open(masterSolutionsFile,ios::out);
		
		if(!file.is_open())
		{
			cerr << __FUNCTION__ << "(): Impossibile scrivere i risultati sul file: " << masterSolutionsFile << endl;
		}

		cout << "File list: " << endl;
		for(unsigned int i = 0; i < fileList.size(); i++)
		{
			cout << fileList[i] << endl;
		}

		//column names
		file << "Istanza\t";
		file << "Tempo (sec)\t";
		file << "# droni\t";
		file << "Opened Nodes\t";
		file << "Best Obj\t";
		file << "Cutoff\t";
		file << "Cuts\t";
		file << "Gap\t";
		file << "Status" << endl;

		for(unsigned int i = 0; i < fileList.size(); i++)
		{
			instSolution = executeInstance(fileList[i],verbose,contRelax);
			if(instSolution == NULL)  //check solution status
				globalStatus = 1;
			else
			{
				if(file.is_open())
				{
					int dronesCount = 0;
					for(unsigned int i = 0; i < instSolution->yPositions.size(); i++)
					{
						if (round(instSolution->yPositions[i]) == 1)
						{
							dronesCount++;
						}
					}
					//remove the ../instances/ //TODO: fixare
					file << instSolution->instName.substr(13,string::npos) << "\t";
					//file << instSolution->objValue << "\t";
					file << instSolution->execTime << "\t";
					//file << dronesCount << "/" << getDrnsNum() << "\t";
					file << dronesCount << "\t";
					file << instSolution->openedNodes << "\t";
					file << instSolution->bestobj << "\t";
					file << instSolution->cutoff << "\t";
					file << instSolution->cuts << "\t";
					file << instSolution->gap << "\t";

					file << instSolution->statusCode << endl;


					delete instSolution;
				}
			}
		}
	}
	else
	{
		cerr << __FUNCTION__ << "(): Nessuna istanza da caricare." << endl;
		globalStatus = 1;
		if(instSolution != NULL)
			delete instSolution;
	}

	if(file.is_open())
		file.close();

	return globalStatus;
}



int main(int argc, char const *argv[])
{
	//string lpFile(".lp"), solution(".txt"), instance("test4.txt"), solFile(".sol"), clpFile("conflict.clp");
	//string lpFile("randomTest.lp"), solution("randomTest.txt"), instance("randomTest"), solFile("randomTest.sol"), clpFile("conflict.clp");
	
	bool verbose = false;
	//srand(time(NULL)); /* seed random number generator */
	//int contRelax[] = {0,0,0,0,0};
	int contRelax[] = {0};
	int typeOfLPVariables = 5;
	//string baseFileName;
	
	// ./main -D ../instances users gridL gridH step
	//     0   1       2        3     4     5     6
	if(string(argv[1]).compare(string("-D")) == 0)
	{
		vector<int> users;
		vector<int> gridL;
		vector<int> gridH;
		vector<int> step;
		
		users.push_back(atoi(argv[3]));
		gridL.push_back(atoi(argv[4]));
		gridH.push_back(atoi(argv[5]));
		step.push_back(atoi(argv[6]));
		string basePath("../");
		string folderName( basePath +  to_string(users[0]) + "_" + to_string( gridL[0] * gridH[0]) + "/"  );
		
		//int tInf = 0;
		//int tSup = floor(getTXCapacity() * 18.0 / 100.0); //test
		//DATASET
			createBatchInstances(
			3, //instQuantity
			users, //users
			vector<int> {30}, //drones
			gridL, //gridLength
			gridH,  //gridHeight
			step,  //gridStep
			0, 	//tInf
			40, //tSup
			5,	//cInf
			10, //cSup
			200, //dInf
			200, //dSup
			string("dataset"),
			folderName);
		return 0;
	}	
	if(argc == 1)
	{
		//int instQuantity, vector<int> users, vector<int> drones, vector<int> gridLength, vector<int> gridHeight, vector<int> gridStep, int tInf, int tSup, double cInf, double cSup, double dInf, double dSup, string instRootName)
		/*createBatchInstances(
			20, //instQuantity
			vector<int> {5}, //users
			vector<int> {20}, //drones
			vector<int> {5,10,10}, //gridLength
			vector<int> {5,5,10},  //gridHeight
			vector<int> {2,2,2},  //gridStep
			0, 	//tInf
			30, //tSup
			5,	//cInf
			10, //cSup
			100, //dInf
			200, //dSup
			string("automatictest"));*/

			//DATASET
			createBatchInstances(
			5, //instQuantity
			vector<int> {10,20,25,30,40,50}, //users
			vector<int> {20}, //drones
			vector<int> {10,15,20,30,25,50}, //gridLength
			vector<int> {10,10,10,10,20,20},  //gridHeight
			vector<int> {5,5,5,5,5,5},  //gridStep
			0, 	//tInf
			150, //tSup
			5,	//cInf
			10, //cSup
			200, //dInf
			200, //dSup
			string("dataset"),
			string(INSTANCE_PATH));
			
			/*createBatchInstances(
			10, //instQuantity
			vector<int> {5}, //users
			vector<int> {20}, //drones
			vector<int> {9}, //gridLength
			vector<int> {9},  //gridHeight
			vector<int> {2},  //gridStep
			50, 	//tInf
			1000, //tSup
			5,	//cInf
			10, //cSup
			200, //dInf
			200, //dSup
			string("dataset"));*/
	}
	else
	{
		if(string(argv[1]).compare(string("-h")) == 0)
		{
			printHelper();
			return 0;
		}

		if(string(argv[1]).compare(string("-t")) == 0) //test branch
		{

			//shadowingTest(10000,1.02802e-10); 
			cout << endl << "Shadowing model test: " << endl;
			shadowingTest(10000); 

			cout << endl << "interference model test: " << endl;
			interferenceModelTest();
			
			return 0;
		}
		
		if(argc == 3 && string(argv[1]).compare(string("-r")) == 0)
		{
			string pathToMaster(argv[2]);
			pathToMaster = pathToMaster + MASTER_FILE;

			//string pathToMasterSolFile(argv[2]);
			//pathToMasterSolFile = pathToMasterSolFile + MASTER_SOLUTIONS_FILE;
			vector< string > listOfInstance = loadFileList(pathToMaster.c_str()); 
			if (listOfInstance.size() > 0)
			{
				string csvCumulativeFile = pathToMaster + ".csv";
				ofstream csvFile;
					
				csvFile.open(csvCumulativeFile, ios::out);
				if(csvFile.is_open())
				{
					for(unsigned int i = 0; i < listOfInstance.size(); i++)
					{
						string relaxedFile = listOfInstance[i] + ".relsol";
						ofstream relFile;
						int count[1];
						
						relFile.open(relaxedFile,ios::out);
						
						if(relFile.is_open())
						{
							relFile << relaxedFile << endl;
							csvFile << listOfInstance[i] << '\t';	
							vector< int > yValue;						
							for(int j = 0; j < (int) pow(2.0,double(typeOfLPVariables)); j++) 
							{
								//if( (j == 0) || (j >= 12 && j <= 15)||(j >= 28 && j <= 31)) // debug: sottoinsieme di categorie delle istanze da eseguire
								if(j >= 28 && j <= 31)
								{
									if( (j & 16) != 16)
										relFile << "f" << '\t';
									else
										relFile << "rel_f" << '\t';
									
									if( (j & 8) != 8)
										relFile << "y" << '\t';
									else
										relFile << "rel_y" << '\t';
									
									if( (j & 4) != 4)
										relFile << "x" << '\t';
									else
										relFile << "rel_x" << '\t';
									
									if( (j & 2) != 2)
										relFile << "z" << '\t';
									else
										relFile << "rel_z" << '\t';
									
									if( (j & 1) != 1)
										relFile << "sp" << '\t';
									else
										relFile << "rel_sp" << '\t';
									
									relFile << endl;
									
									count[0] = j;
									solution_t *instSolution = executeInstance(listOfInstance[i],false,count);
									if(instSolution != NULL)
									{
										int dronesCount = 0;
										
										relFile << instSolution->objValue << '\t';
										relFile << instSolution->execTime << '\t';
										relFile << instSolution->statusCode << '\t';
										
										csvFile << instSolution->objValue << '\t';
										csvFile << instSolution->execTime << '\t';
										csvFile << instSolution->statusCode << '\t';
										
										for(unsigned int l = 0; l < instSolution->yPositions.size(); l++)
										{
											//if (round(instSolution->yPositions[l]) == 1) //
											double rounded = round(instSolution->yPositions[l] * 10.0) / 10.0;
											if(rounded >= 0.1)
											{
												dronesCount++;
												yValue.push_back(l);
											}
										}
										relFile <<  dronesCount << "/" << getDrnsNum() << '\t';
										csvFile <<  dronesCount << "/" << getDrnsNum() << '\t';

										for(unsigned int i = 0; i< yValue.size(); i++)
										{
											csvFile << "y" << yValue[i]+getUsrsNum();

											if( i < yValue.size()-1)
												csvFile << ",";
										}
										csvFile << '\t';
										yValue.clear();
										
										delete instSolution;
										relFile << endl << endl;
									}
								}
							}
							csvFile << endl;
							relFile.close();
						}
						else
						{
							cerr << __FUNCTION__ << "(): Impossibile creare il file: " << relaxedFile << endl;
							return 1;
						}
					}
					cout << "Max physical memory usage: " << getPeakRSS( ) << " KB" << endl;
				}
				else
					cerr << __FUNCTION__ << "(): Impossibile creare il file: " << csvCumulativeFile << endl;
				return 1;
			}
			else
			{
				cerr << __FUNCTION__ << "(): Nessuna istanza presente nel file: " << pathToMaster << endl;
				return 1;
			}
		}
		

		/*if(argc == 3 && string(argv[1]).compare(string("-X")) == 0)
		{
			string pathToMaster(argv[2]);
			pathToMaster = pathToMaster + MASTER_FILE;

			//string pathToMasterSolFile(argv[2]);
			//pathToMasterSolFile = pathToMasterSolFile + MASTER_SOLUTIONS_FILE;
			vector< string > listOfInstance = loadFileList(pathToMaster.c_str()); 
			if (listOfInstance.size() > 0)
			{
				string csvCumulativeFile = pathToMaster + ".heur.csv";
				ofstream csvFile;
					
				csvFile.open(csvCumulativeFile, ios::out);
				if(csvFile.is_open())
				{
					vector < solution_t* > solutions(3);
					int count[1];
					vector < int > yValue;

					for(unsigned int i = 0; i < listOfInstance.size(); i++)
					{	
						contRelax[0] = 0;

						solutions[0] = executeInstance(listOfInstance[i],true,contRelax);
						if(solutions[0] != NULL)
						{
							contRelax[0] = 31;
							solutions[1] = executeInstance(listOfInstance[i],true,contRelax);
							if(solutions[1] != NULL)
							{
								solutions[2] = executeHeurInstance(listOfInstance[i], true, contRelax);
								if(solutions[2] != NULL)
								{
									// csv
									csvFile << listOfInstance[i] << '\t';	

									for(unsigned int j = 0; j < solutions.size(); j++)
									{
										csvFile << solutions[j]->objValue << '\t';
										csvFile << solutions[j]->execTime << '\t';
										csvFile << solutions[j]->statusCode << '\t';

										int dronesCount = 0;
									
										for(unsigned int l = 0; l < solutions[j]->yPositions.size(); l++)
										{
											//if (round(instSolution->yPositions[l]) == 1) //
											double rounded = round(solutions[j]->yPositions[l] * 10.0) / 10.0;
											if(rounded >= 0.1)
											{
												dronesCount++;
												yValue.push_back(l);
											}
										}
										
										csvFile <<  dronesCount << "/" << getDrnsNum() << '\t';

										for(unsigned int l = 0; l < yValue.size(); l++)
										{
											csvFile << "y" << yValue[l]+getUsrsNum();

											if( l < yValue.size()-1)
												csvFile << ",";
										}
										csvFile << '\t';

										yValue.clear();
										delete solutions[j];
									}
									csvFile << endl;
								}
								else
									return 1;
							}
							else
								return 1;
						}
						else
							return 1;
					}	
						
					cout << "Max physical memory usage: " << getPeakRSS( ) << " KB" << endl;
				}
				else
					cerr << __FUNCTION__ << "(): Impossibile creare il file: " << csvCumulativeFile << endl;
				return 1;
			}
			else
			{
				cerr << __FUNCTION__ << "(): Nessuna istanza presente nel file: " << pathToMaster << endl;
				return 1;
			}
		}*/

		if(string(argv[1]).compare(string("-X")) == 0)
		{
			string pathToMaster(argv[2]);
			pathToMaster = pathToMaster + MASTER_FILE;

			if(argc == 4)
			{
				int gridType = atoi(argv[3]);
				setSpecialGrid(gridType);
			}

			DECL_ENV(env);
			//string pathToMasterSolFile(argv[2]);
			//pathToMasterSolFile = pathToMasterSolFile + MASTER_SOLUTIONS_FILE;
			if ( CPXreadcopyparam(env, "CPLEXparams.prm") != 0)
				cerr << __FUNCTION__ << "(): Impossibile leggere i parametri CPLEX dal file di configurazione, si usera' la configurazione standard." << endl;
	
			vector< string > listOfInstance = loadFileList(pathToMaster.c_str()); 
			if (listOfInstance.size() > 0)
			{
				string csvCumulativeFile = pathToMaster + ".heur.csv";
				ofstream csvFile;
					
				csvFile.open(csvCumulativeFile, ios::out);
				if(csvFile.is_open())
				{
					//vector < solution_t* > solutions(2);
					//int count[1];
					vector < int > yValue;

					csvFile << "Istanza\t";
					csvFile << "Tempo (sec)\t";
					csvFile << "# droni\t";
					csvFile << "Opened Nodes\t";
					csvFile << "Best Obj\t";
					csvFile << "Cutoff\t";
					csvFile << "Cuts\t";
					csvFile << "Gap\t";
					csvFile << "Status\t";
					csvFile <<"Iteration Count\t";
					csvFile << "Drones: " << endl;
					for(unsigned int i = 0; i < listOfInstance.size(); i++)
					{	
						//solve 1
						cout << "Esecuzione euristica 1." << endl;
						DECL_PROB(env, lp1);
						if (loadInstance(listOfInstance[i].c_str()) != 0)
						{
							cerr << __FUNCTION__ <<" Impossibile caricare l'istanza: " << listOfInstance[i] << endl;
							CPXcloseCPLEX(&env);
							return 1;
						}
						else
						{
							
							cout << "Instance " << listOfInstance[i] << " loaded." << endl;
							
							std::chrono::high_resolution_clock::time_point heustart;
							heustart = std::chrono::high_resolution_clock::now(); 
							solution_t * heurSol = solve3(env,lp1, listOfInstance[i].c_str(), true);
							cout << "***TEMPO TOTALE IMPIEGATO: " 
							<< std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - heustart).count() //casting per la conversione in millisecondi; (end-start).count() calcola la differenza di tempo e la restituisce come valore numerico
							<< "ms. \n";
							//printSolution(heurSol);
							// free
							
							//CPXcloseCPLEX(&env);
							
							if(heurSol == NULL)
							{
								cerr << __FUNCTION__ << "(): Soluzione non trovata per l'istanza " << listOfInstance[i] << endl;
								csvFile << listOfInstance[i] << '\t';
								csvFile << "-1" << '\t';
								csvFile << "-1" << '\t';
								csvFile << "-1" << '\t';
								csvFile << "-1" << '\t';
								csvFile << "-1" << '\t';
								csvFile << "-1" <<'\t';
								csvFile << "-1" << '\t';
								csvFile << "-1" << '\t';
								csvFile << "-1" << '\t';
								csvFile << "-1" <<'\t';
							}
							else
							{
								/*printSolution(heurSol);
								cout << "Soluzione finale:"<<endl;
								cout << listOfInstance[i] <<endl;
								cout << heurSol->objValue << endl;
								cout << heurSol->execTime << endl;
								cout << heurSol->statusCode << endl << endl;*/


								int dronesCount = 0;
									
								for(unsigned int l = 0; l < heurSol->yPositions.size(); l++)
								{
									//if (round(instSolution->yPositions[l]) == 1) //
									//double rounded = round(solutions[j]->yPositions[l] * 10.0) / 10.0;
									if(heurSol->yPositions[l] > 0.0)
									{
										dronesCount++;
										yValue.push_back(l);
									}
								}
									
								csvFile << listOfInstance[i] << '\t';
								//csvFile << heurSol->objValue << '\t';
								csvFile << heurSol->execTime << '\t';
								csvFile << dronesCount << "\t";
								csvFile << heurSol->openedNodes << "\t";
								csvFile << heurSol->bestobj << "\t";
								csvFile << heurSol->cutoff << "\t";
								csvFile << heurSol->cuts << "\t";
								csvFile << heurSol->gap << "\t";

								csvFile << heurSol->statusCode << '\t';
								csvFile << heurSol->heurIterationCount << '\t';				

								for(unsigned int l = 0; l < yValue.size(); l++)
								{
									csvFile << "y" << yValue[l]+getUsrsNum();

									if( l < yValue.size()-1)
										csvFile << ",";
								}
								csvFile << '\t';


								delete heurSol;
								yValue.clear();
							}
							CPXfreeprob(env, &lp1);
							csvFile << endl;
						}



						//solve 2
						/*cout << endl << "---------------------------------------------------------------------" << endl
							<< "Esecuzione euristica 2." << endl;

						DECL_PROB(env, lp2);
						if (loadInstance(listOfInstance[i].c_str()) != 0)
						{
							cerr << __FUNCTION__ <<" Impossibile caricare l'istanza: " << listOfInstance[i] << endl;
							CPXcloseCPLEX(&env);
							return 1;
						}
						else
						{
							
							cout << "Instance " << listOfInstance[i] << " loaded." << endl;
							
							solution_t * heurSol = solve2(env,lp2, listOfInstance[i].c_str(),true);
							
							// free
							CPXfreeprob(env, &lp2);
							
							
							if(heurSol == NULL)
							{
								cerr << __FUNCTION__ << "(): Soluzione non trovata per l'istanza " << listOfInstance[i] << endl;
								csvFile << listOfInstance[i] << '\t';
								csvFile << "-1" << '\t';
								csvFile << "-1" << '\t';
								csvFile << "-1" << '\t';
								csvFile << "-1" << '\t';
								csvFile << "-1" << '\t';
								csvFile << "-1" <<'\t';
								csvFile << "-1" << '\t';
								csvFile << "-1" << '\t';
								csvFile << "-1" << '\t';
								csvFile << "-1" <<'\t';
								csvFile << endl;
							}
							else
							{
								//printSolution(heurSol);
								
								//csvFile << heurSol->objValue << '\t';
								int dronesCount = 0;
									
								for(unsigned int l = 0; l < heurSol->yPositions.size(); l++)
								{
									//if (round(instSolution->yPositions[l]) == 1) //
									//double rounded = round(solutions[j]->yPositions[l] * 10.0) / 10.0;
									if(heurSol->yPositions[l] > 0.0)
									{
										dronesCount++;
										yValue.push_back(l);
									}
								}
									
								csvFile << listOfInstance[i] << '\t';
								//csvFile << heurSol->objValue << '\t';
								csvFile << heurSol->execTime << '\t';
								csvFile << dronesCount << "\t";
								csvFile << heurSol->openedNodes << "\t";
								csvFile << heurSol->bestobj << "\t";
								csvFile << heurSol->cutoff << "\t";
								csvFile << heurSol->cuts << "\t";
								csvFile << heurSol->gap << "\t";

								csvFile << heurSol->statusCode << '\t';
								csvFile << heurSol->heurIterationCount << '\t';	

								for(unsigned int l = 0; l < yValue.size(); l++)
								{
									csvFile << "y" << yValue[l]+getUsrsNum();

									if( l < yValue.size()-1)
										csvFile << ",";
								}
								csvFile << '\t';


								delete heurSol;
								yValue.clear();
							}
							csvFile << endl;
						}*/
					}	
						
					cout << "Max physical memory usage: " << getPeakRSS( ) << " KB" << endl;
					CPXcloseCPLEX(&env);
					return 0;
				}
				else
					cerr << __FUNCTION__ << "(): Impossibile creare il file: " << csvCumulativeFile << endl;
				CPXcloseCPLEX(&env);
				return 1;
			}
			else
			{
				cerr << __FUNCTION__ << "(): Nessuna istanza presente nel file: " << pathToMaster << endl;
				CPXcloseCPLEX(&env);
				return 1;
			}
		}
		
		if(string(argv[1]).compare(string("-H")) == 0)
		{
			/*//arg 3: file istanza
			contRelax[0] = 31;

			//esegue rilassamento lineare e salva le vars y su vector
			solution_t *instSolution = executeHeurInstance(argv[2], true, contRelax);
			if(instSolution == NULL)
				return 1;
			else
			{
				delete instSolution;
				
				cout << "Max physical memory usage: " << getPeakRSS( ) << " KB" << endl;
				return 0;
			}*/

			//euristica threshold
			//arg 3: file istanza
			

			string instance(argv[2]);
			
			DECL_ENV(env);
			DECL_PROB(env, lp);
			if ( CPXreadcopyparam(env, "CPLEXparams.prm") != 0)
				cerr << __FUNCTION__ << "(): Impossibile leggere i parametri CPLEX dal file di configurazione, si usera' la configurazione standard." << endl;
	
	
			if (loadInstance(instance.c_str()) != 0)
			{
				cerr << __FUNCTION__ <<" Impossibile caricare l'istanza: " << instance << endl;
				return 1;
			}
			else
			{
				string logFile = instance + ".log";
				CPXFILEptr logF = CPXfopen(logFile.c_str(),"w"); 
				CPXsetlogfile(env,logF);

				cout << "Instance " << instance << " loaded." << endl;
				if(verbose == true)
				{
					
					cout << "Users: " << getUsrsNum() << endl;
					cout << "Drones: " << getDrnsNum() << endl;
					cout << "Potential drones positions: " << getTotalPotentialNodes() << endl << endl;
					//printNodesIOFlow();
					printTrafficMatrix();
				}

				solution_t * heurSol = solve2(env,lp, argv[2],true);
				
				// free
				CPXfreeprob(env, &lp);
				CPXcloseCPLEX(&env);
				CPXfclose(logF);
				
				if(heurSol == NULL)
				{
					cerr << __FUNCTION__ << "(): Soluzione non trovata per l'istanza " << argv[2] << endl;
					return 1;
				}
				else
				{
					printSolution(heurSol);
					delete heurSol;
					return 0;
				}
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
			
		}

		if(string(argv[1]).compare(string("-a")) == 0)
		{
			// no flight zones special grids
			if(argc == 4) // BTNGRIDS it's a miserable workaround, no time to implement nicely, apologies
			{
				contRelax[0] = 0;
				string pathToMaster(argv[2]);
				pathToMaster = pathToMaster + MASTER_FILE;

				string pathToMasterSolFile(argv[2]);
				pathToMasterSolFile = pathToMasterSolFile + MASTER_SOLUTIONS_FILE;

				int gridType = atoi(argv[3]);
				setSpecialGrid(gridType);
				
				int status = executeMaster(loadFileList(pathToMaster.c_str()), pathToMasterSolFile.c_str(), true, contRelax);

				cout << "Max physical memory usage: " << getPeakRSS( ) << " KB" << endl;

				return status;
			}
			if(argc == 3)
			{
				contRelax[0] = 0;
				string pathToMaster(argv[2]);
				pathToMaster = pathToMaster + MASTER_FILE;

				string pathToMasterSolFile(argv[2]);
				pathToMasterSolFile = pathToMasterSolFile + MASTER_SOLUTIONS_FILE;
				
				int status = executeMaster(loadFileList(pathToMaster.c_str()), pathToMasterSolFile.c_str(), true, contRelax);

				cout << "Max physical memory usage: " << getPeakRSS( ) << " KB" << endl;

				return status;
			}
			else
			{
				contRelax[0] = 0;
				int status = executeMaster(loadFileList(MASTER_FILE), MASTER_SOLUTIONS_FILE, verbose, contRelax);
				
				cout << "Max physical memory usage: " << getPeakRSS( ) << " KB" << endl;

				return status;
			}
		}
		else
		{
			//arg 2: file istanza
			contRelax[0] = 0;
			solution_t *instSolution = executeInstance(string(argv[1]),true,contRelax);
			if(instSolution == NULL)
			{
				cerr << __FUNCTION__ << "(): No solution found in instance " << argv[1] << endl;
				return 1;
			}
			else
			{
				delete instSolution;
				
				cout << "Max physical memory usage: " << getPeakRSS( ) << " KB" << endl;
				return 0;
			}
		}
	}
}
