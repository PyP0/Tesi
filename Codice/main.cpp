/**
* @file modelv0.9.2.cpp
* @brief
*/
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include "cpxmacro.h"
#include "instance.h"
#include "utility.h"
#include "lpsolver.h"

using namespace std;

void printHelper() //TODO
{
	cout << endl << "Helper to do" <<endl;
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
		while(!file.eof())
		{
			file >> fileLine;
			cout <<fileLine <<endl;
			fileList.push_back(fileLine);
			if(!file.eof())
				count++;
		}
		cout << "Sono state caricate " << count << " istanze." << endl;
		file.close();
		return fileList;
	}
	else
	{
		cerr << __FUNCTION__ << "(): Impossibile aprire il file: " << file << endl;
		return fileList;
	}
}

solution_t *executeInstance(string fileName, bool verbose)
{
	solution_t *instSolution = NULL;
	
	// init
	DECL_ENV(env);
	DECL_PROB(env, lp);
	CPXsetintparam(env, CPX_PARAM_CONFLICTDISPLAY, 2);

	string baseFileName (fileName);
	string instance = baseFileName;
	string solution = baseFileName + ".txt";
	string solFile = baseFileName + ".sol";
	string clpFile = baseFileName + ".clp";

	if (loadInstance(instance.c_str()) != 0)
	{
		cerr << __FUNCTION__ <<" Impossibile caricare l'istanza: " << instance << endl;
		return NULL;
	}
	else
	{
		if(verbose == true)
		{
			cout << "Instance " << instance << " loaded." << endl;
			cout << "Users: " << getUsrsNum() << endl;
			cout << "Drones: " << getDrnsNum() << endl;
			cout << "Potential drones positions: " << getPosNum() << endl << endl;
			printNodesIOFlow();
		}

		try
		{
			instSolution = solveLP(env, lp, instance);
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
						printSolution(instSolution);

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
	return instSolution;

}

int executeMaster(vector< string > fileList, string masterSolutionsFile)
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

		for(unsigned int i = 0; i < fileList.size(); i++)
		{
			instSolution = executeInstance(fileList[i],false);
			if(instSolution == NULL)  //check solution status
				globalStatus = 1;
			else
			{
				if(file.is_open())
				{
					//remove the ../instances/
					file << instSolution->instName.substr(13,string::npos) << "\t";
					file << instSolution->objValue << "\t";
					file << instSolution->execTime << endl;
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
	

	//srand(time(NULL)); /* seed random number generator */

	
	//string baseFileName;
	if(argc < 1)
	{
		printHelper();
		return 0;
	}
	if(argc == 1)
		//int instQuantity, vector<int> users, vector<int> drones, vector<int> positions, vector<int> gridLength, vector<int> gridHeight, vector<int> gridStep, int tInf, int tSup, double cInf, double cSup, double dInf, double dSup, string instRootName)
		createBatchInstances(1, vector<int>(1,5),vector<int>(1,20),vector<int>(1,24),vector<int>(1,6),vector<int>(1,4),vector<int>(1,2),0,30,5,10,20,50,string("automatictest"));
	else
	{
		/*solution_t *instSolution=NULL;
	
		// init
		DECL_ENV(env);
		DECL_PROB(env, lp);
		CPXsetintparam(env, CPX_PARAM_CONFLICTDISPLAY, 2);*/

		if(string(argv[1]).compare(string("-a")) == 0)
		{
			int status = executeMaster(loadFileList(MASTER_FILE), MASTER_SOLUTIONS_FILE);
			return status;
		}
		else
		{
			//arg 2: file istanza
			solution_t *instSolution = executeInstance(string(argv[1]),true);
			if(instSolution == NULL)
				return 1;
			else
			{
				delete instSolution;
				return 0;
			}
		}
		//arg 2: file istanza
		/*string baseFileName (argv[1]);
		string instance = baseFileName;
		string lpFile = baseFileName + ".lp";
		string solution = baseFileName + ".txt";
		string solFile = baseFileName + ".sol";
		string clpFile = baseFileName + ".clp";

		
		//createRandomInstance(users,drones,positions,tInf,tSup,cInf,cSup,dInf,dSup,gridLength,gridHeight,gridStep,filename);
		//createRandomInstance(5,6,6, 0,20, 1,150, 50,60, 2,3,10,"randomTest");
		if (loadInstance(instance.c_str()) != 0)
		{
			cerr << __FUNCTION__ <<" Impossibile caricare l'istanza: " << instance << endl;
		}
		else
		{
			cout << "Instance " << instance << " loaded." << endl;
			cout << "Users: " << getUsrsNum() << endl;
			cout << "Drones: " << getDrnsNum() << endl;
			cout << "Potential drones positions: " << getPosNum() << endl << endl;
			

			//saveInstance(instance.c_str());
			printNodesIOFlow();
			try
			{
				int status = solveLP(env, lp, lpFile);
				cout << "Status code: " << status << endl;

				if (status == 103) //CPXMIP_INFEASIBLE 
				{
					// infeasibility detected
					printConflictFile(clpFile, env, lp);
				}
				else
				{
					//printSolutionData(env, lp, y_index, solution);
					instSolution = getSolution(env, lp);
					if (instSolution == NULL)
						cerr << __FUNCTION__ << "(): Impossibile salvare la soluzione dell'istanza risolta." << endl;
					else
						printSolution(instSolution);
					
					printSimplifiedSolFile(env, lp, solution.c_str());
					printVarsValue(env, lp);
					//printNetworkUsage(env, lp);
					//testSolutionFile("refIST3.txt", solution.c_str());

					CHECKED_CPX_CALL(CPXsolwrite, env, lp, solFile.c_str());
				}

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
		}
		return 0;*/
		
	}
}
