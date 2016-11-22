#ifndef _LP_SOLVER_H
#define _LP_SOLVER_H

#include "cpxmacro.h"
#include "instance.h"
#include <string>
#include <vector>

#define MASTER_SOLUTIONS_FILE "master_solutions.txt"

struct solution_t
{
	std::string instName;
	std::vector< double > yPositions;
	double objValue;
	int statusCode;
	unsigned long int execTime;

	unsigned long int openedNodes;
	double rootGap;

	int heurIterationCount;

};

int gety_index();

solution_t *getSolution(CEnv,Prob,unsigned long int,std::string);

solution_t *getHeurSolution(CEnv,Prob,unsigned long int,std::string,int);

solution_t *copySolution(solution_t*);

int dronesCount(solution_t*, double);
void printVarsValue(CEnv, Prob);
//solution_t *getSolution(CEnv, Prob);
void printSolution(solution_t*);
void printRelaxedSolution(solution_t*);
void printConflictFile(std::string,CEnv,Prob);
solution_t *solveLP(CEnv,Prob,std::string,bool,int*); 
solution_t *solveHeurLP(CEnv,Prob,std::string,bool,int*);
//void printNetworkUsage(CEnv,Prob);
void printSimplifiedSolFile(CEnv,Prob,const char*);
int testSolutionFile(const char *,const char *);
int interfaceToGraphicModule(CEnv,Prob,std::string);
void swapToInt(CEnv,Prob);
void swapToCont(CEnv,Prob);
#endif
