#ifndef _HEURISTIC_H_
#define _HEURISTIC_H_

#include "cpxmacro.h"
#include "lpsolver.h"

struct heurSolution_t
{
	std::string instName;
	std::vector< double > yPositions;
	double objValue;
	//unsigned long int execTime;
};

struct move
{
	int dummy;
};

/*
There  are  seven  key  interfaces  that  must  be  implemented,  described  as  
follows.  
1. The solution  interface allows the user to encapsulate the data structure(s) 
that  represents  the  problem’s  solution.  The  framework  does  not  impose  
any restriction on how the user defines the solution or the data structures used 
since it never manipulates Solution objects directly.  
2. The Objective  Function interface  evaluates  the  objective  value  of  the  
solution.  
3. The Neighborhood  Generator interface  is  an  iterator  that generates  a  
collection of neighbors based on the Move and Constraint interfaces.  
4. The Move interface  defines  the  move  neighborhood,  where  it  constructs  
possible translations of Solution object. When the engine determines the 
best move in an iteration, the Solution object is then translated to its new 
state.      
5. The Constraint interface  computes  the  degree  of  violation  for  these  
translations.   
6. The Tabu List interface records the tabu-ed solutions or moves.  
7. The Aspiration Criteria interface   allows   specification   of   aspiration   
criteria for tabu-ed moves. 

*/

bool solutionExists(int);
double evaluate(solution_t*);
void findBestNeighbor(move); 
solution_t *executeHeurInstance(std::string, bool, int[]);
solution_t *startingSolution(CEnv,Prob,std::string,int[]);
solution_t *solve(CEnv,Prob,std::string,bool);
solution_t *solve2(Env,Prob,std::string,bool);
solution_t *solve3(Env,Prob,std::string,bool);
#endif