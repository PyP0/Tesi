#ifndef _HEURISTIC_H_
#define _HEURISTIC_H_

struct heurSolution_t
{
	std::string instName;
	std::vector< double > yPositions;
	double objValue;
	unsigned long int execTime;
};

struct move
{
	int boh;
};

double evaluate(solution_t*);


#endif