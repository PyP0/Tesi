/**
* @file modelv1.2.cpp
* @brief
*/

#include "instance.h"
#include "utility.h"
#include <iostream>
#include <vector>

using namespace std;

double evaluate(solution_t *currentSol)
{
	double objF = 0.0;

	for(unsigned int i = 0; i < currentSol->yPositions.size(); i++)
	{
		objF+= currentSol->yPositions[i] * getDeploycost(i);
	}

	return objF;
}