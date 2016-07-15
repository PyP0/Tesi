#include <iostream>
#include <vector>
#include "cpxmacro.h"
#include "lpsolver.h"

using namespace std;


bool areOutOfSight(int i, int j)
{
	int k = 0;
	bool flag = true;
	while (k < K && flag == true)
	{
		if (c[i][j][k] <= threshold)
			flag = false;
		k++;
	}
	return flag;
}

void setupLP(CEnv env, Prob lp)
{
	;
}